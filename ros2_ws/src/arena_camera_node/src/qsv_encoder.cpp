#include "qsv_encoder.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>

extern "C" {
#include <libavutil/opt.h>
}

QsvAv1Encoder::QsvAv1Encoder(int width, int height, int framerate,
                             const char* qsv_device)
    : width_(width), height_(height)
{
    // QSV hardware device context.
    // On Linux, QSV is implemented on top of VAAPI — there is no standalone QSV
    // device node. The render node path goes into the "child_device" dictionary
    // option; the device string argument to av_hwdevice_ctx_create must be nullptr.
    AVDictionary* device_opts = nullptr;
    av_dict_set(&device_opts, "child_device", qsv_device, 0);

    int create_ret = av_hwdevice_ctx_create(&hw_device_ctx_, AV_HWDEVICE_TYPE_QSV,
                                            nullptr, device_opts, 0);
    av_dict_free(&device_opts);
    if (create_ret < 0) {
        char errbuf[128];
        av_strerror(create_ret, errbuf, sizeof(errbuf));
        throw std::runtime_error(std::string("Failed to create QSV device: ") + errbuf);
    }

    const AVCodec* codec = avcodec_find_encoder_by_name("av1_qsv");
    if (!codec)
        throw std::runtime_error("av1_qsv encoder not found");

    codec_ctx_ = avcodec_alloc_context3(codec);
    codec_ctx_->width          = width;
    codec_ctx_->height         = height;
    codec_ctx_->time_base      = {1, framerate};
    codec_ctx_->framerate      = {framerate, 1};
    codec_ctx_->pix_fmt        = AV_PIX_FMT_QSV;
    codec_ctx_->global_quality = 1;
    codec_ctx_->gop_size       = 1;  // all-intra — each frame independently decodable
    codec_ctx_->qmax           = 0;

    // Hardware frames context
    hw_frames_ctx_ = av_hwframe_ctx_alloc(hw_device_ctx_);
    auto* frames_ctx = reinterpret_cast<AVHWFramesContext*>(hw_frames_ctx_->data);
    frames_ctx->format            = AV_PIX_FMT_QSV;
    frames_ctx->sw_format         = AV_PIX_FMT_NV12;
    frames_ctx->width             = width;
    frames_ctx->height            = height;
    frames_ctx->initial_pool_size = 64;

    if (av_hwframe_ctx_init(hw_frames_ctx_) < 0)
        throw std::runtime_error("Failed to init QSV hw frames context");

    codec_ctx_->hw_frames_ctx = av_buffer_ref(hw_frames_ctx_);

    av_opt_set(codec_ctx_->priv_data, "tile_cols", "2", 0);
    av_opt_set(codec_ctx_->priv_data, "tile_rows", "2", 0);
    av_opt_set_int(codec_ctx_->priv_data, "async_depth", 16, 0);
    av_log_set_level(AV_LOG_VERBOSE);

    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0)
        throw std::runtime_error("Failed to open av1_qsv codec");

    // CPU-side NV12 frame — data pointers are assigned directly to the
    // pinned host buffer each frame (no private buffer needed).
    sw_frame_ = av_frame_alloc();
    sw_frame_->format = AV_PIX_FMT_NV12;
    sw_frame_->width  = width;
    sw_frame_->height = height;

    // Pre-allocate persistent QSV surface pool (avoids per-frame allocation).
    for (int i = 0; i < QSV_SURFACE_POOL; ++i) {
        AVFrame* frame = av_frame_alloc();
        if (av_hwframe_get_buffer(hw_frames_ctx_, frame, 0) < 0) {
            throw std::runtime_error("Failed to allocate QSV surface");
        }
        surfaces_.push_back(frame);
    }

    pkt_ = av_packet_alloc();
}

std::optional<std::vector<uint8_t>> QsvAv1Encoder::encode(const uint8_t* nv12_data,
                                                           int64_t pts)
{
    // Check if encoder is saturated — drop frame to prevent unbounded queueing
    if (frames_in_flight_.load(std::memory_order_acquire) >= MAX_FRAMES_IN_FLIGHT) {
        frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return std::nullopt;
    }

    frames_in_flight_.fetch_add(1, std::memory_order_release);

    try {
        // Round-robin surface from persistent pool (no per-frame allocation)
        AVFrame* hw_frame = surfaces_[next_surface_];
        next_surface_ = (next_surface_ + 1) % surfaces_.size();

        // --- Map pinned NV12 buffer directly as AVFrame backing memory ---
        // Eliminates ~600 MB/s CPU memcpy traffic.
        sw_frame_->data[0]     = const_cast<uint8_t*>(nv12_data);
        sw_frame_->data[1]     = const_cast<uint8_t*>(nv12_data) + width_ * height_;
        sw_frame_->linesize[0] = width_;
        sw_frame_->linesize[1] = width_;
        sw_frame_->pts         = pts;

        // --- Upload CPU frame → QSV surface (GPU) ---
        int ret = av_hwframe_transfer_data(hw_frame, sw_frame_, 0);
        if (ret < 0) {
            frames_in_flight_.fetch_sub(1, std::memory_order_release);
            char errbuf[128];
            av_strerror(ret, errbuf, sizeof(errbuf));
            throw std::runtime_error(std::string("hwframe transfer failed: ") + errbuf);
        }
        hw_frame->pts = pts;

        // --- Send frame to hardware encoder ---
        ret = avcodec_send_frame(codec_ctx_, hw_frame);
        if (ret < 0) {
            frames_in_flight_.fetch_sub(1, std::memory_order_release);
            char errbuf[128];
            av_strerror(ret, errbuf, sizeof(errbuf));
            throw std::runtime_error(std::string("send_frame failed: ") + errbuf);
        }

        // --- Drain all available encoded packets without blocking ---
        std::vector<uint8_t> result;
        while (avcodec_receive_packet(codec_ctx_, pkt_) == 0) {
            result.insert(result.end(), pkt_->data, pkt_->data + pkt_->size);
            av_packet_unref(pkt_);
        }

        frames_in_flight_.fetch_sub(1, std::memory_order_release);
        frames_encoded_.fetch_add(1, std::memory_order_relaxed);
        return result;

    } catch (...) {
        frames_in_flight_.fetch_sub(1, std::memory_order_release);
        throw;
    }
}

QsvAv1Encoder::~QsvAv1Encoder()
{
    av_frame_free(&sw_frame_);
    for (auto* surface : surfaces_) {
        av_frame_free(&surface);
    }
    surfaces_.clear();
    av_packet_free(&pkt_);
    avcodec_free_context(&codec_ctx_);
    av_buffer_unref(&hw_frames_ctx_);
    av_buffer_unref(&hw_device_ctx_);
}
