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
    frames_ctx->initial_pool_size = MAX_FRAMES_IN_FLIGHT;

    if (av_hwframe_ctx_init(hw_frames_ctx_) < 0)
        throw std::runtime_error("Failed to init QSV hw frames context");

    codec_ctx_->hw_frames_ctx = av_buffer_ref(hw_frames_ctx_);

    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0)
        throw std::runtime_error("Failed to open av1_qsv codec");

    // CPU-side NV12 frame (staging buffer for upload to QSV surface)
    sw_frame_ = av_frame_alloc();
    sw_frame_->format = AV_PIX_FMT_NV12;
    sw_frame_->width  = width;
    sw_frame_->height = height;
    if (av_frame_get_buffer(sw_frame_, 0) < 0)
        throw std::runtime_error("Failed to allocate sw_frame buffer");

    // QSV surface frame
    hw_frame_ = av_frame_alloc();
    if (av_hwframe_get_buffer(hw_frames_ctx_, hw_frame_, 0) < 0)
        throw std::runtime_error("Failed to allocate hw_frame buffer");

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
        // Re-acquire QSV surface each frame
        av_frame_unref(hw_frame_);
        if (av_hwframe_get_buffer(hw_frames_ctx_, hw_frame_, 0) < 0) {
            frames_in_flight_.fetch_sub(1, std::memory_order_release);
            throw std::runtime_error("Failed to get hw frame buffer");
        }

        // --- NV12 CPU copy ---
        // Copy contiguous NV12 into sw_frame planes (respecting linesize/stride).
        // Y plane — row by row (linesize may exceed width due to alignment)
        for (int row = 0; row < height_; ++row) {
            std::memcpy(sw_frame_->data[0] + row * sw_frame_->linesize[0],
                        nv12_data + row * width_,
                        width_);
        }
        // UV plane
        const size_t y_size = static_cast<size_t>(width_) * height_;
        for (int row = 0; row < height_ / 2; ++row) {
            std::memcpy(sw_frame_->data[1] + row * sw_frame_->linesize[1],
                        nv12_data + y_size + row * width_,
                        width_);
        }
        sw_frame_->pts = pts;

        // --- Upload CPU frame → QSV surface (GPU) ---
        int ret = av_hwframe_transfer_data(hw_frame_, sw_frame_, 0);
        if (ret < 0) {
            frames_in_flight_.fetch_sub(1, std::memory_order_release);
            char errbuf[128];
            av_strerror(ret, errbuf, sizeof(errbuf));
            throw std::runtime_error(std::string("hwframe transfer failed: ") + errbuf);
        }
        hw_frame_->pts = pts;

        // --- Send frame to hardware encoder ---
        ret = avcodec_send_frame(codec_ctx_, hw_frame_);
        if (ret < 0) {
            frames_in_flight_.fetch_sub(1, std::memory_order_release);
            char errbuf[128];
            av_strerror(ret, errbuf, sizeof(errbuf));
            throw std::runtime_error(std::string("send_frame failed: ") + errbuf);
        }

        // --- Drain all available encoded packets ---
        std::vector<uint8_t> result;
        while (true) {
            ret = avcodec_receive_packet(codec_ctx_, pkt_);
            if (ret == 0) {
                result.insert(result.end(), pkt_->data, pkt_->data + pkt_->size);
                av_packet_unref(pkt_);
            } else if (ret == AVERROR(EAGAIN)) {
                break;
            } else {
                frames_in_flight_.fetch_sub(1, std::memory_order_release);
                char errbuf[128];
                av_strerror(ret, errbuf, sizeof(errbuf));
                throw std::runtime_error(std::string("receive_packet error: ") + errbuf);
            }
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
    av_frame_free(&hw_frame_);
    av_packet_free(&pkt_);
    avcodec_free_context(&codec_ctx_);
    av_buffer_unref(&hw_frames_ctx_);
    av_buffer_unref(&hw_device_ctx_);
}
