#include "qsv_encoder.hpp"

#include <stdexcept>
#include <string>

extern "C" {
#include <libavutil/opt.h>
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

QsvAv1Encoder::QsvAv1Encoder(int width, int height, int framerate,
                             const char* device_path)
    : width_(width), height_(height)
{
    // VAAPI device — direct access to VA-API for DMA-BUF zero-copy surfaces.
    // On Linux with Intel Arc GPUs, VAAPI talks to the same Quick Sync
    // hardware that av1_qsv uses, without the MediaSDK/oneVPL wrapper layer.
    int ret = av_hwdevice_ctx_create(
        &hw_device_ctx_,
        AV_HWDEVICE_TYPE_VAAPI,
        device_path,   // e.g. "/dev/dri/renderD128"
        nullptr,
        0);
    if (ret < 0) {
        char errbuf[128];
        av_strerror(ret, errbuf, sizeof(errbuf));
        throw std::runtime_error(
            std::string("Failed to create VAAPI device (") + device_path +
            "): " + errbuf);
    }

    // --- Codec ----------------------------------------------------------------
    const AVCodec* codec = avcodec_find_encoder_by_name("av1_vaapi");
    if (!codec)
        throw std::runtime_error(
            "av1_vaapi encoder not found — ensure FFmpeg is built with VAAPI "
            "AV1 encode support and the driver exposes AV1 encoding "
            "(check `vainfo` for VAEntrypointEncSlice under AV1Profile0)");

    codec_ctx_ = avcodec_alloc_context3(codec);
    codec_ctx_->width     = width;
    codec_ctx_->height    = height;
    codec_ctx_->time_base = {1, framerate};
    codec_ctx_->framerate = {framerate, 1};
    codec_ctx_->pix_fmt   = AV_PIX_FMT_VAAPI;
    codec_ctx_->gop_size  = 1;  // all-intra — each frame independently decodable

    // --- Hardware frames context ----------------------------------------------
    hw_frames_ctx_ = av_hwframe_ctx_alloc(hw_device_ctx_);
    if (!hw_frames_ctx_)
        throw std::runtime_error("Failed to allocate VAAPI hw frames context");

    auto* frames_ctx = reinterpret_cast<AVHWFramesContext*>(hw_frames_ctx_->data);
    frames_ctx->format            = AV_PIX_FMT_VAAPI;
    frames_ctx->sw_format         = AV_PIX_FMT_NV12;
    frames_ctx->width             = width;
    frames_ctx->height            = height;
    frames_ctx->initial_pool_size = 64;

    ret = av_hwframe_ctx_init(hw_frames_ctx_);
    if (ret < 0) {
        char errbuf[128];
        av_strerror(ret, errbuf, sizeof(errbuf));
        throw std::runtime_error(
            std::string("Failed to init VAAPI hw frames context: ") + errbuf);
    }

    codec_ctx_->hw_frames_ctx = av_buffer_ref(hw_frames_ctx_);

    // --- Encoder tuning (VAAPI AV1) -------------------------------------------
    // Quality: global_quality maps to QP for VAAPI CQP mode.
    // QP 1 = very high quality (matching prior av1_qsv configuration).
    codec_ctx_->global_quality = 1;

    // Tiles — "CxR" format; silently ignored if driver doesn't support it.
    av_opt_set(codec_ctx_->priv_data, "tiles", "2x2", 0);

    // Async pipeline depth.
    av_opt_set_int(codec_ctx_->priv_data, "async_depth", 16, 0);

    av_log_set_level(AV_LOG_VERBOSE);

    ret = avcodec_open2(codec_ctx_, codec, nullptr);
    if (ret < 0) {
        char errbuf[128];
        av_strerror(ret, errbuf, sizeof(errbuf));
        throw std::runtime_error(
            std::string("Failed to open av1_vaapi codec: ") + errbuf);
    }

    pkt_ = av_packet_alloc();
}

// ---------------------------------------------------------------------------
// va_display — extract VADisplay from the VAAPI device context
// ---------------------------------------------------------------------------

VADisplay QsvAv1Encoder::va_display() const
{
    auto* hw_ctx = reinterpret_cast<AVHWDeviceContext*>(hw_device_ctx_->data);
    auto* va_ctx = static_cast<AVVAAPIDeviceContext*>(hw_ctx->hwctx);
    return va_ctx->display;
}

// ---------------------------------------------------------------------------
// encode_surface — zero-copy: surface is already in GPU memory
// ---------------------------------------------------------------------------

std::optional<std::vector<uint8_t>>
QsvAv1Encoder::encode_surface(AVFrame* hw_frame, int64_t pts)
{
    // Back-pressure: drop frame if encoder pipeline is saturated.
    if (frames_in_flight_.load(std::memory_order_acquire) >= MAX_FRAMES_IN_FLIGHT) {
        frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return std::nullopt;
    }

    frames_in_flight_.fetch_add(1, std::memory_order_release);

    try {
        hw_frame->pts = pts;

        // --- Submit surface directly to hardware encoder (zero-copy) ---
        // No av_hwframe_transfer_data() — the SYCL kernel already wrote
        // NV12 data into this VAAPI surface via DMA-BUF.
        int ret = avcodec_send_frame(codec_ctx_, hw_frame);
        if (ret < 0) {
            frames_in_flight_.fetch_sub(1, std::memory_order_release);
            char errbuf[128];
            av_strerror(ret, errbuf, sizeof(errbuf));
            throw std::runtime_error(
                std::string("send_frame failed: ") + errbuf);
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

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------

QsvAv1Encoder::~QsvAv1Encoder()
{
    av_packet_free(&pkt_);
    avcodec_free_context(&codec_ctx_);
    av_buffer_unref(&hw_frames_ctx_);
    av_buffer_unref(&hw_device_ctx_);
}
