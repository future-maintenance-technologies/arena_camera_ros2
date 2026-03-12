#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_vaapi.h>
#include <va/va.h>
}

/// Hardware-accelerated AV1 encoder via VAAPI (Video Acceleration API).
///
/// Uses the GPU's fixed-function AV1 encoder through the VAAPI interface.
/// Surfaces are managed externally by DmaBufSurfacePool and passed directly
/// to encode_surface() — no CPU upload, no av_hwframe_transfer_data().
///
/// The underlying hardware (Intel Quick Sync on Arc GPUs) is the same as the
/// prior av1_qsv path; VAAPI is used instead of the QSV/oneVPL wrapper to
/// enable direct DMA-BUF surface export for zero-copy SYCL interop.
class QsvAv1Encoder {
public:
    /// @param width        Frame width  (must match surface dimensions).
    /// @param height       Frame height (must match surface dimensions).
    /// @param framerate    Used for time_base/framerate codec settings.
    /// @param device_path  Render node path (e.g. "/dev/dri/renderD128").
    QsvAv1Encoder(int width, int height, int framerate, const char* device_path);
    ~QsvAv1Encoder();

    QsvAv1Encoder(const QsvAv1Encoder&) = delete;
    QsvAv1Encoder& operator=(const QsvAv1Encoder&) = delete;

    /// Encode a VAAPI surface directly (zero-copy).
    ///
    /// @param hw_frame  VAAPI surface pre-filled with NV12 data (e.g. from
    ///                  DmaBufSurfacePool).  Must belong to hw_frames_ctx().
    /// @param pts       Presentation timestamp forwarded to the bitstream.
    /// @return Encoded AV1 bitstream bytes, or nullopt if dropped (saturation).
    std::optional<std::vector<uint8_t>> encode_surface(AVFrame* hw_frame,
                                                       int64_t pts);

    int width()  const { return width_; }
    int height() const { return height_; }

    /// VAAPI display handle — needed by DmaBufSurfacePool for DMA-BUF export.
    VADisplay va_display() const;

    /// Hardware frames context — used by DmaBufSurfacePool to allocate surfaces.
    AVBufferRef* hw_frames_ctx() const { return hw_frames_ctx_; }

    // Telemetry
    uint64_t frames_encoded()   const { return frames_encoded_.load(std::memory_order_relaxed); }
    uint64_t frames_dropped()   const { return frames_dropped_.load(std::memory_order_relaxed); }
    int      frames_in_flight() const { return frames_in_flight_.load(std::memory_order_acquire); }

private:
    static constexpr int MAX_FRAMES_IN_FLIGHT = 4;

    int width_;
    int height_;

    // Telemetry counters
    std::atomic<int>      frames_in_flight_{0};
    std::atomic<uint64_t> frames_encoded_{0};
    std::atomic<uint64_t> frames_dropped_{0};

    AVBufferRef*    hw_device_ctx_ = nullptr;
    AVBufferRef*    hw_frames_ctx_ = nullptr;
    AVCodecContext* codec_ctx_     = nullptr;
    AVPacket*       pkt_           = nullptr;
};
