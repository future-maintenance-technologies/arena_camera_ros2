#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
}

/// Hardware-accelerated AV1 encoder using Intel QSV (Quick Sync Video).
///
/// Accepts CPU-side contiguous NV12 frames, uploads them to a QSV surface,
/// encodes with the GPU's fixed-function AV1 encoder, and returns the bitstream.
/// Implements backpressure: drops frames when encoder is saturated.
class QsvAv1Encoder {
public:
    /// @param width        Frame width  (must match input NV12 data).
    /// @param height       Frame height (must match input NV12 data).
    /// @param framerate    Used for time_base/framerate codec settings.
    /// @param qsv_device   QSV device path (e.g. "/dev/dri/renderD*").
    QsvAv1Encoder(int width, int height, int framerate = 22, const char* qsv_device);
    ~QsvAv1Encoder();

    QsvAv1Encoder(const QsvAv1Encoder&) = delete;
    QsvAv1Encoder& operator=(const QsvAv1Encoder&) = delete;

    /// Encode one NV12 frame to AV1.
    /// @param nv12_data  Contiguous NV12 buffer: Y plane (width*height bytes)
    ///                   followed by interleaved UV plane (width*height/2 bytes).
    /// @param pts        Presentation timestamp.
    /// @return Encoded AV1 bytes, or nullopt if frame was dropped due to saturation.
    std::optional<std::vector<uint8_t>> encode(const uint8_t* nv12_data, int64_t pts);

    int width()  const { return width_; }
    int height() const { return height_; }

    // Telemetry
    uint64_t frames_encoded() const { return frames_encoded_.load(std::memory_order_relaxed); }
    uint64_t frames_dropped() const { return frames_dropped_.load(std::memory_order_relaxed); }
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
    AVFrame*        sw_frame_      = nullptr;
    AVFrame*        hw_frame_      = nullptr;
    AVPacket*       pkt_           = nullptr;
};
