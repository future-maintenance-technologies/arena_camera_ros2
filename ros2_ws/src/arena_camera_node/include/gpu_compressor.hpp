#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <sycl/sycl.hpp>

class QsvAv1Encoder;  // forward declaration — keeps libav headers out of here

/// Compresses raw 8-bit Bayer camera frames to AV1 using SYCL + QSV.
///
/// Pipeline per frame:
///   1. Bayer (CPU)  →  GPU upload
///   2. GPU kernel   :  de-mosaic into tiled NV12 (Y = tiled quadrants, UV = 128)
///   3. GPU → CPU    :  DMA to pinned host buffer
///   4. QSV encode   :  CPU sw_frame → GPU surface → hardware AV1 → CPU bitstream
///
/// All GPU buffers are pre-allocated once to avoid per-frame allocation overhead.
class GpuCompressor {
public:
    /// @param width      Image width in pixels (must be even).
    /// @param height     Image height in pixels (must be even).
    /// @param framerate  Expected frame rate for encoder timing.
    /// @param qsv_device  QSV device path (e.g. "/dev/dri/renderD129" for Intel).
    GpuCompressor(int width, int height, int framerate = 22,
                  const char* qsv_device = "/dev/dri/renderD129");
    ~GpuCompressor();

    GpuCompressor(const GpuCompressor&) = delete;
    GpuCompressor& operator=(const GpuCompressor&) = delete;

    /// Compress a single raw 8-bit Bayer frame to AV1.
    /// @param bayer_data  CPU pointer to raw Bayer data (width * height bytes).
    /// @param pixel_format  Pixel format string (e.g. "bayer_rggb8" or "mono8").
    /// @param pts         Presentation timestamp forwarded to the encoder.
    /// @return AV1-encoded bytes, or nullopt if frame was dropped due to encoder saturation.
    std::optional<std::vector<uint8_t>> compress(const uint8_t* input_data, std::string const& pixel_format, int64_t pts);

    /// Human-readable GPU device name (for logging).
    const std::string& device_name() const { return device_name_; }

    // Telemetry
    uint64_t frames_compressed() const { return frames_compressed_; }
    uint64_t frames_dropped() const;
    uint64_t total_gpu_time_us() const { return total_gpu_time_us_; }
    uint64_t total_encode_time_us() const { return total_encode_time_us_; }

private:
    int    width_;
    int    height_;
    size_t y_size_;     // width * height
    size_t nv12_size_;  // width * height * 3 / 2

    // SYCL resources
    sycl::device gpu_;
    sycl::queue  q_;    // in-order queue — serializes all GPU work
    std::string  device_name_;

    // Pre-allocated GPU buffers (lifetime = this object)
    uint8_t* d_input_ = nullptr;  // device: raw Bayer input
    uint8_t* d_nv12_  = nullptr;  // device: tiled NV12 output

    // Pre-allocated pinned host buffer for efficient GPU→CPU DMA
    uint8_t* h_nv12_ = nullptr;

    // QSV AV1 encoder
    std::unique_ptr<QsvAv1Encoder> encoder_;

    // Telemetry counters
    uint64_t frames_compressed_     = 0;
    uint64_t total_gpu_time_us_     = 0;  // SYCL kernel + memcpy time
    uint64_t total_encode_time_us_  = 0;  // QSV encode time

    /// Find the first available GPU device.
    static sycl::device find_gpu();

    /// SYCL kernel: Bayer → tiled NV12 (Y = quadrant-tiled grayscale, UV = 128).
    /// Writes only the Y plane of d_nv12_; UV plane is pre-set to 128.
    void bayer_to_tiled_nv12();
    void mono8_to_nv12();
};
