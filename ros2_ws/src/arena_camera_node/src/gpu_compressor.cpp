#include "gpu_compressor.hpp"
#include "qsv_encoder.hpp"

#include <stdexcept>

// ---------------------------------------------------------------------------
// Static helper
// ---------------------------------------------------------------------------

sycl::device GpuCompressor::find_gpu()
{
    for (const auto& platform : sycl::platform::get_platforms()) {
        for (const auto& device : platform.get_devices()) {
            if (device.is_gpu()) {
                return device;
            }
        }
    }
    throw std::runtime_error("No GPU device found for SYCL");
}

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

GpuCompressor::GpuCompressor(int width, int height, int framerate,
                             const char* qsv_device)
    : width_(width)
    , height_(height)
    , y_size_(static_cast<size_t>(width) * height)
    , nv12_size_(y_size_ + static_cast<size_t>(width) * (height / 2))
    , gpu_(find_gpu())
    , q_(gpu_, sycl::property::queue::in_order{})
    , device_name_(gpu_.get_info<sycl::info::device::name>())
{
    // Pre-allocate GPU buffers
    d_input_ = sycl::malloc_device<uint8_t>(y_size_, q_);
    d_nv12_  = sycl::malloc_device<uint8_t>(nv12_size_, q_);
    if (!d_input_ || !d_nv12_)
        throw std::runtime_error("Failed to allocate SYCL device memory");

    // Pinned host buffer for fast GPU→CPU DMA transfer
    h_nv12_ = sycl::malloc_host<uint8_t>(nv12_size_, q_);
    if (!h_nv12_)
        throw std::runtime_error("Failed to allocate SYCL pinned host memory");

    // UV plane is always 128 (neutral chroma) — set once, never overwritten.
    q_.memset(d_nv12_ + y_size_, 128, nv12_size_ - y_size_).wait();

    encoder_ = std::make_unique<QsvAv1Encoder>(width, height, framerate, qsv_device);
}

GpuCompressor::~GpuCompressor()
{
    encoder_.reset();
    if (d_input_) sycl::free(d_input_, q_);
    if (d_nv12_)  sycl::free(d_nv12_, q_);
    if (h_nv12_)  sycl::free(h_nv12_, q_);
}

// ---------------------------------------------------------------------------
// SYCL kernel — Bayer → tiled NV12
// ---------------------------------------------------------------------------

void GpuCompressor::bayer_to_tiled_nv12()
{
    const size_t w      = width_;
    const size_t h      = height_;
    const size_t half_w = w / 2;
    const size_t half_h = h / 2;

    uint8_t* in  = d_input_;
    uint8_t* out = d_nv12_;

    // Separate Bayer channels into quadrants for spatial-aware compression.
    // Each 2×2 Bayer block contributes one pixel to each quadrant:
    //   TL = (even row, even col)   TR = (even row, odd col)
    //   BL = (odd  row, even col)   BR = (odd  row, odd col)
    //
    // Writes only the Y plane [0, y_size_). UV plane stays at 128.
    q_.parallel_for(
        sycl::range<2>(h, w),
        [=](sycl::id<2> idx) {
            const size_t row = idx[0];
            const size_t col = idx[1];

            const uint8_t val = in[row * w + col];

            const size_t sub_row = row / 2;
            const size_t sub_col = col / 2;

            const size_t quad_x = (col & 1) ? half_w : 0;
            const size_t quad_y = (row & 1) ? half_h : 0;

            out[(quad_y + sub_row) * w + (quad_x + sub_col)] = val;
        }
    );
    // No .wait() — in-order queue serializes with the next memcpy.
}

void GpuCompressor::mono8_to_nv12() {
    const size_t w = width_;
    const size_t h = height_;

    uint8_t* in  = d_input_;
    uint8_t* out = d_nv12_;

    q_.parallel_for(
        sycl::range<2>(h, w),
        [=](sycl::id<2> idx) {
            const size_t row = idx[0];
            const size_t col = idx[1];

            const uint8_t val = in[row * w + col];
            out[row * w + col] = val; // Y plane
            // UV plane stays at 128 (neutral chroma)
        }
    );
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::optional<std::vector<uint8_t>> GpuCompressor::compress(uint8_t const* input_data, std::string pixel_format, int64_t pts)
{
    using namespace std::chrono;
    auto gpu_start = high_resolution_clock::now();

    // 1. Upload raw Bayer to GPU
    q_.memcpy(d_input_, input_data, y_size_);

    if (pixel_format == "bayer_rggb8") {
        // 2. Bayer → tiled NV12 on GPU (Y plane written; UV stays at 128)
        bayer_to_tiled_nv12();
    } else if (pixel_format == "mono8") {
        // 2. mono8 → NV12 on GPU (Y plane copied; UV stays at 128)
        mono8_to_nv12();
    } else {
        throw std::runtime_error("Unsupported pixel format for compression: " + pixel_format);
    }

    // 3. Download NV12 from GPU to pinned host memory
    q_.memcpy(h_nv12_, d_nv12_, nv12_size_);

    // Single sync point — all three GPU operations complete here
    q_.wait();

    auto gpu_end = high_resolution_clock::now();
    total_gpu_time_us_ += duration_cast<microseconds>(gpu_end - gpu_start).count();

    // 4. AV1 encode via VAAPI (CPU sw_frame → GPU encode → CPU bitstream)
    auto encode_start = high_resolution_clock::now();
    auto result = encoder_->encode(h_nv12_, pixel_format, pts);
    auto encode_end = high_resolution_clock::now();
    total_encode_time_us_ += duration_cast<microseconds>(encode_end - encode_start).count();

    if (result) {
        frames_compressed_++;
    }

    return result;
}

uint64_t GpuCompressor::frames_dropped() const
{
    return encoder_->frames_dropped();
}
