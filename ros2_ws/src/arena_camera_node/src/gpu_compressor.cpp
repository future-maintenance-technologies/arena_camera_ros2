#include "gpu_compressor.hpp"
#include "qsv_encoder.hpp"
#include "dmabuf_surface_pool.hpp"

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
                             const char* device_path)
    : width_(width)
    , height_(height)
    , y_size_(static_cast<size_t>(width) * height)
    , gpu_(find_gpu())
    , q_(gpu_, sycl::property::queue::in_order{})
    , device_name_(gpu_.get_info<sycl::info::device::name>())
{
    // Raw camera input buffer on GPU (Bayer / Mono8 uploaded each frame).
    d_input_ = sycl::malloc_device<uint8_t>(y_size_, q_);
    if (!d_input_)
        throw std::runtime_error("Failed to allocate SYCL device memory for input");

    // 1. Create VAAPI AV1 encoder (allocates hw device + frames contexts).
    encoder_ = std::make_unique<QsvAv1Encoder>(width, height, framerate, device_path);

    // 2. Create DMA-BUF surface pool — allocates VAAPI surfaces, exports them
    //    as DMA-BUF FDs, and imports into Level Zero for zero-copy SYCL access.
    //    UV planes are pre-filled with 128 (neutral chroma).
    surface_pool_ = std::make_unique<DmaBufSurfacePool>(
        DMABUF_POOL_SIZE,
        encoder_->hw_frames_ctx(),
        encoder_->va_display(),
        q_);
}

GpuCompressor::~GpuCompressor()
{
    // Destroy pool first (frees Level Zero memory + DMA-BUF FDs),
    // then encoder (frees VAAPI contexts).  Order matters.
    surface_pool_.reset();
    encoder_.reset();
    if (d_input_) sycl::free(d_input_, q_);
}

// ---------------------------------------------------------------------------
// SYCL kernel — Bayer → tiled NV12 into DMA-BUF surface
// ---------------------------------------------------------------------------

void GpuCompressor::bayer_to_tiled_nv12(DmaBufSurface& surface)
{
    const size_t w      = width_;
    const size_t h      = height_;
    const size_t half_w = w / 2;
    const size_t half_h = h / 2;
    const size_t pitch  = surface.y_pitch;
    const size_t y_off  = surface.y_offset;

    uint8_t* in  = d_input_;
    uint8_t* out = surface.sycl_ptr;

    // Separate Bayer channels into quadrants for spatial-aware compression.
    // Each 2×2 Bayer block contributes one pixel to each quadrant:
    //   TL = (even row, even col)   TR = (even row, odd col)
    //   BL = (odd  row, even col)   BR = (odd  row, odd col)
    //
    // Writes only the Y plane at the surface's y_offset with y_pitch stride.
    // UV plane stays at 128 (pre-set at pool construction time).
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

            out[y_off + (quad_y + sub_row) * pitch + (quad_x + sub_col)] = val;
        }
    );
    // No .wait() — in-order queue serializes with the next operation.
}

void GpuCompressor::mono8_to_nv12(DmaBufSurface& surface)
{
    const size_t w     = width_;
    const size_t h     = height_;
    const size_t pitch = surface.y_pitch;
    const size_t y_off = surface.y_offset;

    uint8_t* in  = d_input_;
    uint8_t* out = surface.sycl_ptr;

    q_.parallel_for(
        sycl::range<2>(h, w),
        [=](sycl::id<2> idx) {
            const size_t row = idx[0];
            const size_t col = idx[1];

            const uint8_t val = in[row * w + col];
            out[y_off + row * pitch + col] = val;  // Y plane
        }
    );
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::optional<std::vector<uint8_t>> GpuCompressor::compress(uint8_t const* input_data, std::string const& pixel_format, int64_t pts)
{
    using namespace std::chrono;
    auto gpu_start = high_resolution_clock::now();

    // 1. Upload raw camera data (Bayer / Mono8) to GPU
    q_.memcpy(d_input_, input_data, y_size_);

    // 2. Acquire a DMA-BUF surface from the pool (round-robin)
    auto& surface = surface_pool_->acquire();

    // 3. SYCL kernel writes NV12 directly into the VAAPI surface (zero-copy)
    if (pixel_format == "bayer_rggb8") {
        bayer_to_tiled_nv12(surface);
    } else if (pixel_format == "mono8") {
        mono8_to_nv12(surface);
    } else {
        throw std::runtime_error("Unsupported pixel format for compression: " + pixel_format);
    }

    // 4. Synchronize — ensures SYCL writes are visible to the VAAPI encoder.
    //    On Intel GPUs, q.wait() is sufficient because Level Zero compute and
    //    VAAPI fixed-function engines share the same GPU and LLC.
    q_.wait();

    auto gpu_end = high_resolution_clock::now();
    total_gpu_time_us_ += duration_cast<microseconds>(gpu_end - gpu_start).count();

    // 5. Encode: submit VAAPI surface directly — zero CPU copies.
    //    No av_hwframe_transfer_data(), no GPU→CPU→GPU round-trip.
    auto encode_start = high_resolution_clock::now();
    auto result = encoder_->encode_surface(surface.hw_frame, pts);
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
