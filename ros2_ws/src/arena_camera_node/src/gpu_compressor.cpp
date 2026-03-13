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
    const size_t w = width_;
    const size_t h = height_;

    const size_t half_w = w >> 1;
    const size_t half_h = h >> 1;

    const size_t pitch = surface.y_pitch;
    const size_t y_off = surface.y_offset;

    uint8_t* in  = d_input_;
    uint8_t* out = surface.sycl_ptr;

    constexpr size_t WG_X = 16;
    constexpr size_t WG_Y = 16;

    size_t global_y = ((half_h + WG_Y - 1) / WG_Y) * WG_Y;
    size_t global_x = ((half_w + WG_X - 1) / WG_X) * WG_X;

    sycl::nd_range<2> launch(
        {global_y, global_x},
        {WG_Y, WG_X}
    );

    q_.parallel_for(
        launch,
        [=](sycl::nd_item<2> item)
        {
            const size_t r = item.get_global_id(0);
            const size_t c = item.get_global_id(1);

            if (r >= half_h || c >= half_w)
                return;

            const size_t in_row = r << 1;
            const size_t in_col = c << 1;

            const size_t base = in_row * w + in_col;

            uint8_t p00 = in[base];
            uint8_t p01 = in[base + 1];
            uint8_t p10 = in[base + w];
            uint8_t p11 = in[base + w + 1];

            const size_t y0 = y_off + r * pitch + c;
            const size_t y1 = y_off + r * pitch + (c + half_w);
            const size_t y2 = y_off + (r + half_h) * pitch + c;
            const size_t y3 = y_off + (r + half_h) * pitch + (c + half_w);

            out[y0] = p00;
            out[y1] = p01;
            out[y2] = p10;
            out[y3] = p11;
        }
    );
}

void GpuCompressor::mono8_to_nv12(DmaBufSurface& surface)
{
    const size_t w = width_;
    const size_t h = height_;

    const size_t pitch = surface.y_pitch;
    const size_t y_off = surface.y_offset;

    uint8_t* in  = d_input_;
    uint8_t* out = surface.sycl_ptr;

    constexpr size_t WG_X = 16;
    constexpr size_t WG_Y = 16;

    constexpr size_t vec_width = 16;

    const size_t w_vec = w / vec_width;

    size_t global_y = ((h + WG_Y - 1) / WG_Y) * WG_Y;
    size_t global_x = ((w_vec + WG_X - 1) / WG_X) * WG_X;

    sycl::nd_range<2> launch(
        {global_y, global_x},
        {WG_Y, WG_X}
    );

    q_.parallel_for(
        launch,
        [=](sycl::nd_item<2> item)
        {
            const size_t row = item.get_global_id(0);
            const size_t col = item.get_global_id(1);

            if (row >= h || col >= w_vec)
                return;

            const size_t in_offset  = row * w + col * vec_width;
            const size_t out_offset = y_off + row * pitch + col * vec_width;

            const uint8_t* src = in + in_offset;
            uint8_t* dst = out + out_offset;

            #pragma unroll
            for (int i = 0; i < vec_width; i++)
                dst[i] = src[i];
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
