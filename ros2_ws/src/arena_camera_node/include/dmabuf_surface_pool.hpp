#pragma once
/// @file dmabuf_surface_pool.hpp
/// Zero-copy DMA-BUF surface pool for SYCL → VAAPI AV1 encoding.
///
/// Allocates VAAPI NV12 surfaces, exports each as a DMA-BUF file descriptor,
/// and imports them into SYCL via Level Zero external memory so that GPU
/// compute kernels can write directly into encode surfaces — eliminating all
/// GPU→CPU and CPU→GPU copies from the pipeline.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <sycl/sycl.hpp>

extern "C" {
#include <libavutil/frame.h>
#include <libavutil/hwcontext.h>
#include <va/va.h>
}

/// Metadata for a single zero-copy DMA-BUF surface.
struct DmaBufSurface {
    AVFrame*      hw_frame   = nullptr;             ///< VAAPI surface (owns VA surface)
    VASurfaceID   va_surface = VA_INVALID_SURFACE;  ///< Underlying VA surface ID
    int           dma_buf_fd = -1;                  ///< Exported DMA-BUF file descriptor
    uint8_t*      sycl_ptr   = nullptr;             ///< Level-Zero-imported USM device ptr
    size_t        y_offset   = 0;                   ///< Y  plane byte offset in DMA-BUF
    size_t        uv_offset  = 0;                   ///< UV plane byte offset in DMA-BUF
    size_t        y_pitch    = 0;                   ///< Y  plane row stride  (bytes)
    size_t        uv_pitch   = 0;                   ///< UV plane row stride  (bytes)
    size_t        total_size = 0;                   ///< Total DMA-BUF buffer size (bytes)
};

constexpr int DMABUF_POOL_SIZE = 32;

/// Pool of DMA-BUF–backed VAAPI NV12 surfaces importable into SYCL.
///
/// Lifecycle:
///   1. Construction allocates all surfaces, exports DMA-BUF FDs,
///      imports them into Level Zero, and pre-fills the UV plane with 128.
///   2. acquire() returns the next surface (round-robin).
///   3. Caller writes NV12 Y-plane data via `sycl_ptr + y_offset`
///      using `y_pitch` as the row stride, then calls `q.wait()`.
///   4. Caller passes `hw_frame` directly to the encoder — no transfer.
class DmaBufSurfacePool {
public:
    /// @param pool_size       Number of surfaces (e.g. DMABUF_POOL_SIZE).
    /// @param hw_frames_ctx   Initialized VAAPI hardware frames context.
    /// @param va_display      VAAPI display handle.
    /// @param q               SYCL in-order queue (same GPU as VAAPI device).
    DmaBufSurfacePool(int pool_size,
                      AVBufferRef* hw_frames_ctx,
                      VADisplay va_display,
                      sycl::queue& q);
    ~DmaBufSurfacePool();

    DmaBufSurfacePool(const DmaBufSurfacePool&) = delete;
    DmaBufSurfacePool& operator=(const DmaBufSurfacePool&) = delete;

    /// Get the next surface (round-robin).  Caller must not hold more
    /// than `pool_size - encoder_async_depth` surfaces simultaneously.
    DmaBufSurface& acquire();

    size_t size() const { return surfaces_.size(); }

    /// Human-readable description for logging.
    std::string describe() const;

private:
    sycl::queue& q_;
    VADisplay    va_display_;
    void*        ze_context_ = nullptr;   ///< Cached Level Zero context handle

    std::vector<DmaBufSurface> surfaces_;
    size_t next_ = 0;

    void allocate_surface(DmaBufSurface& s, AVBufferRef* hw_frames_ctx);
    void export_dmabuf(DmaBufSurface& s);
    void import_to_sycl(DmaBufSurface& s);
    void init_uv_plane(DmaBufSurface& s);
};
