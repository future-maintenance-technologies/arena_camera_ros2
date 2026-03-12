#include "dmabuf_surface_pool.hpp"

#include <sstream>
#include <stdexcept>
#include <string>

#include <unistd.h>  // close()

extern "C" {
#include <libavutil/hwcontext_vaapi.h>
#include <va/va_drmcommon.h>
}

#include <level_zero/ze_api.h>
#include <sycl/ext/oneapi/backend/level_zero.hpp>

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

DmaBufSurfacePool::DmaBufSurfacePool(int pool_size,
                                     AVBufferRef* hw_frames_ctx,
                                     VADisplay va_display,
                                     sycl::queue& q)
    : q_(q), va_display_(va_display)
{
    // Cache Level Zero context for cleanup in destructor.
    ze_context_ = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(
        q_.get_context());

    surfaces_.resize(static_cast<size_t>(pool_size));

    for (auto& s : surfaces_) {
        allocate_surface(s, hw_frames_ctx);
        export_dmabuf(s);
        import_to_sycl(s);
        init_uv_plane(s);   // queued, not yet executed
    }

    q_.wait();  // ensure all UV memsets complete before first use
}

DmaBufSurfacePool::~DmaBufSurfacePool()
{
    auto ze_ctx = static_cast<ze_context_handle_t>(ze_context_);

    for (auto& s : surfaces_) {
        if (s.sycl_ptr) {
            zeMemFree(ze_ctx, s.sycl_ptr);
            s.sycl_ptr = nullptr;
        }
        if (s.dma_buf_fd >= 0) {
            ::close(s.dma_buf_fd);
            s.dma_buf_fd = -1;
        }
        if (s.hw_frame) {
            av_frame_free(&s.hw_frame);
        }
    }
}

// ---------------------------------------------------------------------------
// acquire
// ---------------------------------------------------------------------------

DmaBufSurface& DmaBufSurfacePool::acquire()
{
    auto& s = surfaces_[next_];
    next_ = (next_ + 1) % surfaces_.size();
    return s;
}

// ---------------------------------------------------------------------------
// describe
// ---------------------------------------------------------------------------

std::string DmaBufSurfacePool::describe() const
{
    if (surfaces_.empty()) return "DmaBufSurfacePool(empty)";

    const auto& s0 = surfaces_[0];
    std::ostringstream oss;
    oss << "DmaBufSurfacePool: " << surfaces_.size() << " surfaces"
        << ", y_pitch="    << s0.y_pitch
        << ", uv_pitch="   << s0.uv_pitch
        << ", y_offset="   << s0.y_offset
        << ", uv_offset="  << s0.uv_offset
        << ", total_size=" << s0.total_size;
    return oss.str();
}

// ---------------------------------------------------------------------------
// Internal: allocate VAAPI surface from frames context
// ---------------------------------------------------------------------------

void DmaBufSurfacePool::allocate_surface(DmaBufSurface& s,
                                         AVBufferRef* hw_frames_ctx)
{
    s.hw_frame = av_frame_alloc();
    if (!s.hw_frame)
        throw std::runtime_error("DmaBufSurfacePool: av_frame_alloc() failed");

    int ret = av_hwframe_get_buffer(hw_frames_ctx, s.hw_frame, 0);
    if (ret < 0) {
        char errbuf[128];
        av_strerror(ret, errbuf, sizeof(errbuf));
        throw std::runtime_error(
            std::string("DmaBufSurfacePool: av_hwframe_get_buffer failed: ") +
            errbuf);
    }

    // VAAPI frames store the VASurfaceID in data[3] (cast from uintptr_t).
    s.va_surface = static_cast<VASurfaceID>(
        reinterpret_cast<uintptr_t>(s.hw_frame->data[3]));
}

// ---------------------------------------------------------------------------
// Internal: export VAAPI surface as DMA-BUF file descriptor
// ---------------------------------------------------------------------------

void DmaBufSurfacePool::export_dmabuf(DmaBufSurface& s)
{
    VADRMPRIMESurfaceDescriptor prime{};

    VAStatus st = vaExportSurfaceHandle(
        va_display_,
        s.va_surface,
        VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
        VA_EXPORT_SURFACE_READ_WRITE | VA_EXPORT_SURFACE_SEPARATE_LAYERS,
        &prime);

    if (st != VA_STATUS_SUCCESS)
        throw std::runtime_error(
            std::string("DmaBufSurfacePool: vaExportSurfaceHandle failed: ") +
            vaErrorStr(st));

    if (prime.num_objects < 1)
        throw std::runtime_error(
            "DmaBufSurfacePool: vaExportSurfaceHandle returned 0 objects");

    // NV12: single DMA-BUF object containing two layers (Y + UV).
    s.dma_buf_fd  = prime.objects[0].fd;
    s.total_size  = prime.objects[0].size;

    if (prime.num_layers < 2)
        throw std::runtime_error(
            "DmaBufSurfacePool: expected >= 2 NV12 layers, got " +
            std::to_string(prime.num_layers));

    s.y_offset  = prime.layers[0].offset[0];
    s.y_pitch   = prime.layers[0].pitch[0];
    s.uv_offset = prime.layers[1].offset[0];
    s.uv_pitch  = prime.layers[1].pitch[0];

    // Close surplus DMA-BUF FDs (shouldn't happen for NV12, but be safe).
    for (uint32_t i = 1; i < prime.num_objects; ++i)
        ::close(prime.objects[i].fd);
}

// ---------------------------------------------------------------------------
// Internal: import DMA-BUF FD into SYCL as USM device pointer via Level Zero
// ---------------------------------------------------------------------------

void DmaBufSurfacePool::import_to_sycl(DmaBufSurface& s)
{
    auto ze_ctx = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(
        q_.get_context());
    auto ze_dev = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(
        q_.get_device());

    // Import the DMA-BUF FD as Level Zero external memory.
    ze_external_memory_import_fd_t import_fd{};
    import_fd.stype = ZE_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMPORT_FD;
    import_fd.flags = ZE_EXTERNAL_MEMORY_TYPE_FLAG_DMA_BUF;
    import_fd.fd    = s.dma_buf_fd;

    ze_device_mem_alloc_desc_t dev_desc{};
    dev_desc.stype = ZE_STRUCTURE_TYPE_DEVICE_MEM_ALLOC_DESC;
    dev_desc.pNext = &import_fd;

    void* ptr = nullptr;
    ze_result_t res = zeMemAllocDevice(
        ze_ctx, &dev_desc, s.total_size, 0 /* alignment */, ze_dev, &ptr);

    if (res != ZE_RESULT_SUCCESS)
        throw std::runtime_error(
            "DmaBufSurfacePool: zeMemAllocDevice (DMA-BUF import) failed, "
            "ze_result_t=" + std::to_string(static_cast<int>(res)));

    s.sycl_ptr = static_cast<uint8_t*>(ptr);
}

// ---------------------------------------------------------------------------
// Internal: fill UV plane with 128 (neutral grey chroma)
// ---------------------------------------------------------------------------

void DmaBufSurfacePool::init_uv_plane(DmaBufSurface& s)
{
    // UV plane dimensions: full width * half height (NV12 4:2:0).
    auto* fctx = reinterpret_cast<AVHWFramesContext*>(
        s.hw_frame->hw_frames_ctx->data);
    const size_t uv_height = static_cast<size_t>(fctx->height) / 2;
    const size_t uv_bytes  = s.uv_pitch * uv_height;

    q_.memset(s.sycl_ptr + s.uv_offset, 128, uv_bytes);
    // Caller calls q_.wait() after all surfaces are initialized.
}
