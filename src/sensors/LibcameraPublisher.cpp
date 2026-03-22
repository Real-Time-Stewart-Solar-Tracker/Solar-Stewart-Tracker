#if SOLAR_HAVE_LIBCAMERA

#include "sensors/LibcameraPublisher.hpp"

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <sys/mman.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace solar {
namespace {

/**
 * @brief RAII read-only mmap helper for dmabuf planes.
 *
 * Keeps all mmap/munmap handling local to this translation unit.
 */
class MMapPlaneRO final {
public:
    MMapPlaneRO() = default;
    ~MMapPlaneRO() { reset(); }

    MMapPlaneRO(const MMapPlaneRO&) = delete;
    MMapPlaneRO& operator=(const MMapPlaneRO&) = delete;

    MMapPlaneRO(MMapPlaneRO&& other) noexcept
        : base_(other.base_),
          mapLen_(other.mapLen_),
          delta_(other.delta_) {
        other.base_ = nullptr;
        other.mapLen_ = 0;
        other.delta_ = 0;
    }

    MMapPlaneRO& operator=(MMapPlaneRO&& other) noexcept {
        if (this != &other) {
            reset();
            base_ = other.base_;
            mapLen_ = other.mapLen_;
            delta_ = other.delta_;
            other.base_ = nullptr;
            other.mapLen_ = 0;
            other.delta_ = 0;
        }
        return *this;
    }

    static MMapPlaneRO map(int fd, std::size_t length, std::size_t offset) {
        if (fd < 0 || length == 0U) {
            throw std::runtime_error("MMapPlaneRO: invalid fd/length");
        }

        const long page = ::sysconf(_SC_PAGESIZE);
        if (page <= 0) {
            throw std::runtime_error("MMapPlaneRO: sysconf(_SC_PAGESIZE) failed");
        }

        const std::size_t pageSize = static_cast<std::size_t>(page);
        const std::size_t alignedOffset = offset & ~(pageSize - 1U);
        const std::size_t delta = offset - alignedOffset;
        const std::size_t mapLen = length + delta;

        void* mem = ::mmap(
            nullptr,
            mapLen,
            PROT_READ,
            MAP_SHARED,
            fd,
            static_cast<off_t>(alignedOffset));

        if (mem == MAP_FAILED) {
            throw std::runtime_error("MMapPlaneRO: mmap failed");
        }

        MMapPlaneRO r;
        r.base_ = mem;
        r.mapLen_ = mapLen;
        r.delta_ = delta;
        return r;
    }

    void reset() noexcept {
        if (base_ && base_ != MAP_FAILED && mapLen_ > 0U) {
            ::munmap(base_, mapLen_);
        }
        base_ = nullptr;
        mapLen_ = 0U;
        delta_ = 0U;
    }

    [[nodiscard]] const uint8_t* ptr() const noexcept {
        return static_cast<const uint8_t*>(base_) + delta_;
    }

private:
    void* base_{nullptr};
    std::size_t mapLen_{0U};
    std::size_t delta_{0U};
};

/**
 * @brief Copy one plane into a packed destination buffer, respecting source stride.
 *
 * Destination layout is packed: rowBytes * rows.
 */
static void copyPlaneStrideAware(uint8_t* dst,
                                 std::size_t dstSize,
                                 int fd,
                                 std::size_t length,
                                 std::size_t offset,
                                 int rowBytes,
                                 int rows,
                                 int strideBytes) {
    if (!dst) {
        throw std::runtime_error("copyPlaneStrideAware: dst null");
    }
    if (rowBytes <= 0 || rows <= 0 || strideBytes <= 0) {
        throw std::runtime_error("copyPlaneStrideAware: invalid dimensions");
    }

    const std::size_t needed =
        static_cast<std::size_t>(rowBytes) * static_cast<std::size_t>(rows);
    if (dstSize < needed) {
        throw std::runtime_error("copyPlaneStrideAware: destination too small");
    }

    MMapPlaneRO map = MMapPlaneRO::map(fd, length, offset);
    const uint8_t* src = map.ptr();

    for (int y = 0; y < rows; ++y) {
        const std::size_t srcRow =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(strideBytes);
        const std::size_t dstRow =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(rowBytes);

        if (srcRow + static_cast<std::size_t>(rowBytes) > length) {
            throw std::runtime_error("copyPlaneStrideAware: plane bounds exceeded");
        }

        std::memcpy(dst + dstRow, src + srcRow, static_cast<std::size_t>(rowBytes));
    }
}

/**
 * @brief Compute a conservative stride estimate from plane size and row count.
 */
static int strideFromPlane(const libcamera::FrameBuffer::Plane& p,
                           int rows,
                           int minRowBytes) {
    if (rows <= 0) {
        return minRowBytes;
    }

    int stride = static_cast<int>(p.length / static_cast<std::size_t>(rows));
    if (stride < minRowBytes) {
        stride = minRowBytes;
    }
    return stride;
}

} // namespace

LibcameraPublisher::LibcameraPublisher(Logger& log, Config cfg)
    : log_(log), cfg_(std::move(cfg)) {}

LibcameraPublisher::~LibcameraPublisher() {
    stop();
}

void LibcameraPublisher::registerFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(cbMutex_);
    frameCb_ = std::move(cb);
}

bool LibcameraPublisher::start() {
    if (running_) {
        return true;
    }

    running_ = true;
    try {
        thread_ = std::thread(&LibcameraPublisher::run_, this);
    } catch (...) {
        running_ = false;
        log_.error("LibcameraPublisher: failed to start thread");
        return false;
    }

    log_.info("LibcameraPublisher started");
    return true;
}

void LibcameraPublisher::stop() {
    if (!running_) {
        return;
    }

    running_ = false;
    runCv_.notify_all();

    if (thread_.joinable()) {
        thread_.join();
    }

    log_.info("LibcameraPublisher stopped");
}

bool LibcameraPublisher::isRunning() const noexcept {
    return running_.load();
}

LibcameraPublisher::Config LibcameraPublisher::config() const {
    return cfg_;
}

void LibcameraPublisher::run_() {
    log_.info("LibcameraPublisher: libcamera thread starting");

    auto cm = std::make_unique<libcamera::CameraManager>();
    if (cm->start() < 0) {
        log_.error("LibcameraPublisher: CameraManager start failed");
        running_ = false;
        return;
    }

    if (cm->cameras().empty()) {
        log_.error("LibcameraPublisher: no cameras found");
        cm->stop();
        running_ = false;
        return;
    }

    std::shared_ptr<libcamera::Camera> cam;

    if (!cfg_.camera_id.empty()) {
        for (const auto& c : cm->cameras()) {
            if (c->id() == cfg_.camera_id) {
                cam = c;
                break;
            }
        }
        if (!cam) {
            log_.warn("LibcameraPublisher: camera_id not found; using first camera");
            cam = cm->cameras().front();
        }
    } else {
        cam = cm->cameras().front();
    }

    if (cam->acquire() < 0) {
        log_.error("LibcameraPublisher: failed to acquire camera");
        cm->stop();
        running_ = false;
        return;
    }

    auto camCfg = cam->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!camCfg || camCfg->empty()) {
        log_.error("LibcameraPublisher: generateConfiguration failed");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    libcamera::StreamConfiguration& sc = camCfg->at(0);
    sc.size.width = static_cast<unsigned int>(cfg_.width);
    sc.size.height = static_cast<unsigned int>(cfg_.height);

    // Keep capture efficient on Raspberry Pi and convert to Gray8 at the FrameEvent boundary
    // by copying only the Y plane into a packed grayscale buffer.
    sc.pixelFormat = libcamera::formats::YUV420;

    if (camCfg->validate() == libcamera::CameraConfiguration::Invalid) {
        log_.error("LibcameraPublisher: configuration invalid");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    if (cam->configure(camCfg.get()) < 0) {
        log_.error("LibcameraPublisher: configure failed");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    libcamera::Stream* stream = sc.stream();
    if (!stream) {
        log_.error("LibcameraPublisher: stream is null");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    libcamera::FrameBufferAllocator allocator(cam);
    if (allocator.allocate(stream) < 0) {
        log_.error("LibcameraPublisher: buffer allocation failed");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    const auto& buffers = allocator.buffers(stream);
    if (buffers.empty()) {
        log_.error("LibcameraPublisher: no buffers allocated");
        allocator.free(stream);
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    cam->requestCompleted.connect(this, [this, cam, stream](libcamera::Request* request) {
        if (!request || !running_) {
            return;
        }
        if (request->status() == libcamera::Request::RequestCancelled) {
            return;
        }

        auto it = request->buffers().find(stream);
        if (it == request->buffers().end() || !it->second) {
            request->reuse(libcamera::Request::ReuseBuffers);
            cam->queueRequest(request);
            return;
        }

        libcamera::FrameBuffer* fb = it->second;
        if (!fb || fb->planes().empty()) {
            request->reuse(libcamera::Request::ReuseBuffers);
            cam->queueRequest(request);
            return;
        }

        FrameEvent fe;
        fe.frame_id = frameId_.fetch_add(1) + 1U;
        fe.t_capture = std::chrono::steady_clock::now();
        fe.width = cfg_.width;
        fe.height = cfg_.height;
        fe.format = PixelFormat::Gray8;
        fe.stride_bytes = cfg_.width;

        try {
            const int w = cfg_.width;
            const int h = cfg_.height;
            const std::size_t ySize =
                static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

            fe.data.resize(ySize);

            const auto& pY = fb->planes()[0];
            const int strideY = strideFromPlane(pY, h, w);

            copyPlaneStrideAware(
                fe.data.data(),
                fe.data.size(),
                pY.fd.get(),
                pY.length,
                pY.offset,
                w,
                h,
                strideY);

            FrameCallback cbCopy;
            {
                std::lock_guard<std::mutex> lock(cbMutex_);
                cbCopy = frameCb_;
            }
            if (cbCopy) {
                cbCopy(fe);
            }
        } catch (const std::exception& e) {
            log_.error(std::string("LibcameraPublisher: frame copy failed: ") + e.what());
        }

        request->reuse(libcamera::Request::ReuseBuffers);
        cam->queueRequest(request);
    });

    libcamera::ControlList controls(cam->controls());
    if (cfg_.fps > 0) {
        const int64_t frame_us = 1000000LL / cfg_.fps;
        const std::array<int64_t, 2> limits{frame_us, frame_us};
        controls.set(libcamera::controls::FrameDurationLimits, limits);
    }

    if (cam->start(&controls) < 0) {
        log_.error("LibcameraPublisher: camera start failed");
        allocator.free(stream);
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    std::vector<std::unique_ptr<libcamera::Request>> requests;
    requests.reserve(buffers.size());

    for (auto& buf : buffers) {
        auto req = cam->createRequest();
        if (!req) {
            log_.error("LibcameraPublisher: createRequest failed");
            continue;
        }
        if (req->addBuffer(stream, buf.get()) < 0) {
            log_.error("LibcameraPublisher: addBuffer failed");
            continue;
        }
        requests.push_back(std::move(req));
    }

    for (auto& req : requests) {
        if (cam->queueRequest(req.get()) < 0) {
            log_.error("LibcameraPublisher: queueRequest failed");
        }
    }

    log_.info("LibcameraPublisher: streaming started");

    {
        std::unique_lock<std::mutex> lk(runMutex_);
        runCv_.wait(lk, [this] { return !running_.load(); });
    }

    cam->stop();
    allocator.free(stream);
    cam->release();
    cm->stop();

    log_.info("LibcameraPublisher: libcamera thread exiting");
}

} // namespace solar

#endif // SOLAR_HAVE_LIBCAMERA