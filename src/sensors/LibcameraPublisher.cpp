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
#include <algorithm>

namespace solar {
namespace {

// ------------------------------------------------------------
// RAII mmap helper for dmabuf planes with NON-page-aligned offsets.
//
// NOTE: POSIX mmap() returns a void* by design.
// We keep it strictly inside this translation unit and manage it with RAII.
// No void* crosses module boundaries; ownership is local and deterministic.
// ------------------------------------------------------------
class MMapPlaneRO final {
public:
    MMapPlaneRO() = default;
    ~MMapPlaneRO() { reset(); }

    MMapPlaneRO(const MMapPlaneRO&) = delete;
    MMapPlaneRO& operator=(const MMapPlaneRO&) = delete;

    MMapPlaneRO(MMapPlaneRO&& other) noexcept
        : base_(other.base_), mapLen_(other.mapLen_), delta_(other.delta_) {
        other.base_  = nullptr;
        other.mapLen_ = 0;
        other.delta_  = 0;
    }

    MMapPlaneRO& operator=(MMapPlaneRO&& other) noexcept {
        if (this != &other) {
            reset();
            base_   = other.base_;
            mapLen_ = other.mapLen_;
            delta_  = other.delta_;
            other.base_  = nullptr;
            other.mapLen_ = 0;
            other.delta_  = 0;
        }
        return *this;
    }

    static MMapPlaneRO map(int fd, size_t length, size_t offset) {
        if (fd < 0 || length == 0) {
            throw std::runtime_error("MMapPlaneRO: invalid fd/length");
        }

        const long page = ::sysconf(_SC_PAGESIZE);
        if (page <= 0) {
            throw std::runtime_error("MMapPlaneRO: sysconf(_SC_PAGESIZE) failed");
        }

        const size_t pageSize = static_cast<size_t>(page);

        // Down-align the mapping offset to page boundary and compute delta.
        const size_t alignedOffset = offset & ~(pageSize - 1);
        const size_t delta         = offset - alignedOffset;

        // Map extra so [offset .. offset+length) is valid.
        const size_t mapLen = length + delta;

        void* mem = ::mmap(nullptr, mapLen, PROT_READ, MAP_SHARED, fd, static_cast<off_t>(alignedOffset));
        if (mem == MAP_FAILED) {
            throw std::runtime_error("MMapPlaneRO: mmap failed");
        }

        MMapPlaneRO r;
        r.base_   = mem;
        r.mapLen_ = mapLen;
        r.delta_  = delta;
        return r;
    }

    void reset() noexcept {
        if (base_ && base_ != MAP_FAILED && mapLen_ > 0) {
            ::munmap(base_, mapLen_);
        }
        base_ = nullptr;
        mapLen_ = 0;
        delta_ = 0;
    }

    [[nodiscard]] const uint8_t* ptr() const noexcept {
        return static_cast<const uint8_t*>(base_) + delta_;
    }

    [[nodiscard]] bool valid() const noexcept {
        return base_ && base_ != MAP_FAILED && mapLen_ > 0;
    }

private:
    void*  base_{nullptr}; // pointer returned by mmap (page-aligned mapping base)
    size_t mapLen_{0};     // length passed to mmap
    size_t delta_{0};      // offset inside mapping to the requested "offset"
};

// ------------------------------------------------------------
// Copy ONE plane into dst, respecting stride.
// dst layout is packed (rowBytes * rows).
// ------------------------------------------------------------
static void copyPlane_StrideAware(uint8_t* dst,
                                  size_t dstSize,
                                  int fd,
                                  size_t length,
                                  size_t offset,
                                  int rowBytes,
                                  int rows,
                                  int strideBytes)
{
    if (!dst) throw std::runtime_error("copyPlane: dst null");
    if (rowBytes <= 0 || rows <= 0 || strideBytes <= 0) throw std::runtime_error("copyPlane: bad dims");
    const size_t needed = static_cast<size_t>(rowBytes) * static_cast<size_t>(rows);
    if (dstSize < needed) throw std::runtime_error("copyPlane: dst too small");

    MMapPlaneRO map = MMapPlaneRO::map(fd, length, offset);
    const uint8_t* src = map.ptr();

    // We only guarantee [src .. src+length) is valid, so validate row reads.
    for (int y = 0; y < rows; ++y) {
        const size_t srcRow = static_cast<size_t>(y) * static_cast<size_t>(strideBytes);
        const size_t dstRow = static_cast<size_t>(y) * static_cast<size_t>(rowBytes);

        if (srcRow + static_cast<size_t>(rowBytes) > length) {
            throw std::runtime_error("copyPlane: out of bounds (stride/length mismatch)");
        }

        std::memcpy(dst + dstRow, src + srcRow, static_cast<size_t>(rowBytes));
    }
}

// Compute a safe stride from plane length / rows (never less than rowBytes)
static int strideFromPlane(const libcamera::FrameBuffer::Plane& p, int rows, int rowBytes) {
    if (rows <= 0) return rowBytes;
    int s = static_cast<int>(p.length / static_cast<size_t>(rows));
    if (s < rowBytes) s = rowBytes;
    return s;
}

} // namespace

LibcameraPublisher::LibcameraPublisher(Logger& log, Config cfg)
    : log_(log), cfg_(std::move(cfg)) {}

LibcameraPublisher::~LibcameraPublisher() { stop(); }

void LibcameraPublisher::registerFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(cbMutex_);
    frameCb_ = std::move(cb);
}

bool LibcameraPublisher::start() {
    if (running_) return true;

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
    if (!running_) return;

    running_ = false;
    runCv_.notify_all();

    if (thread_.joinable()) thread_.join();
    log_.info("LibcameraPublisher stopped");
}

bool LibcameraPublisher::isRunning() const noexcept { return running_.load(); }

LibcameraPublisher::Config LibcameraPublisher::config() const { return cfg_; }

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
        for (auto& c : cm->cameras()) {
            if (c->id() == cfg_.camera_id) { cam = c; break; }
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
    sc.size.width  = static_cast<unsigned int>(cfg_.width);
    sc.size.height = static_cast<unsigned int>(cfg_.height);

    // YUV420: fast on Pi, SunTracker uses Y plane, UI can reconstruct RGB.
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
        if (!request || !running_) return;
        if (request->status() == libcamera::Request::RequestCancelled) return;

        auto it = request->buffers().find(stream);
        if (it == request->buffers().end() || !it->second) {
            request->reuse(libcamera::Request::ReuseBuffers);
            cam->queueRequest(request);
            return;
        }

        libcamera::FrameBuffer* fb = it->second;
        if (!fb || fb->planes().size() < 3) { // need Y,U,V
            request->reuse(libcamera::Request::ReuseBuffers);
            cam->queueRequest(request);
            return;
        }

        FrameEvent fe;
        fe.frame_id  = frameId_.fetch_add(1) + 1;
        fe.t_capture = std::chrono::steady_clock::now();
        fe.width     = cfg_.width;
        fe.height    = cfg_.height;

        try {
            const int w = cfg_.width;
            const int h = cfg_.height;
            const int uvW = w / 2;
            const int uvH = h / 2;

            const size_t ySize  = static_cast<size_t>(w)   * static_cast<size_t>(h);
            const size_t uvSize = static_cast<size_t>(uvW) * static_cast<size_t>(uvH);

            // packed as [Y][U][V]
            fe.data.resize(ySize + uvSize + uvSize);

            const auto& pY = fb->planes()[0];
            const auto& pU = fb->planes()[1];
            const auto& pV = fb->planes()[2];

            const int strideY = strideFromPlane(pY, h,  w);
            const int strideU = strideFromPlane(pU, uvH, uvW);
            const int strideV = strideFromPlane(pV, uvH, uvW);

            copyPlane_StrideAware(fe.data.data(), fe.data.size(),
                                  pY.fd.get(), pY.length, pY.offset,
                                  w, h, strideY);

            copyPlane_StrideAware(fe.data.data() + ySize, fe.data.size() - ySize,
                                  pU.fd.get(), pU.length, pU.offset,
                                  uvW, uvH, strideU);

            copyPlane_StrideAware(fe.data.data() + ySize + uvSize,
                                  fe.data.size() - (ySize + uvSize),
                                  pV.fd.get(), pV.length, pV.offset,
                                  uvW, uvH, strideV);

            FrameCallback cbCopy;
            {
                std::lock_guard<std::mutex> lock(cbMutex_);
                cbCopy = frameCb_;
            }
            if (cbCopy) cbCopy(fe);

        } catch (const std::exception& e) {
            log_.error(std::string("LibcameraPublisher: frame copy failed: ") + e.what());
        }

        request->reuse(libcamera::Request::ReuseBuffers);
        cam->queueRequest(request);
    });

    // FPS control
    libcamera::ControlList controls(cam->controls());
    if (cfg_.fps > 0) {
        const int64_t frame_us = 1000000LL / cfg_.fps;
        const std::array<int64_t, 2> limits{ frame_us, frame_us };
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

    // Create + queue requests
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