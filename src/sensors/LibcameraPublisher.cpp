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
#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace solar {
namespace {

// Copy Y plane into packed grayscale buffer (width*height), respecting stride.
static void copyPlaneY_StrideAware(std::vector<uint8_t>& outGray,
                                  int fd,
                                  std::size_t length,
                                  std::size_t offset,
                                  int width,
                                  int height,
                                  int strideBytes)
{
    if (fd < 0 || length == 0) {
        throw std::runtime_error("Invalid buffer plane (fd/length)");
    }
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Invalid dimensions");
    }
    if (strideBytes <= 0) {
        throw std::runtime_error("Invalid stride");
    }

    void* mem = mmap(nullptr, length, PROT_READ, MAP_SHARED, fd, static_cast<off_t>(offset));
    if (mem == MAP_FAILED) {
        throw std::runtime_error("mmap failed for buffer plane");
    }

    const uint8_t* src = static_cast<const uint8_t*>(mem);
    outGray.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));

    for (int y = 0; y < height; ++y) {
        const std::size_t srcRow = static_cast<std::size_t>(y) * static_cast<std::size_t>(strideBytes);
        const std::size_t dstRow = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);

        if (srcRow + static_cast<std::size_t>(width) > length) {
            munmap(mem, length);
            throw std::runtime_error("Plane copy out of bounds (stride/length mismatch)");
        }

        std::memcpy(outGray.data() + dstRow, src + srcRow, static_cast<std::size_t>(width));
    }

    munmap(mem, length);
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
        for (auto& c : cm->cameras()) {
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
    sc.size.width  = static_cast<unsigned int>(cfg_.width);
    sc.size.height = static_cast<unsigned int>(cfg_.height);

    // Try a format with a dedicated luma plane.
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

    const int strideBytes = static_cast<int>(sc.stride);

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

    std::vector<std::unique_ptr<libcamera::Request>> requests;
    requests.reserve(buffers.size());

    cam->requestCompleted.connect([this, stream, strideBytes](libcamera::Request* request) {
        if (!request) return;
        if (!running_) return;

        if (request->status() == libcamera::Request::RequestCancelled) {
            return;
        }

        auto it = request->buffers().find(stream);
        if (it == request->buffers().end() || !it->second) {
            request->reuse(libcamera::Request::ReuseBuffers);
            request->camera()->queueRequest(request);
            return;
        }

        libcamera::FrameBuffer* fb = it->second;
        if (fb->planes().empty()) {
            request->reuse(libcamera::Request::ReuseBuffers);
            request->camera()->queueRequest(request);
            return;
        }

        FrameEvent fe;
        fe.frame_id  = frameId_.fetch_add(1) + 1;
        fe.t_capture = std::chrono::steady_clock::now();
        fe.width     = cfg_.width;
        fe.height    = cfg_.height;

        try {
            const auto& p0 = fb->planes().front();
            copyPlaneY_StrideAware(fe.data,
                                   p0.fd.get(),
                                   p0.length,
                                   p0.offset,
                                   cfg_.width,
                                   cfg_.height,
                                   strideBytes);

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
        request->camera()->queueRequest(request);
    });

    libcamera::ControlList controls(cam->controls());
    if (cfg_.fps > 0) {
        const int64_t frame_us = 1000000LL / cfg_.fps;
        const std::array<int64_t, 2> limits{frame_us, frame_us};
        controls.set(libcamera::controls::FrameDurationLimits,
                     libcamera::Span<const int64_t>(limits.data(), limits.size()));
    }

    if (cam->start(&controls) < 0) {
        log_.error("LibcameraPublisher: camera start failed");
        allocator.free(stream);
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

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
