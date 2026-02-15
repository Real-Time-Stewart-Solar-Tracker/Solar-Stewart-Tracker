#ifdef __linux__
#include "sensors/LibcameraPublisher.hpp"

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <chrono>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace solar {

namespace {

// Copy one plane into dst using mmap (read-only).
static void copyPlaneTo(std::vector<uint8_t>& dst, int fd, std::size_t length, std::size_t offset) {
    void* mem = mmap(nullptr, length, PROT_READ, MAP_SHARED, fd, static_cast<off_t>(offset));
    if (mem == MAP_FAILED) {
        throw std::runtime_error("mmap failed for buffer plane");
    }

    const std::size_t oldSize = dst.size();
    dst.resize(oldSize + length);
    std::memcpy(dst.data() + oldSize, mem, length);

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

    std::unique_ptr<libcamera::CameraManager> cm = std::make_unique<libcamera::CameraManager>();
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

    // Select camera (by id if provided, else first camera)
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

    // Configure one Viewfinder stream
    std::unique_ptr<libcamera::CameraConfiguration> cfg =
        cam->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!cfg || cfg->empty()) {
        log_.error("LibcameraPublisher: generateConfiguration failed");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    libcamera::StreamConfiguration& sc = cfg->at(0);
    sc.size.width  = static_cast<unsigned int>(cfg_.width);
    sc.size.height = static_cast<unsigned int>(cfg_.height);

    if (cfg->validate() == libcamera::CameraConfiguration::Invalid) {
        log_.error("LibcameraPublisher: configuration invalid");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    if (cam->configure(cfg.get()) < 0) {
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

    // Allocate buffers
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

    // Requests must stay alive while streaming
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    requests.reserve(buffers.size());

    // Event-driven callback: called when a request completes
    cam->requestCompleted.connect([this, stream](libcamera::Request* request) {
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

        FrameEvent fe;
        fe.frame_id = ++frameId_;
        fe.t_capture = std::chrono::steady_clock::now();
        fe.width = cfg_.width;
        fe.height = cfg_.height;
        fe.data.clear();

        try {
            // Copy all planes into fe.data (concatenated).
            // Later you may interpret/convert formats (e.g., YUV -> grayscale).
            for (const auto& plane : fb->planes()) {
                const int fd = plane.fd.get();
                const std::size_t len = plane.length;
                const std::size_t off = plane.offset;
                if (fd < 0 || len == 0) continue;

                copyPlaneTo(fe.data, fd, len, off);
            }

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

        // Reuse and requeue request
        request->reuse(libcamera::Request::ReuseBuffers);
        request->camera()->queueRequest(request);
    });

    // Start camera with best-effort FPS control
    libcamera::ControlList controls(cam->controls());
    if (cfg_.fps > 0) {
        const int64_t frame_us = 1000000LL / cfg_.fps;
        controls.set(libcamera::controls::FrameDurationLimits,
                     libcamera::Span<const int64_t>({frame_us, frame_us}));
    }

    if (cam->start(&controls) < 0) {
        log_.error("LibcameraPublisher: camera start failed");
        allocator.free(stream);
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    // Create & queue requests
    for (auto& buf : buffers) {
        std::unique_ptr<libcamera::Request> req = cam->createRequest();
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

    log_.info("LibcameraPublisher: streaming started (event-driven)");

    // Block until stop requested (no polling, no sleep)
    {
        std::unique_lock<std::mutex> lk(runMutex_);
        runCv_.wait(lk, [this] { return !running_.load(); });
    }

    // Cleanup
    cam->stop();
    allocator.free(stream);
    cam->release();
    cm->stop();

    log_.info("LibcameraPublisher: libcamera thread exiting");
}

} // namespace solar
#endif