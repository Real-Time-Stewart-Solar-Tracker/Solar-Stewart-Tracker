#include "sensors/LibcameraPublisher.hpp"

#if SOLAR_HAVE_LIBCAMERA

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
 * @brief Read-only mmap helper for one dmabuf plane.
 */
class MMapPlaneRO final {
public:
    MMapPlaneRO() = default;
    ~MMapPlaneRO() { reset(); }

    MMapPlaneRO(const MMapPlaneRO&) = delete;
    MMapPlaneRO& operator=(const MMapPlaneRO&) = delete;

    MMapPlaneRO(MMapPlaneRO&& other) noexcept
        : base_(other.base_),
          map_len_(other.map_len_),
          delta_(other.delta_) {
        other.base_ = nullptr;
        other.map_len_ = 0U;
        other.delta_ = 0U;
    }

    MMapPlaneRO& operator=(MMapPlaneRO&& other) noexcept {
        if (this != &other) {
            reset();
            base_ = other.base_;
            map_len_ = other.map_len_;
            delta_ = other.delta_;
            other.base_ = nullptr;
            other.map_len_ = 0U;
            other.delta_ = 0U;
        }
        return *this;
    }

    static MMapPlaneRO map(int fd, std::size_t length, std::size_t offset) {
        if (fd < 0 || length == 0U) {
            throw std::runtime_error("MMapPlaneRO: invalid fd or length");
        }

        const long page = ::sysconf(_SC_PAGESIZE);
        if (page <= 0) {
            throw std::runtime_error("MMapPlaneRO: sysconf(_SC_PAGESIZE) failed");
        }

        const std::size_t page_size = static_cast<std::size_t>(page);
        const std::size_t aligned_offset = offset & ~(page_size - 1U);
        const std::size_t delta = offset - aligned_offset;
        const std::size_t map_len = length + delta;

        void* mem = ::mmap(
            nullptr,
            map_len,
            PROT_READ,
            MAP_SHARED,
            fd,
            static_cast<off_t>(aligned_offset));

        if (mem == MAP_FAILED) {
            throw std::runtime_error("MMapPlaneRO: mmap failed");
        }

        MMapPlaneRO out;
        out.base_ = mem;
        out.map_len_ = map_len;
        out.delta_ = delta;
        return out;
    }

    void reset() noexcept {
        if (base_ && base_ != MAP_FAILED && map_len_ > 0U) {
            ::munmap(base_, map_len_);
        }
        base_ = nullptr;
        map_len_ = 0U;
        delta_ = 0U;
    }

    [[nodiscard]] const std::uint8_t* ptr() const noexcept {
        return static_cast<const std::uint8_t*>(base_) + delta_;
    }

private:
    void* base_{nullptr};
    std::size_t map_len_{0U};
    std::size_t delta_{0U};
};

/**
 * @brief Copy one plane into a packed destination buffer respecting source stride.
 */
void copyPlaneStrideAware(std::uint8_t* dst,
                          std::size_t dst_size,
                          int fd,
                          std::size_t length,
                          std::size_t offset,
                          int row_bytes,
                          int rows,
                          int stride_bytes) {
    if (!dst) {
        throw std::runtime_error("copyPlaneStrideAware: null destination");
    }
    if (row_bytes <= 0 || rows <= 0 || stride_bytes <= 0) {
        throw std::runtime_error("copyPlaneStrideAware: invalid dimensions");
    }

    const std::size_t needed =
        static_cast<std::size_t>(row_bytes) * static_cast<std::size_t>(rows);
    if (dst_size < needed) {
        throw std::runtime_error("copyPlaneStrideAware: destination too small");
    }

    MMapPlaneRO map = MMapPlaneRO::map(fd, length, offset);
    const std::uint8_t* src = map.ptr();

    for (int y = 0; y < rows; ++y) {
        const std::size_t src_row =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(stride_bytes);
        const std::size_t dst_row =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(row_bytes);

        if (src_row + static_cast<std::size_t>(row_bytes) > length) {
            throw std::runtime_error("copyPlaneStrideAware: plane bounds exceeded");
        }

        std::memcpy(dst + dst_row, src + src_row, static_cast<std::size_t>(row_bytes));
    }
}

/**
 * @brief Conservative stride estimate when libcamera plane metadata is limited.
 */
int strideFromPlane(const libcamera::FrameBuffer::Plane& p,
                    int rows,
                    int min_row_bytes) {
    if (rows <= 0) {
        return min_row_bytes;
    }

    int stride = static_cast<int>(p.length / static_cast<std::size_t>(rows));
    if (stride < min_row_bytes) {
        stride = min_row_bytes;
    }
    return stride;
}

} // namespace

LibcameraPublisher::LibcameraPublisher(Logger& log, Config cfg)
    : log_(log),
      cfg_(std::move(cfg)) {
}

LibcameraPublisher::~LibcameraPublisher() {
    stop();
}

void LibcameraPublisher::registerFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    frame_cb_ = std::move(cb);
}

bool LibcameraPublisher::start() {
    // Libcamera drives frame delivery through callbacks; there is no polling loop here.
    if (running_) {
        return true;
    }

    running_ = true;
    try {
        thread_ = std::thread(&LibcameraPublisher::run_, this);
    } catch (...) {
        running_ = false;
        log_.error("LibcameraPublisher: failed to start worker thread");
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
    run_cv_.notify_all();

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
    log_.info("LibcameraPublisher: worker thread starting");

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
            log_.warn("LibcameraPublisher: requested camera_id not found; using first camera");
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

    auto cam_cfg = cam->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!cam_cfg || cam_cfg->empty()) {
        log_.error("LibcameraPublisher: generateConfiguration failed");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    libcamera::StreamConfiguration& sc = cam_cfg->at(0);
    sc.size.width = static_cast<unsigned int>(cfg_.width);
    sc.size.height = static_cast<unsigned int>(cfg_.height);
    sc.pixelFormat = libcamera::formats::YUV420;

    if (cam_cfg->validate() == libcamera::CameraConfiguration::Invalid) {
        log_.error("LibcameraPublisher: configuration invalid");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    if (cam->configure(cam_cfg.get()) < 0) {
        log_.error("LibcameraPublisher: configure failed");
        cam->release();
        cm->stop();
        running_ = false;
        return;
    }

    libcamera::Stream* stream = sc.stream();
    if (!stream) {
        log_.error("LibcameraPublisher: null stream");
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

        FrameEvent fe{};
        fe.frame_id = frame_id_.fetch_add(1U) + 1U;
        fe.t_capture = std::chrono::steady_clock::now();
        fe.width = cfg_.width;
        fe.height = cfg_.height;
        fe.format = PixelFormat::Gray8;
        fe.stride_bytes = cfg_.width;

        try {
            const int w = cfg_.width;
            const int h = cfg_.height;
            const std::size_t y_size =
                static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

            fe.data.resize(y_size);

            const auto& p_y = fb->planes()[0];
            const int stride_y = strideFromPlane(p_y, h, w);

            copyPlaneStrideAware(
                fe.data.data(),
                fe.data.size(),
                p_y.fd.get(),
                p_y.length,
                p_y.offset,
                w,
                h,
                stride_y);

            FrameCallback cb_copy;
            {
                std::lock_guard<std::mutex> lock(cb_mutex_);
                cb_copy = frame_cb_;
            }
            if (cb_copy) {
                cb_copy(fe);
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

    // libcamera delivers frames asynchronously through its own internal event
    // loop via the requestCompleted signal connected above. This worker thread
    // does not need to poll or block on a file descriptor to receive frames;
    // instead it parks here on a condition variable and keeps the camera
    // manager alive until stop() signals shutdown. This is the integration
    // model prescribed by the libcamera API and by the libcamera2opencv
    // wrapper that is used as the primary camera backend.
    {
        std::unique_lock<std::mutex> lk(run_mutex_);
        run_cv_.wait(lk, [this] { return !running_.load(); });
    }

    cam->stop();
    allocator.free(stream);
    cam->release();
    cm->stop();

    log_.info("LibcameraPublisher: worker thread exiting");
}

} // namespace solar

#endif // SOLAR_HAVE_LIBCAMERA