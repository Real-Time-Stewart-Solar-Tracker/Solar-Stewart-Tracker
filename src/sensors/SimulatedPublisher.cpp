#include "sensors/SimulatedPublisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#if defined(__linux__)
  #include <cerrno>
  #include <cstring>
  #include <poll.h>
  #include <sys/eventfd.h>
  #include <sys/timerfd.h>
  #include <unistd.h>
#elif defined(_WIN32)
  #include <windows.h>
#endif

namespace solar {

SimulatedPublisher::SimulatedPublisher(Logger& log, Config cfg)
    : log_(log),
      cfg_(cfg),
      rng_(std::random_device{}()) {
    if (cfg_.noise_std > 0.0f) {
        noise_.emplace(0.0f, cfg_.noise_std);
    }
}

SimulatedPublisher::~SimulatedPublisher() {
    stop();
}

void SimulatedPublisher::registerFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(cbMutex_);
    frameCb_ = std::move(cb);
}

bool SimulatedPublisher::start() {
    if (running_.load()) return true;

    if (cfg_.fps <= 0 || cfg_.width <= 0 || cfg_.height <= 0) {
        log_.error("SimulatedPublisher: invalid config (width/height/fps)");
        return false;
    }

    running_.store(true);

    try {
        worker_ = std::thread(&SimulatedPublisher::run_, this);
    } catch (...) {
        running_.store(false);
        log_.error("SimulatedPublisher: failed to start thread");
        return false;
    }

    log_.info("SimulatedPublisher started");
    return true;
}

void SimulatedPublisher::stop() {
    if (!running_.exchange(false)) return;

#if defined(__linux__)
    if (stopFd_ >= 0) {
        const uint64_t one = 1;
        (void)::write(stopFd_, &one, sizeof(one));
    }
#elif defined(_WIN32)
    if (stopEvent_) {
        SetEvent(stopEvent_);
    }
#endif

    if (worker_.joinable()) worker_.join();
    log_.info("SimulatedPublisher stopped");
}

bool SimulatedPublisher::isRunning() const noexcept {
    return running_.load();
}

void SimulatedPublisher::run_() {
    using clock = std::chrono::steady_clock;

#if defined(__linux__)

    timerFd_ = ::timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (timerFd_ < 0) {
        log_.error(std::string("SimulatedPublisher: timerfd_create failed: ") + std::strerror(errno));
        running_.store(false);
        return;
    }

    stopFd_ = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (stopFd_ < 0) {
        log_.error(std::string("SimulatedPublisher: eventfd failed: ") + std::strerror(errno));
        ::close(timerFd_);
        timerFd_ = -1;
        running_.store(false);
        return;
    }

    const double period_s = 1.0 / static_cast<double>(cfg_.fps);

    itimerspec its{};
    its.it_interval.tv_sec  = static_cast<time_t>(period_s);
    its.it_interval.tv_nsec =
        static_cast<long>((period_s - static_cast<double>(its.it_interval.tv_sec)) * 1e9);
    its.it_value = its.it_interval;

    if (::timerfd_settime(timerFd_, 0, &its, nullptr) != 0) {
        log_.error(std::string("SimulatedPublisher: timerfd_settime failed: ") + std::strerror(errno));
        ::close(stopFd_);
        ::close(timerFd_);
        stopFd_ = -1;
        timerFd_ = -1;
        running_.store(false);
        return;
    }

    pollfd fds[2]{};
    fds[0].fd = timerFd_;
    fds[0].events = POLLIN;
    fds[1].fd = stopFd_;
    fds[1].events = POLLIN;

    while (running_.load()) {
        const int pr = ::poll(fds, 2, -1);
        if (pr < 0) {
            if (errno == EINTR) continue;
            log_.error(std::string("SimulatedPublisher: poll failed: ") + std::strerror(errno));
            break;
        }

        if (fds[1].revents & POLLIN) {
            uint64_t v = 0;
            (void)::read(stopFd_, &v, sizeof(v));
            break;
        }

        if (fds[0].revents & POLLIN) {
            uint64_t expirations = 0;
            const ssize_t n = ::read(timerFd_, &expirations, sizeof(expirations));
            if (n != static_cast<ssize_t>(sizeof(expirations))) continue;

            FrameEvent fe;
            fe.frame_id     = ++frameId_;
            fe.t_capture    = clock::now();
            fe.width        = cfg_.width;
            fe.height       = cfg_.height;
            fe.format       = PixelFormat::Gray8;
            fe.stride_bytes = cfg_.width; // packed Gray8: 1 byte per pixel

            generateFrame_(fe);

            FrameCallback cb;
            {
                std::lock_guard<std::mutex> lock(cbMutex_);
                cb = frameCb_;
            }
            if (cb) cb(fe);
        }
    }

    ::close(stopFd_);
    ::close(timerFd_);
    stopFd_ = -1;
    timerFd_ = -1;

#elif defined(_WIN32)

    // Windows: waitable timer + stop event (no sleep-based timing)
    stopEvent_ = CreateEvent(nullptr, TRUE, FALSE, nullptr);
    if (!stopEvent_) {
        log_.error("SimulatedPublisher: CreateEvent failed");
        running_.store(false);
        return;
    }

    timer_ = CreateWaitableTimer(nullptr, FALSE, nullptr);
    if (!timer_) {
        log_.error("SimulatedPublisher: CreateWaitableTimer failed");
        CloseHandle(stopEvent_);
        stopEvent_ = nullptr;
        running_.store(false);
        return;
    }

    const double period_s = 1.0 / static_cast<double>(cfg_.fps);
    const LONGLONG period_100ns = static_cast<LONGLONG>(period_s * 10'000'000.0);

    LARGE_INTEGER dueTime;
    dueTime.QuadPart = -period_100ns;

    if (!SetWaitableTimer(timer_, &dueTime, static_cast<LONG>(period_s * 1000.0), nullptr, nullptr, FALSE)) {
        log_.error("SimulatedPublisher: SetWaitableTimer failed");
        CloseHandle(timer_);
        CloseHandle(stopEvent_);
        timer_ = nullptr;
        stopEvent_ = nullptr;
        running_.store(false);
        return;
    }

    HANDLE handles[2] = { stopEvent_, timer_ };

    while (running_.load()) {
        const DWORD r = WaitForMultipleObjects(2, handles, FALSE, INFINITE);
        if (!running_.load()) break;

        if (r == WAIT_OBJECT_0) {
            break;
        }
        if (r == WAIT_OBJECT_0 + 1) {
            FrameEvent fe;
            fe.frame_id     = ++frameId_;
            fe.t_capture    = clock::now();
            fe.width        = cfg_.width;
            fe.height       = cfg_.height;
            fe.format       = PixelFormat::Gray8;
            fe.stride_bytes = cfg_.width; // packed Gray8: 1 byte per pixel

            generateFrame_(fe);

            FrameCallback cb;
            {
                std::lock_guard<std::mutex> lock(cbMutex_);
                cb = frameCb_;
            }
            if (cb) cb(fe);
        }
    }

    CancelWaitableTimer(timer_);
    CloseHandle(timer_);
    CloseHandle(stopEvent_);
    timer_ = nullptr;
    stopEvent_ = nullptr;

#else
    log_.error("SimulatedPublisher: unsupported platform");
    running_.store(false);
    return;
#endif
}

void SimulatedPublisher::generateFrame_(FrameEvent& fe) {
    const int w = cfg_.width;
    const int h = cfg_.height;
    const int stride = fe.stride_bytes;
    const int r = std::max(1, cfg_.spot_radius);

    fe.data.assign(static_cast<std::size_t>(stride) * static_cast<std::size_t>(h),
                   cfg_.background);

    float cx = static_cast<float>(w) * 0.5f;
    float cy = static_cast<float>(h) * 0.5f;

    if (cfg_.moving_spot) {
        phase_ += 0.05f;
        cx += static_cast<float>(w) * 0.25f * std::cos(phase_);
        cy += static_cast<float>(h) * 0.20f * std::sin(phase_);
    }

    const int icx = static_cast<int>(std::round(cx));
    const int icy = static_cast<int>(std::round(cy));
    const int r2  = r * r;

    for (int y = icy - r; y <= icy + r; ++y) {
        if (y < 0 || y >= h) continue;
        for (int x = icx - r; x <= icx + r; ++x) {
            if (x < 0 || x >= w) continue;
            const int dx = x - icx;
            const int dy = y - icy;
            if ((dx * dx + dy * dy) <= r2) {
                fe.data[static_cast<std::size_t>(y) * static_cast<std::size_t>(stride) +
                        static_cast<std::size_t>(x)] = cfg_.spot_value;
            }
        }
    }

    if (noise_) {
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                auto& px = fe.data[static_cast<std::size_t>(y) * static_cast<std::size_t>(stride) +
                                   static_cast<std::size_t>(x)];
                float v = static_cast<float>(px) + (*noise_)(rng_);
                v = std::clamp(v, 0.0f, 255.0f);
                px = static_cast<uint8_t>(v);
            }
        }
    }
}

} // namespace solar