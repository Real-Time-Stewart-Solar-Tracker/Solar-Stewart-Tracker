#include "sensors/SimulatedPublisher.hpp"

#include "common/Types.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <utility>
#include <vector>

#include <poll.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <unistd.h>

namespace solar {

SimulatedPublisher::SimulatedPublisher(Logger& log, Config cfg)
    : log_(log),
      cfg_(std::move(cfg)) {
    // Allocate the frame pixel buffer once at construction time so the worker
    // loop can fill it on each timer tick without triggering heap allocation.
    const int w = std::max(cfg_.width, 1);
    const int h = std::max(cfg_.height, 1);
    pixel_buf_.resize(static_cast<std::size_t>(w * h));
}

SimulatedPublisher::~SimulatedPublisher() {
    stop();
}

void SimulatedPublisher::registerFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mtx_);
    callback_ = std::move(cb);
}

bool SimulatedPublisher::openTimingFds_() {
    timer_fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (timer_fd_ < 0) {
        log_.error(std::string("SimulatedPublisher: timerfd_create failed: ") + std::strerror(errno));
        return false;
    }

    wake_fd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (wake_fd_ < 0) {
        log_.error(std::string("SimulatedPublisher: eventfd failed: ") + std::strerror(errno));
        closeTimingFds_();
        return false;
    }

    if (!armTimer_()) {
        closeTimingFds_();
        return false;
    }

    return true;
}

void SimulatedPublisher::closeTimingFds_() {
    if (timer_fd_ >= 0) {
        ::close(timer_fd_);
        timer_fd_ = -1;
    }
    if (wake_fd_ >= 0) {
        ::close(wake_fd_);
        wake_fd_ = -1;
    }
}

bool SimulatedPublisher::armTimer_() const {
    const int fps = std::max(cfg_.fps, 1);

    itimerspec spec{};
    spec.it_interval.tv_sec = 0;
    spec.it_interval.tv_nsec = static_cast<long>(1'000'000'000LL / fps);
    spec.it_value = spec.it_interval;

    if (timerfd_settime(timer_fd_, 0, &spec, nullptr) != 0) {
        log_.error(std::string("SimulatedPublisher: timerfd_settime failed: ") + std::strerror(errno));
        return false;
    }

    return true;
}

bool SimulatedPublisher::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true;
    }

    if (!openTimingFds_()) {
        running_.store(false);
        return false;
    }

    worker_ = std::thread([this] {
        workerLoop_();
    });

    log_.info("SimulatedPublisher started");
    return true;
}

void SimulatedPublisher::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }

    if (wake_fd_ >= 0) {
        const std::uint64_t one = 1U;
        const ssize_t written = ::write(wake_fd_, &one, sizeof(one));
        (void)written;
    }

    if (worker_.joinable()) {
        worker_.join();
    }

    closeTimingFds_();
    log_.info("SimulatedPublisher stopped");
}

bool SimulatedPublisher::isRunning() const {
    return running_.load();
}

void SimulatedPublisher::workerLoop_() {
    std::uint64_t frame_id = 1U;

    pollfd fds[2]{};
    fds[0].fd = timer_fd_;
    fds[0].events = POLLIN;
    fds[1].fd = wake_fd_;
    fds[1].events = POLLIN;

    // The worker thread sleeps in poll() and wakes only on a timer expiration
    // or an explicit shutdown signal. No sleep-based pacing is used.
    while (running_.load()) {
        const int ret = ::poll(fds, 2, -1);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }

            log_.error(std::string("SimulatedPublisher: poll failed: ") + std::strerror(errno));
            break;
        }

        if ((fds[1].revents & POLLIN) != 0) {
            std::uint64_t wake_value = 0U;
            const ssize_t n = ::read(wake_fd_, &wake_value, sizeof(wake_value));
            (void)n;
            break;
        }

        if ((fds[0].revents & POLLIN) == 0) {
            continue;
        }

        std::uint64_t expirations = 0U;
        const ssize_t n = ::read(timer_fd_, &expirations, sizeof(expirations));
        if (n != static_cast<ssize_t>(sizeof(expirations))) {
            log_.error("SimulatedPublisher: timer event read failed");
            break;
        }

        // If the process was paused and multiple expirations accumulated, only
        // publish the newest synthetic frame to keep downstream behaviour fresh.
        const float phase = static_cast<float>(frame_id % 10000U) * 0.03F;
        FrameEvent fe = buildFrame_(frame_id, phase);

        FrameCallback cb;
        {
            std::lock_guard<std::mutex> lock(cb_mtx_);
            cb = callback_;
        }

        if (cb) {
            cb(fe);
        }

        ++frame_id;
    }
}

FrameEvent SimulatedPublisher::buildFrame_(const std::uint64_t frame_id, const float phase) const {
    const int width  = std::max(cfg_.width, 1);
    const int height = std::max(cfg_.height, 1);

    // Reuse the pre-allocated buffer: fill with the background value and then
    // draw the spot on top. This avoids a heap allocation on every frame.
    std::fill(pixel_buf_.begin(), pixel_buf_.end(), cfg_.background);

    int cx = width / 2;
    int cy = height / 2;

    if (cfg_.moving_spot) {
        const float rx = static_cast<float>(width)  * 0.28F;
        const float ry = static_cast<float>(height) * 0.22F;

        cx = static_cast<int>(std::lround(static_cast<float>(width)  * 0.5F + rx * std::cos(phase)));
        cy = static_cast<int>(std::lround(static_cast<float>(height) * 0.5F + ry * std::sin(phase * 0.8F)));
    }

    drawSpot_(cx, cy);

    if (cfg_.noise_std > 0.0F) {
        std::mt19937 rng(static_cast<std::uint32_t>(frame_id));
        std::normal_distribution<float> noise(0.0F, cfg_.noise_std);

        for (auto& px : pixel_buf_) {
            const int noisy = static_cast<int>(std::lround(static_cast<float>(px) + noise(rng)));
            px = static_cast<std::uint8_t>(std::clamp(noisy, 0, 255));
        }
    }

    FrameEvent fe{};
    fe.frame_id    = frame_id;
    fe.t_capture   = std::chrono::steady_clock::now();
    fe.data        = pixel_buf_;   // copy the pre-allocated buffer into the event
    fe.width       = width;
    fe.height      = height;
    fe.stride_bytes = width;
    fe.format      = PixelFormat::Gray8;

    return fe;
}

void SimulatedPublisher::drawSpot_(const int cx, const int cy) const {
    const int width     = std::max(cfg_.width, 1);
    const int height    = std::max(cfg_.height, 1);
    const int radius    = std::max(cfg_.spot_radius, 1);
    const int radius_sq = radius * radius;

    for (int y = cy - radius; y <= cy + radius; ++y) {
        if (y < 0 || y >= height) {
            continue;
        }

        for (int x = cx - radius; x <= cx + radius; ++x) {
            if (x < 0 || x >= width) {
                continue;
            }

            const int dx = x - cx;
            const int dy = y - cy;
            const int dist_sq = dx * dx + dy * dy;
            if (dist_sq > radius_sq) {
                continue;
            }

            const float atten =
                1.0F - static_cast<float>(dist_sq) / static_cast<float>(radius_sq);
            const int value = static_cast<int>(
                std::lround(static_cast<float>(cfg_.background) +
                            atten * static_cast<float>(cfg_.spot_value - cfg_.background)));

            pixel_buf_[static_cast<std::size_t>(y * width + x)] =
                static_cast<std::uint8_t>(std::clamp(value, 0, 255));
        }
    }
}

} // namespace solar
