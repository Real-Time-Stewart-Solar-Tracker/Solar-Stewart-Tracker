// #include "sensors/SimulatedPublisher.hpp"

// #include <algorithm>
// #include <chrono>
// #include <cmath>

// namespace solar {

// SimulatedPublisher::SimulatedPublisher(Logger& log, Config cfg)
//     : log_(log),
//       cfg_(cfg),
//       rng_(std::random_device{}()),
//       noise_(0.0f, cfg.noise_std) {}

// SimulatedPublisher::~SimulatedPublisher() {
//     stop();
// }

// void SimulatedPublisher::registerFrameCallback(FrameCallback cb) {
//     std::lock_guard<std::mutex> lock(cbMutex_);
//     frameCb_ = std::move(cb);
// }

// bool SimulatedPublisher::start() {
//     if (running_) return true;

//     if (cfg_.width <= 0 || cfg_.height <= 0 || cfg_.fps <= 0) {
//         log_.error("SimulatedPublisher: invalid config (width/height/fps)");
//         return false;
//     }

//     running_ = true;

//     try {
//         thread_ = std::thread(&SimulatedPublisher::run_, this);
//     } catch (...) {
//         running_ = false;
//         log_.error("SimulatedPublisher: failed to start thread");
//         return false;
//     }

//     log_.info("SimulatedPublisher started");
//     return true;
// }

// void SimulatedPublisher::stop() {
//     if (!running_) return;

//     running_ = false;

//     if (thread_.joinable()) {
//         thread_.join();
//     }

//     log_.info("SimulatedPublisher stopped");
// }

// bool SimulatedPublisher::isRunning() const noexcept {
//     return running_.load();
// }

// SimulatedPublisher::Config SimulatedPublisher::config() const {
//     return cfg_;
// }

// void SimulatedPublisher::run_() {
//     using clock = std::chrono::steady_clock;

//     const auto period = std::chrono::microseconds(static_cast<int>(1000000.0 / cfg_.fps));
//     auto nextTick = clock::now();

//     while (running_) {
//         nextTick += period;

//         FrameEvent fe;
//         fe.frame_id = ++frameId_;
//         fe.t_capture = clock::now();
//         fe.width = cfg_.width;
//         fe.height = cfg_.height;
//         fe.data.clear();
//         fe.data.reserve(static_cast<std::size_t>(cfg_.width) * static_cast<std::size_t>(cfg_.height));

//         generateFrame_(fe);

//         FrameCallback cbCopy;
//         {
//             std::lock_guard<std::mutex> lock(cbMutex_);
//             cbCopy = frameCb_;
//         }
//         if (cbCopy) {
//             cbCopy(fe);
//         }

//         // This is a simulator: we use a timed wait to emulate fps.
//         // The realtime path uses libcamera callback (event-driven).
//         std::this_thread::sleep_until(nextTick);
//     }
// }

// void SimulatedPublisher::generateFrame_(FrameEvent& fe) {
//     const int w = cfg_.width;
//     const int h = cfg_.height;
//     const int r = std::max(1, cfg_.spot_radius);

//     // Background
//     fe.data.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), cfg_.background);

//     // Determine spot center
//     float cx = w * 0.5f;
//     float cy = h * 0.5f;

//     if (cfg_.moving_spot) {
//         // Smooth circular motion around center
//         phase_ += 0.05f;
//         const float ax = w * 0.25f;
//         const float ay = h * 0.20f;
//         cx = (w * 0.5f) + ax * std::cos(phase_);
//         cy = (h * 0.5f) + ay * std::sin(phase_ * 0.9f);
//     }
//     log_.info("SIM spot cx=" + std::to_string(cx) + " cy=" + std::to_string(cy));
//     const int icx = static_cast<int>(std::round(cx));
//     const int icy = static_cast<int>(std::round(cy));

//     // Draw a filled circle "sun spot"
//     const int r2 = r * r;
//     for (int y = icy - r; y <= icy + r; ++y) {
//         if (y < 0 || y >= h) continue;
//         for (int x = icx - r; x <= icx + r; ++x) {
//             if (x < 0 || x >= w) continue;

//             const int dx = x - icx;
//             const int dy = y - icy;
//             if ((dx * dx + dy * dy) <= r2) {
//                 const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w)
//                                       + static_cast<std::size_t>(x);
//                 fe.data[idx] = cfg_.spot_value;
//             }
//         }
//     }

//     // Add Gaussian noise
//     if (cfg_.noise_std > 0.0f) {
//         for (auto& px : fe.data) {
//             float v = static_cast<float>(px) + noise_(rng_);
//             v = std::clamp(v, 0.0f, 255.0f);
//             px = static_cast<uint8_t>(v);
//         }
//     }
// }

// } // namespace solar










#include "sensors/SimulatedPublisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace solar {

SimulatedPublisher::SimulatedPublisher(Logger& log, Config cfg)
    : log_(log),
      cfg_(cfg),
      rng_(std::random_device{}()) {

    // Only construct the distribution if sigma is valid
    if (cfg_.noise_std > 0.0f) {
        noise_.emplace(0.0f, cfg_.noise_std);
    } else {
        // Noise disabled
        noise_.reset();
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
    if (running_) return true;

    if (cfg_.width <= 0 || cfg_.height <= 0 || cfg_.fps <= 0) {
        log_.error("SimulatedPublisher: invalid config (width/height/fps)");
        return false;
    }

    running_ = true;

    try {
        thread_ = std::thread(&SimulatedPublisher::run_, this);
    } catch (...) {
        running_ = false;
        log_.error("SimulatedPublisher: failed to start thread");
        return false;
    }

    log_.info("SimulatedPublisher started");
    return true;
}

void SimulatedPublisher::stop() {
    if (!running_) return;

    running_ = false;

    if (thread_.joinable()) {
        thread_.join();
    }

    log_.info("SimulatedPublisher stopped");
}

bool SimulatedPublisher::isRunning() const noexcept {
    return running_.load();
}

SimulatedPublisher::Config SimulatedPublisher::config() const {
    return cfg_;
}

void SimulatedPublisher::run_() {
    using clock = std::chrono::steady_clock;

    const auto period =
        std::chrono::microseconds(static_cast<int>(1000000.0 / cfg_.fps));

    auto nextTick = clock::now();

    while (running_) {
        nextTick += period;

        FrameEvent fe;
        fe.frame_id = ++frameId_;
        fe.t_capture = clock::now();
        fe.width = cfg_.width;
        fe.height = cfg_.height;

        fe.data.clear();
        fe.data.reserve(static_cast<std::size_t>(cfg_.width) *
                        static_cast<std::size_t>(cfg_.height));

        generateFrame_(fe);

        FrameCallback cbCopy;
        {
            std::lock_guard<std::mutex> lock(cbMutex_);
            cbCopy = frameCb_;
        }

        if (cbCopy) {
            cbCopy(fe);
        }

        // This is a simulator: emulate FPS with a timed wait.
        std::this_thread::sleep_until(nextTick);
    }
}

void SimulatedPublisher::generateFrame_(FrameEvent& fe) {
    const int w = cfg_.width;
    const int h = cfg_.height;
    const int r = std::max(1, cfg_.spot_radius);

    // Background
    fe.data.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h),
                   cfg_.background);

    // Determine spot center
    float cx = w * 0.5f;
    float cy = h * 0.5f;

    if (cfg_.moving_spot) {
        // Smooth circular motion around center
        phase_ += 0.05f;
        const float ax = w * 0.25f;
        const float ay = h * 0.20f;
        cx = (w * 0.5f) + ax * std::cos(phase_);
        cy = (h * 0.5f) + ay * std::sin(phase_ * 0.9f);
    }

    // Debug log (optional)
    log_.info("SIM spot cx=" + std::to_string(cx) + " cy=" + std::to_string(cy));

    const int icx = static_cast<int>(std::round(cx));
    const int icy = static_cast<int>(std::round(cy));

    // Draw a filled circle "sun spot"
    const int r2 = r * r;
    for (int y = icy - r; y <= icy + r; ++y) {
        if (y < 0 || y >= h) continue;

        for (int x = icx - r; x <= icx + r; ++x) {
            if (x < 0 || x >= w) continue;

            const int dx = x - icx;
            const int dy = y - icy;

            if ((dx * dx + dy * dy) <= r2) {
                const std::size_t idx =
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                    static_cast<std::size_t>(x);

                fe.data[idx] = cfg_.spot_value;
            }
        }
    }

    // Add Gaussian noise (only if enabled and distribution exists)
    if (noise_.has_value()) {
        for (auto& px : fe.data) {
            float v = static_cast<float>(px) + (*noise_)(rng_);
            v = std::clamp(v, 0.0f, 255.0f);
            px = static_cast<uint8_t>(v);
        }
    }
}

} // namespace solar