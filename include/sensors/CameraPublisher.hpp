#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * CameraPublisher
 *
 * Produces FrameEvent objects and publishes them via a callback.
 *
 * Design requirements (A1):
 * - Event-driven architecture (frames delivered to downstream via callback)
 * - Clean start/stop lifecycle, no detached threads
 * - Thread-safe callback registration
 *
 * Implementation note:
 * - The Raspberry Pi libcamera integration will live in CameraPublisher.cpp
 * - The class interface remains stable regardless of camera backend
 */
class CameraPublisher {
public:
    using FrameCallback = std::function<void(const FrameEvent&)>;

    struct Config {
        int width{640};
        int height{480};
        int fps{30};
        std::string camera_id{}; // optional: allow selecting camera
    };

    CameraPublisher(Logger& log, Config cfg);

    CameraPublisher(const CameraPublisher&) = delete;
    CameraPublisher& operator=(const CameraPublisher&) = delete;

    ~CameraPublisher();

    // Register consumer for frame events. Safe to call before start().
    void registerFrameCallback(FrameCallback cb);

    // Start/stop camera acquisition.
    // start() is idempotent; stop() is idempotent.
    bool start();
    void stop();

    bool isRunning() const noexcept;

    Config config() const;

private:
    // Implemented in .cpp.
    // Camera backend thread or callback glue lives here.
    void run_();

    Logger& log_;
    Config cfg_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    // Protect callback assignment/usage.
    mutable std::mutex cbMutex_;
    FrameCallback frameCb_{};

    uint64_t frameId_{0};
};

} // namespace solar