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
 * @brief Publishes camera frames as FrameEvent objects via callback.
 *
 * Provides an event-driven interface with a safe start/stop lifecycle and
 * thread-safe callback registration. The backend implementation may differ
 * between development and target platforms, while the interface remains stable.
 */
class CameraPublisher {
public:
    /// @brief Callback type used to deliver captured frames.
    using FrameCallback = std::function<void(const FrameEvent&)>;

    /// @brief Camera acquisition configuration.
    struct Config {
        /// @brief Output frame width (pixels).
        int width{640};

        /// @brief Output frame height (pixels).
        int height{480};

        /// @brief Target frames per second.
        int fps{30};

        /// @brief Optional camera identifier (backend-specific).
        std::string camera_id{};
    };

    /// @brief Construct publisher with logger and configuration.
    CameraPublisher(Logger& log, Config cfg);

    CameraPublisher(const CameraPublisher&) = delete;
    CameraPublisher& operator=(const CameraPublisher&) = delete;

    /// @brief Destructor stops acquisition and joins internal thread if running.
    ~CameraPublisher();

    /// @brief Register callback to receive FrameEvent updates (safe before start()).
    void registerFrameCallback(FrameCallback cb);

    /// @brief Start camera acquisition (idempotent).
    /// @return true if running after the call.
    bool start();

    /// @brief Stop camera acquisition (idempotent).
    void stop();

    /// @brief Check whether acquisition is currently running.
    bool isRunning() const noexcept;

    /// @brief Get current configuration.
    Config config() const;

private:
    /// @brief Backend acquisition loop (implemented in .cpp).
    void run_();

    Logger& log_;
    Config cfg_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    /// @brief Mutex protecting callback assignment and invocation.
    mutable std::mutex cbMutex_;

    /// @brief Registered frame callback (may be empty).
    FrameCallback frameCb_{};

    /// @brief Monotonic frame identifier counter.
    uint64_t frameId_{0};
};

} // namespace solar