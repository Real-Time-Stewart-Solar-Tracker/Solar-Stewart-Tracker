#pragma once

/**
 * @file SimulatedPublisher.hpp
 * @brief Event-driven simulated camera publisher.
 */

#include "common/Logger.hpp"
#include "sensors/ICamera.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace solar {

/**
 * @brief Event-driven synthetic camera publisher.
 *
 * This backend generates frames on its own worker thread and pushes them out via
 * the normal camera callback interface. It exists for:
 * - non-hardware builds
 * - CI
 * - deterministic demos and smoke tests
 *
 * Frame timing is driven by a @c timerfd (CLOCK_MONOTONIC), which keeps the
 * worker thread sleeping in @c poll() between frames rather than consuming CPU
 * in a sleep-based loop. A companion @c eventfd provides a low-latency wake-up
 * path for clean shutdown.
 */
class SimulatedPublisher final : public ICamera {
public:
    /**
     * @brief Runtime configuration for the simulated camera.
     */
    struct Config {
        int width{640};               ///< Frame width in pixels.
        int height{480};              ///< Frame height in pixels.
        int fps{30};                  ///< Published frame rate.
        bool moving_spot{true};       ///< Move the synthetic sun spot over time.
        float noise_std{5.0F};        ///< Optional additive noise standard deviation.
        std::uint8_t background{20};  ///< Background intensity.
        std::uint8_t spot_value{240}; ///< Synthetic sun-spot intensity.
        int spot_radius{12};          ///< Synthetic sun-spot radius in pixels.
    };

    /**
     * @brief Construct the simulated publisher.
     *
     * Pre-allocates the internal pixel buffer to the configured frame dimensions
     * so the worker loop can reuse it on every frame without heap allocation.
     *
     * @param log Shared logger.
     * @param cfg Simulated camera settings.
     */
    SimulatedPublisher(Logger& log, Config cfg);

    /**
     * @brief Destructor stops the worker thread if still running.
     */
    ~SimulatedPublisher() override;

    SimulatedPublisher(const SimulatedPublisher&) = delete;
    SimulatedPublisher& operator=(const SimulatedPublisher&) = delete;

    /**
     * @brief Register the output frame callback.
     *
     * @param cb Callback receiving generated frames.
     */
    void registerFrameCallback(FrameCallback cb) override;

    /**
     * @brief Start the simulated publisher worker thread.
     *
     * @return True on success.
     */
    bool start() override;

    /**
     * @brief Stop the simulated publisher worker thread.
     */
    void stop() override;

    /**
     * @brief Check whether the publisher is currently running.
     *
     * @return True if the worker thread is running.
     */
    bool isRunning() const override;

private:
    bool openTimingFds_();
    void closeTimingFds_();
    bool armTimer_() const;
    void workerLoop_();
    FrameEvent buildFrame_(std::uint64_t frame_id, float phase) const;
    void drawSpot_(int cx, int cy) const;

private:
    Logger& log_;
    Config cfg_{};

    mutable std::mutex cb_mtx_;
    FrameCallback callback_{};

    std::atomic<bool> running_{false};
    std::thread worker_;

    int timer_fd_{-1}; ///< Blocking periodic timer for synthetic frame pacing.
    int wake_fd_{-1};  ///< Explicit wake-up used to stop the worker promptly.

    /**
     * Pre-allocated pixel buffer reused across every generated frame.
     * Declared @c mutable because @c buildFrame_() is @c const but must
     * modify the buffer as a performance optimisation (equivalent to a cached
     * intermediate).
     */
    mutable std::vector<std::uint8_t> pixel_buf_;
};

} // namespace solar
