#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <random>
#include <thread>

#include "common/Logger.hpp"
#include "sensors/ICamera.hpp"

#if defined(__linux__)
  #include <sys/eventfd.h>
#elif defined(_WIN32)
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #include <windows.h>
#endif

namespace solar {

/**
 * @brief Simulation backend implementing ICamera.
 *
 * Generates synthetic frames for development and testing:
 * - Optional moving bright spot
 * - Optional additive Gaussian noise
 *
 * Provides an event-driven interface via ICamera::registerFrameCallback().
 * Timing is handled by the platform-specific mechanism:
 * - Linux: timerfd + eventfd (implementation in .cpp)
 * - Windows: waitable timer + stop event (implementation in .cpp)
 */
class SimulatedPublisher final : public ICamera {
public:
    /// @brief Configuration for synthetic frame generation.
    struct Config {
        /// @brief Output frame width (pixels).
        int width{640};

        /// @brief Output frame height (pixels).
        int height{480};

        /// @brief Target frames per second.
        int fps{30};

        /// @brief If true, move the bright spot over time.
        bool moving_spot{true};

        /// @brief Standard deviation of Gaussian noise (0 disables noise).
        float noise_std{5.0f};

        /// @brief Background pixel value (0..255).
        uint8_t background{20};

        /// @brief Bright spot pixel value (0..255).
        uint8_t spot_value{240};

        /// @brief Bright spot radius (pixels).
        int spot_radius{12};
    };

    /// @brief Construct simulated camera with logger and configuration.
    SimulatedPublisher(Logger& log, Config cfg);

    /// @brief Destructor stops acquisition and joins worker thread.
    ~SimulatedPublisher() override;

    SimulatedPublisher(const SimulatedPublisher&) = delete;
    SimulatedPublisher& operator=(const SimulatedPublisher&) = delete;

    /// @brief Register callback for receiving FrameEvent updates.
    void registerFrameCallback(FrameCallback cb) override;

    /// @brief Start frame generation (idempotent).
    /// @return true if running after the call.
    bool start() override;

    /// @brief Stop frame generation (idempotent).
    void stop() override;

    /// @brief Check whether generation is currently running.
    bool isRunning() const noexcept override;

private:
    /// @brief Worker loop implementing periodic frame generation.
    void run_();

    /// @brief Generate one synthetic frame.
    void generateFrame_(FrameEvent& fe);

    Logger& log_;
    Config cfg_;

    std::atomic<bool> running_{false};
    std::thread worker_;

    /// @brief Mutex protecting callback registration and invocation.
    std::mutex cbMutex_;

    /// @brief Registered frame callback (may be empty).
    FrameCallback frameCb_{};

    /// @brief Monotonic frame identifier counter.
    uint64_t frameId_{0};

    /// @brief Phase accumulator used for moving-spot motion.
    float phase_{0.0f};

    /// @brief Random generator for noise synthesis.
    std::mt19937 rng_;

    /// @brief Optional Gaussian noise distribution (present when noise_std > 0).
    std::optional<std::normal_distribution<float>> noise_;

#if defined(__linux__)
    /// @brief Linux timer file descriptor (timerfd).
    int timerFd_{-1};

    /// @brief Linux stop signal descriptor (eventfd).
    int stopFd_{-1};
#elif defined(_WIN32)
    /// @brief Windows waitable timer for periodic ticks.
    HANDLE timer_{nullptr};

    /// @brief Windows event used to stop immediately.
    HANDLE stopEvent_{nullptr};
#endif
};

} // namespace solar