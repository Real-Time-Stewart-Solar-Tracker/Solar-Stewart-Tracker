#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * @brief Hardware output layer for driving servos.
 *
 * Converts ActuatorCommand targets into hardware-specific output signals (e.g., PWM).
 * Provides a safe, idempotent start/stop lifecycle. On non-target platforms, this may
 * be implemented as logging-only output.
 */
class ServoDriver {
public:
    /// @brief Configuration for output clamping and optional log rate limiting.
    struct Config {
        /// @brief Minimum allowed output per channel (safety clamp).
        std::array<float, 3> min_out{-1.0f, -1.0f, -1.0f};

        /// @brief Maximum allowed output per channel (safety clamp).
        std::array<float, 3> max_out{ 1.0f,  1.0f,  1.0f};

        /// @brief Log once every N apply() calls (0 disables apply() logging).
        std::uint32_t log_every_n{30};
    };

    /// @brief Construct with logger and configuration.
    ServoDriver(Logger& log, Config cfg);

    ServoDriver(const ServoDriver&) = delete;
    ServoDriver& operator=(const ServoDriver&) = delete;

    /// @brief Destructor stops the driver if it is running.
    ~ServoDriver();

    /// @brief Start the driver (safe to call multiple times).
    /// @return true if the driver is running after the call.
    bool start();

    /// @brief Stop the driver (safe to call multiple times).
    void stop();

    /// @brief Apply an actuator command to the hardware output layer.
    void apply(const ActuatorCommand& cmd);

private:
    Logger& log_;
    Config cfg_;

    /// @brief True when the driver is started and allowed to output signals.
    std::atomic<bool> running_{false};

    /// @brief Counter used to rate-limit apply() logging.
    std::atomic<std::uint32_t> applyCount_{0};

    /// @brief Ensures we warn at most once if apply() is called while stopped.
    std::atomic<bool> warnedStopped_{false};
};

} // namespace solar