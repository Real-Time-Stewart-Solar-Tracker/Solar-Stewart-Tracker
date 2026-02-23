#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar::actuators { class PCA9685; }

namespace solar {

/**
 * @file ServoDriver.hpp
 * @brief High-level hardware output layer for Stewart platform servos.
 *
 * Converts ActuatorCommand targets into calibrated servo PWM pulses.
 * Uses a PCA9685 driver when available (Raspberry Pi), otherwise falls back to log-only.
 *
 * @note Intended to be called from the actuator thread (no internal locking).
 */
class ServoDriver {
public:
    /**
     * @brief Per-channel servo configuration (calibration + safety).
     */
    struct ChannelConfig {
        uint8_t channel{0};          ///< PCA9685 channel index [0..15]
        float   min_pulse_us{1100};  ///< Safety minimum pulse width (start conservative)
        float   max_pulse_us{1900};  ///< Safety maximum pulse width (start conservative)
        bool    invert{false};       ///< Invert mapping if mechanics are mirrored
        float   neutral_us{1500};    ///< Neutral/park pulse width
    };

    /**
     * @brief ServoDriver configuration.
     *
     * Targets are interpreted as normalized values in [min_norm, max_norm] and
     * mapped linearly to each channel’s pulse range with safety clamps.
     */
    struct Config {
        std::array<ChannelConfig, 3> ch{}; ///< 3 servos for the 3RRS platform

        // Normalized input range expected from higher-level control/kinematics
        float min_norm{-1.0f};
        float max_norm{ 1.0f};

        // PCA9685 / I2C settings (used on Linux when hardware is available)
        std::string i2c_dev{"/dev/i2c-1"};
        uint8_t     pca9685_addr{0x40};
        float       pwm_hz{50.0f};

        // Behaviour
        bool park_on_start{true};  ///< Move to neutral at start (recommended)
        bool park_on_stop{true};   ///< Move to neutral at stop  (recommended)

        // Optional logging
        std::uint32_t log_every_n{0}; ///< Log every N apply() calls (0 disables)
    };

    /// @brief Construct with logger and configuration (hardware created internally on Linux).
    ServoDriver(Logger& log, Config cfg);

    /**
     * @brief Construct with injected PCA9685 instance (unit tests / custom wiring).
     * @note If injected is nullptr, driver operates in log-only mode.
     */
    ServoDriver(Logger& log, Config cfg, std::unique_ptr<solar::actuators::PCA9685> injected);

    ServoDriver(const ServoDriver&)            = delete;
    ServoDriver& operator=(const ServoDriver&) = delete;

    /// @brief Destructor stops the driver if running.
    ~ServoDriver();

    /// @brief Start driver (idempotent). Creates/initialises hardware where supported.
    /// @return true if running after the call.
    bool start();

    /// @brief Stop driver (idempotent). Optionally parks outputs to neutral.
    void stop();

    /// @brief Apply actuator command (fast, deterministic, no sleep/polling).
    void apply(const ActuatorCommand& cmd);

private:
    float norm_to_pulse_us(float norm, const ChannelConfig& c) const noexcept;
    void  write_pulse_us(uint8_t channel, float pulse_us) noexcept;
    void  park_all() noexcept;

    Logger& log_;
    Config  cfg_;

    std::atomic<bool> running_{false};
    std::atomic<std::uint32_t> applyCount_{0};
    std::atomic<bool> warnedStopped_{false};

    // Null => log-only fallback (still deterministic and testable)
    std::unique_ptr<solar::actuators::PCA9685> pca_;
};

} // namespace solar