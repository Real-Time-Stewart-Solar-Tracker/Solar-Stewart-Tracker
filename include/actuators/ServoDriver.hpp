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
 * Converts ActuatorCommand targets (SERVO ANGLES IN DEGREES) into calibrated servo PWM pulses.
 * Uses a PCA9685 driver when available (Raspberry Pi), otherwise falls back to log-only.
 *
 * @note Intended to be called from the actuator thread (no internal locking).
 */
class ServoDriver {
public:
    /**
     * @brief Per-channel servo configuration (calibration + safety).
     *
     * Interpretation:
     * - input target is angle in degrees
     * - angle is clamped to [min_deg, max_deg]
     * - angle is mapped linearly to [min_pulse_us, max_pulse_us]
     * - invert flips the mapping direction (mirrored installation)
     */
    struct ChannelConfig {
        uint8_t channel{0};          ///< PCA9685 channel index [0..15]

        // Electrical calibration (aligned with the reference PCA9685 implementation)
        // Reference uses 500us..2500us mapped over 0..180 degrees.
        float   min_pulse_us{500.f};  ///< Pulse at min_deg
        float   max_pulse_us{2500.f}; ///< Pulse at max_deg

        // Angle domain (what Kinematics outputs)
        float   min_deg{0.f};         ///< Safe minimum servo angle
        float   max_deg{180.f};       ///< Safe maximum servo angle

        // Neutral / park position (in degrees)
        float   neutral_deg{90.f};    ///< Park angle (mapped to pulse via range)

        bool    invert{false};        ///< Invert direction if mechanics are mirrored
    };

    /**
     * @brief ServoDriver configuration.
     *
     * Targets are interpreted as SERVO ANGLES IN DEGREES.
     */
    struct Config {
        std::array<ChannelConfig, 3> ch{}; ///< 3 servos for the 3RRS platform

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

    /**
     * @brief Apply actuator command (fast, deterministic, no sleep/polling).
     *
     * @note cmd.actuator_targets[i] is interpreted as SERVO ANGLE IN DEGREES.
     */
    void apply(const ActuatorCommand& cmd);

private:
    float deg_to_pulse_us(float deg, const ChannelConfig& c) const noexcept;
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