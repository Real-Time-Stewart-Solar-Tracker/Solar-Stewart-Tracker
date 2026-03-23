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
 * Converts @ref ActuatorCommand targets (servo angles in degrees) into
 * calibrated PCA9685 PWM pulse widths.
 *
 * Runtime startup policy is explicit:
 * - PreferHardware: try PCA9685, fall back to log-only if unavailable
 * - RequireHardware: fail fast if PCA9685 hardware cannot be started
 * - LogOnly: never attempt hardware access; run in deterministic log-only mode
 *
 * This avoids the ambiguous behaviour where start() appears successful even
 * though real hardware was requested but not actually achieved.
 *
 * @note Intended to be called from a single actuator thread (no internal locking).
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
        uint8_t channel{0};            ///< PCA9685 channel index [0..15]

        float min_pulse_us{500.f};     ///< Pulse width at min_deg
        float max_pulse_us{2500.f};    ///< Pulse width at max_deg

        float min_deg{0.f};            ///< Safe minimum servo angle
        float max_deg{180.f};          ///< Safe maximum servo angle

        float neutral_deg{41.f};       ///< Rig-calibrated park angle in degrees

        bool invert{false};            ///< Mirror mapping direction
    };

    /**
     * @brief Startup policy for ServoDriver.
     */
    enum class StartupPolicy : uint8_t {
        PreferHardware,   ///< Try hardware first, otherwise fall back to log-only
        RequireHardware,  ///< Fail start() if real hardware cannot be started
        LogOnly           ///< Never attempt hardware access
    };

    /**
     * @brief Actual runtime mode after start().
     */
    enum class RuntimeMode : uint8_t {
        Stopped,   ///< Driver is not running
        Hardware,  ///< Running with real PCA9685 output
        LogOnly    ///< Running without real hardware output
    };

    /**
     * @brief ServoDriver configuration.
     *
     * Targets are interpreted as servo angles in degrees.
     */
    struct Config {
        std::array<ChannelConfig, 3> ch{}; ///< Three servos for the 3RRS platform

        // PCA9685 / I2C settings
        std::string i2c_dev{"/dev/i2c-1"};
        uint8_t     pca9685_addr{0x40};
        float       pwm_hz{50.0f};

        // Startup/runtime behaviour
        StartupPolicy startup_policy{StartupPolicy::PreferHardware};

        // Parking behaviour
        bool park_on_start{true};  ///< Move to neutral after a successful start
        bool park_on_stop{true};   ///< Move to neutral on stop

        // Optional logging
        std::uint32_t log_every_n{0}; ///< Log every N apply() calls (0 disables)
    };

    /**
     * @brief Construct with logger and configuration.
     *
     * On Linux, hardware may be created internally during start() depending on
     * @ref Config::startup_policy.
     */
    ServoDriver(Logger& log, Config cfg);

    /**
     * @brief Construct with an injected PCA9685 instance.
     *
     * Useful for tests or explicit hardware wiring.
     *
     * Behaviour:
     * - injected != nullptr: hardware path is available immediately
     * - injected == nullptr: start() follows @ref Config::startup_policy
     */
    ServoDriver(Logger& log, Config cfg, std::unique_ptr<solar::actuators::PCA9685> injected);

    ServoDriver(const ServoDriver&)            = delete;
    ServoDriver& operator=(const ServoDriver&) = delete;

    /**
     * @brief Destructor stops the driver if running.
     */
    ~ServoDriver();

    /**
     * @brief Start the driver.
     *
     * Semantics:
     * - returns true only if the requested startup policy is satisfied
     * - does not pretend success when @ref StartupPolicy::RequireHardware was
     *   requested but real hardware could not be started
     * - sets a concrete runtime mode that can be queried with @ref runtimeMode()
     *
     * @return true if the driver is running after the call, false otherwise
     */
    bool start();

    /**
     * @brief Stop the driver.
     *
     * Safe to call multiple times.
     * Optionally parks outputs to neutral if currently running.
     */
    void stop();

    /**
     * @brief Apply actuator command.
     *
     * @note cmd.actuator_targets[i] is interpreted as servo angle in degrees.
     * If running in @ref RuntimeMode::LogOnly, the mapping still occurs, but no
     * real hardware write is performed.
     */
    void apply(const ActuatorCommand& cmd);

    /**
     * @brief Return whether the driver is currently running.
     */
    [[nodiscard]] bool isRunning() const noexcept { return running_.load(); }

    /**
     * @brief Return whether the driver is currently controlling real hardware.
     */
    [[nodiscard]] bool hardwareActive() const noexcept {
        return runtimeMode_.load() == RuntimeMode::Hardware;
    }

    /**
     * @brief Return the current runtime mode.
     */
    [[nodiscard]] RuntimeMode runtimeMode() const noexcept {
        return runtimeMode_.load();
    }

private:
    [[nodiscard]] float deg_to_pulse_us(float deg, const ChannelConfig& c) const noexcept;
    void write_pulse_us(uint8_t channel, float pulse_us) noexcept;
    void park_all() noexcept;

    [[nodiscard]] bool tryCreateAndInitHardware_();
    void setRuntimeMode_(RuntimeMode mode) noexcept;
    [[nodiscard]] static const char* runtimeModeName_(RuntimeMode mode) noexcept;
    [[nodiscard]] static const char* startupPolicyName_(StartupPolicy policy) noexcept;

private:
    Logger& log_;
    Config  cfg_;

    std::atomic<bool> running_{false};
    std::atomic<RuntimeMode> runtimeMode_{RuntimeMode::Stopped};
    std::atomic<std::uint32_t> applyCount_{0};
    std::atomic<bool> warnedStopped_{false};

    // Null when operating in log-only mode
    std::unique_ptr<solar::actuators::PCA9685> pca_;
};

} // namespace solar