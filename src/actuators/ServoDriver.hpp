#pragma once

/**
 * @file ServoDriver.hpp
 * @brief Servo output stage using PCA9685 or explicit log-only fallback.
 *
 * Responsibilities:
 * - own the final degree -> pulse mapping
 * - own PCA9685 hardware access policy
 * - apply safe actuator commands already conditioned upstream
 * - support explicit startup/stop parking
 *
 * Non-responsibilities:
 * - control logic
 * - kinematics
 * - realtime scheduling
 * - queueing
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <array>
#include <cstdint>
#include <memory>

namespace solar {

class PCA9685;

/**
 * @brief Final servo output driver.
 */
class ServoDriver {
public:
    /**
     * @brief Startup policy.
     */
    enum class StartupPolicy : std::uint8_t {
        LogOnly,        ///< Never require hardware; just log commands.
        PreferHardware, ///< Use hardware if available, otherwise log-only.
        RequireHardware ///< Fail startup if hardware cannot be initialised.
    };

    /**
     * @brief Per-servo calibration/mapping parameters.
     */
    struct ChannelConfig {
        int channel{0};               ///< PCA9685 output channel.
        float min_pulse_us{500.0F};   ///< Pulse width for min_deg.
        float max_pulse_us{2500.0F};  ///< Pulse width for max_deg.
        float min_deg{0.0F};          ///< Minimum servo degree.
        float max_deg{180.0F};        ///< Maximum servo degree.
        float neutral_deg{90.0F};     ///< Neutral/park degree.
        bool invert{false};           ///< Invert degree mapping around [min_deg,max_deg].
    };

    /**
     * @brief Runtime configuration.
     */
    struct Config {
        StartupPolicy startup_policy{StartupPolicy::LogOnly}; ///< Hardware startup policy.
        int i2c_bus{1};                                      ///< Linux I2C bus for PCA9685.
        std::uint8_t pca9685_addr{0x40U};                    ///< PCA9685 I2C address.
        float pwm_hz{50.0F};                                 ///< Servo PWM frequency.
        bool park_on_start{true};                            ///< Park once on start().
        bool park_on_stop{true};                             ///< Park once on stop().
        int log_every_n{10};                                 ///< Log every N apply() calls in log-only mode.
        std::array<ChannelConfig, 3> ch{};                   ///< Three-servo channel mapping.
    };

    /**
     * @brief Construct the servo driver.
     *
     * @param log Shared logger.
     * @param cfg Servo driver configuration.
     */
    ServoDriver(Logger& log, Config cfg);

    /**
     * @brief Destructor stops the driver if still running.
     */
    ~ServoDriver();

    ServoDriver(const ServoDriver&) = delete;
    ServoDriver& operator=(const ServoDriver&) = delete;

    /**
     * @brief Start the servo driver according to startup policy.
     *
     * @return True if startup succeeds according to policy.
     */
    bool start();

    /**
     * @brief Stop the servo driver.
     *
     * If park_on_stop is enabled and hardware/log mode is active, one final park
     * command is applied before shutdown.
     */
    void stop();

    /**
     * @brief Apply one actuator command.
     *
     * Input targets are interpreted as servo degrees.
     *
     * @param cmd Safe actuator command from ActuatorManager.
     */
    void apply(const ActuatorCommand& cmd);

    /**
     * @brief Return whether the driver is running.
     *
     * @return True if running.
     */
    bool isRunning() const noexcept;

    /**
     * @brief Return whether hardware mode is active.
     *
     * @return True if PCA9685 hardware is active.
     */
    bool isHardwareMode() const noexcept;

    /**
     * @brief Inject a PCA9685 instance for tests.
     *
     * If injected before start(), startup uses this instance instead of creating
     * one internally.
     *
     * @param pca Owned PCA9685 implementation.
     */
    void injectPCA9685(std::unique_ptr<PCA9685> pca);

    /**
     * @brief Return current configuration.
     *
     * @return Current config.
     */
    const Config& config() const noexcept;

private:
    float clampDeg_(float deg, const ChannelConfig& ch) const;
    float maybeInvertDeg_(float deg, const ChannelConfig& ch) const;
    float degToPulseUs_(float deg, const ChannelConfig& ch) const;
    void applyPark_();

private:
    Logger& log_;
    Config cfg_{};

    bool running_{false};
    bool hardware_mode_{false};
    std::uint64_t apply_count_{0U};

    std::unique_ptr<PCA9685> pca_;
};

} // namespace solar