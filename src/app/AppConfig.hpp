#pragma once

/**
 * @file AppConfig.hpp
 * @brief Top-level runtime configuration types for the solar tracker application.
 *
 * This header collects the strongly typed configuration used to assemble the
 * final runtime graph. The goal is to keep startup policy and hardware defaults
 * explicit instead of scattering them throughout entry points.
 *
 * Hardware-level register addresses and initialisation constants belong inside
 * their respective device driver classes, not here. Application-level config
 * exposes only the settings that a system integrator might reasonably change:
 * bus numbers, GPIO lines, data rates, and identity expectations.
 */

#include "actuators/ActuatorManager.hpp"
#include "actuators/ServoDriver.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "control/ManualInputMapper.hpp"
#include "vision/SunTracker.hpp"

#include <cstddef>
#include <cstdint>

namespace solar::app {

/**
 * @brief Camera backend selected for frame acquisition.
 */
enum class CameraBackend : std::uint8_t {
    Simulated, ///< Use the internal simulated camera publisher.
    Libcamera  ///< Use the Raspberry Pi libcamera backend.
};

/**
 * @brief Initial runtime mode selected at application startup.
 */
enum class StartupMode : std::uint8_t {
    Auto,   ///< Start in automatic tracking mode.
    Manual  ///< Start in manual mode.
};

/**
 * @brief Manual-input backend selection.
 */
enum class ManualInputBackend : std::uint8_t {
    None,   ///< Disable hardware manual input.
    ADS1115 ///< Use the ADS1115 potentiometer backend.
};

/**
 * @brief IMU backend selection.
 */
enum class ImuBackend : std::uint8_t {
    None,        ///< Disable IMU input.
    Mpu6050Gpio  ///< Use the GPIO-driven MPU-6050/ICM-20600 backend.
};

/**
 * @brief IMU feedback operating mode.
 */
enum class ImuFeedbackMode : std::uint8_t {
    Disabled, ///< IMU correction disabled.
    Shadow,   ///< IMU path active for observation only; correction not applied.
    Live      ///< IMU path actively corrects control output.
};

/**
 * @brief Configuration for the simulated camera backend.
 */
struct SimulatedCameraConfig {
    int width{640};                ///< Frame width in pixels.
    int height{480};               ///< Frame height in pixels.
    int fps{30};                   ///< Published frame rate.
    bool moving_spot{true};        ///< Move the synthetic spot over time.
    float noise_std{5.0F};         ///< Additive noise standard deviation.
    std::uint8_t background{20U};  ///< Background pixel intensity.
    std::uint8_t spot_value{240U}; ///< Synthetic spot intensity.
    int spot_radius{12};           ///< Synthetic spot radius in pixels.
};

/**
 * @brief Configuration for the libcamera backend.
 */
struct LibcameraConfig {
    int width{640};  ///< Requested frame width in pixels.
    int height{480}; ///< Requested frame height in pixels.
    int fps{30};     ///< Requested frame rate.
};

/**
 * @brief Configuration for the ADS1115 manual-input backend.
 */
struct ADS1115ManualConfig {
    bool enabled{false};                ///< Enable ADS1115 manual input.
    int i2c_bus{1};                     ///< Linux I2C bus number.
    std::uint8_t i2c_address{0x48U};    ///< ADS1115 I2C address.

    int gpio_chip_index{0};             ///< GPIO chip index for ALERT/RDY.
    unsigned int alert_rdy_gpio{17U};   ///< ALERT/RDY GPIO line number.

    std::uint8_t tilt_channel{0U};      ///< ADS1115 channel for tilt potentiometer.
    std::uint8_t pan_channel{1U};       ///< ADS1115 channel for pan potentiometer.
    std::uint8_t spare_channel{0xFFU};  ///< Optional spare channel; 0xFF disables it.

    float full_scale_voltage{4.096F};   ///< ADS1115 full-scale input range in volts.
    unsigned int sample_rate_hz{128U};  ///< ADS1115 conversion rate in hertz.
};

/**
 * @brief Application-level configuration for the MPU-6050/ICM-20600 IMU backend.
 *
 * This struct exposes only the settings a system integrator may need to change:
 * bus assignment, GPIO routing, device identity, and startup behaviour.
 * Register-level initialisation constants are internal to @c Mpu6050Publisher
 * and are not part of the application configuration surface.
 */
struct AppMpu6050Config {
    bool enabled{false};                ///< Enable the MPU-6050/ICM-20600 backend.
    int i2c_bus{1};                     ///< Linux I2C bus number.
    std::uint8_t i2c_address{0x68U};    ///< Device I2C address.

    int gpio_chip_index{0};             ///< GPIO chip index for the data-ready line.
    unsigned int data_ready_gpio{27U};  ///< Data-ready GPIO line number.

    /**
     * @brief Expected WHO_AM_I register value.
     *
     * The ICM-20600 (pin-compatible MPU-6050 successor) returns 0x70,
     * whereas the original MPU-6050 returns 0x68. The default here matches
     * the ICM-20600 fitted to the physical rig.
     */
    std::uint8_t who_am_i_expected{0x70U};

    /**
     * @brief Number of samples discarded after startup to allow the device
     *        to stabilise.
     */
    std::size_t startup_discard_samples{8U};
};

/**
 * @brief Configuration for optional IMU feedback correction.
 */
struct ImuFeedbackConfig {
    float gain{0.12F};                       ///< Proportional gain applied to measured tilt error.
    float deadband_rad{0.01745329252F};      ///< Symmetric deadband in radians (1 degree).
    float max_correction_rad{0.01745329252F};///< Maximum correction magnitude in radians (1 degree).
};

/**
 * @brief Top-level application configuration.
 *
 * This structure groups all runtime configuration objects used when building the
 * final application. It is intentionally explicit so that the composition root
 * can remain small and deterministic.
 */
struct AppConfig {
    CameraBackend camera_backend{CameraBackend::Simulated}; ///< Selected camera backend.
    StartupMode startup_mode{StartupMode::Auto};            ///< Initial runtime mode.
    ManualInputBackend manual_input_backend{ManualInputBackend::None}; ///< Manual-input backend.
    ImuBackend imu_backend{ImuBackend::None};               ///< IMU backend.
    ImuFeedbackMode imu_feedback_mode{ImuFeedbackMode::Disabled}; ///< IMU feedback mode.
    ImuFeedbackConfig imu_feedback{};                       ///< IMU feedback parameters.

    SunTracker::Config tracker{};               ///< Vision-stage configuration.
    Controller::Config controller{};            ///< Controller-stage configuration.
    Kinematics3RRS::Config kinematics{};        ///< Kinematics-stage configuration.
    ActuatorManager::Config actuator{};         ///< Actuator-manager configuration.
    ServoDriver::Config servo{};                ///< Servo-driver configuration.

    SimulatedCameraConfig simulated_camera{};   ///< Simulated camera backend configuration.
    LibcameraConfig libcamera{};                ///< Libcamera backend configuration.
    ADS1115ManualConfig ads1115_manual{};        ///< ADS1115 manual-input configuration.
    solar::ManualInputMapperConfig manual_mapping{}; ///< Potentiometer-to-command mapping config.
    AppMpu6050Config mpu6050{};                  ///< MPU-6050/ICM-20600 backend configuration.

    std::uint32_t tick_hz{30U};                 ///< Headless event-loop tick rate in hertz.
};

/**
 * @brief Construct the repository baseline runtime configuration.
 *
 * @return A fully-populated default configuration.
 */
AppConfig defaultConfig();

/**
 * @brief Construct the default configuration for the Qt GUI runtime.
 *
 * The Qt runtime shares the same baseline as the headless runtime. The only
 * policy difference is that the GUI default enables live IMU correction so
 * the user sees the real closed-loop behaviour.
 *
 * @return A fully-populated Qt runtime configuration.
 */
AppConfig defaultQtConfig();

} // namespace solar::app
