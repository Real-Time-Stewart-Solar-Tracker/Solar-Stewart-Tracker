#pragma once

#include "actuators/ActuatorManager.hpp"
#include "actuators/ServoDriver.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "sensors/ICamera.hpp"
#include "vision/SunTracker.hpp"

#if SOLAR_HAVE_LIBCAMERA
#include "sensors/LibcameraPublisher.hpp"
#endif

#include "sensors/SimulatedPublisher.hpp"

#if SOLAR_HAVE_IMU
#include "sensors/MPU6050Publisher.hpp"
#endif

#include <cstdint>

namespace solar::app {

/// @brief Sensor input source selection.
enum class SensorMode {
    CAMERA, ///< Vision pipeline: camera -> SunTracker -> Controller -> platform
    IMU,    ///< IMU pipeline: MPU6050 angles -> platform directly
};

/**
 * @brief Bundle of all runtime configuration for the application.
 *
 * Keeps numeric constants out of main()/UI glue and in one place.
 */
struct AppConfig {
    SunTracker::Config       tracker{};
    Controller::Config       controller{};
    Kinematics3RRS::Config   kinematics{};
    ActuatorManager::Config  actuator{};
    ServoDriver::Config      servo{};

#if SOLAR_HAVE_LIBCAMERA
    LibcameraPublisher::Config camera{};
#else
    SimulatedPublisher::Config camera{};
#endif

#if SOLAR_HAVE_IMU
    MPU6050Publisher::Config imu{};
#endif

    // App-level behaviour knobs
    std::uint32_t tick_hz{30};
    bool simulated_moving_spot{true};
    SensorMode sensor_mode{SensorMode::CAMERA};
};

/**
 * @brief the project's default configuration.
 */
AppConfig defaultConfig();

} // namespace solar::app
