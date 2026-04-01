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

#include <cstdint>

namespace solar::app {

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

    // App-level behaviour knobs
    std::uint32_t tick_hz{30};
    bool simulated_moving_spot{true};
};

/**
 * @brief the project's default configuration.
 */
AppConfig defaultConfig();

} // namespace solar::app
