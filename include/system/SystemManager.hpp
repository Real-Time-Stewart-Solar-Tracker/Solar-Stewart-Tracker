#pragma once

#include <memory>

#include "actuators/ActuatorManager.hpp"
#include "actuators/ServoDriver.hpp"
#include "common/Logger.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "sensors/ICamera.hpp"
#include "vision/SunTracker.hpp"

namespace solar {

/**
 * SystemManager
 *
 * Owns all modules and wires the event-driven pipeline:
 * ICamera -> SunTracker -> Controller -> Kinematics3RRS -> ActuatorManager -> ServoDriver
 *
 * Responsibilities:
 * - Dependency injection (ICamera implementation chosen outside)
 * - Lifecycle management: start/stop everything safely
 * - No business logic here (keeps SOLID clean)
 */
class SystemManager {
public:
    struct Config {
        // Allow passing configs down if needed later (kept minimal here)
    };

    SystemManager(Logger& log,
                  std::unique_ptr<ICamera> camera,
                  SunTracker::Config trackerCfg,
                  Controller::Config controllerCfg,
                  Kinematics3RRS::Config kinCfg,
                  ActuatorManager::Config actCfg,
                  ServoDriver::Config drvCfg);

    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    ~SystemManager();

    bool start();
    void stop();

private:
    Logger& log_;

    std::unique_ptr<ICamera> camera_;

    SunTracker tracker_;
    Controller controller_;
    Kinematics3RRS kinematics_;
    ActuatorManager actuatorMgr_;
    ServoDriver driver_;

    bool running_{false};
};

} // namespace solar