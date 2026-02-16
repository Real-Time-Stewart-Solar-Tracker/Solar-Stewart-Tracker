#include "system/SystemManager.hpp"
#include "common/Logger.hpp"
#include <utility>
#include "common/LatencyMonitor.hpp"

namespace solar {

SystemManager::SystemManager(Logger& log,
                             std::unique_ptr<ICamera> camera,
                             SunTracker::Config trackerCfg,
                             Controller::Config controllerCfg,
                             Kinematics3RRS::Config kinCfg,
                             ActuatorManager::Config actCfg,
                             ServoDriver::Config drvCfg)
    : log_(log),
      camera_(std::move(camera)),
      tracker_(log_, trackerCfg),
      controller_(log_, controllerCfg),
      kinematics_(log_, kinCfg),
      actuatorMgr_(log_, actCfg),
      driver_(log_, drvCfg),
      latency_(log_) {
    // Wire pipeline callbacks

    // Camera -> SunTracker
    if (camera_) {
        camera_->registerFrameCallback([this](const FrameEvent& fe) {
            latency_.onCapture(fe.frame_id, fe.t_capture);
            tracker_.onFrame(fe);
        });
    }

    // SunTracker -> Controller
    tracker_.registerEstimateCallback([this](const SunEstimate& est) {
        latency_.onEstimate(est.frame_id, est.t_estimate);
        controller_.onEstimate(est);
    });

    // Controller -> Kinematics
    controller_.registerSetpointCallback([this](const PlatformSetpoint& sp) {
        latency_.onControl(sp.frame_id, sp.t_control);
        kinematics_.onSetpoint(sp);
    });

    // Kinematics -> ActuatorManager
    kinematics_.registerCommandCallback([this](const ActuatorCommand& cmd) {
        actuatorMgr_.onCommand(cmd);
    });

    // ActuatorManager -> ServoDriver
    actuatorMgr_.registerSafeCommandCallback([this](const ActuatorCommand& safeCmd) {
        latency_.onActuate(safeCmd.frame_id, safeCmd.t_actuate);
        driver_.apply(safeCmd);
    });
}

SystemManager::~SystemManager() {
    stop();
}

bool SystemManager::start() {
    if (running_) return true;

    if (!camera_) {
        log_.error("SystemManager: camera is null");
        return false;
    }

    if (!driver_.start()) {
        log_.error("SystemManager: ServoDriver start failed");
        return false;
    }

    if (!camera_->start()) {
        log_.error("SystemManager: Camera start failed");
        driver_.stop();
        return false;
    }

    running_ = true;
    log_.info("SystemManager started");
    return true;
}

void SystemManager::stop() {
    if (!running_) return;

    // Stop camera first (stops new frames entering pipeline)
    if (camera_) {
        camera_->stop();
    }

    // Stop driver last (safe final step)
    driver_.stop();

    running_ = false;
    latency_.printSummary();
    log_.info("SystemManager stopped");
}

} // namespace solar