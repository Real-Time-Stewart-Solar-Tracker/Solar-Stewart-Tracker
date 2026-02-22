#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "actuators/ActuatorManager.hpp"
#include "actuators/ServoDriver.hpp"
#include "common/LatencyMonitor.hpp"
#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "sensors/ICamera.hpp"
#include "system/TrackerState.hpp"
#include "vision/SunTracker.hpp"

namespace solar {

/**
 * @brief Top-level system orchestrator for the solar tracking pipeline.
 *
 * Connects all subsystems:
 * - Camera acquisition
 * - Vision (SunTracker)
 * - Control (Controller)
 * - Kinematics (3RRS inverse kinematics)
 * - Actuator safety layer
 * - Servo driver
 * - Latency monitoring
 *
 * Implements lifecycle control and system state management.
 */
class SystemManager {
public:
    /**
     * @brief Observer callback for latency reporting.
     *
     * @param frame_id Frame identifier
     * @param cap_to_est_ms Capture → estimate latency (ms)
     * @param est_to_ctrl_ms Estimate → control latency (ms)
     * @param ctrl_to_act_ms Control → actuation latency (ms)
     */
    using LatencyObserver = std::function<void(uint64_t frame_id,
                                               float cap_to_est_ms,
                                               float est_to_ctrl_ms,
                                               float ctrl_to_act_ms)>;

    /**
     * @brief Construct the full system manager with all subsystem configurations.
     */
    SystemManager(Logger& log,
                  std::unique_ptr<ICamera> camera,
                  SunTracker::Config trackerCfg,
                  Controller::Config controllerCfg,
                  Kinematics3RRS::Config kinCfg,
                  ActuatorManager::Config actCfg,
                  ServoDriver::Config drvCfg);

    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    /// @brief Destructor stops the system if running.
    ~SystemManager();

    /// @brief Start the full tracking pipeline.
    bool start();

    /// @brief Stop the full tracking pipeline.
    void stop();

    // ------------------------------------------------------------------
    // State Machine API
    // ------------------------------------------------------------------

    /// @brief Get current tracker state.
    TrackerState state() const;

    /// @brief Enter manual control mode.
    void enterManual();

    /// @brief Exit manual mode and resume automatic tracking.
    void exitManual();

    /// @brief Set manual platform setpoint (used in manual mode).
    void setManualSetpoint(float tilt_rad, float pan_rad);

    /// @brief Update vision threshold safely at runtime.
    void setTrackerThreshold(uint8_t thr);

    // ------------------------------------------------------------------
    // Observers (Event-Driven Hooks for UI / Telemetry)
    // ------------------------------------------------------------------

    /// @brief Register frame observer.
    void registerFrameObserver(ICamera::FrameCallback cb);

    /// @brief Register estimate observer.
    void registerEstimateObserver(SunTracker::EstimateCallback cb);

    /// @brief Register setpoint observer.
    void registerSetpointObserver(Controller::SetpointCallback cb);

    /// @brief Register actuator command observer.
    void registerCommandObserver(Kinematics3RRS::CommandCallback cb);

    /// @brief Register latency observer.
    void registerLatencyObserver(LatencyObserver cb);

private:
    /// @brief Internal helper for state transitions.
    void setState_(TrackerState s);

    /// @brief Apply neutral actuator command once during transitions.
    void applyNeutralOnce_();

    Logger& log_;
    std::unique_ptr<ICamera> camera_;

    SunTracker tracker_;
    Controller controller_;
    Kinematics3RRS kinematics_;
    ActuatorManager actuatorMgr_;
    ServoDriver driver_;
    LatencyMonitor latency_;

    bool running_{false};
    TrackerState state_{TrackerState::IDLE};
    float min_confidence_{0.0f};

    PlatformSetpoint manual_sp_{};
};

} // namespace solar