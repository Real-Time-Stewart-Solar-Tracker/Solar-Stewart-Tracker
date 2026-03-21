#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "actuators/ActuatorManager.hpp"
#include "actuators/ServoDriver.hpp"
#include "common/LatencyMonitor.hpp"
#include "common/Logger.hpp"
#include "common/ThreadSafeQueue.hpp"
#include "common/Types.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "sensors/ICamera.hpp"
#include "system/TrackerState.hpp"
#include "vision/SunTracker.hpp"

namespace solar {

/**
 * @brief Top-level orchestrator of the solar tracking system.
 *
 * Coordinates the complete event-driven processing pipeline:
 *
 * - Camera backend emits FrameEvent via callback
 * - SystemManager pushes frames into a bounded queue
 * - Control thread blocks on frame queue and executes:
 *      SunTracker -> Controller -> Kinematics3RRS
 * - Actuator thread blocks on command queue and executes:
 *      ActuatorManager -> ServoDriver
 *
 * The design avoids polling loops and sleep-based timing in the realtime path.
 */
class SystemManager {
public:
    using LatencyObserver = std::function<void(uint64_t frame_id,
                                               float cap_to_est_ms,
                                               float est_to_ctrl_ms,
                                               float ctrl_to_act_ms)>;

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

    TrackerState state() const;

    void enterManual();
    void exitManual();
    void setManualSetpoint(float tilt_rad, float pan_rad);
    void setTrackerThreshold(uint8_t thr);

    void registerFrameObserver(ICamera::FrameCallback cb);
    void registerEstimateObserver(SunTracker::EstimateCallback cb);
    void registerSetpointObserver(Controller::SetpointCallback cb);
    void registerCommandObserver(Kinematics3RRS::CommandCallback cb);
    void registerLatencyObserver(LatencyObserver cb);

private:
    void controlLoop_();
    void actuatorLoop_();
    void onFrame_(const FrameEvent& fe);
    void setState_(TrackerState s);

    bool canAutoProcess_(TrackerState s) const noexcept;
    uint64_t nextSyntheticFrameId_() noexcept;

    void applyNeutralOnce_();
    void applyParkOnce_(float servo_deg);

    Logger& log_;
    std::unique_ptr<ICamera> camera_;

    SunTracker tracker_;
    Controller controller_;
    Kinematics3RRS kinematics_;
    ActuatorManager actuatorMgr_;
    float startup_park_deg_;
    ServoDriver driver_;
    LatencyMonitor latency_;

    ThreadSafeQueue<FrameEvent> frame_q_{1};
    ThreadSafeQueue<ActuatorCommand> cmd_q_{1};

    std::thread control_thread_;
    std::thread actuator_thread_;

    mutable std::mutex obs_mtx_;
    ICamera::FrameCallback frame_obs_{};
    SunTracker::EstimateCallback estimate_obs_{};
    Controller::SetpointCallback setpoint_obs_{};
    Kinematics3RRS::CommandCallback command_obs_{};
    LatencyObserver latency_obs_{};

    /// Serializes access to kinematics continuity state.
    mutable std::mutex kin_mtx_;

    std::atomic<bool> running_{false};
    std::atomic<TrackerState> state_{TrackerState::IDLE};
    std::atomic<uint64_t> next_synthetic_frame_id_{1ULL << 63};

    float min_confidence_{0.0f};
};

} // namespace solar