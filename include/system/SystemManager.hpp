#pragma once

#include <atomic>
#include <chrono>       // FIX: required for Times and msBetween_
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

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
 * @brief Top-level system orchestrator for the solar tracking pipeline.
 *
 * Architecture (implemented):
 * - Camera backend emits FrameEvent via callback (camera owns capture thread)
 * - SystemManager pushes FrameEvent into a bounded queue (freshest-data policy)
 * - Control thread blocks on Frame queue and runs:
 *      SunTracker -> Controller -> Kinematics3RRS
 *   then pushes ActuatorCommand into a bounded queue
 * - Actuator thread blocks on Command queue and runs:
 *      ActuatorManager -> ServoDriver
 *
 * No polling loops or sleep-based timing are used in the realtime path.
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

    // ------------------------------------------------------------------
    // State Machine API
    // ------------------------------------------------------------------
    TrackerState state() const;

    void enterManual();
    void exitManual();

    void setManualSetpoint(float tilt_rad, float pan_rad);
    void setTrackerThreshold(uint8_t thr);

    // ------------------------------------------------------------------
    // Observers (UI / Telemetry hooks)
    // ------------------------------------------------------------------
    void registerFrameObserver(ICamera::FrameCallback cb);
    void registerEstimateObserver(SunTracker::EstimateCallback cb);
    void registerSetpointObserver(Controller::SetpointCallback cb);
    void registerCommandObserver(Kinematics3RRS::CommandCallback cb);
    void registerLatencyObserver(LatencyObserver cb);

private:
    // Thread loops
    void controlLoop_();
    void actuatorLoop_();

    // Camera callback
    void onFrame_(const FrameEvent& fe);

    // State helpers
    void setState_(TrackerState s);
    void applyNeutralOnce_();

    // Live latency cache (for LatencyObserver)
    struct Times {
        std::chrono::steady_clock::time_point t_cap{};
        std::chrono::steady_clock::time_point t_est{};
        std::chrono::steady_clock::time_point t_ctrl{};
        std::chrono::steady_clock::time_point t_act{};
        bool has_cap{false};
        bool has_est{false};
        bool has_ctrl{false};
        bool has_act{false};
    };

    static float msBetween_(const std::chrono::steady_clock::time_point& a,
                            const std::chrono::steady_clock::time_point& b);

    void capMapSize_();
    void tryEmitLatency_(uint64_t frame_id);

    Logger& log_;
    std::unique_ptr<ICamera> camera_;

    SunTracker tracker_;
    Controller controller_;
    Kinematics3RRS kinematics_;
    ActuatorManager actuatorMgr_;
    ServoDriver driver_;
    LatencyMonitor latency_;

    // Bounded queues (freshest-data semantics)
    ThreadSafeQueue<FrameEvent> frame_q_{1};       // keep latest frame
    ThreadSafeQueue<ActuatorCommand> cmd_q_{1};    // keep latest command

    // Worker threads
    std::thread control_thread_;
    std::thread actuator_thread_;

    // Observers (no globals)
    mutable std::mutex obs_mtx_;
    ICamera::FrameCallback frame_obs_{};
    SunTracker::EstimateCallback estimate_obs_{};
    Controller::SetpointCallback setpoint_obs_{};
    Kinematics3RRS::CommandCallback command_obs_{};
    LatencyObserver latency_obs_{};

    // Latency cache
    mutable std::mutex lat_mtx_;
    std::unordered_map<uint64_t, Times> times_;

    // State
    std::atomic<bool> running_{false};
    std::atomic<TrackerState> state_{TrackerState::IDLE};
    float min_confidence_{0.0f};

    PlatformSetpoint manual_sp_{};
};

} // namespace solar