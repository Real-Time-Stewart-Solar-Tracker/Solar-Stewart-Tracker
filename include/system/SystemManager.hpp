#pragma once

#include <atomic>
#include <chrono>
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
 * @brief Top-level orchestrator of the solar tracking system.
 *
 * Coordinates the complete event-driven processing pipeline:
 *
 * - Camera backend emits FrameEvent via callback (capture thread owned by camera)
 * - SystemManager pushes frames into a bounded queue (freshest-data policy)
 * - Control thread blocks on frame queue and executes:
 *      SunTracker → Controller → Kinematics3RRS
 * - Actuator thread blocks on command queue and executes:
 *      ActuatorManager → ServoDriver
 *
 * The design avoids polling loops and sleep-based timing in the realtime path.
 * All inter-stage communication is performed via bounded thread-safe queues.
 */
class SystemManager {
public:

    /**
     * @brief Callback type used to report latency measurements.
     *
     * @param frame_id        Frame identifier
     * @param cap_to_est_ms   Capture → Estimate latency (ms)
     * @param est_to_ctrl_ms  Estimate → Control latency (ms)
     * @param ctrl_to_act_ms  Control → Actuation latency (ms)
     */
    using LatencyObserver = std::function<void(uint64_t frame_id,
                                               float cap_to_est_ms,
                                               float est_to_ctrl_ms,
                                               float ctrl_to_act_ms)>;

    /**
     * @brief Construct the system manager and all processing modules.
     *
     * @param log           Logger reference
     * @param camera        Camera backend (ownership transferred)
     * @param trackerCfg    SunTracker configuration
     * @param controllerCfg Controller configuration
     * @param kinCfg        Kinematics configuration
     * @param actCfg        ActuatorManager configuration
     * @param drvCfg        ServoDriver configuration
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

    /**
     * @brief Destructor.
     *
     * Ensures the system is safely stopped and all worker threads are joined.
     */
    ~SystemManager();

    /**
     * @brief Start the processing pipeline.
     *
     * Initializes hardware, starts worker threads, and transitions the
     * system into active tracking mode.
     *
     * @return true if startup succeeds
     * @return false if initialization fails
     */
    bool start();

    /**
     * @brief Stop the processing pipeline.
     *
     * Safely halts camera capture, joins worker threads, sends a final
     * neutral command, and stops PWM output.
     *
     * This function is idempotent.
     */
    void stop();

    // ------------------------------------------------------------------
    // State Machine API
    // ------------------------------------------------------------------

    /**
     * @brief Get current tracker state.
     *
     * @return Current state of the system state machine.
     */
    TrackerState state() const;

    /**
     * @brief Enter manual control mode.
     *
     * In manual mode, automatic tracking is disabled and user-defined
     * setpoints are applied.
     */
    void enterManual();

    /**
     * @brief Exit manual mode and resume automatic tracking.
     */
    void exitManual();

    /**
     * @brief Apply a manual platform setpoint (radians).
     *
     * Only effective when in MANUAL state.
     *
     * @param tilt_rad Tilt angle in radians
     * @param pan_rad  Pan angle in radians
     */
    void setManualSetpoint(float tilt_rad, float pan_rad);

    /**
     * @brief Update SunTracker threshold parameter.
     *
     * @param thr Threshold value (0–255)
     */
    void setTrackerThreshold(uint8_t thr);

    // ------------------------------------------------------------------
    // Observers (UI / Telemetry hooks)
    // ------------------------------------------------------------------

    /**
     * @brief Register observer for raw camera frames.
     */
    void registerFrameObserver(ICamera::FrameCallback cb);

    /**
     * @brief Register observer for SunTracker estimates.
     */
    void registerEstimateObserver(SunTracker::EstimateCallback cb);

    /**
     * @brief Register observer for platform setpoints.
     */
    void registerSetpointObserver(Controller::SetpointCallback cb);

    /**
     * @brief Register observer for actuator commands.
     */
    void registerCommandObserver(Kinematics3RRS::CommandCallback cb);

    /**
     * @brief Register observer for latency measurements.
     */
    void registerLatencyObserver(LatencyObserver cb);

private:

    // Thread loops
    void controlLoop_();
    void actuatorLoop_();

    // Camera callback
    void onFrame_(const FrameEvent& fe);

    // State helpers
    void setState_(TrackerState s);

    /**
     * @brief Send a single neutral setpoint through the kinematics stage.
     */
    void applyNeutralOnce_();

    /**
     * @brief Send a servo-degree command through the actuation pipeline.
     *
     * @param servo_deg Target servo angle in degrees
     */
    void applyParkOnce_(float servo_deg);

    // ------------------------------------------------------------------
    // Latency bookkeeping
    // ------------------------------------------------------------------

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

    mutable std::mutex lat_mtx_;
    std::unordered_map<uint64_t, Times> times_;

    /// Serializes access to kinematics continuity state.
    mutable std::mutex kin_mtx_;

    std::atomic<bool> running_{false};
    std::atomic<TrackerState> state_{TrackerState::IDLE};
    float min_confidence_{0.0f};

    PlatformSetpoint manual_sp_{};
};

} // namespace solar