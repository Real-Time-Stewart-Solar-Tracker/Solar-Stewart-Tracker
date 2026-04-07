#pragma once

/**
 * @file SystemManager.hpp
 * @brief Runtime pipeline orchestrator for the solar tracker.
 *
 * SystemManager is the runtime orchestration boundary. Its responsibilities
 * are:
 * - composing the pipeline stages
 * - owning the two worker threads and inter-thread queues
 * - managing the runtime state machine
 * - routing events between pipeline stages
 * - lifecycle management (start/stop)
 *
 * Backend lifecycle (ADS1115, IMU) is owned by @c BackendCoordinator.
 * GUI manual dispatch (event-driven, independent of camera frames) is owned
 * by @c GuiManualDispatcher.
 *
 * SystemManager does not own backend hardware details or GUI manual timing
 * policy. These are delegated to the appropriate single-responsibility class.
 */

#include "actuators/ActuatorManager.hpp"
#include "actuators/ServoDriver.hpp"
#include "app/AppConfig.hpp"
#include "common/LatencyMonitor.hpp"
#include "common/Logger.hpp"
#include "common/ThreadSafeQueue.hpp"
#include "common/Types.hpp"
#include "control/Controller.hpp"
#include "control/Kinematics3RRS.hpp"
#include "control/ManualImuCoordinator.hpp"
#include "sensors/ICamera.hpp"
#include "system/BackendCoordinator.hpp"
#include "system/GuiManualDispatcher.hpp"
#include "system/TrackerState.hpp"
#include "vision/SunTracker.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace solar {

namespace control {
struct ImuSample;
}

/**
 * @brief Runtime orchestrator for the solar tracker pipeline.
 */
class SystemManager {
public:
    /// Observer for runtime state transitions.
    using StateObserver = std::function<void(TrackerState new_state)>;

    /// Observer for raw manual potentiometer samples.
    using ManualObserver = std::function<void(const ManualPotSample& sample)>;

    /// Observer for IMU updates.
    using ImuObserver = std::function<void(const control::ImuSample& sample,
                                           float tilt_rad,
                                           bool valid)>;

    /// Observer for per-frame latency breakdown.
    using LatencyObserver = std::function<void(std::uint64_t frame_id,
                                               double cap_to_est_ms,
                                               double est_to_ctrl_ms,
                                               double ctrl_to_act_ms)>;

    /// Manual command source alias.
    using ManualCommandSource = ManualImuCoordinator::ManualCommandSource;

    /**
     * @brief Construct from full application configuration.
     *
     * @param log    Shared logger.
     * @param camera Camera backend (transferred ownership).
     * @param cfg    Full runtime configuration.
     */
    SystemManager(Logger& log,
                  std::unique_ptr<ICamera> camera,
                  const app::AppConfig& cfg);

    /**
     * @brief Construct from explicit per-stage configuration.
     *
     * Used primarily by tests that need to compose a runtime without a full
     * AppConfig.
     *
     * @param log          Shared logger.
     * @param camera       Camera backend (transferred ownership).
     * @param trackerCfg   Vision-stage configuration.
     * @param controllerCfg Controller-stage configuration.
     * @param kinCfg       Kinematics-stage configuration.
     * @param actCfg       Actuator-manager configuration.
     * @param drvCfg       Servo-driver configuration.
     */
    SystemManager(Logger& log,
                  std::unique_ptr<ICamera> camera,
                  SunTracker::Config trackerCfg,
                  Controller::Config controllerCfg,
                  Kinematics3RRS::Config kinCfg,
                  ActuatorManager::Config actCfg,
                  ServoDriver::Config drvCfg);

    ~SystemManager();

    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    /**
     * @brief Start the runtime, threads, and optional backends.
     *
     * @return True on successful startup.
     */
    bool start();

    /**
     * @brief Stop the runtime and all owned threads/backends cleanly.
     */
    void stop();

    /**
     * @brief Return the current tracker state.
     */
    TrackerState state() const;

    /** @brief Enter manual mode. */
    void enterManual();

    /** @brief Exit manual mode and return to automatic processing. */
    void exitManual();

    /**
     * @brief Submit a GUI manual setpoint.
     *
     * Pushes to the @c GuiManualDispatcher queue which wakes its dedicated
     * thread immediately. This path is completely independent of camera-frame
     * timing.
     *
     * @param tilt_rad Tilt command in radians.
     * @param pan_rad  Pan command in radians.
     */
    void setManualSetpoint(float tilt_rad, float pan_rad);

    /**
     * @brief Select which manual command source owns manual control.
     *
     * @param src Manual command source.
     */
    void setManualCommandSource(ManualCommandSource src);

    /** @brief Update the vision threshold at runtime. */
    void setTrackerThreshold(std::uint8_t threshold);

    /** @brief Update the minimum tracking confidence at runtime. */
    void setMinConfidence(float confidence);

    void registerFrameObserver(ICamera::FrameCallback cb);
    void registerEstimateObserver(SunTracker::EstimateCallback cb);
    void registerSetpointObserver(Controller::SetpointCallback cb);
    void registerCommandObserver(Kinematics3RRS::CommandCallback cb);
    void registerStateObserver(StateObserver cb);
    void registerManualObserver(ManualObserver cb);
    void registerImuObserver(ImuObserver cb);
    void registerLatencyObserver(LatencyObserver cb);

private:
    void initialiseCallbacks_();
    void controlLoop_();
    void actuatorLoop_();
    void onFrame_(const FrameEvent& fe);
    void onManualPotSample_(const ManualPotSample& sample);
    void onImuSample_(const control::ImuSample& sample);

    bool canAutoProcess_(TrackerState s) const noexcept;
    void setState_(TrackerState s);

    std::uint64_t nextSyntheticFrameId_() noexcept;

    void applyParkOnce_(float deg);
    void applyNeutralOnce_();

    Logger& log_;
    std::unique_ptr<ICamera> camera_;

    // Pipeline stages — each has a single, focused responsibility.
    SunTracker       tracker_;
    Controller       controller_;
    Kinematics3RRS   kinematics_;
    ActuatorManager  actuatorMgr_;
    float            startup_park_deg_{41.0F};
    ServoDriver      driver_;
    LatencyMonitor   latency_;

    app::AppConfig config_;

    // Inter-thread queues.
    ThreadSafeQueue<FrameEvent>      frame_q_{2};
    ThreadSafeQueue<ActuatorCommand> cmd_q_{8};

    // Worker threads.
    std::thread control_thread_;
    std::thread actuator_thread_;

    std::atomic<bool>         running_{false};
    std::atomic<TrackerState> state_{TrackerState::IDLE};
    std::atomic<std::uint64_t> next_synthetic_frame_id_{1};

    // Manual/IMU coordination policy — no threads, no hardware.
    ManualImuCoordinator manual_imu_;

    // Potentiometer manual state (accessed from ADS1115 callback thread).
    std::mutex manual_pot_mtx_;
    std::optional<ManualPotSample> latest_manual_pot_sample_;

    // Dedicated event-driven GUI manual dispatcher.
    // Owns its own thread and queue — GUI manual timing is independent of
    // camera-frame cadence.
    GuiManualDispatcher gui_dispatcher_;

    // Hardware backend lifecycle (ADS1115, IMU).
    // Separated from SystemManager so backend resource ownership is explicit.
    BackendCoordinator backends_;

    // Note: no kin_mtx_ — Kinematics3RRS::onSetpoint() is internally
    // thread-safe via its own ik_mtx_, requiring no external synchronisation.
    mutable std::mutex obs_mtx_;

    ICamera::FrameCallback           frame_obs_{};
    SunTracker::EstimateCallback     estimate_obs_{};
    Controller::SetpointCallback     setpoint_obs_{};
    Kinematics3RRS::CommandCallback  command_obs_{};
    StateObserver                    state_obs_{};
    ManualObserver                   manual_obs_{};
    ImuObserver                      imu_obs_{};
    LatencyObserver                  latency_obs_{};
};

} // namespace solar
