#pragma once

/**
 * @file GuiManualDispatcher.hpp
 * @brief Event-driven dispatcher for GUI manual setpoints.
 *
 * This class gives the GUI manual mode a first-class, independently-timed
 * event path that is structurally equivalent to the hardware potentiometer
 * path. When a GUI slider moves, the setpoint is pushed into a bounded
 * freshest-data queue. A dedicated worker thread blocks on that queue and
 * wakes immediately on each new setpoint, dispatching directly to
 * @c Kinematics3RRS without depending on the camera-frame cadence.
 *
 * This removes the asymmetry in the previous design where GUI manual timing
 * was driven by camera-frame arrivals while potentiometer timing was driven
 * by the ADS1115 ALERT/RDY edge. Both manual input paths are now
 * independently event-driven.
 *
 * Responsibilities:
 * - own one bounded freshest-data queue for incoming GUI setpoints
 * - own one dedicated worker thread blocking on that queue
 * - dispatch each setpoint directly to kinematics + actuation pipeline
 *
 * Non-responsibilities:
 * - state machine policy (checked by caller before calling setSetpoint)
 * - IMU correction (applied via ManualImuCoordinator)
 * - latency monitoring (notified after dispatch)
 * - thread count, camera path, or backend lifecycle
 */

#include "common/LatencyMonitor.hpp"
#include "common/Logger.hpp"
#include "common/ThreadSafeQueue.hpp"
#include "common/Types.hpp"
#include "control/Kinematics3RRS.hpp"
#include "control/ManualImuCoordinator.hpp"
#include "system/TrackerState.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace solar {

/**
 * @brief Dedicated event-driven dispatcher for GUI manual setpoints.
 *
 * The dispatcher owns a single worker thread that blocks on a freshest-data
 * queue. Calling @c setSetpoint() pushes a new target and immediately wakes
 * the thread. The thread builds a platform setpoint via
 * @c ManualImuCoordinator, applies optional IMU correction, and dispatches
 * directly to @c Kinematics3RRS — independent of camera-frame timing.
 */
class GuiManualDispatcher {
public:
    /**
     * @brief Observer called each time a GUI setpoint is dispatched.
     *
     * Matches the Controller::SetpointCallback signature so GUI setpoints
     * can be observed by the same Qt plotting path as automatic setpoints.
     */
    using SetpointObserver = std::function<void(const PlatformSetpoint&)>;

    /**
     * @brief Construct the dispatcher.
     *
     * @param log            Shared logger.
     * @param coordinator    Manual-input and IMU correction policy.
     * @param kinematics     Kinematics stage — receives dispatched setpoints.
     * @param latency        Latency monitor — notified on each dispatch.
     * @param state          Atomic tracker state — checked before dispatching.
     * @param max_tilt_rad   Clamp limit for tilt setpoints.
     * @param max_pan_rad    Clamp limit for pan setpoints.
     */
    GuiManualDispatcher(Logger& log,
                        ManualImuCoordinator& coordinator,
                        Kinematics3RRS& kinematics,
                        LatencyMonitor& latency,
                        const std::atomic<TrackerState>& state,
                        std::atomic<std::uint64_t>& next_frame_id,
                        float max_tilt_rad,
                        float max_pan_rad);

    ~GuiManualDispatcher();

    GuiManualDispatcher(const GuiManualDispatcher&) = delete;
    GuiManualDispatcher& operator=(const GuiManualDispatcher&) = delete;

    /**
     * @brief Start the dispatcher worker thread.
     */
    void start();

    /**
     * @brief Stop the dispatcher worker thread cleanly.
     */
    void stop();

    /**
     * @brief Submit a new GUI manual setpoint.
     *
     * This method is safe to call from any thread (e.g. the Qt UI thread).
     * If the queue is full the oldest entry is dropped, keeping only the
     * latest setpoint — consistent with the freshest-data policy used
     * throughout the pipeline.
     *
     * The setpoint is only dispatched if the system is currently in
     * @c TrackerState::MANUAL when the worker thread processes it.
     *
     * @param tilt_rad   Tilt command in radians.
     * @param pan_rad    Pan command in radians.
     */
    void setSetpoint(float tilt_rad, float pan_rad);

    /**
     * @brief Register an observer for dispatched GUI setpoints.
     *
     * @param cb Observer callback.
     */
    void registerSetpointObserver(SetpointObserver cb);

private:
    struct GuiSetpoint {
        float tilt_rad{0.0F};
        float pan_rad{0.0F};
    };

    void workerLoop_();
    void dispatch_(const GuiSetpoint& sp);

    Logger& log_;
    ManualImuCoordinator& coordinator_;
    Kinematics3RRS& kinematics_;
    LatencyMonitor& latency_;
    const std::atomic<TrackerState>& state_;
    std::atomic<std::uint64_t>& next_frame_id_;

    float max_tilt_rad_{0.35F};
    float max_pan_rad_{0.35F};

    /// Bounded freshest-data queue — capacity 1 so only the latest GUI
    /// setpoint is ever in flight. The worker thread always sees the most
    /// recent slider position, not a backlog of intermediate positions.
    ThreadSafeQueue<GuiSetpoint> queue_{1};

    std::thread worker_;
    std::atomic<bool> running_{false};

    mutable std::mutex obs_mtx_;
    SetpointObserver setpoint_obs_{};
};

} // namespace solar
