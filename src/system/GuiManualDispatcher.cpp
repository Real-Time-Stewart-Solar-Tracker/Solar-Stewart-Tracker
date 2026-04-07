#include "system/GuiManualDispatcher.hpp"

#include <algorithm>
#include <chrono>

namespace solar {

GuiManualDispatcher::GuiManualDispatcher(Logger& log,
                                         ManualImuCoordinator& coordinator,
                                         Kinematics3RRS& kinematics,
                                         LatencyMonitor& latency,
                                         const std::atomic<TrackerState>& state,
                                         std::atomic<std::uint64_t>& next_frame_id,
                                         const float max_tilt_rad,
                                         const float max_pan_rad)
    : log_(log),
      coordinator_(coordinator),
      kinematics_(kinematics),
      latency_(latency),
      state_(state),
      next_frame_id_(next_frame_id),
      max_tilt_rad_(max_tilt_rad),
      max_pan_rad_(max_pan_rad) {
}

GuiManualDispatcher::~GuiManualDispatcher() {
    stop();
}

void GuiManualDispatcher::start() {
    queue_.reset();
    running_.store(true);
    worker_ = std::thread([this] { workerLoop_(); });
}

void GuiManualDispatcher::stop() {
    running_.store(false);
    queue_.stop();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void GuiManualDispatcher::setSetpoint(const float tilt_rad, const float pan_rad) {
    if (!running_.load()) {
        return;
    }

    GuiSetpoint sp{};
    sp.tilt_rad = std::clamp(tilt_rad, -max_tilt_rad_, max_tilt_rad_);
    sp.pan_rad  = std::clamp(pan_rad,  -max_pan_rad_,  max_pan_rad_);

    // push_latest drops the oldest entry if the queue is full, keeping only
    // the most recent slider position in flight.
    (void)queue_.push_latest(sp);
}

void GuiManualDispatcher::registerSetpointObserver(SetpointObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    setpoint_obs_ = std::move(cb);
}

void GuiManualDispatcher::workerLoop_() {
    // This thread blocks on queue_.wait_pop() and wakes exactly when the GUI
    // calls setSetpoint(). It is completely independent of the camera-frame
    // cadence. The timing of GUI manual output is driven by the rate at which
    // the operator moves the slider, not by when the next camera frame arrives.
    while (true) {
        const auto item = queue_.wait_pop();
        if (!item.has_value()) {
            break;
        }
        if (!running_.load()) {
            break;
        }
        dispatch_(*item);
    }
}

void GuiManualDispatcher::dispatch_(const GuiSetpoint& raw) {
    // Check that the system is in MANUAL mode and the GUI source owns the path.
    // If not, the setpoint is silently discarded — this can happen during a
    // mode transition where a queued setpoint arrives after exitManual().
    if (state_.load() != TrackerState::MANUAL) {
        return;
    }
    if (coordinator_.manualCommandSource() != ManualImuCoordinator::ManualCommandSource::Gui) {
        return;
    }

    const std::uint64_t id = next_frame_id_.fetch_add(1U, std::memory_order_relaxed);
    const auto t_now = std::chrono::steady_clock::now();

    PlatformSetpoint sp{};
    const bool ok = coordinator_.buildManualSetpointFromGui(
        raw.tilt_rad,
        raw.pan_rad,
        state_.load(),
        id,
        t_now,
        sp);

    if (!ok) {
        return;
    }

    // Notify the setpoint observer (e.g. Qt plot) with the raw GUI setpoint
    // before applying IMU correction, consistent with how the automatic path
    // notifies observers.
    SetpointObserver obs;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        obs = setpoint_obs_;
    }
    if (obs) {
        obs(sp);
    }

    latency_.onControl(sp.frame_id, sp.t_control);

    // Apply optional IMU correction (no-op in Disabled or Shadow mode).
    const PlatformSetpoint corrected = coordinator_.applyImuCorrection(sp);

    kinematics_.onSetpoint(corrected);
}

} // namespace solar
