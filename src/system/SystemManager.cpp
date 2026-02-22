#include "system/SystemManager.hpp"

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

namespace solar {

// -----------------------------------------------------------------------------
// File-local observer storage (keeps header clean + avoids UI coupling)
// -----------------------------------------------------------------------------
namespace {
std::mutex g_obs_mtx;

ICamera::FrameCallback              g_frame_obs;
SunTracker::EstimateCallback        g_est_obs;
Controller::SetpointCallback        g_setpoint_obs;
Kinematics3RRS::CommandCallback     g_cmd_obs;
SystemManager::LatencyObserver      g_lat_obs;

// Per-frame timing cache for live latency computation (ms)
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

// Keep it small and bounded: erase on completion, and also cap size.
std::mutex g_lat_mtx;
std::unordered_map<uint64_t, Times> g_times;

static inline float msBetween(const std::chrono::steady_clock::time_point& a,
                              const std::chrono::steady_clock::time_point& b) {
    return std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(b - a).count();
}

static inline void capMapSize() {
    // Very simple cap to prevent unbounded growth if something goes wrong.
    constexpr std::size_t MAX_FRAMES = 4096;
    if (g_times.size() <= MAX_FRAMES) return;

    // Erase arbitrary old entries (not ordered). Good enough for safety.
    auto it = g_times.begin();
    const std::size_t toErase = g_times.size() - MAX_FRAMES;
    for (std::size_t i = 0; i < toErase && it != g_times.end(); ++i) {
        it = g_times.erase(it);
    }
}

static inline void tryEmitLatency(uint64_t frame_id) {
    SystemManager::LatencyObserver cb;
    {
        std::lock_guard<std::mutex> lk(g_obs_mtx);
        cb = g_lat_obs;
    }
    if (!cb) return;

    Times t{};
    {
        std::lock_guard<std::mutex> lk(g_lat_mtx);
        auto it = g_times.find(frame_id);
        if (it == g_times.end()) return;
        t = it->second;

        // Emit only when we have the full chain
        if (!(t.has_cap && t.has_est && t.has_ctrl && t.has_act)) return;

        // Done -> erase immediately
        g_times.erase(it);
    }

    const float cap_to_est = msBetween(t.t_cap,  t.t_est);
    const float est_to_ctl = msBetween(t.t_est,  t.t_ctrl);
    const float ctl_to_act = msBetween(t.t_ctrl, t.t_act);

    cb(frame_id, cap_to_est, est_to_ctl, ctl_to_act);
}

} // namespace

// -----------------------------------------------------------------------------
// SystemManager
// -----------------------------------------------------------------------------
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
      latency_(log_),
      min_confidence_(controllerCfg.min_confidence) {

    // Camera -> SunTracker (+ frame observer)
    if (camera_) {
        camera_->registerFrameCallback([this](const FrameEvent& fe) {
            latency_.onCapture(fe.frame_id, fe.t_capture);

            // cache capture time for live latency
            {
                std::lock_guard<std::mutex> lk(g_lat_mtx);
                auto& ts = g_times[fe.frame_id];
                ts.t_cap = fe.t_capture;
                ts.has_cap = true;
                capMapSize();
            }

            // observer hook
            ICamera::FrameCallback cb;
            {
                std::lock_guard<std::mutex> lk(g_obs_mtx);
                cb = g_frame_obs;
            }
            if (cb) cb(fe);

            tracker_.onFrame(fe);
        });
    }

    // SunTracker -> Controller (+ estimate observer, + SEARCHING/TRACKING transitions)
    tracker_.registerEstimateCallback([this](const SunEstimate& est) {
        latency_.onEstimate(est.frame_id, est.t_estimate);

        // cache estimate time for live latency
        {
            std::lock_guard<std::mutex> lk(g_lat_mtx);
            auto& ts = g_times[est.frame_id];
            ts.t_est = est.t_estimate;
            ts.has_est = true;
            capMapSize();
        }

        // observer hook
        SunTracker::EstimateCallback cb;
        {
            std::lock_guard<std::mutex> lk(g_obs_mtx);
            cb = g_est_obs;
        }
        if (cb) cb(est);

        // confidence drives SEARCHING/TRACKING (unless MANUAL/FAULT)
        if (running_ && state_ != TrackerState::MANUAL && state_ != TrackerState::FAULT) {
            if (est.confidence >= min_confidence_) setState_(TrackerState::TRACKING);
            else                                   setState_(TrackerState::SEARCHING);
        }

        // In MANUAL mode, ignore AUTO controller updates
        if (state_ != TrackerState::MANUAL) {
            controller_.onEstimate(est);
        }
    });

    // Controller -> Kinematics (+ setpoint observer)
    controller_.registerSetpointCallback([this](const PlatformSetpoint& sp) {
        latency_.onControl(sp.frame_id, sp.t_control);

        // cache control time for live latency
        {
            std::lock_guard<std::mutex> lk(g_lat_mtx);
            auto& ts = g_times[sp.frame_id];
            ts.t_ctrl = sp.t_control;
            ts.has_ctrl = true;
            capMapSize();
        }

        // observer hook
        Controller::SetpointCallback cb;
        {
            std::lock_guard<std::mutex> lk(g_obs_mtx);
            cb = g_setpoint_obs;
        }
        if (cb) cb(sp);

        kinematics_.onSetpoint(sp);
    });

    // Kinematics -> ActuatorManager (+ command observer)
    kinematics_.registerCommandCallback([this](const ActuatorCommand& cmd) {
        // observer hook
        Kinematics3RRS::CommandCallback cb;
        {
            std::lock_guard<std::mutex> lk(g_obs_mtx);
            cb = g_cmd_obs;
        }
        if (cb) cb(cmd);

        actuatorMgr_.onCommand(cmd);
    });

    // ActuatorManager -> ServoDriver (+ latency complete)
    actuatorMgr_.registerSafeCommandCallback([this](const ActuatorCommand& safeCmd) {
        latency_.onActuate(safeCmd.frame_id, safeCmd.t_actuate);

        // cache actuate time for live latency, then try emit
        {
            std::lock_guard<std::mutex> lk(g_lat_mtx);
            auto& ts = g_times[safeCmd.frame_id];
            ts.t_act = safeCmd.t_actuate;
            ts.has_act = true;
            capMapSize();
        }

        tryEmitLatency(safeCmd.frame_id);

        driver_.apply(safeCmd);
    });
}

SystemManager::~SystemManager() {
    stop();
}

bool SystemManager::start() {
    if (running_) return true;

    setState_(TrackerState::STARTUP);

    if (!camera_) {
        log_.error("SystemManager: camera is null");
        setState_(TrackerState::FAULT);
        return false;
    }

    if (!driver_.start()) {
        log_.error("SystemManager: ServoDriver start failed");
        setState_(TrackerState::FAULT);
        return false;
    }

    // IMPORTANT: set running_ before camera starts producing callbacks
    running_ = true;

    if (!camera_->start()) {
        log_.error("SystemManager: Camera start failed");
        running_ = false;
        driver_.stop();
        setState_(TrackerState::FAULT);
        return false;
    }

    setState_(TrackerState::NEUTRAL);
    applyNeutralOnce_();
    setState_(TrackerState::SEARCHING);

    log_.info("SystemManager started");
    return true;
}

void SystemManager::stop() {
    if (!running_) return;

    setState_(TrackerState::STOPPING);

    applyNeutralOnce_();

    if (camera_) camera_->stop();
    driver_.stop();

    running_ = false;

    latency_.printSummary();

    setState_(TrackerState::IDLE);
    log_.info("SystemManager stopped");
}

// -----------------------------------------------------------------------------
// Observer API
// -----------------------------------------------------------------------------
void SystemManager::registerFrameObserver(ICamera::FrameCallback cb) {
    std::lock_guard<std::mutex> lk(g_obs_mtx);
    g_frame_obs = std::move(cb);
}

void SystemManager::registerEstimateObserver(SunTracker::EstimateCallback cb) {
    std::lock_guard<std::mutex> lk(g_obs_mtx);
    g_est_obs = std::move(cb);
}

void SystemManager::registerSetpointObserver(Controller::SetpointCallback cb) {
    std::lock_guard<std::mutex> lk(g_obs_mtx);
    g_setpoint_obs = std::move(cb);
}

void SystemManager::registerCommandObserver(Kinematics3RRS::CommandCallback cb) {
    std::lock_guard<std::mutex> lk(g_obs_mtx);
    g_cmd_obs = std::move(cb);
}

void SystemManager::registerLatencyObserver(LatencyObserver cb) {
    std::lock_guard<std::mutex> lk(g_obs_mtx);
    g_lat_obs = std::move(cb);
}

// -----------------------------------------------------------------------------
// State machine API
// -----------------------------------------------------------------------------
TrackerState SystemManager::state() const {
    return state_;
}

void SystemManager::enterManual() {
    if (state_ == TrackerState::FAULT) return;
    setState_(TrackerState::MANUAL);
}

void SystemManager::exitManual() {
    if (state_ == TrackerState::FAULT) return;
    setState_(TrackerState::SEARCHING);
}

void SystemManager::setManualSetpoint(float tilt_rad, float pan_rad) {
    if (state_ != TrackerState::MANUAL) return;

    PlatformSetpoint sp;
    sp.frame_id = 0;
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad = tilt_rad;
    sp.pan_rad = pan_rad;

    kinematics_.onSetpoint(sp);
}

void SystemManager::setTrackerThreshold(uint8_t thr) {
    // requires SunTracker::setThreshold(uint8_t) in your SunTracker class
    tracker_.setThreshold(thr);
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
void SystemManager::setState_(TrackerState s) {
    if (state_ == s) return;
    state_ = s;
    log_.info(std::string("STATE -> ") + toString(state_));
}

void SystemManager::applyNeutralOnce_() {
    PlatformSetpoint sp;
    sp.frame_id = 0;
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad = 0.0f;
    sp.pan_rad = 0.0f;

    kinematics_.onSetpoint(sp);
}

} // namespace solar