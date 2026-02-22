#include "system/SystemManager.hpp"

#include <string>
#include <utility>

namespace solar {

static inline std::string stateToMsg(TrackerState s) {
    return std::string("STATE -> ") + toString(s);
}

float SystemManager::msBetween_(const std::chrono::steady_clock::time_point& a,
                                const std::chrono::steady_clock::time_point& b) {
    return std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(b - a).count();
}

void SystemManager::capMapSize_() {
    constexpr std::size_t MAX_FRAMES = 4096;
    if (times_.size() <= MAX_FRAMES) return;

    auto it = times_.begin();
    const std::size_t toErase = times_.size() - MAX_FRAMES;
    for (std::size_t i = 0; i < toErase && it != times_.end(); ++i) {
        it = times_.erase(it);
    }
}

void SystemManager::tryEmitLatency_(uint64_t frame_id) {
    LatencyObserver cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        cb = latency_obs_;
    }
    if (!cb) return;

    Times t{};
    {
        std::lock_guard<std::mutex> lk(lat_mtx_);
        auto it = times_.find(frame_id);
        if (it == times_.end()) return;

        t = it->second;
        if (!(t.has_cap && t.has_est && t.has_ctrl && t.has_act)) return;

        times_.erase(it);
    }

    cb(frame_id,
       msBetween_(t.t_cap,  t.t_est),
       msBetween_(t.t_est,  t.t_ctrl),
       msBetween_(t.t_ctrl, t.t_act));
}

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

    // Camera -> queue
    if (camera_) {
        camera_->registerFrameCallback([this](const FrameEvent& fe) {
            onFrame_(fe);
        });
    }

    // SunTracker -> Controller
    tracker_.registerEstimateCallback([this](const SunEstimate& est) {
        latency_.onEstimate(est.frame_id, est.t_estimate);

        {
            std::lock_guard<std::mutex> lk(lat_mtx_);
            auto& ts = times_[est.frame_id];
            ts.t_est = est.t_estimate;
            ts.has_est = true;
            capMapSize_();
        }

        SunTracker::EstimateCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = estimate_obs_;
        }
        if (cb) cb(est);

        if (running_.load() &&
            state_.load() != TrackerState::MANUAL &&
            state_.load() != TrackerState::FAULT) {
            if (est.confidence >= min_confidence_) setState_(TrackerState::TRACKING);
            else                                   setState_(TrackerState::SEARCHING);
        }

        if (state_.load() != TrackerState::MANUAL) {
            controller_.onEstimate(est);
        }
    });

    // Controller -> Kinematics
    controller_.registerSetpointCallback([this](const PlatformSetpoint& sp) {
        latency_.onControl(sp.frame_id, sp.t_control);

        {
            std::lock_guard<std::mutex> lk(lat_mtx_);
            auto& ts = times_[sp.frame_id];
            ts.t_ctrl = sp.t_control;
            ts.has_ctrl = true;
            capMapSize_();
        }

        Controller::SetpointCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = setpoint_obs_;
        }
        if (cb) cb(sp);

        kinematics_.onSetpoint(sp);
    });

    // Kinematics -> cmd queue
    kinematics_.registerCommandCallback([this](const ActuatorCommand& cmd) {
        Kinematics3RRS::CommandCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = command_obs_;
        }
        if (cb) cb(cmd);

        (void)cmd_q_.push_latest(cmd);
    });

    // ActuatorManager -> ServoDriver
    actuatorMgr_.registerSafeCommandCallback([this](const ActuatorCommand& safeCmd) {
        latency_.onActuate(safeCmd.frame_id, safeCmd.t_actuate);

        {
            std::lock_guard<std::mutex> lk(lat_mtx_);
            auto& ts = times_[safeCmd.frame_id];
            ts.t_act = safeCmd.t_actuate;
            ts.has_act = true;
            capMapSize_();
        }

        tryEmitLatency_(safeCmd.frame_id);
        driver_.apply(safeCmd);
    });
}

SystemManager::~SystemManager() {
    stop();
}

void SystemManager::onFrame_(const FrameEvent& fe) {
    latency_.onCapture(fe.frame_id, fe.t_capture);

    {
        std::lock_guard<std::mutex> lk(lat_mtx_);
        auto& ts = times_[fe.frame_id];
        ts.t_cap = fe.t_capture;
        ts.has_cap = true;
        capMapSize_();
    }

    ICamera::FrameCallback cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        cb = frame_obs_;
    }
    if (cb) cb(fe);

    (void)frame_q_.push_latest(fe);
}

bool SystemManager::start() {
    if (running_.load()) return true;

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

    // reset queues for clean run
    frame_q_.clear();
    cmd_q_.clear();
    frame_q_.reset();
    cmd_q_.reset();

    // Start worker threads before camera starts producing frames
    running_.store(true);
    control_thread_  = std::thread([this] { controlLoop_(); });
    actuator_thread_ = std::thread([this] { actuatorLoop_(); });

    if (!camera_->start()) {
        log_.error("SystemManager: Camera start failed");

        frame_q_.stop();
        cmd_q_.stop();

        if (control_thread_.joinable())  control_thread_.join();
        if (actuator_thread_.joinable()) actuator_thread_.join();

        running_.store(false);
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
    if (!running_.load()) return;

    setState_(TrackerState::STOPPING);

    // Send neutral while pipeline still running
    applyNeutralOnce_();

    if (camera_) camera_->stop();

    frame_q_.stop();
    cmd_q_.stop();

    if (control_thread_.joinable())  control_thread_.join();
    if (actuator_thread_.joinable()) actuator_thread_.join();

    driver_.stop();

    running_.store(false);

    latency_.printSummary();

    setState_(TrackerState::IDLE);
    log_.info("SystemManager stopped");
}

// -----------------------------------------------------------------------------
// Threads
// -----------------------------------------------------------------------------
void SystemManager::controlLoop_() {
    while (auto fe = frame_q_.wait_pop()) {
        tracker_.onFrame(*fe);
    }
}

void SystemManager::actuatorLoop_() {
    while (auto cmd = cmd_q_.wait_pop()) {
        actuatorMgr_.onCommand(*cmd);
    }
}

// -----------------------------------------------------------------------------
// Observer API
// -----------------------------------------------------------------------------
void SystemManager::registerFrameObserver(ICamera::FrameCallback cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    frame_obs_ = std::move(cb);
}

void SystemManager::registerEstimateObserver(SunTracker::EstimateCallback cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    estimate_obs_ = std::move(cb);
}

void SystemManager::registerSetpointObserver(Controller::SetpointCallback cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    setpoint_obs_ = std::move(cb);
}

void SystemManager::registerCommandObserver(Kinematics3RRS::CommandCallback cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    command_obs_ = std::move(cb);
}

void SystemManager::registerLatencyObserver(LatencyObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    latency_obs_ = std::move(cb);
}

// -----------------------------------------------------------------------------
// State machine API
// -----------------------------------------------------------------------------
TrackerState SystemManager::state() const {
    return state_.load();
}

void SystemManager::enterManual() {
    if (state_.load() == TrackerState::FAULT) return;
    setState_(TrackerState::MANUAL);
}

void SystemManager::exitManual() {
    if (state_.load() == TrackerState::FAULT) return;
    setState_(TrackerState::SEARCHING);
}

void SystemManager::setManualSetpoint(float tilt_rad, float pan_rad) {
    if (state_.load() != TrackerState::MANUAL) return;

    PlatformSetpoint sp;
    sp.frame_id = 0;
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad = tilt_rad;
    sp.pan_rad = pan_rad;

    kinematics_.onSetpoint(sp);
}

void SystemManager::setTrackerThreshold(uint8_t thr) {
    tracker_.setThreshold(thr);
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
void SystemManager::setState_(TrackerState s) {
    if (state_.load() == s) return;
    state_.store(s);
    log_.info(stateToMsg(s));
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