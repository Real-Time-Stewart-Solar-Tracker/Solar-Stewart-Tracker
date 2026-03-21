#include "system/SystemManager.hpp"

#include <algorithm>
#include <string>
#include <utility>

namespace solar {

static inline std::string stateToMsg(TrackerState s) {
    return std::string("STATE -> ") + toString(s);
}

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
      startup_park_deg_(drvCfg.ch[0].neutral_deg),
      driver_(log_, drvCfg),
      latency_(log_),
      min_confidence_(controllerCfg.min_confidence) {

    latency_.registerObserver([this](uint64_t frame_id,
                                     double cap_to_est_ms,
                                     double est_to_ctrl_ms,
                                     double ctrl_to_act_ms) {
        LatencyObserver cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = latency_obs_;
        }
        if (cb) {
            cb(frame_id,
               static_cast<float>(cap_to_est_ms),
               static_cast<float>(est_to_ctrl_ms),
               static_cast<float>(ctrl_to_act_ms));
        }
    });

    if (camera_) {
        camera_->registerFrameCallback([this](const FrameEvent& fe) {
            onFrame_(fe);
        });
    }

    tracker_.registerEstimateCallback([this](const SunEstimate& est) {
        latency_.onEstimate(est.frame_id, est.t_estimate);

        SunTracker::EstimateCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = estimate_obs_;
        }
        if (cb) {
            cb(est);
        }

        if (!running_.load()) {
            return;
        }

        const TrackerState before = state_.load();
        if (!canAutoProcess_(before)) {
            return;
        }

        setState_(est.confidence >= min_confidence_
                      ? TrackerState::TRACKING
                      : TrackerState::SEARCHING);

        const TrackerState after = state_.load();
        if (!canAutoProcess_(after)) {
            return;
        }

        controller_.onEstimate(est);
    });

    controller_.registerSetpointCallback([this](const PlatformSetpoint& sp) {
        latency_.onControl(sp.frame_id, sp.t_control);

        Controller::SetpointCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = setpoint_obs_;
        }
        if (cb) {
            cb(sp);
        }

        if (!running_.load()) {
            return;
        }

        if (!canAutoProcess_(state_.load())) {
            return;
        }

        {
            std::lock_guard<std::mutex> lk(kin_mtx_);
            kinematics_.onSetpoint(sp);
        }
    });

    kinematics_.registerCommandCallback([this](const ActuatorCommand& cmd) {
        Kinematics3RRS::CommandCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = command_obs_;
        }
        if (cb) {
            cb(cmd);
        }

        if (cmd.status != CommandStatus::Ok) {
            log_.error("SystemManager: kinematics produced degraded/invalid command; entering FAULT");
            setState_(TrackerState::FAULT);
            return;
        }

        (void)cmd_q_.push_latest(cmd);
    });

    actuatorMgr_.registerSafeCommandCallback([this](const ActuatorCommand& safeCmd) {
        auto out = safeCmd;
        out.t_actuate = std::chrono::steady_clock::now();

        latency_.onActuate(out.frame_id, out.t_actuate);
        driver_.apply(out);
    });
}

SystemManager::~SystemManager() {
    stop();
}

void SystemManager::onFrame_(const FrameEvent& fe) {
    latency_.onCapture(fe.frame_id, fe.t_capture);

    ICamera::FrameCallback cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        cb = frame_obs_;
    }
    if (cb) {
        cb(fe);
    }

    (void)frame_q_.push_latest(fe);
}

bool SystemManager::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true;
    }

    setState_(TrackerState::STARTUP);

    if (!camera_) {
        log_.error("SystemManager: camera is null");
        setState_(TrackerState::FAULT);
        running_.store(false);
        return false;
    }

    if (!driver_.start()) {
        log_.error("SystemManager: ServoDriver start failed");
        setState_(TrackerState::FAULT);
        running_.store(false);
        return false;
    }

    frame_q_.clear();
    cmd_q_.clear();
    frame_q_.reset();
    cmd_q_.reset();

    control_thread_  = std::thread([this] { controlLoop_(); });
    actuator_thread_ = std::thread([this] { actuatorLoop_(); });

    if (!camera_->start()) {
        log_.error("SystemManager: Camera start failed");

        frame_q_.stop();
        cmd_q_.stop();

        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        if (actuator_thread_.joinable()) {
            actuator_thread_.join();
        }

        driver_.stop();
        setState_(TrackerState::FAULT);
        running_.store(false);
        return false;
    }

    setState_(TrackerState::NEUTRAL);
    applyParkOnce_(startup_park_deg_);
    setState_(TrackerState::SEARCHING);

    log_.info("SystemManager started");
    return true;
}

void SystemManager::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }

    setState_(TrackerState::STOPPING);

    if (camera_) {
        camera_->stop();
    }

    frame_q_.stop();
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    cmd_q_.clear();
    cmd_q_.reset();

    applyNeutralOnce_();

    cmd_q_.stop();
    if (actuator_thread_.joinable()) {
        actuator_thread_.join();
    }

    driver_.stop();
    latency_.printSummary();

    setState_(TrackerState::IDLE);
    log_.info("SystemManager stopped");
}

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

TrackerState SystemManager::state() const {
    return state_.load();
}

void SystemManager::enterManual() {
    const TrackerState st = state_.load();
    if (st == TrackerState::FAULT || st == TrackerState::STOPPING || st == TrackerState::IDLE) {
        return;
    }
    setState_(TrackerState::MANUAL);
}

void SystemManager::exitManual() {
    const TrackerState st = state_.load();
    if (st == TrackerState::FAULT || st == TrackerState::STOPPING || st == TrackerState::IDLE) {
        return;
    }
    setState_(TrackerState::SEARCHING);
}

void SystemManager::setManualSetpoint(float tilt_rad, float pan_rad) {
    if (state_.load() != TrackerState::MANUAL) {
        return;
    }
    if (!running_.load()) {
        return;
    }

    tilt_rad = std::clamp(tilt_rad, -0.35f, 0.35f);
    pan_rad  = std::clamp(pan_rad,  -0.35f, 0.35f);

    PlatformSetpoint sp;
    sp.frame_id  = nextSyntheticFrameId_();
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad  = tilt_rad;
    sp.pan_rad   = pan_rad;

    {
        std::lock_guard<std::mutex> lk(kin_mtx_);
        kinematics_.onSetpoint(sp);
    }
}

void SystemManager::setTrackerThreshold(uint8_t thr) {
    tracker_.setThreshold(thr);
}

void SystemManager::setState_(TrackerState s) {
    if (state_.load() == s) {
        return;
    }
    state_.store(s);
    log_.info(stateToMsg(s));
}

bool SystemManager::canAutoProcess_(TrackerState s) const noexcept {
    return s == TrackerState::SEARCHING || s == TrackerState::TRACKING;
}

uint64_t SystemManager::nextSyntheticFrameId_() noexcept {
    return next_synthetic_frame_id_.fetch_add(1, std::memory_order_relaxed);
}

void SystemManager::applyNeutralOnce_() {
    PlatformSetpoint sp;
    sp.frame_id  = nextSyntheticFrameId_();
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad  = 0.0f;
    sp.pan_rad   = 0.0f;

    {
        std::lock_guard<std::mutex> lk(kin_mtx_);
        kinematics_.onSetpoint(sp);
    }
}

void SystemManager::applyParkOnce_(float servo_deg) {
    ActuatorCommand cmd;
    cmd.frame_id  = nextSyntheticFrameId_();
    cmd.t_actuate = std::chrono::steady_clock::now();
    cmd.actuator_targets[0] = servo_deg;
    cmd.actuator_targets[1] = servo_deg;
    cmd.actuator_targets[2] = servo_deg;

    (void)cmd_q_.push_latest(cmd);
}

} // namespace solar