#include "system/SystemManager.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>
#include <utility>

namespace solar {

namespace {

std::string stateToMsg(const TrackerState s) {
    return std::string("STATE -> ") + toString(s);
}

app::AppConfig makeNoBackendConfig_(const Controller::Config& controllerCfg) {
    app::AppConfig cfg = app::defaultConfig();
    cfg.manual_input_backend = app::ManualInputBackend::None;
    cfg.ads1115_manual.enabled = false;
    cfg.imu_backend = app::ImuBackend::None;
    cfg.mpu6050.enabled = false;
    cfg.imu_feedback_mode = app::ImuFeedbackMode::Disabled;
    cfg.controller = controllerCfg;
    return cfg;
}

} // namespace

// ---------------------------------------------------------------------------
// Constructors / destructor
// ---------------------------------------------------------------------------

SystemManager::SystemManager(
    Logger& log,
    std::unique_ptr<ICamera> camera,
    const app::AppConfig& cfg)
    : log_(log),
      camera_(std::move(camera)),
      tracker_(log_, cfg.tracker),
      controller_(log_, cfg.controller),
      kinematics_(log_, cfg.kinematics),
      actuatorMgr_(log_, cfg.actuator),
      startup_park_deg_(cfg.servo.ch[0].neutral_deg),
      driver_(log_, cfg.servo),
      latency_(log_),
      config_(cfg),
      manual_imu_(cfg.manual_mapping,
                  cfg.imu_feedback,
                  cfg.imu_feedback_mode,
                  cfg.controller.max_tilt_rad,
                  cfg.controller.max_pan_rad),
      gui_dispatcher_(log_,
                      manual_imu_,
                      kinematics_,
                      latency_,
                      state_,
                      next_synthetic_frame_id_,
                      cfg.controller.max_tilt_rad,
                      cfg.controller.max_pan_rad),
      backends_(cfg,
                [this](const ManualPotSample& sample) { onManualPotSample_(sample); },
                [this](const control::ImuSample& sample) { onImuSample_(sample); }) {
    initialiseCallbacks_();
}

SystemManager::SystemManager(
    Logger& log,
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
      config_(makeNoBackendConfig_(controllerCfg)),
      manual_imu_({3.3F, false, false, 0.0F, 0.0F, 20.0F, 20.0F},
                  {0.12F, 0.01745329252F, 0.01745329252F},
                  app::ImuFeedbackMode::Disabled,
                  controllerCfg.max_tilt_rad,
                  controllerCfg.max_pan_rad),
      gui_dispatcher_(log_,
                      manual_imu_,
                      kinematics_,
                      latency_,
                      state_,
                      next_synthetic_frame_id_,
                      controllerCfg.max_tilt_rad,
                      controllerCfg.max_pan_rad),
      backends_(config_,
                [this](const ManualPotSample& sample) { onManualPotSample_(sample); },
                [this](const control::ImuSample& sample) { onImuSample_(sample); }) {
    initialiseCallbacks_();
}

SystemManager::~SystemManager() {
    stop();
}

void SystemManager::setManualCommandSource(const ManualCommandSource src) {
    manual_imu_.setManualCommandSource(src);
}

// ---------------------------------------------------------------------------
// Callback wiring
// ---------------------------------------------------------------------------

void SystemManager::initialiseCallbacks_() {
    latency_.registerObserver([this](std::uint64_t frame_id,
                                     double cap_to_est_ms,
                                     double est_to_ctrl_ms,
                                     double ctrl_to_act_ms) {
        LatencyObserver cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = latency_obs_;
        }
        if (cb) {
            cb(frame_id, cap_to_est_ms, est_to_ctrl_ms, ctrl_to_act_ms);
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

        const TrackerState current = state_.load();
        if (!canAutoProcess_(current)) {
            return;
        }

        setState_(est.confidence >= config_.controller.min_confidence
                      ? TrackerState::TRACKING
                      : TrackerState::SEARCHING);

        if (!canAutoProcess_(state_.load())) {
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

        const PlatformSetpoint corrected = manual_imu_.applyImuCorrection(sp);
        // kinematics_.onSetpoint() is thread-safe (internally protected by ik_mtx_).
        kinematics_.onSetpoint(corrected);
    });

    // Wire GuiManualDispatcher setpoint observer to the shared setpoint
    // observer so Qt plots receive GUI setpoints on the same channel as
    // automatic setpoints.
    gui_dispatcher_.registerSetpointObserver([this](const PlatformSetpoint& sp) {
        Controller::SetpointCallback cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = setpoint_obs_;
        }
        if (cb) {
            cb(sp);
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

        if (cmd.status == CommandStatus::KinematicsInvalidConfig) {
            log_.error("SystemManager: kinematics invalid; entering FAULT");
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

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

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
    frame_q_.reset();
    cmd_q_.clear();
    cmd_q_.reset();
    actuatorMgr_.resetHistory();
    manual_imu_.reset();

    {
        std::lock_guard<std::mutex> lk(manual_pot_mtx_);
        latest_manual_pot_sample_.reset();
    }

    // Start the GUI manual dispatcher. From this point, setManualSetpoint()
    // wakes the dispatcher's dedicated thread immediately.
    gui_dispatcher_.start();

    control_thread_ = std::thread([this] { controlLoop_(); });
    actuator_thread_ = std::thread([this] { actuatorLoop_(); });

    const BackendCoordinator::StartResult backend_result = backends_.start(log_);
    if (!backend_result.manual_ok) {
        log_.error("SystemManager: manual backend start failed");
        stop();
        setState_(TrackerState::FAULT);
        return false;
    }

    if (!backend_result.imu_ok) {
        if (manual_imu_.imuFeedbackMode() == app::ImuFeedbackMode::Live) {
            log_.error("SystemManager: IMU backend start failed in LIVE mode");
            stop();
            setState_(TrackerState::FAULT);
            return false;
        }

        log_.warn("SystemManager: IMU backend start failed; continuing without IMU");
        manual_imu_.setImuFeedbackMode(app::ImuFeedbackMode::Disabled);
    }

    if (!camera_->start()) {
        log_.error("SystemManager: camera start failed");
        stop();
        setState_(TrackerState::FAULT);
        return false;
    }

    setState_(TrackerState::NEUTRAL);
    applyParkOnce_(startup_park_deg_);

    if (config_.startup_mode == app::StartupMode::Manual) {
        setState_(TrackerState::MANUAL);
        manual_imu_.setManualCommandSource(ManualCommandSource::Pot);
    } else {
        setState_(TrackerState::SEARCHING);
    }

    log_.info("SystemManager started");
    return true;
}

void SystemManager::stop() {
    const bool was_running = running_.exchange(false);
    if (!was_running) {
        return;
    }

    setState_(TrackerState::STOPPING);

    if (camera_) {
        camera_->stop();
    }

    backends_.stop();

    // Stop the GUI dispatcher before stopping the queues so its thread
    // exits cleanly.
    gui_dispatcher_.stop();

    frame_q_.stop();
    cmd_q_.stop();

    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    if (actuator_thread_.joinable()) {
        actuator_thread_.join();
    }

    applyNeutralOnce_();
    driver_.stop();
    latency_.printSummary();

    setState_(TrackerState::IDLE);
    log_.info("SystemManager stopped");
}

// ---------------------------------------------------------------------------
// Worker threads
// ---------------------------------------------------------------------------

void SystemManager::controlLoop_() {
    // The control thread handles only the automatic processing path and the
    // potentiometer-manual path.
    //
    // - Automatic mode: full vision/control pipeline driven by camera frames.
    // - Pot manual mode: handled directly in onManualPotSample_() from the
    //   ADS1115 callback thread. The control thread is NOT involved.
    // - GUI manual mode: handled by GuiManualDispatcher on its own dedicated
    //   thread. The control thread is NOT involved.
    //
    // The control thread therefore has exactly one responsibility: processing
    // camera frames for the automatic tracking pipeline.
    while (true) {
        const auto item = frame_q_.wait_pop();
        if (!item.has_value()) {
            break;
        }

        if (!running_.load()) {
            break;
        }

        const TrackerState s = state_.load();

        if (!canAutoProcess_(s)) {
            // In MANUAL or FAULT states, camera frames are drained from the
            // queue but not processed. This keeps the queue healthy and
            // prevents stale frames from accumulating.
            continue;
        }

        tracker_.onFrame(*item);
    }
}

void SystemManager::actuatorLoop_() {
    while (true) {
        const auto item = cmd_q_.wait_pop();
        if (!item.has_value()) {
            break;
        }

        if (!running_.load()) {
            break;
        }

        actuatorMgr_.onCommand(*item);
    }
}

// ---------------------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------------------

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

void SystemManager::onManualPotSample_(const ManualPotSample& sample) {
    if (!running_.load()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lk(manual_pot_mtx_);
        latest_manual_pot_sample_ = sample;
    }

    ManualObserver cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        cb = manual_obs_;
    }
    if (cb) {
        cb(sample);
    }

    if (state_.load() != TrackerState::MANUAL) {
        return;
    }
    if (manual_imu_.manualCommandSource() != ManualCommandSource::Pot) {
        return;
    }

    // Potentiometer-driven manual path: dispatch directly to kinematics from
    // the ADS1115 callback context. Timing is driven by the ADS1115 ALERT/RDY
    // edge cadence, independent of camera frames.
    const std::uint64_t id = nextSyntheticFrameId_();
    const auto t_now = std::chrono::steady_clock::now();

    PlatformSetpoint sp{};
    const bool ok = manual_imu_.buildManualSetpointFromPot(
        sample, state_.load(), id, t_now, sp);

    if (!ok) {
        return;
    }

    Controller::SetpointCallback sp_cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        sp_cb = setpoint_obs_;
    }
    if (sp_cb) {
        sp_cb(sp);
    }

    latency_.onControl(sp.frame_id, sp.t_control);

    const PlatformSetpoint corrected = manual_imu_.applyImuCorrection(sp);
    // kinematics_.onSetpoint() is thread-safe (internally protected by ik_mtx_).
    kinematics_.onSetpoint(corrected);
}

void SystemManager::onImuSample_(const control::ImuSample& sample) {
    if (!running_.load()) {
        return;
    }

    manual_imu_.updateImuSample(sample);

    float tilt = 0.0F;
    const bool valid = manual_imu_.latestImuTiltRad(tilt);

    ImuObserver imu_cb;
    {
        std::lock_guard<std::mutex> lk(obs_mtx_);
        imu_cb = imu_obs_;
    }
    if (imu_cb) {
        imu_cb(sample, tilt, valid);
    }
}

// ---------------------------------------------------------------------------
// Public control API
// ---------------------------------------------------------------------------

TrackerState SystemManager::state() const {
    return state_.load();
}

void SystemManager::enterManual() {
    manual_imu_.setManualCommandSource(ManualCommandSource::Pot);
    setState_(TrackerState::MANUAL);
}

void SystemManager::exitManual() {
    if (!running_.load()) {
        setState_(TrackerState::IDLE);
        return;
    }
    setState_(TrackerState::SEARCHING);
}

void SystemManager::setManualSetpoint(const float tilt_rad, const float pan_rad) {
    // Forward to the GUI dispatcher which wakes its own dedicated thread.
    // This path is independent of camera-frame timing.
    gui_dispatcher_.setSetpoint(tilt_rad, pan_rad);
}

void SystemManager::setTrackerThreshold(const std::uint8_t thr) {
    tracker_.setThreshold(thr);
}

void SystemManager::setMinConfidence(const float confidence) {
    config_.controller.min_confidence = confidence;
}

// ---------------------------------------------------------------------------
// Observer registration
// ---------------------------------------------------------------------------

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

void SystemManager::registerStateObserver(StateObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    state_obs_ = std::move(cb);
}

void SystemManager::registerManualObserver(ManualObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    manual_obs_ = std::move(cb);
}

void SystemManager::registerImuObserver(ImuObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    imu_obs_ = std::move(cb);
}

void SystemManager::registerLatencyObserver(LatencyObserver cb) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    latency_obs_ = std::move(cb);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void SystemManager::setState_(const TrackerState s) {
    const TrackerState prev = state_.exchange(s);
    if (prev != s) {
        log_.info(stateToMsg(s));

        StateObserver cb;
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            cb = state_obs_;
        }
        if (cb) {
            cb(s);
        }
    }
}

bool SystemManager::canAutoProcess_(const TrackerState s) const noexcept {
    return s == TrackerState::SEARCHING || s == TrackerState::TRACKING;
}

std::uint64_t SystemManager::nextSyntheticFrameId_() noexcept {
    return next_synthetic_frame_id_.fetch_add(1U, std::memory_order_relaxed);
}

void SystemManager::applyNeutralOnce_() {
    PlatformSetpoint sp{};
    sp.frame_id  = nextSyntheticFrameId_();
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad  = 0.0F;
    sp.pan_rad   = 0.0F;

    // kinematics_.onSetpoint() is thread-safe.
    kinematics_.onSetpoint(sp);

    auto maybe_cmd = cmd_q_.try_pop();
    if (maybe_cmd.has_value()) {
        actuatorMgr_.onCommand(*maybe_cmd);
    }
}

void SystemManager::applyParkOnce_(const float servo_deg) {
    ActuatorCommand cmd{};
    cmd.frame_id         = nextSyntheticFrameId_();
    cmd.t_actuate        = std::chrono::steady_clock::now();
    cmd.status           = CommandStatus::Ok;
    cmd.actuator_targets = {servo_deg, servo_deg, servo_deg};

    actuatorMgr_.onCommand(cmd);
}

} // namespace solar
