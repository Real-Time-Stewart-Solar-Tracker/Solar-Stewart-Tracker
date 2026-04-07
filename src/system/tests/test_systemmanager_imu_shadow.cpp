/**
 * @file test_systemmanager_imu_shadow.cpp
 * @brief Runtime tests for SystemManager IMU shadow and feedback mode behaviour.
 *
 * Covered scenarios:
 * - startup without IMU backend in Shadow mode
 * - manual/auto transitions unaffected by IMU feedback mode
 * - Shadow mode does not block FAULT state on camera failure
 * - imu_feedback_mode=Disabled does not prevent normal operation
 * - imu_feedback_mode=Live requires Live mode config but still starts normally
 *   when no IMU backend is configured (degrades gracefully)
 */

#include "app/AppConfig.hpp"
#include "common/Logger.hpp"
#include "sensors/ICamera.hpp"
#include "system/SystemManager.hpp"
#include "src/tests/support/test_common.hpp"

#include <atomic>
#include <memory>
#include <thread>
#include <utility>

using solar::ICamera;
using solar::Logger;
using solar::SystemManager;
using solar::TrackerState;
using solar::app::AppConfig;
using solar::app::CameraBackend;
using solar::app::ImuBackend;
using solar::app::ImuFeedbackMode;
using solar::app::ManualInputBackend;

namespace {

class FakeCamera final : public ICamera {
public:
    void registerFrameCallback(FrameCallback cb) override {
        cb_ = std::move(cb);
    }

    bool start() override {
        started_ = true;
        return start_ok_;
    }

    void stop() override {
        started_ = false;
    }

    bool isRunning() const override {
        return started_;
    }

    void setStartResult(bool ok) {
        start_ok_ = ok;
    }

private:
    bool started_{false};
    bool start_ok_{true};
    FrameCallback cb_{};
};

AppConfig makeBaseConfig(const ImuFeedbackMode imu_mode = ImuFeedbackMode::Shadow) {
    AppConfig cfg = solar::app::defaultConfig();
    cfg.camera_backend        = CameraBackend::Simulated;
    cfg.manual_input_backend  = ManualInputBackend::None;
    cfg.imu_backend           = ImuBackend::None;
    cfg.imu_feedback_mode     = imu_mode;
    cfg.servo.startup_policy  = solar::ServoDriver::StartupPolicy::LogOnly;
    cfg.startup_mode          = solar::app::StartupMode::Auto;
    return cfg;
}

} // namespace

// ---------------------------------------------------------------------------

TEST_CASE(SystemManager_ImuShadow_StartsWithoutImuBackend) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    SystemManager system(log, std::move(cam), makeBaseConfig(ImuFeedbackMode::Shadow));

    REQUIRE(system.start());
    REQUIRE(system.state() == TrackerState::SEARCHING);
    system.stop();
    REQUIRE(system.state() == TrackerState::IDLE);
}

TEST_CASE(SystemManager_ImuDisabled_StartsNormally) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    SystemManager system(log, std::move(cam), makeBaseConfig(ImuFeedbackMode::Disabled));

    REQUIRE(system.start());
    REQUIRE(system.state() == TrackerState::SEARCHING);
    system.stop();
    REQUIRE(system.state() == TrackerState::IDLE);
}

TEST_CASE(SystemManager_ImuShadow_ManualModeTransitions) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    SystemManager system(log, std::move(cam), makeBaseConfig(ImuFeedbackMode::Shadow));

    REQUIRE(system.start());

    system.enterManual();
    REQUIRE(system.state() == TrackerState::MANUAL);

    system.exitManual();
    REQUIRE(system.state() == TrackerState::SEARCHING);

    system.stop();
    REQUIRE(system.state() == TrackerState::IDLE);
}

TEST_CASE(SystemManager_ImuShadow_NullCameraEntersFault) {
    Logger log;
    SystemManager system(log, nullptr, makeBaseConfig(ImuFeedbackMode::Shadow));

    REQUIRE(!system.start());
    REQUIRE(system.state() == TrackerState::FAULT);
}

TEST_CASE(SystemManager_ImuShadow_CameraStartFailureEntersFault) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    cam->setStartResult(false);
    SystemManager system(log, std::move(cam), makeBaseConfig(ImuFeedbackMode::Shadow));

    REQUIRE(!system.start());
    REQUIRE(system.state() == TrackerState::FAULT);
}

TEST_CASE(SystemManager_ImuShadow_StopFromIdleIsIdempotent) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    SystemManager system(log, std::move(cam), makeBaseConfig(ImuFeedbackMode::Shadow));

    // stop() before start() must be a no-op and leave the state as IDLE.
    system.stop();
    REQUIRE(system.state() == TrackerState::IDLE);
}

TEST_CASE(SystemManager_ImuLive_StartsNormallyWhenNoImuBackendConfigured) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    // Live mode without an IMU backend should degrade to Disabled gracefully
    // rather than failing to start.
    SystemManager system(log, std::move(cam), makeBaseConfig(ImuFeedbackMode::Live));

    REQUIRE(system.start());
    REQUIRE(system.state() == TrackerState::SEARCHING);
    system.stop();
}

TEST_CASE(SystemManager_ImuShadow_StateObserverReceivesTransitions) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    SystemManager system(log, std::move(cam), makeBaseConfig(ImuFeedbackMode::Shadow));

    std::atomic<int> transition_count{0};
    system.registerStateObserver([&](TrackerState) {
        transition_count.fetch_add(1, std::memory_order_relaxed);
    });

    REQUIRE(system.start());
    // STARTUP -> NEUTRAL -> SEARCHING = at least 2 transitions observed
    REQUIRE(transition_count.load() >= 2);
    system.stop();
}
