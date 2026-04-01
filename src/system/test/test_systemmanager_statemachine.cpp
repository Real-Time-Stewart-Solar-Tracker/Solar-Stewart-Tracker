#include "test_common.hpp"

#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "sensors/ICamera.hpp"
#include "system/SystemManager.hpp"
#include "system/TrackerState.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

using solar::ActuatorCommand;
using solar::ActuatorManager;
using solar::Controller;
using solar::FrameEvent;
using solar::ICamera;
using solar::Kinematics3RRS;
using solar::Logger;
using solar::ServoDriver;
using solar::SunTracker;
using solar::SystemManager;
using solar::TrackerState;

namespace {

// Minimal fake camera: no internal thread; the test drives frames via emit().
class FakeCamera final : public ICamera {
public:
    void registerFrameCallback(FrameCallback cb) override {
        cb_ = std::move(cb);
    }

    bool start() override {
        running_ = start_ok_;
        return start_ok_;
    }

    void stop() override {
        running_ = false;
    }

    bool isRunning() const noexcept override {
        return running_;
    }

    void setStartOk(bool ok) {
        start_ok_ = ok;
    }

    void emit(const FrameEvent& fe) {
        if (running_ && cb_) {
            cb_(fe);
        }
    }

private:
    FrameCallback cb_{};
    bool running_{false};
    bool start_ok_{true};
};

static FrameEvent makeBrightFrame(std::uint64_t id, int w, int h, std::uint8_t value = 255) {
    FrameEvent fe{};
    fe.frame_id = id;
    fe.t_capture = std::chrono::steady_clock::now();
    fe.width = w;
    fe.height = h;
    fe.stride_bytes = w;
    fe.format = solar::PixelFormat::Gray8;
    fe.data.assign(static_cast<std::size_t>(w * h), value);
    return fe;
}

static bool waitUntil(const std::function<bool()>& pred, int timeout_ms = 300) {
    const auto start = std::chrono::steady_clock::now();
    while (!pred()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed > timeout_ms) {
            return false;
        }
    }
    return true;
}

static SunTracker::Config makeTrackerCfg() {
    SunTracker::Config trk{};
    trk.threshold = 200;
    trk.min_pixels = 10;
    trk.confidence_scale = 10.0f;
    return trk;
}

static Controller::Config makeControllerCfg() {
    Controller::Config ctrl{};
    ctrl.width = 10;
    ctrl.height = 10;
    ctrl.min_confidence = 0.01f; // easy to enter TRACKING in tests
    return ctrl;
}

static Kinematics3RRS::Config makeKinematicsCfg() {
    Kinematics3RRS::Config kin{};
    return kin;
}

static ActuatorManager::Config makeActuatorCfg() {
    ActuatorManager::Config act{};
    act.min_out  = {0.0f, 0.0f, 0.0f};
    act.max_out  = {180.0f, 180.0f, 180.0f};
    act.max_step = {500.0f, 500.0f, 500.0f}; // effectively disable slew limiting in tests
    return act;
}

static ServoDriver::Config makeServoCfg(solar::ServoDriver::StartupPolicy policy) {
    ServoDriver::Config drv{};
    drv.startup_policy = policy;
    drv.pwm_hz = 50.0f;
    drv.i2c_dev = "/dev/i2c-1";
    drv.pca9685_addr = 0x40;
    drv.park_on_start = false;
    drv.park_on_stop  = false;
    drv.log_every_n   = 0;

    drv.ch[0].channel = 0;
    drv.ch[0].min_pulse_us = 500.0f;
    drv.ch[0].max_pulse_us = 2500.0f;
    drv.ch[0].min_deg = 0.0f;
    drv.ch[0].max_deg = 180.0f;
    drv.ch[0].neutral_deg = 90.0f;
    drv.ch[0].invert = false;

    drv.ch[1] = drv.ch[0];
    drv.ch[1].channel = 1;

    drv.ch[2] = drv.ch[0];
    drv.ch[2].channel = 2;

    return drv;
}

} // namespace

TEST(SystemManager_start_to_searching_then_tracking_on_bright_frame) {
    Logger log;

    auto cam = std::make_unique<FakeCamera>();
    FakeCamera* camPtr = cam.get();

    SystemManager sys(log,
                      std::move(cam),
                      makeTrackerCfg(),
                      makeControllerCfg(),
                      makeKinematicsCfg(),
                      makeActuatorCfg(),
                      makeServoCfg(ServoDriver::StartupPolicy::LogOnly));

    REQUIRE(sys.state() == TrackerState::IDLE);
    REQUIRE(sys.start());
    REQUIRE(sys.state() == TrackerState::SEARCHING);

    // Send a very bright frame -> should produce high confidence estimate.
    camPtr->emit(makeBrightFrame(1, 10, 10, 255));

    REQUIRE(waitUntil([&] { return sys.state() == TrackerState::TRACKING; }, 500));

    sys.stop();
    REQUIRE(sys.state() == TrackerState::IDLE);
}

TEST(SystemManager_manual_mode_uses_nonzero_synthetic_frame_ids) {
    Logger log;

    auto cam = std::make_unique<FakeCamera>();

    SystemManager sys(log,
                      std::move(cam),
                      makeTrackerCfg(),
                      makeControllerCfg(),
                      makeKinematicsCfg(),
                      makeActuatorCfg(),
                      makeServoCfg(ServoDriver::StartupPolicy::LogOnly));

    std::mutex m;
    std::condition_variable cv;
    bool gotCmd = false;
    std::uint64_t seenId = 0;

    sys.registerCommandObserver([&](const ActuatorCommand& cmd) {
        std::lock_guard<std::mutex> lk(m);
        gotCmd = true;
        seenId = cmd.frame_id;
        cv.notify_one();
    });

    REQUIRE(sys.start());

    sys.enterManual();
    REQUIRE(sys.state() == TrackerState::MANUAL);

    sys.setManualSetpoint(0.10f, -0.10f);

    {
        std::unique_lock<std::mutex> lk(m);
        const bool ok = cv.wait_for(
            lk,
            std::chrono::milliseconds(500),
            [&] { return gotCmd; });
        REQUIRE(ok);
    }

    REQUIRE(seenId != 0);

    sys.stop();
}

TEST(SystemManager_manual_mode_emits_commands) {
    Logger log;

    auto cam = std::make_unique<FakeCamera>();

    SystemManager sys(log,
                      std::move(cam),
                      makeTrackerCfg(),
                      makeControllerCfg(),
                      makeKinematicsCfg(),
                      makeActuatorCfg(),
                      makeServoCfg(ServoDriver::StartupPolicy::LogOnly));

    std::mutex m;
    std::condition_variable cv;
    bool gotCmd = false;

    sys.registerCommandObserver([&](const ActuatorCommand&) {
        std::lock_guard<std::mutex> lk(m);
        gotCmd = true;
        cv.notify_one();
    });

    REQUIRE(sys.start());

    sys.enterManual();
    REQUIRE(sys.state() == TrackerState::MANUAL);

    sys.setManualSetpoint(0.10f, -0.10f);

    {
        std::unique_lock<std::mutex> lk(m);
        const bool ok = cv.wait_for(
            lk,
            std::chrono::milliseconds(500),
            [&] { return gotCmd; });
        REQUIRE(ok);
    }

    sys.stop();
    REQUIRE(sys.state() == TrackerState::IDLE);
}

TEST(SystemManager_start_with_null_camera_enters_fault_and_fails) {
    Logger log;

    std::unique_ptr<ICamera> nullCamera{};

    SystemManager sys(log,
                      std::move(nullCamera),
                      makeTrackerCfg(),
                      makeControllerCfg(),
                      makeKinematicsCfg(),
                      makeActuatorCfg(),
                      makeServoCfg(ServoDriver::StartupPolicy::LogOnly));

    REQUIRE(sys.state() == TrackerState::IDLE);
    REQUIRE(!sys.start());
    REQUIRE(sys.state() == TrackerState::FAULT);
}

TEST(SystemManager_start_when_servo_driver_requires_missing_hardware_enters_fault) {
    Logger log;

    auto cam = std::make_unique<FakeCamera>();
    auto drv = makeServoCfg(ServoDriver::StartupPolicy::RequireHardware);
    drv.i2c_dev = "/definitely/not/a/real/i2c/device";

    SystemManager sys(log,
                      std::move(cam),
                      makeTrackerCfg(),
                      makeControllerCfg(),
                      makeKinematicsCfg(),
                      makeActuatorCfg(),
                      drv);

    REQUIRE(sys.state() == TrackerState::IDLE);
    REQUIRE(!sys.start());
    REQUIRE(sys.state() == TrackerState::FAULT);
}

TEST(SystemManager_start_when_camera_start_fails_enters_fault) {
    Logger log;

    auto cam = std::make_unique<FakeCamera>();
    cam->setStartOk(false);

    SystemManager sys(log,
                      std::move(cam),
                      makeTrackerCfg(),
                      makeControllerCfg(),
                      makeKinematicsCfg(),
                      makeActuatorCfg(),
                      makeServoCfg(ServoDriver::StartupPolicy::LogOnly));

    REQUIRE(sys.state() == TrackerState::IDLE);
    REQUIRE(!sys.start());
    REQUIRE(sys.state() == TrackerState::FAULT);
}