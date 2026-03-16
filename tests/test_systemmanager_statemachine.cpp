#include "test_common.hpp"

#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "sensors/ICamera.hpp"
#include "system/SystemManager.hpp"
#include "system/TrackerState.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using solar::ActuatorCommand;
using solar::ActuatorManager;
using solar::Controller;
using solar::FrameEvent;
using solar::ICamera;
using solar::Kinematics3RRS;
using solar::Logger;
using solar::SunTracker;
using solar::SystemManager;
using solar::TrackerState;
using solar::ServoDriver;

namespace {

// Minimal fake camera: no internal thread; test drives frames via emit().
class FakeCamera final : public ICamera {
public:
    void registerFrameCallback(FrameCallback cb) override {
        cb_ = std::move(cb);
    }

    bool start() override {
        running_ = true;
        return start_ok_;
    }

    void stop() override {
        running_ = false;
    }

    bool isRunning() const noexcept override {
        return running_;
    }

    void setStartOk(bool ok) { start_ok_ = ok; }

    void emit(const FrameEvent& fe) {
        if (running_ && cb_) cb_(fe);
    }

private:
    FrameCallback cb_{};
    bool running_{false};
    bool start_ok_{true};
};

static FrameEvent makeBrightFrame(uint64_t id, int w, int h, uint8_t value = 255) {
    FrameEvent fe;
    fe.frame_id = id;
    fe.t_capture = std::chrono::steady_clock::now();
    fe.width = w;
    fe.height = h;
    fe.stride_bytes = w;
    fe.format = solar::PixelFormat::Gray8;
    fe.data.assign(static_cast<std::size_t>(w * h), value);
    return fe;
}

static bool waitUntil(std::function<bool()> pred, int timeout_ms = 300) {
    const auto start = std::chrono::steady_clock::now();
    while (!pred()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed > timeout_ms) return false;
    }
    return true;
}

} // namespace

TEST(SystemManager_start_to_searching_then_tracking_on_bright_frame) {
    Logger log;

    auto cam = std::make_unique<FakeCamera>();
    FakeCamera* camPtr = cam.get();

    // Small frame for fast tests
    SunTracker::Config trk{};
    trk.threshold = 200;
    trk.min_pixels = 10;
    trk.confidence_scale = 10.0f;

    Controller::Config ctrl{};
    ctrl.width = 10;
    ctrl.height = 10;
    ctrl.min_confidence = 0.01f; // easy to enter TRACKING for the test

    Kinematics3RRS::Config kin{};

    ActuatorManager::Config act{};
    // Your pipeline treats kinematics outputs as SERVO DEGREES, so make safety match that.
    act.min_out  = {0.0f, 0.0f, 0.0f};
    act.max_out  = {180.0f, 180.0f, 180.0f};
    act.max_step = {500.0f, 500.0f, 500.0f}; // effectively disable slew limiting for test

    ServoDriver::Config drv{};
    drv.pwm_hz = 50.0f;
    drv.i2c_dev = "/dev/i2c-1";
    drv.pca9685_addr = 0x40;
    drv.park_on_start = false;
    drv.park_on_stop  = false;
    drv.log_every_n   = 0;
    drv.ch[0] = ServoDriver::ChannelConfig{0, 500.f, 2500.f, 0.f, 180.f, 90.f, false};
    drv.ch[1] = ServoDriver::ChannelConfig{1, 500.f, 2500.f, 0.f, 180.f, 90.f, false};
    drv.ch[2] = ServoDriver::ChannelConfig{2, 500.f, 2500.f, 0.f, 180.f, 90.f, false};

    SystemManager sys(log, std::move(cam), trk, ctrl, kin, act, drv);

    REQUIRE(sys.state() == TrackerState::IDLE);
    REQUIRE(sys.start());
    REQUIRE(sys.state() == TrackerState::SEARCHING);

    // Send a very bright frame → should produce high confidence estimate
    camPtr->emit(makeBrightFrame(1, 10, 10, 255));

    // Wait until state becomes TRACKING (async pipeline)
    REQUIRE(waitUntil([&] { return sys.state() == TrackerState::TRACKING; }, 500));

    sys.stop();
    REQUIRE(sys.state() == TrackerState::IDLE);
}

TEST(SystemManager_manual_mode_emits_commands) {
    Logger log;

    auto cam = std::make_unique<FakeCamera>();

    SunTracker::Config trk{};
    trk.threshold = 200;
    trk.min_pixels = 10;

    Controller::Config ctrl{};
    ctrl.width = 10;
    ctrl.height = 10;
    ctrl.min_confidence = 0.01f;

    Kinematics3RRS::Config kin{};

    ActuatorManager::Config act{};
    act.min_out  = {0.0f, 0.0f, 0.0f};
    act.max_out  = {180.0f, 180.0f, 180.0f};
    act.max_step = {500.0f, 500.0f, 500.0f};

    ServoDriver::Config drv{};
    drv.park_on_start = false;
    drv.park_on_stop  = false;
    drv.ch[0] = ServoDriver::ChannelConfig{0, 500.f, 2500.f, 0.f, 180.f, 90.f, false};
    drv.ch[1] = ServoDriver::ChannelConfig{1, 500.f, 2500.f, 0.f, 180.f, 90.f, false};
    drv.ch[2] = ServoDriver::ChannelConfig{2, 500.f, 2500.f, 0.f, 180.f, 90.f, false};

    SystemManager sys(log, std::move(cam), trk, ctrl, kin, act, drv);

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

    // Should produce a kinematics command (and flow through actuator thread)
    sys.setManualSetpoint(0.10f, -0.10f);

    {
        std::unique_lock<std::mutex> lk(m);
        const bool ok = cv.wait_for(lk, std::chrono::milliseconds(500), [&] { return gotCmd; });
        REQUIRE(ok);
    }

    sys.stop();
    REQUIRE(sys.state() == TrackerState::IDLE);
}