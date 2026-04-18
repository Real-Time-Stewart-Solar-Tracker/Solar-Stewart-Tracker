/**
 * @file test_systemmanager_pipeline.cpp
 * @brief End-to-end pipeline tests verifying that frames produce actuator commands.
 *
 * These tests close the gap between state-machine tests (which verify transitions)
 * and full pipeline validation (which verifies that actuator commands are actually
 * produced through the complete automatic and manual paths).
 */

#include "app/AppConfig.hpp"
#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "sensors/ICamera.hpp"
#include "system/SystemManager.hpp"
#include "src/tests/support/test_common.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>

using solar::ActuatorCommand;
using solar::FrameEvent;
using solar::ICamera;
using solar::Logger;
using solar::PixelFormat;
using solar::PlatformSetpoint;
using solar::SunEstimate;
using solar::SystemManager;
using solar::TrackerState;
using solar::app::AppConfig;
using solar::app::CameraBackend;
using solar::app::ImuBackend;
using solar::app::ImuFeedbackMode;
using solar::app::ManualInputBackend;

namespace {

template <typename Predicate>
bool waitFor(Predicate condition, int timeout_ms = 1000) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!condition()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        std::this_thread::yield();
    }
    return true;
}

class FakeCamera final : public ICamera {
public:
    void registerFrameCallback(FrameCallback cb) override {
        cb_ = std::move(cb);
    }

    bool start() override {
        started_ = true;
        return true;
    }

    void stop() override {
        started_ = false;
    }

    bool isRunning() const override {
        return started_;
    }

    void emitBrightFrame(std::uint64_t frame_id, int cx, int cy) {
        if (!cb_) {
            return;
        }

        FrameEvent fe{};
        fe.frame_id = frame_id;
        fe.t_capture = std::chrono::steady_clock::now();
        fe.width = 64;
        fe.height = 64;
        fe.stride_bytes = 64;
        fe.format = PixelFormat::Gray8;
        fe.data.assign(64U * 64U, 0U);

        // Draw a bright spot at (cx, cy) so the tracker produces a detection
        // with high confidence and the controller produces a non-zero setpoint.
        for (int dy = -3; dy <= 3; ++dy) {
            for (int dx = -3; dx <= 3; ++dx) {
                const int px = cx + dx;
                const int py = cy + dy;
                if (px >= 0 && px < 64 && py >= 0 && py < 64) {
                    fe.data[static_cast<std::size_t>(py * 64 + px)] = 255U;
                }
            }
        }

        cb_(fe);
    }

private:
    bool started_{false};
    FrameCallback cb_{};
};

AppConfig makePipelineTestConfig() {
    AppConfig cfg = solar::app::defaultConfig();
    cfg.camera_backend = CameraBackend::Simulated;
    cfg.manual_input_backend = ManualInputBackend::None;
    cfg.imu_backend = ImuBackend::None;
    cfg.imu_feedback_mode = ImuFeedbackMode::Disabled;
    cfg.servo.startup_policy = solar::ServoDriver::StartupPolicy::LogOnly;
    cfg.startup_mode = solar::app::StartupMode::Auto;
    cfg.tracker.min_pixels = 1U;
    cfg.tracker.threshold = 200U;
    cfg.controller.min_confidence = 0.0F;
    cfg.controller.deadband = 0.0F;
    cfg.controller.image_width = 64;
    cfg.controller.image_height = 64;
    return cfg;
}

} // namespace

// ---------------------------------------------------------------------------
// End-to-end automatic pipeline: frame → vision → control → kinematics → actuator
// ---------------------------------------------------------------------------

TEST_CASE(SystemManager_automatic_pipeline_produces_actuator_commands) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    auto* cam_ptr = cam.get();

    auto cfg = makePipelineTestConfig();

    SystemManager system(log, std::move(cam), cfg);

    std::atomic<int> command_count{0};
    ActuatorCommand last_cmd{};
    system.registerCommandObserver([&](const ActuatorCommand& cmd) {
        last_cmd = cmd;
        command_count.fetch_add(1, std::memory_order_relaxed);
    });

    REQUIRE(system.start());
    REQUIRE(system.state() == TrackerState::SEARCHING);

    // Emit a bright frame off-centre to produce a non-zero setpoint.
    // The full pipeline must run: SunTracker → Controller → Kinematics → ActuatorManager.
    cam_ptr->emitBrightFrame(1, 48, 48);

    // Wait for the actuator command to emerge from the full pipeline.
    const bool got_cmd = waitFor([&] {
        return command_count.load(std::memory_order_acquire) > 0;
    });

    REQUIRE(got_cmd);
    REQUIRE(last_cmd.frame_id >= 1);
    REQUIRE(last_cmd.status == solar::CommandStatus::Ok ||
            last_cmd.status == solar::CommandStatus::KinematicsFallbackLastValid);

    system.stop();
}

// ---------------------------------------------------------------------------
// End-to-end: multiple frames produce increasing command count
// ---------------------------------------------------------------------------

TEST_CASE(SystemManager_multiple_frames_produce_multiple_commands) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();
    auto* cam_ptr = cam.get();

    auto cfg = makePipelineTestConfig();

    SystemManager system(log, std::move(cam), cfg);

    std::atomic<int> command_count{0};
    system.registerCommandObserver([&](const ActuatorCommand&) {
        command_count.fetch_add(1, std::memory_order_relaxed);
    });

    REQUIRE(system.start());

    // Emit three frames and verify at least two commands arrive.
    // The queue uses push_latest with capacity 2, so some frames may be
    // dropped under load, but at least two should pass through.
    cam_ptr->emitBrightFrame(1, 40, 40);
    cam_ptr->emitBrightFrame(2, 42, 42);
    cam_ptr->emitBrightFrame(3, 44, 44);

    const bool got_multiple = waitFor([&] {
        return command_count.load(std::memory_order_acquire) >= 2;
    });

    REQUIRE(got_multiple);

    system.stop();
}

// ---------------------------------------------------------------------------
// GUI manual path is independent of camera frames
// ---------------------------------------------------------------------------

TEST_CASE(SystemManager_gui_manual_produces_commands_without_camera_frames) {
    Logger log;
    auto cam = std::make_unique<FakeCamera>();

    auto cfg = makePipelineTestConfig();

    SystemManager system(log, std::move(cam), cfg);

    REQUIRE(system.start());
    system.enterManual();
    system.setManualCommandSource(SystemManager::ManualCommandSource::Gui);
    REQUIRE(system.state() == TrackerState::MANUAL);

    std::atomic<int> command_count{0};
    system.registerCommandObserver([&](const ActuatorCommand&) {
        command_count.fetch_add(1, std::memory_order_relaxed);
    });

    // No camera frames emitted — the GUI manual path must produce commands
    // independently via GuiManualDispatcher.
    system.setManualSetpoint(0.15F, -0.10F);

    const bool got_cmd = waitFor([&] {
        return command_count.load(std::memory_order_acquire) > 0;
    });
    REQUIRE(got_cmd);

    // Second setpoint also produces a command independently.
    const int before = command_count.load(std::memory_order_acquire);
    system.setManualSetpoint(-0.05F, 0.20F);

    const bool got_second = waitFor([&] {
        return command_count.load(std::memory_order_acquire) > before;
    });
    REQUIRE(got_second);

    system.stop();
}