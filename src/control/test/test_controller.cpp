#include "test_common.hpp"

#include "control/Controller.hpp"
#include "common/Types.hpp"
#include "common/Logger.hpp"

#include <chrono>
#include <cmath>

using namespace solar;

static SunEstimate makeEstimate(uint64_t id, float cx, float cy, float confidence) {
    SunEstimate e;
    e.frame_id = id;
    e.t_estimate = std::chrono::steady_clock::now();
    e.cx = cx;
    e.cy = cy;
    e.confidence = confidence;
    return e;
}

TEST(Controller_LowConfidence_NoMotion) {
    Logger log;

    Controller::Config cfg;
    cfg.width = 640;
    cfg.height = 480;
    cfg.min_confidence = 0.5f;

    Controller ctrl(log, cfg);

    PlatformSetpoint out;
    bool got = false;
    ctrl.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        got = true;
    });

    auto est = makeEstimate(1, 320.0f, 240.0f, 0.1f);
    ctrl.onEstimate(est);

    REQUIRE(got);
    REQUIRE(out.tilt_rad == 0.0f);
    REQUIRE(out.pan_rad == 0.0f);
}

TEST(Controller_WithinDeadband_NoMotion) {
    Logger log;

    Controller::Config cfg;
    cfg.width = 640;
    cfg.height = 480;
    cfg.min_confidence = 0.0f;
    cfg.deadband = 0.05f; // normalized deadband

    Controller ctrl(log, cfg);

    PlatformSetpoint out;
    bool got = false;
    ctrl.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        got = true;
    });

    // Center is (320,240). Small shift: 10px/320 = 0.03125 (within 0.05 deadband)
    auto est = makeEstimate(2, 330.0f, 245.0f, 1.0f);
    ctrl.onEstimate(est);

    REQUIRE(got);
    REQUIRE(out.tilt_rad == 0.0f);
    REQUIRE(out.pan_rad == 0.0f);
}

TEST(Controller_OutsideDeadband_ProducesMotion) {
    Logger log;

    Controller::Config cfg;
    cfg.width = 640;
    cfg.height = 480;
    cfg.min_confidence = 0.0f;
    cfg.deadband = 0.01f;
    cfg.k_pan = 0.8f;
    cfg.k_tilt = 0.8f;

    Controller ctrl(log, cfg);

    PlatformSetpoint out;
    bool got = false;
    ctrl.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        got = true;
    });

    // Far from center: 80px/320=0.25 normalized error -> should produce non-zero output
    auto est = makeEstimate(3, 400.0f, 300.0f, 1.0f);
    ctrl.onEstimate(est);

    REQUIRE(got);
    REQUIRE(out.pan_rad != 0.0f);
    REQUIRE(out.tilt_rad != 0.0f);
}

TEST(Controller_OutputClamped) {
    Logger log;

    Controller::Config cfg;
    cfg.width = 640;
    cfg.height = 480;
    cfg.min_confidence = 0.0f;
    cfg.deadband = 0.0f;

    // Big gains + small limits to force clamping
    cfg.k_pan = 100.0f;
    cfg.k_tilt = 100.0f;
    cfg.max_pan_rad = 0.2f;
    cfg.max_tilt_rad = 0.2f;

    Controller ctrl(log, cfg);

    PlatformSetpoint out;
    bool got = false;
    ctrl.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        got = true;
    });

    auto est = makeEstimate(4, 1000.0f, 1000.0f, 1.0f);
    ctrl.onEstimate(est);

    REQUIRE(got);
    REQUIRE(std::fabs(out.pan_rad) <= 0.2f);
    REQUIRE(std::fabs(out.tilt_rad) <= 0.2f);
}