#include "test_common.hpp"

#include "control/Controller.hpp"
#include "common/Types.hpp"
#include "common/Logger.hpp"

#include <chrono>

using namespace solar;

static SunEstimate makeEstimate(
    uint64_t id,
    float cx,
    float cy,
    float confidence)
{
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
    cfg.confidence_threshold = 0.5f;

    Controller ctrl(log, cfg);

    PlatformSetpoint out;
    bool got = false;

    ctrl.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        got = true;
    });

    auto est = makeEstimate(1, 320.0f, 240.0f, 0.1f); // low confidence
    ctrl.onEstimate(est);

    REQUIRE(got);
    REQUIRE(out.tilt_rad == 0.0f);
    REQUIRE(out.pan_rad == 0.0f);
}

TEST(Controller_CenterDeadband_NoMotion) {
    Logger log;

    Controller::Config cfg;
    cfg.width = 640;
    cfg.height = 480;
    cfg.deadband_px = 10.0f;

    Controller ctrl(log, cfg);

    PlatformSetpoint out;
    bool got = false;

    ctrl.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        got = true;
    });

    // Slightly off center but within deadband
    auto est = makeEstimate(2, 325.0f, 243.0f, 1.0f);
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
    cfg.deadband_px = 5.0f;
    cfg.kp = 0.01f;

    Controller ctrl(log, cfg);

    PlatformSetpoint out;
    bool got = false;

    ctrl.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        got = true;
    });

    // Far from center
    auto est = makeEstimate(3, 400.0f, 300.0f, 1.0f);
    ctrl.onEstimate(est);

    REQUIRE(got);
    REQUIRE(out.tilt_rad != 0.0f);
    REQUIRE(out.pan_rad != 0.0f);
}

TEST(Controller_OutputClamped) {
    Logger log;

    Controller::Config cfg;
    cfg.width = 640;
    cfg.height = 480;
    cfg.kp = 1.0f;             // large gain
    cfg.max_angle_rad = 0.2f;  // clamp

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
    REQUIRE(out.tilt_rad <= 0.2f);
    REQUIRE(out.pan_rad <= 0.2f);
}