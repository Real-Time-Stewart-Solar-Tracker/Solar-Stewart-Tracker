/**
 * @file test_controller.cpp
 * @brief Unit tests for Controller.
 */

#include "control/Controller.hpp"
#include "src/tests/support/test_common.hpp"

#include <cmath>

using solar::Controller;
using solar::Logger;
using solar::PlatformSetpoint;
using solar::SunEstimate;

namespace {

bool nearlyEqual(float a, float b, float eps = 1e-5F) {
    return std::fabs(a - b) <= eps;
}

} // namespace

TEST_CASE(Controller_LowConfidence_NoMotion) {
    Logger log;

    Controller::Config cfg{};
    cfg.image_width = 640;
    cfg.image_height = 480;
    cfg.k_pan = 1.0F;
    cfg.k_tilt = 1.0F;
    cfg.max_pan_rad = 0.35F;
    cfg.max_tilt_rad = 0.35F;
    cfg.deadband = 0.0F;
    cfg.min_confidence = 0.5F;

    Controller controller(log, cfg);

    PlatformSetpoint out{};
    bool called = false;
    controller.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        called = true;
    });

    SunEstimate est{};
    est.frame_id = 1;
    est.cx = 320.0F;
    est.cy = 240.0F;
    est.confidence = 0.49F;

    controller.onEstimate(est);

    REQUIRE(called);
    REQUIRE(out.frame_id == 1U);
    REQUIRE(nearlyEqual(out.pan_rad, 0.0F));
    REQUIRE(nearlyEqual(out.tilt_rad, 0.0F));
}

TEST_CASE(Controller_WithinDeadband_NoMotion) {
    Logger log;

    Controller::Config cfg{};
    cfg.image_width = 640;
    cfg.image_height = 480;
    cfg.k_pan = 1.0F;
    cfg.k_tilt = 1.0F;
    cfg.max_pan_rad = 0.35F;
    cfg.max_tilt_rad = 0.35F;
    cfg.deadband = 0.05F;
    cfg.min_confidence = 0.1F;

    Controller controller(log, cfg);

    PlatformSetpoint out{};
    bool called = false;
    controller.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        called = true;
    });

    SunEstimate est{};
    est.frame_id = 1;
    est.cx = 320.0F + 10.0F;
    est.cy = 240.0F + 5.0F;
    est.confidence = 1.0F;

    controller.onEstimate(est);

    REQUIRE(called);
    REQUIRE(nearlyEqual(out.pan_rad, 0.0F));
    REQUIRE(nearlyEqual(out.tilt_rad, 0.0F));
}

TEST_CASE(Controller_OutsideDeadband_ProducesMotion) {
    Logger log;

    Controller::Config cfg{};
    cfg.image_width = 640;
    cfg.image_height = 480;
    cfg.k_pan = 1.0F;
    cfg.k_tilt = 1.0F;
    cfg.max_pan_rad = 0.35F;
    cfg.max_tilt_rad = 0.35F;
    cfg.deadband = 0.0F;
    cfg.min_confidence = 0.1F;

    Controller controller(log, cfg);

    PlatformSetpoint out{};
    bool called = false;
    controller.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        called = true;
    });

    SunEstimate est{};
    est.frame_id = 5;
    est.cx = 640.0F;
    est.cy = 0.0F;
    est.confidence = 1.0F;

    controller.onEstimate(est);

    REQUIRE(called);
    REQUIRE(out.frame_id == 5U);
    REQUIRE(out.pan_rad > 0.0F);
    REQUIRE(out.tilt_rad < 0.0F);
}

TEST_CASE(Controller_OutputClamped) {
    Logger log;

    Controller::Config cfg{};
    cfg.image_width = 640;
    cfg.image_height = 480;
    cfg.k_pan = 10.0F;
    cfg.k_tilt = 10.0F;
    cfg.max_pan_rad = 0.20F;
    cfg.max_tilt_rad = 0.10F;
    cfg.deadband = 0.0F;
    cfg.min_confidence = 0.1F;

    Controller controller(log, cfg);

    PlatformSetpoint out{};
    bool called = false;
    controller.registerSetpointCallback([&](const PlatformSetpoint& sp) {
        out = sp;
        called = true;
    });

    SunEstimate est{};
    est.frame_id = 9;
    est.cx = 0.0F;
    est.cy = 480.0F;
    est.confidence = 1.0F;

    controller.onEstimate(est);

    REQUIRE(called);
    REQUIRE(nearlyEqual(out.pan_rad, -0.20F));
    REQUIRE(nearlyEqual(out.tilt_rad, 0.10F));
}
