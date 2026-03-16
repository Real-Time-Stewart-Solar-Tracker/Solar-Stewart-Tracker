#include "test_common.hpp"

#include "common/Logger.hpp"
#include "control/Kinematics3RRS.hpp"

#include <atomic>
#include <cmath>

using solar::ActuatorCommand;
using solar::Kinematics3RRS;
using solar::Logger;
using solar::PlatformSetpoint;

static bool isFinite(float x) {
    return std::isfinite(static_cast<double>(x)) != 0;
}

TEST(Kinematics3RRS_outputs_in_range_and_integer_like) {
    Logger log;
    Kinematics3RRS::Config cfg; // defaults
    Kinematics3RRS kin(log, cfg);

    std::atomic<bool> got{false};
    ActuatorCommand last{};

    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        last = cmd;
        got.store(true);
    });

    PlatformSetpoint sp;
    sp.frame_id = 123;
    sp.tilt_rad = 0.10f;
    sp.pan_rad  = -0.08f;

    kin.onSetpoint(sp);

    REQUIRE(got.load());

    for (int i = 0; i < 3; ++i) {
        const float deg = last.actuator_targets[static_cast<std::size_t>(i)];
        REQUIRE(isFinite(deg));
        REQUIRE(deg >= 0.0f);
        REQUIRE(deg <= 180.0f);

        // Kinematics rounds via lround() → should be very close to an integer.
        const float nearest = std::round(deg);
        REQUIRE_NEAR(deg, nearest, 1e-6f);
    }
}

TEST(Kinematics3RRS_continuity_small_setpoint_changes_small_output_changes) {
    Logger log;
    Kinematics3RRS::Config cfg; // defaults
    Kinematics3RRS kin(log, cfg);

    ActuatorCommand last{};
    ActuatorCommand prev{};
    bool havePrev = false;

    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        prev = last;
        last = cmd;
        if (!havePrev) {
            havePrev = true;
            prev = last;
        }
    });

    PlatformSetpoint sp{};
    sp.frame_id = 1;

    // Start at a gentle pose, then nudge slightly.
    sp.tilt_rad = 0.05f;
    sp.pan_rad  = 0.02f;
    kin.onSetpoint(sp);

    sp.frame_id = 2;
    sp.tilt_rad = 0.051f;
    sp.pan_rad  = 0.021f;
    kin.onSetpoint(sp);

    // Since inputs changed very slightly, outputs shouldn't jump wildly.
    // We allow a few degrees because rounding + branch logic can still move a bit.
    for (int i = 0; i < 3; ++i) {
        const float a = prev.actuator_targets[static_cast<std::size_t>(i)];
        const float b = last.actuator_targets[static_cast<std::size_t>(i)];
        REQUIRE(std::fabs(a - b) <= 10.0f);
    }
}

TEST(Kinematics3RRS_invalid_geometry_falls_back_to_last_valid) {
    Logger log;

    Kinematics3RRS::Config cfg;
    cfg.horn_length_m = 0.0f; // invalid (forces fallback path)
    Kinematics3RRS kin(log, cfg);

    ActuatorCommand last{};
    bool got = false;

    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        last = cmd;
        got = true;
    });

    PlatformSetpoint sp{};
    sp.frame_id = 99;
    sp.tilt_rad = 0.2f;
    sp.pan_rad  = 0.1f;

    kin.onSetpoint(sp);
    REQUIRE(got);

    // Should output last_valid_deg_ (initialized to neutral config)
    for (int i = 0; i < 3; ++i) {
        REQUIRE_NEAR(last.actuator_targets[static_cast<std::size_t>(i)],
                     cfg.servo_neutral_deg[static_cast<std::size_t>(i)], 1e-6f);
    }
}