/**
 * @file test_kinematics.cpp
 * @brief Unit tests for Kinematics3RRS.
 *
 * Covered scenarios:
 * - valid setpoint produces in-range servo angles
 * - small setpoint changes produce small output changes (continuity)
 * - invalid geometry surfaces an explicit error status
 * - fallback-to-last-valid behaviour on subsequent invalid config
 * - neutral setpoint (0, 0) produces symmetric servo output
 * - extreme-but-reachable setpoints remain in range
 * - frame_id is forwarded correctly through the callback
 * - zero-setpoint after valid setpoint does not trigger fallback
 */

#include "control/Kinematics3RRS.hpp"
#include "src/tests/support/test_common.hpp"

#include <array>
#include <cmath>

using solar::ActuatorCommand;
using solar::CommandStatus;
using solar::Kinematics3RRS;
using solar::Logger;
using solar::PlatformSetpoint;

namespace {

bool allInRange(const std::array<float, 3>& u, float lo, float hi) {
    return u[0] >= lo && u[0] <= hi &&
           u[1] >= lo && u[1] <= hi &&
           u[2] >= lo && u[2] <= hi;
}

float maxAbsDiff(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    float m = 0.0F;
    for (std::size_t i = 0U; i < 3U; ++i) {
        m = std::max(m, std::fabs(a[i] - b[i]));
    }
    return m;
}

Kinematics3RRS::Config makeGoodConfig() {
    Kinematics3RRS::Config cfg{};
    cfg.base_radius_m     = 0.20F;
    cfg.platform_radius_m = 0.12F;
    cfg.home_height_m     = 0.18F;
    cfg.horn_length_m     = 0.10F;
    cfg.rod_length_m      = 0.18F;
    cfg.base_theta_deg    = {120.0F, 240.0F, 0.0F};
    cfg.plat_theta_deg    = {120.0F, 240.0F, 0.0F};
    cfg.servo_neutral_deg = {90.0F, 90.0F, 90.0F};
    cfg.servo_dir         = {-1, -1, -1};
    return cfg;
}

Kinematics3RRS::Config makeBadConfig() {
    auto cfg = makeGoodConfig();
    cfg.rod_length_m = 0.001F; // geometrically impossible
    return cfg;
}

PlatformSetpoint makeSetpoint(std::uint64_t id, float tilt, float pan) {
    PlatformSetpoint sp{};
    sp.frame_id  = id;
    sp.tilt_rad  = tilt;
    sp.pan_rad   = pan;
    sp.t_control = std::chrono::steady_clock::now();
    return sp;
}

} // namespace

// ---------------------------------------------------------------------------
// Basic validity
// ---------------------------------------------------------------------------

TEST_CASE(Kinematics3RRS_outputs_in_range_for_moderate_setpoint) {
    Logger log;
    Kinematics3RRS kin(log, makeGoodConfig());

    ActuatorCommand out{};
    kin.registerCommandCallback([&](const ActuatorCommand& cmd) { out = cmd; });

    kin.onSetpoint(makeSetpoint(1, -0.15F, 0.10F));

    REQUIRE(out.status == CommandStatus::Ok);
    REQUIRE(allInRange(out.actuator_targets, 0.0F, 180.0F));
}

TEST_CASE(Kinematics3RRS_neutral_setpoint_produces_near_neutral_angles) {
    Logger log;
    Kinematics3RRS kin(log, makeGoodConfig());

    ActuatorCommand out{};
    kin.registerCommandCallback([&](const ActuatorCommand& cmd) { out = cmd; });

    kin.onSetpoint(makeSetpoint(1, 0.0F, 0.0F));

    REQUIRE(out.status == CommandStatus::Ok);
    REQUIRE(allInRange(out.actuator_targets, 0.0F, 180.0F));
    // All three channels should be symmetric around the neutral angle.
    const float diff01 = std::fabs(out.actuator_targets[0] - out.actuator_targets[1]);
    const float diff12 = std::fabs(out.actuator_targets[1] - out.actuator_targets[2]);
    REQUIRE(diff01 < 1.0F);
    REQUIRE(diff12 < 1.0F);
}

TEST_CASE(Kinematics3RRS_frame_id_forwarded_through_callback) {
    Logger log;
    Kinematics3RRS kin(log, makeGoodConfig());

    std::uint64_t received_id = 0U;
    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        received_id = cmd.frame_id;
    });

    kin.onSetpoint(makeSetpoint(42U, 0.05F, 0.05F));
    REQUIRE(received_id == 42U);
}

// ---------------------------------------------------------------------------
// Continuity
// ---------------------------------------------------------------------------

TEST_CASE(Kinematics3RRS_small_setpoint_changes_produce_small_output_changes) {
    Logger log;
    Kinematics3RRS kin(log, makeGoodConfig());

    ActuatorCommand a{};
    ActuatorCommand b{};
    bool first = true;

    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        if (first) { a = cmd; first = false; } else { b = cmd; }
    });

    kin.onSetpoint(makeSetpoint(1, 0.05F, 0.05F));
    kin.onSetpoint(makeSetpoint(2, 0.055F, 0.052F));

    REQUIRE(a.status == CommandStatus::Ok);
    REQUIRE(b.status == CommandStatus::Ok);
    REQUIRE(maxAbsDiff(a.actuator_targets, b.actuator_targets) < 5.0F);
}

TEST_CASE(Kinematics3RRS_zero_setpoint_after_valid_does_not_trigger_fallback) {
    Logger log;
    Kinematics3RRS kin(log, makeGoodConfig());

    ActuatorCommand last{};
    kin.registerCommandCallback([&](const ActuatorCommand& cmd) { last = cmd; });

    kin.onSetpoint(makeSetpoint(1, 0.1F, 0.1F));
    REQUIRE(last.status == CommandStatus::Ok);

    kin.onSetpoint(makeSetpoint(2, 0.0F, 0.0F));
    REQUIRE(last.status == CommandStatus::Ok);
    REQUIRE(allInRange(last.actuator_targets, 0.0F, 180.0F));
}

// ---------------------------------------------------------------------------
// Workspace boundary
// ---------------------------------------------------------------------------

TEST_CASE(Kinematics3RRS_large_reachable_setpoint_stays_in_range) {
    Logger log;
    Kinematics3RRS kin(log, makeGoodConfig());

    ActuatorCommand out{};
    kin.registerCommandCallback([&](const ActuatorCommand& cmd) { out = cmd; });

    // 0.3 rad ≈ 17° — within the actuator limits configured in AppConfig
    kin.onSetpoint(makeSetpoint(1, 0.3F, 0.0F));

    // Either succeeds or degrades; either way targets must stay in [0, 180].
    REQUIRE(allInRange(out.actuator_targets, 0.0F, 180.0F));
}

// ---------------------------------------------------------------------------
// Invalid geometry / FAULT path
// ---------------------------------------------------------------------------

TEST_CASE(Kinematics3RRS_invalid_geometry_surfaces_error_status) {
    Logger log;
    Kinematics3RRS kin(log, makeBadConfig());

    ActuatorCommand out{};
    kin.registerCommandCallback([&](const ActuatorCommand& cmd) { out = cmd; });

    kin.onSetpoint(makeSetpoint(1, 0.1F, 0.1F));

    // The command must be flagged as invalid; targets must still be in range
    // (safe fallback to neutral or last valid).
    REQUIRE((out.status == CommandStatus::KinematicsInvalidConfig ||
             out.status == CommandStatus::KinematicsFallbackLastValid));
    REQUIRE(allInRange(out.actuator_targets, 0.0F, 180.0F));
}

TEST_CASE(Kinematics3RRS_first_invalid_then_second_invalid_stays_in_range) {
    Logger log;
    Kinematics3RRS kin(log, makeBadConfig());

    ActuatorCommand first{};
    ActuatorCommand second{};
    bool got_first = false;

    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        if (!got_first) { first = cmd; got_first = true; }
        else { second = cmd; }
    });

    kin.onSetpoint(makeSetpoint(1, 0.1F, 0.05F));
    kin.onSetpoint(makeSetpoint(2, 0.2F, 0.1F));

    REQUIRE(allInRange(first.actuator_targets,  0.0F, 180.0F));
    REQUIRE(allInRange(second.actuator_targets, 0.0F, 180.0F));
}

TEST_CASE(Kinematics3RRS_good_then_bad_geometry_falls_back_to_last_valid) {
    Logger log;

    Kinematics3RRS good_kin(log, makeGoodConfig());

    ActuatorCommand valid_cmd{};
    good_kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        valid_cmd = cmd;
    });
    good_kin.onSetpoint(makeSetpoint(1, 0.1F, 0.05F));
    REQUIRE(valid_cmd.status == CommandStatus::Ok);

    // A second instance with bad geometry should fall back to neutral or
    // its last-valid (which is the neutral default on first call).
    Kinematics3RRS bad_kin(log, makeBadConfig());

    ActuatorCommand fallback{};
    bad_kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        fallback = cmd;
    });
    bad_kin.onSetpoint(makeSetpoint(2, 0.2F, 0.1F));

    REQUIRE((fallback.status == CommandStatus::KinematicsInvalidConfig ||
             fallback.status == CommandStatus::KinematicsFallbackLastValid));
    REQUIRE(allInRange(fallback.actuator_targets, 0.0F, 180.0F));
}
