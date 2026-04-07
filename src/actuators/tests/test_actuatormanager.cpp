/**
 * @file test_actuatormanager.cpp
 * @brief Unit tests for ActuatorManager.
 */

#include "actuators/ActuatorManager.hpp"
#include "src/tests/support/test_common.hpp"

#include <array>
#include <cmath>

using solar::ActuatorCommand;
using solar::ActuatorManager;
using solar::Logger;

namespace {

bool nearlyEqual(float a, float b, float eps = 1e-5F) {
    return std::fabs(a - b) <= eps;
}

} // namespace

TEST_CASE(ActuatorManager_first_command_is_saturated_without_history_limit) {
    Logger log;

    ActuatorManager::Config cfg{};
    cfg.min_out = {10.0F, 20.0F, 30.0F};
    cfg.max_out = {100.0F, 110.0F, 120.0F};
    cfg.max_step = {5.0F, 5.0F, 5.0F};

    ActuatorManager mgr(log, cfg);

    ActuatorCommand out{};
    mgr.registerSafeCommandCallback([&](const ActuatorCommand& cmd) {
        out = cmd;
    });

    ActuatorCommand in{};
    in.actuator_targets = {0.0F, 200.0F, 50.0F};

    mgr.onCommand(in);

    REQUIRE(nearlyEqual(out.actuator_targets[0], 10.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[1], 110.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[2], 50.0F));
}

TEST_CASE(ActuatorManager_subsequent_commands_are_rate_limited_per_channel) {
    Logger log;

    ActuatorManager::Config cfg{};
    cfg.min_out = {0.0F, 0.0F, 0.0F};
    cfg.max_out = {180.0F, 180.0F, 180.0F};
    cfg.max_step = {5.0F, 10.0F, 2.0F};

    ActuatorManager mgr(log, cfg);

    ActuatorCommand out{};
    mgr.registerSafeCommandCallback([&](const ActuatorCommand& cmd) {
        out = cmd;
    });

    ActuatorCommand a{};
    a.actuator_targets = {50.0F, 50.0F, 50.0F};
    mgr.onCommand(a);

    ActuatorCommand b{};
    b.actuator_targets = {100.0F, 90.0F, 60.0F};
    mgr.onCommand(b);

    REQUIRE(nearlyEqual(out.actuator_targets[0], 55.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[1], 60.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[2], 52.0F));
}

TEST_CASE(ActuatorManager_saturation_happens_before_history_update) {
    Logger log;

    ActuatorManager::Config cfg{};
    cfg.min_out = {0.0F, 0.0F, 0.0F};
    cfg.max_out = {100.0F, 100.0F, 100.0F};
    cfg.max_step = {50.0F, 50.0F, 50.0F};

    ActuatorManager mgr(log, cfg);

    ActuatorCommand out{};
    mgr.registerSafeCommandCallback([&](const ActuatorCommand& cmd) {
        out = cmd;
    });

    ActuatorCommand a{};
    a.actuator_targets = {200.0F, 200.0F, 200.0F};
    mgr.onCommand(a);

    REQUIRE(nearlyEqual(out.actuator_targets[0], 100.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[1], 100.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[2], 100.0F));

    ActuatorCommand b{};
    b.actuator_targets = {0.0F, 0.0F, 0.0F};
    mgr.onCommand(b);

    REQUIRE(nearlyEqual(out.actuator_targets[0], 50.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[1], 50.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[2], 50.0F));
}

TEST_CASE(ActuatorManager_resetHistory_disables_slew_limit_for_next_command) {
    Logger log;

    ActuatorManager::Config cfg{};
    cfg.min_out = {0.0F, 0.0F, 0.0F};
    cfg.max_out = {180.0F, 180.0F, 180.0F};
    cfg.max_step = {5.0F, 5.0F, 5.0F};

    ActuatorManager mgr(log, cfg);

    ActuatorCommand out{};
    mgr.registerSafeCommandCallback([&](const ActuatorCommand& cmd) {
        out = cmd;
    });

    ActuatorCommand a{};
    a.actuator_targets = {50.0F, 50.0F, 50.0F};
    mgr.onCommand(a);

    mgr.resetHistory();

    ActuatorCommand b{};
    b.actuator_targets = {100.0F, 120.0F, 140.0F};
    mgr.onCommand(b);

    REQUIRE(nearlyEqual(out.actuator_targets[0], 100.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[1], 120.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[2], 140.0F));
}

TEST_CASE(ActuatorManager_zero_or_negative_step_holds_previous_value) {
    Logger log;

    ActuatorManager::Config cfg{};
    cfg.min_out = {0.0F, 0.0F, 0.0F};
    cfg.max_out = {180.0F, 180.0F, 180.0F};
    cfg.max_step = {0.0F, -1.0F, 5.0F};

    ActuatorManager mgr(log, cfg);

    ActuatorCommand out{};
    mgr.registerSafeCommandCallback([&](const ActuatorCommand& cmd) {
        out = cmd;
    });

    ActuatorCommand a{};
    a.actuator_targets = {30.0F, 40.0F, 50.0F};
    mgr.onCommand(a);

    ActuatorCommand b{};
    b.actuator_targets = {100.0F, 100.0F, 100.0F};
    mgr.onCommand(b);

    REQUIRE(nearlyEqual(out.actuator_targets[0], 30.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[1], 40.0F));
    REQUIRE(nearlyEqual(out.actuator_targets[2], 55.0F));
}