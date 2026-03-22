#include "test_common.hpp"

#include "actuators/ActuatorManager.hpp"
#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <array>
#include <atomic>
#include <optional>

namespace {

solar::ActuatorCommand makeCommand(float a0, float a1, float a2) {
    solar::ActuatorCommand cmd{};
    cmd.frame_id = 1;
    cmd.actuator_targets = {a0, a1, a2};
    return cmd;
}

} // namespace

TEST(ActuatorManager_first_command_is_saturated_without_history_limit) {
    solar::Logger log;

    solar::ActuatorManager::Config cfg{};
    cfg.min_out  = {-1.0f, -1.0f, -1.0f};
    cfg.max_out  = { 1.0f,  1.0f,  1.0f};
    cfg.max_step = {0.02f, 0.02f, 0.02f};

    solar::ActuatorManager mgr(log, cfg);

    std::optional<solar::ActuatorCommand> out;
    mgr.registerSafeCommandCallback([&](const solar::ActuatorCommand& cmd) {
        out = cmd;
    });

    mgr.onCommand(makeCommand(5.0f, -5.0f, 0.5f));

    REQUIRE(out.has_value());
    REQUIRE_NEAR(out->actuator_targets[0],  1.0f, 1e-6f);
    REQUIRE_NEAR(out->actuator_targets[1], -1.0f, 1e-6f);
    REQUIRE_NEAR(out->actuator_targets[2],  0.5f, 1e-6f);
}

TEST(ActuatorManager_subsequent_commands_are_rate_limited_per_channel) {
    solar::Logger log;

    solar::ActuatorManager::Config cfg{};
    cfg.min_out  = {-1.0f, -1.0f, -1.0f};
    cfg.max_out  = { 1.0f,  1.0f,  1.0f};
    cfg.max_step = {0.02f, 0.05f, 0.10f};

    solar::ActuatorManager mgr(log, cfg);

    std::optional<solar::ActuatorCommand> out;
    mgr.registerSafeCommandCallback([&](const solar::ActuatorCommand& cmd) {
        out = cmd;
    });

    mgr.onCommand(makeCommand(0.0f, 0.0f, 0.0f));
    REQUIRE(out.has_value());

    mgr.onCommand(makeCommand(1.0f, -1.0f, 1.0f));
    REQUIRE(out.has_value());

    REQUIRE_NEAR(out->actuator_targets[0],  0.02f, 1e-6f);
    REQUIRE_NEAR(out->actuator_targets[1], -0.05f, 1e-6f);
    REQUIRE_NEAR(out->actuator_targets[2],  0.10f, 1e-6f);
}

TEST(ActuatorManager_saturation_happens_before_rate_limit) {
    solar::Logger log;

    solar::ActuatorManager::Config cfg{};
    cfg.min_out  = {0.0f, 0.0f, 0.0f};
    cfg.max_out  = {180.0f, 180.0f, 180.0f};
    cfg.max_step = {10.0f, 10.0f, 10.0f};

    solar::ActuatorManager mgr(log, cfg);

    std::optional<solar::ActuatorCommand> out;
    mgr.registerSafeCommandCallback([&](const solar::ActuatorCommand& cmd) {
        out = cmd;
    });

    mgr.onCommand(makeCommand(90.0f, 90.0f, 90.0f));
    REQUIRE(out.has_value());
    REQUIRE_NEAR(out->actuator_targets[0], 90.0f, 1e-6f);

    mgr.onCommand(makeCommand(999.0f, -999.0f, 200.0f));
    REQUIRE(out.has_value());

    REQUIRE_NEAR(out->actuator_targets[0], 100.0f, 1e-6f);
    REQUIRE_NEAR(out->actuator_targets[1],  80.0f, 1e-6f);
    REQUIRE_NEAR(out->actuator_targets[2], 100.0f, 1e-6f);
}

TEST(ActuatorManager_runtime_like_large_max_step_effectively_disables_slew_limit) {
    solar::Logger log;

    solar::ActuatorManager::Config cfg{};
    cfg.min_out  = {0.0f, 0.0f, 0.0f};
    cfg.max_out  = {180.0f, 180.0f, 180.0f};
    cfg.max_step = {999.0f, 999.0f, 999.0f};

    solar::ActuatorManager mgr(log, cfg);

    std::optional<solar::ActuatorCommand> out;
    mgr.registerSafeCommandCallback([&](const solar::ActuatorCommand& cmd) {
        out = cmd;
    });

    mgr.onCommand(makeCommand(10.0f, 20.0f, 30.0f));
    REQUIRE(out.has_value());

    mgr.onCommand(makeCommand(170.0f, 160.0f, 150.0f));
    REQUIRE(out.has_value());

    REQUIRE_NEAR(out->actuator_targets[0], 170.0f, 1e-6f);
    REQUIRE_NEAR(out->actuator_targets[1], 160.0f, 1e-6f);
    REQUIRE_NEAR(out->actuator_targets[2], 150.0f, 1e-6f);
}

TEST(ActuatorManager_callback_can_reenter_onCommand_without_deadlock) {
    solar::Logger log;

    solar::ActuatorManager::Config cfg{};
    cfg.min_out  = {0.0f, 0.0f, 0.0f};
    cfg.max_out  = {180.0f, 180.0f, 180.0f};
    cfg.max_step = {10.0f, 10.0f, 10.0f};

    solar::ActuatorManager mgr(log, cfg);

    std::atomic<int> callbackCount{0};
    std::array<float, 3> firstOut{0.0f, 0.0f, 0.0f};
    std::array<float, 3> secondOut{0.0f, 0.0f, 0.0f};

    mgr.registerSafeCommandCallback([&](const solar::ActuatorCommand& cmd) {
        const int n = ++callbackCount;

        if (n == 1) {
            firstOut = cmd.actuator_targets;

            // Re-enter onCommand from inside the callback.
            mgr.onCommand(makeCommand(100.0f, 100.0f, 100.0f));
        } else if (n == 2) {
            secondOut = cmd.actuator_targets;
        }
    });

    mgr.onCommand(makeCommand(90.0f, 90.0f, 90.0f));

    REQUIRE(callbackCount.load() == 2);
    REQUIRE_NEAR(firstOut[0], 90.0f, 1e-6f);
    REQUIRE_NEAR(firstOut[1], 90.0f, 1e-6f);
    REQUIRE_NEAR(firstOut[2], 90.0f, 1e-6f);

    // Second command is rate-limited from the first output.
    REQUIRE_NEAR(secondOut[0], 100.0f, 1e-6f);
    REQUIRE_NEAR(secondOut[1], 100.0f, 1e-6f);
    REQUIRE_NEAR(secondOut[2], 100.0f, 1e-6f);
}