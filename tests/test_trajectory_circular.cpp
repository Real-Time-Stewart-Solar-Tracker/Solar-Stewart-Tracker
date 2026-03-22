#include "test_common.hpp"

#include "common/Logger.hpp"
#include "control/Kinematics3RRS.hpp"

#include <atomic>
#include <cmath>
#include <vector>

using solar::ActuatorCommand;
using solar::Kinematics3RRS;
using solar::Logger;
using solar::PlatformSetpoint;

static bool finiteDegs(const ActuatorCommand& cmd) {
    for (int i = 0; i < 3; ++i) {
        const float d = cmd.actuator_targets[static_cast<std::size_t>(i)];
        if (!(std::isfinite(static_cast<double>(d)) && d >= 0.0f && d <= 180.0f)) return false;
    }
    return true;
}

TEST(Trajectory_CircularSetpoints_ProduceValidServoOutputs) {
    Logger log;
    Kinematics3RRS kin(log, Kinematics3RRS::Config{});

    std::atomic<int> count{0};
    ActuatorCommand last{};

    kin.registerCommandCallback([&](const ActuatorCommand& cmd) {
        last = cmd;
        count.fetch_add(1);
    });

    const int N = 60;
    const float A = 0.10f; // amplitude in radians (small, safe)

    for (int k = 0; k < N; ++k) {
       constexpr float kPi = 3.14159265358979323846f;
    const float theta = (2.0f * kPi) * (static_cast<float>(k) / static_cast<float>(N));
        PlatformSetpoint sp{};
        sp.frame_id = static_cast<uint64_t>(k + 1);
        sp.tilt_rad = A * std::sin(theta);
        sp.pan_rad  = A * std::cos(theta);

        kin.onSetpoint(sp);

        REQUIRE(finiteDegs(last));
    }

    REQUIRE(count.load() == N);
}