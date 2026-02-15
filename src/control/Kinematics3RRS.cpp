#include "control/Kinematics3RRS.hpp"
#include "common/Logger.hpp"
#include <algorithm>
#include <string>
#include <chrono>

namespace solar {

Kinematics3RRS::Kinematics3RRS(Logger& log, Config cfg)
    : log_(log), cfg_(cfg) {}

void Kinematics3RRS::registerCommandCallback(CommandCallback cb) {
    cmdCb_ = std::move(cb);
}

Kinematics3RRS::Config Kinematics3RRS::config() const {
    return cfg_;
}

void Kinematics3RRS::onSetpoint(const PlatformSetpoint& sp) {
    ActuatorCommand cmd;
    cmd.t_actuate = std::chrono::steady_clock::now();

    const float tilt = sp.tilt_rad;
    const float pan  = sp.pan_rad;

    // Compute: neutral + A * [tilt, pan]^T
    for (int i = 0; i < 3; ++i) {
        const float a_tilt = cfg_.A[static_cast<std::size_t>(i)][0];
        const float a_pan  = cfg_.A[static_cast<std::size_t>(i)][1];

        float out = cfg_.neutral[static_cast<std::size_t>(i)]
                  + a_tilt * tilt
                  + a_pan * pan;

        // Clamp per actuator
        out = std::clamp(out,
                         cfg_.min_out[static_cast<std::size_t>(i)],
                         cfg_.max_out[static_cast<std::size_t>(i)]);

        cmd.actuator_targets[static_cast<std::size_t>(i)] = out;
    }

    if (cmdCb_) {
        cmdCb_(cmd);
    }
}

} // namespace solar