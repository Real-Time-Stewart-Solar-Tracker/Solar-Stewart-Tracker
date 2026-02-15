#include "actuators/ActuatorManager.hpp"

#include <algorithm>  // std::clamp
#include <cmath>      // std::fabs
#include <utility>    // std::move

namespace solar {

ActuatorManager::ActuatorManager(Logger& log, Config cfg)
    : log_(log),
      cfg_(cfg),
      safeCb_(),
      lastOut_{0.0f, 0.0f, 0.0f},
      hasLast_(false) {}

void ActuatorManager::registerSafeCommandCallback(SafeCommandCallback cb) {
    safeCb_ = std::move(cb);
}

ActuatorManager::Config ActuatorManager::config() const {
    return cfg_;
}

void ActuatorManager::onCommand(const ActuatorCommand& cmd) {
    // Copy input command so we preserve any timestamps / metadata
    ActuatorCommand safe = cmd;

    for (std::size_t idx = 0; idx < 3; ++idx) {
        float desired = cmd.actuator_targets[idx];

        // Clamp to safe bounds
        desired = std::clamp(desired, cfg_.min_out[idx], cfg_.max_out[idx]);

        // Rate limit (max step per update)
        float out = desired;
        if (hasLast_) {
            const float prev  = lastOut_[idx];
            const float delta = desired - prev;
            const float step  = cfg_.max_step[idx];

            if (std::fabs(delta) > step) {
                out = prev + (delta > 0.0f ? step : -step);
            }
        }

        safe.actuator_targets[idx] = out;
        lastOut_[idx] = out;
    }

    hasLast_ = true;

    // Emit safe command to downstream driver (if registered)
    if (safeCb_) {
        safeCb_(safe);
    }
}

} // namespace solar