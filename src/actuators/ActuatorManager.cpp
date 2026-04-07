#include "actuators/ActuatorManager.hpp"

#include <algorithm>
#include <cmath>

namespace solar {

ActuatorManager::ActuatorManager(Logger& log, Config cfg)
    : log_(log),
      cfg_(std::move(cfg)) {
}

void ActuatorManager::registerSafeCommandCallback(SafeCommandCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mtx_);
    cb_ = std::move(cb);
}

ActuatorManager::Config ActuatorManager::config() const {
    std::lock_guard<std::mutex> lock(cfg_mtx_);
    return cfg_;
}

void ActuatorManager::resetHistory() {
    std::lock_guard<std::mutex> lock(hist_mtx_);
    have_last_ = false;
    last_targets_ = {0.0F, 0.0F, 0.0F};
}

float ActuatorManager::clamp_(const float v, const float lo, const float hi) const {
    return std::max(lo, std::min(v, hi));
}

float ActuatorManager::limitStep_(const float target, const float prev, const float max_step) const {
    if (max_step <= 0.0F) {
        return prev;
    }

    const float delta = target - prev;
    if (std::fabs(delta) <= max_step) {
        return target;
    }

    return prev + ((delta > 0.0F) ? max_step : -max_step);
}

void ActuatorManager::onCommand(const ActuatorCommand& cmd) {
    // The actuator stage consumes already solved servo targets; it does not redo control math.
    Config cfg_copy;
    {
        std::lock_guard<std::mutex> lock(cfg_mtx_);
        cfg_copy = cfg_;
    }

    ActuatorCommand safe = cmd;

    // Clamp first so every downstream stage only sees safe actuator angles.
    for (std::size_t i = 0; i < 3U; ++i) {
        safe.actuator_targets[i] = clamp_(
            safe.actuator_targets[i],
            cfg_copy.min_out[i],
            cfg_copy.max_out[i]);
    }

    // Slew limiting keeps the three channels from jumping too far in one update.
    {
        std::lock_guard<std::mutex> lock(hist_mtx_);

        if (have_last_) {
            for (std::size_t i = 0; i < 3U; ++i) {
                safe.actuator_targets[i] = limitStep_(
                    safe.actuator_targets[i],
                    last_targets_[i],
                    cfg_copy.max_step[i]);

                // Clamp again defensively after step limiting.
                safe.actuator_targets[i] = clamp_(
                    safe.actuator_targets[i],
                    cfg_copy.min_out[i],
                    cfg_copy.max_out[i]);
            }
        }

        last_targets_ = safe.actuator_targets;
        have_last_ = true;
    }

    SafeCommandCallback cb;
    {
        std::lock_guard<std::mutex> lock(cb_mtx_);
        cb = cb_;
    }

    if (cb) {
        cb(safe);
    } else {
        log_.warn("ActuatorManager: no safe-command callback registered");
    }
}

} // namespace solar