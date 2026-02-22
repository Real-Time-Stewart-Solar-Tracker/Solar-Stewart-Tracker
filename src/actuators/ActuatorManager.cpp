#include "actuators/ActuatorManager.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <utility>

namespace solar {

ActuatorManager::ActuatorManager(Logger& log, Config cfg)
    : log_(log),
      cfg_(cfg),
      safeCb_(),
      lastOut_{0.0f, 0.0f, 0.0f},
      hasLast_(false) {}

void ActuatorManager::registerSafeCommandCallback(SafeCommandCallback cb) {
    std::lock_guard<std::mutex> lk(cbMtx_);
    safeCb_ = std::move(cb);
}

ActuatorManager::Config ActuatorManager::config() const {
    return cfg_;
}

void ActuatorManager::onCommand(const ActuatorCommand& cmd) {
    // Protect internal state (lastOut_/hasLast_)
    std::lock_guard<std::mutex> lk(mtx_);

    ActuatorCommand safe = cmd;

    for (std::size_t idx = 0; idx < 3; ++idx) {
        float desired = cmd.actuator_targets[idx];

        desired = std::clamp(desired, cfg_.min_out[idx], cfg_.max_out[idx]);

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

    SafeCommandCallback cb;
    {
        std::lock_guard<std::mutex> lk2(cbMtx_);
        cb = safeCb_;
    }
    if (cb) cb(safe);
}

} // namespace solar