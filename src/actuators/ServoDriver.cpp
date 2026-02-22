#include "actuators/ServoDriver.hpp"
#include "common/Logger.hpp"

#include <algorithm>
#include <string>

namespace solar {

ServoDriver::ServoDriver(Logger& log, Config cfg)
    : log_(log), cfg_(cfg) {}

ServoDriver::~ServoDriver() {
    stop();
}

bool ServoDriver::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true; // already running
    }

    warnedStopped_.store(false);
    applyCount_.store(0);
    log_.info("ServoDriver started");
    return true;
}

void ServoDriver::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return; // already stopped
    }
    log_.info("ServoDriver stopped");
}

void ServoDriver::apply(const ActuatorCommand& cmd) {
    if (!running_.load()) {
        // Warn once to avoid log spam
        bool expected = false;
        if (warnedStopped_.compare_exchange_strong(expected, true)) {
            log_.warn("ServoDriver: apply called while stopped");
        }
        return;
    }

    // Clamp again for safety
    const float a0 = std::clamp(cmd.actuator_targets[0], cfg_.min_out[0], cfg_.max_out[0]);
    const float a1 = std::clamp(cmd.actuator_targets[1], cfg_.min_out[1], cfg_.max_out[1]);
    const float a2 = std::clamp(cmd.actuator_targets[2], cfg_.min_out[2], cfg_.max_out[2]);

    // Rate-limited logging (avoid realtime jitter)
    const uint32_t n = applyCount_.fetch_add(1) + 1;
    if (cfg_.log_every_n > 0 && (n % cfg_.log_every_n) == 0) {
        log_.info("ServoDriver apply: [" +
                  std::to_string(a0) + ", " +
                  std::to_string(a1) + ", " +
                  std::to_string(a2) + "]");
    }

    // TODO (hardware): write to PCA9685 / PWM driver here
}

} // namespace solar