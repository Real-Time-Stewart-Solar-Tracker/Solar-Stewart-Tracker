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
    if (running_) return true;
    running_ = true;
    log_.info("ServoDriver started");
    return true;
}

void ServoDriver::stop() {
    if (!running_) return;
    running_ = false;
    log_.info("ServoDriver stopped");
}

void ServoDriver::apply(const ActuatorCommand& cmd) {
    if (!running_) {
        log_.warn("ServoDriver: apply called while stopped");
        return;
    }

    // Clamp again for safety and log the outputs
    float a0 = std::clamp(cmd.actuator_targets[0], cfg_.min_out[0], cfg_.max_out[0]);
    float a1 = std::clamp(cmd.actuator_targets[1], cfg_.min_out[1], cfg_.max_out[1]);
    float a2 = std::clamp(cmd.actuator_targets[2], cfg_.min_out[2], cfg_.max_out[2]);

    log_.info("ServoDriver apply: [" +
              std::to_string(a0) + ", " +
              std::to_string(a1) + ", " +
              std::to_string(a2) + "]");
}

} // namespace solar