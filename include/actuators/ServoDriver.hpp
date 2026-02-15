#pragma once

#include <array>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar { class Logger; }
namespace solar {

/**
 * ServoDriver
 *
 * Hardware output layer.
 *
 * Responsibilities:
 * - Convert ActuatorCommand targets to hardware-specific signals (PWM, etc.)
 * - Provide a safe, idempotent start/stop lifecycle
 *
 * For Windows/dev builds, this can simply log outputs.
 * For Raspberry Pi, implement actual servo output (e.g., pigpio / PCA9685).
 */
class ServoDriver {
public:
    struct Config {
        // Driver will clamp to these just in case
        std::array<float, 3> min_out{-1.0f, -1.0f, -1.0f};
        std::array<float, 3> max_out{ 1.0f,  1.0f,  1.0f};
    };

    ServoDriver(Logger& log, Config cfg);

    ServoDriver(const ServoDriver&) = delete;
    ServoDriver& operator=(const ServoDriver&) = delete;

    ~ServoDriver();

    bool start();
    void stop();

    void apply(const ActuatorCommand& cmd);

private:
    Logger& log_;
    Config cfg_;
    bool running_{false};
};

} // namespace solar