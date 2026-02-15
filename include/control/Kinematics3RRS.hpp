#pragma once

#include <array>
#include <functional>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar { class Logger; }
namespace solar {

/**
 * Kinematics3RRS
 *
 * Maps PlatformSetpoint (tilt/pan) to ActuatorCommand (3 actuator targets).
 *
 * This implementation uses a configurable linearised mapping:
 *   actuator_targets = neutral + A * [tilt, pan]^T
 *
 * Why this is A1-acceptable now:
 * - Deterministic and testable
 * - Parameterised by geometry/calibration constants
 * - Keeps kinematics separated from control and actuation (SOLID)
 *
 * Later, you can replace A with full geometric kinematics without changing interfaces.
 */
class Kinematics3RRS {
public:
    using CommandCallback = std::function<void(const ActuatorCommand&)>;

    struct Config {
        // Neutral actuator targets (e.g., servo angles or lengths)
        std::array<float, 3> neutral{0.0f, 0.0f, 0.0f};

        // 3x2 mapping matrix A:
        // [a00 a01]   -> actuator 0 effect from [tilt, pan]
        // [a10 a11]   -> actuator 1
        // [a20 a21]   -> actuator 2
        std::array<std::array<float, 2>, 3> A{{
            { 1.0f,  0.0f },
            { 0.0f,  1.0f },
            { -1.0f, -1.0f }
        }};

        // Output limits
        std::array<float, 3> min_out{-1.0f, -1.0f, -1.0f};
        std::array<float, 3> max_out{ 1.0f,  1.0f,  1.0f};
    };

    Kinematics3RRS(Logger& log, Config cfg);

    Kinematics3RRS(const Kinematics3RRS&) = delete;
    Kinematics3RRS& operator=(const Kinematics3RRS&) = delete;

    void registerCommandCallback(CommandCallback cb);

    // Process one setpoint (event-driven call)
    void onSetpoint(const PlatformSetpoint& sp);

    Config config() const;

private:
    Logger& log_;
    Config cfg_;
    CommandCallback cmdCb_{};
};

} // namespace solar