#pragma once

#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>

namespace solar {

/**
 * @brief 3RRS inverse kinematics producing per-leg *servo degrees*.
 *  - yaw fixed to 0
 *  - input setpoint: pitch/roll (radians) via PlatformSetpoint
 *  - output command: servo degrees [0..180] in ActuatorCommand::actuator_targets
 *  - continuity via per-leg previous solution branch (q_prev_)
 *
 * Failure handling:
 * - invalid static geometry config is surfaced via ActuatorCommand::status
 * - per-leg degenerate / unreachable pose cases are surfaced via
 *   ActuatorCommand::status rather than being silently hidden
 * - last valid servo outputs are still available as a bounded fail-safe
 */
class Kinematics3RRS {
public:
    using CommandCallback = std::function<void(const ActuatorCommand&)>;

    struct Config {
        // Geometry (meters)
        float base_radius_m{0.20f};
        float platform_radius_m{0.12f};
        float home_height_m{0.18f};
        float horn_length_m{0.10f};
        float rod_length_m{0.18f};

        // Layout angles (degrees)
        std::array<float, 3> base_theta_deg{0.f, 120.f, 240.f};
        std::array<float, 3> plat_theta_deg{0.f, 120.f, 240.f};

        // Servo mapping (degrees)
        std::array<float, 3> servo_neutral_deg{90.f, 90.f, 90.f};
        std::array<int,   3> servo_dir{-1, -1, -1};
    };

    Kinematics3RRS(Logger& log, Config cfg);

    void registerCommandCallback(CommandCallback cb);
    Config config() const;

    void onSetpoint(const PlatformSetpoint& sp);

private:
    void computeIK_(const PlatformSetpoint& sp);
    void emitCommand_(const ActuatorCommand& cmd);

    // Continuity state (mechanism angles, radians)
    std::array<float, 3> q_prev_{0.f, 0.f, 0.f};

    // Fallback output (servo degrees)
    std::array<float, 3> last_valid_deg_{90.f, 90.f, 90.f};

    Logger& log_;
    Config cfg_;

    mutable std::mutex cbMtx_;
    CommandCallback cmdCb_{};
};

} // namespace solar