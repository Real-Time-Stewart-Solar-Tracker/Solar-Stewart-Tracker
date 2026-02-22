#pragma once

#include <array>
#include <functional>
#include <mutex>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * @brief Inverse kinematics solver for a 3-RRS platform.
 *
 * Converts platform tilt/pan setpoints into three actuator targets
 * (servo angles) using deterministic 3-RRS inverse kinematics.
 *
 * Pure computation layer:
 * - No hardware access
 * - No internal threads
 * - Emits ActuatorCommand via callback
 */
class Kinematics3RRS {
public:
    /// @brief Geometric and calibration parameters for the 3-RRS mechanism.
    struct Config {
        /// @brief Base radius (meters).
        float base_radius_m{0.05f};

        /// @brief Platform radius (meters).
        float platform_radius_m{0.045f};

        /// @brief Home platform height (meters).
        float home_height_m{0.15f};

        /// @brief Servo horn length (meters).
        float horn_length_m{0.10f};

        /// @brief Connecting rod length (meters).
        float rod_length_m{0.12f};

        /// @brief Base joint angular positions (degrees).
        std::array<float,3> base_theta_deg{{0.f, 120.f, 240.f}};

        /// @brief Platform joint angular positions (degrees).
        std::array<float,3> plat_theta_deg{{0.f, 120.f, 240.f}};

        /// @brief Servo neutral angle offsets (radians).
        std::array<float,3> servo_neutral_rad{{0.f,0.f,0.f}};

        /// @brief Servo direction multipliers (+1 or -1).
        std::array<int,3> servo_dir{{1,1,1}};
    };

    /// @brief Callback delivering computed actuator commands.
    using CommandCallback = std::function<void(const ActuatorCommand&)>;

    /// @brief Construct solver with logger and configuration.
    Kinematics3RRS(Logger& log, Config cfg);

    Kinematics3RRS(const Kinematics3RRS&) = delete;
    Kinematics3RRS& operator=(const Kinematics3RRS&) = delete;

    /// @brief Register callback to receive computed actuator commands.
    void registerCommandCallback(CommandCallback cb);

    /// @brief Get current configuration.
    Config config() const;

    /// @brief Process a platform setpoint and compute actuator targets.
    void onSetpoint(const PlatformSetpoint& sp);

private:
    /// @brief Internal inverse kinematics computation.
    void computeIK_(const PlatformSetpoint& sp);

    Logger& log_;
    Config cfg_;

    /// @brief Mutex protecting callback registration and invocation.
    mutable std::mutex cbMtx_;

    /// @brief Registered actuator command callback (may be empty).
    CommandCallback cmdCb_{};

    /// @brief Previous actuator solution used for branch continuity.
    std::array<float,3> q_prev_{{0.f,0.f,0.f}};
};

} // namespace solar