#pragma once

/**
 * @file Kinematics3RRS.hpp
 * @brief 3-RRS platform kinematics stage converting setpoints into servo commands.
 *
 * This class sits between the controller and the actuator stage. It receives a
 * platform setpoint, computes servo angles via inverse kinematics, and emits an
 * @ref ActuatorCommand via callback.
 *
 * ## Thread safety
 *
 * @c onSetpoint() is internally thread-safe. Callers from multiple threads
 * (the automatic control thread, the pot-manual ADS1115 callback, and the
 * GUI manual dispatcher thread) may all call @c onSetpoint() concurrently.
 * An internal mutex protects the mutable solver state (@c q_prev_ and
 * @c last_valid_deg_) so no external synchronisation is required.
 *
 * ## Rotation convention
 *
 * The IK solver uses a ZYX intrinsic Euler rotation. The tilt setpoint maps
 * to a rotation about the Y axis (roll in the rig frame) and the pan setpoint
 * maps to a rotation about the X axis (pitch in the rig frame). The resulting
 * 3x3 rotation matrix R maps platform-local leg anchor positions into the
 * global base frame before the inverse-kinematics equations are applied.
 *
 * Reference: standard 3-RRS parallel mechanism inverse kinematics as described
 * in Dasgupta & Mruthyunjaya, "The Stewart platform manipulator: a review,"
 * Mechanism and Machine Theory 35 (2000) 15–40.
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>

namespace solar {

/**
 * @brief Kinematics stage for the 3-RRS parallel platform.
 */
class Kinematics3RRS {
public:
    /**
     * @brief Callback receiving one actuator command per solved setpoint.
     */
    using CommandCallback = std::function<void(const ActuatorCommand& cmd)>;

    /**
     * @brief Geometry and calibration parameters for the 3-RRS mechanism.
     */
    struct Config {
        float base_radius_m{0.20f};     ///< Base platform radius in metres.
        float platform_radius_m{0.12f}; ///< Moving platform radius in metres.
        float home_height_m{0.18f};     ///< Neutral platform height above base in metres.
        float horn_length_m{0.10f};     ///< Servo horn length (L1) in metres.
        float rod_length_m{0.18f};      ///< Connecting rod length (L2) in metres.

        std::array<float, 3> base_theta_deg{0.f, 120.f, 240.f}; ///< Base anchor angles (°).
        std::array<float, 3> plat_theta_deg{0.f, 120.f, 240.f}; ///< Platform anchor angles (°).

        std::array<float, 3> servo_neutral_deg{90.f, 90.f, 90.f}; ///< Servo neutral calibration (°).
        std::array<int, 3>   servo_dir{-1, -1, -1};               ///< Per-servo sign convention.
    };

    /**
     * @brief Construct the kinematics stage.
     *
     * @param log Shared logger.
     * @param cfg Kinematics configuration.
     */
    Kinematics3RRS(Logger& log, Config cfg);

    Kinematics3RRS(const Kinematics3RRS&) = delete;
    Kinematics3RRS& operator=(const Kinematics3RRS&) = delete;

    /**
     * @brief Register the output command callback.
     *
     * @param cb Callback receiving computed actuator commands.
     */
    void registerCommandCallback(CommandCallback cb);

    /**
     * @brief Return the current configuration.
     */
    Config config() const;

    /**
     * @brief Process one platform setpoint.
     *
     * Thread-safe — may be called concurrently from multiple threads.
     *
     * @param sp Desired platform setpoint (tilt/pan in radians).
     */
    void onSetpoint(const PlatformSetpoint& sp);

private:
    void computeIK_(const PlatformSetpoint& sp);
    void emitCommand_(const ActuatorCommand& cmd);

    // Mutable solver state — protected by ik_mtx_ so onSetpoint() is
    // thread-safe without requiring external synchronisation from callers.
    mutable std::mutex           ik_mtx_;
    std::array<float, 3>         q_prev_{0.f, 0.f, 0.f};
    std::array<float, 3>         last_valid_deg_{90.f, 90.f, 90.f};

    Logger& log_;
    Config  cfg_;

    mutable std::mutex cbMtx_;
    CommandCallback    cmdCb_{};
};

} // namespace solar
