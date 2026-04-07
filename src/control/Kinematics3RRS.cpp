#include "control/Kinematics3RRS.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>

namespace solar {

// Internal angle conversion constants.
static constexpr float PI = 3.14159265358979323846f;

static float deg2rad(const float d) { return d * PI / 180.0f; }
static float rad2deg(const float r) { return r * 180.0f / PI; }

static float wrapAngle(float x) {
    const float two_pi = 2.0f * PI;
    x = std::fmod(x + PI, two_pi);
    if (x < 0.0f) x += two_pi;
    return x - PI;
}

static float chooseClosest(const float q1, const float q2, const float q_prev) {
    const float w1 = wrapAngle(q1);
    const float w2 = wrapAngle(q2);
    const float wp = wrapAngle(q_prev);
    return (std::fabs(w1 - wp) <= std::fabs(w2 - wp)) ? w1 : w2;
}

static float otherBranch(const float q1, const float q2, const float q_cur) {
    const float w1 = wrapAngle(q1);
    const float w2 = wrapAngle(q2);
    const float wc = wrapAngle(q_cur);
    return (std::fabs(wc - w1) < std::fabs(wc - w2)) ? w2 : w1;
}

Kinematics3RRS::Kinematics3RRS(Logger& log, Config cfg)
    : log_(log), cfg_(std::move(cfg)) {
    last_valid_deg_ = cfg_.servo_neutral_deg;
}

void Kinematics3RRS::registerCommandCallback(CommandCallback cb) {
    std::lock_guard<std::mutex> lk(cbMtx_);
    cmdCb_ = std::move(cb);
}

Kinematics3RRS::Config Kinematics3RRS::config() const { return cfg_; }

void Kinematics3RRS::onSetpoint(const PlatformSetpoint& sp) {
    // Acquire ik_mtx_ before entering the solver so that concurrent callers
    // (automatic control thread, pot callback, GUI dispatcher) do not race on
    // q_prev_ or last_valid_deg_.
    std::lock_guard<std::mutex> lk(ik_mtx_);
    computeIK_(sp);
}

void Kinematics3RRS::emitCommand_(const ActuatorCommand& cmd) {
    CommandCallback cb;
    {
        std::lock_guard<std::mutex> lk(cbMtx_);
        cb = cmdCb_;
    }
    if (cb) cb(cmd);
}

void Kinematics3RRS::computeIK_(const PlatformSetpoint& sp) {
    // Must be called with ik_mtx_ already held.
    //
    // Rotation convention: ZYX intrinsic Euler.
    //   roll  (sp.tilt_rad) — rotation about the rig Y axis
    //   pitch (sp.pan_rad)  — rotation about the rig X axis
    //
    // The resulting 3×3 rotation matrix R maps platform-local leg anchor
    // positions into the base frame. The IK equations then solve for the
    // horn angle q for each leg independently.
    //
    // Reference: Dasgupta & Mruthyunjaya, "The Stewart platform manipulator:
    // a review," Mechanism and Machine Theory 35 (2000) 15–40.
    const float roll  = sp.tilt_rad;
    const float pitch = sp.pan_rad;

    const float cp  = std::cos(pitch);
    const float spc = std::sin(pitch);
    const float cr  = std::cos(roll);
    const float sr  = std::sin(roll);

    float R[3][3];
    R[0][0] =  cp;      R[0][1] = spc * sr;  R[0][2] = spc * cr;
    R[1][0] =  0.f;     R[1][1] = cr;         R[1][2] = -sr;
    R[2][0] = -spc;     R[2][1] = cp * sr;    R[2][2] =  cp * cr;

    const float h  = cfg_.home_height_m;
    const float Rb = cfg_.base_radius_m;
    const float Rp = cfg_.platform_radius_m;
    const float L1 = cfg_.horn_length_m;
    const float L2 = cfg_.rod_length_m;

    ActuatorCommand cmd{};
    cmd.frame_id  = sp.frame_id;
    cmd.status    = CommandStatus::Ok;
    cmd.t_actuate = std::chrono::steady_clock::now();

    // Reject obviously invalid geometry before entering the per-leg loop.
    if (L1 <= 0.0f || L2 <= 0.0f || Rb <= 0.0f || Rp <= 0.0f || h <= 0.0f) {
        log_.error("Kinematics3RRS: invalid geometry config for frame " +
                   std::to_string(sp.frame_id));
        cmd.status = CommandStatus::KinematicsInvalidConfig;
        for (int i = 0; i < 3; ++i) {
            cmd.actuator_targets[static_cast<std::size_t>(i)] =
                last_valid_deg_[static_cast<std::size_t>(i)];
        }
        emitCommand_(cmd);
        return;
    }

    bool usedFallback = false;

    for (int i = 0; i < 3; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i);

        const float thb = deg2rad(cfg_.base_theta_deg[idx]);
        const float ctb = std::cos(thb);
        const float stb = std::sin(thb);

        // Base anchor position in the global frame.
        const float Bx = Rb * ctb;
        const float By = Rb * stb;

        // Platform anchor position in the platform-local frame.
        const float thp  = deg2rad(cfg_.plat_theta_deg[idx]);
        const float Px_l = Rp * std::cos(thp);
        const float Py_l = Rp * std::sin(thp);

        // Rotate platform anchor into the global frame and apply home height.
        const float Px = R[0][0] * Px_l + R[0][1] * Py_l;
        const float Py = R[1][0] * Px_l + R[1][1] * Py_l;
        const float Pz = R[2][0] * Px_l + R[2][1] * Py_l + h;

        const float dx = Px - Bx;
        const float dy = Py - By;
        const float dz = Pz;

        // Project displacement into the leg's local radial/tangential axes.
        const float ex_x =  ctb;  const float ex_y =  stb;
        const float ey_x = -stb;  const float ey_y =  ctb;

        const float x     = dx * ex_x + dy * ex_y;
        const float yperp = dx * ey_x + dy * ey_y;
        const float z     = dz;

        const float Rxz = std::hypot(x, z);
        if (Rxz < 1e-6f) {
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
            usedFallback = true;
            continue;
        }

        // Cosine rule to find the horn angle from the leg geometry.
        const float C     = (x * x + yperp * yperp + z * z + L1 * L1 - L2 * L2)
                            / (2.0f * L1);
        const float ratio = C / Rxz;

        // Guard against unreachable configurations (acos domain).
        if (!std::isfinite(ratio) || ratio < -1.0f || ratio > 1.0f) {
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
            usedFallback = true;
            continue;
        }

        const float a     = std::acos(ratio);
        const float gamma = std::atan2(z, x);

        const float qA = gamma + a;
        const float qB = gamma - a;

        float q = chooseClosest(qA, qB, q_prev_[idx]);

        // Prefer the branch where the horn points upward (cos(q) >= 0).
        if (std::cos(q) < 0.0f) {
            q = otherBranch(qA, qB, q);
        }

        if (!std::isfinite(q)) {
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
            usedFallback = true;
            continue;
        }

        q_prev_[idx] = q;

        // Convert horn angle to servo output degree, applying calibration
        // offset and sign convention.
        float servo_deg = cfg_.servo_neutral_deg[idx]
                        + static_cast<float>(cfg_.servo_dir[idx]) * rad2deg(q);

        if (!std::isfinite(servo_deg)) {
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
            usedFallback = true;
            continue;
        }

        // Hard clamp to the physical servo travel range. Integer rounding
        // removes sub-degree noise that the PCA9685 cannot resolve.
        servo_deg = std::clamp(servo_deg, 0.0f, 180.0f);
        servo_deg = static_cast<float>(std::lround(servo_deg));

        last_valid_deg_[idx] = servo_deg;
        cmd.actuator_targets[idx] = servo_deg;
    }

    if (usedFallback) {
        cmd.status = CommandStatus::KinematicsFallbackLastValid;
    }

    emitCommand_(cmd);
}

} // namespace solar
