#include "control/Kinematics3RRS.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>

namespace solar {

static constexpr float PI = 3.14159265358979323846f;

static float deg2rad(float d) { return d * PI / 180.0f; }
static float rad2deg(float r) { return r * 180.0f / PI; }

static float wrapAngle(float x) {
    const float two_pi = 2.0f * PI;
    x = std::fmod(x + PI, two_pi);
    if (x < 0.0f) x += two_pi;
    return x - PI;
}

static float chooseClosest(float q1, float q2, float q_prev) {
    q1 = wrapAngle(q1);
    q2 = wrapAngle(q2);
    q_prev = wrapAngle(q_prev);
    return (std::fabs(q1 - q_prev) <= std::fabs(q2 - q_prev)) ? q1 : q2;
}

static float otherBranch(float q1, float q2, float q_cur) {
    q1 = wrapAngle(q1);
    q2 = wrapAngle(q2);
    q_cur = wrapAngle(q_cur);
    return (std::fabs(q_cur - q1) < std::fabs(q_cur - q2)) ? q2 : q1;
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

void Kinematics3RRS::onSetpoint(const PlatformSetpoint& sp) { computeIK_(sp); }

void Kinematics3RRS::emitCommand_(const ActuatorCommand& cmd) {
    CommandCallback cb;
    {
        std::lock_guard<std::mutex> lk(cbMtx_);
        cb = cmdCb_;
    }
    if (cb) cb(cmd);
}

void Kinematics3RRS::computeIK_(const PlatformSetpoint& sp) {
    const float roll  = sp.tilt_rad; // Rx
    const float pitch = sp.pan_rad;  // Ry

    // Rotation matrix R = Ry(pitch) * Rx(roll) (yaw fixed to 0)
    const float cp  = std::cos(pitch);
    const float spc = std::sin(pitch);
    const float cr  = std::cos(roll);
    const float sr  = std::sin(roll);

    float R[3][3];
    R[0][0] =  cp;
    R[0][1] =  spc * sr;
    R[0][2] =  spc * cr;

    R[1][0] =  0.f;
    R[1][1] =  cr;
    R[1][2] = -sr;

    R[2][0] = -spc;
    R[2][1] =  cp * sr;
    R[2][2] =  cp * cr;

    const float h  = cfg_.home_height_m;
    const float Rb = cfg_.base_radius_m;
    const float Rp = cfg_.platform_radius_m;
    const float L1 = cfg_.horn_length_m;
    const float L2 = cfg_.rod_length_m;

    ActuatorCommand cmd{};
    cmd.frame_id = sp.frame_id;
    cmd.status   = CommandStatus::Ok;

    // Static geometry validation.
    if (L1 <= 0.0f || L2 <= 0.0f || Rb <= 0.0f || Rp <= 0.0f || h <= 0.0f) {
        log_.error("Kinematics3RRS: invalid geometry config for frame " +
                   std::to_string(sp.frame_id));

        cmd.status = CommandStatus::KinematicsInvalidConfig;
        for (int i = 0; i < 3; ++i) {
            const std::size_t idx = static_cast<std::size_t>(i);
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
        }

        emitCommand_(cmd);
        return;
    }

    bool usedFallback = false;

    for (int i = 0; i < 3; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i);

        // Base joint B_i
        const float thb = deg2rad(cfg_.base_theta_deg[idx]);
        const float ctb = std::cos(thb);
        const float stb = std::sin(thb);

        const float Bx = Rb * ctb;
        const float By = Rb * stb;
        const float Bz = 0.f;

        // Platform joint P_i in local frame
        const float thp  = deg2rad(cfg_.plat_theta_deg[idx]);
        const float Px_l = Rp * std::cos(thp);
        const float Py_l = Rp * std::sin(thp);
        const float Pz_l = 0.f;

        // Rotate + translate to world
        float Px = R[0][0] * Px_l + R[0][1] * Py_l + R[0][2] * Pz_l;
        float Py = R[1][0] * Px_l + R[1][1] * Py_l + R[1][2] * Pz_l;
        float Pz = R[2][0] * Px_l + R[2][1] * Py_l + R[2][2] * Pz_l;
        Pz += h;

        const float dx = Px - Bx;
        const float dy = Py - By;
        const float dz = Pz - Bz;

        const float ex_x = ctb,  ex_y = stb;
        const float ey_x = -stb, ey_y = ctb;

        const float x     = dx * ex_x + dy * ex_y;
        const float yperp = dx * ey_x + dy * ey_y;
        const float z     = dz;

        const float Rxz = std::hypot(x, z);
        if (Rxz < 1e-6f) {
            log_.warn("Kinematics3RRS: degenerate pose on leg " +
                      std::to_string(i) + " for frame " + std::to_string(sp.frame_id) +
                      " -> using last valid command");
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
            usedFallback = true;
            continue;
        }

        const float C = (x * x + yperp * yperp + z * z + L1 * L1 - L2 * L2) / (2.0f * L1);
        const float ratio = C / Rxz;

        // Do NOT silently clamp unreachable geometry into a fake valid solution.
        if (!std::isfinite(ratio) || ratio < -1.0f || ratio > 1.0f) {
            log_.warn("Kinematics3RRS: unreachable geometry on leg " +
                      std::to_string(i) + " for frame " + std::to_string(sp.frame_id) +
                      " (ratio=" + std::to_string(ratio) +
                      ") -> using last valid command");
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
            usedFallback = true;
            continue;
        }

        const float a     = std::acos(ratio);
        const float gamma = std::atan2(z, x);

        const float qA = gamma + a;
        const float qB = gamma - a;

        float q = chooseClosest(qA, qB, q_prev_[idx]);

        if (std::cos(q) < 0.0f) {
            q = otherBranch(qA, qB, q);
        }

        if (!std::isfinite(q)) {
            log_.warn("Kinematics3RRS: non-finite mechanism angle on leg " +
                      std::to_string(i) + " for frame " + std::to_string(sp.frame_id) +
                      " -> using last valid command");
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
            usedFallback = true;
            continue;
        }

        q_prev_[idx] = q;

        float servo_deg = cfg_.servo_neutral_deg[idx]
                        + static_cast<float>(cfg_.servo_dir[idx]) * rad2deg(q);

        if (!std::isfinite(servo_deg)) {
            log_.warn("Kinematics3RRS: non-finite servo mapping on leg " +
                      std::to_string(i) + " for frame " + std::to_string(sp.frame_id) +
                      " -> using last valid command");
            cmd.actuator_targets[idx] = last_valid_deg_[idx];
            usedFallback = true;
            continue;
        }

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