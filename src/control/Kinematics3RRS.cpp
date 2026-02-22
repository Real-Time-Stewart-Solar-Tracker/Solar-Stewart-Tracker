#include "control/Kinematics3RRS.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

namespace solar {

static constexpr float PI = 3.14159265358979323846f;

static float deg2rad(float d) {
    return d * PI / 180.0f;
}

Kinematics3RRS::Kinematics3RRS(Logger& log, Config cfg)
    : log_(log), cfg_(cfg) {}

void Kinematics3RRS::registerCommandCallback(CommandCallback cb) {
    std::lock_guard<std::mutex> lk(cbMtx_);
    cmdCb_ = std::move(cb);
}

Kinematics3RRS::Config Kinematics3RRS::config() const {
    return cfg_;
}

void Kinematics3RRS::onSetpoint(const PlatformSetpoint& sp) {
    computeIK_(sp);
}

void Kinematics3RRS::computeIK_(const PlatformSetpoint& sp) {
    // Mapping:
    // - sp.tilt_rad -> pitch
    // - sp.pan_rad  -> roll
    const float pitch = sp.tilt_rad;
    const float roll  = sp.pan_rad;

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

    ActuatorCommand cmd;
    cmd.frame_id  = sp.frame_id;
    cmd.t_actuate = std::chrono::steady_clock::now();

    // Basic parameter sanity guard (prevents NaNs)
    if (L1 <= 0.0f || L2 <= 0.0f || Rb <= 0.0f || Rp <= 0.0f) {
        log_.warn("Kinematics3RRS: invalid geometry config (non-positive length/radius)");
        // Fail-safe: hold previous commanded angles (already in q_prev_ with neutral applied below)
        for (int i = 0; i < 3; ++i) {
            const std::size_t idx = static_cast<std::size_t>(i);
            float chosen = q_prev_[idx];
            chosen = static_cast<float>(cfg_.servo_dir[idx]) * chosen + cfg_.servo_neutral_rad[idx];
            cmd.actuator_targets[idx] = chosen;
        }

        CommandCallback cb;
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            cb = cmdCb_;
        }
        if (cb) cb(cmd);
        return;
    }

    constexpr float EPS = 1e-6f;

    for (int i = 0; i < 3; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i);

        // Base joint B_i
        const float thb = deg2rad(cfg_.base_theta_deg[idx]);
        const float Bx = Rb * std::cos(thb);
        const float By = Rb * std::sin(thb);
        const float Bz = 0.f;

        // Platform joint P_i in platform local
        const float thp = deg2rad(cfg_.plat_theta_deg[idx]);
        const float Px_l = Rp * std::cos(thp);
        const float Py_l = Rp * std::sin(thp);
        const float Pz_l = 0.f;

        // Rotate + translate
        float Px = R[0][0] * Px_l + R[0][1] * Py_l + R[0][2] * Pz_l;
        float Py = R[1][0] * Px_l + R[1][1] * Py_l + R[1][2] * Pz_l;
        float Pz = R[2][0] * Px_l + R[2][1] * Py_l + R[2][2] * Pz_l;

        Pz += h;

        // Vector base->platform
        const float dx = Px - Bx;
        const float dy = Py - By;
        const float dz = Pz - Bz;

        // Projection along base leg plane direction
        const float C = dx * std::cos(thb) + dy * std::sin(thb);

        const float Rxz = std::sqrt(C * C + dz * dz);

        // Guard against degenerate geometry / division by zero
        if (Rxz <= EPS) {
            log_.warn("Kinematics3RRS: degenerate Rxz; holding previous angle");
            float chosen = q_prev_[idx];
            chosen = static_cast<float>(cfg_.servo_dir[idx]) * chosen + cfg_.servo_neutral_rad[idx];
            cmd.actuator_targets[idx] = chosen;
            continue;
        }

        float D = (L1 * L1 + Rxz * Rxz - L2 * L2) / (2.f * L1 * Rxz);
        D = std::clamp(D, -1.f, 1.f);

        const float alpha = std::atan2(dz, C);
        const float beta  = std::acos(D);

        const float q1 = alpha - beta;
        const float q2 = alpha + beta;

        // Choose branch closest to previous
        const float prev = q_prev_[idx];
        float chosen = (std::abs(q1 - prev) < std::abs(q2 - prev)) ? q1 : q2;

        q_prev_[idx] = chosen;

        // Apply direction and neutral offsets
        chosen = static_cast<float>(cfg_.servo_dir[idx]) * chosen
               + cfg_.servo_neutral_rad[idx];

        cmd.actuator_targets[idx] = chosen;
    }

    CommandCallback cb;
    {
        std::lock_guard<std::mutex> lk(cbMtx_);
        cb = cmdCb_;
    }
    if (cb) cb(cmd);
}

} // namespace solar