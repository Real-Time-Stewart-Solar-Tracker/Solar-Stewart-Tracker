#include "control/Controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace solar {

Controller::Controller(Logger& log, Config cfg)
    : log_(log), cfg_(cfg) {}

void Controller::registerSetpointCallback(SetpointCallback cb) {
    std::lock_guard<std::mutex> lk(cbMtx_);
    setpointCb_ = std::move(cb);
}

Controller::Config Controller::config() const {
    return cfg_;
}

void Controller::onEstimate(const SunEstimate& est) {

    PlatformSetpoint sp;
    sp.frame_id  = est.frame_id;
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad  = 0.0f;
    sp.pan_rad   = 0.0f;

    // Confidence gate
    if (est.confidence < cfg_.min_confidence) {
        SetpointCallback cb;
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            cb = setpointCb_;
        }
        if (cb) cb(sp);
        return;
    }

    if (cfg_.width <= 0 || cfg_.height <= 0) {
        log_.warn("Controller: invalid image dimensions in config");
        SetpointCallback cb;
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            cb = setpointCb_;
        }
        if (cb) cb(sp);
        return;
    }

    // Image center
    const float cx0 = 0.5f * static_cast<float>(cfg_.width);
    const float cy0 = 0.5f * static_cast<float>(cfg_.height);

    // Normalized error in [-1, 1]
    const float ex = (est.cx - cx0) / cx0;
    const float ey = (est.cy - cy0) / cy0;

    auto applyDeadband = [](float e, float db) -> float {
        const float ae = std::fabs(e);
        if (ae <= db) return 0.0f;
        const float sign = (e >= 0.0f) ? 1.0f : -1.0f;
        return sign * (ae - db) / (1.0f - db);
    };

    const float ex_db = applyDeadband(ex, cfg_.deadband);
    const float ey_db = applyDeadband(ey, cfg_.deadband);

    float pan  = cfg_.k_pan  * ex_db;
    float tilt = cfg_.k_tilt * ey_db;

    pan  = std::clamp(pan,  -cfg_.max_pan_rad,  cfg_.max_pan_rad);
    tilt = std::clamp(tilt, -cfg_.max_tilt_rad, cfg_.max_tilt_rad);

    sp.pan_rad  = pan;
    sp.tilt_rad = tilt;

    SetpointCallback cb;
    {
        std::lock_guard<std::mutex> lk(cbMtx_);
        cb = setpointCb_;
    }

    if (cb) cb(sp);
}

} // namespace solar