#include "control/Controller.hpp"

#include <algorithm>
#include <cmath>

namespace solar {

Controller::Controller(Logger& log, Config cfg)
    : log_(log), cfg_(cfg) {}

void Controller::registerSetpointCallback(SetpointCallback cb) {
    setpointCb_ = std::move(cb);
}

Controller::Config Controller::config() const {
    return cfg_;
}

void Controller::onEstimate(const SunEstimate& est) {
    PlatformSetpoint sp;
    sp.t_control = std::chrono::steady_clock::now();
    sp.tilt_rad = 0.0f;
    sp.pan_rad = 0.0f;

    // Confidence gate: if sun is not reliable, output safe zero setpoint
    if (est.confidence < cfg_.min_confidence) {
        if (setpointCb_) setpointCb_(sp);
        return;
    }

    if (cfg_.width <= 0 || cfg_.height <= 0) {
        log_.warn("Controller: invalid image dimensions in config");
        if (setpointCb_) setpointCb_(sp);
        return;
    }

    // Normalized error in [-1, 1]
    // Center at (width/2, height/2)
    const float cx0 = 0.5f * static_cast<float>(cfg_.width);
    const float cy0 = 0.5f * static_cast<float>(cfg_.height);

    // error positive means sun is to the right / below center
    const float ex = (est.cx - cx0) / cx0;
    const float ey = (est.cy - cy0) / cy0;

    auto applyDeadband = [](float e, float db) -> float {
        const float ae = std::fabs(e);
        if (ae <= db) return 0.0f;
        // re-scale so output starts smoothly after deadband
        const float sign = (e >= 0.0f) ? 1.0f : -1.0f;
        return sign * (ae - db) / (1.0f - db);
    };

    const float ex_db = applyDeadband(ex, cfg_.deadband);
    const float ey_db = applyDeadband(ey, cfg_.deadband);

    // Simple proportional control:
    // - pan responds to x error
    // - tilt responds to y error (sign depends on your physical axis convention)
    float pan = cfg_.k_pan * ex_db;
    float tilt = cfg_.k_tilt * ey_db;

    // Clamp outputs
    pan = std::clamp(pan, -cfg_.max_pan_rad, cfg_.max_pan_rad);
    tilt = std::clamp(tilt, -cfg_.max_tilt_rad, cfg_.max_tilt_rad);

    sp.pan_rad = pan;
    sp.tilt_rad = tilt;

    if (setpointCb_) {
        setpointCb_(sp);
    }
}

} // namespace solar