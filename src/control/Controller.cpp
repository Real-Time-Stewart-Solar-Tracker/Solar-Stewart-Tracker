#include "control/Controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <utility>

namespace solar {

Controller::Controller(Logger& log, Config cfg)
    : log_(log),
      cfg_(cfg) {
}

void Controller::registerSetpointCallback(SetpointCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mtx_);
    cb_ = std::move(cb);
}

void Controller::setMinConfidence(float c) {
    std::lock_guard<std::mutex> lock(cfg_mtx_);
    cfg_.min_confidence = c;
}

Controller::Config Controller::config() const {
    std::lock_guard<std::mutex> lock(cfg_mtx_);
    return cfg_;
}

float Controller::clamp_(float v, float lo, float hi) const {
    return std::max(lo, std::min(v, hi));
}

float Controller::applyDeadband_(float value) const {
    float db = 0.0F;
    {
        std::lock_guard<std::mutex> lock(cfg_mtx_);
        db = cfg_.deadband;
    }

    if (std::fabs(value) < db) {
        return 0.0F;
    }

    const float sign = (value > 0.0F) ? 1.0F : -1.0F;
    const float mag = (std::fabs(value) - db) / (1.0F - db);
    return sign * mag;
}

void Controller::onEstimate(const SunEstimate& est) {
    // Local configuration for this control update.
    Config cfg_copy;
    {
        std::lock_guard<std::mutex> lock(cfg_mtx_);
        cfg_copy = cfg_;
    }

    // Start from a neutral command and only drive once the estimate is valid.
    PlatformSetpoint sp{};
    sp.frame_id = est.frame_id;
    sp.t_control = std::chrono::steady_clock::now();
    sp.pan_rad = 0.0F;
    sp.tilt_rad = 0.0F;

    SetpointCallback cb;
    {
        std::lock_guard<std::mutex> lock(cb_mtx_);
        cb = cb_;
    }

    if (!cb) {
        return;
    }

    // Drop weak detections before they reach the control law.
    if (est.confidence < cfg_copy.min_confidence) {
        cb(sp);
        return;
    }

    // Guard against invalid geometry before normalising pixel error.
    if (cfg_copy.image_width <= 0 || cfg_copy.image_height <= 0) {
        log_.warn("Controller: invalid image dimensions in config");
        cb(sp);
        return;
    }

    // Image-centre reference for normalised tracking error.
    const float cx = static_cast<float>(cfg_copy.image_width) * 0.5F;
    const float cy = static_cast<float>(cfg_copy.image_height) * 0.5F;

    const float ex = (est.cx - cx) / cx;
    const float ey = (est.cy - cy) / cy;

    // Deadband shaping around the optical centre.
    const float ex_db = applyDeadband_(ex);
    const float ey_db = applyDeadband_(ey);

    // Calibrated sign convention from the working tracker.
    float pan = cfg_copy.k_pan * ex_db;
    float tilt = cfg_copy.k_tilt * ey_db;

    // Clamp before publishing so downstream stages see only safe setpoints.
    pan = clamp_(pan, -cfg_copy.max_pan_rad, cfg_copy.max_pan_rad);
    tilt = clamp_(tilt, -cfg_copy.max_tilt_rad, cfg_copy.max_tilt_rad);

    sp.pan_rad = pan;
    sp.tilt_rad = tilt;

    cb(sp);
}

} // namespace solar