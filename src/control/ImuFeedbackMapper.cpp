#include "control/ImuFeedbackMapper.hpp"

#include <algorithm>
#include <cmath>

namespace solar::control {

float ImuFeedbackMapper::apply(const float commanded_tilt_rad,
                               const float measured_tilt_rad,
                               const bool valid) const {
    if (!valid) {
        return commanded_tilt_rad;
    }

    const float error_rad = commanded_tilt_rad - measured_tilt_rad;
    const float error_db = applyDeadband_(error_rad, cfg_.deadband_rad);
    const float correction =
        clampSymmetric_(cfg_.gain * error_db, cfg_.max_correction_rad);

    return commanded_tilt_rad + correction;
}

float ImuFeedbackMapper::correctionFromTilt(const float tilt_rad) const {
    const float error_db = applyDeadband_(-tilt_rad, cfg_.deadband_rad);
    return clampSymmetric_(cfg_.gain * error_db, cfg_.max_correction_rad);
}

float ImuFeedbackMapper::applyDeadband_(const float value, const float deadband) {
    const float db = std::max(deadband, 0.0f);

    if (std::fabs(value) <= db) {
        return 0.0f;
    }

    if (value > 0.0f) {
        return value - db;
    }

    return value + db;
}

float ImuFeedbackMapper::clampSymmetric_(const float value, const float limit) {
    const float lim = std::max(limit, 0.0f);
    return std::clamp(value, -lim, lim);
}

} // namespace solar::control