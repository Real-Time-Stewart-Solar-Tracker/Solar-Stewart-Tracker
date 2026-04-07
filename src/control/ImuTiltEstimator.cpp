#include "control/ImuTiltEstimator.hpp"

#include <cmath>

namespace solar::control {

float ImuTiltEstimator::estimateTiltRad(const ImuSample& sample) const {
    if (!sample.valid) return 0.0f;

    // Use the gravity vector projection to estimate tilt from the accelerometer only.
    const float denom = std::sqrt(sample.ay * sample.ay + sample.az * sample.az);
    if (denom <= 1e-6f) return 0.0f;

    return std::atan2(sample.ax, denom);
}

} // namespace solar::control