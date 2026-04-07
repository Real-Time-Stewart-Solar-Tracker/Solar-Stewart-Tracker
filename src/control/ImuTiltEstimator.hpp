#pragma once

/**
 * @file ImuTiltEstimator.hpp
 * @brief Pure tilt-estimation helper derived from IMU acceleration.
 */

#include <cstdint>

namespace solar::control {

/**
 * @brief Reduced IMU sample used by the tilt estimator.
 */
struct ImuSample {
    float ax{0.0f};           ///< Acceleration x-component.
    float ay{0.0f};           ///< Acceleration y-component.
    float az{1.0f};           ///< Acceleration z-component.
    std::uint64_t timestamp_us{0}; ///< Monotonic timestamp in microseconds.
    bool valid{false};        ///< True if the sample is valid.
};

/**
 * @brief Estimates platform tilt from a reduced IMU sample.
 *
 * This class is intentionally stateless and contains only the mathematical
 * mapping from acceleration to a tilt estimate.
 */
class ImuTiltEstimator {
public:
    /**
     * @brief Construct the estimator.
     */
    ImuTiltEstimator() = default;

    /**
     * @brief Estimate tilt in radians from one IMU sample.
     *
     * @param sample Input IMU sample.
     * @return Estimated tilt in radians.
     */
    float estimateTiltRad(const ImuSample& sample) const;
};

} // namespace solar::control
