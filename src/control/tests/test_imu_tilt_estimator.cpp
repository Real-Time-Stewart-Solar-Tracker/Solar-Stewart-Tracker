/**
 * @file test_imu_tilt_estimator.cpp
 * @brief Unit tests for ImuTiltEstimator.
 */

#include "control/ImuTiltEstimator.hpp"
#include "src/tests/support/test_common.hpp"

#include <cmath>

using solar::control::ImuSample;
using solar::control::ImuTiltEstimator;

namespace {

bool nearlyEqual(float a, float b, float eps = 1e-4F) {
    return std::fabs(a - b) <= eps;
}

} // namespace

TEST_CASE(ImuTiltEstimator_InvalidSample_ReturnsZero) {
    ImuTiltEstimator estimator;
    ImuSample s{};
    s.valid = false;

    REQUIRE(nearlyEqual(estimator.estimateTiltRad(s), 0.0F));
}

TEST_CASE(ImuTiltEstimator_LevelGravity_ReturnsZeroTilt) {
    ImuTiltEstimator estimator;
    ImuSample s{};
    s.ax = 0.0F;
    s.ay = 0.0F;
    s.az = 9.81F;
    s.valid = true;

    REQUIRE(nearlyEqual(estimator.estimateTiltRad(s), 0.0F));
}

TEST_CASE(ImuTiltEstimator_PositiveAx_WithGravity_ReturnsPositiveTilt) {
    ImuTiltEstimator estimator;
    ImuSample s{};
    s.ax = 9.81F;
    s.ay = 0.0F;
    s.az = 9.81F;
    s.valid = true;

    const float tilt = estimator.estimateTiltRad(s);
    REQUIRE(tilt > 0.0F);
    REQUIRE(nearlyEqual(tilt, static_cast<float>(std::atan2(9.81F, 9.81F))));
}

TEST_CASE(ImuTiltEstimator_NegativeAx_WithGravity_ReturnsNegativeTilt) {
    ImuTiltEstimator estimator;
    ImuSample s{};
    s.ax = -9.81F;
    s.ay = 0.0F;
    s.az = 9.81F;
    s.valid = true;

    const float tilt = estimator.estimateTiltRad(s);
    REQUIRE(tilt < 0.0F);
}

TEST_CASE(ImuTiltEstimator_DegenerateDenominator_ReturnsZero) {
    ImuTiltEstimator estimator;
    ImuSample s{};
    s.ax = 1.0F;
    s.ay = 0.0F;
    s.az = 0.0F;
    s.valid = true;

    REQUIRE(nearlyEqual(estimator.estimateTiltRad(s), 0.0F));
}