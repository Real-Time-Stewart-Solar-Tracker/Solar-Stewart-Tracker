/**
 * @file test_imu_feedback_mapper.cpp
 * @brief Unit tests for ImuFeedbackMapper.
 */

#include "control/ImuFeedbackMapper.hpp"
#include "src/tests/support/test_common.hpp"

#include <cmath>

using solar::control::ImuFeedbackMapper;

namespace {

bool nearlyEqual(float a, float b, float eps = 1e-6F) {
    return std::fabs(a - b) <= eps;
}

} // namespace

TEST_CASE(ImuFeedbackMapper_ZeroTilt_ProducesZeroCorrection) {
    ImuFeedbackMapper mapper(ImuFeedbackMapper::Config{1.0F, 0.0F, 10.0F});
    REQUIRE(nearlyEqual(mapper.correctionFromTilt(0.0F), 0.0F));
}

TEST_CASE(ImuFeedbackMapper_PositiveTilt_ProducesNegativeCorrection) {
    ImuFeedbackMapper mapper(ImuFeedbackMapper::Config{2.0F, 0.0F, 10.0F});
    REQUIRE(nearlyEqual(mapper.correctionFromTilt(0.5F), -1.0F));
}

TEST_CASE(ImuFeedbackMapper_NegativeTilt_ProducesPositiveCorrection) {
    ImuFeedbackMapper mapper(ImuFeedbackMapper::Config{3.0F, 0.0F, 10.0F});
    REQUIRE(nearlyEqual(mapper.correctionFromTilt(-0.25F), 0.75F));
}

TEST_CASE(ImuFeedbackMapper_RuntimeGainChange_TakesEffectImmediately) {
    ImuFeedbackMapper mapper(ImuFeedbackMapper::Config{1.0F, 0.0F, 10.0F});
    REQUIRE(nearlyEqual(mapper.correctionFromTilt(0.25F), -0.25F));

    mapper.setGain(4.0F);
    REQUIRE(nearlyEqual(mapper.correctionFromTilt(0.25F), -1.0F));
}
