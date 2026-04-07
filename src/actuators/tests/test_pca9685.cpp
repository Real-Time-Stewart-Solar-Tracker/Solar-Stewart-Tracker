/**
 * @file test_pca9685.cpp
 * @brief Contract-level tests for PCA9685.
 */

#include "actuators/PCA9685.hpp"
#include "src/tests/support/test_common.hpp"

using solar::PCA9685;

TEST_CASE(PCA9685_default_constructor_is_not_started) {
    PCA9685 pca(1, 0x40U);
    REQUIRE(!pca.isStarted());
}

TEST_CASE(PCA9685_set_pulse_us_fails_before_start) {
    PCA9685 pca(1, 0x40U);
    REQUIRE(!pca.setPulseWidthUs(0, 1500.0F));
}

TEST_CASE(PCA9685_invalid_channel_rejected_even_if_not_started) {
    PCA9685 pca(1, 0x40U);
    REQUIRE(!pca.setPulseWidthUs(-1, 1500.0F));
    REQUIRE(!pca.setPulseWidthUs(16, 1500.0F));
}

TEST_CASE(PCA9685_stop_is_safe_when_not_started) {
    PCA9685 pca(1, 0x40U);
    pca.stop();
    REQUIRE(!pca.isStarted());
}