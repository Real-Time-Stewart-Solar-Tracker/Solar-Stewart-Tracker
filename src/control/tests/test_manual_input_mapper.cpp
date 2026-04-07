/**
 * @file test_manual_input_mapper.cpp
 * @brief Unit tests for ManualInputMapper.
 */

#include "control/ManualInputMapper.hpp"
#include "src/tests/support/test_common.hpp"

#include <cmath>

using solar::ManualInputMapper;
using solar::ManualInputMapperConfig;
using solar::ManualPotSample;

namespace {

bool nearlyEqual(float a, float b, float eps = 1e-4F) {
    return std::fabs(a - b) <= eps;
}

} // namespace

TEST_CASE(ManualInputMapper_CentreVoltage_MapsToZeroCommand) {
    ManualInputMapperConfig cfg{};
    cfg.pot_supply_voltage = 3.3F;
    cfg.max_manual_pan_deg = 20.0F;
    cfg.max_manual_tilt_deg = 20.0F;

    ManualInputMapper mapper(cfg);

    ManualPotSample sample{};
    sample.tilt_voltage_v = 1.65F;
    sample.pan_voltage_v = 1.65F;
    sample.sequence = 42U;

    const auto cmd = mapper.map(sample);

    REQUIRE(nearlyEqual(cmd.tilt_norm, 0.0F));
    REQUIRE(nearlyEqual(cmd.pan_norm, 0.0F));
    REQUIRE(nearlyEqual(cmd.tilt_deg, 0.0F));
    REQUIRE(nearlyEqual(cmd.pan_deg, 0.0F));
    REQUIRE(cmd.sequence == 42U);
}

TEST_CASE(ManualInputMapper_LowAndHighVoltages_MapToMinusAndPlusOne) {
    ManualInputMapperConfig cfg{};
    cfg.pot_supply_voltage = 3.3F;
    cfg.max_manual_pan_deg = 20.0F;
    cfg.max_manual_tilt_deg = 20.0F;

    ManualInputMapper mapper(cfg);

    ManualPotSample low{};
    low.tilt_voltage_v = 0.0F;
    low.pan_voltage_v = 0.0F;

    ManualPotSample high{};
    high.tilt_voltage_v = 3.3F;
    high.pan_voltage_v = 3.3F;

    const auto low_cmd = mapper.map(low);
    const auto high_cmd = mapper.map(high);

    REQUIRE(nearlyEqual(low_cmd.tilt_norm, -1.0F));
    REQUIRE(nearlyEqual(low_cmd.pan_norm, -1.0F));
    REQUIRE(nearlyEqual(low_cmd.tilt_deg, -20.0F));
    REQUIRE(nearlyEqual(low_cmd.pan_deg, -20.0F));

    REQUIRE(nearlyEqual(high_cmd.tilt_norm, 1.0F));
    REQUIRE(nearlyEqual(high_cmd.pan_norm, 1.0F));
    REQUIRE(nearlyEqual(high_cmd.tilt_deg, 20.0F));
    REQUIRE(nearlyEqual(high_cmd.pan_deg, 20.0F));
}

TEST_CASE(ManualInputMapper_InversionFlags_FlipDirections) {
    ManualInputMapperConfig cfg{};
    cfg.pot_supply_voltage = 3.3F;
    cfg.invert_tilt = true;
    cfg.invert_pan = true;
    cfg.max_manual_pan_deg = 20.0F;
    cfg.max_manual_tilt_deg = 20.0F;

    ManualInputMapper mapper(cfg);

    ManualPotSample sample{};
    sample.tilt_voltage_v = 3.3F;
    sample.pan_voltage_v = 3.3F;

    const auto cmd = mapper.map(sample);

    REQUIRE(nearlyEqual(cmd.tilt_norm, -1.0F));
    REQUIRE(nearlyEqual(cmd.pan_norm, -1.0F));
    REQUIRE(nearlyEqual(cmd.tilt_deg, -20.0F));
    REQUIRE(nearlyEqual(cmd.pan_deg, -20.0F));
}

TEST_CASE(ManualInputMapper_Deadband_ZerosSmallInputsAndRescalesBeyondDeadband) {
    ManualInputMapperConfig cfg{};
    cfg.pot_supply_voltage = 3.3F;
    cfg.tilt_deadband_norm = 0.2F;
    cfg.pan_deadband_norm = 0.2F;
    cfg.max_manual_pan_deg = 20.0F;
    cfg.max_manual_tilt_deg = 20.0F;

    ManualInputMapper mapper(cfg);

    // 0.55 * 3.3 => norm about +0.1, should be suppressed by deadband
    ManualPotSample small{};
    small.tilt_voltage_v = 1.815F;
    small.pan_voltage_v = 1.815F;

    const auto small_cmd = mapper.map(small);
    REQUIRE(nearlyEqual(small_cmd.tilt_norm, 0.0F));
    REQUIRE(nearlyEqual(small_cmd.pan_norm, 0.0F));

    // 0.75 * 3.3 => norm about +0.5, should survive and be rescaled
    ManualPotSample bigger{};
    bigger.tilt_voltage_v = 2.475F;
    bigger.pan_voltage_v = 2.475F;

    const auto bigger_cmd = mapper.map(bigger);
    REQUIRE(bigger_cmd.tilt_norm > 0.0F);
    REQUIRE(bigger_cmd.pan_norm > 0.0F);
    REQUIRE(bigger_cmd.tilt_deg > 0.0F);
    REQUIRE(bigger_cmd.pan_deg > 0.0F);
}

TEST_CASE(ManualInputMapper_OutOfRangeVoltages_AreClampedSafely) {
    ManualInputMapperConfig cfg{};
    cfg.pot_supply_voltage = 3.3F;
    cfg.max_manual_pan_deg = 20.0F;
    cfg.max_manual_tilt_deg = 20.0F;

    ManualInputMapper mapper(cfg);

    ManualPotSample sample{};
    sample.tilt_voltage_v = -10.0F;
    sample.pan_voltage_v = 99.0F;

    const auto cmd = mapper.map(sample);

    REQUIRE(nearlyEqual(cmd.tilt_norm, -1.0F));
    REQUIRE(nearlyEqual(cmd.pan_norm, 1.0F));
    REQUIRE(nearlyEqual(cmd.tilt_deg, -20.0F));
    REQUIRE(nearlyEqual(cmd.pan_deg, 20.0F));
}