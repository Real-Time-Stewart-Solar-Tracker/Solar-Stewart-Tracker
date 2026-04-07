#include "control/ManualInputMapper.hpp"

#include <algorithm>
#include <cmath>

namespace solar {
namespace {

float clampValue(const float value, const float lo, const float hi) {
    return std::max(lo, std::min(value, hi));
}

} // namespace

ManualInputMapper::ManualInputMapper(const ManualInputMapperConfig& config)
    : config_(config) {
}

ManualCommand ManualInputMapper::map(const ManualPotSample& sample) const {
    float tilt_norm = voltageToNormalised(sample.tilt_voltage_v);
    float pan_norm = voltageToNormalised(sample.pan_voltage_v);

    tilt_norm = applyDeadband(tilt_norm, config_.tilt_deadband_norm);
    pan_norm = applyDeadband(pan_norm, config_.pan_deadband_norm);

    if (config_.invert_tilt) {
        tilt_norm = -tilt_norm;
    }
    if (config_.invert_pan) {
        pan_norm = -pan_norm;
    }

    tilt_norm = clampUnit(tilt_norm);
    pan_norm = clampUnit(pan_norm);

    ManualCommand command{};
    command.tilt_norm = tilt_norm;
    command.pan_norm = pan_norm;
    command.tilt_voltage_v = sample.tilt_voltage_v;
    command.pan_voltage_v = sample.pan_voltage_v;
    command.sequence = sample.sequence;
    command.timestamp = sample.timestamp;

    command.pan_deg = pan_norm * config_.max_manual_pan_deg;
    command.tilt_deg = tilt_norm * config_.max_manual_tilt_deg;

    return command;
}

float ManualInputMapper::voltageToNormalised(const float voltage_v) const {
    const float supply = (config_.pot_supply_voltage > 0.0F)
        ? config_.pot_supply_voltage
        : 3.3F;

    const float clamped_voltage = clampValue(voltage_v, 0.0F, supply);
    const float normalised = ((clamped_voltage / supply) * 2.0F) - 1.0F;
    return clampUnit(normalised);
}

float ManualInputMapper::applyDeadband(const float value, const float deadband) const {
    const float x = clampUnit(value);
    const float db = clampValue(deadband, 0.0F, 0.999F);

    if (std::fabs(x) <= db) {
        return 0.0F;
    }

    const float sign = (x >= 0.0F) ? 1.0F : -1.0F;
    const float magnitude = (std::fabs(x) - db) / (1.0F - db);
    return sign * clampUnit(magnitude);
}

float ManualInputMapper::clampUnit(const float value) const {
    return clampValue(value, -1.0F, 1.0F);
}

} // namespace solar