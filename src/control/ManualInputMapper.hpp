#pragma once

/**
 * @file ManualInputMapper.hpp
 * @brief Pure mapping logic from manual potentiometer samples to bounded tilt/pan commands.
 */

#include "common/ManualInputTypes.hpp"

#include <cstdint>

namespace solar {

/**
 * @brief Control-facing configuration for manual input mapping.
 *
 * This is intentionally separate from ADS1115 hardware settings.
 */
struct ManualInputMapperConfig {
    float pot_supply_voltage{3.3F};   ///< Potentiometer supply/reference voltage.
    bool invert_tilt{false};          ///< Invert tilt direction.
    bool invert_pan{false};           ///< Invert pan direction.
    float tilt_deadband_norm{0.0F};   ///< Tilt deadband around centre in [-1, 1].
    float pan_deadband_norm{0.0F};    ///< Pan deadband around centre in [-1, 1].
    float max_manual_tilt_deg{20.0F}; ///< Maximum manual tilt command in degrees.
    float max_manual_pan_deg{20.0F};  ///< Maximum manual pan command in degrees.
};

/**
 * @brief Manual command generated from manual potentiometer input.
 *
 * Convention used throughout the project:
 * - x = pan
 * - y = tilt
 */
struct ManualCommand {
    float pan_deg{0.0F};          ///< Manual pan command in degrees.
    float tilt_deg{0.0F};         ///< Manual tilt command in degrees.
    float tilt_norm{0.0F};        ///< Normalised tilt input in [-1, 1].
    float pan_norm{0.0F};         ///< Normalised pan input in [-1, 1].
    float tilt_voltage_v{0.0F};   ///< Raw tilt potentiometer voltage.
    float pan_voltage_v{0.0F};    ///< Raw pan potentiometer voltage.
    std::uint64_t sequence{0U};   ///< Forwarded sample sequence.
    ManualInputTimePoint timestamp{}; ///< Forwarded timestamp.
};

/**
 * @brief Maps manual pot voltages to bounded manual commands.
 */
class ManualInputMapper {
public:
    /**
     * @brief Construct the mapper.
     *
     * @param config Mapping configuration.
     */
    explicit ManualInputMapper(const ManualInputMapperConfig& config);

    /**
     * @brief Map one hardware sample into a bounded manual command.
     *
     * @param sample Raw manual potentiometer sample.
     * @return Bounded manual command.
     */
    ManualCommand map(const ManualPotSample& sample) const;

private:
    float voltageToNormalised(float voltage_v) const;
    float applyDeadband(float value, float deadband) const;
    float clampUnit(float value) const;

    ManualInputMapperConfig config_{};
};

} // namespace solar