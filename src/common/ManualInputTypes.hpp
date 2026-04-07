#pragma once

/**
 * @file ManualInputTypes.hpp
 * @brief Shared data types for manual potentiometer input.
 */

#include <chrono>
#include <cstdint>

namespace solar {

/**
 * @brief Monotonic clock used for manual-input timestamps.
 */
using ManualInputClock = std::chrono::steady_clock;

/**
 * @brief Timestamp type for manual-input samples.
 */
using ManualInputTimePoint = ManualInputClock::time_point;

/**
 * @brief One complete manual potentiometer sample.
 *
 * This is a hardware-facing data packet emitted by the ADS1115 backend after
 * tilt and pan channels have both been measured.
 */
struct ManualPotSample {
    float tilt_voltage_v{0.0F};     ///< Raw tilt potentiometer voltage.
    float pan_voltage_v{0.0F};      ///< Raw pan potentiometer voltage.
    float spare_voltage_v{0.0F};    ///< Optional spare channel voltage.
    std::uint64_t sequence{0U};     ///< Monotonic sample sequence.
    ManualInputTimePoint timestamp{}; ///< Monotonic sample timestamp.
};

} // namespace solar