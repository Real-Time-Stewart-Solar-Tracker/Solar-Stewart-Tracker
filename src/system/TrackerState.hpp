#pragma once

/**
 * @file TrackerState.hpp
 * @brief Runtime state enumeration for the Solar Stewart Tracker.
 *
 * This file defines the top-level runtime states used by SystemManager.
 */

namespace solar {

/**
 * @brief Top-level runtime state.
 */
enum class TrackerState {
    IDLE,      ///< Runtime not started.
    STARTUP,   ///< Startup sequence in progress.
    NEUTRAL,   ///< Temporary neutral/park transition state.
    SEARCHING, ///< Automatic mode active, but target not confidently tracked.
    TRACKING,  ///< Automatic mode active, target confidently tracked.
    MANUAL,    ///< Manual mode active.
    STOPPING,  ///< Shutdown sequence in progress.
    FAULT      ///< Fatal runtime failure.
};

/**
 * @brief Convert a runtime state to a readable string.
 *
 * @param s Runtime state.
 * @return Readable constant string.
 */
inline constexpr const char* toString(const TrackerState s) noexcept {
    switch (s) {
        case TrackerState::IDLE:      return "IDLE";
        case TrackerState::STARTUP:   return "STARTUP";
        case TrackerState::NEUTRAL:   return "NEUTRAL";
        case TrackerState::SEARCHING: return "SEARCHING";
        case TrackerState::TRACKING:  return "TRACKING";
        case TrackerState::MANUAL:    return "MANUAL";
        case TrackerState::STOPPING:  return "STOPPING";
        case TrackerState::FAULT:     return "FAULT";
    }
    return "UNKNOWN";
}

} // namespace solar