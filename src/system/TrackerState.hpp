#pragma once

namespace solar {

/**
 * @brief High-level state machine for the solar tracking system.
 *
 * Represents the operational mode of the SystemManager.
 */
enum class TrackerState {

    /// @brief System inactive and not running.
    IDLE,

    /// @brief System initialization in progress.
    STARTUP,

    /// @brief Platform moving to neutral reference position.
    NEUTRAL,

    /// @brief Searching for the sun (no reliable estimate yet).
    SEARCHING,

    /// @brief Actively tracking the sun using vision feedback.
    TRACKING,

    /// @brief Manual control mode (user-defined setpoint).
    MANUAL,

    /// @brief Graceful shutdown in progress.
    STOPPING,

    /// @brief Fault condition detected.
    FAULT
};

/**
 * @brief Convert TrackerState to human-readable string.
 * @param s TrackerState value.
 * @return Null-terminated string representation.
 */
inline const char* toString(TrackerState s) {
    switch (s) {
        case TrackerState::IDLE: return "IDLE";
        case TrackerState::STARTUP: return "STARTUP";
        case TrackerState::NEUTRAL: return "NEUTRAL";
        case TrackerState::SEARCHING: return "SEARCHING";
        case TrackerState::TRACKING: return "TRACKING";
        case TrackerState::MANUAL: return "MANUAL";
        case TrackerState::STOPPING: return "STOPPING";
        case TrackerState::FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

} // namespace solar