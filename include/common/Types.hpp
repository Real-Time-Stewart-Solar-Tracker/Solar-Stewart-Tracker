#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

namespace solar {

/**
 * @brief Represents a captured camera frame.
 *
 * Carries raw image data and the associated capture timestamp.
 */
struct FrameEvent {
    /// @brief Unique frame identifier.
    uint64_t frame_id{0};

    /// @brief Capture timestamp (steady_clock).
    std::chrono::steady_clock::time_point t_capture;

    /// @brief Raw image buffer (8-bit pixel data).
    std::vector<uint8_t> data;

    /// @brief Image width in pixels.
    int width{0};

    /// @brief Image height in pixels.
    int height{0};
};


/**
 * @brief Output of the vision module (sun detection result).
 *
 * Contains estimated sun centroid position and confidence score.
 */
struct SunEstimate {
    /// @brief Associated frame identifier.
    uint64_t frame_id{0};

    /// @brief Timestamp of estimate generation.
    std::chrono::steady_clock::time_point t_estimate;

    /// @brief Estimated sun centroid x-coordinate (pixels).
    float cx{0.0f};

    /// @brief Estimated sun centroid y-coordinate (pixels).
    float cy{0.0f};

    /// @brief Confidence score in range [0.0, 1.0].
    float confidence{0.0f};
};


/**
 * @brief Desired platform orientation computed by the controller.
 */
struct PlatformSetpoint {
    /// @brief Associated frame identifier.
    uint64_t frame_id{0};

    /// @brief Timestamp of control computation.
    std::chrono::steady_clock::time_point t_control;

    /// @brief Desired tilt angle (radians).
    float tilt_rad{0.0f};

    /// @brief Desired pan angle (radians).
    float pan_rad{0.0f};
};


/**
 * @brief Output of the kinematics layer.
 *
 * Contains actuator target values for the 3RRS platform.
 */
struct ActuatorCommand {
    /// @brief Associated frame identifier.
    uint64_t frame_id{0};

    /// @brief Timestamp of actuation command generation.
    std::chrono::steady_clock::time_point t_actuate;

    /// @brief Target values for the three actuators.
    std::array<float, 3> actuator_targets{0.0f, 0.0f, 0.0f};
};

} // namespace solar