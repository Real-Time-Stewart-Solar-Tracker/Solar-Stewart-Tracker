#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

namespace solar {

/**
 * FrameEvent
 *
 * Represents a captured camera frame.
 * This struct carries raw image data and the timestamp
 * when the frame was captured.
 */
struct FrameEvent {
    uint64_t frame_id{0};

    std::chrono::steady_clock::time_point t_capture;

    // Raw image buffer (can later be replaced with OpenCV or libcamera buffer)
    std::vector<uint8_t> data;

    int width{0};
    int height{0};
};


/**
 * SunEstimate
 *
 * Output of the vision module.
 * Contains estimated sun centroid and confidence.
 */
struct SunEstimate {
    uint64_t frame_id{0};

    std::chrono::steady_clock::time_point t_estimate;

    float cx{0.0f};   // x-coordinate (pixels)
    float cy{0.0f};   // y-coordinate (pixels)

    float confidence{0.0f};  // 0.0 to 1.0
};


/**
 * PlatformSetpoint
 *
 * Desired platform orientation computed by controller.
 */
struct PlatformSetpoint {
    std::chrono::steady_clock::time_point t_control;

    float tilt_rad{0.0f};
    float pan_rad{0.0f};
};


/**
 * ActuatorCommand
 *
 * Output of kinematics layer.
 * Contains three actuator target values.
 */
struct ActuatorCommand {
    std::chrono::steady_clock::time_point t_actuate;

    std::array<float, 3> actuator_targets{0.0f, 0.0f, 0.0f};
};

} // namespace solar