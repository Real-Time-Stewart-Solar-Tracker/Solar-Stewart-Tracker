#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace solar {

/**
 * @brief Supported pixel layouts carried by FrameEvent.
 *
 * The public frame contract supports:
 * - Gray8:  1 byte per pixel
 * - RGB888: 3 bytes per pixel, in R-G-B byte order
 * - BGR888: 3 bytes per pixel, in B-G-R byte order
 */
enum class PixelFormat : uint8_t {
    Gray8,
    RGB888,
    BGR888
};

/**
 * @brief Return the number of bytes per pixel for a given pixel format.
 */
inline constexpr std::size_t bytesPerPixel(PixelFormat fmt) noexcept {
    switch (fmt) {
        case PixelFormat::Gray8:  return 1U;
        case PixelFormat::RGB888: return 3U;
        case PixelFormat::BGR888: return 3U;
    }
    return 1U;
}

/**
 * @brief Return a readable name for a pixel format.
 */
inline constexpr const char* pixelFormatName(PixelFormat fmt) noexcept {
    switch (fmt) {
        case PixelFormat::Gray8:  return "Gray8";
        case PixelFormat::RGB888: return "RGB888";
        case PixelFormat::BGR888: return "BGR888";
    }
    return "Unknown";
}

/**
 * @brief Represents one captured frame passed into the processing pipeline.
 *
 * Contract:
 * - @ref data stores raw pixel bytes for a single image.
 * - @ref width and @ref height are image dimensions in pixels.
 * - @ref format defines how bytes in @ref data are interpreted.
 * - @ref stride_bytes is the number of bytes per image row.
 *
 * Valid row layout rules:
 * - Gray8  : each row contains at least width bytes
 * - RGB888 : each row contains at least width * 3 bytes
 * - BGR888 : each row contains at least width * 3 bytes
 *
 * Buffer size rule:
 * - data.size() must be at least stride_bytes * height
 *
 * Packed images use:
 * - Gray8  -> stride_bytes = width
 * - RGB888 -> stride_bytes = width * 3
 * - BGR888 -> stride_bytes = width * 3
 */
struct FrameEvent {
    /// @brief Monotonic frame identifier.
    uint64_t frame_id{0};

    /// @brief Capture timestamp.
    std::chrono::steady_clock::time_point t_capture;

    /// @brief Raw frame bytes in the layout described by format and stride_bytes.
    std::vector<uint8_t> data;

    /// @brief Frame width in pixels.
    int width{0};

    /// @brief Frame height in pixels.
    int height{0};

    /// @brief Number of bytes between the start of one row and the next.
    int stride_bytes{0};

    /// @brief Pixel layout used by @ref data.
    PixelFormat format{PixelFormat::Gray8};
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
 * @brief Status of an actuator command produced by kinematics.
 *
 * This allows downstream stages to distinguish:
 * - a normal valid command
 * - a command that reused the last valid output
 * - a command generated under invalid geometry configuration
 */
enum class CommandStatus : uint8_t {
    Ok,                        ///< Normal command
    KinematicsFallbackLastValid, ///< Degraded command using last valid output
    KinematicsInvalidConfig      ///< Invalid kinematics configuration detected
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

    /// @brief Target servo angles in degrees for the three actuators.
    std::array<float, 3> actuator_targets{0.0f, 0.0f, 0.0f};

    /// @brief Command quality / validity status from kinematics.
    CommandStatus status{CommandStatus::Ok};
};

} // namespace solar