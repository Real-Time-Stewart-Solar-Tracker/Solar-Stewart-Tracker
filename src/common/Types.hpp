#pragma once

/**
 * @file Types.hpp
 * @brief Small shared runtime data types used across the tracker pipeline.
 *
 * This file defines the event/data contracts passed between stages:
 * - camera -> vision
 * - vision -> controller
 * - controller -> kinematics
 * - kinematics -> actuation
 *
 * These are plain data structures on purpose.
 * No behaviour. No hidden ownership tricks. No policy.
 */

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace solar {

/**
 * @brief Monotonic clock used throughout the runtime.
 */
using Clock = std::chrono::steady_clock;

/**
 * @brief Timestamp type based on the monotonic runtime clock.
 */
using TimePoint = Clock::time_point;

/**
 * @brief Number of servos in the 3-RRS platform.
 */
inline constexpr std::size_t kServoCount = 3U;

/**
 * @brief Supported pixel layouts carried by FrameEvent.
 */
enum class PixelFormat : std::uint8_t {
    Gray8,  ///< 1 byte per pixel.
    RGB888, ///< 3 bytes per pixel in R-G-B order.
    BGR888  ///< 3 bytes per pixel in B-G-R order.
};

/**
 * @brief Return the number of bytes per pixel for a given pixel format.
 *
 * @param fmt Pixel format.
 * @return Bytes per pixel.
 */
inline constexpr std::size_t bytesPerPixel(const PixelFormat fmt) noexcept {
    switch (fmt) {
        case PixelFormat::Gray8:  return 1U;
        case PixelFormat::RGB888: return 3U;
        case PixelFormat::BGR888: return 3U;
    }
    return 1U;
}

/**
 * @brief Return a readable name for a pixel format.
 *
 * @param fmt Pixel format.
 * @return Constant readable pixel-format name.
 */
inline constexpr const char* pixelFormatName(const PixelFormat fmt) noexcept {
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
 * - @ref data stores raw pixel bytes for one image.
 * - @ref width and @ref height are image dimensions in pixels.
 * - @ref stride_bytes is the number of bytes between row starts.
 * - @ref format defines how bytes inside @ref data are interpreted.
 *
 * Buffer size rule:
 * - data.size() >= stride_bytes * height
 */
struct FrameEvent {
    std::uint64_t frame_id{0}; ///< Monotonic frame identifier.
    TimePoint t_capture{};     ///< Frame capture timestamp.

    std::vector<std::uint8_t> data; ///< Raw frame bytes.

    int width{0};              ///< Frame width in pixels.
    int height{0};             ///< Frame height in pixels.
    int stride_bytes{0};       ///< Number of bytes between consecutive rows.
    PixelFormat format{PixelFormat::Gray8}; ///< Pixel format of @ref data.
};

/**
 * @brief Vision output: estimated sun position and confidence.
 */
struct SunEstimate {
    std::uint64_t frame_id{0}; ///< Associated frame identifier.
    TimePoint t_estimate{};    ///< Estimate timestamp.

    float cx{0.0F};            ///< Estimated sun centroid x-coordinate in pixels.
    float cy{0.0F};            ///< Estimated sun centroid y-coordinate in pixels.
    float confidence{0.0F};    ///< Confidence in range [0, 1].
};

/**
 * @brief Controller output: desired platform orientation.
 */
struct PlatformSetpoint {
    std::uint64_t frame_id{0}; ///< Associated frame identifier.
    TimePoint t_control{};     ///< Control-computation timestamp.

    float tilt_rad{0.0F};      ///< Desired tilt in radians.
    float pan_rad{0.0F};       ///< Desired pan in radians.
};

/**
 * @brief Status of an actuator command produced by kinematics.
 */
enum class CommandStatus : std::uint8_t {
    Ok,                          ///< Normal valid command.
    KinematicsFallbackLastValid, ///< Degraded command reused from last valid output.
    KinematicsInvalidConfig      ///< Invalid geometry/configuration detected.
};

/**
 * @brief Kinematics output: servo/actuator targets.
 */
struct ActuatorCommand {
    std::uint64_t frame_id{0}; ///< Associated frame identifier.
    TimePoint t_actuate{};     ///< Actuation-command timestamp.

    std::array<float, kServoCount> actuator_targets{0.0F, 0.0F, 0.0F};
    CommandStatus status{CommandStatus::Ok};
};

} // namespace solar