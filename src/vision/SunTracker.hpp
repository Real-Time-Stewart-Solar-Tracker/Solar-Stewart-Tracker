#pragma once

/**
 * @file SunTracker.hpp
 * @brief Bright-region sun detection from one event-driven frame.
 *
 * This class has one job:
 * - consume one @ref FrameEvent
 * - produce one @ref SunEstimate
 *
 * It does not own worker threads.
 * It does not own camera hardware.
 * It does not perform control.
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>

namespace solar {

/**
 * @brief Vision module that detects the sun centroid in a frame.
 *
 * Supported frame formats:
 * - @ref PixelFormat::Gray8
 * - @ref PixelFormat::RGB888
 * - @ref PixelFormat::BGR888
 *
 * Supported memory layouts:
 * - packed rows
 * - padded rows, as long as @ref FrameEvent::stride_bytes is correct
 */
class SunTracker {
public:
    /**
     * @brief Callback type used to deliver sun-detection results.
     */
    using EstimateCallback = std::function<void(const SunEstimate&)>;

    /**
     * @brief Runtime configuration for the sun detector.
     */
    struct Config {
        std::uint8_t threshold{200};    ///< Pixel threshold for bright-region segmentation.
        std::size_t min_pixels{30U};    ///< Minimum above-threshold pixels for a valid detection.
        float confidence_scale{10.0F};  ///< Confidence scaling factor.
    };

    /**
     * @brief Construct the tracker.
     *
     * @param log Shared logger.
     * @param cfg Tracker configuration.
     */
    SunTracker(Logger& log, Config cfg);

    SunTracker(const SunTracker&) = delete;
    SunTracker& operator=(const SunTracker&) = delete;

    /**
     * @brief Register the estimate callback.
     *
     * @param cb Callback receiving the latest estimate.
     */
    void registerEstimateCallback(EstimateCallback cb);

    /**
     * @brief Update the segmentation threshold at runtime.
     *
     * @param thr New threshold.
     */
    void setThreshold(std::uint8_t thr);

    /**
     * @brief Return the current tracker configuration.
     *
     * @return Current tracker configuration.
     */
    Config config() const;

    /**
     * @brief Process one frame.
     *
     * If the frame is valid, the tracker analyses it and emits one estimate
     * through the registered callback.
     *
     * Invalid frames are rejected safely.
     *
     * @param frame Input frame to analyse.
     */
    void onFrame(const FrameEvent& frame);

private:
    /**
     * @brief Validate the incoming frame contract.
     *
     * @param frame Frame to validate.
     * @return True if the frame is safe to read.
     */
    bool isFrameValid_(const FrameEvent& frame) const;

    /**
     * @brief Read one pixel intensity in an 8-bit grayscale-like form.
     *
     * @param frame Source frame.
     * @param x Pixel x-coordinate.
     * @param y Pixel y-coordinate.
     * @return Intensity in range [0, 255].
     */
    std::uint8_t intensityAt_(const FrameEvent& frame, int x, int y) const;

private:
    Logger& log_;
    Config cfg_{};

    mutable std::mutex cfg_mtx_;
    mutable std::mutex cb_mtx_;
    EstimateCallback estimate_cb_{};
};

} // namespace solar