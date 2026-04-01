#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * @brief Vision module that detects the sun centroid in a frame.
 *
 * The tracker consumes @ref FrameEvent objects and emits @ref SunEstimate
 * results through a registered callback. The class is event-driven and does
 * not create internal worker threads.
 *
 * Supported frame formats:
 * - @ref PixelFormat::Gray8
 * - @ref PixelFormat::RGB888
 * - @ref PixelFormat::BGR888
 *
 * Supported memory layouts:
 * - packed rows
 * - padded rows, provided @ref FrameEvent::stride_bytes correctly describes
 *   the byte distance between row starts
 *
 * Frame contract requirements:
 * - width > 0
 * - height > 0
 * - stride_bytes >= width * bytesPerPixel(format)
 * - data.size() >= stride_bytes * height
 *
 * Unsupported or invalid frames are rejected safely and produce no estimate.
 */
class SunTracker {
public:
    /// @brief Callback type used to deliver sun detection results.
    using EstimateCallback = std::function<void(const SunEstimate&)>;

    /// @brief Sun detection configuration parameters.
    struct Config {
        /// @brief Pixel-intensity threshold used for sun segmentation.
        uint8_t threshold{200};

        /// @brief Minimum number of above-threshold pixels required for a valid detection.
        std::size_t min_pixels{30};

        /// @brief Scaling factor used when mapping detection strength to confidence.
        float confidence_scale{10.0f};
    };

    /**
     * @brief Construct the tracker with logger and configuration.
     * @param log Logger used for warnings/debug information.
     * @param cfg Runtime configuration.
     */
    SunTracker(Logger& log, Config cfg);

    SunTracker(const SunTracker&) = delete;
    SunTracker& operator=(const SunTracker&) = delete;

    /**
     * @brief Register the callback used to receive @ref SunEstimate outputs.
     * @param cb Callback to invoke after processing a valid frame.
     */
    void registerEstimateCallback(EstimateCallback cb);

    /**
     * @brief Update the segmentation threshold at runtime.
     * @param thr New threshold value.
     */
    void setThreshold(uint8_t thr);

    /**
     * @brief Process a single frame.
     *
     * The frame may be Gray8, RGB888, or BGR888 and may use padded rows.
     * The implementation validates the frame contract before reading pixels.
     *
     * On successful processing, a @ref SunEstimate is emitted through the
     * registered callback if one is present.
     *
     * Invalid frames are rejected safely.
     *
     * @param frame Input frame to analyse.
     */
    void onFrame(const FrameEvent& frame);

    /**
     * @brief Return the current tracker configuration.
     */
    Config config() const;

private:
    /**
     * @brief Validate that a frame satisfies the current public frame contract.
     * @param frame Frame to validate.
     * @return true if the frame layout is valid and readable.
     */
    bool isFrameValid_(const FrameEvent& frame) const;

    /**
     * @brief Read one pixel intensity as an 8-bit grayscale-like value.
     *
     * Gray8 returns the source byte directly.
     * RGB888 and BGR888 are converted to luminance-style intensity.
     *
     * @param frame Source frame.
     * @param x Pixel x-coordinate.
     * @param y Pixel y-coordinate.
     * @return Pixel intensity in range [0, 255].
     */
    uint8_t intensityAt_(const FrameEvent& frame, int x, int y) const;

private:
    Logger& log_;
    Config cfg_;

    /// @brief Protects runtime configuration updates.
    mutable std::mutex cfgMutex_;

    /// @brief Protects callback registration and callback access.
    mutable std::mutex cbMtx_;

    /// @brief Registered estimate callback (may be empty).
    EstimateCallback estimateCb_{};
};

} // namespace solar