#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <mutex>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * @brief Vision module that detects the sun centroid in a frame.
 *
 * Consumes FrameEvent objects (grayscale input) and produces SunEstimate outputs
 * via a registered callback. Designed for event-driven pipelines (no internal threads).
 *
 * Input:
 * - FrameEvent::data contains width * height bytes (8-bit grayscale).
 *
 * Output:
 * - Centroid (cx, cy) in pixel coordinates
 * - Confidence in range [0, 1]
 */
class SunTracker {
public:
    /// @brief Callback type used to deliver sun detection results.
    using EstimateCallback = std::function<void(const SunEstimate&)>;

    /// @brief Sun detection configuration parameters.
    struct Config {
        /// @brief Pixel intensity threshold for sun segmentation.
        uint8_t threshold{200};

        /// @brief Minimum number of above-threshold pixels required for a valid detection.
        std::size_t min_pixels{30};

        /// @brief Scaling factor used when mapping detection strength to confidence.
        float confidence_scale{10.0f};
    };

    /// @brief Construct tracker with logger and configuration.
    SunTracker(Logger& log, Config cfg);

    SunTracker(const SunTracker&) = delete;
    SunTracker& operator=(const SunTracker&) = delete;

    /// @brief Register callback to receive SunEstimate outputs.
    void registerEstimateCallback(EstimateCallback cb);

    /// @brief Update threshold at runtime (thread-safe).
    void setThreshold(uint8_t thr);

    /// @brief Process one frame and emit a SunEstimate if a callback is set.
    void onFrame(const FrameEvent& frame);

    /// @brief Get current configuration.
    Config config() const;

private:
    Logger& log_;
    Config cfg_;

    /// @brief Protects cfg_ for runtime updates.
    mutable std::mutex cfgMutex_;

    /// @brief Protects estimate callback registration and invocation.
    mutable std::mutex cbMtx_;

    /// @brief Registered estimate callback (may be empty).
    EstimateCallback estimateCb_{};
};

} // namespace solar