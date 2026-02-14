#pragma once

#include <cstdint>
#include <functional>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * SunTracker
 *
 * Consumes FrameEvent and produces SunEstimate.
 *
 * Current supported input:
 * - Grayscale image where FrameEvent::data contains width*height bytes.
 *
 * Output:
 * - (cx, cy) centroid in pixels
 * - confidence in [0,1]
 *
 * Notes:
 * - Pure processing logic (no threads inside)
 * - Designed for event-driven pipeline: onFrame(...) called from camera callback thread
 */
class SunTracker {
public:
    using EstimateCallback = std::function<void(const SunEstimate&)>;

    struct Config {
        // Pixel is considered "bright" if >= threshold.
        uint8_t threshold{200};

        // Minimum number of bright pixels required to accept detection.
        std::size_t min_pixels{30};

        // Confidence scales with fraction of pixels above threshold (clamped).
        float confidence_scale{10.0f};
    };

    SunTracker(Logger& log, Config cfg);

    SunTracker(const SunTracker&) = delete;
    SunTracker& operator=(const SunTracker&) = delete;

    void registerEstimateCallback(EstimateCallback cb);

    // Process one frame (event-driven call).
    void onFrame(const FrameEvent& frame);

    Config config() const;

private:
    Logger& log_;
    Config cfg_;
    EstimateCallback estimateCb_{};
};

} // namespace solar