// #include "vision/SunTracker.hpp"
// #include "common/Logger.hpp"
// #include <algorithm>
// #include <cmath>
// #include <string>
// #include <chrono>

// namespace solar {

// SunTracker::SunTracker(Logger& log, Config cfg)
//     : log_(log), cfg_(cfg) {}

// void SunTracker::registerEstimateCallback(EstimateCallback cb) {
//     estimateCb_ = std::move(cb);
// }

// SunTracker::Config SunTracker::config() const {
//     return cfg_;
// }

// void SunTracker::onFrame(const FrameEvent& frame) {
//     // Validate buffer shape for current supported format (grayscale 8-bit)
//     const int w = frame.width;
//     const int h = frame.height;

//     if (w <= 0 || h <= 0) {
//         log_.warn("SunTracker: invalid frame dimensions");
//         return;
//     }

//     const std::size_t expected = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
//     if (frame.data.size() < expected) {
//         log_.warn("SunTracker: frame buffer too small for grayscale format");
//         return;
//     }

//     // Compute centroid of pixels above threshold (simple robust "bright blob" detection)
//     const uint8_t thr = cfg_.threshold;

//     double sumX = 0.0;
//     double sumY = 0.0;
//     std::size_t count = 0;

//     // Optional intensity-weighted centroid for better accuracy
//     // Use weight = pixel value (0..255) to emphasize bright region
//     double wSum = 0.0;
//     double wSumX = 0.0;
//     double wSumY = 0.0;

//     for (int y = 0; y < h; ++y) {
//         const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(w);
//         for (int x = 0; x < w; ++x) {
//             const uint8_t px = frame.data[row + static_cast<std::size_t>(x)];
//             if (px >= thr) {
//                 ++count;
//                 sumX += x;
//                 sumY += y;

//                 const double weight = static_cast<double>(px);
//                 wSum += weight;
//                 wSumX += weight * static_cast<double>(x);
//                 wSumY += weight * static_cast<double>(y);
//             }
//         }
//     }

//     SunEstimate est;
//     est.frame_id = frame.frame_id;
//     est.t_estimate = std::chrono::steady_clock::now();

//     if (count < cfg_.min_pixels) {
//         // Not enough bright pixels → treat as not detected
//         est.cx = 0.0f;
//         est.cy = 0.0f;
//         est.confidence = 0.0f;

//         if (estimateCb_) {
//             estimateCb_(est);
//         }
//         return;
//     }

//     // Choose centroid method:
//     // - intensity-weighted centroid if weights available
//     // - otherwise unweighted centroid
//     double cx = 0.0;
//     double cy = 0.0;

//     if (wSum > 0.0) {
//         cx = wSumX / wSum;
//         cy = wSumY / wSum;
//     } else {
//         cx = sumX / static_cast<double>(count);
//         cy = sumY / static_cast<double>(count);
//     }

//     // Confidence: based on proportion of pixels above threshold (clamped).
//     const double fraction = static_cast<double>(count) / static_cast<double>(expected);
//     float conf = static_cast<float>(std::clamp(fraction * cfg_.confidence_scale, 0.0, 1.0));

//     est.cx = static_cast<float>(cx);
//     est.cy = static_cast<float>(cy);
//     est.confidence = conf;

//     if (estimateCb_) {
//         estimateCb_(est);
//     }
// }

// } // namespace solar




#include "vision/SunTracker.hpp"
#include "common/Logger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>

namespace solar {

SunTracker::SunTracker(Logger& log, Config cfg)
    : log_(log), cfg_(cfg) {}

void SunTracker::registerEstimateCallback(EstimateCallback cb) {
    estimateCb_ = std::move(cb);
}

SunTracker::Config SunTracker::config() const {
    return cfg_;
}

void SunTracker::onFrame(const FrameEvent& frame) {
    // Validate buffer shape for current supported format (grayscale 8-bit)
    const int w = frame.width;
    const int h = frame.height;

    if (w <= 0 || h <= 0) {
        log_.warn("SunTracker: invalid frame dimensions");
        return;
    }

    const std::size_t expected =
        static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

    if (frame.data.size() < expected) {
        log_.warn("SunTracker: frame buffer too small for grayscale format");
        return;
    }

    const uint8_t thr = cfg_.threshold;

    // Unweighted sums (fallback)
    double sumX = 0.0;
    double sumY = 0.0;
    std::size_t count = 0;

    // Intensity-weighted sums (preferred)
    double wSum = 0.0;
    double wSumX = 0.0;
    double wSumY = 0.0;

    for (int y = 0; y < h; ++y) {
        const std::size_t row =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(w);

        for (int x = 0; x < w; ++x) {
            const uint8_t px = frame.data[row + static_cast<std::size_t>(x)];
            if (px >= thr) {
                ++count;
                sumX += static_cast<double>(x);
                sumY += static_cast<double>(y);

                const double weight = static_cast<double>(px);
                wSum += weight;
                wSumX += weight * static_cast<double>(x);
                wSumY += weight * static_cast<double>(y);
            }
        }
    }

    SunEstimate est;
    est.frame_id = frame.frame_id;
    est.t_estimate = std::chrono::steady_clock::now();

    // Not detected
    if (count < cfg_.min_pixels) {
        est.cx = 0.0f;
        est.cy = 0.0f;
        est.confidence = 0.0f;

        if (estimateCb_) {
            estimateCb_(est);
        }
        return;
    }

    // Centroid (weighted if possible)
    double cx = 0.0;
    double cy = 0.0;

    if (wSum > 0.0) {
        cx = wSumX / wSum;
        cy = wSumY / wSum;
    } else {
        cx = sumX / static_cast<double>(count);
        cy = sumY / static_cast<double>(count);
    }

    est.cx = static_cast<float>(cx);
    est.cy = static_cast<float>(cy);

    // -----------------------------
    // Confidence (FIXED)
    // -----------------------------
    // Old method used count/(w*h) which is tiny for small blobs in big images.
    // New method uses "how many bright pixels relative to min_pixels".
    // If count == min_pixels -> confidence ~ 0
    // If count is ~10x min_pixels -> confidence ~ 1
    const std::size_t minPix = std::max<std::size_t>(1, cfg_.min_pixels);
    const double ratio = static_cast<double>(count) / static_cast<double>(minPix);
    double conf01 = (ratio - 1.0) / 9.0; // maps [1 .. 10] to [0 .. 1]
    conf01 = std::clamp(conf01, 0.0, 1.0);

    // Keep your scale knob if you want it (default should be 1.0)
    conf01 = std::clamp(conf01 * static_cast<double>(cfg_.confidence_scale), 0.0, 1.0);

    est.confidence = static_cast<float>(conf01);

    // Optional debug (uncomment if you want)
    // log_.info("TRACK est cx=" + std::to_string(est.cx) +
    //           " cy=" + std::to_string(est.cy) +
    //           " conf=" + std::to_string(est.confidence) +
    //           " count=" + std::to_string(count));

    if (estimateCb_) {
        estimateCb_(est);
    }
}

} // namespace solar