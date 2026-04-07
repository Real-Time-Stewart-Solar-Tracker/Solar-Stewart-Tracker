#include "vision/SunTracker.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace solar {

SunTracker::SunTracker(Logger& log, Config cfg)
    : log_(log),
      cfg_(cfg) {
}

void SunTracker::registerEstimateCallback(EstimateCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mtx_);
    estimate_cb_ = std::move(cb);
}

void SunTracker::setThreshold(const std::uint8_t thr) {
    std::lock_guard<std::mutex> lock(cfg_mtx_);
    cfg_.threshold = thr;
}

SunTracker::Config SunTracker::config() const {
    std::lock_guard<std::mutex> lock(cfg_mtx_);
    return cfg_;
}

bool SunTracker::isFrameValid_(const FrameEvent& frame) const {
    if (frame.width <= 0 || frame.height <= 0) {
        log_.warn("SunTracker: invalid frame dimensions");
        return false;
    }

    const std::size_t bpp = bytesPerPixel(frame.format);
    const std::size_t min_stride = static_cast<std::size_t>(frame.width) * bpp;

    if (frame.stride_bytes <= 0) {
        log_.warn("SunTracker: stride_bytes must be positive");
        return false;
    }

    // Guard against malformed buffers before walking them row by row.
    if (static_cast<std::size_t>(frame.stride_bytes) < min_stride) {
        log_.warn(std::string("SunTracker: stride_bytes too small for format ") +
                  pixelFormatName(frame.format));
        return false;
    }

    const std::size_t required_size =
        static_cast<std::size_t>(frame.stride_bytes) *
        static_cast<std::size_t>(frame.height);

    if (frame.data.size() < required_size) {
        log_.warn("SunTracker: frame buffer smaller than stride_bytes * height");
        return false;
    }

    return true;
}

std::uint8_t SunTracker::intensityAt_(const FrameEvent& frame, const int x, const int y) const {
    const std::size_t row =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.stride_bytes);

    switch (frame.format) {
        case PixelFormat::Gray8: {
            const std::size_t idx = row + static_cast<std::size_t>(x);
            return frame.data[idx];
        }

        case PixelFormat::RGB888: {
            const std::size_t idx = row + static_cast<std::size_t>(x) * 3U;
            const std::uint8_t r = frame.data[idx + 0U];
            const std::uint8_t g = frame.data[idx + 1U];
            const std::uint8_t b = frame.data[idx + 2U];

            return static_cast<std::uint8_t>(
                (77U * static_cast<unsigned>(r) +
                 150U * static_cast<unsigned>(g) +
                 29U * static_cast<unsigned>(b)) >> 8U);
        }

        case PixelFormat::BGR888: {
            const std::size_t idx = row + static_cast<std::size_t>(x) * 3U;
            const std::uint8_t b = frame.data[idx + 0U];
            const std::uint8_t g = frame.data[idx + 1U];
            const std::uint8_t r = frame.data[idx + 2U];

            return static_cast<std::uint8_t>(
                (77U * static_cast<unsigned>(r) +
                 150U * static_cast<unsigned>(g) +
                 29U * static_cast<unsigned>(b)) >> 8U);
        }
    }

    return 0U;
}

// Image-thresholding stage for sun-centre estimation.
void SunTracker::onFrame(const FrameEvent& frame) {
    if (!isFrameValid_(frame)) {
        return;
    }

    Config cfg_copy;
    {
        std::lock_guard<std::mutex> lock(cfg_mtx_);
        cfg_copy = cfg_;
    }

    const int width = frame.width;
    const int height = frame.height;
    const std::uint8_t threshold = cfg_copy.threshold;

    double weighted_sum_x = 0.0;
    double weighted_sum_y = 0.0;
    double weight_sum = 0.0;
    std::size_t bright_count = 0U;

    // A simple thresholded centroid is enough for the bright simulated sun spot.
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::uint8_t px = intensityAt_(frame, x, y);
            if (px < threshold) {
                continue;
            }

            ++bright_count;

            const double weight = static_cast<double>(px);
            weight_sum += weight;
            weighted_sum_x += weight * static_cast<double>(x);
            weighted_sum_y += weight * static_cast<double>(y);
        }
    }

    SunEstimate estimate{};
    estimate.frame_id = frame.frame_id;
    estimate.t_estimate = std::chrono::steady_clock::now();

    if (bright_count < cfg_copy.min_pixels || weight_sum <= 0.0) {
        estimate.cx = 0.0F;
        estimate.cy = 0.0F;
        estimate.confidence = 0.0F;

        EstimateCallback cb;
        {
            std::lock_guard<std::mutex> lock(cb_mtx_);
            cb = estimate_cb_;
        }
        if (cb) {
            cb(estimate);
        }
        return;
    }

    estimate.cx = static_cast<float>(weighted_sum_x / weight_sum);
    estimate.cy = static_cast<float>(weighted_sum_y / weight_sum);

    const std::size_t min_pixels = std::max<std::size_t>(1U, cfg_copy.min_pixels);
    double ratio = static_cast<double>(bright_count) / static_cast<double>(min_pixels);
    double confidence = (ratio - 1.0) / 9.0;
    confidence = std::clamp(confidence, 0.0, 1.0);
    confidence = std::clamp(confidence * static_cast<double>(cfg_copy.confidence_scale), 0.0, 1.0);

    estimate.confidence = static_cast<float>(confidence);

    EstimateCallback cb;
    {
        std::lock_guard<std::mutex> lock(cb_mtx_);
        cb = estimate_cb_;
    }
    if (cb) {
        cb(estimate);
    }
}

} // namespace solar