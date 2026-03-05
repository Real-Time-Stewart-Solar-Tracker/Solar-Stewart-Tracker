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
    std::lock_guard<std::mutex> lk(cbMtx_);
    estimateCb_ = std::move(cb);
}

void SunTracker::setThreshold(uint8_t thr) {
    std::lock_guard<std::mutex> lock(cfgMutex_);
    cfg_.threshold = thr;
}

SunTracker::Config SunTracker::config() const {
    std::lock_guard<std::mutex> lock(cfgMutex_);
    return cfg_;
}

void SunTracker::onFrame(const FrameEvent& frame) {
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

    // Copy config once under lock (prevents cfg_ races)
    Config cfgCopy;
    {
        std::lock_guard<std::mutex> lock(cfgMutex_);
        cfgCopy = cfg_;
    }

    const uint8_t thr = cfgCopy.threshold;

    double sumX = 0.0;
    double sumY = 0.0;
    std::size_t count = 0;

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
    est.frame_id   = frame.frame_id;
    est.t_estimate = std::chrono::steady_clock::now();

    if (count < cfgCopy.min_pixels) {
        est.cx = 0.0f;
        est.cy = 0.0f;
        est.confidence = 0.0f;

        EstimateCallback cb;
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            cb = estimateCb_;
        }
        if (cb) cb(est);
        return;
    }

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

    // Confidence based on bright pixel count vs min_pixels
    const std::size_t minPix = std::max<std::size_t>(1, cfgCopy.min_pixels);
    const double ratio = static_cast<double>(count) / static_cast<double>(minPix);

    double conf01 = (ratio - 1.0) / 9.0; // [1..10] -> [0..1]
    conf01 = std::clamp(conf01, 0.0, 1.0);

    conf01 = std::clamp(conf01 * static_cast<double>(cfgCopy.confidence_scale), 0.0, 1.0);
    est.confidence = static_cast<float>(conf01);

    EstimateCallback cb;
    {
        std::lock_guard<std::mutex> lk(cbMtx_);
        cb = estimateCb_;
    }
    if (cb) cb(est);
}

} // namespace solar