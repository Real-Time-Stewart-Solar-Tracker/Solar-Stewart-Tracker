#include "vision/SunTracker.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

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

bool SunTracker::isFrameValid_(const FrameEvent& frame) const {
    if (frame.width <= 0 || frame.height <= 0) {
        log_.warn("SunTracker: invalid frame dimensions");
        return false;
    }

    const std::size_t bpp = bytesPerPixel(frame.format);
    const std::size_t minStride =
        static_cast<std::size_t>(frame.width) * bpp;

    if (frame.stride_bytes <= 0) {
        log_.warn("SunTracker: stride_bytes must be positive");
        return false;
    }

    if (static_cast<std::size_t>(frame.stride_bytes) < minStride) {
        log_.warn(std::string("SunTracker: stride_bytes too small for format ") +
                  pixelFormatName(frame.format));
        return false;
    }

    const std::size_t required =
        static_cast<std::size_t>(frame.stride_bytes) *
        static_cast<std::size_t>(frame.height);

    if (frame.data.size() < required) {
        log_.warn(std::string("SunTracker: frame buffer too small for format ") +
                  pixelFormatName(frame.format));
        return false;
    }

    return true;
}

uint8_t SunTracker::intensityAt_(const FrameEvent& frame, int x, int y) const {
    const std::size_t row =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.stride_bytes);

    switch (frame.format) {
        case PixelFormat::Gray8: {
            const std::size_t idx = row + static_cast<std::size_t>(x);
            return frame.data[idx];
        }

        case PixelFormat::RGB888: {
            const std::size_t idx =
                row + static_cast<std::size_t>(x) * 3U;
            const uint8_t r = frame.data[idx + 0U];
            const uint8_t g = frame.data[idx + 1U];
            const uint8_t b = frame.data[idx + 2U];

            return static_cast<uint8_t>(
                (77U * static_cast<unsigned>(r) +
                 150U * static_cast<unsigned>(g) +
                 29U * static_cast<unsigned>(b)) >> 8U);
        }

        case PixelFormat::BGR888: {
            const std::size_t idx =
                row + static_cast<std::size_t>(x) * 3U;
            const uint8_t b = frame.data[idx + 0U];
            const uint8_t g = frame.data[idx + 1U];
            const uint8_t r = frame.data[idx + 2U];

            return static_cast<uint8_t>(
                (77U * static_cast<unsigned>(r) +
                 150U * static_cast<unsigned>(g) +
                 29U * static_cast<unsigned>(b)) >> 8U);
        }
    }

    return 0U;
}

void SunTracker::onFrame(const FrameEvent& frame) {
    if (!isFrameValid_(frame)) {
        return;
    }

    Config cfgCopy;
    {
        std::lock_guard<std::mutex> lock(cfgMutex_);
        cfgCopy = cfg_;
    }

    const int w = frame.width;
    const int h = frame.height;
    const uint8_t thr = cfgCopy.threshold;

    double sumX = 0.0;
    double sumY = 0.0;
    std::size_t count = 0;

    double wSum = 0.0;
    double wSumX = 0.0;
    double wSumY = 0.0;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const uint8_t px = intensityAt_(frame, x, y);
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
        if (cb) {
            cb(est);
        }
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

    const std::size_t minPix = std::max<std::size_t>(1, cfgCopy.min_pixels);
    const double ratio =
        static_cast<double>(count) / static_cast<double>(minPix);

    double conf01 = (ratio - 1.0) / 9.0;
    conf01 = std::clamp(conf01, 0.0, 1.0);
    conf01 = std::clamp(conf01 * static_cast<double>(cfgCopy.confidence_scale), 0.0, 1.0);
    est.confidence = static_cast<float>(conf01);

    EstimateCallback cb;
    {
        std::lock_guard<std::mutex> lk(cbMtx_);
        cb = estimateCb_;
    }
    if (cb) {
        cb(est);
    }
}

} // namespace solar