/**
 * @file test_suntracker.cpp
 * @brief Unit tests for SunTracker.
 */

#include "vision/SunTracker.hpp"
#include "src/tests/support/test_common.hpp"

#include <vector>

using solar::FrameEvent;
using solar::Logger;
using solar::PixelFormat;
using solar::SunEstimate;
using solar::SunTracker;

namespace {

FrameEvent makeFrame(int w, int h, int px, int py) {
    FrameEvent f{};
    f.frame_id = 1;
    f.width = w;
    f.height = h;
    f.stride_bytes = w;
    f.format = PixelFormat::Gray8;
    f.data.assign(w * h, 0);

    if (px >= 0 && py >= 0 && px < w && py < h) {
        f.data[py * w + px] = 255;
    }

    return f;
}

} // namespace

TEST_CASE(SunTracker_DetectsSingleBrightPixel) {
    Logger log;
    SunTracker::Config cfg{};
    cfg.threshold = 200;
    cfg.min_pixels = 1;

    SunTracker tracker(log, cfg);

    SunEstimate out{};
    tracker.registerEstimateCallback([&](const SunEstimate& e) {
        out = e;
    });

    auto frame = makeFrame(100, 100, 40, 60);
    tracker.onFrame(frame);

    REQUIRE(out.confidence >= 0.0F);
    REQUIRE(out.cx == 40.0F);
    REQUIRE(out.cy == 60.0F);
}

TEST_CASE(SunTracker_NoBrightPixels_ProducesZeroConfidence) {
    Logger log;
    SunTracker tracker(log, {});

    SunEstimate out{};
    tracker.registerEstimateCallback([&](const SunEstimate& e) {
        out = e;
    });

    FrameEvent f{};
    f.frame_id = 1;
    f.width = 50;
    f.height = 50;
    f.stride_bytes = 50;
    f.format = PixelFormat::Gray8;
    f.data.assign(50 * 50, 10);

    tracker.onFrame(f);

    REQUIRE(out.confidence == 0.0F);
}

TEST_CASE(SunTracker_WeightedCentroid_IsCorrect) {
    Logger log;
    SunTracker::Config cfg{};
    cfg.threshold = 100;
    cfg.min_pixels = 1;

    SunTracker tracker(log, cfg);

    SunEstimate out{};
    tracker.registerEstimateCallback([&](const SunEstimate& e) {
        out = e;
    });

    FrameEvent f{};
    f.frame_id = 1;
    f.width = 10;
    f.height = 10;
    f.stride_bytes = 10;
    f.format = PixelFormat::Gray8;
    f.data.assign(100, 0);

    f.data[2 * 10 + 2] = 200;
    f.data[2 * 10 + 6] = 200;

    tracker.onFrame(f);

    REQUIRE(out.cx == 4.0F);
    REQUIRE(out.cy == 2.0F);
}
