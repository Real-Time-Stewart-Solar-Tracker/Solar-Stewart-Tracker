#include "test_common.hpp"

#include "vision/SunTracker.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

namespace {

solar::FrameEvent makeFrame(int width,
                            int height,
                            int stride_bytes,
                            solar::PixelFormat format,
                            std::vector<std::uint8_t> data) {
    solar::FrameEvent frame{};
    frame.frame_id = 42;
    frame.t_capture = std::chrono::steady_clock::now();
    frame.width = width;
    frame.height = height;
    frame.stride_bytes = stride_bytes;
    frame.format = format;
    frame.data = std::move(data);
    return frame;
}

solar::FrameEvent makeGray8PackedFrame(int width, int height, std::uint8_t fill = 0U) {
    const int stride = width;
    return makeFrame(width,
                     height,
                     stride,
                     solar::PixelFormat::Gray8,
                     std::vector<std::uint8_t>(static_cast<std::size_t>(stride * height), fill));
}

solar::FrameEvent makeGray8PaddedFrame(int width, int height, int stride_bytes, std::uint8_t fill = 0U) {
    return makeFrame(width,
                     height,
                     stride_bytes,
                     solar::PixelFormat::Gray8,
                     std::vector<std::uint8_t>(static_cast<std::size_t>(stride_bytes * height), fill));
}

solar::FrameEvent makeRgbFrame(int width, int height, std::uint8_t fill = 0U) {
    const int stride = width * 3;
    return makeFrame(width,
                     height,
                     stride,
                     solar::PixelFormat::RGB888,
                     std::vector<std::uint8_t>(static_cast<std::size_t>(stride * height), fill));
}

solar::FrameEvent makeBgrFrame(int width, int height, std::uint8_t fill = 0U) {
    const int stride = width * 3;
    return makeFrame(width,
                     height,
                     stride,
                     solar::PixelFormat::BGR888,
                     std::vector<std::uint8_t>(static_cast<std::size_t>(stride * height), fill));
}

void setGrayPixel(solar::FrameEvent& frame, int x, int y, std::uint8_t value) {
    const std::size_t idx =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.stride_bytes) +
        static_cast<std::size_t>(x);
    frame.data[idx] = value;
}

void setRgbPixel(solar::FrameEvent& frame,
                 int x,
                 int y,
                 std::uint8_t r,
                 std::uint8_t g,
                 std::uint8_t b) {
    const std::size_t idx =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.stride_bytes) +
        static_cast<std::size_t>(x) * 3U;
    frame.data[idx + 0U] = r;
    frame.data[idx + 1U] = g;
    frame.data[idx + 2U] = b;
}

void setBgrPixel(solar::FrameEvent& frame,
                 int x,
                 int y,
                 std::uint8_t b,
                 std::uint8_t g,
                 std::uint8_t r) {
    const std::size_t idx =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.stride_bytes) +
        static_cast<std::size_t>(x) * 3U;
    frame.data[idx + 0U] = b;
    frame.data[idx + 1U] = g;
    frame.data[idx + 2U] = r;
}

struct EstimateCapture {
    bool called{false};
    solar::SunEstimate estimate{};
};

} // namespace

TEST(SunTracker_NoBrightPixels_ConfidenceZero) {
    solar::Logger log;
    solar::SunTracker::Config cfg{};
    cfg.threshold = 200;
    cfg.min_pixels = 3;
    cfg.confidence_scale = 1.0f;

    solar::SunTracker tracker(log, cfg);

    EstimateCapture cap{};
    tracker.registerEstimateCallback([&](const solar::SunEstimate& est) {
        cap.called = true;
        cap.estimate = est;
    });

    auto frame = makeGray8PackedFrame(8, 6, 0U);
    tracker.onFrame(frame);

    REQUIRE(cap.called);
    REQUIRE(cap.estimate.frame_id == frame.frame_id);
    REQUIRE(cap.estimate.confidence == 0.0f);
    REQUIRE(cap.estimate.cx == 0.0f);
    REQUIRE(cap.estimate.cy == 0.0f);
}

TEST(SunTracker_BrightSpot_Gray8Packed_CentroidApproxCorrect) {
    solar::Logger log;
    solar::SunTracker::Config cfg{};
    cfg.threshold = 200;
    cfg.min_pixels = 1;
    cfg.confidence_scale = 1.0f;

    solar::SunTracker tracker(log, cfg);

    EstimateCapture cap{};
    tracker.registerEstimateCallback([&](const solar::SunEstimate& est) {
        cap.called = true;
        cap.estimate = est;
    });

    auto frame = makeGray8PackedFrame(10, 10, 0U);

    // 3x3 bright block centred at (6, 4)
    for (int y = 3; y <= 5; ++y) {
        for (int x = 5; x <= 7; ++x) {
            setGrayPixel(frame, x, y, 255U);
        }
    }

    tracker.onFrame(frame);

    REQUIRE(cap.called);
    REQUIRE(cap.estimate.confidence > 0.0f);
    REQUIRE_NEAR(cap.estimate.cx, 6.0f, 0.01f);
    REQUIRE_NEAR(cap.estimate.cy, 4.0f, 0.01f);
}

TEST(SunTracker_BrightSpot_Gray8PaddedStride_CentroidApproxCorrect) {
    solar::Logger log;
    solar::SunTracker::Config cfg{};
    cfg.threshold = 200;
    cfg.min_pixels = 1;
    cfg.confidence_scale = 1.0f;

    solar::SunTracker tracker(log, cfg);

    EstimateCapture cap{};
    tracker.registerEstimateCallback([&](const solar::SunEstimate& est) {
        cap.called = true;
        cap.estimate = est;
    });

    auto frame = makeGray8PaddedFrame(8, 6, 12, 0U);

    // Bright block centred at (2, 3)
    for (int y = 2; y <= 4; ++y) {
        for (int x = 1; x <= 3; ++x) {
            setGrayPixel(frame, x, y, 250U);
        }
    }

    tracker.onFrame(frame);

    REQUIRE(cap.called);
    REQUIRE(cap.estimate.confidence > 0.0f);
    REQUIRE_NEAR(cap.estimate.cx, 2.0f, 0.01f);
    REQUIRE_NEAR(cap.estimate.cy, 3.0f, 0.01f);
}

TEST(SunTracker_BrightSpot_RGB888_CentroidApproxCorrect) {
    solar::Logger log;
    solar::SunTracker::Config cfg{};
    cfg.threshold = 200;
    cfg.min_pixels = 1;
    cfg.confidence_scale = 1.0f;

    solar::SunTracker tracker(log, cfg);

    EstimateCapture cap{};
    tracker.registerEstimateCallback([&](const solar::SunEstimate& est) {
        cap.called = true;
        cap.estimate = est;
    });

    auto frame = makeRgbFrame(9, 7, 0U);

    // Bright white block centred at (4, 2)
    for (int y = 1; y <= 3; ++y) {
        for (int x = 3; x <= 5; ++x) {
            setRgbPixel(frame, x, y, 255U, 255U, 255U);
        }
    }

    tracker.onFrame(frame);

    REQUIRE(cap.called);
    REQUIRE(cap.estimate.confidence > 0.0f);
    REQUIRE_NEAR(cap.estimate.cx, 4.0f, 0.01f);
    REQUIRE_NEAR(cap.estimate.cy, 2.0f, 0.01f);
}

TEST(SunTracker_BrightSpot_BGR888_CentroidApproxCorrect) {
    solar::Logger log;
    solar::SunTracker::Config cfg{};
    cfg.threshold = 200;
    cfg.min_pixels = 1;
    cfg.confidence_scale = 1.0f;

    solar::SunTracker tracker(log, cfg);

    EstimateCapture cap{};
    tracker.registerEstimateCallback([&](const solar::SunEstimate& est) {
        cap.called = true;
        cap.estimate = est;
    });

    auto frame = makeBgrFrame(9, 7, 0U);

    // Bright white block centred at (6, 5)
    for (int y = 4; y <= 6; ++y) {
        for (int x = 5; x <= 7; ++x) {
            setBgrPixel(frame, x, y, 255U, 255U, 255U);
        }
    }

    tracker.onFrame(frame);

    REQUIRE(cap.called);
    REQUIRE(cap.estimate.confidence > 0.0f);
    REQUIRE_NEAR(cap.estimate.cx, 6.0f, 0.01f);
    REQUIRE_NEAR(cap.estimate.cy, 5.0f, 0.01f);
}

TEST(SunTracker_InvalidBuffer_DoesNotEmitEstimate) {
    solar::Logger log;
    solar::SunTracker::Config cfg{};
    cfg.threshold = 200;
    cfg.min_pixels = 1;
    cfg.confidence_scale = 1.0f;

    solar::SunTracker tracker(log, cfg);

    EstimateCapture cap{};
    tracker.registerEstimateCallback([&](const solar::SunEstimate& est) {
        cap.called = true;
        cap.estimate = est;
    });

    solar::FrameEvent frame{};
    frame.frame_id = 77;
    frame.t_capture = std::chrono::steady_clock::now();
    frame.width = 8;
    frame.height = 6;
    frame.stride_bytes = 8;
    frame.format = solar::PixelFormat::Gray8;
    frame.data.resize(10U, 0U); // intentionally too small; should be at least 48 bytes

    tracker.onFrame(frame);

    REQUIRE(!cap.called);
}