#include "vision/SunTracker.hpp"
#include "common/Types.hpp"
#include "common/Logger.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

// Use the macros from test_main.cpp
// (They are in the same test executable)
#define TEST(name) void name(); static Register reg_##name(#name, name); void name()
#define REQUIRE(cond) do{ if(!(cond)) throw std::runtime_error("REQUIRE failed: " #cond);}while(0)
#define REQUIRE_NEAR(a,b,eps) do{ auto _a=(a); auto _b=(b); auto _e=(eps); if(!(_a>=_b-_e && _a<=_b+_e)) throw std::runtime_error("REQUIRE_NEAR failed"); }while(0)

// Forward decls from test_main.cpp registry (link-time shared)
struct Register {
    Register(const std::string& name, std::function<void()> fn);
};

using namespace solar;

static FrameEvent makeFrame(int w, int h, uint8_t bg) {
    FrameEvent fe;
    fe.frame_id = 1;
    fe.t_capture = std::chrono::steady_clock::now();
    fe.width = w;
    fe.height = h;
    fe.data.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), bg);
    return fe;
}

static void drawSpot(FrameEvent& fe, int cx, int cy, int r, uint8_t val) {
    const int w = fe.width;
    const int h = fe.height;
    const int r2 = r * r;

    for (int y = cy - r; y <= cy + r; ++y) {
        if (y < 0 || y >= h) continue;
        for (int x = cx - r; x <= cx + r; ++x) {
            if (x < 0 || x >= w) continue;
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= r2) {
                fe.data[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = val;
            }
        }
    }
}

TEST(SunTracker_NoBrightPixels_ConfidenceZero) {
    Logger log;
    SunTracker::Config cfg;
    cfg.threshold = 200;
    cfg.min_pixels = 10;

    SunTracker trk(log, cfg);

    SunEstimate out;
    bool got = false;
    trk.registerEstimateCallback([&](const SunEstimate& e) {
        out = e;
        got = true;
    });

    auto fe = makeFrame(64, 48, 50); // all below threshold
    trk.onFrame(fe);

    REQUIRE(got);
    REQUIRE(out.confidence == 0.0f);
}

TEST(SunTracker_BrightSpot_CentroidApproxCorrect) {
    Logger log;
    SunTracker::Config cfg;
    cfg.threshold = 200;
    cfg.min_pixels = 10;

    SunTracker trk(log, cfg);

    SunEstimate out;
    bool got = false;
    trk.registerEstimateCallback([&](const SunEstimate& e) {
        out = e;
        got = true;
    });

    auto fe = makeFrame(64, 48, 10);
    drawSpot(fe, 20, 30, 4, 240);

    trk.onFrame(fe);

    REQUIRE(got);
    REQUIRE(out.confidence > 0.0f);
    REQUIRE_NEAR(out.cx, 20.0f, 1.5f);
    REQUIRE_NEAR(out.cy, 30.0f, 1.5f);
}

TEST(SunTracker_BrightSpot_HigherAreaHigherConfidence) {
    Logger log;
    SunTracker::Config cfg;
    cfg.threshold = 200;
    cfg.min_pixels = 10;
    cfg.confidence_scale = 20.0f;

    SunTracker trk(log, cfg);

    SunEstimate a, b;
    int count = 0;
    trk.registerEstimateCallback([&](const SunEstimate& e) {
        if (count == 0) a = e;
        else b = e;
        count++;
    });

    // small spot
    auto fe1 = makeFrame(64, 48, 10);
    fe1.frame_id = 1;
    drawSpot(fe1, 32, 24, 2, 240);
    trk.onFrame(fe1);

    // larger spot
    auto fe2 = makeFrame(64, 48, 10);
    fe2.frame_id = 2;
    drawSpot(fe2, 32, 24, 6, 240);
    trk.onFrame(fe2);

    REQUIRE(count == 2);
    REQUIRE(b.confidence >= a.confidence);
}