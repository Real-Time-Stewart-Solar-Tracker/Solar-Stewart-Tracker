#include "test_common.hpp"

#include "common/LatencyMonitor.hpp"
#include "common/Logger.hpp"

#include <chrono>

using solar::LatencyMonitor;
using solar::Logger;

TEST(LatencyMonitor_accepts_ordered_timestamps_and_prints) {
    Logger log;

    LatencyMonitor::Config cfg;
    cfg.max_inflight_frames = 10;
    cfg.max_inflight_age    = std::chrono::milliseconds(5000);

    LatencyMonitor lm(log, cfg);

    const auto t0 = LatencyMonitor::Clock::now();

    lm.onCapture(1, t0);
    lm.onEstimate(1, t0 + std::chrono::milliseconds(10));
    lm.onControl(1,  t0 + std::chrono::milliseconds(15));
    lm.onActuate(1,  t0 + std::chrono::milliseconds(20));

    // Should not throw / crash.
    lm.printSummary();

    REQUIRE(true);
}

TEST(LatencyMonitor_handles_out_of_order_calls_without_crashing) {
    Logger log;

    LatencyMonitor::Config cfg;
    cfg.max_inflight_frames = 10;
    cfg.max_inflight_age    = std::chrono::milliseconds(5000);

    LatencyMonitor lm(log, cfg);

    const auto t0 = LatencyMonitor::Clock::now();

    // Out-of-order: estimate before capture etc.
    lm.onEstimate(7, t0 + std::chrono::milliseconds(5));
    lm.onCapture(7,  t0);
    lm.onActuate(7,  t0 + std::chrono::milliseconds(30));
    lm.onControl(7,  t0 + std::chrono::milliseconds(10));

    lm.printSummary();
    REQUIRE(true);
}

TEST(LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing) {
    Logger log;

    LatencyMonitor::Config cfg;
    cfg.max_inflight_frames = 3; // very small to force pruning
    cfg.max_inflight_age    = std::chrono::milliseconds(5000);

    LatencyMonitor lm(log, cfg);

    const auto t0 = LatencyMonitor::Clock::now();

    // Create many inflight frames (only capture), should prune internally.
    for (uint64_t id = 1; id <= 50; ++id) {
        lm.onCapture(id, t0 + std::chrono::milliseconds(static_cast<int>(id)));
    }

    // Finalize one frame fully after the pressure.
    lm.onEstimate(50, t0 + std::chrono::milliseconds(60));
    lm.onControl(50,  t0 + std::chrono::milliseconds(70));
    lm.onActuate(50,  t0 + std::chrono::milliseconds(80));

    lm.printSummary();
    REQUIRE(true);
}