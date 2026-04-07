/**
 * @file test_latency_monitor.cpp
 * @brief Unit tests for LatencyMonitor.
 */

#include "common/LatencyMonitor.hpp"
#include "common/Logger.hpp"
#include "src/tests/support/test_common.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>

using solar::Clock;
using solar::LatencyMonitor;
using solar::Logger;
using solar::TimePoint;

namespace {

bool nearlyEqual(const double a, const double b, const double eps = 1e-3) {
    return std::fabs(a - b) <= eps;
}

} // namespace

TEST_CASE(LatencyMonitor_accepts_ordered_timestamps_and_prints) {
    Logger log;
    LatencyMonitor lm(log);

    struct Record {
        std::uint64_t frame_id{0};
        double cap_to_est{0.0};
        double est_to_ctrl{0.0};
        double ctrl_to_act{0.0};
    } out{};

    bool called = false;
    lm.registerObserver([&](std::uint64_t frame_id,
                            double cap_to_est,
                            double est_to_ctrl,
                            double ctrl_to_act) {
        called = true;
        out.frame_id = frame_id;
        out.cap_to_est = cap_to_est;
        out.est_to_ctrl = est_to_ctrl;
        out.ctrl_to_act = ctrl_to_act;
    });

    const TimePoint t0 = Clock::now();
    const TimePoint t1 = t0 + std::chrono::milliseconds(2);
    const TimePoint t2 = t1 + std::chrono::milliseconds(3);
    const TimePoint t3 = t2 + std::chrono::milliseconds(4);

    lm.onCapture(1U, t0);
    lm.onEstimate(1U, t1);
    lm.onControl(1U, t2);
    lm.onActuate(1U, t3);

    REQUIRE(called);
    REQUIRE(out.frame_id == 1U);
    REQUIRE(nearlyEqual(out.cap_to_est, 2.0));
    REQUIRE(nearlyEqual(out.est_to_ctrl, 3.0));
    REQUIRE(nearlyEqual(out.ctrl_to_act, 4.0));

    const auto summary = lm.summary();
    REQUIRE(summary.completed_frames == 1U);
    REQUIRE(nearlyEqual(summary.avg_cap_to_est_ms, 2.0));
    REQUIRE(nearlyEqual(summary.avg_est_to_ctrl_ms, 3.0));
    REQUIRE(nearlyEqual(summary.avg_ctrl_to_act_ms, 4.0));
    REQUIRE(nearlyEqual(summary.avg_total_ms, 9.0));
    REQUIRE(nearlyEqual(summary.jitter_total_ms, 0.0));

    const auto records = lm.completedRecords();
    REQUIRE(records.size() == 1U);
    REQUIRE(records.front().frame_id == 1U);
    REQUIRE(nearlyEqual(records.front().total_ms, 9.0));

    lm.printSummary();
}

TEST_CASE(LatencyMonitor_handles_out_of_order_calls_without_crashing) {
    Logger log;
    LatencyMonitor lm(log);

    bool called = false;
    lm.registerObserver([&](std::uint64_t, double, double, double) {
        called = true;
    });

    const TimePoint t0 = Clock::now();
    const TimePoint t1 = t0 + std::chrono::milliseconds(1);
    const TimePoint t2 = t1 + std::chrono::milliseconds(1);
    const TimePoint t3 = t2 + std::chrono::milliseconds(1);

    lm.onControl(2U, t2);
    lm.onCapture(2U, t0);
    lm.onEstimate(2U, t1);
    lm.onActuate(2U, t3);

    REQUIRE(called);
    const auto summary = lm.summary();
    REQUIRE(summary.completed_frames == 1U);
    REQUIRE(nearlyEqual(summary.avg_total_ms, 3.0));
}

TEST_CASE(LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing) {
    Logger log;
    LatencyMonitor lm(log);

    const TimePoint base = Clock::now();

    for (std::uint64_t i = 0; i < 5000U; ++i) {
        lm.onCapture(i, base);
    }

    bool called = false;
    lm.registerObserver([&](std::uint64_t frame_id, double, double, double) {
        called = (frame_id == 4999U);
    });

    lm.onEstimate(4999U, base + std::chrono::milliseconds(1));
    lm.onControl(4999U, base + std::chrono::milliseconds(2));
    lm.onActuate(4999U, base + std::chrono::milliseconds(3));

    REQUIRE(called);
    REQUIRE(lm.summary().completed_frames == 1U);
}

TEST_CASE(LatencyMonitor_reports_jitter_over_multiple_records) {
    Logger log;
    LatencyMonitor lm(log);

    const TimePoint base = Clock::now();

    lm.onCapture(10U, base);
    lm.onEstimate(10U, base + std::chrono::milliseconds(1));
    lm.onControl(10U, base + std::chrono::milliseconds(3));
    lm.onActuate(10U, base + std::chrono::milliseconds(6));

    lm.onCapture(11U, base + std::chrono::milliseconds(10));
    lm.onEstimate(11U, base + std::chrono::milliseconds(12));
    lm.onControl(11U, base + std::chrono::milliseconds(15));
    lm.onActuate(11U, base + std::chrono::milliseconds(19));

    const auto summary = lm.summary();
    REQUIRE(summary.completed_frames == 2U);
    REQUIRE(nearlyEqual(summary.min_cap_to_est_ms, 1.0));
    REQUIRE(nearlyEqual(summary.max_cap_to_est_ms, 2.0));
    REQUIRE(nearlyEqual(summary.jitter_cap_to_est_ms, 1.0));
    REQUIRE(nearlyEqual(summary.min_total_ms, 6.0));
    REQUIRE(nearlyEqual(summary.max_total_ms, 9.0));
    REQUIRE(nearlyEqual(summary.jitter_total_ms, 3.0));
}
