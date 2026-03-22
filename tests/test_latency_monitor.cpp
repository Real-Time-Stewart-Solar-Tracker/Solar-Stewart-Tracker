#include "test_common.hpp"

#include "common/LatencyMonitor.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

namespace fs = std::filesystem;

fs::path makeTempCsvPath(const std::string& stem) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return fs::temp_directory_path() /
           (stem + "_" + std::to_string(now) + ".csv");
}

std::string readWholeFile(const fs::path& p) {
    std::ifstream in(p);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::size_t countLines(const std::string& s) {
    std::size_t n = 0;
    for (char c : s) {
        if (c == '\n') ++n;
    }
    return n;
}

} // namespace

TEST(LatencyMonitor_accepts_ordered_timestamps_and_prints) {
    solar::Logger log;
    solar::LatencyMonitor lm(log);

    using Clock = solar::LatencyMonitor::Clock;
    const auto t0 = Clock::now();

    lm.onCapture(1, t0);
    lm.onEstimate(1, t0 + std::chrono::milliseconds(2));
    lm.onControl(1, t0 + std::chrono::milliseconds(3));
    lm.onActuate(1, t0 + std::chrono::milliseconds(5));

    // Should not crash and should summarize one finalized frame.
    lm.printSummary();
}

TEST(LatencyMonitor_handles_out_of_order_calls_without_crashing) {
    solar::Logger log;
    solar::LatencyMonitor lm(log);

    using Clock = solar::LatencyMonitor::Clock;
    const auto t0 = Clock::now();

    // Deliberately out-of-order arrival of timestamps.
    lm.onControl(7,  t0 + std::chrono::milliseconds(3));
    lm.onEstimate(7, t0 + std::chrono::milliseconds(2));
    lm.onCapture(7,  t0 + std::chrono::milliseconds(1));
    lm.onActuate(7,  t0 + std::chrono::milliseconds(4));

    // Should still finalize without crashing.
    lm.printSummary();
}

TEST(LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing) {
    solar::Logger log;

    solar::LatencyMonitor::Config cfg{};
    cfg.max_inflight_frames = 3;
    cfg.max_inflight_age = std::chrono::milliseconds(0); // disable age pruning

    solar::LatencyMonitor lm(log, cfg);

    using Clock = solar::LatencyMonitor::Clock;
    const auto t0 = Clock::now();

    // Feed many incomplete frames to force count-based pruning.
    for (std::uint64_t id = 1; id <= 20; ++id) {
        lm.onCapture(id, t0 + std::chrono::milliseconds(static_cast<int>(id)));
    }

    // No complete frame exists, but this should not crash.
    lm.printSummary();
}

TEST(LatencyMonitor_writes_raw_csv_for_finalized_frames) {
    solar::Logger log;

    const fs::path csvPath = makeTempCsvPath("solar_latency_test");

    solar::LatencyMonitor::Config cfg{};
    cfg.raw_csv_path = csvPath.string();
    cfg.truncate_raw_csv = true;

    {
        solar::LatencyMonitor lm(log, cfg);

        using Clock = solar::LatencyMonitor::Clock;
        const auto t0 = Clock::now();

        lm.onCapture(42,  t0);
        lm.onEstimate(42, t0 + std::chrono::milliseconds(2));
        lm.onControl(42,  t0 + std::chrono::milliseconds(3));
        lm.onActuate(42,  t0 + std::chrono::milliseconds(6));

        lm.printSummary();
    } // ensure file is closed/flushed

    REQUIRE(fs::exists(csvPath));

    const std::string text = readWholeFile(csvPath);
    REQUIRE(!text.empty());

    // Header must exist.
    REQUIRE(text.find("frame_id,capture_us,estimate_us,control_us,actuate_us,vision_ms,control_ms,actuate_ms,total_ms") != std::string::npos);

    // Finalized frame row must exist.
    REQUIRE(text.find("42,") != std::string::npos);

    // Expect header + one data row.
    REQUIRE(countLines(text) >= 2);

    std::error_code ec;
    fs::remove(csvPath, ec);
}

TEST(LatencyMonitor_invokes_observer_when_frame_is_finalized) {
    solar::Logger log;
    solar::LatencyMonitor lm(log);

    bool gotObserver = false;
    std::uint64_t gotId = 0;
    double gotVision = 0.0;
    double gotControl = 0.0;
    double gotActuate = 0.0;

    lm.registerObserver([&](std::uint64_t frame_id,
                            double cap_to_est_ms,
                            double est_to_ctrl_ms,
                            double ctrl_to_act_ms) {
        gotObserver = true;
        gotId = frame_id;
        gotVision = cap_to_est_ms;
        gotControl = est_to_ctrl_ms;
        gotActuate = ctrl_to_act_ms;
    });

    using Clock = solar::LatencyMonitor::Clock;
    const auto t0 = Clock::now();

    lm.onCapture(9,  t0);
    lm.onEstimate(9, t0 + std::chrono::milliseconds(2));
    lm.onControl(9,  t0 + std::chrono::milliseconds(3));
    lm.onActuate(9,  t0 + std::chrono::milliseconds(5));

    REQUIRE(gotObserver);
    REQUIRE(gotId == 9);
    REQUIRE(gotVision >= 0.0);
    REQUIRE(gotControl >= 0.0);
    REQUIRE(gotActuate >= 0.0);
}

TEST(LatencyMonitor_appends_rows_when_truncate_disabled) {
    solar::Logger log;

    const fs::path csvPath = makeTempCsvPath("solar_latency_append_test");

    solar::LatencyMonitor::Config cfg1{};
    cfg1.raw_csv_path = csvPath.string();
    cfg1.truncate_raw_csv = true;

    {
        solar::LatencyMonitor lm(log, cfg1);

        using Clock = solar::LatencyMonitor::Clock;
        const auto t0 = Clock::now();

        lm.onCapture(1,  t0);
        lm.onEstimate(1, t0 + std::chrono::milliseconds(1));
        lm.onControl(1,  t0 + std::chrono::milliseconds(2));
        lm.onActuate(1,  t0 + std::chrono::milliseconds(4));
    }

    solar::LatencyMonitor::Config cfg2{};
    cfg2.raw_csv_path = csvPath.string();
    cfg2.truncate_raw_csv = false;

    {
        solar::LatencyMonitor lm(log, cfg2);

        using Clock = solar::LatencyMonitor::Clock;
        const auto t0 = Clock::now();

        lm.onCapture(2,  t0);
        lm.onEstimate(2, t0 + std::chrono::milliseconds(2));
        lm.onControl(2,  t0 + std::chrono::milliseconds(3));
        lm.onActuate(2,  t0 + std::chrono::milliseconds(5));
    }

    const std::string text = readWholeFile(csvPath);

    REQUIRE(text.find("1,") != std::string::npos);
    REQUIRE(text.find("2,") != std::string::npos);

    // Header should still be present and file should contain at least 3 lines:
    // header + row1 + row2
    REQUIRE(countLines(text) >= 3);

    std::error_code ec;
    fs::remove(csvPath, ec);
}