#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "common/Logger.hpp"

namespace solar {

/**
 * @brief Measures and aggregates end-to-end latency across the processing pipeline.
 *
 * Tracks timestamps per frame_id and computes:
 * - Total latency (capture -> actuate)
 * - Vision latency (capture -> estimate)
 * - Control latency (estimate -> control)
 * - Actuation latency (control -> actuate)
 *
 * Uses std::chrono::steady_clock for monotonic timing.
 * Thread-safe (synchronization implemented in .cpp).
 */
class LatencyMonitor {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    /**
     * @brief Observer for finalized per-frame segment latencies.
     *
     * Parameters:
     * - frame_id
     * - capture -> estimate (ms)
     * - estimate -> control (ms)
     * - control -> actuate (ms)
     */
    using Observer = std::function<void(uint64_t, double, double, double)>;

    struct Config {
        /// Maximum number of inflight frames tracked simultaneously.
        std::size_t max_inflight_frames{2000};

        /// Maximum allowed age of inflight frames (0 disables age pruning).
        std::chrono::milliseconds max_inflight_age{2000};

        /// Optional raw CSV output path. Empty disables CSV export.
        std::string raw_csv_path{};

        /// If true, truncate CSV on startup. If false, append.
        bool truncate_raw_csv{false};
    };

    explicit LatencyMonitor(Logger& log) : LatencyMonitor(log, Config{}) {}
    explicit LatencyMonitor(Logger& log, Config cfg);
    ~LatencyMonitor();

    LatencyMonitor(const LatencyMonitor&) = delete;
    LatencyMonitor& operator=(const LatencyMonitor&) = delete;

    void onCapture(uint64_t frame_id, TimePoint t_capture);
    void onEstimate(uint64_t frame_id, TimePoint t_estimate);
    void onControl(uint64_t frame_id, TimePoint t_control);
    void onActuate(uint64_t frame_id, TimePoint t_actuate);

    void registerObserver(Observer cb);
    void printSummary();

private:
    struct Stamps {
        std::optional<TimePoint> capture;
        std::optional<TimePoint> estimate;
        std::optional<TimePoint> control;
        std::optional<TimePoint> actuate;
    };

    struct Stats {
        std::size_t count{0};

        double total_sum_ms{0.0};
        double total_min_ms{0.0};
        double total_max_ms{0.0};

        double vision_sum_ms{0.0};
        double vision_min_ms{0.0};
        double vision_max_ms{0.0};

        double control_sum_ms{0.0};
        double control_min_ms{0.0};
        double control_max_ms{0.0};

        double actuate_sum_ms{0.0};
        double actuate_min_ms{0.0};
        double actuate_max_ms{0.0};

        bool initialized{false};
    };

    struct FinalizedSample {
        uint64_t frame_id{0};
        double capture_to_estimate_ms{0.0};
        double estimate_to_control_ms{0.0};
        double control_to_actuate_ms{0.0};
        double total_ms{0.0};
    };

    Stamps& ensureEntry_(uint64_t frame_id);
    void pruneInflight_(TimePoint now);
    std::optional<FinalizedSample> tryFinalizeLocked_(uint64_t frame_id);

    void initRawCsv_();
    void writeRawCsvHeaderIfNeeded_();
    void writeRawCsvRow_(uint64_t frame_id,
                         double total_ms,
                         double vision_ms,
                         double control_ms,
                         double actuate_ms);

    Logger& log_;
    Config cfg_;

    std::unordered_map<uint64_t, Stamps> stamps_;
    std::deque<uint64_t> order_;

    Stats stats_;
    Observer observer_{};

    bool rawCsvEnabled_{false};
    std::string rawCsvPath_;
    std::ofstream rawCsv_;

    struct ImplMutex;
    std::unique_ptr<ImplMutex> mtx_;
};

} // namespace solar