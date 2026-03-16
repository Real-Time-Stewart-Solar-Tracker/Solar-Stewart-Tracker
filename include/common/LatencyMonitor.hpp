#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <unordered_map>

#include "common/Logger.hpp"

namespace solar {

/**
 * @brief Measures and aggregates end-to-end latency across the processing pipeline.
 *
 * Tracks timestamps per frame_id and computes:
 * - Total latency (capture → actuate)
 * - Vision latency (capture → estimate)
 * - Control latency (estimate → control)
 * - Actuation latency (control → actuate)
 *
 * Uses std::chrono::steady_clock for monotonic timing.
 * Thread-safe (synchronization implemented in .cpp).
 */
class LatencyMonitor {
public:
    /// @brief Monotonic clock type used for latency measurement.
    using Clock     = std::chrono::steady_clock;

    /// @brief Time point type associated with Clock.
    using TimePoint = Clock::time_point;

    /// @brief Configuration controlling inflight frame tracking limits.
    struct Config {
        /// @brief Maximum number of inflight frames tracked simultaneously.
        std::size_t max_inflight_frames{2000};

        /// @brief Maximum allowed age of inflight frames (0 disables age pruning).
        std::chrono::milliseconds max_inflight_age{2000};
    };

    /// @brief Construct monitor with logger and optional configuration.
    explicit LatencyMonitor(Logger& log) : LatencyMonitor(log, Config{}) {}
    explicit LatencyMonitor(Logger& log, Config cfg);

    /// @brief Destructor.
    ~LatencyMonitor();

    LatencyMonitor(const LatencyMonitor&) = delete;
    LatencyMonitor& operator=(const LatencyMonitor&) = delete;

    /// @brief Record capture timestamp for a frame.
    void onCapture(uint64_t frame_id, TimePoint t_capture);

    /// @brief Record estimate (vision) timestamp for a frame.
    void onEstimate(uint64_t frame_id, TimePoint t_estimate);

    /// @brief Record control timestamp for a frame.
    void onControl(uint64_t frame_id, TimePoint t_control);

    /// @brief Record actuation timestamp for a frame.
    void onActuate(uint64_t frame_id, TimePoint t_actuate);

    /// @brief Print aggregated latency statistics to the logger.
    void printSummary();

private:
    /// @brief Per-frame timestamp storage.
    struct Stamps {
        std::optional<TimePoint> capture;
        std::optional<TimePoint> estimate;
        std::optional<TimePoint> control;
        std::optional<TimePoint> actuate;
    };

    /// @brief Aggregated latency statistics.
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

    /// @brief Ensure frame entry exists and preserve insertion order.
    Stamps& ensureEntry_(uint64_t frame_id);

    /// @brief Prune incomplete frames to enforce memory bounds.
    void pruneInflight_(TimePoint now);

    /// @brief Finalize a frame if all timestamps are present.
    void tryFinalize_(uint64_t frame_id);

    Logger& log_;
    Config cfg_;

    std::unordered_map<uint64_t, Stamps> stamps_;
    std::deque<uint64_t> order_;

    Stats stats_;

    struct ImplMutex;
    std::unique_ptr<ImplMutex> mtx_;
};

} // namespace solar