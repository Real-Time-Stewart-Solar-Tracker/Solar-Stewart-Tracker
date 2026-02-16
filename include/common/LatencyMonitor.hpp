#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <unordered_map>

#include "common/Logger.hpp"

namespace solar {

/**
 * LatencyMonitor
 *
 * Collects per-frame timestamps (by frame_id) and computes latency metrics:
 * - L_total    = t_actuate  - t_capture
 * - L_vision   = t_estimate - t_capture
 * - L_control  = t_control  - t_estimate
 * - L_actuate  = t_actuate  - t_control
 *
 * Uses steady_clock (monotonic) for correct latency measurement.
 * Thread-safe via internal mutex (implementation in .cpp).
 */
class LatencyMonitor {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit LatencyMonitor(Logger& log);

    LatencyMonitor(const LatencyMonitor&) = delete;
    LatencyMonitor& operator=(const LatencyMonitor&) = delete;

    // Record timestamps (frame_id ties the pipeline together)
    void onCapture(uint64_t frame_id, TimePoint t_capture);
    void onEstimate(uint64_t frame_id, TimePoint t_estimate);
    void onControl(uint64_t frame_id, TimePoint t_control);
    void onActuate(uint64_t frame_id, TimePoint t_actuate);

    // Print summary metrics (call on shutdown)
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

    void tryFinalize_(uint64_t frame_id);

    Logger& log_;
    std::unordered_map<uint64_t, Stamps> stamps_;
    Stats stats_;

    // mutex is in .cpp to keep header minimal
    struct ImplMutex;
    ImplMutex* mtx_;
};

} // namespace solar