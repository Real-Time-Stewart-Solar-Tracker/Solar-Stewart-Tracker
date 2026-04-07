#pragma once

/**
 * @file LatencyMonitor.hpp
 * @brief Records monotonic per-frame stage timestamps and emits reproducible latency evidence.
 *
 * This monitor tracks four pipeline stages:
 * - capture
 * - estimate
 * - control
 * - actuation
 *
 * It computes per-frame stage latencies and total end-to-end latency using
 * monotonic timestamps only. Raw completed-frame records can be exported to a
 * CSV file by setting the environment variable `SOLAR_LATENCY_CSV` before
 * running the application.
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace solar {

/**
 * @brief Collects per-frame runtime latency evidence.
 */
class LatencyMonitor {
public:
    /**
     * @brief Observer callback for completed latency samples.
     *
     * @param frame_id Frame identifier.
     * @param cap_to_est_ms Capture-to-estimate latency in milliseconds.
     * @param est_to_ctrl_ms Estimate-to-control latency in milliseconds.
     * @param ctrl_to_act_ms Control-to-actuation latency in milliseconds.
     */
    using Observer = std::function<void(std::uint64_t frame_id,
                                        double cap_to_est_ms,
                                        double est_to_ctrl_ms,
                                        double ctrl_to_act_ms)>;

    /**
     * @brief Raw completed per-frame record.
     */
    struct CompletedRecord {
        std::uint64_t frame_id{0U}; ///< Frame identifier.

        std::uint64_t t_capture_ns{0U}; ///< Capture timestamp in nanoseconds.
        std::uint64_t t_estimate_ns{0U};///< Estimate timestamp in nanoseconds.
        std::uint64_t t_control_ns{0U}; ///< Control timestamp in nanoseconds.
        std::uint64_t t_actuate_ns{0U}; ///< Actuation timestamp in nanoseconds.

        double cap_to_est_ms{0.0};  ///< Capture-to-estimate latency in milliseconds.
        double est_to_ctrl_ms{0.0}; ///< Estimate-to-control latency in milliseconds.
        double ctrl_to_act_ms{0.0}; ///< Control-to-actuation latency in milliseconds.
        double total_ms{0.0};       ///< End-to-end latency in milliseconds.
    };

    /**
     * @brief Summary statistics over all completed records.
     */
    struct Summary {
        std::uint64_t completed_frames{0U}; ///< Number of completed frames.

        double avg_cap_to_est_ms{0.0};      ///< Average capture-to-estimate latency.
        double min_cap_to_est_ms{0.0};      ///< Minimum capture-to-estimate latency.
        double max_cap_to_est_ms{0.0};      ///< Maximum capture-to-estimate latency.
        double jitter_cap_to_est_ms{0.0};   ///< Capture-to-estimate jitter (max-min).

        double avg_est_to_ctrl_ms{0.0};     ///< Average estimate-to-control latency.
        double min_est_to_ctrl_ms{0.0};     ///< Minimum estimate-to-control latency.
        double max_est_to_ctrl_ms{0.0};     ///< Maximum estimate-to-control latency.
        double jitter_est_to_ctrl_ms{0.0};  ///< Estimate-to-control jitter (max-min).

        double avg_ctrl_to_act_ms{0.0};     ///< Average control-to-actuation latency.
        double min_ctrl_to_act_ms{0.0};     ///< Minimum control-to-actuation latency.
        double max_ctrl_to_act_ms{0.0};     ///< Maximum control-to-actuation latency.
        double jitter_ctrl_to_act_ms{0.0};  ///< Control-to-actuation jitter (max-min).

        double avg_total_ms{0.0};           ///< Average total end-to-end latency.
        double min_total_ms{0.0};           ///< Minimum total end-to-end latency.
        double max_total_ms{0.0};           ///< Maximum total end-to-end latency.
        double jitter_total_ms{0.0};        ///< Total-latency jitter (max-min).
    };

    /**
     * @brief Construct the latency monitor.
     *
     * @param log Shared logger used for summary output.
     */
    explicit LatencyMonitor(Logger& log);

    /**
     * @brief Register a callback for completed latency records.
     *
     * @param cb Observer callback.
     */
    void registerObserver(Observer cb);

    /**
     * @brief Record the capture timestamp for one frame.
     *
     * @param frame_id Frame identifier.
     * @param t Monotonic capture timestamp.
     */
    void onCapture(std::uint64_t frame_id, TimePoint t);

    /**
     * @brief Record the estimate timestamp for one frame.
     *
     * @param frame_id Frame identifier.
     * @param t Monotonic estimate timestamp.
     */
    void onEstimate(std::uint64_t frame_id, TimePoint t);

    /**
     * @brief Record the control timestamp for one frame.
     *
     * @param frame_id Frame identifier.
     * @param t Monotonic control timestamp.
     */
    void onControl(std::uint64_t frame_id, TimePoint t);

    /**
     * @brief Record the actuation timestamp for one frame.
     *
     * When the actuation timestamp completes a full pipeline record, the record
     * is moved into the completed-record set.
     *
     * @param frame_id Frame identifier.
     * @param t Monotonic actuation timestamp.
     */
    void onActuate(std::uint64_t frame_id, TimePoint t);

    /**
     * @brief Clear all in-flight and completed latency data.
     */
    void reset();

    /**
     * @brief Print a summary and optionally write raw CSV evidence.
     */
    void printSummary() const;

    /**
     * @brief Compute summary statistics over completed records.
     *
     * @return Summary.
     */
    [[nodiscard]] Summary summary() const;

    /**
     * @brief Return a copy of the completed raw records.
     *
     * @return Completed raw records.
     */
    [[nodiscard]] std::vector<CompletedRecord> completedRecords() const;

private:
    struct StageTimes {
        bool have_capture{false};
        bool have_estimate{false};
        bool have_control{false};

        TimePoint t_capture{};
        TimePoint t_estimate{};
        TimePoint t_control{};
    };

    struct Stats {
        double sum{0.0};
        double min{0.0};
        double max{0.0};
    };

    void pruneLocked_();
    void resetLocked_();
    void recordStageLocked_(std::uint64_t frame_id, TimePoint t, int stage);
    void updateStatsLocked_(Stats& stats, double value);
    [[nodiscard]] Summary summaryLocked_() const;
    void writeCsvSnapshot_(const std::vector<CompletedRecord>& records) const;

    static double msBetween_(TimePoint a, TimePoint b);
    static std::uint64_t nsFromEpoch_(TimePoint t);

private:
    Logger& log_;
    mutable std::mutex mutex_;
    Observer observer_{};

    std::unordered_map<std::uint64_t, StageTimes> inflight_{};
    std::size_t cap_map_size_{1024U};

    std::vector<CompletedRecord> completed_records_{};

    std::uint64_t completed_{0U};

    Stats cap_to_est_{};
    Stats est_to_ctrl_{};
    Stats ctrl_to_act_{};
    Stats total_{};

    std::string csv_path_{};
};

} // namespace solar
