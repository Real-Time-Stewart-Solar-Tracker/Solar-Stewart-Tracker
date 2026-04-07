#include "common/LatencyMonitor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace solar {
namespace {

constexpr int kStageCapture = 0;
constexpr int kStageEstimate = 1;
constexpr int kStageControl = 2;

} // namespace

LatencyMonitor::LatencyMonitor(Logger& log)
    : log_(log) {
    // CSV export is configured once at startup and written on shutdown.
    if (const char* env = std::getenv("SOLAR_LATENCY_CSV"); env != nullptr) {
        csv_path_ = env;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    completed_records_.reserve(4096U);
    resetLocked_();
}

void LatencyMonitor::registerObserver(Observer cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    observer_ = std::move(cb);
}

void LatencyMonitor::onCapture(const std::uint64_t frame_id, const TimePoint t) {
    // Capture timestamps open a new inflight record keyed by frame id.
    std::lock_guard<std::mutex> lk(mutex_);
    recordStageLocked_(frame_id, t, kStageCapture);
}

void LatencyMonitor::onEstimate(const std::uint64_t frame_id, const TimePoint t) {
    std::lock_guard<std::mutex> lk(mutex_);
    recordStageLocked_(frame_id, t, kStageEstimate);
}

void LatencyMonitor::onControl(const std::uint64_t frame_id, const TimePoint t) {
    std::lock_guard<std::mutex> lk(mutex_);
    recordStageLocked_(frame_id, t, kStageControl);
}

void LatencyMonitor::onActuate(const std::uint64_t frame_id, const TimePoint t) {
    // Actuation closes the record so only complete traces enter the summary.
    Observer observer_copy{};
    CompletedRecord rec{};
    bool have_record = false;

    {
        std::lock_guard<std::mutex> lk(mutex_);
        const auto it = inflight_.find(frame_id);
        if (it == inflight_.end()) {
            return;
        }

        const StageTimes s = it->second;
        inflight_.erase(it);

        // Ignore incomplete traces; only fully observed paths enter the summary.
        if (!s.have_capture || !s.have_estimate || !s.have_control) {
            return;
        }

        rec.frame_id = frame_id;
        rec.t_capture_ns = nsFromEpoch_(s.t_capture);
        rec.t_estimate_ns = nsFromEpoch_(s.t_estimate);
        rec.t_control_ns = nsFromEpoch_(s.t_control);
        rec.t_actuate_ns = nsFromEpoch_(t);

        rec.cap_to_est_ms = msBetween_(s.t_capture, s.t_estimate);
        rec.est_to_ctrl_ms = msBetween_(s.t_estimate, s.t_control);
        rec.ctrl_to_act_ms = msBetween_(s.t_control, t);
        rec.total_ms = msBetween_(s.t_capture, t);

        updateStatsLocked_(cap_to_est_, rec.cap_to_est_ms);
        updateStatsLocked_(est_to_ctrl_, rec.est_to_ctrl_ms);
        updateStatsLocked_(ctrl_to_act_, rec.ctrl_to_act_ms);
        updateStatsLocked_(total_, rec.total_ms);

        ++completed_;
        completed_records_.push_back(rec);
        observer_copy = observer_;
        have_record = true;
    }

    if (have_record && observer_copy) {
        observer_copy(rec.frame_id, rec.cap_to_est_ms, rec.est_to_ctrl_ms, rec.ctrl_to_act_ms);
    }
}

void LatencyMonitor::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    resetLocked_();
}

void LatencyMonitor::printSummary() const {
    Summary s{};
    std::vector<CompletedRecord> records;

    {
        std::lock_guard<std::mutex> lk(mutex_);
        s = summaryLocked_();
        records = completed_records_;
    }

    if (s.completed_frames == 0U) {
        log_.info("LatencyMonitor: no completed frames");
        return;
    }

    // Summary line group for reportable runtime evidence.
    log_.info("LatencyMonitor summary:");
    log_.info("  completed frames = " + std::to_string(s.completed_frames));

    log_.info("  cap->est avg/min/max/jitter [ms] = " +
              std::to_string(s.avg_cap_to_est_ms) + " / " +
              std::to_string(s.min_cap_to_est_ms) + " / " +
              std::to_string(s.max_cap_to_est_ms) + " / " +
              std::to_string(s.jitter_cap_to_est_ms));

    log_.info("  est->ctrl avg/min/max/jitter [ms] = " +
              std::to_string(s.avg_est_to_ctrl_ms) + " / " +
              std::to_string(s.min_est_to_ctrl_ms) + " / " +
              std::to_string(s.max_est_to_ctrl_ms) + " / " +
              std::to_string(s.jitter_est_to_ctrl_ms));

    log_.info("  ctrl->act avg/min/max/jitter [ms] = " +
              std::to_string(s.avg_ctrl_to_act_ms) + " / " +
              std::to_string(s.min_ctrl_to_act_ms) + " / " +
              std::to_string(s.max_ctrl_to_act_ms) + " / " +
              std::to_string(s.jitter_ctrl_to_act_ms));

    log_.info("  total avg/min/max/jitter [ms] = " +
              std::to_string(s.avg_total_ms) + " / " +
              std::to_string(s.min_total_ms) + " / " +
              std::to_string(s.max_total_ms) + " / " +
              std::to_string(s.jitter_total_ms));

    // Export raw samples after the run so the hot path stays free of file I/O.
    if (!csv_path_.empty()) {
        writeCsvSnapshot_(records);
        log_.info("  raw CSV written to: " + csv_path_);
    }
}

LatencyMonitor::Summary LatencyMonitor::summary() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return summaryLocked_();
}

std::vector<LatencyMonitor::CompletedRecord> LatencyMonitor::completedRecords() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return completed_records_;
}

void LatencyMonitor::pruneLocked_() {
    if (inflight_.size() <= cap_map_size_) {
        return;
    }

    std::vector<std::uint64_t> ids;
    ids.reserve(inflight_.size());
    for (const auto& kv : inflight_) {
        ids.push_back(kv.first);
    }

    std::sort(ids.begin(), ids.end());

    const std::size_t excess = inflight_.size() - cap_map_size_;
    for (std::size_t i = 0; i < excess; ++i) {
        inflight_.erase(ids[i]);
    }
}

void LatencyMonitor::resetLocked_() {
    inflight_.clear();
    completed_records_.clear();
    completed_ = 0U;

    cap_to_est_ = {};
    est_to_ctrl_ = {};
    ctrl_to_act_ = {};
    total_ = {};
}

void LatencyMonitor::recordStageLocked_(const std::uint64_t frame_id, const TimePoint t, const int stage) {
    auto& s = inflight_[frame_id];
    switch (stage) {
    case kStageCapture:
        s.have_capture = true;
        s.t_capture = t;
        break;
    case kStageEstimate:
        s.have_estimate = true;
        s.t_estimate = t;
        break;
    case kStageControl:
        s.have_control = true;
        s.t_control = t;
        break;
    default:
        return;
    }
    pruneLocked_();
}

void LatencyMonitor::updateStatsLocked_(Stats& stats, const double value) {
    if (completed_ == 0U) {
        stats.sum = value;
        stats.min = value;
        stats.max = value;
        return;
    }

    stats.sum += value;
    stats.min = std::min(stats.min, value);
    stats.max = std::max(stats.max, value);
}

LatencyMonitor::Summary LatencyMonitor::summaryLocked_() const {
    Summary out{};
    out.completed_frames = completed_;
    if (completed_ == 0U) {
        return out;
    }

    const double n = static_cast<double>(completed_);

    out.avg_cap_to_est_ms = cap_to_est_.sum / n;
    out.min_cap_to_est_ms = cap_to_est_.min;
    out.max_cap_to_est_ms = cap_to_est_.max;
    out.jitter_cap_to_est_ms = cap_to_est_.max - cap_to_est_.min;

    out.avg_est_to_ctrl_ms = est_to_ctrl_.sum / n;
    out.min_est_to_ctrl_ms = est_to_ctrl_.min;
    out.max_est_to_ctrl_ms = est_to_ctrl_.max;
    out.jitter_est_to_ctrl_ms = est_to_ctrl_.max - est_to_ctrl_.min;

    out.avg_ctrl_to_act_ms = ctrl_to_act_.sum / n;
    out.min_ctrl_to_act_ms = ctrl_to_act_.min;
    out.max_ctrl_to_act_ms = ctrl_to_act_.max;
    out.jitter_ctrl_to_act_ms = ctrl_to_act_.max - ctrl_to_act_.min;

    out.avg_total_ms = total_.sum / n;
    out.min_total_ms = total_.min;
    out.max_total_ms = total_.max;
    out.jitter_total_ms = total_.max - total_.min;

    return out;
}

void LatencyMonitor::writeCsvSnapshot_(const std::vector<CompletedRecord>& records) const {
    if (csv_path_.empty()) {
        return;
    }

    std::ofstream out(csv_path_, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        log_.warn("LatencyMonitor: failed to open CSV path: " + csv_path_);
        return;
    }

    out << "frame_id,t_capture_ns,t_estimate_ns,t_control_ns,t_actuate_ns,"
           "cap_to_est_ms,est_to_ctrl_ms,ctrl_to_act_ms,total_ms\n";

    for (const auto& r : records) {
        out << r.frame_id << ','
            << r.t_capture_ns << ','
            << r.t_estimate_ns << ','
            << r.t_control_ns << ','
            << r.t_actuate_ns << ','
            << r.cap_to_est_ms << ','
            << r.est_to_ctrl_ms << ','
            << r.ctrl_to_act_ms << ','
            << r.total_ms << '\n';
    }
}

double LatencyMonitor::msBetween_(const TimePoint a, const TimePoint b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

std::uint64_t LatencyMonitor::nsFromEpoch_(const TimePoint t) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
}

} // namespace solar
