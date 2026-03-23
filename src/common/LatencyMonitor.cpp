#include "common/LatencyMonitor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace solar {

struct LatencyMonitor::ImplMutex {
    std::mutex m;
};

namespace {

double to_ms(LatencyMonitor::TimePoint a, LatencyMonitor::TimePoint b) {
    using Ms = std::chrono::duration<double, std::milli>;
    return std::chrono::duration_cast<Ms>(b - a).count();
}

std::int64_t to_us_since_epoch(LatencyMonitor::TimePoint t) {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               t.time_since_epoch())
        .count();
}

} // namespace

LatencyMonitor::LatencyMonitor(Logger& log, Config cfg)
    : log_(log),
      cfg_(std::move(cfg)),
      mtx_(std::make_unique<ImplMutex>()) {
    initRawCsv_();
}

LatencyMonitor::~LatencyMonitor() = default;

LatencyMonitor::Stamps& LatencyMonitor::ensureEntry_(uint64_t frame_id) {
    auto it = stamps_.find(frame_id);
    if (it == stamps_.end()) {
        order_.push_back(frame_id);
        it = stamps_.emplace(frame_id, Stamps{}).first;
    }
    return it->second;
}

void LatencyMonitor::registerObserver(Observer cb) {
    std::lock_guard<std::mutex> lock(mtx_->m);
    observer_ = std::move(cb);
}

void LatencyMonitor::initRawCsv_() {
    if (cfg_.raw_csv_path.empty()) {
        if (const char* env = std::getenv("SOLAR_LATENCY_CSV")) {
            cfg_.raw_csv_path = env;
        }
    }

    if (cfg_.raw_csv_path.empty()) {
        rawCsvEnabled_ = false;
        return;
    }

    rawCsvPath_ = cfg_.raw_csv_path;

    const auto mode = cfg_.truncate_raw_csv ? std::ios::out | std::ios::trunc
                                            : std::ios::out | std::ios::app;
    rawCsv_.open(rawCsvPath_, mode);

    if (!rawCsv_.is_open()) {
        rawCsvEnabled_ = false;
        log_.warn("LatencyMonitor: failed to open raw CSV path: " + rawCsvPath_);
        return;
    }

    rawCsvEnabled_ = true;
    writeRawCsvHeaderIfNeeded_();

    log_.info("LatencyMonitor: raw latency CSV enabled at " + rawCsvPath_);
}

void LatencyMonitor::writeRawCsvHeaderIfNeeded_() {
    if (!rawCsvEnabled_ || !rawCsv_.is_open()) {
        return;
    }

    bool needHeader = cfg_.truncate_raw_csv;

    if (!needHeader) {
        std::ifstream in(rawCsvPath_, std::ios::binary);
        if (!in.good() || in.peek() == std::ifstream::traits_type::eof()) {
            needHeader = true;
        }
    }

    if (needHeader) {
        rawCsv_ << "frame_id,"
                   "capture_us,estimate_us,control_us,actuate_us,"
                   "vision_ms,control_ms,actuate_ms,total_ms\n";
        rawCsv_.flush();
    }
}

void LatencyMonitor::writeRawCsvRow_(uint64_t frame_id,
                                     double total_ms,
                                     double vision_ms,
                                     double control_ms,
                                     double actuate_ms) {
    if (!rawCsvEnabled_ || !rawCsv_.is_open()) {
        return;
    }

    const auto it = stamps_.find(frame_id);
    if (it == stamps_.end()) {
        return;
    }

    const Stamps& s = it->second;
    if (!s.capture || !s.estimate || !s.control || !s.actuate) {
        return;
    }

    rawCsv_ << frame_id << ','
            << to_us_since_epoch(*s.capture) << ','
            << to_us_since_epoch(*s.estimate) << ','
            << to_us_since_epoch(*s.control) << ','
            << to_us_since_epoch(*s.actuate) << ','
            << vision_ms << ','
            << control_ms << ','
            << actuate_ms << ','
            << total_ms << '\n';
}

void LatencyMonitor::pruneInflight_(TimePoint now) {
    // 1) Prune by age
    if (cfg_.max_inflight_age.count() > 0) {
        while (!order_.empty()) {
            const uint64_t oldest_id = order_.front();
            auto it = stamps_.find(oldest_id);
            if (it == stamps_.end()) {
                order_.pop_front();
                continue;
            }

            const Stamps& s = it->second;
            if (!s.capture) {
                break;
            }

            if ((now - *s.capture) > cfg_.max_inflight_age) {
                stamps_.erase(it);
                order_.pop_front();
            } else {
                break;
            }
        }
    }

    // 2) Prune by count
    while (cfg_.max_inflight_frames > 0 && stamps_.size() > cfg_.max_inflight_frames) {
        if (order_.empty()) {
            break;
        }

        const uint64_t oldest_id = order_.front();
        order_.pop_front();

        auto it = stamps_.find(oldest_id);
        if (it != stamps_.end()) {
            stamps_.erase(it);
        }
    }
}

void LatencyMonitor::onCapture(uint64_t frame_id, TimePoint t_capture) {
    std::optional<FinalizedSample> sample;
    Observer observer;

    {
        std::unique_lock<std::mutex> lock(mtx_->m);

        Stamps& s = ensureEntry_(frame_id);
        s.capture = t_capture;

        pruneInflight_(t_capture);
        sample = tryFinalizeLocked_(frame_id);
        observer = observer_;
    }

    if (sample && observer) {
        observer(sample->frame_id,
                 sample->capture_to_estimate_ms,
                 sample->estimate_to_control_ms,
                 sample->control_to_actuate_ms);
    }
}

void LatencyMonitor::onEstimate(uint64_t frame_id, TimePoint t_estimate) {
    std::optional<FinalizedSample> sample;
    Observer observer;

    {
        std::unique_lock<std::mutex> lock(mtx_->m);

        Stamps& s = ensureEntry_(frame_id);
        s.estimate = t_estimate;

        pruneInflight_(t_estimate);
        sample = tryFinalizeLocked_(frame_id);
        observer = observer_;
    }

    if (sample && observer) {
        observer(sample->frame_id,
                 sample->capture_to_estimate_ms,
                 sample->estimate_to_control_ms,
                 sample->control_to_actuate_ms);
    }
}

void LatencyMonitor::onControl(uint64_t frame_id, TimePoint t_control) {
    std::optional<FinalizedSample> sample;
    Observer observer;

    {
        std::unique_lock<std::mutex> lock(mtx_->m);

        Stamps& s = ensureEntry_(frame_id);
        s.control = t_control;

        pruneInflight_(t_control);
        sample = tryFinalizeLocked_(frame_id);
        observer = observer_;
    }

    if (sample && observer) {
        observer(sample->frame_id,
                 sample->capture_to_estimate_ms,
                 sample->estimate_to_control_ms,
                 sample->control_to_actuate_ms);
    }
}

void LatencyMonitor::onActuate(uint64_t frame_id, TimePoint t_actuate) {
    std::optional<FinalizedSample> sample;
    Observer observer;

    {
        std::unique_lock<std::mutex> lock(mtx_->m);

        Stamps& s = ensureEntry_(frame_id);
        s.actuate = t_actuate;

        pruneInflight_(t_actuate);
        sample = tryFinalizeLocked_(frame_id);
        observer = observer_;
    }

    if (sample && observer) {
        observer(sample->frame_id,
                 sample->capture_to_estimate_ms,
                 sample->estimate_to_control_ms,
                 sample->control_to_actuate_ms);
    }
}

std::optional<LatencyMonitor::FinalizedSample>
LatencyMonitor::tryFinalizeLocked_(uint64_t frame_id) {
    auto it = stamps_.find(frame_id);
    if (it == stamps_.end()) {
        return std::nullopt;
    }

    const Stamps& s = it->second;
    if (!s.capture || !s.estimate || !s.control || !s.actuate) {
        return std::nullopt;
    }

    const double total_ms   = to_ms(*s.capture,  *s.actuate);
    const double vision_ms  = to_ms(*s.capture,  *s.estimate);
    const double control_ms = to_ms(*s.estimate, *s.control);
    const double actuate_ms = to_ms(*s.control,  *s.actuate);

    if (!stats_.initialized) {
        stats_.initialized = true;
        stats_.total_min_ms   = stats_.total_max_ms   = total_ms;
        stats_.vision_min_ms  = stats_.vision_max_ms  = vision_ms;
        stats_.control_min_ms = stats_.control_max_ms = control_ms;
        stats_.actuate_min_ms = stats_.actuate_max_ms = actuate_ms;
    } else {
        stats_.total_min_ms   = std::min(stats_.total_min_ms,   total_ms);
        stats_.total_max_ms   = std::max(stats_.total_max_ms,   total_ms);

        stats_.vision_min_ms  = std::min(stats_.vision_min_ms,  vision_ms);
        stats_.vision_max_ms  = std::max(stats_.vision_max_ms,  vision_ms);

        stats_.control_min_ms = std::min(stats_.control_min_ms, control_ms);
        stats_.control_max_ms = std::max(stats_.control_max_ms, control_ms);

        stats_.actuate_min_ms = std::min(stats_.actuate_min_ms, actuate_ms);
        stats_.actuate_max_ms = std::max(stats_.actuate_max_ms, actuate_ms);
    }

    stats_.count++;
    stats_.total_sum_ms   += total_ms;
    stats_.vision_sum_ms  += vision_ms;
    stats_.control_sum_ms += control_ms;
    stats_.actuate_sum_ms += actuate_ms;

    writeRawCsvRow_(frame_id, total_ms, vision_ms, control_ms, actuate_ms);
    if (rawCsvEnabled_) {
        rawCsv_.flush();
    }

    FinalizedSample sample{};
    sample.frame_id = frame_id;
    sample.capture_to_estimate_ms = vision_ms;
    sample.estimate_to_control_ms = control_ms;
    sample.control_to_actuate_ms  = actuate_ms;
    sample.total_ms = total_ms;

    stamps_.erase(it);
    return sample;
}

void LatencyMonitor::printSummary() {
    std::lock_guard<std::mutex> lock(mtx_->m);

    if (stats_.count == 0 || !stats_.initialized) {
        log_.warn("LatencyMonitor: no complete frames to summarize");
        return;
    }

    const double n = static_cast<double>(stats_.count);
    const double jitter_total = stats_.total_max_ms - stats_.total_min_ms;

    log_.info("---- Latency Summary ----");
    log_.info("Frames: " + std::to_string(stats_.count));

    log_.info("Total   avg(ms)=" + std::to_string(stats_.total_sum_ms / n) +
              " min=" + std::to_string(stats_.total_min_ms) +
              " max=" + std::to_string(stats_.total_max_ms) +
              " jitter=" + std::to_string(jitter_total));

    log_.info("Vision  avg(ms)=" + std::to_string(stats_.vision_sum_ms / n) +
              " min=" + std::to_string(stats_.vision_min_ms) +
              " max=" + std::to_string(stats_.vision_max_ms));

    log_.info("Control avg(ms)=" + std::to_string(stats_.control_sum_ms / n) +
              " min=" + std::to_string(stats_.control_min_ms) +
              " max=" + std::to_string(stats_.control_max_ms));

    log_.info("Actuate avg(ms)=" + std::to_string(stats_.actuate_sum_ms / n) +
              " min=" + std::to_string(stats_.actuate_min_ms) +
              " max=" + std::to_string(stats_.actuate_max_ms));

    if (rawCsvEnabled_) {
        log_.info("Raw latency CSV: " + rawCsvPath_);
    }
}

} // namespace solar