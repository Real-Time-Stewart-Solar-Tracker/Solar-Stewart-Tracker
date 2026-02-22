// src/common/LatencyMonitor.cpp
#include "common/LatencyMonitor.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace solar {

struct LatencyMonitor::ImplMutex {
    std::mutex m;
};

static double to_ms(LatencyMonitor::TimePoint a, LatencyMonitor::TimePoint b) {
    using Ms = std::chrono::duration<double, std::milli>;
    return std::chrono::duration_cast<Ms>(b - a).count();
}

LatencyMonitor::LatencyMonitor(Logger& log, Config cfg)
    : log_(log),
      cfg_(cfg),
      mtx_(std::make_unique<ImplMutex>()) {}

LatencyMonitor::~LatencyMonitor() = default;

LatencyMonitor::Stamps& LatencyMonitor::ensureEntry_(uint64_t frame_id) {
    auto it = stamps_.find(frame_id);
    if (it == stamps_.end()) {
        // First time we see this frame_id
        order_.push_back(frame_id);
        it = stamps_.emplace(frame_id, Stamps{}).first;
    }
    return it->second;
}

void LatencyMonitor::pruneInflight_(TimePoint now) {
    // 1) Prune by age (only if enabled)
    if (cfg_.max_inflight_age.count() > 0) {
        while (!order_.empty()) {
            const uint64_t oldest_id = order_.front();
            auto it = stamps_.find(oldest_id);
            if (it == stamps_.end()) {
                order_.pop_front();
                continue;
            }

            const Stamps& s = it->second;

            // If we never got capture time, we cannot age-test reliably.
            // Keep it and let count-based pruning handle it.
            if (!s.capture) break;

            if ((now - *s.capture) > cfg_.max_inflight_age) {
                stamps_.erase(it);
                order_.pop_front();
            } else {
                break; // oldest is within age, so newer ones will be too
            }
        }
    }

    // 2) Prune by count
    while (cfg_.max_inflight_frames > 0 && stamps_.size() > cfg_.max_inflight_frames) {
        if (order_.empty()) break;

        const uint64_t oldest_id = order_.front();
        order_.pop_front();

        auto it = stamps_.find(oldest_id);
        if (it != stamps_.end()) {
            stamps_.erase(it);
        }
    }
}

void LatencyMonitor::onCapture(uint64_t frame_id, TimePoint t_capture) {
    std::lock_guard<std::mutex> lock(mtx_->m);

    Stamps& s = ensureEntry_(frame_id);
    s.capture = t_capture;

    pruneInflight_(t_capture);
    tryFinalize_(frame_id);
}

void LatencyMonitor::onEstimate(uint64_t frame_id, TimePoint t_estimate) {
    std::lock_guard<std::mutex> lock(mtx_->m);

    Stamps& s = ensureEntry_(frame_id);
    s.estimate = t_estimate;

    pruneInflight_(t_estimate);
    tryFinalize_(frame_id);
}

void LatencyMonitor::onControl(uint64_t frame_id, TimePoint t_control) {
    std::lock_guard<std::mutex> lock(mtx_->m);

    Stamps& s = ensureEntry_(frame_id);
    s.control = t_control;

    pruneInflight_(t_control);
    tryFinalize_(frame_id);
}

void LatencyMonitor::onActuate(uint64_t frame_id, TimePoint t_actuate) {
    std::lock_guard<std::mutex> lock(mtx_->m);

    Stamps& s = ensureEntry_(frame_id);
    s.actuate = t_actuate;

    pruneInflight_(t_actuate);
    tryFinalize_(frame_id);
}

void LatencyMonitor::tryFinalize_(uint64_t frame_id) {
    auto it = stamps_.find(frame_id);
    if (it == stamps_.end()) return;

    const Stamps& s = it->second;
    if (!s.capture || !s.estimate || !s.control || !s.actuate) return;

    const double L_total   = to_ms(*s.capture,  *s.actuate);
    const double L_vision  = to_ms(*s.capture,  *s.estimate);
    const double L_control = to_ms(*s.estimate, *s.control);
    const double L_actuate = to_ms(*s.control,  *s.actuate);

    if (!stats_.initialized) {
        stats_.initialized = true;
        stats_.total_min_ms   = stats_.total_max_ms   = L_total;
        stats_.vision_min_ms  = stats_.vision_max_ms  = L_vision;
        stats_.control_min_ms = stats_.control_max_ms = L_control;
        stats_.actuate_min_ms = stats_.actuate_max_ms = L_actuate;
    } else {
        stats_.total_min_ms   = (std::min)(stats_.total_min_ms,   L_total);
        stats_.total_max_ms   = (std::max)(stats_.total_max_ms,   L_total);

        stats_.vision_min_ms  = (std::min)(stats_.vision_min_ms,  L_vision);
        stats_.vision_max_ms  = (std::max)(stats_.vision_max_ms,  L_vision);

        stats_.control_min_ms = (std::min)(stats_.control_min_ms, L_control);
        stats_.control_max_ms = (std::max)(stats_.control_max_ms, L_control);

        stats_.actuate_min_ms = (std::min)(stats_.actuate_min_ms, L_actuate);
        stats_.actuate_max_ms = (std::max)(stats_.actuate_max_ms, L_actuate);
    }

    stats_.count++;
    stats_.total_sum_ms   += L_total;
    stats_.vision_sum_ms  += L_vision;
    stats_.control_sum_ms += L_control;
    stats_.actuate_sum_ms += L_actuate;

    // Erase finalized frame (bounded growth)
    stamps_.erase(it);

    // Note: we do NOT remove frame_id from order_ here to keep O(1) finalize.
    // pruneInflight_ cleans stale IDs from order_ as it encounters them.
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

    log_.info("-------------------------");
}

} // namespace solar