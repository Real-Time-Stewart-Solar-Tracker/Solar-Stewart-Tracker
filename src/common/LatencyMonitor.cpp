// #include "common/LatencyMonitor.hpp"

// #include <algorithm>
// #include <cmath>
// #include <mutex>
// #include <string>

// namespace solar {

// struct LatencyMonitor::ImplMutex {
//     std::mutex m;
// };

// static double to_ms(LatencyMonitor::TimePoint a, LatencyMonitor::TimePoint b) {
//     // returns (b - a) in milliseconds
//     return std::chrono::duration<double, std::milli>(b - a).count();
// }

// LatencyMonitor::LatencyMonitor(Logger& log)
//     : log_(log), mtx_(new ImplMutex()) {}

// void LatencyMonitor::onCapture(uint64_t frame_id, TimePoint t_capture) {
//     std::lock_guard<std::mutex> lock(mtx_->m);
//     stamps_[frame_id].capture = t_capture;
//     tryFinalize_(frame_id);
// }

// void LatencyMonitor::onEstimate(uint64_t frame_id, TimePoint t_estimate) {
//     std::lock_guard<std::mutex> lock(mtx_->m);
//     stamps_[frame_id].estimate = t_estimate;
//     tryFinalize_(frame_id);
// }

// void LatencyMonitor::onControl(uint64_t frame_id, TimePoint t_control) {
//     std::lock_guard<std::mutex> lock(mtx_->m);
//     stamps_[frame_id].control = t_control;
//     tryFinalize_(frame_id);
// }

// void LatencyMonitor::onActuate(uint64_t frame_id, TimePoint t_actuate) {
//     std::lock_guard<std::mutex> lock(mtx_->m);
//     stamps_[frame_id].actuate = t_actuate;
//     tryFinalize_(frame_id);
// }

// void LatencyMonitor::tryFinalize_(uint64_t frame_id) {
//     auto it = stamps_.find(frame_id);
//     if (it == stamps_.end()) return;

//     const Stamps& s = it->second;
//     if (!s.capture || !s.estimate || !s.control || !s.actuate) {
//         return;
//     }

//     const double L_total   = to_ms(*s.capture, *s.actuate);
//     const double L_vision  = to_ms(*s.capture, *s.estimate);
//     const double L_control = to_ms(*s.estimate, *s.control);
//     const double L_actuate = to_ms(*s.control, *s.actuate);

//     // initialize mins/max once
//     if (!stats_.initialized) {
//         stats_.initialized = true;

//         stats_.total_min_ms = stats_.total_max_ms = L_total;
//         stats_.vision_min_ms = stats_.vision_max_ms = L_vision;
//         stats_.control_min_ms = stats_.control_max_ms = L_control;
//         stats_.actuate_min_ms = stats_.actuate_max_ms = L_actuate;
//     } else {
//         stats_.total_min_ms = std::min(stats_.total_min_ms, L_total);
//         stats_.total_max_ms = std::max(stats_.total_max_ms, L_total);

//         stats_.vision_min_ms = std::min(stats_.vision_min_ms, L_vision);
//         stats_.vision_max_ms = std::max(stats_.vision_max_ms, L_vision);

//         stats_.control_min_ms = std::min(stats_.control_min_ms, L_control);
//         stats_.control_max_ms = std::max(stats_.control_max_ms, L_control);

//         stats_.actuate_min_ms = std::min(stats_.actuate_min_ms, L_actuate);
//         stats_.actuate_max_ms = std::max(stats_.actuate_max_ms, L_actuate);
//     }

//     stats_.count += 1;
//     stats_.total_sum_ms += L_total;
//     stats_.vision_sum_ms += L_vision;
//     stats_.control_sum_ms += L_control;
//     stats_.actuate_sum_ms += L_actuate;

//     // Remove to avoid unbounded growth
//     stamps_.erase(it);
// }

// void LatencyMonitor::printSummary() {
//     std::lock_guard<std::mutex> lock(mtx_->m);

//     if (stats_.count == 0 || !stats_.initialized) {
//         log_.warn("LatencyMonitor: no complete frames recorded");
//         return;
//     }

//     const double avg_total   = stats_.total_sum_ms / static_cast<double>(stats_.count);
//     const double avg_vision  = stats_.vision_sum_ms / static_cast<double>(stats_.count);
//     const double avg_control = stats_.control_sum_ms / static_cast<double>(stats_.count);
//     const double avg_actuate = stats_.actuate_sum_ms / static_cast<double>(stats_.count);

//     const double jitter_total = stats_.total_max_ms - stats_.total_min_ms;

//     log_.info("---- Latency Summary ----");
//     log_.info("Frames: " + std::to_string(stats_.count));

//     log_.info("Total   avg(ms)=" + std::to_string(avg_total) +
//               " min=" + std::to_string(stats_.total_min_ms) +
//               " max=" + std::to_string(stats_.total_max_ms) +
//               " jitter=" + std::to_string(jitter_total));

//     log_.info("Vision  avg(ms)=" + std::to_string(avg_vision) +
//               " min=" + std::to_string(stats_.vision_min_ms) +
//               " max=" + std::to_string(stats_.vision_max_ms));

//     log_.info("Control avg(ms)=" + std::to_string(avg_control) +
//               " min=" + std::to_string(stats_.control_min_ms) +
//               " max=" + std::to_string(stats_.control_max_ms));

//     log_.info("Actuate avg(ms)=" + std::to_string(avg_actuate) +
//               " min=" + std::to_string(stats_.actuate_min_ms) +
//               " max=" + std::to_string(stats_.actuate_max_ms));

//     log_.info("-------------------------");
// }

// } // namespace solar

#include "common/LatencyMonitor.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>

// If some Windows header defined min/max macros, this avoids breaking std::min/max
#ifndef NOMINMAX
#define NOMINMAX
#endif

namespace solar {

// Define the hidden mutex type that was forward-declared in the header
struct LatencyMonitor::ImplMutex {
    std::mutex m;
};

// Convert duration to milliseconds
static double to_ms(LatencyMonitor::TimePoint a, LatencyMonitor::TimePoint b) {
    using Ms = std::chrono::duration<double, std::milli>;
    return std::chrono::duration_cast<Ms>(b - a).count();
}

LatencyMonitor::LatencyMonitor(Logger& log)
    : log_(log),
      mtx_(new ImplMutex{}) {}

void LatencyMonitor::onCapture(uint64_t frame_id, TimePoint t_capture) {
    std::lock_guard<std::mutex> lock(mtx_->m);
    stamps_[frame_id].capture = t_capture;
    tryFinalize_(frame_id);
}

void LatencyMonitor::onEstimate(uint64_t frame_id, TimePoint t_estimate) {
    std::lock_guard<std::mutex> lock(mtx_->m);
    stamps_[frame_id].estimate = t_estimate;
    tryFinalize_(frame_id);
}

void LatencyMonitor::onControl(uint64_t frame_id, TimePoint t_control) {
    std::lock_guard<std::mutex> lock(mtx_->m);
    stamps_[frame_id].control = t_control;
    tryFinalize_(frame_id);
}

void LatencyMonitor::onActuate(uint64_t frame_id, TimePoint t_actuate) {
    std::lock_guard<std::mutex> lock(mtx_->m);
    stamps_[frame_id].actuate = t_actuate;
    tryFinalize_(frame_id);
}

void LatencyMonitor::tryFinalize_(uint64_t frame_id) {
    auto it = stamps_.find(frame_id);
    if (it == stamps_.end()) return;

    const Stamps& s = it->second;

    if (!s.capture || !s.estimate || !s.control || !s.actuate) {
        return; // wait until we have all stamps
    }

    const double L_total   = to_ms(*s.capture,  *s.actuate);
    const double L_vision  = to_ms(*s.capture,  *s.estimate);
    const double L_control = to_ms(*s.estimate, *s.control);
    const double L_actuate = to_ms(*s.control,  *s.actuate);

    if (!stats_.initialized) {
        stats_.initialized = true;

        stats_.total_min_ms = stats_.total_max_ms = L_total;
        stats_.vision_min_ms = stats_.vision_max_ms = L_vision;
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

    // Remove to stop map growth
    stamps_.erase(it);
}

void LatencyMonitor::printSummary() {
    std::lock_guard<std::mutex> lock(mtx_->m);

    if (stats_.count == 0) {
        log_.warn("LatencyMonitor: no complete frames to summarize");
        return;
    }

    const double n = static_cast<double>(stats_.count);

    log_.info("Latency summary over " + std::to_string(stats_.count) + " frames:");
    log_.info("  Total   avg=" + std::to_string(stats_.total_sum_ms / n) +
              " ms, min=" + std::to_string(stats_.total_min_ms) +
              " ms, max=" + std::to_string(stats_.total_max_ms) + " ms");

    log_.info("  Vision  avg=" + std::to_string(stats_.vision_sum_ms / n) +
              " ms, min=" + std::to_string(stats_.vision_min_ms) +
              " ms, max=" + std::to_string(stats_.vision_max_ms) + " ms");

    log_.info("  Control avg=" + std::to_string(stats_.control_sum_ms / n) +
              " ms, min=" + std::to_string(stats_.control_min_ms) +
              " ms, max=" + std::to_string(stats_.control_max_ms) + " ms");

    log_.info("  Actuate avg=" + std::to_string(stats_.actuate_sum_ms / n) +
              " ms, min=" + std::to_string(stats_.actuate_min_ms) +
              " ms, max=" + std::to_string(stats_.actuate_max_ms) + " ms");
}

} // namespace solar