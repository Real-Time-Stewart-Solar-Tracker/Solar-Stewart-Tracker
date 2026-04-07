#include "control/ManualImuCoordinator.hpp"

#include <algorithm>

namespace solar {
namespace {
constexpr float kPi = 3.14159265358979323846F;

float degToRad(const float deg) {
    return deg * kPi / 180.0F;
}
} // namespace

ManualImuCoordinator::ManualImuCoordinator(const ManualInputMapperConfig& manual_cfg,
                                           const app::ImuFeedbackConfig& imu_feedback_cfg,
                                           const app::ImuFeedbackMode imu_feedback_mode,
                                           const float max_manual_tilt_rad,
                                           const float max_manual_pan_rad)
    : manual_mapper_(manual_cfg),
      imu_estimator_(),
      imu_feedback_mapper_({
          imu_feedback_cfg.gain,
          imu_feedback_cfg.deadband_rad,
          imu_feedback_cfg.max_correction_rad}),
      manual_source_(ManualCommandSource::Pot),
      imu_feedback_mode_(imu_feedback_mode),
      max_manual_tilt_rad_(max_manual_tilt_rad),
      max_manual_pan_rad_(max_manual_pan_rad) {
}

void ManualImuCoordinator::reset() {
    manual_source_.store(ManualCommandSource::Pot, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lk(imu_mtx_);
    latest_imu_raw_ = {};
    latest_imu_tilt_rad_ = 0.0F;
    latest_imu_valid_ = false;
}

void ManualImuCoordinator::setManualCommandSource(const ManualCommandSource src) noexcept {
    manual_source_.store(src, std::memory_order_relaxed);
}

ManualImuCoordinator::ManualCommandSource ManualImuCoordinator::manualCommandSource() const noexcept {
    return manual_source_.load(std::memory_order_relaxed);
}

void ManualImuCoordinator::setImuFeedbackMode(const app::ImuFeedbackMode mode) noexcept {
    imu_feedback_mode_.store(mode, std::memory_order_relaxed);
}

app::ImuFeedbackMode ManualImuCoordinator::imuFeedbackMode() const noexcept {
    return imu_feedback_mode_.load(std::memory_order_relaxed);
}

bool ManualImuCoordinator::buildManualSetpointFromPot(const ManualPotSample& sample,
                                                      const TrackerState state,
                                                      const std::uint64_t frame_id,
                                                      const TimePoint t_control,
                                                      PlatformSetpoint& out_setpoint) {
    if (state != TrackerState::MANUAL) {
        return false;
    }
    // Pot input is ignored once the GUI has taken ownership of manual mode.
    if (manualCommandSource() != ManualCommandSource::Pot) {
        return false;
    }

    const auto mapped = manual_mapper_.map(sample);
    out_setpoint.frame_id = frame_id;
    out_setpoint.t_control = t_control;
    out_setpoint.tilt_rad = std::clamp(degToRad(mapped.tilt_deg), -max_manual_tilt_rad_, max_manual_tilt_rad_);
    out_setpoint.pan_rad = std::clamp(degToRad(mapped.pan_deg), -max_manual_pan_rad_, max_manual_pan_rad_);
    return true;
}

bool ManualImuCoordinator::buildManualSetpointFromGui(const float tilt_rad,
                                                      const float pan_rad,
                                                      const TrackerState state,
                                                      const std::uint64_t frame_id,
                                                      const TimePoint t_control,
                                                      PlatformSetpoint& out_setpoint) {
    if (state != TrackerState::MANUAL) {
        return false;
    }

    setManualCommandSource(ManualCommandSource::Gui);

    out_setpoint.frame_id = frame_id;
    out_setpoint.t_control = t_control;
    // Clamp GUI commands to the same manual limits as the potentiometers.
    out_setpoint.tilt_rad = std::clamp(tilt_rad, -max_manual_tilt_rad_, max_manual_tilt_rad_);
    out_setpoint.pan_rad = std::clamp(pan_rad, -max_manual_pan_rad_, max_manual_pan_rad_);
    return true;
}

void ManualImuCoordinator::updateImuSample(const control::ImuSample& sample) {
    const float tilt = imu_estimator_.estimateTiltRad(sample);
    std::lock_guard<std::mutex> lk(imu_mtx_);
    latest_imu_raw_ = sample;
    latest_imu_tilt_rad_ = tilt;
    latest_imu_valid_ = sample.valid;
}

bool ManualImuCoordinator::latestImuTiltRad(float& tilt_rad) const {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    if (!latest_imu_valid_) {
        return false;
    }
    tilt_rad = latest_imu_tilt_rad_;
    return true;
}

PlatformSetpoint ManualImuCoordinator::applyImuCorrection(const PlatformSetpoint& input) const {
    PlatformSetpoint corrected = input;
    if (imuFeedbackMode() != app::ImuFeedbackMode::Live) {
        return corrected;
    }

    float measured_tilt_rad = 0.0F;
    const bool have_imu = latestImuTiltRad(measured_tilt_rad);
    corrected.tilt_rad = imu_feedback_mapper_.apply(corrected.tilt_rad, measured_tilt_rad, have_imu);
    return corrected;
}

} // namespace solar
