#pragma once

/**
 * @file ManualImuCoordinator.hpp
 * @brief Runtime policy helper for manual-input mapping and optional IMU correction.
 */

#include "app/AppConfig.hpp"
#include "common/ManualInputTypes.hpp"
#include "common/Types.hpp"
#include "control/ImuFeedbackMapper.hpp"
#include "control/ImuTiltEstimator.hpp"
#include "control/ManualInputMapper.hpp"
#include "system/TrackerState.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>

namespace solar {

/**
 * @brief Runtime coordinator for manual-input policy and IMU feedback policy.
 *
 * This class intentionally contains only policy/state for:
 * - mapping ADS1115 manual samples into platform setpoints
 * - selecting potentiometer-vs-GUI manual command ownership
 * - storing the latest IMU sample/tilt estimate
 * - applying optional IMU live correction to controller setpoints
 *
 * It does not own threads, queues, hardware publishers, camera flow, actuator
 * flow, or the top-level runtime state machine.
 */
class ManualImuCoordinator {
public:
    /**
     * @brief Source currently allowed to issue manual commands.
     */
    enum class ManualCommandSource : std::uint8_t {
        Pot, ///< Hardware potentiometer path owns manual commands.
        Gui  ///< GUI path owns manual commands.
    };

    /**
     * @brief Construct the coordinator.
     *
     * @param manual_cfg Mapping config for manual potentiometer samples.
     * @param imu_feedback_cfg IMU feedback correction parameters.
     * @param imu_feedback_mode Initial IMU feedback mode.
     * @param max_manual_tilt_rad Maximum allowed manual tilt command in radians.
     * @param max_manual_pan_rad Maximum allowed manual pan command in radians.
     */
    ManualImuCoordinator(const ManualInputMapperConfig& manual_cfg,
                         const app::ImuFeedbackConfig& imu_feedback_cfg,
                         app::ImuFeedbackMode imu_feedback_mode,
                         float max_manual_tilt_rad,
                         float max_manual_pan_rad);

    /**
     * @brief Reset stored IMU state and transient runtime policy state.
     */
    void reset();

    /**
     * @brief Select the active manual command source.
     *
     * @param src Manual command source.
     */
    void setManualCommandSource(ManualCommandSource src) noexcept;

    /**
     * @brief Return the currently active manual command source.
     *
     * @return Active manual command source.
     */
    ManualCommandSource manualCommandSource() const noexcept;

    /**
     * @brief Update IMU feedback mode.
     *
     * @param mode New feedback mode.
     */
    void setImuFeedbackMode(app::ImuFeedbackMode mode) noexcept;

    /**
     * @brief Return the current IMU feedback mode.
     *
     * @return Current IMU feedback mode.
     */
    app::ImuFeedbackMode imuFeedbackMode() const noexcept;

    /**
     * @brief Build a manual platform setpoint from a potentiometer sample.
     *
     * @param sample Raw completed manual sample.
     * @param state Current tracker state.
     * @param frame_id Synthetic frame identifier used for tracing.
     * @param t_control Control-stage timestamp.
     * @param out_setpoint Output setpoint populated on success.
     * @return True if a setpoint was produced.
     */
    bool buildManualSetpointFromPot(const ManualPotSample& sample,
                                    TrackerState state,
                                    std::uint64_t frame_id,
                                    TimePoint t_control,
                                    PlatformSetpoint& out_setpoint);

    /**
     * @brief Build a manual platform setpoint from GUI input.
     *
     * @param tilt_rad Requested tilt in radians.
     * @param pan_rad Requested pan in radians.
     * @param state Current tracker state.
     * @param frame_id Synthetic frame identifier used for tracing.
     * @param t_control Control-stage timestamp.
     * @param out_setpoint Output setpoint populated on success.
     * @return True if a setpoint was produced.
     */
    bool buildManualSetpointFromGui(float tilt_rad,
                                    float pan_rad,
                                    TrackerState state,
                                    std::uint64_t frame_id,
                                    TimePoint t_control,
                                    PlatformSetpoint& out_setpoint);

    /**
     * @brief Update the stored IMU sample and derived tilt estimate.
     *
     * @param sample Latest IMU sample.
     */
    void updateImuSample(const control::ImuSample& sample);

    /**
     * @brief Return the latest valid IMU tilt estimate.
     *
     * @param tilt_rad Output tilt in radians.
     * @return True if a valid tilt estimate is available.
     */
    bool latestImuTiltRad(float& tilt_rad) const;

    /**
     * @brief Apply optional IMU correction to a controller setpoint.
     *
     * If IMU feedback is disabled or no valid estimate exists, the input
     * setpoint is returned unchanged.
     *
     * @param input Input setpoint.
     * @return Corrected or unchanged setpoint.
     */
    PlatformSetpoint applyImuCorrection(const PlatformSetpoint& input) const;

private:
    ManualInputMapper manual_mapper_;
    control::ImuTiltEstimator imu_estimator_;
    control::ImuFeedbackMapper imu_feedback_mapper_;

    std::atomic<ManualCommandSource> manual_source_{ManualCommandSource::Pot};
    std::atomic<app::ImuFeedbackMode> imu_feedback_mode_{app::ImuFeedbackMode::Disabled};

    float max_manual_tilt_rad_{0.35F};
    float max_manual_pan_rad_{0.35F};

    mutable std::mutex imu_mtx_;
    control::ImuSample latest_imu_raw_{};
    float latest_imu_tilt_rad_{0.0F};
    bool latest_imu_valid_{false};
};

} // namespace solar
