#pragma once

#include <cstdint>
#include <opencv2/core.hpp>

#include "common/Types.hpp"

namespace solar::ui {

/**
 * @brief Draw tracking and control overlay onto a camera frame.
 *
 * @param img  Image to draw on (modified in-place).
 * @param est  Latest sun detection estimate.
 * @param sp   Current platform setpoint (tilt/pan).
 * @param cmd  Latest actuator command.
 * @param thr  Threshold value used by the tracker.
 *
 * @details
 * Rendering-only utility. Does not modify system state.
 */
void drawOverlay(cv::Mat& img,
                 const SunEstimate& est,
                 const PlatformSetpoint& sp,
                 const ActuatorCommand& cmd,
                 uint8_t thr);

} // namespace solar::ui