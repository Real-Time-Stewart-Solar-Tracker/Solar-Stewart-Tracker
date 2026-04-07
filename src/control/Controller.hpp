#pragma once

/**
 * @file Controller.hpp
 * @brief Converts vision estimates into platform setpoints.
 *
 * This class:
 * - receives SunEstimate (event-driven)
 * - computes tilt/pan setpoints
 * - emits PlatformSetpoint via callback
 *
 * It does NOT:
 * - run threads
 * - own hardware
 * - perform blocking operations
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <functional>
#include <mutex>

namespace solar {

/**
 * @brief Control stage converting image error into tilt/pan commands.
 */
class Controller {
public:
    /**
     * @brief Callback for produced setpoints.
     */
    using SetpointCallback = std::function<void(const PlatformSetpoint&)>;

    /**
     * @brief Controller configuration.
     */
    struct Config {
        int image_width{640};     ///< Image width in pixels.
        int image_height{480};    ///< Image height in pixels.

        float k_pan{0.8F};        ///< Proportional gain for pan.
        float k_tilt{0.8F};       ///< Proportional gain for tilt.

        float max_pan_rad{0.35F}; ///< Max pan angle (rad).
        float max_tilt_rad{0.35F};///< Max tilt angle (rad).

        float deadband{0.02F};    ///< Normalised deadband.
        float min_confidence{0.4F}; ///< Minimum confidence threshold.
    };

    /**
     * @brief Construct controller.
     *
     * @param log Shared logger.
     * @param cfg Controller configuration.
     */
    Controller(Logger& log, Config cfg);

    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    /**
     * @brief Register callback for setpoints.
     *
     * @param cb Setpoint callback.
     */
    void registerSetpointCallback(SetpointCallback cb);

    /**
     * @brief Process one estimate.
     *
     * @param est Input estimate.
     */
    void onEstimate(const SunEstimate& est);

    /**
     * @brief Update confidence threshold at runtime.
     *
     * @param c New minimum confidence.
     */
    void setMinConfidence(float c);

    /**
     * @brief Get current config.
     */
    Config config() const;

private:
    float applyDeadband_(float value) const;
    float clamp_(float v, float lo, float hi) const;

private:
    Logger& log_;
    Config cfg_{};

    mutable std::mutex cfg_mtx_;
    mutable std::mutex cb_mtx_;
    SetpointCallback cb_{};
};

} // namespace solar