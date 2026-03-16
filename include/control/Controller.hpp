#pragma once

#include <functional>
#include <mutex>

#include "common/Logger.hpp"
#include "common/Types.hpp"

namespace solar {

/**
 * @brief Converts SunEstimate into a platform tilt/pan setpoint.
 *
 * Implements event-driven control logic:
 * - Computes centroid error
 * - Applies proportional gains
 * - Applies deadband and output limits
 * - Emits PlatformSetpoint via callback
 *
 * No internal threads. Callback registration is thread-safe.
 */
class Controller {
public:
    /// @brief Callback type used to deliver computed platform setpoints.
    using SetpointCallback = std::function<void(const PlatformSetpoint&)>;

    /// @brief Controller configuration parameters.
    struct Config {
        /// @brief Expected image width (pixels).
        int width{640};

        /// @brief Expected image height (pixels).
        int height{480};

        /// @brief Deadband around image center (normalized units).
        float deadband{0.02f};

        /// @brief Proportional gain for pan axis.
        float k_pan{0.8f};

        /// @brief Proportional gain for tilt axis.
        float k_tilt{0.8f};

        /// @brief Maximum allowed pan angle (radians).
        float max_pan_rad{0.6f};

        /// @brief Maximum allowed tilt angle (radians).
        float max_tilt_rad{0.6f};

        /// @brief Minimum required confidence to accept an estimate.
        float min_confidence{0.2f};
    };

    /// @brief Construct controller with logger and configuration.
    Controller(Logger& log, Config cfg);

    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    /// @brief Register callback to receive computed setpoints.
    void registerSetpointCallback(SetpointCallback cb);

    /// @brief Process a sun estimate and emit a platform setpoint if valid.
    void onEstimate(const SunEstimate& est);

    /// @brief Get current configuration.
    Config config() const;

private:
    Logger& log_;
    Config cfg_;

    /// @brief Mutex protecting callback registration and invocation.
    mutable std::mutex cbMtx_;

    /// @brief Registered setpoint callback (may be empty).
    SetpointCallback setpointCb_{};
};

} // namespace solar