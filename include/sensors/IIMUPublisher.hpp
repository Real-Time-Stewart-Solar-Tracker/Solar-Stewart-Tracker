#pragma once

#include <functional>
#include "common/Types.hpp"

namespace solar {

/**
 * @brief Hardware-agnostic IMU publisher interface.
 *
 * Reads orientation from an inertial measurement unit and emits
 * PlatformSetpoint directly, bypassing the camera/vision/controller pipeline.
 *
 * Implementations must:
 * - Deliver PlatformSetpoint objects via callback (event-driven)
 * - Provide a safe start/stop lifecycle
 */
class IIMUPublisher {
public:
    /// @brief Callback type used to deliver computed platform setpoints.
    using SetpointCallback = std::function<void(const PlatformSetpoint&)>;

    /// @brief Virtual destructor.
    virtual ~IIMUPublisher() = default;

    /// @brief Register callback for receiving PlatformSetpoint updates.
    virtual void registerSetpointCallback(SetpointCallback cb) = 0;

    /**
     * @brief Start IMU acquisition.
     * @return true on successful start, false on failure.
     */
    virtual bool start() = 0;

    /**
     * @brief Stop IMU acquisition.
     *
     * Must be safe to call multiple times (idempotent).
     */
    virtual void stop() = 0;

    /// @brief Check whether acquisition is currently running.
    virtual bool isRunning() const noexcept = 0;
};

} // namespace solar
