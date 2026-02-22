#pragma once

#include <functional>
#include "common/Types.hpp"

namespace solar {

/**
 * @brief Hardware-agnostic camera interface.
 *
 * Enables backend abstraction (e.g., libcamera or simulation) and supports
 * dependency inversion for unit and integration testing.
 *
 * Implementations must:
 * - Deliver FrameEvent objects via callback (event-driven)
 * - Provide a safe start/stop lifecycle
 */
class ICamera {
public:
    /// @brief Callback type used to deliver captured frames.
    using FrameCallback = std::function<void(const FrameEvent&)>;

    /// @brief Virtual destructor.
    virtual ~ICamera() = default;

    /// @brief Register callback for receiving FrameEvent updates.
    virtual void registerFrameCallback(FrameCallback cb) = 0;

    /**
     * @brief Start frame acquisition.
     * @return true on successful start, false on failure.
     */
    virtual bool start() = 0;

    /**
     * @brief Stop frame acquisition.
     *
     * Must be safe to call multiple times (idempotent).
     */
    virtual void stop() = 0;

    /// @brief Check whether acquisition is currently running.
    virtual bool isRunning() const noexcept = 0;
};

} // namespace solar