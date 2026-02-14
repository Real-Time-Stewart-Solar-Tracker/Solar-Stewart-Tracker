#pragma once

#include <functional>
#include "common/Types.hpp"

namespace solar {

/**
 * ICamera
 *
 * Hardware-agnostic camera interface.
 * - Allows libcamera (real hardware) and simulated camera (tests/dev)
 * - Enables Dependency Inversion (SOLID) and clean unit/integration testing
 *
 * Contract:
 * - registerFrameCallback() must be called before start() (recommended)
 * - start() begins producing FrameEvent via callback (event-driven)
 * - stop() stops producing frames and releases resources
 */
class ICamera {
public:
    using FrameCallback = std::function<void(const FrameEvent&)>;

    virtual ~ICamera() = default;

    virtual void registerFrameCallback(FrameCallback cb) = 0;

    // Returns true on success, false on failure.
    virtual bool start() = 0;

    // Must be safe to call multiple times (idempotent).
    virtual void stop() = 0;

    // Optional but useful for diagnostics.
    virtual bool isRunning() const noexcept = 0;
};

} // namespace solar