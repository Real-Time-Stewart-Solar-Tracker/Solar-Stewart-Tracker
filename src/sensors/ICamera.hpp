#pragma once

/**
 * @file ICamera.hpp
 * @brief Abstract event-driven camera interface.
 *
 * A camera backend is expected to:
 * - be started explicitly
 * - emit frames via callback
 * - stop explicitly
 *
 * This keeps the camera as a publisher and the rest of the system as subscribers.
 */

#include "common/Types.hpp"

#include <functional>

namespace solar {

/**
 * @brief Abstract camera publisher interface.
 */
class ICamera {
public:
    /**
     * @brief Callback receiving one frame event.
     */
    using FrameCallback = std::function<void(const FrameEvent&)>;

    /**
     * @brief Virtual destructor.
     */
    virtual ~ICamera() = default;

    /**
     * @brief Register the frame callback.
     *
     * @param cb Callback receiving each frame.
     */
    virtual void registerFrameCallback(FrameCallback cb) = 0;

    /**
     * @brief Start the camera publisher.
     *
     * @return True on success.
     */
    virtual bool start() = 0;

    /**
     * @brief Stop the camera publisher.
     */
    virtual void stop() = 0;

    /**
     * @brief Check whether the backend is currently running.
     *
     * @return True if running.
     */
    virtual bool isRunning() const = 0;
};

} // namespace solar