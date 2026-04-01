#pragma once

#if SOLAR_HAVE_LIBCAMERA

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "sensors/ICamera.hpp"

namespace solar {

/**
 * @brief libcamera-based implementation of @ref ICamera for Raspberry Pi / Linux.
 *
 * This backend captures frames using libcamera and emits @ref FrameEvent
 * objects through the standard camera callback interface.
 *
 * Current public frame contract used by this backend:
 * - emitted frames use @ref PixelFormat::Gray8
 * - emitted frames are tightly packed
 * - @ref FrameEvent::stride_bytes is set to @ref FrameEvent::width
 * - @ref FrameEvent::data contains width * height bytes
 *
 * Implementation note:
 * - libcamera may internally deliver frames in a different native format
 *   (for example YUV420)
 * - this backend converts/copies the frame into the repository's public
 *   packed Gray8 FrameEvent contract before invoking the callback
 *
 * Only compiled when `SOLAR_HAVE_LIBCAMERA` is enabled.
 */
class LibcameraPublisher final : public ICamera {
public:
    /**
     * @brief Configuration for libcamera acquisition.
     */
    struct Config {
        /// @brief Requested output frame width in pixels.
        int width{640};

        /// @brief Requested output frame height in pixels.
        int height{480};

        /// @brief Target frames per second (best-effort).
        int fps{30};

        /// @brief Optional backend-specific camera identifier.
        std::string camera_id{};
    };

    /**
     * @brief Construct libcamera backend with logger and configuration.
     * @param log Logger used for runtime status/error reporting.
     * @param cfg Acquisition configuration.
     */
    LibcameraPublisher(Logger& log, Config cfg);

    /**
     * @brief Destructor stops acquisition and joins the internal thread.
     */
    ~LibcameraPublisher() override;

    LibcameraPublisher(const LibcameraPublisher&) = delete;
    LibcameraPublisher& operator=(const LibcameraPublisher&) = delete;

    /**
     * @brief Register callback used to deliver captured @ref FrameEvent objects.
     * @param cb Callback to invoke on each delivered frame.
     */
    void registerFrameCallback(FrameCallback cb) override;

    /**
     * @brief Start camera acquisition.
     * @return true on successful start, false otherwise.
     */
    bool start() override;

    /**
     * @brief Stop camera acquisition.
     *
     * Safe to call multiple times.
     */
    void stop() override;

    /**
     * @brief Check whether acquisition is currently running.
     * @return true if the backend is active.
     */
    bool isRunning() const noexcept override;

    /**
     * @brief Return the current libcamera configuration.
     */
    Config config() const;

private:
    /**
     * @brief Internal acquisition thread body.
     *
     * Opens libcamera, configures the stream, queues requests, receives frame
     * completions, converts the camera buffer into the public Gray8 packed
     * FrameEvent representation, and emits frames through the registered
     * callback.
     */
    void run_();

private:
    Logger& log_;
    Config cfg_;

    /// @brief Running flag shared between public API and worker thread.
    std::atomic<bool> running_{false};

    /// @brief Internal worker thread that owns the libcamera runtime.
    std::thread thread_;

    /// @brief Protects callback registration and callback access.
    mutable std::mutex cbMutex_;

    /// @brief Registered frame callback (may be empty).
    FrameCallback frameCb_{};

    /// @brief Monotonic frame identifier counter.
    std::atomic<uint64_t> frameId_{0};

    /// @brief Synchronisation used to keep the worker thread alive until stop().
    mutable std::mutex runMutex_;
    std::condition_variable runCv_;
};

} // namespace solar

#endif // SOLAR_HAVE_LIBCAMERA