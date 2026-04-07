#pragma once

/**
 * @file LibcameraPublisher.hpp
 * @brief libcamera-based implementation of the event-driven camera interface.
 *
 * This backend:
 * - captures frames with libcamera
 * - converts them into the repository's public @ref FrameEvent contract
 * - emits them through the standard @ref ICamera callback
 *
 * Public frame contract emitted by this backend:
 * - @ref PixelFormat::Gray8
 * - packed row layout
 * - @ref FrameEvent::stride_bytes == width
 * - @ref FrameEvent::data contains width * height bytes
 *
 * Internal camera-native formats may differ. That stays hidden here.
 */

#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "sensors/ICamera.hpp"

#if SOLAR_HAVE_LIBCAMERA

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace solar {

/**
 * @brief libcamera-based implementation of @ref ICamera.
 */
class LibcameraPublisher final : public ICamera {
public:
    /**
     * @brief Runtime configuration for libcamera capture.
     */
    struct Config {
        int width{640};            ///< Requested frame width in pixels.
        int height{480};           ///< Requested frame height in pixels.
        int fps{30};               ///< Requested frame rate.
        std::string camera_id{};   ///< Optional explicit camera identifier.
    };

    /**
     * @brief Construct the libcamera publisher.
     *
     * @param log Shared logger.
     * @param cfg Camera runtime configuration.
     */
    LibcameraPublisher(Logger& log, Config cfg);

    /**
     * @brief Destructor stops the worker thread if still running.
     */
    ~LibcameraPublisher() override;

    LibcameraPublisher(const LibcameraPublisher&) = delete;
    LibcameraPublisher& operator=(const LibcameraPublisher&) = delete;

    void registerFrameCallback(FrameCallback cb) override;
    bool start() override;
    void stop() override;
    bool isRunning() const noexcept override;

    /**
     * @brief Return the current backend configuration.
     *
     * @return Current config.
     */
    Config config() const;

private:
    /**
     * @brief Worker-thread body owning the libcamera runtime.
     */
    void run_();

private:
    Logger& log_;
    Config cfg_{};

    std::atomic<bool> running_{false};
    std::thread thread_{};

    mutable std::mutex cb_mutex_;
    FrameCallback frame_cb_{};

    std::atomic<std::uint64_t> frame_id_{0U};

    mutable std::mutex run_mutex_;
    std::condition_variable run_cv_;
};

} // namespace solar

#else

namespace solar {

/**
 * @brief Stub definition used when libcamera support is not compiled in.
 */
class LibcameraPublisher final : public ICamera {
public:
    struct Config {
        int width{640};
        int height{480};
        int fps{30};
        std::string camera_id{};
    };

    LibcameraPublisher(Logger& log, Config cfg)
        : log_(log), cfg_(std::move(cfg)) {}

    ~LibcameraPublisher() override = default;

    void registerFrameCallback(FrameCallback cb) override { frame_cb_ = std::move(cb); }
    bool start() override { log_.warn("LibcameraPublisher: SOLAR_HAVE_LIBCAMERA is OFF"); return false; }
    void stop() override {}
    bool isRunning() const noexcept override { return false; }
    Config config() const { return cfg_; }

private:
    Logger& log_;
    Config cfg_{};
    FrameCallback frame_cb_{};
};

} // namespace solar

#endif // SOLAR_HAVE_LIBCAMERA