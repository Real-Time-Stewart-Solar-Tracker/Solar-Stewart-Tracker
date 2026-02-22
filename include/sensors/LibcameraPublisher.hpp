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
 * @brief libcamera-based implementation of ICamera (Raspberry Pi backend).
 *
 * Captures frames using libcamera and emits FrameEvent objects via callback.
 * FrameEvent::data contains an 8-bit grayscale buffer of size width × height.
 *
 * Only compiled when SOLAR_HAVE_LIBCAMERA is enabled.
 */
class LibcameraPublisher final : public ICamera {
public:
    /// @brief Configuration for libcamera acquisition.
    struct Config {
        /// @brief Output frame width (pixels).
        int width{640};

        /// @brief Output frame height (pixels).
        int height{480};

        /// @brief Target frames per second (best-effort).
        int fps{30};

        /// @brief Optional camera identifier (backend-specific).
        std::string camera_id{};
    };

    /// @brief Construct libcamera backend with logger and configuration.
    LibcameraPublisher(Logger& log, Config cfg);

    /// @brief Destructor stops acquisition and joins internal thread.
    ~LibcameraPublisher() override;

    LibcameraPublisher(const LibcameraPublisher&) = delete;
    LibcameraPublisher& operator=(const LibcameraPublisher&) = delete;

    /// @brief Register callback for receiving FrameEvent updates.
    void registerFrameCallback(FrameCallback cb) override;

    /**
     * @brief Start camera acquisition.
     * @return true on successful start, false otherwise.
     */
    bool start() override;

    /// @brief Stop camera acquisition (idempotent).
    void stop() override;

    /// @brief Check whether acquisition is currently running.
    bool isRunning() const noexcept override;

    /// @brief Get current configuration.
    Config config() const;

private:
    /// @brief Internal acquisition loop.
    void run_();

    Logger& log_;
    Config cfg_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    /// @brief Mutex protecting callback registration and invocation.
    mutable std::mutex cbMutex_;

    /// @brief Registered frame callback (may be empty).
    FrameCallback frameCb_{};

    /// @brief Monotonic frame identifier counter.
    std::atomic<uint64_t> frameId_{0};

    mutable std::mutex runMutex_;
    std::condition_variable runCv_;
};

} // namespace solar

#endif // SOLAR_HAVE_LIBCAMERA