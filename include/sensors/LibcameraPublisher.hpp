#pragma once

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
 * LibcameraPublisher
 *
 * Real camera backend using libcamera (Linux/Raspberry Pi).
 * Frames are delivered event-driven via libcamera requestCompleted callback.
 *
 * Notes:
 * - This class is Linux-only (libcamera).
 * - Keep the interface stable; SystemManager depends only on ICamera.
 */
class LibcameraPublisher final : public ICamera {
public:
    struct Config {
        int width{640};
        int height{480};
        int fps{30};              // best-effort frame duration request
        std::string camera_id{};  // optional camera selection by id
    };

    LibcameraPublisher(Logger& log, Config cfg);

    LibcameraPublisher(const LibcameraPublisher&) = delete;
    LibcameraPublisher& operator=(const LibcameraPublisher&) = delete;

    ~LibcameraPublisher() override;

    void registerFrameCallback(FrameCallback cb) override;

    bool start() override;
    void stop() override;

    bool isRunning() const noexcept override;

    Config config() const;

private:
    void run_();

    Logger& log_;
    Config cfg_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    mutable std::mutex cbMutex_;
    FrameCallback frameCb_{};

    uint64_t frameId_{0};

    mutable std::mutex runMutex_;
    std::condition_variable runCv_;
};

} // namespace solar