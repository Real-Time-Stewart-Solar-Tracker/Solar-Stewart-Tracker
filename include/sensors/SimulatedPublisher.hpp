// #pragma once

// #include <atomic>
// #include <cstdint>
// #include <functional>
// #include <mutex>
// #include <random>
// #include <string>
// #include <thread>

// #include "common/Logger.hpp"
// #include "common/Types.hpp"
// #include "sensors/ICamera.hpp"

// namespace solar {

// /**
//  * SimulatedPublisher
//  *
//  * Cross-platform camera simulation for development and testing.
//  *
//  * Purpose (A1):
//  * - Enables builds/tests on Windows and CI without camera hardware
//  * - Keeps SystemManager dependent only on ICamera (DIP/SOLID)
//  *
//  * Behaviour:
//  * - Generates synthetic grayscale frames (1 byte per pixel)
//  * - Emits frames in an event-driven manner using a periodic timer thread
//  *
//  * Note:
//  * - This is not the realtime final path; the libcamera backend is.
//  */
// class SimulatedPublisher final : public ICamera {
// public:
//     struct Config {
//         int width{640};
//         int height{480};
//         int fps{30};
//         bool moving_spot{true};   // simulate sun spot moving across the image
//         float noise_std{5.0f};    // gaussian noise (pixel units)
//         uint8_t background{20};   // background intensity
//         uint8_t spot_value{240};  // spot intensity
//         int spot_radius{12};      // radius in pixels
//     };

//     SimulatedPublisher(Logger& log, Config cfg);

//     SimulatedPublisher(const SimulatedPublisher&) = delete;
//     SimulatedPublisher& operator=(const SimulatedPublisher&) = delete;

//     ~SimulatedPublisher() override;

//     void registerFrameCallback(FrameCallback cb) override;

//     bool start() override;
//     void stop() override;

//     bool isRunning() const noexcept override;

//     Config config() const;

// private:
//     void run_();
//     void generateFrame_(FrameEvent& fe);

//     Logger& log_;
//     Config cfg_;

//     std::atomic<bool> running_{false};
//     std::thread thread_;

//     mutable std::mutex cbMutex_;
//     FrameCallback frameCb_{};

//     uint64_t frameId_{0};

//     // Synthetic motion state
//     float phase_{0.0f};

//     // Random noise
//     std::mt19937 rng_;
//     std::normal_distribution<float> noise_;
// };

// } // namespace solar

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>

#include "common/Logger.hpp"
#include "common/Types.hpp"
#include "sensors/ICamera.hpp"

namespace solar {

/**
 * SimulatedPublisher
 *
 * Cross-platform camera simulation for development and testing.
 *
 * Purpose (A1):
 * - Enables builds/tests on Windows and CI without camera hardware
 * - Keeps SystemManager dependent only on ICamera (DIP/SOLID)
 *
 * Behaviour:
 * - Generates synthetic grayscale frames (1 byte per pixel)
 * - Emits frames in an event-driven manner using a periodic timer thread
 *
 * Note:
 * - This is not the realtime final path; the libcamera backend is.
 */
class SimulatedPublisher final : public ICamera {
public:
    struct Config {
        int width{640};
        int height{480};
        int fps{30};
        bool moving_spot{true};   // simulate sun spot moving across the image
        float noise_std{5.0f};    // gaussian noise (pixel units). <= 0 disables noise.
        uint8_t background{20};   // background intensity
        uint8_t spot_value{240};  // spot intensity
        int spot_radius{12};      // radius in pixels
    };

    SimulatedPublisher(Logger& log, Config cfg);

    SimulatedPublisher(const SimulatedPublisher&) = delete;
    SimulatedPublisher& operator=(const SimulatedPublisher&) = delete;

    ~SimulatedPublisher() override;

    void registerFrameCallback(FrameCallback cb) override;

    bool start() override;
    void stop() override;

    bool isRunning() const noexcept override;

    Config config() const;

private:
    void run_();
    void generateFrame_(FrameEvent& fe);

    Logger& log_;
    Config cfg_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    mutable std::mutex cbMutex_;
    FrameCallback frameCb_{};

    uint64_t frameId_{0};

    // Synthetic motion state
    float phase_{0.0f};

    // Random noise
    std::mt19937 rng_;
    std::optional<std::normal_distribution<float>> noise_;
};

} // namespace solar