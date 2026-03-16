#pragma once

#include "sensors/IIMUPublisher.hpp"
#include "hal/II2CDevice.hpp"
#include "common/Logger.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace solar {

/**
 * @brief MPU6050 IMU publisher using I2C.
 *
 * Reads accelerometer and gyroscope data from an MPU6050 at the configured
 * sample rate, applies a complementary filter to estimate roll and pitch,
 * and emits PlatformSetpoint via callback.
 *
 * Roll  -> tilt_rad (platform tilt axis)
 * Pitch -> pan_rad  (platform pan axis)
 *
 * Wiring (Raspberry Pi):
 *   VCC -> 3.3V, GND -> GND
 *   SCL -> GPIO3 (Pin 5), SDA -> GPIO2 (Pin 3)
 *   AD0 -> GND  (I2C address 0x68)
 */
class MPU6050Publisher final : public IIMUPublisher {
public:
    struct Config {
        /// I2C bus device path.
        std::string i2c_bus{"/dev/i2c-1"};

        /// 7-bit I2C address (AD0 low = 0x68, AD0 high = 0x69).
        uint8_t i2c_addr{0x68};

        /// IMU sampling frequency in Hz.
        uint32_t sample_hz{50};

        /// Complementary filter gyro weight (0–1). Higher = trust gyro more.
        float alpha{0.96f};

        /// Output clamping limits (radians). Match Controller defaults (±0.35 rad).
        float max_tilt_rad{0.35f};
        float max_pan_rad{0.35f};
    };

    MPU6050Publisher(Logger& log, Config cfg);
    ~MPU6050Publisher() override;

    MPU6050Publisher(const MPU6050Publisher&)            = delete;
    MPU6050Publisher& operator=(const MPU6050Publisher&) = delete;

    void registerSetpointCallback(SetpointCallback cb) override;
    bool start() override;
    void stop() override;
    bool isRunning() const noexcept override;

private:
    void loop_();
    bool initHardware_();

    struct RawIMU {
        int16_t ax{0}, ay{0}, az{0};
        int16_t gx{0}, gy{0};
    };
    bool readRaw_(RawIMU& out);

    Logger& log_;
    Config  cfg_;
    std::unique_ptr<solar::hal::II2CDevice> dev_;

    SetpointCallback cb_{};
    std::thread      thread_;
    std::atomic<bool> running_{false};

    float    roll_{0.0f};
    float    pitch_{0.0f};
    bool     first_sample_{true};
    uint64_t frame_id_{0};
};

} // namespace solar
