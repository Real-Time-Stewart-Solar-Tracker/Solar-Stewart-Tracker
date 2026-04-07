#include "sensors/imu/Mpu6050Publisher.hpp"

#include <array>
#include <chrono>
#include <cmath>

namespace solar {
namespace {

constexpr float kG        = 9.80665F;
constexpr float kPi       = 3.14159265358979323846F;
constexpr float kDegToRad = kPi / 180.0F;

} // namespace

Mpu6050Publisher::Mpu6050Publisher(II2CDevice& device, Mpu6050Config config)
    : device_(device),
      config_(config) {
}

Mpu6050Publisher::~Mpu6050Publisher() {
    stop();
}

void Mpu6050Publisher::registerEventCallback(CallbackInterface* cb) {
    callback_ = cb;
}

bool Mpu6050Publisher::start() {
    // Bring up I2C first, then arm the GPIO data-ready callback.
    if (running_) {
        return false;
    }
    if (callback_ == nullptr) {
        return false;
    }

    if (!device_.open(config_.i2c)) {
        return false;
    }

    if (!initialise_()) {
        device_.close();
        return false;
    }

    discard_remaining_ = config_.startup_discard_samples;

    // The GPIO data-ready edge is the sole wake event; the I2C transfer is
    // performed only after the edge arrives. No polling loop is involved.
    gpio_pin_.registerCallback([this](const gpiod::edge_event& event) {
        onGpioEvent_(event);
    });

    try {
        gpio_pin_.start(config_.gpio_pin_no, config_.gpio_chip_no);
    } catch (...) {
        device_.close();
        return false;
    }

    running_ = true;
    return true;
}

void Mpu6050Publisher::stop() {
    if (!running_) {
        if (device_.isOpen()) {
            device_.close();
        }
        discard_remaining_ = 0U;
        return;
    }

    gpio_pin_.stop();
    if (device_.isOpen()) {
        device_.close();
    }

    running_ = false;
    discard_remaining_ = 0U;
}

bool Mpu6050Publisher::initialise_() {
    // Verify device identity before writing any configuration registers.
    std::uint8_t who = 0U;
    if (!device_.readByte(kRegWhoAmI, who)) {
        return false;
    }
    if (who != config_.who_am_i_expected) {
        return false;
    }

    // Apply standard MPU-6050/ICM-20600 initialisation sequence.
    if (!device_.writeByte(kRegPowerMgmt,     kWakeValue))          return false;
    if (!device_.writeByte(kRegSampleRateDiv, kSampleRateDivValue)) return false;
    if (!device_.writeByte(kRegConfig,        kConfigValue))        return false;
    if (!device_.writeByte(kRegGyroConfig,    kGyroConfigValue))    return false;
    if (!device_.writeByte(kRegAccelConfig,   kAccelConfigValue))   return false;
    if (!device_.writeByte(kRegIntEnable,     kIntEnableValue))     return false;

    return true;
}

bool Mpu6050Publisher::readSample_(IImuSample& sample) {
    // Confirm the data-ready flag before reading the sample registers.
    std::uint8_t int_status = 0U;
    if (!device_.readByte(kRegIntStatus, int_status)) {
        return false;
    }
    if ((int_status & kDataReadyMask) == 0U) {
        return false;
    }

    // Burst-read the 14-byte accel + temp + gyro register block.
    std::array<std::uint8_t, 14U> data{};
    if (!device_.readBytes(kRegSampleStart, data.data(), data.size())) {
        return false;
    }

    const std::int16_t ax_raw = toInt16_(data[0], data[1]);
    const std::int16_t ay_raw = toInt16_(data[2], data[3]);
    const std::int16_t az_raw = toInt16_(data[4], data[5]);
    // data[6..7] is the temperature register, skipped.
    const std::int16_t gx_raw = toInt16_(data[8],  data[9]);
    const std::int16_t gy_raw = toInt16_(data[10], data[11]);
    const std::int16_t gz_raw = toInt16_(data[12], data[13]);

    sample.ax_mps2 = (static_cast<float>(ax_raw) / config_.accel_lsb_per_g) * kG;
    sample.ay_mps2 = (static_cast<float>(ay_raw) / config_.accel_lsb_per_g) * kG;
    sample.az_mps2 = (static_cast<float>(az_raw) / config_.accel_lsb_per_g) * kG;

    sample.gx_rps = (static_cast<float>(gx_raw) / config_.gyro_lsb_per_dps) * kDegToRad;
    sample.gy_rps = (static_cast<float>(gy_raw) / config_.gyro_lsb_per_dps) * kDegToRad;
    sample.gz_rps = (static_cast<float>(gz_raw) / config_.gyro_lsb_per_dps) * kDegToRad;

    sample.timestamp_us = monotonicNowUs_();
    sample.valid = true;
    return true;
}

void Mpu6050Publisher::onGpioEvent_(const gpiod::edge_event& event) {
    // Only edge-triggered samples reach the callback chain. No polling loop.
    if (!running_) {
        return;
    }

    if (event.type() != gpiod::edge_event::event_type::RISING_EDGE &&
        event.type() != gpiod::edge_event::event_type::FALLING_EDGE) {
        return;
    }

    IImuSample sample{};
    if (!readSample_(sample)) {
        return;
    }

    if (discard_remaining_ > 0U) {
        --discard_remaining_;
        return;
    }

    if (callback_ != nullptr) {
        callback_->hasSample(sample);
    }
}

std::int16_t Mpu6050Publisher::toInt16_(const std::uint8_t msb, const std::uint8_t lsb) {
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(msb) << 8U) |
        static_cast<std::uint16_t>(lsb));
}

std::uint64_t Mpu6050Publisher::monotonicNowUs_() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

} // namespace solar
