#include "actuators/PCA9685.hpp"
#include "hal/I2CBusPath.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace solar {
namespace {

constexpr std::uint8_t MODE1      = 0x00U;
constexpr std::uint8_t MODE2      = 0x01U;
constexpr std::uint8_t PRESCALE   = 0xFEU;
constexpr std::uint8_t LED0_ON_L  = 0x06U;

constexpr std::uint8_t MODE1_SLEEP   = 0x10U;
constexpr std::uint8_t MODE1_AI      = 0x20U;
constexpr std::uint8_t MODE1_RESTART = 0x80U;

constexpr std::uint8_t MODE2_OUTDRV  = 0x04U;

constexpr float kOscillatorHz   = 25'000'000.0F;
constexpr float kCountsPerCycle = 4096.0F;

} // namespace

PCA9685::PCA9685(const int i2c_bus, const std::uint8_t i2c_address)
    : bus_(i2c_bus),
      addr_(i2c_address) {
}

PCA9685::~PCA9685() {
    stop();
}

bool PCA9685::start(const float pwm_hz) {
    stop();

    const std::string path = i2cBusPath(bus_);

    fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        fd_ = -1;
        return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, static_cast<int>(addr_)) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    pwm_hz_ = (pwm_hz > 1.0F) ? pwm_hz : 50.0F;

    // The prescaler register controls the PWM period. Its value is derived
    // from the internal 25 MHz oscillator frequency and the 12-bit counter.
    const float prescale_f = (kOscillatorHz / (kCountsPerCycle * pwm_hz_)) - 1.0F;
    const auto prescale = static_cast<std::uint8_t>(
        std::lround(std::max(3.0F, std::min(255.0F, prescale_f))));

    std::uint8_t old_mode1 = 0U;
    if (!readReg8_(MODE1, old_mode1)) {
        stop();
        return false;
    }

    // The PCA9685 datasheet (section 7.3.5) requires the oscillator to be
    // stopped (MODE1_SLEEP = 1) before writing the prescaler register, and
    // specifies a minimum 500 µs stabilisation period before the oscillator
    // is restarted. On a Raspberry Pi I2C bus running at 100 kHz the I2C
    // transactions following the sleep write naturally satisfy this minimum
    // (each byte transfer takes approximately 90 µs). No explicit delay is
    // inserted because the bus timing provides the required margin.
    const std::uint8_t sleep_mode = static_cast<std::uint8_t>((old_mode1 & 0x7FU) | MODE1_SLEEP);
    if (!writeReg8_(MODE1, sleep_mode)) {
        stop();
        return false;
    }

    if (!writeReg8_(PRESCALE, prescale)) {
        stop();
        return false;
    }

    if (!writeReg8_(MODE2, MODE2_OUTDRV)) {
        stop();
        return false;
    }

    if (!writeReg8_(MODE1, static_cast<std::uint8_t>(MODE1_AI))) {
        stop();
        return false;
    }

    if (!writeReg8_(MODE1, static_cast<std::uint8_t>(MODE1_AI | MODE1_RESTART))) {
        stop();
        return false;
    }

    started_ = true;
    return true;
}

void PCA9685::stop() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    started_ = false;
}

bool PCA9685::setPulseWidthUs(const int channel, const float pulse_us) {
    if (!started_) {
        return false;
    }
    if (channel < 0 || channel > 15) {
        return false;
    }

    const float period_us = 1'000'000.0F / pwm_hz_;
    const float clamped_pulse = std::max(0.0F, std::min(pulse_us, period_us));
    const float alpha = clamped_pulse / period_us;

    auto off_count = static_cast<std::uint16_t>(std::lround(alpha * (kCountsPerCycle - 1.0F)));
    if (off_count > 4095U) {
        off_count = 4095U;
    }

    return setPwmCounts_(channel, 0U, off_count);
}

bool PCA9685::isStarted() const noexcept {
    return started_;
}

bool PCA9685::writeReg8_(const std::uint8_t reg, const std::uint8_t value) {
    if (fd_ < 0) {
        return false;
    }

    const std::uint8_t buffer[2] = {reg, value};
    return ::write(fd_, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

bool PCA9685::readReg8_(const std::uint8_t reg, std::uint8_t& value) {
    if (fd_ < 0) {
        return false;
    }

    if (::write(fd_, &reg, 1) != 1) {
        return false;
    }

    return ::read(fd_, &value, 1) == 1;
}

bool PCA9685::setPwmCounts_(const int channel, const std::uint16_t on_count, const std::uint16_t off_count) {
    if (fd_ < 0) {
        return false;
    }
    if (channel < 0 || channel > 15) {
        return false;
    }

    const std::uint8_t reg = static_cast<std::uint8_t>(LED0_ON_L + 4 * channel);
    const std::uint8_t buffer[5] = {
        reg,
        static_cast<std::uint8_t>(on_count & 0xFFU),
        static_cast<std::uint8_t>((on_count >> 8U) & 0x0FU),
        static_cast<std::uint8_t>(off_count & 0xFFU),
        static_cast<std::uint8_t>((off_count >> 8U) & 0x0FU)
    };

    return ::write(fd_, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

} // namespace solar
