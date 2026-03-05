#include "actuators/PCA9685.hpp"

#include <cmath>

namespace solar::actuators {

PCA9685::PCA9685(std::shared_ptr<solar::hal::II2CDevice> i2c, Config cfg) noexcept
    : i2c_(std::move(i2c)), cfg_(cfg) {}

bool PCA9685::init() noexcept {
    if (!i2c_) return false;

    // MODE2: totem-pole output drive, optional inversion
    uint8_t mode2 = MODE2_OUTDRV;
    if (cfg_.invert_outputs) mode2 |= MODE2_INVRT;
    if (!i2c_->write_reg_u8(REG_MODE2, mode2)) return false;

    // MODE1: auto-increment recommended for multi-byte register writes
    uint8_t mode1 = 0x00;
    if (cfg_.enable_auto_increment) mode1 |= MODE1_AI;
    if (!i2c_->write_reg_u8(REG_MODE1, mode1)) return false;

    return set_pwm_freq(cfg_.pwm_hz);
}

bool PCA9685::set_pwm(uint8_t channel, uint16_t on_count, uint16_t off_count) noexcept {
    if (!i2c_ || channel > 15) return false;

    on_count  &= 0x0FFF;
    off_count &= 0x0FFF;

    const uint8_t base = static_cast<uint8_t>(REG_LED0_ON_L + 4U * channel);
    const uint8_t data[4] = {
        static_cast<uint8_t>(on_count & 0xFF),
        static_cast<uint8_t>((on_count >> 8) & 0x0F),
        static_cast<uint8_t>(off_count & 0xFF),
        static_cast<uint8_t>((off_count >> 8) & 0x0F),
    };

    return i2c_->write_reg_bytes(base, data, sizeof(data));
}

bool PCA9685::set_pulse_us(uint8_t channel, float pulse_us) noexcept {
    if (!i2c_) return false;

    // Convert pulse width to PCA counts (12-bit across the PWM period)
    const float period_us    = 1'000'000.0f / cfg_.pwm_hz;
    const float ticks_per_us = 4096.0f / period_us;

    if (pulse_us < 0.0f) pulse_us = 0.0f;
    if (pulse_us > period_us) pulse_us = period_us;

    const uint16_t off = static_cast<uint16_t>(std::lround(pulse_us * ticks_per_us)) & 0x0FFF;
    return set_pwm(channel, 0, off);
}

bool PCA9685::set_pwm_freq(float hz) noexcept {
    if (!i2c_ || hz <= 1.0f) return false;

    // PCA9685 prescale formula: prescale = round(osc/(4096*hz) - 1)
    const float prescale_f = (cfg_.oscillator_hz / (4096.0f * hz)) - 1.0f;
    int prescale = static_cast<int>(std::lround(prescale_f));
    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;

    uint8_t old_mode1 = 0;
    if (!i2c_->read_reg_bytes(REG_MODE1, &old_mode1, 1)) return false;

    // Enter sleep before writing prescale
    const uint8_t sleep_mode = static_cast<uint8_t>((old_mode1 & 0x7F) | MODE1_SLEEP);
    if (!i2c_->write_reg_u8(REG_MODE1, sleep_mode)) return false;

    if (!i2c_->write_reg_u8(REG_PRESCALE, static_cast<uint8_t>(prescale))) return false;

    // Restore mode1 (wake)
    if (!i2c_->write_reg_u8(REG_MODE1, old_mode1)) return false;

    // Restart (recommended after sleep/prescale change)
    const uint8_t restart = static_cast<uint8_t>(old_mode1 | MODE1_RESTART);
    if (!i2c_->write_reg_u8(REG_MODE1, restart)) return false;

    cfg_.pwm_hz = hz;
    return true;
}

} // namespace solar::actuators