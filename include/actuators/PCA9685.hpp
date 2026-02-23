#pragma once

#include "hal/II2CDevice.hpp"

#include <cstdint>
#include <memory>

namespace solar::actuators {

/**
 * @file PCA9685.hpp
 * @brief Driver for PCA9685 16-channel 12-bit PWM controller (I2C).
 *
 * Hardware access is performed via hal::II2CDevice to keep the driver
 * unit-testable and platform-independent (DIP).
 *
 * @note Not internally synchronized; use from a single actuator thread.
 */
class PCA9685 {
public:
    /**
     * @brief Configuration for the PWM controller.
     *
     * @param oscillator_hz Internal oscillator frequency (typ. 25 MHz).
     * @param pwm_hz Target PWM frequency (servos typically 50 Hz).
     * @param enable_auto_increment Enables PCA9685 auto-increment (recommended).
     * @param invert_outputs Invert output polarity (rarely needed).
     */
    struct Config {
        float oscillator_hz{25'000'000.0f};
        float pwm_hz{50.0f};
        bool  enable_auto_increment{true};
        bool  invert_outputs{false};
    };

    /**
     * @brief Construct a PCA9685 driver using an I2C device abstraction.
     * @param i2c Shared I2C device instance (must outlive this driver).
     * @param cfg Driver configuration.
     */
    explicit PCA9685(std::shared_ptr<solar::hal::II2CDevice> i2c) noexcept
        : PCA9685(std::move(i2c), Config{}) {}

explicit PCA9685(std::shared_ptr<solar::hal::II2CDevice> i2c, Config cfg) noexcept;
    PCA9685(const PCA9685&)            = delete;
    PCA9685& operator=(const PCA9685&) = delete;

    /**
     * @brief Initialise chip registers and set PWM frequency.
     * @return true on success, false on failure.
     */
    bool init() noexcept;

    /**
     * @brief Set raw PWM counts for a channel.
     *
     * @param channel Channel 0..15.
     * @param on_count  12-bit on tick [0..4095].
     * @param off_count 12-bit off tick [0..4095].
     * @return true on success.
     */
    bool set_pwm(uint8_t channel, uint16_t on_count, uint16_t off_count) noexcept;

    /**
     * @brief Set servo-style pulse width in microseconds.
     *
     * Converts pulse_us to PCA9685 counts using configured pwm_hz.
     *
     * @param channel Channel 0..15.
     * @param pulse_us Pulse width in microseconds.
     * @return true on success.
     */
    bool set_pulse_us(uint8_t channel, float pulse_us) noexcept;

    /** @return Current configured PWM frequency in Hz. */
    float pwm_hz() const noexcept { return cfg_.pwm_hz; }

private:
    // PCA9685 register map
    static constexpr uint8_t REG_MODE1     = 0x00;
    static constexpr uint8_t REG_MODE2     = 0x01;
    static constexpr uint8_t REG_LED0_ON_L = 0x06;
    static constexpr uint8_t REG_PRESCALE  = 0xFE;

    // MODE1 bits
    static constexpr uint8_t MODE1_RESTART = 0x80;
    static constexpr uint8_t MODE1_AI      = 0x20;
    static constexpr uint8_t MODE1_SLEEP   = 0x10;

    // MODE2 bits
    static constexpr uint8_t MODE2_OUTDRV  = 0x04;
    static constexpr uint8_t MODE2_INVRT   = 0x10;

    bool set_pwm_freq(float hz) noexcept;

    std::shared_ptr<solar::hal::II2CDevice> i2c_;
    Config cfg_;
};

} // namespace solar::actuators