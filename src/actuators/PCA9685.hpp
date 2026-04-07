#pragma once

/**
 * @file PCA9685.hpp
 * @brief Minimal PCA9685 PWM driver wrapper for servo control.
 *
 * This class wraps the low-level Linux I2C access needed to configure and drive
 * a PCA9685 PWM expander.
 *
 * Responsibilities:
 * - initialise the chip
 * - set PWM frequency
 * - write per-channel pulse widths
 *
 * Non-responsibilities:
 * - servo safety logic
 * - kinematics
 * - control
 * - retries, timing loops, or policy decisions
 */

#include <cstdint>

namespace solar {

/**
 * @brief PCA9685 PWM controller wrapper.
 */
class PCA9685 {
public:
    /**
     * @brief Construct the PCA9685 wrapper.
     *
     * @param i2c_bus Linux I2C bus number.
     * @param i2c_address PCA9685 I2C address.
     */
    PCA9685(int i2c_bus, std::uint8_t i2c_address);

    /**
     * @brief Virtual destructor.
     */
    virtual ~PCA9685();

    PCA9685(const PCA9685&) = delete;
    PCA9685& operator=(const PCA9685&) = delete;

    /**
     * @brief Start the PCA9685 and configure PWM frequency.
     *
     * @param pwm_hz Desired PWM frequency in Hz.
     * @return True on success.
     */
    virtual bool start(float pwm_hz);

    /**
     * @brief Stop the device and close its file descriptor.
     */
    virtual void stop();

    /**
     * @brief Set one channel pulse width in microseconds.
     *
     * @param channel PCA9685 output channel [0, 15].
     * @param pulse_us Pulse width in microseconds.
     * @return True on success.
     */
    virtual bool setPulseWidthUs(int channel, float pulse_us);

    /**
     * @brief Check whether the device is started.
     *
     * @return True if started.
     */
    bool isStarted() const noexcept;

protected:
    /**
     * @brief Write one raw 8-bit register.
     *
     * @param reg Register address.
     * @param value Register value.
     * @return True on success.
     */
    bool writeReg8_(std::uint8_t reg, std::uint8_t value);

    /**
     * @brief Read one raw 8-bit register.
     *
     * @param reg Register address.
     * @param value Output register value.
     * @return True on success.
     */
    bool readReg8_(std::uint8_t reg, std::uint8_t& value);

    /**
     * @brief Write one PWM on/off tuple to one channel.
     *
     * @param channel PCA9685 channel.
     * @param on_count ON counter value.
     * @param off_count OFF counter value.
     * @return True on success.
     */
    bool setPwmCounts_(int channel, std::uint16_t on_count, std::uint16_t off_count);

private:
    int bus_{1};
    std::uint8_t addr_{0x40U};
    int fd_{-1};
    bool started_{false};
    float pwm_hz_{50.0F};
};

} // namespace solar