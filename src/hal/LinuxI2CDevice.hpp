#pragma once

/**
 * @file LinuxI2CDevice.hpp
 * @brief Linux /dev/i2c-* implementation of II2CDevice.
 */

#include "hal/II2CDevice.hpp"

namespace solar {

/**
 * @brief Raw Linux I2C device implementation.
 *
 * This class owns exactly one open file descriptor at a time and talks to one
 * selected I2C slave address on one Linux I2C bus.
 */
class LinuxI2CDevice final : public II2CDevice {
public:
    /**
     * @brief Construct a closed Linux I2C device.
     */
    LinuxI2CDevice() = default;

    /**
     * @brief Destructor closes the device if still open.
     */
    ~LinuxI2CDevice() override;

    LinuxI2CDevice(const LinuxI2CDevice&) = delete;
    LinuxI2CDevice& operator=(const LinuxI2CDevice&) = delete;

    /**
     * @brief Open the Linux I2C device and select the slave address.
     *
     * @param settings Bus number and slave address.
     * @return True on success.
     */
    bool open(const I2CDeviceSettings& settings) override;

    /**
     * @brief Close the Linux I2C device if open.
     */
    void close() override;

    /**
     * @brief Check whether the Linux I2C device is open.
     *
     * @return True if open.
     */
    bool isOpen() const override;

    /**
     * @brief Write one register byte.
     *
     * @param reg Register address.
     * @param value Byte value.
     * @return True on success.
     */
    bool writeByte(std::uint8_t reg, std::uint8_t value) override;

    /**
     * @brief Read one register byte.
     *
     * @param reg Register address.
     * @param value Output byte value.
     * @return True on success.
     */
    bool readByte(std::uint8_t reg, std::uint8_t& value) override;

    /**
     * @brief Read a contiguous block of bytes starting from one register.
     *
     * @param reg   First register address.
     * @param dest  Destination buffer of at least @p count bytes. Must not be null.
     * @param count Number of bytes to read. Must be greater than zero.
     * @return True on success.
     */
    bool readBytes(std::uint8_t reg, std::uint8_t* dest, std::size_t count) override;

private:
    int fd_{-1};
    I2CDeviceSettings settings_{};
};

} // namespace solar
