#pragma once

/**
 * @file II2CDevice.hpp
 * @brief Abstract low-level I2C device interface.
 *
 * This interface exists to:
 * - hide Linux /dev/i2c access details from sensor publishers
 * - support proper unit testing with fake devices
 * - keep sensor logic separate from raw bus mechanics
 */

#include <cstddef>
#include <cstdint>

namespace solar {

/**
 * @brief Linux I2C device selection settings.
 */
struct I2CDeviceSettings {
    int bus{1};                    ///< Linux I2C bus number, e.g. 1 -> /dev/i2c-1.
    std::uint8_t address{0x00U};   ///< 7-bit I2C device address.
};

/**
 * @brief Abstract interface for raw I2C device access.
 */
class II2CDevice {
public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~II2CDevice() = default;

    /**
     * @brief Open the low-level I2C device.
     *
     * @param settings Bus number and device address.
     * @return True on success.
     */
    virtual bool open(const I2CDeviceSettings& settings) = 0;

    /**
     * @brief Close the low-level I2C device.
     */
    virtual void close() = 0;

    /**
     * @brief Check whether the device is currently open.
     *
     * @return True if open.
     */
    virtual bool isOpen() const = 0;

    /**
     * @brief Write one register byte.
     *
     * @param reg Register address.
     * @param value Byte value.
     * @return True on success.
     */
    virtual bool writeByte(std::uint8_t reg, std::uint8_t value) = 0;

    /**
     * @brief Read one register byte.
     *
     * @param reg Register address.
     * @param value Output byte value.
     * @return True on success.
     */
    virtual bool readByte(std::uint8_t reg, std::uint8_t& value) = 0;

    /**
     * @brief Read a contiguous block of bytes starting from one register.
     *
     * The caller is responsible for ensuring that @p dest points to a buffer
     * of at least @p count bytes. Implementations must return false immediately
     * if @p dest is null or @p count is zero.
     *
     * A raw pointer is used here because the interface must remain compatible
     * with the Linux kernel I2C read() system call, which requires a plain
     * memory buffer. Callers should always allocate the destination as a
     * fixed-size stack array or @c std::array before calling this method.
     *
     * @param reg   First register address.
     * @param dest  Destination buffer of at least @p count bytes.
     * @param count Number of bytes to read. Must be greater than zero.
     * @return True on success.
     */
    virtual bool readBytes(std::uint8_t reg, std::uint8_t* dest, std::size_t count) = 0;
};

} // namespace solar
