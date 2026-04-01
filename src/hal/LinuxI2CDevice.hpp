#pragma once

#include "hal/II2CDevice.hpp"

#include <cstdint>
#include <string>

namespace solar::hal {

/**
 * @file LinuxI2CDevice.hpp
 * @brief Linux (/dev/i2c-*) implementation of II2CDevice.
 *
 * Opens an I2C bus device node and targets a 7-bit slave address.
 * Intended for fast, deterministic register access in the actuator thread.
 *
 * @note Platform-specific. On non-Linux platforms this class still compiles but
 *       will report failure at runtime (see .cpp).
 */
class LinuxI2CDevice final : public II2CDevice {
public:
    /**
     * @brief Construct and attempt to open an I2C device.
     *
     * @param dev_path Path like "/dev/i2c-1".
     * @param address 7-bit I2C address (e.g. 0x40 for PCA9685).
     */
    LinuxI2CDevice(std::string dev_path, uint8_t address) noexcept;

    /** @brief Close file descriptor if open. */
    ~LinuxI2CDevice() override;

    LinuxI2CDevice(const LinuxI2CDevice&)            = delete;
    LinuxI2CDevice& operator=(const LinuxI2CDevice&) = delete;

    LinuxI2CDevice(LinuxI2CDevice&&)            = delete;
    LinuxI2CDevice& operator=(LinuxI2CDevice&&) = delete;

    /** @return true if the device node was opened and the slave address set. */
    bool ok() const noexcept;

    bool write_reg_u8(uint8_t reg, uint8_t value) noexcept override;
    bool write_reg_bytes(uint8_t reg, const uint8_t* data, size_t len) noexcept override;
    bool read_reg_bytes(uint8_t reg, uint8_t* data, size_t len) noexcept override;

private:
    int fd_{-1};
};

} // namespace solar::hal