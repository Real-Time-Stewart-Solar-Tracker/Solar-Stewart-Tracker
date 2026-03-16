#pragma once

#include <cstddef>
#include <cstdint>

namespace solar::hal {

/**
 * @file II2CDevice.hpp
 * @brief Platform abstraction for I2C register communication.
 *
 * This interface defines the minimal contract required for register-based
 * communication with I2C peripherals such as the PCA9685 PWM controller.
 *
 * ----------------------------------------------------------------------------
 * DESIGN RATIONALE 
 * ----------------------------------------------------------------------------
 *
 * Single Responsibility Principle (SRP):
 *   This interface has exactly one responsibility:
 *   Abstract register-level I2C communication.
 *
 * Open/Closed Principle (OCP):
 *   Concrete implementations (LinuxI2CDevice, FakeI2CDevice, future RT kernel
 *   drivers) can be added without modifying dependent classes.
 *
 * Liskov Substitution Principle (LSP):
 *   Any implementation of II2CDevice must behave as a valid I2C device.
 *   Drivers using this interface must not depend on platform-specific details.
 *
 * Interface Segregation Principle (ISP):
 *   The interface exposes only the required register-level primitives.
 *   No unnecessary high-level abstractions are included.
 *
 * Dependency Inversion Principle (DIP):
 *   High-level drivers (e.g. PCA9685) depend on this abstraction,
 *   not on Linux system calls directly.
 *
 * ----------------------------------------------------------------------------
 * REALTIME CONSIDERATIONS
 * ----------------------------------------------------------------------------
 *
 * - Functions are designed to execute quickly and deterministically.
 * - No memory allocations occur inside interface methods.
 * - No sleep-based timing is permitted.
 * - Intended to be used in event-driven actuator thread.
 *
 * ----------------------------------------------------------------------------
 * THREAD SAFETY
 * ----------------------------------------------------------------------------
 *
 * Implementations are not required to be thread-safe.
 * The actuator thread is responsible for serialized access.
 *
 * ----------------------------------------------------------------------------
 * MEMORY SAFETY
 * ----------------------------------------------------------------------------
 *
 * - No raw ownership transfer.
 * - No malloc/free.
 * - Designed for use with smart pointers.
 *
 * ----------------------------------------------------------------------------
 */
class II2CDevice {
public:
    virtual ~II2CDevice() = default;

    /**
     * @brief Write a single 8-bit value to a device register.
     *
     * @param reg Register address.
     * @param value Value to write.
     * @return true on success.
     * @return false if the write operation failed.
     *
     * @note Must not throw exceptions in production code.
     *       Error handling should be deterministic.
     */
    virtual bool write_reg_u8(uint8_t reg, uint8_t value) noexcept = 0;

    /**
     * @brief Write multiple bytes starting at a register address.
     *
     * @param reg Starting register address.
     * @param data Pointer to data buffer.
     * @param len Number of bytes to write.
     * @return true on success.
     * @return false on failure.
     *
     * @pre data must not be nullptr if len > 0.
     */
    virtual bool write_reg_bytes(uint8_t reg,
                                 const uint8_t* data,
                                 size_t len) noexcept = 0;

    /**
     * @brief Read multiple bytes from a register.
     *
     * @param reg Starting register address.
     * @param data Output buffer.
     * @param len Number of bytes to read.
     * @return true on success.
     * @return false on failure.
     *
     * @pre data must not be nullptr if len > 0.
     */
    virtual bool read_reg_bytes(uint8_t reg,
                                uint8_t* data,
                                size_t len) noexcept = 0;
};

} // namespace solar::hal