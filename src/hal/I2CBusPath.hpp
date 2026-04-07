#pragma once

/**
 * @file I2CBusPath.hpp
 * @brief Utility for constructing the Linux /dev/i2c-N device path string.
 *
 * Centralising this avoids duplicating the same snprintf pattern across every
 * class that needs to open a Linux I2C bus descriptor.
 */

#include <string>

namespace solar {

/**
 * @brief Return the Linux device path for a given I2C bus number.
 *
 * @param bus Linux I2C bus index (e.g. 1 → @c /dev/i2c-1).
 * @return Device path string.
 */
inline std::string i2cBusPath(int bus) {
    return "/dev/i2c-" + std::to_string(bus);
}

} // namespace solar
