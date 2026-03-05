#pragma once

#include "hal/II2CDevice.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace solar::test {

/**
 * @file FakeI2CDevice.hpp
 * @brief In-memory fake for hal::II2CDevice used by unit tests.
 *
 * Records register writes and serves register reads from an internal map.
 * Allows testing hardware drivers (e.g. PCA9685) without Raspberry Pi hardware.
 */
class FakeI2CDevice final : public solar::hal::II2CDevice {
public:
    struct Write {
        uint8_t reg;
        std::vector<uint8_t> data;
    };

    bool write_reg_u8(uint8_t reg, uint8_t value) noexcept override {
        writes.push_back({reg, {value}});
        regs[reg] = value;
        return true;
    }

    bool write_reg_bytes(uint8_t reg, const uint8_t* data, size_t len) noexcept override {
        if (len > 0 && data == nullptr) return false;
        writes.push_back({reg, std::vector<uint8_t>(data, data + len)});
        for (size_t i = 0; i < len; ++i) {
            regs[static_cast<uint8_t>(reg + i)] = data[i];
        }
        return true;
    }

    bool read_reg_bytes(uint8_t reg, uint8_t* data, size_t len) noexcept override {
        if (len > 0 && data == nullptr) return false;
        for (size_t i = 0; i < len; ++i) {
            const uint8_t r = static_cast<uint8_t>(reg + i);
            auto it = regs.find(r);
            data[i] = (it == regs.end()) ? 0 : it->second;
        }
        return true;
    }

    /// All register writes performed by the code under test, in order.
    std::vector<Write> writes;

    /// Register store used to emulate device state.
    std::map<uint8_t, uint8_t> regs;
};

} // namespace solar::test