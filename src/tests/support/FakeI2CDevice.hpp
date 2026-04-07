#pragma once

/**
 * @file FakeI2CDevice.hpp
 * @brief Reusable fake II2CDevice for unit tests.
 */

#include "hal/II2CDevice.hpp"

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <unordered_map>
#include <utility>
#include <vector>

namespace solar::tests {

/**
 * @brief Reusable fake I2C device for unit tests.
 *
 * This fake supports:
 * - open/close state
 * - single-byte register map
 * - burst reads from a configured start register
 * - recording write operations
 */
class FakeI2CDevice final : public solar::II2CDevice {
public:
    /**
     * @brief One recorded register write.
     */
    struct WriteOp {
        std::uint8_t reg{0U};    ///< Register address.
        std::uint8_t value{0U};  ///< Written value.
    };

    bool open(const solar::I2CDeviceSettings& settings) override {
        opened_ = true;
        settings_ = settings;
        return true;
    }

    void close() override {
        opened_ = false;
    }

    bool isOpen() const override {
        return opened_;
    }

    bool writeByte(const std::uint8_t reg, const std::uint8_t value) override {
        writes_.push_back({reg, value});
        regs_[reg] = value;
        return true;
    }

    bool readByte(const std::uint8_t reg, std::uint8_t& value) override {
        const auto it = regs_.find(reg);
        if (it == regs_.end()) {
            value = 0U;
        } else {
            value = it->second;
        }
        return true;
    }

    bool readBytes(const std::uint8_t reg, std::uint8_t* dest, const std::size_t count) override {
        if (dest == nullptr || count == 0U) {
            return false;
        }
        if (reg != burst_reg_ || burst_data_.size() < count) {
            return false;
        }

        for (std::size_t i = 0; i < count; ++i) {
            dest[i] = burst_data_[i];
        }
        return true;
    }

    /**
     * @brief Set one register value in the fake register map.
     *
     * @param reg Register address.
     * @param value Register value.
     */
    void setRegister(const std::uint8_t reg, const std::uint8_t value) {
        regs_[reg] = value;
    }

    /**
     * @brief Configure one burst-read response.
     *
     * @param reg Start register expected by readBytes().
     * @param bytes Burst data returned by readBytes().
     */
    void setBurst(const std::uint8_t reg, std::initializer_list<std::uint8_t> bytes) {
        burst_reg_ = reg;
        burst_data_.assign(bytes.begin(), bytes.end());
    }

    /**
     * @brief Access recorded writes.
     *
     * @return Recorded write list.
     */
    const std::vector<WriteOp>& writes() const {
        return writes_;
    }

    /**
     * @brief Access last open settings.
     *
     * @return Settings used on open().
     */
    const solar::I2CDeviceSettings& settings() const {
        return settings_;
    }

    /**
     * @brief Clear recorded write history.
     */
    void clearWrites() {
        writes_.clear();
    }

private:
    bool opened_{false};
    solar::I2CDeviceSettings settings_{};
    std::unordered_map<std::uint8_t, std::uint8_t> regs_{};
    std::uint8_t burst_reg_{0U};
    std::vector<std::uint8_t> burst_data_{};
    std::vector<WriteOp> writes_{};
};

} // namespace solar::tests