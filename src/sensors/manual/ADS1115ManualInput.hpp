#pragma once

/**
 * @file ADS1115ManualInput.hpp
 * @brief Event-driven ADS1115 publisher for manual potentiometer input.
 *
 * This class is hardware-facing only.
 *
 * Responsibilities:
 * - configure ADS1115
 * - wait for ALERT/RDY GPIO events
 * - read completed ADC conversions
 * - assemble one ManualPotSample
 * - emit the sample via callback
 *
 * Non-responsibilities:
 * - mapping voltages to control angles
 * - manual/auto state policy
 * - queuing or thread orchestration outside the GPIO callback path
 */

#include "common/ManualInputTypes.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

class GPIOPin;

namespace solar {

/**
 * @brief Runtime settings for ADS1115 manual input acquisition.
 */
struct ADS1115ManualInputSettings {
    bool enabled{false};               ///< Enable manual ADS1115 path.
    int i2c_bus{1};                    ///< Linux I2C bus number.
    std::uint8_t i2c_address{0x48U};   ///< ADS1115 I2C address.

    int gpio_chip_index{0};            ///< GPIO chip index for ALERT/RDY.
    unsigned int alert_rdy_gpio{17U};  ///< ALERT/RDY GPIO line number.

    std::uint8_t tilt_channel{0U};     ///< Tilt potentiometer channel.
    std::uint8_t pan_channel{1U};      ///< Pan potentiometer channel.
    std::uint8_t spare_channel{0xFFU}; ///< Optional spare channel. 0xFF disables it.

    float full_scale_voltage{4.096F};  ///< Full-scale input voltage (V).
    unsigned int sample_rate_hz{128U}; ///< ADS1115 conversion rate (Hz).
};

/**
 * @brief Event-driven ADS1115 publisher.
 */
class ADS1115ManualInput {
public:
    /**
     * @brief Callback receiving one completed potentiometer sample.
     *
     * @param sample Completed manual potentiometer sample with voltages in volts.
     */
    using SampleCallback = std::function<void(const ManualPotSample& sample)>;

    /**
     * @brief Construct the ADS1115 publisher.
     *
     * @param settings Runtime settings.
     */
    explicit ADS1115ManualInput(const ADS1115ManualInputSettings& settings);

    /**
     * @brief Destructor stops the backend if still running.
     */
    ~ADS1115ManualInput();

    ADS1115ManualInput(const ADS1115ManualInput&) = delete;
    ADS1115ManualInput& operator=(const ADS1115ManualInput&) = delete;

    /**
     * @brief Register the output sample callback.
     *
     * @param callback Sample callback.
     */
    void registerCallback(SampleCallback callback);

    /**
     * @brief Start the backend.
     *
     * @return True on success.
     */
    bool start();

    /**
     * @brief Stop the backend.
     */
    void stop();

private:
    /**
     * @brief Internal channel acquisition phase.
     */
    enum class Phase : std::uint8_t {
        Tilt,  ///< Currently acquiring the tilt channel.
        Pan,   ///< Currently acquiring the pan channel.
        Spare  ///< Currently acquiring the spare channel.
    };

    /**
     * @brief RAII owner for the Linux I2C file descriptor.
     *
     * Ensures the descriptor is closed on any exit path from openI2C_() or
     * start(), including early returns due to configuration failures.
     */
    class I2CDescriptor {
    public:
        I2CDescriptor() = default;

        ~I2CDescriptor() {
            close();
        }

        I2CDescriptor(const I2CDescriptor&) = delete;
        I2CDescriptor& operator=(const I2CDescriptor&) = delete;

        /**
         * @brief Open the I2C device and select the slave address.
         *
         * @param bus Linux I2C bus number.
         * @param address 7-bit I2C device address.
         * @return True on success.
         */
        bool open(int bus, std::uint8_t address);

        /**
         * @brief Close the descriptor if open.
         */
        void close();

        /**
         * @brief Return the raw file descriptor, or -1 if not open.
         *
         * @return Raw file descriptor.
         */
        int get() const noexcept { return fd_; }

        /**
         * @brief Return whether the descriptor is currently open.
         *
         * @return True if open.
         */
        bool isOpen() const noexcept { return fd_ >= 0; }

    private:
        int fd_{-1};
    };

    bool writeRegister16_(std::uint8_t reg, std::uint16_t value);
    bool readRegister16_(std::uint8_t reg, std::uint16_t& value);

    bool configureReadyMode_();
    bool startNextConversion_(std::uint8_t channel);
    bool readLatestConversion_(float& voltage_v);

    std::uint16_t buildConfig_(std::uint8_t channel) const;
    std::uint16_t muxBits_(std::uint8_t channel) const;
    std::uint16_t pgaBits_() const;
    std::uint16_t drBits_() const;
    double fullScaleVoltage_() const;

    void onDataReady_();

private:
    ADS1115ManualInputSettings settings_{};
    SampleCallback callback_{};

    std::atomic<bool> running_{false};
    std::unique_ptr<GPIOPin> drdy_pin_{};

    I2CDescriptor i2c_{};
    mutable std::mutex mutex_{};

    Phase phase_{Phase::Tilt};
    bool awaiting_conversion_{false};
    ManualPotSample current_sample_{};
};

} // namespace solar
