#pragma once

/**
 * @file Mpu6050Publisher.hpp
 * @brief MPU-6050/ICM-20600 publisher using GPIO data-ready events and Linux I2C.
 *
 * This class is hardware-facing only.
 *
 * Responsibilities:
 * - initialise MPU-6050/ICM-20600 registers
 * - react to GPIO data-ready events
 * - read one IMU sample from the device
 * - publish that sample via callback
 *
 * Non-responsibilities:
 * - estimation
 * - control
 * - state-machine policy
 *
 * Register addresses and initialisation values are internal constants within
 * this file. They are not part of the application configuration surface because
 * no system integrator should need to change register addresses at runtime; those
 * are fixed by the MPU-6050/ICM-20600 hardware specification.
 */

#include "external/libgpiod_event_demo/gpioevent.h"
#include "hal/II2CDevice.hpp"
#include "sensors/imu/IIMU.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

class GPIOPin;

namespace solar {

/**
 * @brief System-integration configuration for the MPU-6050/ICM-20600 publisher.
 *
 * Only settings that a system integrator may need to change are exposed here.
 * Hardware register addresses and initialisation constants are internal to the
 * @c Mpu6050Publisher implementation.
 */
struct Mpu6050Config {
    I2CDeviceSettings i2c{};                    ///< Linux I2C bus/address.
    int gpio_pin_no{0};                         ///< Data-ready GPIO line number.
    int gpio_chip_no{0};                        ///< GPIO chip index.

    /**
     * @brief Expected WHO_AM_I register value.
     *
     * The ICM-20600 (pin-compatible MPU-6050 successor) returns 0x70,
     * whereas the original MPU-6050 returns 0x68. This must match the
     * physical device fitted to the rig.
     */
    std::uint8_t who_am_i_expected{0x70U};

    float accel_lsb_per_g{16384.0F};   ///< Accel sensitivity (LSB/g) at ±2 g range.
    float gyro_lsb_per_dps{131.0F};    ///< Gyro sensitivity (LSB/°s) at ±250 °/s range.

    std::size_t startup_discard_samples{8U}; ///< Samples discarded after startup to allow settling.
};

/**
 * @brief Event-driven MPU-6050/ICM-20600 publisher.
 *
 * Samples are produced only when the GPIO data-ready edge fires. The I2C
 * burst-read is performed entirely within the GPIO callback context. No
 * polling or sleep-based timing is used.
 */
class Mpu6050Publisher final : public IIMU {
public:
    /**
     * @brief Construct the publisher.
     *
     * @param device Low-level I2C device.
     * @param config System-integration configuration.
     */
    Mpu6050Publisher(II2CDevice& device, Mpu6050Config config = {});

    /**
     * @brief Destructor stops acquisition if still running.
     */
    ~Mpu6050Publisher() override;

    Mpu6050Publisher(const Mpu6050Publisher&) = delete;
    Mpu6050Publisher& operator=(const Mpu6050Publisher&) = delete;

    /**
     * @brief Register the callback sink receiving IMU samples.
     *
     * @param cb Callback sink.
     */
    void registerEventCallback(CallbackInterface* cb) override;

    /**
     * @brief Start acquisition.
     *
     * @return True on success.
     */
    bool start() override;

    /**
     * @brief Stop acquisition.
     */
    void stop() override;

private:
    // -----------------------------------------------------------------------
    // MPU-6050/ICM-20600 register map (hardware constants, not configurable)
    // -----------------------------------------------------------------------
    static constexpr std::uint8_t kRegWhoAmI       = 0x75U;
    static constexpr std::uint8_t kRegPowerMgmt     = 0x6BU;
    static constexpr std::uint8_t kRegSampleRateDiv = 0x19U;
    static constexpr std::uint8_t kRegConfig        = 0x1AU;
    static constexpr std::uint8_t kRegGyroConfig    = 0x1BU;
    static constexpr std::uint8_t kRegAccelConfig   = 0x1CU;
    static constexpr std::uint8_t kRegIntEnable     = 0x38U;
    static constexpr std::uint8_t kRegIntStatus     = 0x3AU;
    static constexpr std::uint8_t kRegSampleStart   = 0x3BU;

    // Initialisation values
    static constexpr std::uint8_t kWakeValue           = 0x00U; ///< Exit sleep mode.
    static constexpr std::uint8_t kSampleRateDivValue  = 0x07U; ///< Divider for ~100 Hz.
    static constexpr std::uint8_t kConfigValue         = 0x03U; ///< DLPF 44 Hz bandwidth.
    static constexpr std::uint8_t kGyroConfigValue     = 0x00U; ///< ±250 °/s full scale.
    static constexpr std::uint8_t kAccelConfigValue    = 0x00U; ///< ±2 g full scale.
    static constexpr std::uint8_t kIntEnableValue      = 0x01U; ///< Data-ready interrupt.
    static constexpr std::uint8_t kDataReadyMask       = 0x01U; ///< Data-ready status bit.

    bool initialise_();
    bool readSample_(IImuSample& sample);
    void onGpioEvent_(const gpiod::edge_event& event);

    static std::int16_t toInt16_(std::uint8_t msb, std::uint8_t lsb);
    static std::uint64_t monotonicNowUs_();

private:
    II2CDevice& device_;
    Mpu6050Config config_{};
    GPIOPin gpio_pin_{};
    CallbackInterface* callback_{nullptr};

    /**
     * Atomic flag required because @c onGpioEvent_() executes in the GPIOPin
     * worker thread while @c stop() is called from the application thread.
     */
    std::atomic<bool> running_{false};

    std::size_t discard_remaining_{0U};
};

} // namespace solar
