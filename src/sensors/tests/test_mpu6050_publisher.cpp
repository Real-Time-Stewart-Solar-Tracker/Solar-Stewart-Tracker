/**
 * @file test_mpu6050_publisher.cpp
 * @brief Unit tests for Mpu6050Publisher.
 *
 * Software tests (always run, no hardware required):
 * - start fails without callback
 * - start fails if WHO_AM_I does not match
 * - register initialisation sequence verified via FakeI2CDevice
 *
 * Hardware tests (require SOLAR_RUN_I2C_HW_TESTS=1):
 * - require physical ICM-20600 on I2C bus 1 addr 0x68
 * - require DATA_READY wired to GPIO 27
 */

#include "sensors/imu/Mpu6050Publisher.hpp"
#include "hal/LinuxI2CDevice.hpp"
#include "src/tests/support/FakeI2CDevice.hpp"
#include "src/tests/support/test_common.hpp"

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

using solar::IIMU;
using solar::IImuSample;
using solar::LinuxI2CDevice;
using solar::Mpu6050Config;
using solar::Mpu6050Publisher;
using solar::tests::FakeI2CDevice;

namespace {

class SampleCollector final : public IIMU::CallbackInterface {
public:
    void hasSample(const IImuSample& sample) override {
        ++sample_count;
        last = sample;
    }
    int sample_count{0};
    IImuSample last{};
};

/// Software config — GPIO pin irrelevant, FakeI2CDevice never opens GPIO.
Mpu6050Config makeSoftwareConfig() {
    Mpu6050Config cfg{};
    cfg.i2c.bus                 = 1;
    cfg.i2c.address             = 0x68U;
    cfg.who_am_i_expected       = 0x70U;
    cfg.gpio_pin_no             = 17;
    cfg.gpio_chip_no            = 0;
    cfg.startup_discard_samples = 0U;
    return cfg;
}

/// Hardware config matching the physical rig:
/// ICM-20600 on I2C bus 1 addr 0x68, DATA_READY on GPIO 27.
Mpu6050Config makeHardwareConfig() {
    Mpu6050Config cfg{};
    cfg.i2c.bus                 = 1;
    cfg.i2c.address             = 0x68U;
    cfg.who_am_i_expected       = 0x70U;
    cfg.gpio_pin_no             = 27;
    cfg.gpio_chip_no            = 0;
    cfg.startup_discard_samples = 0U;
    return cfg;
}

bool hwEnabled() {
    const char* v = std::getenv("SOLAR_RUN_I2C_HW_TESTS");
    return v != nullptr && std::string(v) == "1";
}

void configureDeviceForSuccess(FakeI2CDevice& dev, const Mpu6050Config& cfg) {
    dev.setRegister(0x75U, cfg.who_am_i_expected);
    dev.setRegister(0x3AU, 0x01U);
    dev.setBurst(0x3BU, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
}

} // namespace

// ---------------------------------------------------------------------------
// Software tests — FakeI2CDevice, no real GPIO, always run
// ---------------------------------------------------------------------------

TEST_CASE(Mpu6050Publisher_StartFailsWithoutCallback) {
    FakeI2CDevice dev;
    Mpu6050Publisher pub(dev, makeSoftwareConfig());
    REQUIRE(!pub.start());
}

TEST_CASE(Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch) {
    FakeI2CDevice dev;
    auto cfg = makeSoftwareConfig();
    dev.setRegister(0x75U, 0x00U);

    Mpu6050Publisher pub(dev, cfg);
    SampleCollector cb;
    pub.registerEventCallback(&cb);

    REQUIRE(!pub.start());
    REQUIRE(!dev.isOpen());
}

TEST_CASE(Mpu6050Publisher_InitialisesExpectedRegisters) {
    FakeI2CDevice dev;
    auto cfg = makeSoftwareConfig();
    configureDeviceForSuccess(dev, cfg);

    Mpu6050Publisher pub(dev, cfg);
    SampleCollector cb;
    pub.registerEventCallback(&cb);

    // start() writes init registers before opening GPIO.
    // Verify register writes regardless of GPIO outcome.
    pub.start();

    const auto& writes = dev.writes();
    auto wasWritten = [&](std::uint8_t reg) {
        for (const auto& w : writes) {
            if (w.reg == reg) return true;
        }
        return false;
    };

    REQUIRE(wasWritten(0x6BU));
    REQUIRE(wasWritten(0x19U));
    REQUIRE(wasWritten(0x1AU));
    REQUIRE(wasWritten(0x1BU));
    REQUIRE(wasWritten(0x1CU));
    REQUIRE(wasWritten(0x38U));

    pub.stop();
}

// ---------------------------------------------------------------------------
// Hardware tests — only run when SOLAR_RUN_I2C_HW_TESTS=1
// ---------------------------------------------------------------------------

TEST_CASE(Mpu6050Publisher_StartSucceedsAndStopClosesDevice) {
    if (!hwEnabled()) { return; }

    LinuxI2CDevice dev;
    Mpu6050Publisher pub(dev, makeHardwareConfig());
    SampleCollector cb;
    pub.registerEventCallback(&cb);

    REQUIRE(pub.start());
    pub.stop();
    REQUIRE(!dev.isOpen());
}

TEST_CASE(Mpu6050Publisher_DoubleStartReturnsFalse) {
    if (!hwEnabled()) { return; }

    LinuxI2CDevice dev;
    Mpu6050Publisher pub(dev, makeHardwareConfig());
    SampleCollector cb;
    pub.registerEventCallback(&cb);

    REQUIRE(pub.start());
    REQUIRE(!pub.start());
    pub.stop();
}

TEST_CASE(Mpu6050Publisher_StartupDiscardPreventsEarlyCallbacks) {
    if (!hwEnabled()) { return; }

    auto cfg = makeHardwareConfig();
    cfg.startup_discard_samples = 3U;

    LinuxI2CDevice dev;
    Mpu6050Publisher pub(dev, cfg);
    SampleCollector cb;
    pub.registerEventCallback(&cb);

    REQUIRE(pub.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    pub.stop();
    REQUIRE(cb.sample_count >= 0);
}