#include "test_common.hpp"

#include "actuators/PCA9685.hpp"
#include "FakeI2CDevice.hpp"

#include <memory>

using solar::actuators::PCA9685;

namespace {

// Small helper: check whether a register write happened.
bool wrote_reg(const solar::test::FakeI2CDevice& dev, uint8_t reg) {
    for (const auto& w : dev.writes) {
        if (w.reg == reg) return true;
    }
    return false;
}

// Helper: check a multi-byte write to a base register.
bool wrote_bytes_at(const solar::test::FakeI2CDevice& dev, uint8_t reg, size_t nbytes) {
    for (const auto& w : dev.writes) {
        if (w.reg == reg && w.data.size() == nbytes) return true;
    }
    return false;
}

} // namespace

TEST(PCA9685_init_writes_mode_and_prescale) {
    auto fake = std::make_shared<solar::test::FakeI2CDevice>();

    PCA9685::Config cfg;
    cfg.pwm_hz = 50.0f;

    PCA9685 pca(fake, cfg);
    REQUIRE(pca.init());

    // MODE1 and MODE2 should be touched during init
    REQUIRE(wrote_reg(*fake, 0x00)); // MODE1
    REQUIRE(wrote_reg(*fake, 0x01)); // MODE2

    // PRESCALE should be written when setting PWM frequency
    REQUIRE(wrote_reg(*fake, 0xFE)); // PRESCALE
}

TEST(PCA9685_set_pwm_writes_led_register_block) {
    auto fake = std::make_shared<solar::test::FakeI2CDevice>();
    PCA9685 pca(fake, PCA9685::Config{});
    REQUIRE(pca.init());

    // Channel 2 base register = LED0_ON_L (0x06) + 4*2 = 0x0E
    REQUIRE(pca.set_pwm(2, 0, 2048));
    REQUIRE(wrote_bytes_at(*fake, 0x0E, 4));
}

TEST(PCA9685_set_pulse_us_uses_frequency_conversion) {
    auto fake = std::make_shared<solar::test::FakeI2CDevice>();

    PCA9685::Config cfg;
    cfg.pwm_hz = 50.0f; // 20ms period

    PCA9685 pca(fake, cfg);
    REQUIRE(pca.init());

    // A typical servo neutral pulse (1500us) should map to a mid-ish count.
    REQUIRE(pca.set_pulse_us(0, 1500.0f));

    // Verify it wrote LED0 registers (channel 0 base = 0x06)
    REQUIRE(wrote_bytes_at(*fake, 0x06, 4));
}