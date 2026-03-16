#include "test_common.hpp"

#include "actuators/ServoDriver.hpp"
#include "actuators/PCA9685.hpp"
#include "FakeI2CDevice.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

namespace {

// Find the most recent 4-byte LED write for a specific channel.
// PCA9685 LED0_ON_L = 0x06, and each channel block is 4 bytes.
bool saw_channel_write(const solar::test::FakeI2CDevice& dev, uint8_t channel) {
    const uint8_t base = static_cast<uint8_t>(0x06 + 4U * channel);
    for (const auto& w : dev.writes) {
        if (w.reg == base && w.data.size() == 4) return true;
    }
    return false;
}

// Decode the last written OFF count (12-bit) for a given channel.
// PCA9685 LEDn: ON_L, ON_H, OFF_L, OFF_H.
// We only care about OFF here because ServoDriver uses set_pulse_us -> sets a duty window.
uint16_t last_off_count(const solar::test::FakeI2CDevice& dev, uint8_t channel) {
    const uint8_t base = static_cast<uint8_t>(0x06 + 4U * channel);
    for (auto it = dev.writes.rbegin(); it != dev.writes.rend(); ++it) {
        if (it->reg == base && it->data.size() == 4) {
            const uint8_t off_l = it->data[2];
            const uint8_t off_h = it->data[3];
            const uint16_t off = static_cast<uint16_t>(off_l | ((off_h & 0x0F) << 8));
            return off;
        }
    }
    return 0;
}

} // namespace

TEST(ServoDriver_start_parks_to_neutral_when_enabled) {
    solar::Logger log;

    auto fakeI2C = std::make_shared<solar::test::FakeI2CDevice>();
    auto pca = std::make_unique<solar::actuators::PCA9685>(
        fakeI2C, solar::actuators::PCA9685::Config{});

    solar::ServoDriver::Config cfg{};
    cfg.park_on_start = true;
    cfg.park_on_stop  = false;

    // 3 channels: 0,1,2
    // Pulse window: 1100..1900 us
    // Angle window: 60..120 deg (safe start)
    cfg.ch[0].channel = 0;
    cfg.ch[0].min_pulse_us = 1100.f;
    cfg.ch[0].max_pulse_us = 1900.f;
    cfg.ch[0].min_deg = 60.f;
    cfg.ch[0].max_deg = 120.f;
    cfg.ch[0].neutral_deg = 90.f;
    cfg.ch[0].invert = false;

    cfg.ch[1] = cfg.ch[0];
    cfg.ch[1].channel = 1;

    cfg.ch[2] = cfg.ch[0];
    cfg.ch[2].channel = 2;
    cfg.ch[2].invert = true;

    solar::ServoDriver drv(log, cfg, std::move(pca));
    REQUIRE(drv.start());

    // Park should write all three channels
    REQUIRE(saw_channel_write(*fakeI2C, 0));
    REQUIRE(saw_channel_write(*fakeI2C, 1));
    REQUIRE(saw_channel_write(*fakeI2C, 2));
}

TEST(ServoDriver_apply_clamps_and_writes_channels) {
    solar::Logger log;

    auto fakeI2C = std::make_shared<solar::test::FakeI2CDevice>();
    auto pca = std::make_unique<solar::actuators::PCA9685>(
        fakeI2C, solar::actuators::PCA9685::Config{});

    solar::ServoDriver::Config cfg{};
    cfg.park_on_start = false;
    cfg.park_on_stop  = false;

    // Channel 0: not inverted
    cfg.ch[0].channel = 0;
    cfg.ch[0].min_pulse_us = 1100.f;
    cfg.ch[0].max_pulse_us = 1900.f;
    cfg.ch[0].min_deg = 60.f;
    cfg.ch[0].max_deg = 120.f;
    cfg.ch[0].neutral_deg = 90.f;
    cfg.ch[0].invert = false;

    // Channel 1: not inverted
    cfg.ch[1] = cfg.ch[0];
    cfg.ch[1].channel = 1;

    // Channel 2: inverted
    cfg.ch[2] = cfg.ch[0];
    cfg.ch[2].channel = 2;
    cfg.ch[2].invert = true;

    solar::ServoDriver drv(log, cfg, std::move(pca));
    REQUIRE(drv.start());

    // Out-of-range degrees -> must clamp:
    // ch0: 999 -> clamps to 120
    // ch1: -999 -> clamps to 60
    // ch2: 999 -> clamps to 120, then invert -> maps like 60 on non-inverted
    solar::ActuatorCommand cmd{};
    cmd.actuator_targets = { 999.f, -999.f, 999.f };

    drv.apply(cmd);

    REQUIRE(saw_channel_write(*fakeI2C, 0));
    REQUIRE(saw_channel_write(*fakeI2C, 1));
    REQUIRE(saw_channel_write(*fakeI2C, 2));

    // Compare relative PWM outputs:
    // For non-inverted channels:
    //  - 120 deg -> should map closer to max_pulse
    //  - 60 deg  -> should map closer to min_pulse
    //
    // For inverted channel at 120 deg:
    //  - invert flips mapping, so it should be closer to min_pulse (like 60 deg non-inverted)
    const uint16_t off0 = last_off_count(*fakeI2C, 0);
    const uint16_t off1 = last_off_count(*fakeI2C, 1);
    const uint16_t off2 = last_off_count(*fakeI2C, 2);

    // Expect ordering: off0 (120deg non-inv) > off1 (60deg non-inv)
    REQUIRE(off0 > off1);

    // Inverted at 120deg should behave like low end => off2 should be closer to off1 than off0.
    // We can't guarantee exact equality due to timer rounding, but ordering should hold.
    REQUIRE(off2 < off0);
}
