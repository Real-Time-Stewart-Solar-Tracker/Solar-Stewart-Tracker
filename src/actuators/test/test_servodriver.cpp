#include "test_common.hpp"

#include "FakeI2CDevice.hpp"
#include "actuators/PCA9685.hpp"
#include "actuators/ServoDriver.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace {

// Returns true if the fake device saw a 4-byte LED register block write
// for the given PCA9685 channel.
bool saw_channel_write(const solar::test::FakeI2CDevice& dev, std::uint8_t channel) {
    const std::uint8_t base = static_cast<std::uint8_t>(0x06 + 4U * channel);
    for (const auto& w : dev.writes) {
        if (w.reg == base && w.data.size() == 4U) {
            return true;
        }
    }
    return false;
}

// Counts 4-byte LED register block writes for a given PCA9685 channel.
std::size_t count_channel_writes(const solar::test::FakeI2CDevice& dev, std::uint8_t channel) {
    const std::uint8_t base = static_cast<std::uint8_t>(0x06 + 4U * channel);
    std::size_t count = 0;
    for (const auto& w : dev.writes) {
        if (w.reg == base && w.data.size() == 4U) {
            ++count;
        }
    }
    return count;
}

// Decodes the last OFF count written for a given channel.
// PCA9685 LEDn layout: ON_L, ON_H, OFF_L, OFF_H.
std::uint16_t last_off_count(const solar::test::FakeI2CDevice& dev, std::uint8_t channel) {
    const std::uint8_t base = static_cast<std::uint8_t>(0x06 + 4U * channel);
    for (auto it = dev.writes.rbegin(); it != dev.writes.rend(); ++it) {
        if (it->reg == base && it->data.size() == 4U) {
            const std::uint8_t off_l = it->data[2];
            const std::uint8_t off_h = it->data[3];
            return static_cast<std::uint16_t>(off_l | ((off_h & 0x0F) << 8));
        }
    }
    return 0;
}

std::shared_ptr<solar::test::FakeI2CDevice> makeFakeI2C() {
    return std::make_shared<solar::test::FakeI2CDevice>();
}

std::unique_ptr<solar::actuators::PCA9685> makePca(
    const std::shared_ptr<solar::test::FakeI2CDevice>& fakeI2C) {
    return std::make_unique<solar::actuators::PCA9685>(
        fakeI2C, solar::actuators::PCA9685::Config{});
}

solar::ServoDriver::Config makeCfg() {
    solar::ServoDriver::Config cfg{};
    cfg.park_on_start = false;
    cfg.park_on_stop  = false;
    cfg.log_every_n   = 0;
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::PreferHardware;

    cfg.ch[0].channel      = 0;
    cfg.ch[0].min_pulse_us = 1100.0f;
    cfg.ch[0].max_pulse_us = 1900.0f;
    cfg.ch[0].min_deg      = 60.0f;
    cfg.ch[0].max_deg      = 120.0f;
    cfg.ch[0].neutral_deg  = 90.0f;
    cfg.ch[0].invert       = false;

    cfg.ch[1] = cfg.ch[0];
    cfg.ch[1].channel = 1;

    cfg.ch[2] = cfg.ch[0];
    cfg.ch[2].channel = 2;
    cfg.ch[2].invert  = true;

    return cfg;
}

solar::ActuatorCommand makeCommand(float a0, float a1, float a2) {
    solar::ActuatorCommand cmd{};
    cmd.actuator_targets = {a0, a1, a2};
    return cmd;
}

} // namespace

TEST(ServoDriver_log_only_mode_starts_without_hardware) {
    solar::Logger log;
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::LogOnly;

    solar::ServoDriver drv(log, cfg);

    REQUIRE(drv.start());
    REQUIRE(drv.isRunning());
    REQUIRE(!drv.hardwareActive());
    REQUIRE(drv.runtimeMode() == solar::ServoDriver::RuntimeMode::LogOnly);
}

TEST(ServoDriver_require_hardware_fails_fast_when_unavailable) {
    solar::Logger log;
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;
    cfg.i2c_dev = "/definitely/not/a/real/i2c/device";

    solar::ServoDriver drv(log, cfg);

    REQUIRE(!drv.start());
    REQUIRE(!drv.isRunning());
    REQUIRE(!drv.hardwareActive());
    REQUIRE(drv.runtimeMode() == solar::ServoDriver::RuntimeMode::Stopped);
}

TEST(ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable) {
    solar::Logger log;
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::PreferHardware;
    cfg.i2c_dev = "/definitely/not/a/real/i2c/device";

    solar::ServoDriver drv(log, cfg);

    REQUIRE(drv.start());
    REQUIRE(drv.isRunning());
    REQUIRE(!drv.hardwareActive());
    REQUIRE(drv.runtimeMode() == solar::ServoDriver::RuntimeMode::LogOnly);
}

TEST(ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode) {
    solar::Logger log;

    auto fakeI2C = makeFakeI2C();
    auto pca = makePca(fakeI2C);
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;

    solar::ServoDriver drv(log, cfg, std::move(pca));

    REQUIRE(drv.start());
    REQUIRE(drv.isRunning());
    REQUIRE(drv.hardwareActive());
    REQUIRE(drv.runtimeMode() == solar::ServoDriver::RuntimeMode::Hardware);
}

TEST(ServoDriver_apply_while_stopped_writes_nothing) {
    solar::Logger log;

    auto fakeI2C = makeFakeI2C();
    auto pca = makePca(fakeI2C);
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;

    solar::ServoDriver drv(log, cfg, std::move(pca));

    const std::size_t writes_before = fakeI2C->writes.size();

    drv.apply(makeCommand(90.0f, 90.0f, 90.0f));

    REQUIRE(fakeI2C->writes.size() == writes_before);
    REQUIRE(!saw_channel_write(*fakeI2C, 0));
    REQUIRE(!saw_channel_write(*fakeI2C, 1));
    REQUIRE(!saw_channel_write(*fakeI2C, 2));
}

TEST(ServoDriver_start_without_parking_does_not_write_servo_channels) {
    solar::Logger log;

    auto fakeI2C = makeFakeI2C();
    auto pca = makePca(fakeI2C);
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;
    cfg.park_on_start = false;

    solar::ServoDriver drv(log, cfg, std::move(pca));
    REQUIRE(drv.start());

    REQUIRE(drv.hardwareActive());
    REQUIRE(!saw_channel_write(*fakeI2C, 0));
    REQUIRE(!saw_channel_write(*fakeI2C, 1));
    REQUIRE(!saw_channel_write(*fakeI2C, 2));
}

TEST(ServoDriver_start_parks_to_neutral_when_enabled) {
    solar::Logger log;

    auto fakeI2C = makeFakeI2C();
    auto pca = makePca(fakeI2C);
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;
    cfg.park_on_start = true;

    solar::ServoDriver drv(log, cfg, std::move(pca));
    REQUIRE(drv.start());

    REQUIRE(drv.hardwareActive());
    REQUIRE(saw_channel_write(*fakeI2C, 0));
    REQUIRE(saw_channel_write(*fakeI2C, 1));
    REQUIRE(saw_channel_write(*fakeI2C, 2));
}

TEST(ServoDriver_stop_parks_to_neutral_when_enabled) {
    solar::Logger log;

    auto fakeI2C = makeFakeI2C();
    auto pca = makePca(fakeI2C);
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;
    cfg.park_on_start = false;
    cfg.park_on_stop  = true;

    solar::ServoDriver drv(log, cfg, std::move(pca));
    REQUIRE(drv.start());

    const std::size_t ch0_before = count_channel_writes(*fakeI2C, 0);
    const std::size_t ch1_before = count_channel_writes(*fakeI2C, 1);
    const std::size_t ch2_before = count_channel_writes(*fakeI2C, 2);

    drv.stop();

    REQUIRE(count_channel_writes(*fakeI2C, 0) == ch0_before + 1U);
    REQUIRE(count_channel_writes(*fakeI2C, 1) == ch1_before + 1U);
    REQUIRE(count_channel_writes(*fakeI2C, 2) == ch2_before + 1U);
    REQUIRE(drv.runtimeMode() == solar::ServoDriver::RuntimeMode::Stopped);
}

TEST(ServoDriver_apply_clamps_and_writes_channels) {
    solar::Logger log;

    auto fakeI2C = makeFakeI2C();
    auto pca = makePca(fakeI2C);
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;

    solar::ServoDriver drv(log, cfg, std::move(pca));
    REQUIRE(drv.start());

    // Out-of-range degrees must clamp:
    // ch0: 999  -> clamps to 120 (high end, non-inverted)
    // ch1: -999 -> clamps to 60  (low end, non-inverted)
    // ch2: 999  -> clamps to 120, then invert -> behaves like low end
    drv.apply(makeCommand(999.0f, -999.0f, 999.0f));

    REQUIRE(saw_channel_write(*fakeI2C, 0));
    REQUIRE(saw_channel_write(*fakeI2C, 1));
    REQUIRE(saw_channel_write(*fakeI2C, 2));

    const std::uint16_t off0 = last_off_count(*fakeI2C, 0);
    const std::uint16_t off1 = last_off_count(*fakeI2C, 1);
    const std::uint16_t off2 = last_off_count(*fakeI2C, 2);

    REQUIRE(off0 > off1);
    REQUIRE(off2 < off0);
}

TEST(ServoDriver_neutral_degree_maps_midway_between_low_and_high) {
    solar::Logger log;

    auto fakeI2C = makeFakeI2C();
    auto pca = makePca(fakeI2C);
    auto cfg = makeCfg();
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;

    solar::ServoDriver drv(log, cfg, std::move(pca));
    REQUIRE(drv.start());

    drv.apply(makeCommand(60.0f, 90.0f, 120.0f));

    const std::uint16_t low  = last_off_count(*fakeI2C, 0);
    const std::uint16_t mid  = last_off_count(*fakeI2C, 1);
    

    // Channel 2 is inverted, so use a fresh non-inverted midpoint check on channel 0.
    drv.apply(makeCommand(120.0f, 90.0f, 60.0f));
    const std::uint16_t high_noninv = last_off_count(*fakeI2C, 0);
    const std::uint16_t mid_noninv  = last_off_count(*fakeI2C, 1);

    REQUIRE(low < mid);
    REQUIRE(mid_noninv < high_noninv);

    const int midpoint_expected =
        (static_cast<int>(low) + static_cast<int>(high_noninv)) / 2;
    const int midpoint_actual = static_cast<int>(mid_noninv);

    REQUIRE(std::abs(midpoint_actual - midpoint_expected) <= 1);
}