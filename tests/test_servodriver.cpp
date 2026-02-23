#include "test_common.hpp"

#include "actuators/ServoDriver.hpp"
#include "actuators/PCA9685.hpp"
#include "FakeI2CDevice.hpp"

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

} // namespace

TEST(ServoDriver_start_parks_to_neutral_when_enabled) {
    solar::Logger log; // assuming your test_common provides a usable logger; if not, swap to your test logger

    auto fakeI2C = std::make_shared<solar::test::FakeI2CDevice>();
    auto pca = std::make_unique<solar::actuators::PCA9685>(fakeI2C, solar::actuators::PCA9685::Config{});

    solar::ServoDriver::Config cfg{};
    cfg.park_on_start = true;
    cfg.park_on_stop  = false;
    cfg.ch[0] = solar::ServoDriver::ChannelConfig{0, 1100.0f, 1900.0f, false, 1500.0f};
    cfg.ch[1] = solar::ServoDriver::ChannelConfig{1, 1100.0f, 1900.0f, false, 1500.0f};
    cfg.ch[2] = solar::ServoDriver::ChannelConfig{2, 1100.0f, 1900.0f, true,  1500.0f};

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
    auto pca = std::make_unique<solar::actuators::PCA9685>(fakeI2C, solar::actuators::PCA9685::Config{});

    solar::ServoDriver::Config cfg{};
    cfg.min_norm = -1.0f;
    cfg.max_norm =  1.0f;
    cfg.park_on_start = false;
    cfg.park_on_stop  = false;

    // Safe narrow range for first-run hardware
    cfg.ch[0] = solar::ServoDriver::ChannelConfig{0, 1100.0f, 1900.0f, false, 1500.0f};
    cfg.ch[1] = solar::ServoDriver::ChannelConfig{1, 1100.0f, 1900.0f, false, 1500.0f};
    cfg.ch[2] = solar::ServoDriver::ChannelConfig{2, 1100.0f, 1900.0f, true,  1500.0f};

    solar::ServoDriver drv(log, cfg, std::move(pca));
    REQUIRE(drv.start());

    solar::ActuatorCommand cmd{};
    cmd.actuator_targets = { 2.0f, -2.0f, 0.0f }; // out of range -> must clamp

    drv.apply(cmd);

    // Should write pulses for all channels deterministically
    REQUIRE(saw_channel_write(*fakeI2C, 0));
    REQUIRE(saw_channel_write(*fakeI2C, 1));
    REQUIRE(saw_channel_write(*fakeI2C, 2));
}