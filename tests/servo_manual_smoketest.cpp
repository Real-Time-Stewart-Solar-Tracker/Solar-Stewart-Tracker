#include "actuators/ServoDriver.hpp"
#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

namespace {

using namespace std::chrono_literals;

solar::ServoDriver::ChannelConfig makeChannel(std::uint8_t channel, bool invert = false) {
    solar::ServoDriver::ChannelConfig cfg{};
    cfg.channel      = channel;
    cfg.min_pulse_us = 1100.0f;
    cfg.max_pulse_us = 1900.0f;
    cfg.min_deg      = 60.0f;
    cfg.max_deg      = 120.0f;
    cfg.neutral_deg  = 90.0f;
    cfg.invert       = invert;
    return cfg;
}

void applyAndHold(solar::ServoDriver& drv,
                  const std::array<float, 3>& servo_deg,
                  std::chrono::milliseconds hold) {
    solar::ActuatorCommand cmd{};
    cmd.actuator_targets = servo_deg;
    drv.apply(cmd);
    std::this_thread::sleep_for(hold);
}

} // namespace

int main() {
    solar::Logger log;

    solar::ServoDriver::Config cfg{};
    cfg.pwm_hz         = 50.0f;
    cfg.i2c_dev        = "/dev/i2c-1";
    cfg.pca9685_addr   = 0x40;
    cfg.startup_policy = solar::ServoDriver::StartupPolicy::RequireHardware;

    // Explicit per-channel configuration.
    // Adjust channel numbers / inversion / pulse limits to match real hardware.
    cfg.ch[0] = makeChannel(0, false);
    cfg.ch[1] = makeChannel(1, false);
    cfg.ch[2] = makeChannel(2, false);

    cfg.park_on_start = false;
    cfg.park_on_stop  = true;
    cfg.log_every_n   = 0; // manual smoke test: avoid unnecessary log spam

    solar::ServoDriver drv(log, cfg);
    if (!drv.start()) {
        std::cerr << "Failed to start ServoDriver in required hardware mode.\n";
        return 1;
    }

    constexpr float neutral_deg = 90.0f;
    constexpr float delta_deg   = 15.0f;   // conservative manual movement
    const auto hold             = std::chrono::milliseconds(800);

    std::cout
        << "Manual ServoDriver smoke test\n"
        << "----------------------------------------\n"
        << "This is NOT an automated unit test.\n"
        << "It requires real PCA9685/I2C hardware.\n"
        << "Startup policy: RequireHardware\n"
        << "Sequence per servo: neutral -> neutral-delta -> neutral+delta -> neutral\n"
        << "Neutral: " << neutral_deg << " deg\n"
        << "Delta:   " << delta_deg << " deg\n"
        << "Hold:    " << hold.count() << " ms\n\n"
        << "Verify channel mapping and safe mechanical range before running on hardware.\n";

    // Start from neutral on all channels.
    applyAndHold(drv, {neutral_deg, neutral_deg, neutral_deg}, hold);

    // Servo 0 only
    applyAndHold(drv, {neutral_deg - delta_deg, neutral_deg, neutral_deg}, hold);
    applyAndHold(drv, {neutral_deg + delta_deg, neutral_deg, neutral_deg}, hold);
    applyAndHold(drv, {neutral_deg, neutral_deg, neutral_deg}, hold);

    // Servo 1 only
    applyAndHold(drv, {neutral_deg, neutral_deg - delta_deg, neutral_deg}, hold);
    applyAndHold(drv, {neutral_deg, neutral_deg + delta_deg, neutral_deg}, hold);
    applyAndHold(drv, {neutral_deg, neutral_deg, neutral_deg}, hold);

    // Servo 2 only
    applyAndHold(drv, {neutral_deg, neutral_deg, neutral_deg - delta_deg}, hold);
    applyAndHold(drv, {neutral_deg, neutral_deg, neutral_deg + delta_deg}, hold);
    applyAndHold(drv, {neutral_deg, neutral_deg, neutral_deg}, hold);

    drv.stop();
    return 0;
}