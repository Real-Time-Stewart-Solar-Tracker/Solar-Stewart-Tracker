#include "actuators/ServoDriver.hpp"
#include "common/Logger.hpp"
#include "common/Types.hpp"

#include <chrono>
#include <thread>

int main() {
    solar::Logger log;

    solar::ServoDriver::Config cfg{};
    cfg.pwm_hz = 50.0f;
    cfg.i2c_dev = "/dev/i2c-1";
    cfg.pca9685_addr = 0x40;

    // IMPORTANT: map your three servos to three PCA channels
    cfg.ch[0] = solar::ServoDriver::ChannelConfig{0, 1100, 1900, false, 1500};
    cfg.ch[1] = solar::ServoDriver::ChannelConfig{1, 1100, 1900, false, 1500};
    cfg.ch[2] = solar::ServoDriver::ChannelConfig{2, 1100, 1900, false, 1500};

    cfg.park_on_start = true;
    cfg.park_on_stop  = true;
    cfg.log_every_n   = 1;

    solar::ServoDriver drv(log, cfg);
    if (!drv.start()) return 1;

    solar::ActuatorCommand cmd{};
    cmd.actuator_targets = {0.0f, 0.0f, 0.0f};

    // Move only servo 0 while holding others neutral
    cmd.actuator_targets[0] = -1.0f; drv.apply(cmd); std::this_thread::sleep_for(std::chrono::seconds(1));
    cmd.actuator_targets[0] =  1.0f; drv.apply(cmd); std::this_thread::sleep_for(std::chrono::seconds(1));
    cmd.actuator_targets[0] =  0.0f; drv.apply(cmd); std::this_thread::sleep_for(std::chrono::seconds(1));

    // Move servo 1
    cmd.actuator_targets = {0.0f, -1.0f, 0.0f}; drv.apply(cmd); std::this_thread::sleep_for(std::chrono::seconds(1));
    cmd.actuator_targets = {0.0f,  1.0f, 0.0f}; drv.apply(cmd); std::this_thread::sleep_for(std::chrono::seconds(1));

    // Move servo 2
    cmd.actuator_targets = {0.0f, 0.0f, -1.0f}; drv.apply(cmd); std::this_thread::sleep_for(std::chrono::seconds(1));
    cmd.actuator_targets = {0.0f, 0.0f,  1.0f}; drv.apply(cmd); std::this_thread::sleep_for(std::chrono::seconds(1));

    drv.stop();
    return 0;
}
