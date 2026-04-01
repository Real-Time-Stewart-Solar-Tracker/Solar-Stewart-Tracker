#if !defined(__linux__)

int main() {
    return 77; // skipped
}

#else

#include "actuators/PCA9685.hpp"
#include "hal/LinuxI2CDevice.hpp"

#include <cstdlib>
#include <memory>
#include <string>

int main() {
    const char* run_env = std::getenv("SOLAR_RUN_I2C_HW_TESTS");
    if (!run_env || std::string(run_env) != "1") {
        return 77; // skipped unless explicitly enabled
    }

    const char* dev_env = std::getenv("SOLAR_I2C_DEV");
    const std::string dev_path = dev_env ? dev_env : "/dev/i2c-1";

    auto i2c = std::make_shared<solar::hal::LinuxI2CDevice>(dev_path, 0x40);
    if (!i2c->ok()) {
        return 1;
    }

    solar::actuators::PCA9685::Config cfg{};
    cfg.pwm_hz = 50.0f;

    solar::actuators::PCA9685 pca(i2c, cfg);

    if (!pca.init()) {
        return 1;
    }

    // Harmless real-register write path to prove Linux I2C comms work.
    if (!pca.set_pwm(15, 0, 0)) {
        return 1;
    }

    return 0;
}

#endif