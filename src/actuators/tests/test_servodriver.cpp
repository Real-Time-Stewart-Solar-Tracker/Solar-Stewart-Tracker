#include "actuators/ServoDriver.hpp"
#include "actuators/PCA9685.hpp"
#include "common/Logger.hpp"
#include "src/tests/support/test_common.hpp"

#include <memory>
#include <vector>

using solar::ActuatorCommand;
using solar::CommandStatus;
using solar::Logger;
using solar::PCA9685;
using solar::ServoDriver;

namespace {

struct FakePCA9685 final : public PCA9685 {
    struct Write {
        int channel;
        float pulse_us;
    };

    bool start_ok{true};
    bool started_flag{false};
    int start_calls{0};
    int stop_calls{0};
    float last_start_hz{0.0F};
    std::vector<Write> writes;

    FakePCA9685()
        : PCA9685(1, 0x40U) {
    }

    bool start(float pwm_hz) override {
        ++start_calls;
        last_start_hz = pwm_hz;
        started_flag = start_ok;
        return start_ok;
    }

    void stop() override {
        ++stop_calls;
        started_flag = false;
    }

    bool setPulseWidthUs(int channel, float pulse_us) override {
        writes.push_back({channel, pulse_us});
        return started_flag;
    }
};

ServoDriver::Config makeConfig() {
    ServoDriver::Config cfg{};
    cfg.startup_policy = ServoDriver::StartupPolicy::LogOnly;
    cfg.i2c_bus = 1;
    cfg.pca9685_addr = 0x40U;
    cfg.pwm_hz = 50.0F;
    cfg.park_on_start = true;
    cfg.park_on_stop = true;
    cfg.log_every_n = 1;

    cfg.ch[0].channel = 0;
    cfg.ch[1].channel = 1;
    cfg.ch[2].channel = 2;

    cfg.ch[0].min_deg = cfg.ch[1].min_deg = cfg.ch[2].min_deg = 0.0F;
    cfg.ch[0].max_deg = cfg.ch[1].max_deg = cfg.ch[2].max_deg = 180.0F;

    cfg.ch[0].min_pulse_us = cfg.ch[1].min_pulse_us = cfg.ch[2].min_pulse_us = 500.0F;
    cfg.ch[0].max_pulse_us = cfg.ch[1].max_pulse_us = cfg.ch[2].max_pulse_us = 2500.0F;

    cfg.ch[0].neutral_deg = cfg.ch[1].neutral_deg = cfg.ch[2].neutral_deg = 41.0F;

    return cfg;
}

ActuatorCommand makeCommand(float a, float b, float c) {
    ActuatorCommand cmd{};
    cmd.status = CommandStatus::Ok;
    cmd.actuator_targets = {a, b, c};
    return cmd;
}

} // namespace

TEST_CASE(ServoDriver_log_only_mode_starts_without_hardware) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::LogOnly;

    ServoDriver drv{log, cfg};
    REQUIRE(drv.start());
    REQUIRE(!drv.isHardwareMode());
    drv.stop();
}

TEST_CASE(ServoDriver_require_hardware_fails_fast_when_unavailable) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::RequireHardware;

    auto fake = std::make_unique<FakePCA9685>();
    fake->start_ok = false;

    ServoDriver drv{log, cfg};
    drv.injectPCA9685(std::move(fake));

    REQUIRE(!drv.start());
    REQUIRE(!drv.isHardwareMode());
}

TEST_CASE(ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::PreferHardware;

    auto fake = std::make_unique<FakePCA9685>();
    fake->start_ok = false;

    ServoDriver drv{log, cfg};
    drv.injectPCA9685(std::move(fake));

    REQUIRE(drv.start());
    REQUIRE(!drv.isHardwareMode());
    drv.stop();
}

TEST_CASE(ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::RequireHardware;

    auto fake = std::make_unique<FakePCA9685>();
    fake->start_ok = true;

    ServoDriver drv{log, cfg};
    drv.injectPCA9685(std::move(fake));

    REQUIRE(drv.start());
    REQUIRE(drv.isHardwareMode());
    drv.stop();
}

TEST_CASE(ServoDriver_apply_while_stopped_writes_nothing) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::RequireHardware;

    auto fake = std::make_unique<FakePCA9685>();
    auto* fake_ptr = fake.get();

    ServoDriver drv{log, cfg};
    drv.injectPCA9685(std::move(fake));

    drv.apply(makeCommand(10.0F, 20.0F, 30.0F));
    REQUIRE(fake_ptr->writes.empty());
}

TEST_CASE(ServoDriver_start_parks_to_neutral_when_enabled) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::RequireHardware;
    cfg.park_on_start = true;
    cfg.park_on_stop = false;

    auto fake = std::make_unique<FakePCA9685>();
    auto* fake_ptr = fake.get();

    ServoDriver drv{log, cfg};
    drv.injectPCA9685(std::move(fake));

    REQUIRE(drv.start());
    REQUIRE(fake_ptr->writes.size() == 3U);
    drv.stop();
}

TEST_CASE(ServoDriver_stop_parks_to_neutral_when_enabled) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::RequireHardware;
    cfg.park_on_start = false;
    cfg.park_on_stop = true;

    auto fake = std::make_unique<FakePCA9685>();
    auto* fake_ptr = fake.get();

    ServoDriver drv{log, cfg};
    drv.injectPCA9685(std::move(fake));

    REQUIRE(drv.start());
    fake_ptr->writes.clear();

    drv.stop();
    REQUIRE(fake_ptr->writes.size() == 3U);
}

TEST_CASE(ServoDriver_apply_clamps_and_writes_channels) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::RequireHardware;
    cfg.park_on_start = false;
    cfg.park_on_stop = false;

    cfg.ch[0].min_deg = cfg.ch[1].min_deg = cfg.ch[2].min_deg = 20.0F;
    cfg.ch[0].max_deg = cfg.ch[1].max_deg = cfg.ch[2].max_deg = 160.0F;

    auto fake = std::make_unique<FakePCA9685>();
    auto* fake_ptr = fake.get();

    ServoDriver drv{log, cfg};
    drv.injectPCA9685(std::move(fake));

    REQUIRE(drv.start());
    drv.apply(makeCommand(-100.0F, 90.0F, 999.0F));

    REQUIRE(fake_ptr->writes.size() == 3U);
    REQUIRE(fake_ptr->writes[0].channel == 0);
    REQUIRE(fake_ptr->writes[1].channel == 1);
    REQUIRE(fake_ptr->writes[2].channel == 2);
}

TEST_CASE(ServoDriver_inverted_channel_maps_correctly) {
    Logger log;
    auto cfg = makeConfig();
    cfg.startup_policy = ServoDriver::StartupPolicy::RequireHardware;
    cfg.park_on_start = false;
    cfg.park_on_stop = false;
    cfg.ch[1].invert = true;

    auto fake = std::make_unique<FakePCA9685>();
    auto* fake_ptr = fake.get();

    ServoDriver drv{log, cfg};
    drv.injectPCA9685(std::move(fake));

    REQUIRE(drv.start());
    drv.apply(makeCommand(30.0F, 50.0F, 70.0F));

    REQUIRE(fake_ptr->writes.size() == 3U);
    REQUIRE(fake_ptr->writes[0].pulse_us < fake_ptr->writes[1].pulse_us);
}
