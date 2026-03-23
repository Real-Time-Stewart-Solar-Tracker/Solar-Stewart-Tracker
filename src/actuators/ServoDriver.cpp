#include "actuators/ServoDriver.hpp"

#include "actuators/PCA9685.hpp"
#include "hal/LinuxI2CDevice.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

namespace solar {

namespace {
std::string hexAddr(uint8_t addr) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%02X", static_cast<unsigned>(addr));
    return std::string(buf);
}
} // namespace

ServoDriver::ServoDriver(Logger& log, Config cfg)
    : log_(log), cfg_(std::move(cfg)) {}

ServoDriver::ServoDriver(Logger& log,
                         Config cfg,
                         std::unique_ptr<solar::actuators::PCA9685> injected)
    : log_(log), cfg_(std::move(cfg)), pca_(std::move(injected)) {}

ServoDriver::~ServoDriver() {
    stop();
}

const char* ServoDriver::runtimeModeName_(RuntimeMode mode) noexcept {
    switch (mode) {
        case RuntimeMode::Stopped:  return "Stopped";
        case RuntimeMode::Hardware: return "Hardware";
        case RuntimeMode::LogOnly:  return "LogOnly";
    }
    return "Unknown";
}

const char* ServoDriver::startupPolicyName_(StartupPolicy policy) noexcept {
    switch (policy) {
        case StartupPolicy::PreferHardware:  return "PreferHardware";
        case StartupPolicy::RequireHardware: return "RequireHardware";
        case StartupPolicy::LogOnly:         return "LogOnly";
    }
    return "Unknown";
}

void ServoDriver::setRuntimeMode_(RuntimeMode mode) noexcept {
    runtimeMode_.store(mode);
}

bool ServoDriver::tryCreateAndInitHardware_() {
    if (pca_) {
        if (pca_->init()) {
            setRuntimeMode_(RuntimeMode::Hardware);
            log_.info("ServoDriver: hardware initialised using injected PCA9685");
            return true;
        }

        log_.warn("ServoDriver: injected PCA9685 initialisation failed");
        pca_.reset();
        return false;
    }

#if defined(__linux__)
    auto i2c = std::make_shared<solar::hal::LinuxI2CDevice>(cfg_.i2c_dev, cfg_.pca9685_addr);
    if (!i2c->ok()) {
        log_.warn("ServoDriver: failed to open " + cfg_.i2c_dev +
                  " at " + hexAddr(cfg_.pca9685_addr));
        return false;
    }

    solar::actuators::PCA9685::Config pcfg;
    pcfg.pwm_hz = cfg_.pwm_hz;

    auto candidate = std::make_unique<solar::actuators::PCA9685>(i2c, pcfg);
    if (!candidate->init()) {
        log_.warn("ServoDriver: PCA9685 init failed on " + cfg_.i2c_dev +
                  " at " + hexAddr(cfg_.pca9685_addr));
        return false;
    }

    pca_ = std::move(candidate);
    setRuntimeMode_(RuntimeMode::Hardware);
    log_.info("ServoDriver: real hardware output active on " + cfg_.i2c_dev +
              " at " + hexAddr(cfg_.pca9685_addr));
    return true;
#else
    log_.warn("ServoDriver: real hardware output is not supported on this platform");
    return false;
#endif
}

bool ServoDriver::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true; // already running
    }

    warnedStopped_.store(false);
    applyCount_.store(0);
    setRuntimeMode_(RuntimeMode::Stopped);

    const StartupPolicy policy = cfg_.startup_policy;

    log_.info(std::string("ServoDriver: start requested with policy=") +
              startupPolicyName_(policy));

    bool haveHardware = false;

    switch (policy) {
        case StartupPolicy::LogOnly:
            pca_.reset();
            setRuntimeMode_(RuntimeMode::LogOnly);
            log_.info("ServoDriver: running in explicit log-only mode");
            break;

        case StartupPolicy::PreferHardware:
            haveHardware = tryCreateAndInitHardware_();
            if (!haveHardware) {
                pca_.reset();
                setRuntimeMode_(RuntimeMode::LogOnly);
                log_.warn("ServoDriver: hardware unavailable, falling back to log-only mode");
            }
            break;

        case StartupPolicy::RequireHardware:
            haveHardware = tryCreateAndInitHardware_();
            if (!haveHardware) {
                running_.store(false);
                setRuntimeMode_(RuntimeMode::Stopped);
                pca_.reset();

                log_.error("ServoDriver: RequireHardware requested but real hardware "
                           "could not be started; start() failed");
                return false;
            }
            break;
    }

    if (cfg_.park_on_start) {
        park_all();
    }

    log_.info(std::string("ServoDriver started in mode=") +
              runtimeModeName_(runtimeMode()));

    return true;
}

void ServoDriver::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return; // already stopped
    }

    if (cfg_.park_on_stop) {
        park_all();
    }

    log_.info(std::string("ServoDriver stopped from mode=") +
              runtimeModeName_(runtimeMode()));

    setRuntimeMode_(RuntimeMode::Stopped);
}

float ServoDriver::deg_to_pulse_us(float deg, const ChannelConfig& c) const noexcept {
    const float lo_deg = std::min(c.min_deg, c.max_deg);
    const float hi_deg = std::max(c.min_deg, c.max_deg);
    deg = std::clamp(deg, lo_deg, hi_deg);

    const float span_deg = (hi_deg - lo_deg);
    float t = 0.0f;
    if (span_deg > 1e-6f) {
        t = (deg - lo_deg) / span_deg;
    }
    t = std::clamp(t, 0.0f, 1.0f);

    if (c.invert) {
        t = 1.0f - t;
    }

    const float lo_us = std::min(c.min_pulse_us, c.max_pulse_us);
    const float hi_us = std::max(c.min_pulse_us, c.max_pulse_us);

    const float pulse = lo_us + t * (hi_us - lo_us);
    return std::clamp(pulse, lo_us, hi_us);
}

void ServoDriver::write_pulse_us(uint8_t channel, float pulse_us) noexcept {
    if (runtimeMode() == RuntimeMode::Hardware && pca_) {
        (void)pca_->set_pulse_us(channel, pulse_us);
    }
}

void ServoDriver::park_all() noexcept {
    for (const auto& c : cfg_.ch) {
        const float p = deg_to_pulse_us(c.neutral_deg, c);
        write_pulse_us(c.channel, p);
    }
}

void ServoDriver::apply(const ActuatorCommand& cmd) {
    if (!running_.load()) {
        bool expected = false;
        if (warnedStopped_.compare_exchange_strong(expected, true)) {
            log_.warn("ServoDriver: apply called while stopped");
        }
        return;
    }

    const float p0 = deg_to_pulse_us(cmd.actuator_targets[0], cfg_.ch[0]);
    const float p1 = deg_to_pulse_us(cmd.actuator_targets[1], cfg_.ch[1]);
    const float p2 = deg_to_pulse_us(cmd.actuator_targets[2], cfg_.ch[2]);

    const uint32_t n = applyCount_.fetch_add(1) + 1U;
    if (cfg_.log_every_n > 0 && (n % cfg_.log_every_n) == 0U) {
        log_.info("ServoDriver apply (deg->us): [" +
                  std::to_string(cmd.actuator_targets[0]) + " -> " + std::to_string(p0) + ", " +
                  std::to_string(cmd.actuator_targets[1]) + " -> " + std::to_string(p1) + ", " +
                  std::to_string(cmd.actuator_targets[2]) + " -> " + std::to_string(p2) + "] " +
                  "(mode=" + runtimeModeName_(runtimeMode()) + ")");
    }

    write_pulse_us(cfg_.ch[0].channel, p0);
    write_pulse_us(cfg_.ch[1].channel, p1);
    write_pulse_us(cfg_.ch[2].channel, p2);
}

} // namespace solar