#include "actuators/ServoDriver.hpp"

#include "actuators/PCA9685.hpp"
#include "hal/LinuxI2CDevice.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace solar {

ServoDriver::ServoDriver(Logger& log, Config cfg)
    : log_(log), cfg_(std::move(cfg)) {}

ServoDriver::ServoDriver(Logger& log, Config cfg, std::unique_ptr<solar::actuators::PCA9685> injected)
    : log_(log), cfg_(std::move(cfg)), pca_(std::move(injected)) {}

ServoDriver::~ServoDriver() {
    stop();
}

bool ServoDriver::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true; // already running
    }

    warnedStopped_.store(false);
    applyCount_.store(0);

#if defined(__linux__)
    // If no injected hardware, create the Linux implementation here.
    if (!pca_) {
        auto i2c = std::make_shared<solar::hal::LinuxI2CDevice>(cfg_.i2c_dev, cfg_.pca9685_addr);
        if (!i2c->ok()) {
            log_.warn("ServoDriver: failed to open " + cfg_.i2c_dev + " at addr 0x" +
                      std::to_string(cfg_.pca9685_addr) + " (log-only mode)");
        } else {
            solar::actuators::PCA9685::Config pcfg;
            pcfg.pwm_hz = cfg_.pwm_hz;
            pca_ = std::make_unique<solar::actuators::PCA9685>(i2c, pcfg);
        }
    }
#endif

    // Initialise PCA9685 if present; otherwise keep log-only fallback.
    if (pca_) {
        if (!pca_->init()) {
            log_.warn("ServoDriver: PCA9685 init failed (log-only mode)");
            pca_.reset();
        }
    }

    log_.info(std::string("ServoDriver started ") + (pca_ ? "(PCA9685)" : "(log-only)"));

    if (cfg_.park_on_start) {
        park_all();
    }

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

    log_.info("ServoDriver stopped");
}

float ServoDriver::deg_to_pulse_us(float deg, const ChannelConfig& c) const noexcept {
    // Safety clamp degrees
    const float lo_deg = std::min(c.min_deg, c.max_deg);
    const float hi_deg = std::max(c.min_deg, c.max_deg);
    deg = std::clamp(deg, lo_deg, hi_deg);

    // Normalize within [0..1]
    const float span_deg = (hi_deg - lo_deg);
    float t = 0.0f;
    if (span_deg > 1e-6f) {
        t = (deg - lo_deg) / span_deg;
    }
    t = std::clamp(t, 0.0f, 1.0f);

    // Mirror direction if needed
    if (c.invert) {
        t = 1.0f - t;
    }

    // Map to pulse width and clamp again for safety
    const float lo_us = std::min(c.min_pulse_us, c.max_pulse_us);
    const float hi_us = std::max(c.min_pulse_us, c.max_pulse_us);

    const float pulse = lo_us + t * (hi_us - lo_us);
    return std::clamp(pulse, lo_us, hi_us);
}

void ServoDriver::write_pulse_us(uint8_t channel, float pulse_us) noexcept {
    if (pca_) {
        (void)pca_->set_pulse_us(channel, pulse_us);
    } else {
        // deterministic fallback: log only (rate-limited in apply())
        (void)channel;
        (void)pulse_us;
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

    // Interpret actuator targets as SERVO DEGREES
    const float p0 = deg_to_pulse_us(cmd.actuator_targets[0], cfg_.ch[0]);
    const float p1 = deg_to_pulse_us(cmd.actuator_targets[1], cfg_.ch[1]);
    const float p2 = deg_to_pulse_us(cmd.actuator_targets[2], cfg_.ch[2]);

    // Rate-limited logging (avoid realtime jitter)
    const uint32_t n = applyCount_.fetch_add(1) + 1;
    if (cfg_.log_every_n > 0 && (n % cfg_.log_every_n) == 0) {
        log_.info("ServoDriver apply (deg->us): [" +
                  std::to_string(cmd.actuator_targets[0]) + " -> " + std::to_string(p0) + ", " +
                  std::to_string(cmd.actuator_targets[1]) + " -> " + std::to_string(p1) + ", " +
                  std::to_string(cmd.actuator_targets[2]) + " -> " + std::to_string(p2) + "] " +
                  (pca_ ? "(hw)" : "(log-only)"));
    }

    // Hardware output (fast, deterministic register writes)
    write_pulse_us(cfg_.ch[0].channel, p0);
    write_pulse_us(cfg_.ch[1].channel, p1);
    write_pulse_us(cfg_.ch[2].channel, p2);
}

} // namespace solar