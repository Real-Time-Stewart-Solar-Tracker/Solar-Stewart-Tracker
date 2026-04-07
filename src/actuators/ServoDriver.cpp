#include "actuators/ServoDriver.hpp"

#include "actuators/PCA9685.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace solar {

ServoDriver::ServoDriver(Logger& log, Config cfg)
    : log_(log),
      cfg_(std::move(cfg)) {
}

ServoDriver::~ServoDriver() {
    stop();
}

bool ServoDriver::start() {
    // Hardware initialisation is kept separate from per-command writes.
    if (running_) {
        return true;
    }

    hardware_mode_ = false;

    if (!pca_) {
        if (cfg_.startup_policy != StartupPolicy::LogOnly) {
            auto candidate = std::make_unique<PCA9685>(cfg_.i2c_bus, cfg_.pca9685_addr);
            if (candidate->start(cfg_.pwm_hz)) {
                pca_ = std::move(candidate);
                hardware_mode_ = true;
            }
        }
    } else {
        if (pca_->start(cfg_.pwm_hz)) {
            hardware_mode_ = true;
        }
    }

    switch (cfg_.startup_policy) {
        case StartupPolicy::LogOnly:
            hardware_mode_ = false;
            break;

        case StartupPolicy::PreferHardware:
            // hardware_mode_ already reflects success/failure
            break;

        case StartupPolicy::RequireHardware:
            if (!hardware_mode_) {
                log_.error("ServoDriver: hardware required but PCA9685 failed to start");
                return false;
            }
            break;
    }

    running_ = true;
    apply_count_ = 0U;

    // Park once on startup so the rig begins from a known pose.
    if (cfg_.park_on_start) {
        applyPark_();
    }

    if (hardware_mode_) {
        log_.info("ServoDriver started in hardware mode");
    } else {
        log_.info("ServoDriver started in log-only mode");
    }

    return true;
}

void ServoDriver::stop() {
    if (!running_) {
        return;
    }

    if (cfg_.park_on_stop) {
        applyPark_();
    }

    if (pca_) {
        pca_->stop();
    }

    running_ = false;
    hardware_mode_ = false;
    log_.info("ServoDriver stopped");
}

void ServoDriver::apply(const ActuatorCommand& cmd) {
    if (!running_) {
        return;
    }

    ++apply_count_;

    // Apply each logical actuator target to its configured PCA9685 channel.
    for (std::size_t i = 0; i < 3U; ++i) {
        const ChannelConfig& ch = cfg_.ch[i];

        float deg = clampDeg_(cmd.actuator_targets[i], ch);
        deg = maybeInvertDeg_(deg, ch);
        const float pulse_us = degToPulseUs_(deg, ch);

        if (hardware_mode_ && pca_) {
            pca_->setPulseWidthUs(ch.channel, pulse_us);
        } else {
            if (cfg_.log_every_n <= 1 || (apply_count_ % static_cast<std::uint64_t>(cfg_.log_every_n) == 0U)) {
                log_.info("ServoDriver log-only apply: ch=" + std::to_string(ch.channel) +
                          " deg=" + std::to_string(deg) +
                          " pulse_us=" + std::to_string(pulse_us));
            }
        }
    }
}

bool ServoDriver::isRunning() const noexcept {
    return running_;
}

bool ServoDriver::isHardwareMode() const noexcept {
    return hardware_mode_;
}

void ServoDriver::injectPCA9685(std::unique_ptr<PCA9685> pca) {
    pca_ = std::move(pca);
}

const ServoDriver::Config& ServoDriver::config() const noexcept {
    return cfg_;
}

float ServoDriver::clampDeg_(const float deg, const ChannelConfig& ch) const {
    return std::max(ch.min_deg, std::min(deg, ch.max_deg));
}

float ServoDriver::maybeInvertDeg_(const float deg, const ChannelConfig& ch) const {
    if (!ch.invert) {
        return deg;
    }

    const float span = ch.max_deg - ch.min_deg;
    if (span <= 0.0F) {
        return ch.min_deg;
    }

    return ch.max_deg - (deg - ch.min_deg);
}

float ServoDriver::degToPulseUs_(const float deg, const ChannelConfig& ch) const {
    const float span_deg = ch.max_deg - ch.min_deg;
    const float span_us = ch.max_pulse_us - ch.min_pulse_us;

    if (span_deg <= 0.0F) {
        return ch.min_pulse_us;
    }

    const float alpha = (deg - ch.min_deg) / span_deg;
    return ch.min_pulse_us + alpha * span_us;
}

void ServoDriver::applyPark_() {
    ActuatorCommand park{};
    park.frame_id = 0U;
    park.status = CommandStatus::Ok;
    park.actuator_targets = {
        cfg_.ch[0].neutral_deg,
        cfg_.ch[1].neutral_deg,
        cfg_.ch[2].neutral_deg
    };

    const auto saved_running = running_;
    if (!saved_running) {
        return;
    }

    for (std::size_t i = 0; i < 3U; ++i) {
        const ChannelConfig& ch = cfg_.ch[i];

        float deg = clampDeg_(park.actuator_targets[i], ch);
        deg = maybeInvertDeg_(deg, ch);
        const float pulse_us = degToPulseUs_(deg, ch);

        if (hardware_mode_ && pca_) {
            pca_->setPulseWidthUs(ch.channel, pulse_us);
        } else {
            log_.info("ServoDriver park: ch=" + std::to_string(ch.channel) +
                      " deg=" + std::to_string(deg) +
                      " pulse_us=" + std::to_string(pulse_us));
        }
    }
}

} // namespace solar