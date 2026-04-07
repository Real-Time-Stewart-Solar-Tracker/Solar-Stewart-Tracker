#include "sensors/manual/ADS1115ManualInput.hpp"
#include "hal/I2CBusPath.hpp"

#include "external/libgpiod_event_demo/gpioevent.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace solar {
namespace {

constexpr std::uint8_t REG_CONVERSION = 0x00U;
constexpr std::uint8_t REG_CONFIG     = 0x01U;
constexpr std::uint8_t REG_LO_THRESH  = 0x02U;
constexpr std::uint8_t REG_HI_THRESH  = 0x03U;

constexpr std::uint16_t OS_SINGLE_START = 0x8000U;

constexpr std::uint16_t MUX_AIN0_GND = 0x4000U;
constexpr std::uint16_t MUX_AIN1_GND = 0x5000U;
constexpr std::uint16_t MUX_AIN2_GND = 0x6000U;
constexpr std::uint16_t MUX_AIN3_GND = 0x7000U;

constexpr std::uint16_t PGA_6_144V = 0x0000U;
constexpr std::uint16_t PGA_4_096V = 0x0200U;
constexpr std::uint16_t PGA_2_048V = 0x0400U;
constexpr std::uint16_t PGA_1_024V = 0x0600U;
constexpr std::uint16_t PGA_0_512V = 0x0800U;
constexpr std::uint16_t PGA_0_256V = 0x0A00U;

constexpr std::uint16_t MODE_SINGLESHOT = 0x0100U;

constexpr std::uint16_t DR_8SPS   = 0x0000U;
constexpr std::uint16_t DR_16SPS  = 0x0020U;
constexpr std::uint16_t DR_32SPS  = 0x0040U;
constexpr std::uint16_t DR_64SPS  = 0x0060U;
constexpr std::uint16_t DR_128SPS = 0x0080U;
constexpr std::uint16_t DR_250SPS = 0x00A0U;
constexpr std::uint16_t DR_475SPS = 0x00C0U;
constexpr std::uint16_t DR_860SPS = 0x00E0U;

constexpr std::uint16_t COMP_ASSERT_1 = 0x0000U;
constexpr std::uint16_t RDY_LO_THRESH = 0x0000U;
constexpr std::uint16_t RDY_HI_THRESH = 0x8000U;

bool validChannel(const std::uint8_t ch) {
    return ch <= 3U;
}

std::int16_t toSigned16(const std::uint16_t raw) {
    return static_cast<std::int16_t>(raw);
}

} // namespace

// ---------------------------------------------------------------------------
// I2CDescriptor
// ---------------------------------------------------------------------------

bool ADS1115ManualInput::I2CDescriptor::open(const int bus, const std::uint8_t address) {
    close();

    const std::string path = i2cBusPath(bus);

    fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        fd_ = -1;
        return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, static_cast<int>(address)) < 0) {
        close();
        return false;
    }

    return true;
}

void ADS1115ManualInput::I2CDescriptor::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

// ---------------------------------------------------------------------------
// ADS1115ManualInput
// ---------------------------------------------------------------------------

ADS1115ManualInput::ADS1115ManualInput(const ADS1115ManualInputSettings& settings)
    : settings_(settings) {
}

ADS1115ManualInput::~ADS1115ManualInput() {
    stop();
}

void ADS1115ManualInput::registerCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

bool ADS1115ManualInput::start() {
    // Configure the ADC once, then let the ALERT/RDY GPIO edge wake onDataReady_()
    // for each completed conversion. No polling loop is involved.
    if (running_) {
        return true;
    }

    if (!settings_.enabled) {
        return false;
    }

    if (!validChannel(settings_.tilt_channel) || !validChannel(settings_.pan_channel)) {
        return false;
    }
    if (settings_.spare_channel != 0xFFU && !validChannel(settings_.spare_channel)) {
        return false;
    }

    if (!i2c_.open(settings_.i2c_bus, settings_.i2c_address)) {
        return false;
    }

    if (!configureReadyMode_()) {
        i2c_.close();
        return false;
    }

    current_sample_ = {};
    current_sample_.sequence = 0U;
    phase_ = Phase::Tilt;
    awaiting_conversion_ = false;

    drdy_pin_ = std::make_unique<GPIOPin>();
    drdy_pin_->registerCallback([this](const gpiod::edge_event& event) {
        if (!running_) {
            return;
        }
        if (event.type() != gpiod::edge_event::event_type::FALLING_EDGE) {
            return;
        }
        onDataReady_();
    });

    try {
        drdy_pin_->start(static_cast<int>(settings_.alert_rdy_gpio), settings_.gpio_chip_index);
    } catch (...) {
        drdy_pin_.reset();
        i2c_.close();
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!startNextConversion_(settings_.tilt_channel)) {
            drdy_pin_->stop();
            drdy_pin_.reset();
            i2c_.close();
            return false;
        }
        awaiting_conversion_ = true;
    }

    running_ = true;
    return true;
}

void ADS1115ManualInput::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (drdy_pin_) {
        drdy_pin_->stop();
        drdy_pin_.reset();
    }

    i2c_.close();
}

bool ADS1115ManualInput::writeRegister16_(const std::uint8_t reg, const std::uint16_t value) {
    if (!i2c_.isOpen()) {
        return false;
    }

    const std::uint8_t buffer[3] = {
        reg,
        static_cast<std::uint8_t>((value >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(value & 0xFFU)
    };

    return ::write(i2c_.get(), buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

bool ADS1115ManualInput::readRegister16_(const std::uint8_t reg, std::uint16_t& value) {
    if (!i2c_.isOpen()) {
        return false;
    }

    if (::write(i2c_.get(), &reg, 1) != 1) {
        return false;
    }

    std::uint8_t buffer[2]{};
    if (::read(i2c_.get(), buffer, sizeof(buffer)) != static_cast<ssize_t>(sizeof(buffer))) {
        return false;
    }

    value = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(buffer[0]) << 8U) |
        static_cast<std::uint16_t>(buffer[1]));
    return true;
}

bool ADS1115ManualInput::configureReadyMode_() {
    // Set the comparator thresholds to their ALERT/RDY mode values: MSB of
    // HI_THRESH = 1, MSB of LO_THRESH = 0. This causes the ALERT/RDY pin to
    // pulse on each completed conversion regardless of the result.
    if (!writeRegister16_(REG_LO_THRESH, RDY_LO_THRESH)) {
        return false;
    }
    if (!writeRegister16_(REG_HI_THRESH, RDY_HI_THRESH)) {
        return false;
    }
    return true;
}

bool ADS1115ManualInput::startNextConversion_(const std::uint8_t channel) {
    return writeRegister16_(REG_CONFIG, buildConfig_(channel));
}

bool ADS1115ManualInput::readLatestConversion_(float& voltage_v) {
    std::uint16_t raw = 0U;
    if (!readRegister16_(REG_CONVERSION, raw)) {
        return false;
    }

    const std::int16_t signed_raw = toSigned16(raw);
    const double scale = fullScaleVoltage_() / 32768.0;
    voltage_v = static_cast<float>(static_cast<double>(signed_raw) * scale);
    return true;
}

std::uint16_t ADS1115ManualInput::buildConfig_(const std::uint8_t channel) const {
    return static_cast<std::uint16_t>(
        OS_SINGLE_START |
        muxBits_(channel) |
        pgaBits_() |
        MODE_SINGLESHOT |
        drBits_() |
        COMP_ASSERT_1);
}

std::uint16_t ADS1115ManualInput::muxBits_(const std::uint8_t channel) const {
    switch (channel) {
        case 0U: return MUX_AIN0_GND;
        case 1U: return MUX_AIN1_GND;
        case 2U: return MUX_AIN2_GND;
        case 3U: return MUX_AIN3_GND;
        default: return MUX_AIN0_GND;
    }
}

std::uint16_t ADS1115ManualInput::pgaBits_() const {
    const float fs = settings_.full_scale_voltage;

    if (fs >= 6.144F) return PGA_6_144V;
    if (fs >= 4.096F) return PGA_4_096V;
    if (fs >= 2.048F) return PGA_2_048V;
    if (fs >= 1.024F) return PGA_1_024V;
    if (fs >= 0.512F) return PGA_0_512V;
    return PGA_0_256V;
}

std::uint16_t ADS1115ManualInput::drBits_() const {
    switch (settings_.sample_rate_hz) {
        case 8U:   return DR_8SPS;
        case 16U:  return DR_16SPS;
        case 32U:  return DR_32SPS;
        case 64U:  return DR_64SPS;
        case 128U: return DR_128SPS;
        case 250U: return DR_250SPS;
        case 475U: return DR_475SPS;
        case 860U: return DR_860SPS;
        default:   return DR_128SPS;
    }
}

double ADS1115ManualInput::fullScaleVoltage_() const {
    return settings_.full_scale_voltage > 0.0F
        ? static_cast<double>(settings_.full_scale_voltage)
        : 4.096;
}

void ADS1115ManualInput::onDataReady_() {
    ManualPotSample emitted{};
    SampleCallback cb{};
    bool emit = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!running_ || !awaiting_conversion_) {
            return;
        }

        awaiting_conversion_ = false;

        float voltage = 0.0F;
        if (!readLatestConversion_(voltage)) {
            return;
        }

        current_sample_.timestamp = ManualInputClock::now();

        std::uint8_t next_channel = settings_.tilt_channel;

        switch (phase_) {
            case Phase::Tilt:
                current_sample_.tilt_voltage_v = voltage;
                phase_ = Phase::Pan;
                next_channel = settings_.pan_channel;
                break;

            case Phase::Pan:
                current_sample_.pan_voltage_v = voltage;
                if (settings_.spare_channel != 0xFFU) {
                    phase_ = Phase::Spare;
                    next_channel = settings_.spare_channel;
                } else {
                    current_sample_.spare_voltage_v = 0.0F;
                    ++current_sample_.sequence;
                    emitted = current_sample_;
                    cb = callback_;
                    emit = true;

                    phase_ = Phase::Tilt;
                    next_channel = settings_.tilt_channel;
                }
                break;

            case Phase::Spare:
                current_sample_.spare_voltage_v = voltage;
                ++current_sample_.sequence;
                emitted = current_sample_;
                cb = callback_;
                emit = true;

                phase_ = Phase::Tilt;
                next_channel = settings_.tilt_channel;
                break;
        }

        if (startNextConversion_(next_channel)) {
            awaiting_conversion_ = true;
        }
    }

    if (emit && cb) {
        cb(emitted);
    }
}

} // namespace solar
