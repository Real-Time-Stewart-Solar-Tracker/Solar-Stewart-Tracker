#include "system/BackendCoordinator.hpp"

#include "hal/LinuxI2CDevice.hpp"
#include "sensors/imu/Mpu6050Publisher.hpp"
#include "sensors/manual/ADS1115ManualInput.hpp"

#include <utility>

namespace solar {

// ---------------------------------------------------------------------------
// ImuCallbackForwarder
// ---------------------------------------------------------------------------

void BackendCoordinator::ImuCallbackForwarder::hasSample(const IImuSample& sample) {
    control::ImuSample mapped{};
    mapped.ax           = sample.ax_mps2;
    mapped.ay           = sample.ay_mps2;
    mapped.az           = sample.az_mps2;
    mapped.timestamp_us = sample.timestamp_us;
    mapped.valid        = sample.valid;

    if (callback_) {
        callback_(mapped);
    }
}

// ---------------------------------------------------------------------------
// BackendCoordinator
// ---------------------------------------------------------------------------

BackendCoordinator::BackendCoordinator(const app::AppConfig& cfg,
                                       ManualCallback manual_cb,
                                       ImuCallback imu_cb)
    : config_(cfg),
      manual_callback_(std::move(manual_cb)),
      imu_callback_(std::move(imu_cb)),
      manual_backend_requested_(cfg.manual_input_backend == app::ManualInputBackend::ADS1115
                                 && cfg.ads1115_manual.enabled),
      imu_backend_requested_(cfg.imu_backend == app::ImuBackend::Mpu6050Gpio
                              && cfg.mpu6050.enabled) {
}

BackendCoordinator::~BackendCoordinator() {
    stop();
}

BackendCoordinator::StartResult BackendCoordinator::start(Logger& log) {
    StartResult result{};

    if (!startManual_(log)) {
        result.manual_ok = false;
        return result;
    }

    if (!startImu_(log)) {
        result.imu_ok = false;
    }

    return result;
}

void BackendCoordinator::stop() {
    stopImu_();
    stopManual_();
}

bool BackendCoordinator::startManual_(Logger& log) {
    if (!manual_backend_requested_) {
        return true;
    }

    ADS1115ManualInputSettings s{};
    s.enabled            = config_.ads1115_manual.enabled;
    s.i2c_bus            = config_.ads1115_manual.i2c_bus;
    s.i2c_address        = config_.ads1115_manual.i2c_address;
    s.gpio_chip_index    = config_.ads1115_manual.gpio_chip_index;
    s.alert_rdy_gpio     = config_.ads1115_manual.alert_rdy_gpio;
    s.tilt_channel       = config_.ads1115_manual.tilt_channel;
    s.pan_channel        = config_.ads1115_manual.pan_channel;
    s.spare_channel      = config_.ads1115_manual.spare_channel;
    s.full_scale_voltage = config_.ads1115_manual.full_scale_voltage;
    s.sample_rate_hz     = config_.ads1115_manual.sample_rate_hz;

    manual_input_ = std::make_unique<ADS1115ManualInput>(s);
    manual_input_->registerCallback([this](const ManualPotSample& sample) {
        if (manual_callback_) {
            manual_callback_(sample);
        }
    });

    if (!manual_input_->start()) {
        manual_input_.reset();
        return false;
    }

    log.info("BackendCoordinator: ADS1115 manual input backend started");
    return true;
}

void BackendCoordinator::stopManual_() {
    if (manual_input_) {
        manual_input_->stop();
        manual_input_.reset();
    }
}

bool BackendCoordinator::startImu_(Logger& log) {
    if (!imu_backend_requested_) {
        return true;
    }

    imu_i2c_ = std::make_unique<LinuxI2CDevice>();

    // Map only the system-integration fields from AppConfig. Register-level
    // initialisation constants are encapsulated inside Mpu6050Publisher.
    Mpu6050Config cfg{};
    cfg.i2c.bus                 = config_.mpu6050.i2c_bus;
    cfg.i2c.address             = config_.mpu6050.i2c_address;
    cfg.gpio_pin_no             = static_cast<int>(config_.mpu6050.data_ready_gpio);
    cfg.gpio_chip_no            = config_.mpu6050.gpio_chip_index;
    cfg.who_am_i_expected       = config_.mpu6050.who_am_i_expected;
    cfg.startup_discard_samples = config_.mpu6050.startup_discard_samples;

    imu_input_ = std::make_unique<Mpu6050Publisher>(*imu_i2c_, cfg);
    imu_callback_forwarder_ = std::make_unique<ImuCallbackForwarder>(imu_callback_);
    imu_input_->registerEventCallback(imu_callback_forwarder_.get());

    if (!imu_input_->start()) {
        imu_input_.reset();
        imu_callback_forwarder_.reset();
        imu_i2c_.reset();
        return false;
    }

    log.info("BackendCoordinator: MPU-6050/ICM-20600 IMU backend started");
    return true;
}

void BackendCoordinator::stopImu_() {
    if (imu_input_) {
        imu_input_->stop();
        imu_input_.reset();
    }
    imu_callback_forwarder_.reset();
    imu_i2c_.reset();
}

} // namespace solar
