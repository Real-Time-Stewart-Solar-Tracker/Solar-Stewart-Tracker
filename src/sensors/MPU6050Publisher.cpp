#include "sensors/MPU6050Publisher.hpp"
#include "hal/LinuxI2CDevice.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

namespace solar {

// ---------------------------------------------------------------------------
// MPU6050 register addresses
// ---------------------------------------------------------------------------
static constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
static constexpr uint8_t REG_CONFIG      = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
static constexpr uint8_t REG_ACCEL_XOUT  = 0x3B;
static constexpr uint8_t REG_WHO_AM_I    = 0x75;

// Scale factors for default full-scale ranges
static constexpr float ACCEL_SCALE = 16384.0f; // ±2g  -> LSB/g
static constexpr float GYRO_SCALE  = 131.0f;   // ±250°/s -> LSB/(°/s)
static constexpr float DEG_TO_RAD  = 3.14159265358979323846f / 180.0f;

// ---------------------------------------------------------------------------
MPU6050Publisher::MPU6050Publisher(Logger& log, Config cfg)
    : log_(log), cfg_(std::move(cfg)) {
    dev_ = std::make_unique<solar::hal::LinuxI2CDevice>(cfg_.i2c_bus, cfg_.i2c_addr);
}

MPU6050Publisher::~MPU6050Publisher() {
    stop();
}

void MPU6050Publisher::registerSetpointCallback(SetpointCallback cb) {
    cb_ = std::move(cb);
}

bool MPU6050Publisher::start() {
    if (running_.exchange(true)) return true; // already running

    if (!initHardware_()) {
        log_.error("MPU6050Publisher: hardware init failed");
        running_.store(false);
        return false;
    }

    first_sample_ = true;
    roll_  = 0.0f;
    pitch_ = 0.0f;
    frame_id_ = 0;

    thread_ = std::thread([this] { loop_(); });
    log_.info("MPU6050Publisher started");
    return true;
}

void MPU6050Publisher::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
    log_.info("MPU6050Publisher stopped");
}

bool MPU6050Publisher::isRunning() const noexcept {
    return running_.load();
}

// ---------------------------------------------------------------------------
bool MPU6050Publisher::initHardware_() {
    auto* linux_dev = dynamic_cast<solar::hal::LinuxI2CDevice*>(dev_.get());
    if (linux_dev && !linux_dev->ok()) {
        log_.error("MPU6050Publisher: failed to open " + cfg_.i2c_bus);
        return false;
    }

    // Verify device identity
    uint8_t who = 0;
    if (!dev_->read_reg_bytes(REG_WHO_AM_I, &who, 1)) {
        log_.error("MPU6050Publisher: WHO_AM_I read failed (check wiring)");
        return false;
    }
    // 0x68 = MPU6050/MPU6000, 0x70 = MPU6500, 0x72 = MPU6000 variant
    if (who != 0x68 && who != 0x70 && who != 0x72) {
        log_.error("MPU6050Publisher: unexpected WHO_AM_I=0x" + std::to_string(who));
        return false;
    }

    // Wake up (clear sleep bit in PWR_MGMT_1)
    dev_->write_reg_u8(REG_PWR_MGMT_1, 0x00);

    // Digital low-pass filter: ~21 Hz bandwidth (reduces vibration noise)
    dev_->write_reg_u8(REG_CONFIG, 0x04);

    // Gyroscope full-scale: ±250°/s (highest sensitivity)
    dev_->write_reg_u8(REG_GYRO_CONFIG, 0x00);

    // Accel full-scale is ±2g by default (ACCEL_CONFIG reset value = 0x00)

    // Allow sensor to stabilise after wake-up
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    log_.info("MPU6050Publisher: sensor ready (WHO_AM_I=0x" + std::to_string(who) + ")");
    return true;
}

// ---------------------------------------------------------------------------
bool MPU6050Publisher::readRaw_(RawIMU& out) {
    // Read 14 bytes starting at ACCEL_XOUT_H:
    //   [0-1]  ACCEL_X  [2-3]  ACCEL_Y  [4-5]  ACCEL_Z
    //   [6-7]  TEMP     (skipped)
    //   [8-9]  GYRO_X   [10-11] GYRO_Y  [12-13] GYRO_Z (Z not needed)
    uint8_t buf[14]{};
    if (!dev_->read_reg_bytes(REG_ACCEL_XOUT, buf, 14)) return false;

    out.ax = static_cast<int16_t>((buf[0]  << 8) | buf[1]);
    out.ay = static_cast<int16_t>((buf[2]  << 8) | buf[3]);
    out.az = static_cast<int16_t>((buf[4]  << 8) | buf[5]);
    out.gx = static_cast<int16_t>((buf[8]  << 8) | buf[9]);
    out.gy = static_cast<int16_t>((buf[10] << 8) | buf[11]);
    return true;
}

// ---------------------------------------------------------------------------
void MPU6050Publisher::loop_() {
    const auto period = std::chrono::nanoseconds(
        static_cast<long long>(1'000'000'000LL / cfg_.sample_hz));
    const float dt = static_cast<float>(period.count()) * 1e-9f;

    auto next = std::chrono::steady_clock::now() + period;

    while (running_.load()) {
        const auto now = std::chrono::steady_clock::now();

        RawIMU raw{};
        if (!readRaw_(raw)) {
            log_.error("MPU6050Publisher: I2C read failed");
            std::this_thread::sleep_until(next);
            next += period;
            continue;
        }

        // Convert to physical units
        const float ax = raw.ax / ACCEL_SCALE; // g
        const float ay = raw.ay / ACCEL_SCALE;
        const float az = raw.az / ACCEL_SCALE;
        const float gx_rads = (raw.gx / GYRO_SCALE) * DEG_TO_RAD;
        const float gy_rads = (raw.gy / GYRO_SCALE) * DEG_TO_RAD;

        // Accelerometer-derived angles (static, noisy but drift-free)
        const float acc_roll  = std::atan2(ay, az);
        const float acc_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        if (first_sample_) {
            // Bootstrap filter from accelerometer on first sample
            roll_         = acc_roll;
            pitch_        = acc_pitch;
            first_sample_ = false;
        } else {
            // Complementary filter: integrate gyro, correct with accel
            roll_  = cfg_.alpha * (roll_  + gx_rads * dt) + (1.0f - cfg_.alpha) * acc_roll;
            pitch_ = cfg_.alpha * (pitch_ + gy_rads * dt) + (1.0f - cfg_.alpha) * acc_pitch;
        }

        // Clamp to platform mechanical limits
        const float tilt = std::clamp(roll_,  -cfg_.max_tilt_rad, cfg_.max_tilt_rad);
        const float pan  = std::clamp(pitch_, -cfg_.max_pan_rad,  cfg_.max_pan_rad);

        if (cb_) {
            PlatformSetpoint sp;
            sp.frame_id  = ++frame_id_;
            sp.t_control = now;
            sp.tilt_rad  = tilt;
            sp.pan_rad   = pan;
            cb_(sp);
        }

        std::this_thread::sleep_until(next);
        next += period;
    }
}

} // namespace solar
