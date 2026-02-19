#include "pca9685.hpp"
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

PCA9685::PCA9685(uint8_t address, int bus, int freq)
    : address_(address), fd_(-1), period_us_(0) {

  // Open I2C device / 打开I2C设备
  char filename[20];
  snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);

  fd_ = open(filename, O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open I2C bus: " +
                             std::string(filename));
  }

  // Set I2C slave address / 设置I2C从设备地址
  if (ioctl(fd_, I2C_SLAVE, address) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("Failed to set I2C slave address");
  }

  // Initialize device / 初始化设备
  initDevice(freq);

  std::cout << "[PCA9685] Initialized / 初始化完成 addr=0x" << std::hex
            << (int)address_ << std::dec << " freq=" << freq << "Hz"
            << std::endl;
}

PCA9685::~PCA9685() { close(); }

PCA9685::PCA9685(PCA9685 &&other) noexcept
    : fd_(other.fd_), address_(other.address_), period_us_(other.period_us_) {
  other.fd_ = -1;
}

PCA9685 &PCA9685::operator=(PCA9685 &&other) noexcept {
  if (this != &other) {
    close();
    fd_ = other.fd_;
    address_ = other.address_;
    period_us_ = other.period_us_;
    other.fd_ = -1;
  }
  return *this;
}

void PCA9685::initDevice(int freq) {
  // Enter SLEEP mode / 进入SLEEP模式
  writeRegister(MODE1, 0x10);
  usleep(5000); // Wait 5ms / 等待5ms

  // Calculate prescale value / 计算prescale值
  int prescale = std::round(25000000.0 / (4096.0 * freq)) - 1;
  writeRegister(PRESCALE, prescale);

  // Exit SLEEP, enable auto-increment / 退出SLEEP，启用自增模式
  writeRegister(MODE1, 0x20);
  usleep(5000);

  // Calculate period in microseconds / 计算周期（微秒）
  period_us_ = 1000000.0 / freq;
}

void PCA9685::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buf[2] = {reg, value};
  if (write(fd_, buf, 2) != 2) {
    throw std::runtime_error("Failed to write to I2C register");
  }
}

void PCA9685::setPWM(int channel, uint16_t on, uint16_t off) {
  if (channel < 0 || channel > 15) {
    throw std::invalid_argument("Channel must be between 0 and 15");
  }

  uint8_t reg = LED0_ON_L + 4 * channel;

  uint8_t buf[5] = {
      reg, static_cast<uint8_t>(on & 0xFF), static_cast<uint8_t>(on >> 8),
      static_cast<uint8_t>(off & 0xFF), static_cast<uint8_t>(off >> 8)};

  if (write(fd_, buf, 5) != 5) {
    throw std::runtime_error("Failed to set PWM");
  }
}

void PCA9685::setServoAngle(int channel, int degree) {
  // Clamp angle range / 限制角度范围
  if (degree < 0)
    degree = 0;
  if (degree > 180)
    degree = 180;

  // Calculate pulse width in microseconds / 计算脉宽（微秒）
  double pulse_us =
      SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * degree / 180.0;

  // Convert to PWM count value / 转换为PWM计数值
  int counts = static_cast<int>(pulse_us / period_us_ * 4096);

  // Set PWM / 设置PWM
  setPWM(channel, 0, counts);
}

void PCA9685::close() {
  if (fd_ >= 0) {
    // 仅关闭 I2C，不清零 PWM，让舵机保持最后位置
    // Only close I2C fd, keep PWM output so servos hold position
    ::close(fd_);
    fd_ = -1;
  }
}
