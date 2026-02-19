#ifndef PCA9685_HPP
#define PCA9685_HPP

#include "interfaces.hpp"
#include <cstdint>

/**
 * @brief PCA9685 PWM servo driver / PCA9685 PWM舵机驱动
 *
 * Controls PCA9685 via I2C to drive servos.
 * 通过I2C控制PCA9685芯片驱动舵机。
 */
class PCA9685 : public IServoDriver {
public:
  /**
   * @brief Constructor / 构造函数
   * @param address I2C device address (default 0x40) / I2C设备地址
   * @param bus I2C bus number (RPi5 default: 1) / I2C总线号
   * @param freq PWM frequency (50Hz for servos) / PWM频率
   * @throws std::runtime_error on I2C init failure / I2C初始化失败时抛出
   */
  PCA9685(uint8_t address = 0x40, int bus = 1, int freq = 50);

  /**
   * @brief Destructor, auto-closes I2C device (RAII)
   *        析构函数，自动关闭I2C设备
   */
  ~PCA9685() override;

  // Move semantics support / 移动语义支持
  PCA9685(PCA9685 &&other) noexcept;
  PCA9685 &operator=(PCA9685 &&other) noexcept;

  /**
   * @brief Set servo angle (IServoDriver interface)
   *        设置舵机角度（IServoDriver接口）
   * @param channel Channel number (0-15) / 通道号
   * @param degree Angle (0-180 degrees) / 角度
   */
  void setServoAngle(int channel, int degree) override;

  /**
   * @brief Close I2C device (IServoDriver interface)
   *        关闭I2C设备（IServoDriver接口）
   */
  void close() override;

private:
  int fd_;           // I2C file descriptor / I2C文件描述符
  uint8_t address_;  // I2C device address / I2C设备地址
  double period_us_; // PWM period in microseconds / PWM周期（微秒）

  // PCA9685 register addresses / PCA9685寄存器地址
  static constexpr uint8_t MODE1 = 0x00;
  static constexpr uint8_t PRESCALE = 0xFE;
  static constexpr uint8_t LED0_ON_L = 0x06;

  // Servo pulse width parameters (microseconds) / 舵机脉宽参数（微秒）
  static constexpr int SERVO_MIN_US = 500;  // 0 degrees / 0度
  static constexpr int SERVO_MAX_US = 2500; // 180 degrees / 180度

  /**
   * @brief Initialize PCA9685 device / 初始化PCA9685设备
   * @param freq PWM frequency / PWM频率
   */
  void initDevice(int freq);

  /**
   * @brief Set PWM value / 设置PWM值
   * @param channel Channel number / 通道号
   * @param on PWM start time (0-4095) / PWM开始时间
   * @param off PWM end time (0-4095) / PWM结束时间
   */
  void setPWM(int channel, uint16_t on, uint16_t off);

  /**
   * @brief Write a single byte to register / 写入单个字节到寄存器
   * @param reg Register address / 寄存器地址
   * @param value Write value / 写入值
   */
  void writeRegister(uint8_t reg, uint8_t value);
};

#endif // PCA9685_HPP
