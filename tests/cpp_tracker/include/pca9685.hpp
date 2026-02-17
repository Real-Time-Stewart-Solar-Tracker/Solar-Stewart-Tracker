#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <cstdint>

/**
 * @brief PCA9685 PWM舵机驱动类
 *
 * 通过I2C接口控制PCA9685芯片，驱动舵机。
 * 对应Python版本的PCA9685类。
 */
class PCA9685 {
public:
  /**
   * @brief 构造函数
   * @param address I2C设备地址（默认0x40）
   * @param bus I2C总线号（树莓派5默认为1）
   * @param freq PWM频率（舵机用50Hz）
   */
  PCA9685(uint8_t address = 0x40, int bus = 1, int freq = 50);

  /**
   * @brief 析构函数，关闭I2C设备
   */
  ~PCA9685();

  /**
   * @brief 设置舵机角度
   * @param channel 通道号（0-15）
   * @param degree 角度（0-180度）
   */
  void setServoAngle(int channel, int degree);

  /**
   * @brief 设置PWM值
   * @param channel 通道号
   * @param on PWM开始时间（0-4095）
   * @param off PWM结束时间（0-4095）
   */
  void setPWM(int channel, uint16_t on, uint16_t off);

  /**
   * @brief 关闭I2C设备
   */
  void close();

private:
  int fd;           // I2C文件描述符
  uint8_t address;  // I2C设备地址
  double period_us; // PWM周期（微秒）

  // PCA9685寄存器地址
  static constexpr uint8_t MODE1 = 0x00;
  static constexpr uint8_t PRESCALE = 0xFE;
  static constexpr uint8_t LED0_ON_L = 0x06;

  // 舵机脉宽参数（微秒）
  static constexpr int SERVO_MIN_US = 500;  // 0度
  static constexpr int SERVO_MAX_US = 2500; // 180度

  /**
   * @brief 初始化PCA9685设备
   * @param freq PWM频率
   */
  void initDevice(int freq);

  /**
   * @brief 写入单个字节到寄存器
   * @param reg 寄存器地址
   * @param value 写入值
   */
  void writeRegister(uint8_t reg, uint8_t value);
};

#endif // PCA9685_HPP
