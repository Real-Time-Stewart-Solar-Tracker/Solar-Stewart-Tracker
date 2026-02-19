#ifndef INTERFACES_HPP
#define INTERFACES_HPP

/**
 * @file interfaces.hpp
 * @brief Abstract interface definitions / 抽象接口定义
 */

#include <functional>
#include <vector>

/**
 * @brief Vision detection callback type / 视觉检测回调类型
 * @param found Whether a target was detected / 是否检测到目标
 * @param dx Normalized X deviation [-1, 1] / 归一化X偏差
 * @param dy Normalized Y deviation [-1, 1] / 归一化Y偏差
 */
using VisionCallback = std::function<void(bool found, double dx, double dy)>;

/**
 * @brief Servo driver interface / 舵机驱动接口
 */
class IServoDriver {
public:
  virtual ~IServoDriver() = default;

  /**
   * @brief Set servo angle / 设置舵机角度
   * @param channel Channel number / 通道号
   * @param degree Angle in degrees (0-180) / 角度（0-180度）
   */
  virtual void setServoAngle(int channel, int degree) = 0;

  /**
   * @brief Close the driver / 关闭驱动器
   */
  virtual void close() = 0;

  // Disable copy / 禁止拷贝
  IServoDriver(const IServoDriver &) = delete;
  IServoDriver &operator=(const IServoDriver &) = delete;

protected:
  IServoDriver() = default;
};

/**
 * @brief Inverse kinematics solver interface / 逆运动学求解器接口
 */
class IKinematicsSolver {
public:
  virtual ~IKinematicsSolver() = default;

  /**
   * @brief Inverse kinematics solve / 逆运动学求解
   * @param pitch_deg Pitch angle in degrees / 俯仰角（度）
   * @param roll_deg Roll angle in degrees / 滚转角（度）
   * @return Servo angles in degrees / 各舵机角度（度）
   */
  virtual std::vector<int> solve(double pitch_deg, double roll_deg) = 0;

  // Disable copy / 禁止拷贝
  IKinematicsSolver(const IKinematicsSolver &) = delete;
  IKinematicsSolver &operator=(const IKinematicsSolver &) = delete;

protected:
  IKinematicsSolver() = default;
};

/**
 * @brief Vision source interface / 视觉源接口
 */
class IVisionSource {
public:
  virtual ~IVisionSource() = default;

  /**
   * @brief Start vision capture / 启动视觉采集
   */
  virtual void start() = 0;

  /**
   * @brief Stop vision capture / 停止视觉采集
   */
  virtual void stop() = 0;

  /**
   * @brief Register vision callback / 注册视觉回调
   * @param cb Detection result callback / 检测结果回调函数
   */
  virtual void setCallback(VisionCallback cb) = 0;

  /**
   * @brief Clean up resources / 清理资源
   */
  virtual void cleanup() = 0;

  // Disable copy / 禁止拷贝
  IVisionSource(const IVisionSource &) = delete;
  IVisionSource &operator=(const IVisionSource &) = delete;

protected:
  IVisionSource() = default;
};

#endif // INTERFACES_HPP
