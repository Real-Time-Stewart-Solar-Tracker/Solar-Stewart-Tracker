#ifndef INTERFACES_HPP
#define INTERFACES_HPP

/**
 * @file interfaces.hpp
 * @brief SOLID abstract interface definitions / SOLID 抽象接口定义
 *
 * Design principles / 设计原则:
 * - SRP (Single Responsibility): Each interface defines only one capability
 *   单一职责: 每个接口只定义一种能力
 * - ISP (Interface Segregation): Clients depend only on minimal interfaces
 *   接口隔离: 客户端只依赖需要的最小接口
 * - DIP (Dependency Inversion): Upper modules depend on abstractions
 *   依赖倒置: 上层模块依赖抽象接口而非具体实现
 * - OCP (Open/Closed): New drivers/vision can be added without modifying
 * controller 开闭原则: 可以添加新的驱动/视觉实现而不修改控制器
 * - LSP (Liskov Substitution): Any implementation can seamlessly replace
 * another 里氏替换: 任何接口实现都可以无缝替换
 */

#include <functional>
#include <vector>

/**
 * @brief Vision detection callback type / 视觉检测回调类型
 * @param found Whether a target was detected / 是否检测到目标
 * @param dx Normalized X deviation [-1, 1] / 归一化X偏差
 * @param dy Normalized Y deviation [-1, 1] / 归一化Y偏差
 *
 * Used in event-driven architecture: when the vision system detects a frame,
 * it asynchronously notifies the controller via this callback, avoiding
 * polling.
 * 用于事件驱动架构：视觉系统检测到帧后通过此回调异步通知控制器，避免轮询。
 */
using VisionCallback = std::function<void(bool found, double dx, double dy)>;

/**
 * @brief Servo driver abstract interface (ISP) / 舵机驱动抽象接口
 *
 * Exposes only the minimal interface needed for servo control.
 * PCA9685 is the concrete implementation. Can be replaced with a mock for
 * testing. 仅暴露舵机控制所需的最小接口。PCA9685
 * 是其具体实现，可替换为模拟驱动用于测试。
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
 * @brief Inverse kinematics solver abstract interface (ISP)
 *        逆运动学求解器抽象接口
 *
 * Decouples kinematics solving from control logic.
 * RRSKinematics is the concrete implementation.
 * 将运动学求解与控制逻辑解耦。RRSKinematics 是其具体实现。
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
 * @brief Vision source abstract interface (ISP + DIP)
 *        视觉源抽象接口
 *
 * Vision capture interface with callback registration support.
 * VisionSystem is the concrete implementation (OpenCV + libcamera).
 * Can be replaced with a file replay source for testing.
 * 支持回调注册的视觉采集接口。VisionSystem
 * 是其具体实现，可替换为文件回放源用于测试。
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
   *
   * Whenever a new frame is processed, the vision system pushes results
   * to the registrant via this callback (event-driven mode).
   * 每当有新帧处理完成时，视觉系统将通过此回调推送结果（事件驱动模式）。
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
