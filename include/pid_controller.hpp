#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

/**
 * @brief PID Controller class / PID控制器类
 *
 * Implements Proportional-Integral-Derivative control algorithm.
 * 实现比例-积分-微分控制算法。
 *
 * SOLID design / SOLID设计:
 * - SRP: Only responsible for PID computation, no I/O
 *   单一职责: 仅负责PID计算，不涉及任何I/O操作
 * - Value semantics: safely copyable and assignable (no resource management)
 *   值语义: 可安全拷贝和赋值（无资源管理）
 *
 * All data is private, state is accessed via getters.
 * 所有数据私有，通过getter安全获取状态。
 */
class PIDController {
public:
  /**
   * @brief Constructor / 构造函数
   * @param kp Proportional gain / 比例增益
   * @param ki Integral gain / 积分增益
   * @param kd Derivative gain / 微分增益
   * @param max_tilt Maximum tilt angle in degrees / 最大倾斜角度（度）
   */
  PIDController(double kp, double ki, double kd, double max_tilt);

  /**
   * @brief PID update / PID更新
   * @param error Error value / 误差
   * @param dt Time step in seconds / 时间步长（秒）
   * @return Clamped output angle / 限幅后的输出角度
   */
  double update(double error, double dt);

  /**
   * @brief Reset PID state / 重置PID状态
   */
  void reset();

  // Getters (safe interface, no direct internal state exposure)
  // Getters（安全接口，不暴露内部状态的直接引用）
  double getOutput() const { return output_; }
  double getKp() const { return kp_; }
  double getKi() const { return ki_; }
  double getKd() const { return kd_; }
  double getMaxTilt() const { return max_tilt_; }

private:
  double kp_, ki_, kd_; // PID parameters / PID参数
  double max_tilt_;     // Max tilt angle / 最大倾斜角度
  double integral_;     // Integral term / 积分项
  double last_error_;   // Previous error / 上次误差
  double output_;       // Current output / 当前输出
};

#endif // PID_CONTROLLER_HPP
