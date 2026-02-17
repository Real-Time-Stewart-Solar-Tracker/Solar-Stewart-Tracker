#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

/**
 * @brief PID控制器类
 *
 * 对应Python版本的PIDController类。
 */
class PIDController {
public:
  /**
   * @brief 构造函数
   * @param kp 比例增益
   * @param ki 积分增益
   * @param kd 微分增益
   * @param max_tilt 最大倾斜角度（度）
   */
  PIDController(double kp, double ki, double kd, double max_tilt);

  /**
   * @brief PID更新
   * @param error 误差
   * @param dt 时间步长（秒）
   * @return 限幅后的输出角度
   */
  double update(double error, double dt);

  /**
   * @brief 重置PID状态
   */
  void reset();

  /**
   * @brief 获取当前输出值
   */
  double getOutput() const { return output; }

private:
  double kp, ki, kd; // PID参数
  double max_tilt;   // 最大倾斜角度
  double integral;   // 积分项
  double last_error; // 上次误差
  double output;     // 当前输出
};

#endif // PID_CONTROLLER_HPP
