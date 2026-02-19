#ifndef TRACKER_CONTROLLER_HPP
#define TRACKER_CONTROLLER_HPP

#include "interfaces.hpp"
#include "pid_controller.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

/**
 * @brief Tracker Controller / 追踪控制器
 *
 * Coordinates Vision → PID → IK → Servo control pipeline.
 * 协调 视觉 → PID → IK → 舵机 控制流水线。
 *
 * Uses callback from VisionSystem and a 50Hz timer thread for control loop.
 * 使用 VisionSystem 的回调和 50Hz 定时器线程进行控制循环。
 */
class TrackerController {
public:
  /**
   * @brief Constructor - Dependency Injection / 构造函数 — 依赖注入
   * @param servo Servo driver (ownership transfer) / 舵机驱动（所有权转移）
   * @param ik IK solver (ownership transfer) / 逆运动学求解器（所有权转移）
   * @param vision Vision source (ownership transfer) / 视觉源（所有权转移）
   *
   * Implements DIP: controller doesn't create concrete implementations,
   * but accepts ownership of abstract interfaces for testing and replacement.
   * 通过依赖注入实现DIP原则：控制器不创建具体实现，
   * 而是接受抽象接口的所有权，便于测试和替换。
   */
  TrackerController(std::unique_ptr<IServoDriver> servo,
                    std::unique_ptr<IKinematicsSolver> ik,
                    std::unique_ptr<IVisionSource> vision);

  ~TrackerController();

  // Disable copy and move / 禁止拷贝和移动
  TrackerController(const TrackerController &) = delete;
  TrackerController &operator=(const TrackerController &) = delete;

  /**
   * @brief Start tracking system / 启动追踪系统
   *
   * 1. Center servos / 舵机归中
   * 2. Register vision callback / 注册视觉回调
   * 3. Start timer thread / 启动定时器线程
   * 4. Start vision capture / 启动视觉采集
   */
  void start();

  /**
   * @brief Stop tracking system gracefully / 停止追踪系统，优雅退出
   */
  void stop();

private:
  // Injected components (smart pointers, RAII auto-manages lifetime)
  // 依赖注入的组件（智能指针，RAII自动管理生命周期）
  std::unique_ptr<IServoDriver> servo_;
  std::unique_ptr<IKinematicsSolver> ik_;
  std::unique_ptr<IVisionSource> vision_;

  // PID controllers (stack objects, no dynamic allocation needed)
  // PID控制器（栈上对象，无需动态分配）
  PIDController pid_pitch_;
  PIDController pid_roll_;

  // Control state (mutex protected) / 控制状态（mutex保护）
  std::mutex state_mutex_;
  bool target_found_;
  double target_dx_;
  double target_dy_;

  // Timer thread / 定时器线程
  std::thread timer_thread_;
  std::atomic<bool> running_;
  std::condition_variable timer_cv_;
  std::mutex timer_mutex_;

  // Servo smoothing state / 舵机平滑状态
  std::vector<int> last_sent_angles_;
  std::vector<double> current_servo_angles_;
  static constexpr double SERVO_SMOOTH_FACTOR = 0.15;
  static constexpr double CONTROL_DT = 0.02; // 50Hz
  static constexpr int INITIAL_ANGLE = 60;

  // Status output counter / 状态输出计数
  int loop_count_;

  /**
   * @brief Vision callback handler / 视觉回调处理器
   *
   * Called by vision system in capture thread (event-driven).
   * Only updates shared state, no heavy computation, ensures low latency.
   * 由视觉系统在采集线程中调用（事件驱动）。
   * 仅更新共享状态，不做重计算，确保低延迟。
   */
  void onVisionUpdate(bool found, double dx, double dy);

  /**
   * @brief Timer thread loop / 定时器线程循环
   *
   * Uses condition_variable::wait_for for precise timing,
   * replacing sleep_for busy-wait approach.
   * Can be instantly woken by stop() via notify_all().
   * 使用 condition_variable::wait_for 实现精确定时，替代 sleep_for 忙等待。
   * 可通过 stop() 的 notify_all() 立即唤醒。
   */
  void timerLoop();

  /**
   * @brief Execute one control step / 执行一次控制计算
   *
   * PID update → IK solve → Servo smooth → Command send
   * PID更新 → IK求解 → 舵机平滑 → 命令发送
   */
  void controlStep();
};

#endif // TRACKER_CONTROLLER_HPP
