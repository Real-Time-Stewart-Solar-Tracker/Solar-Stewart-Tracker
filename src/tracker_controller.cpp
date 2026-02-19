#include "tracker_controller.hpp"
#include <cmath>
#include <iostream>

TrackerController::TrackerController(std::unique_ptr<IServoDriver> servo,
                                     std::unique_ptr<IKinematicsSolver> ik,
                                     std::unique_ptr<IVisionSource> vision)
    : servo_(std::move(servo)), ik_(std::move(ik)), vision_(std::move(vision)),
      pid_pitch_(0.3, 0.0, 0.0, 35.0), pid_roll_(0.3, 0.0, 0.0, 35.0),
      target_found_(false), target_dx_(0.0), target_dy_(0.0), running_(false),
      last_sent_angles_{-1, -1, -1},
      current_servo_angles_{static_cast<double>(INITIAL_ANGLE),
                            static_cast<double>(INITIAL_ANGLE),
                            static_cast<double>(INITIAL_ANGLE)},
      loop_count_(0) {}

TrackerController::~TrackerController() { stop(); }

void TrackerController::start() {
  std::cout << "[Controller] Starting tracking system / 启动追踪系统..."
            << std::endl;

  // 1. Center servos / 舵机归中
  if (servo_) {
    for (int ch = 0; ch < 3; ch++) {
      servo_->setServoAngle(ch, INITIAL_ANGLE);
    }
    std::cout << "[Controller] Servos centered / 舵机已归中 (" << INITIAL_ANGLE
              << "°)" << std::endl;
  }

  std::cout << "[Controller] PID ready / 控制器就绪 (Kp=0.3, maxTilt=35°)"
            << std::endl;
  std::cout << "[Controller] IK engine ready / IK引擎就绪" << std::endl;

  // 2. 注册视觉回调 / Register vision callback
  vision_->setCallback([this](bool found, double dx, double dy) {
    this->onVisionUpdate(found, dx, dy);
  });

  // 3. 启动定时器线程（50Hz）/ Start timer thread
  running_ = true;
  timer_thread_ = std::thread(&TrackerController::timerLoop, this);

  // 4. Start vision capture / 启动视觉采集
  vision_->start();
  std::cout << "[Controller] Vision system started / 视觉系统已启动"
            << std::endl;

  std::cout << "[Controller] 🚀 Tracking started / 开始追踪！" << std::endl
            << std::endl;
}

void TrackerController::stop() {
  if (!running_.exchange(false)) {
    return; // Already stopped / 已经停止
  }

  std::cout << std::endl << "[Controller] Stopping / 正在停止..." << std::endl;

  // 1. Wake timer thread to exit / 唤醒定时器线程使其退出
  timer_cv_.notify_all();
  if (timer_thread_.joinable()) {
    timer_thread_.join();
  }

  // 2. Stop vision system / 停止视觉系统
  vision_->cleanup();

  // 3. Center servos and close / 舵机回中并关闭
  if (servo_) {
    for (int ch = 0; ch < 3; ch++) {
      servo_->setServoAngle(ch, INITIAL_ANGLE);
    }
    // Wait for servos to reach position before closing
    // 等待舵机到位后关闭
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    servo_->close();
  }

  std::cout << "[Controller] ✅ Safely exited / 已安全退出" << std::endl;
}

void TrackerController::onVisionUpdate(bool found, double dx, double dy) {
  // 视觉回调，更新共享状态 / Vision callback, update shared state
  std::lock_guard<std::mutex> lock(state_mutex_);
  target_found_ = found;
  target_dx_ = dx;
  target_dy_ = dy;
}

void TrackerController::timerLoop() {
  // 50Hz 定时器循环，用 condition_variable 实现可中断的等待
  // 50Hz timer loop, interruptible via condition_variable

  while (running_) {
    auto t_start = std::chrono::steady_clock::now();

    // Execute control computation / 执行控制计算
    controlStep();

    // 等待剩余时间 / Wait remaining time
    auto t_end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(t_end - t_start).count();
    double remaining = CONTROL_DT - elapsed;

    if (remaining > 0) {
      std::unique_lock<std::mutex> lock(timer_mutex_);
      timer_cv_.wait_for(lock, std::chrono::duration<double>(remaining),
                         [this]() { return !running_.load(); });
    }
  }
}

void TrackerController::controlStep() {
  // 读取视觉状态 / Read vision state
  bool found;
  double dx, dy;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    found = target_found_;
    dx = target_dx_;
    dy = target_dy_;
  }

  // 计算目标姿态 / Compute target pose
  double target_pitch, target_roll;
  if (found) {
    target_pitch = dx * 50.0;
    target_roll = dy * 50.0;
  } else {
    target_pitch = 0.0;
    target_roll = 0.0;
  }

  // PID 更新
  double pitch =
      pid_pitch_.update(target_pitch - pid_pitch_.getOutput(), CONTROL_DT);
  double roll =
      pid_roll_.update(target_roll - pid_roll_.getOutput(), CONTROL_DT);

  // IK 解算
  std::vector<int> target_servo_angles = ik_->solve(pitch, roll);

  // 舵机角度平滑插值 / Servo angle smoothing
  for (size_t i = 0; i < 3; i++) {
    if (last_sent_angles_[i] == -1) {
      current_servo_angles_[i] = target_servo_angles[i];
    } else {
      double angle_diff = target_servo_angles[i] - current_servo_angles_[i];
      current_servo_angles_[i] += angle_diff * SERVO_SMOOTH_FACTOR;
    }
  }

  // 发送到舵机 / Send to servos
  std::vector<int> servo_angles(3);
  for (size_t i = 0; i < 3; i++) {
    servo_angles[i] = static_cast<int>(std::round(current_servo_angles_[i]));
    if (servo_angles[i] != last_sent_angles_[i]) {
      if (servo_) {
        servo_->setServoAngle(i, servo_angles[i]);
      }
      last_sent_angles_[i] = servo_angles[i];
    }
  }

  // 终端输出（~2Hz）
  loop_count_++;
  if (loop_count_ % 25 == 0) {
    std::string src = found ? "dx=" + std::to_string(dx).substr(0, 5) +
                                  " dy=" + std::to_string(dy).substr(0, 5)
                            : "No target / 无目标";

    printf("[%s] pitch=%+6.1f° roll=%+6.1f° → S1=%3d° S2=%3d° S3=%3d°\n",
           src.c_str(), pitch, roll, servo_angles[0], servo_angles[1],
           servo_angles[2]);
  }
}
