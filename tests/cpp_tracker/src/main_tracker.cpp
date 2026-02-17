#include "kinematics.hpp"
#include "pca9685.hpp"
#include "pid_controller.hpp"
#include "vision_system.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>


// 全局变量用于信号处理
static volatile bool running = true;

void signalHandler(int signum) { running = false; }

int main(int argc, char **argv) {
  std::cout << "=================================================="
            << std::endl;
  std::cout << "  3RRS 太阳追踪系统 — 树莓派5 C++版" << std::endl;
  std::cout << "=================================================="
            << std::endl;

  // 注册信号处理
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // ========== 1. 初始化PCA9685 ==========
  PCA9685 *pca = nullptr;
  try {
    pca = new PCA9685(0x40, 1, 50);
    // 舵机归中（60°初始位置）
    for (int ch = 0; ch < 3; ch++) {
      pca->setServoAngle(ch, 60);
    }
    std::cout << "[Main] 舵机已归中 (60°)" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  } catch (const std::exception &e) {
    std::cerr << "[Main] ⚠ PCA9685 初始化失败: " << e.what() << std::endl;
    std::cerr << "[Main]   继续运行（仅终端输出角度）" << std::endl;
    pca = nullptr;
  }

  // ========== 2. 初始化IK ==========
  RRSKinematics ik;
  std::cout << "[Main] IK 引擎就绪" << std::endl;

  // ========== 3. 初始化PID ==========
  // 参数对齐Python版本: Kp=0.3, maxTilt=35°
  PIDController pid_pitch(0.3, 0.0, 0.0, 35.0);
  PIDController pid_roll(0.3, 0.0, 0.0, 35.0);
  std::cout << "[Main] PID 控制器就绪 (Kp=0.3, maxTilt=35°)" << std::endl;

  // ==========  4. 初始化视觉系统 ==========
  VisionSystem vision(640, 480, 5);
  vision.start();
  std::cout << "[Main] 视觉系统已启动" << std::endl;

  // ========== 5. 主控制循环 ==========
  const double dt = 0.02; // 50Hz
  std::vector<int> last_sent_angles = {-1, -1, -1};
  std::vector<double> current_servo_angles = {60.0, 60.0, 60.0};
  const double servo_smooth_factor = 0.15; // 平滑系数

  std::cout << "[Main] 🚀 开始追踪！(Ctrl+C 退出)" << std::endl << std::endl;

  int loop_count = 0;

  while (running) {
    auto t_start = std::chrono::steady_clock::now();

    // --- 获取目标偏差 ---
    auto [found, dx, dy] = vision.getTarget();

    double target_pitch, target_roll;
    if (found) {
      // 视觉偏差 → PID 目标
      // 增大映射系数以提高响应幅度
      target_pitch = dx * 50.0;
      target_roll = dy * 50.0;
    } else {
      target_pitch = 0.0;
      target_roll = 0.0;
    }

    // PID 更新
    double pitch = pid_pitch.update(target_pitch - pid_pitch.getOutput(), dt);
    double roll = pid_roll.update(target_roll - pid_roll.getOutput(), dt);

    // --- IK 解算 ---
    std::vector<int> target_servo_angles = ik.solve(pitch, roll);

    // --- 舵机角度平滑插值 ---
    for (size_t i = 0; i < 3; i++) {
      if (last_sent_angles[i] == -1) {
        // 第一次直接使用目标角度
        current_servo_angles[i] = target_servo_angles[i];
      } else {
        // 线性插值：逐步接近目标角度
        double angle_diff = target_servo_angles[i] - current_servo_angles[i];
        current_servo_angles[i] += angle_diff * servo_smooth_factor;
      }
    }

    // --- 发送到舵机（四舍五入到整数）---
    std::vector<int> servo_angles(3);
    bool changed = false;
    for (size_t i = 0; i < 3; i++) {
      servo_angles[i] = static_cast<int>(std::round(current_servo_angles[i]));
      if (servo_angles[i] != last_sent_angles[i]) {
        changed = true;
        if (pca) {
          pca->setServoAngle(i, servo_angles[i]);
        }
        last_sent_angles[i] = servo_angles[i];
      }
    }

    // --- 终端状态输出（约2Hz）---
    loop_count++;
    if (loop_count % 25 == 0) {
      std::string src = found ? "dx=" + std::to_string(dx).substr(0, 5) +
                                    " dy=" + std::to_string(dy).substr(0, 5)
                              : "无目标";

      printf("[%s] pitch=%+6.1f° roll=%+6.1f° → S1=%3d° S2=%3d° S3=%3d°\n",
             src.c_str(), pitch, roll, servo_angles[0], servo_angles[1],
             servo_angles[2]);
    }

    // --- 控制循环频率 50Hz ---
    auto t_end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(t_end - t_start).count();
    double sleep_time = dt - elapsed;
    if (sleep_time > 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
  }

  // ========== 6. 清理退出 ==========
  std::cout << std::endl << "[Main] 正在退出..." << std::endl;
  vision.cleanup();

  if (pca) {
    // 舵机回中后关闭
    for (int ch = 0; ch < 3; ch++) {
      pca->setServoAngle(ch, 60);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    pca->close();
    delete pca;
  }

  std::cout << "[Main] ✅ 已安全退出" << std::endl;

  return 0;
}
