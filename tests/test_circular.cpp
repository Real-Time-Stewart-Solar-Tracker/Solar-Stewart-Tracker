#include "kinematics.hpp"
#include "pca9685.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

static volatile bool running = true;

void signalHandler(int signum) { running = false; }

int main() {
  std::cout << "============================================================"
            << std::endl;
  std::cout << "  3RRS Circular Trajectory IK Test" << std::endl;
  std::cout << "  3RRS 圆形轨迹逆运动学测试" << std::endl;
  std::cout << "============================================================"
            << std::endl;

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // ========== 1. Initialize PCA9685 / 初始化PCA9685 ==========
  PCA9685 *pca = nullptr;
  try {
    pca = new PCA9685(0x40, 1, 50);
  } catch (const std::exception &e) {
    std::cerr << "[Error] PCA9685 init failed / 初始化失败: " << e.what()
              << std::endl;
    return 1;
  }

  // Center servos (60° initial position) / 舵机归中
  std::cout << std::endl
            << "[Init] Centering servos to 60° / 舵机归中..." << std::endl;
  for (int ch = 0; ch < 3; ch++) {
    pca->setServoAngle(ch, 60);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // ========== 2. Initialize inverse kinematics / 初始化逆运动学 ==========
  RRSKinematics ik;
  std::cout << "[Init] 3RRS IK engine ready / 逆运动学引擎就绪" << std::endl;

  // ========== 3. Circular trajectory parameters / 圆形轨迹参数 ==========
  const double radius = 20.0; // Circle radius (degrees) / 圆的半径（度）
  const double period = 3.0;  // Circle period (seconds) / 圆的周期（秒）
  const double dt = 0.02;     // Time step (seconds) - 50Hz / 时间步长

  std::cout << std::endl << "[Trajectory / 轨迹参数]" << std::endl;
  std::cout << "  Radius / 半径: " << radius << "°" << std::endl;
  std::cout << "  Period / 周期: " << period << "s" << std::endl;
  std::cout << "  Frequency / 频率: " << (int)(1 / dt) << "Hz" << std::endl;

  std::cout << std::endl
            << "[Start] Platform will follow circular trajectory" << std::endl;
  std::cout << "        平台将按圆形轨迹运动" << std::endl;
  std::cout << "        Press Ctrl+C to exit / 按 Ctrl+C 退出" << std::endl
            << std::endl;

  // ========== 4. Main control loop / 主控制循环 ==========
  double t = 0.0;
  int loop_count = 0;

  while (running) {
    auto t_start = std::chrono::steady_clock::now();

    // Compute pitch and roll on circular trajectory
    // 计算圆形轨迹上的 pitch 和 roll
    const double omega = 2 * M_PI / period; // Angular velocity / 角速度
    double pitch = radius * std::sin(omega * t);
    double roll = radius * std::cos(omega * t);

    // Inverse kinematics solve / 逆运动学求解
    auto servo_angles = ik.solve(pitch, roll);

    // Send to servos / 发送到舵机
    for (size_t ch = 0; ch < 3; ch++) {
      pca->setServoAngle(ch, servo_angles[ch]);
    }

    // Terminal output (~10Hz) / 终端输出
    loop_count++;
    if (loop_count % 5 == 0) {
      double circle_angle = std::fmod(omega * t * 180.0 / M_PI, 360.0);
      printf("[Circle:%6.1f°] pitch=%+6.1f° roll=%+6.1f° → S1=%3d° S2=%3d° "
             "S3=%3d°\n",
             circle_angle, pitch, roll, servo_angles[0], servo_angles[1],
             servo_angles[2]);
    }

    // Update time / 更新时间
    t += dt;

    // Control loop frequency / 控制循环频率
    auto t_end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(t_end - t_start).count();
    double sleep_time = dt - elapsed;
    if (sleep_time > 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
  }

  // ========== 5. Cleanup and exit / 清理退出 ==========
  std::cout << std::endl
            << std::endl
            << "[Exit] Centering servos / 舵机回中..." << std::endl;
  for (int ch = 0; ch < 3; ch++) {
    pca->setServoAngle(ch, 60);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  pca->close();
  delete pca;

  std::cout << "[Done] ✅ Safely exited / 已安全退出" << std::endl;

  return 0;
}
