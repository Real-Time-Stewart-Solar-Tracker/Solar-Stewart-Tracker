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
  std::cout << "  3RRS 圆形轨迹逆运动学测试" << std::endl;
  std::cout << "============================================================"
            << std::endl;

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // ========== 1. 初始化PCA9685 ==========
  PCA9685 *pca = nullptr;
  try {
    pca = new PCA9685(0x40, 1, 50);
  } catch (const std::exception &e) {
    std::cerr << "[错误] PCA9685 初始化失败: " << e.what() << std::endl;
    return 1;
  }

  // 舵机归中（60°初始位置）
  std::cout << std::endl << "[初始化] 舵机归中到 60°..." << std::endl;
  for (int ch = 0; ch < 3; ch++) {
    pca->setServoAngle(ch, 60);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // ========== 2. 初始化逆运动学 ==========
  RRSKinematics ik;
  std::cout << "[初始化] 3RRS逆运动学引擎就绪" << std::endl;

  // ========== 3. 圆形轨迹参数 ==========
  const double radius = 20.0; // 圆的半径（度）
  const double period = 3.0;  // 圆的周期（秒）
  const double dt = 0.02;     // 时间步长（秒）- 50Hz

  std::cout << std::endl << "[轨迹参数]" << std::endl;
  std::cout << "  圆形半径: " << radius << "°" << std::endl;
  std::cout << "  旋转周期: " << period << "秒" << std::endl;
  std::cout << "  控制频率: " << (int)(1 / dt) << "Hz" << std::endl;

  std::cout << std::endl << "[开始] 平台将按圆形轨迹运动" << std::endl;
  std::cout << "       按 Ctrl+C 退出" << std::endl << std::endl;

  // ========== 4. 主控制循环 ==========
  double t = 0.0;
  int loop_count = 0;

  while (running) {
    auto t_start = std::chrono::steady_clock::now();

    // 计算圆形轨迹上的 pitch 和 roll
    const double omega = 2 * M_PI / period; // 角速度
    double pitch = radius * std::sin(omega * t);
    double roll = radius * std::cos(omega * t);

    // 逆运动学求解
    auto servo_angles = ik.solve(pitch, roll);

    // 发送到舵机
    for (size_t ch = 0; ch < 3; ch++) {
      pca->setServoAngle(ch, servo_angles[ch]);
    }

    // 终端输出（约10Hz）
    loop_count++;
    if (loop_count % 5 == 0) {
      double circle_angle = std::fmod(omega * t * 180.0 / M_PI, 360.0);
      printf("[圆周:%6.1f°] pitch=%+6.1f° roll=%+6.1f° → S1=%3d° S2=%3d° "
             "S3=%3d°\n",
             circle_angle, pitch, roll, servo_angles[0], servo_angles[1],
             servo_angles[2]);
    }

    // 更新时间
    t += dt;

    // 控制循环频率
    auto t_end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(t_end - t_start).count();
    double sleep_time = dt - elapsed;
    if (sleep_time > 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
  }

  // ========== 5. 清理退出 ==========
  std::cout << std::endl << std::endl << "[退出] 舵机回中..." << std::endl;
  for (int ch = 0; ch < 3; ch++) {
    pca->setServoAngle(ch, 60);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  pca->close();
  delete pca;

  std::cout << "[完成] ✅ 已安全退出" << std::endl;

  return 0;
}
