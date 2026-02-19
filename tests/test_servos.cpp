#include "pca9685.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

static volatile bool running = true;

void signalHandler(int signum) { running = false; }

int main() {
  std::cout << "=================================================="
            << std::endl;
  std::cout << "  Simple Servo Test - 0~60 degree sweep" << std::endl;
  std::cout << "  简单舵机测试 - 0~60度循环运动" << std::endl;
  std::cout << "=================================================="
            << std::endl;

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // Initialize PCA9685 / 初始化PCA9685
  PCA9685 *pca = nullptr;
  try {
    pca = new PCA9685(0x40, 1, 50);
  } catch (const std::exception &e) {
    std::cerr << "[Error] PCA9685 init failed / 初始化失败: " << e.what()
              << std::endl;
    std::cerr << "Check / 请检查:" << std::endl;
    std::cerr << "  1. I2C enabled / I2C 是否已启用" << std::endl;
    std::cerr << "  2. PCA9685 connection / 连接是否正确" << std::endl;
    return 1;
  }

  // Center servos (60° initial position) / 舵机归中
  std::cout << std::endl
            << "[Init] Centering servos to 60° / 舵机归中..." << std::endl;
  for (int ch = 0; ch < 3; ch++) {
    pca->setServoAngle(ch, 60);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << std::endl
            << "[Start] Three servos will sweep 0~60 degrees slowly"
            << std::endl;
  std::cout << "        三个舵机将在0~60度之间慢慢循环运动" << std::endl;
  std::cout << "        Press Ctrl+C to exit / 按 Ctrl+C 退出" << std::endl
            << std::endl;

  // Parameter setup / 参数设置
  const int min_angle = 0;
  const int max_angle = 60;
  const int step = 1;
  const int delay_ms = 50; // Delay per step (ms) / 每步延迟时间（毫秒）

  int current_angle = min_angle;
  int direction = 1; // 1 = increasing, -1 = decreasing

  while (running) {
    // Set all three servos to current angle
    // 设置所有三个舵机到当前角度
    for (int ch = 0; ch < 3; ch++) {
      pca->setServoAngle(ch, current_angle);
    }

    printf("\rCurrent angle / 当前角度: %3d°  ", current_angle);
    fflush(stdout);

    // Update angle / 更新角度
    current_angle += step * direction;

    // Check bounds and reverse direction / 检查边界并反转方向
    if (current_angle >= max_angle) {
      current_angle = max_angle;
      direction = -1;
    } else if (current_angle <= min_angle) {
      current_angle = min_angle;
      direction = 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  }

  // Cleanup and exit / 清理退出
  std::cout << std::endl
            << std::endl
            << "[Exit] Returning servos to 60° / 舵机回到60°..." << std::endl;
  for (int ch = 0; ch < 3; ch++) {
    pca->setServoAngle(ch, 60);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  pca->close();
  delete pca;

  std::cout << "[Done] ✅ Safely exited / 已安全退出" << std::endl;

  return 0;
}
