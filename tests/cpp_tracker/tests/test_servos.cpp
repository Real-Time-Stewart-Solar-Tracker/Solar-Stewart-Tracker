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
  std::cout << "  简单舵机测试程序 - 0~60度循环运动" << std::endl;
  std::cout << "=================================================="
            << std::endl;

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // 初始化PCA9685
  PCA9685 *pca = nullptr;
  try {
    pca = new PCA9685(0x40, 1, 50);
  } catch (const std::exception &e) {
    std::cerr << "[错误] PCA9685 初始化失败: " << e.what() << std::endl;
    std::cerr << "请检查：" << std::endl;
    std::cerr << "  1. I2C 是否已启用" << std::endl;
    std::cerr << "  2. PCA9685 连接是否正确" << std::endl;
    return 1;
  }

  // 舵机归中（90°初始位置）
  std::cout << std::endl << "[初始化] 舵机归中到 90°..." << std::endl;
  for (int ch = 0; ch < 3; ch++) {
    pca->setServoAngle(ch, 90);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << std::endl
            << "[开始] 三个舵机将在0~60度之间慢慢循环运动" << std::endl;
  std::cout << "       按 Ctrl+C 退出" << std::endl << std::endl;

  // 参数设置
  const int min_angle = 0;
  const int max_angle = 60;
  const int step = 1;
  const int delay_ms = 50; // 每步延迟时间（毫秒）

  int current_angle = min_angle;
  int direction = 1; // 1表示增加，-1表示减少

  while (running) {
    // 设置所有三个舵机到当前角度
    for (int ch = 0; ch < 3; ch++) {
      pca->setServoAngle(ch, current_angle);
    }

    printf("\r当前角度: %3d°  ", current_angle);
    fflush(stdout);

    // 更新角度
    current_angle += step * direction;

    // 检查边界并反转方向
    if (current_angle >= max_angle) {
      current_angle = max_angle;
      direction = -1;
    } else if (current_angle <= min_angle) {
      current_angle = min_angle;
      direction = 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  }

  // 清理退出
  std::cout << std::endl << std::endl << "[退出] 舵机回中..." << std::endl;
  for (int ch = 0; ch < 3; ch++) {
    pca->setServoAngle(ch, 90);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  pca->close();
  delete pca;

  std::cout << "[完成] ✅ 已安全退出" << std::endl;

  return 0;
}
