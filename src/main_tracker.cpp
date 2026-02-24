/**
 * @file main_tracker.cpp
 * @brief 3RRS Sun Tracking System - Main Entry Point
 *        3RRS 太阳追踪系统 — 主程序入口
 */

#include "kinematics.hpp"
#include "pca9685.hpp"
#include "tracker_controller.hpp"
#include "vision_system.hpp"
#include <csignal>
#include <iostream>
#include <memory>
#include <signal.h>

int main(int argc, char **argv) {
  std::cout << "=================================================="
            << std::endl;
  std::cout << "  3RRS Sun Tracking System - Raspberry Pi 5 C++" << std::endl;
  std::cout << "  3RRS 太阳追踪系统 — 树莓派5 C++版" << std::endl;
  std::cout << "=================================================="
            << std::endl;

  // 1. 创建组件 / Create components

  // 舵机驱动 / Servo driver
  std::unique_ptr<IServoDriver> servo;
  try {
    servo = std::make_unique<PCA9685>(0x40, 1, 50);
  } catch (const std::exception &e) {
    std::cerr << "[Main] PCA9685 init failed / 初始化失败: " << e.what()
              << std::endl;
    std::cerr << "[Main] Continuing without servos / 继续运行（仅终端输出角度）"
              << std::endl;
    // servo stays nullptr, controller will output angles to terminal only
    // servo 保持 nullptr，控制器仅终端输出角度
  }

  // IK solver / 逆运动学求解器
  auto ik = std::make_unique<RRSKinematics>();

  // Vision system / 视觉系统
  auto vision =
      std::make_unique<VisionSystem>(640, 480, 30); // width, height, fps

  // 2. 创建控制器 / Create controller
  TrackerController controller(std::move(servo), std::move(ik),
                               std::move(vision));

  // 3. 启动系统 / Start system
  controller.start();

  // 4. 等待退出信号 / Wait for exit signal (Ctrl+C)
  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, SIGINT);
  sigaddset(&sigset, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &sigset, nullptr);

  std::cout << "[Main] Waiting for exit signal (Ctrl+C) / 等待退出信号..."
            << std::endl;

  int sig;
  sigwait(&sigset, &sig);

  std::cout << std::endl
            << "[Main] Signal " << sig
            << " received, exiting / 收到信号，正在退出..." << std::endl;

  // 5. 停止并退出 / Stop and exit
  controller.stop();

  return 0;
}
