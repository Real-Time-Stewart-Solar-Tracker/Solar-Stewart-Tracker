/**
 * @file main_tracker.cpp
 * @brief 3RRS Sun Tracking System - Main Entry Point
 *        3RRS 太阳追踪系统 — 主程序入口
 *
 * Real-time architecture overview / 实时架构概览:
 * ┌─────────────┐    Callback       ┌───────────────────┐
 * │ VisionSystem │ ──────────────→  │ TrackerController │
 * │ (capture     │   onVisionUpdate │  (timer thread)   │
 * │  thread)     │                  │                   │
 * └─────────────┘                   │  PID → IK → Servo│
 *                                   └───────────────────┘
 *
 * Main thread uses sigwait to block-wait for exit signals (SIGINT/SIGTERM),
 * implementing a zero-polling event-driven architecture.
 * 主线程使用 sigwait 阻塞等待退出信号，实现零轮询的事件驱动架构。
 *
 * All components managed via smart pointers (RAII), ensuring exception
 * safety and zero memory leaks.
 * 所有组件通过智能指针管理（RAII），确保异常安全和零内存泄漏。
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
  std::cout << "  [Event-driven | SOLID | RAII]" << std::endl;
  std::cout << "=================================================="
            << std::endl;

  // ========== 1. Create components (smart pointers, RAII lifetime) ==========
  // ========== 1. 创建组件（智能指针，RAII自动管理生命周期）==========

  // Servo driver → IServoDriver interface / 舵机驱动 → IServoDriver 接口
  std::unique_ptr<IServoDriver> servo;
  try {
    servo = std::make_unique<PCA9685>(0x40, 1, 50);
  } catch (const std::exception &e) {
    std::cerr << "[Main] PCA9685 init failed / 初始化失败: " << e.what()
              << std::endl;
    std::cerr << "[Main] Continuing without servos / 继续运行（仅终端输出角度）"
              << std::endl;
    // servo stays nullptr, TrackerController will skip servo operations
    // servo 保持 nullptr，TrackerController 将跳过舵机操作
  }

  // IK solver → IKinematicsSolver interface / 逆运动学求解器
  auto ik = std::make_unique<RRSKinematics>();

  // Vision system → IVisionSource interface / 视觉系统
  auto vision = std::make_unique<VisionSystem>(640, 480, 5);

  // ========== 2. Create controller via dependency injection (DIP) ==========
  // ========== 2. 依赖注入创建控制器（DIP原则）==========
  TrackerController controller(std::move(servo), std::move(ik),
                               std::move(vision));

  // ========== 3. Start system / 启动系统 ==========
  controller.start();

  // ========== 4. Main thread waits for exit signal (event-driven, zero
  // polling)
  // ========== 4. 主线程等待退出信号（事件驱动，零轮询）==========
  // Uses sigwait to block, replacing while(running) polling loop
  // 使用 sigwait 阻塞等待，替代 while(running) 轮询循环
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

  // ========== 5. Graceful exit (RAII auto-cleanup) ==========
  // ========== 5. 优雅退出（RAII自动清理）==========
  controller.stop();
  // All smart pointers auto-release resources at scope end
  // 所有智能指针在作用域结束时自动释放资源

  return 0;
}
