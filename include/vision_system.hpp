#ifndef VISION_SYSTEM_HPP
#define VISION_SYSTEM_HPP

#include "interfaces.hpp"
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

/**
 * @brief Vision System class / 视觉系统类
 *
 * Implements IVisionSource interface (DIP).
 * Uses OpenCV for HSV color detection and target tracking.
 * 实现 IVisionSource 接口，使用OpenCV进行HSV颜色检测和目标追踪。
 *
 * Event-driven design / 事件驱动设计:
 * - Capture thread runs in background, notifies controller via callback
 *   after processing each frame.
 *   采集线程在后台运行，处理完每帧后通过回调通知控制器
 * - No longer provides polling-style getTarget() interface.
 *   不再提供轮询式 getTarget() 接口
 * - Callback is invoked in capture thread; receiver handles thread sync.
 *   回调在采集线程中调用，由回调接收方负责线程同步
 *
 * SOLID design / SOLID设计:
 * - SRP: Only responsible for visual capture and color detection
 *   单一职责: 仅负责视觉采集和颜色检测
 * - DIP: Pushes results to upper layer via VisionCallback
 *   依赖倒置: 通过 VisionCallback 将结果推送给上层
 */
class VisionSystem : public IVisionSource {
public:
  /**
   * @brief Constructor / 构造函数
   * @param width Image width / 图像宽度
   * @param height Image height / 图像高度
   * @param smooth_window Smoothing window size / 平滑窗口大小
   */
  VisionSystem(int width = 640, int height = 480, int smooth_window = 5);

  ~VisionSystem() override;

  /**
   * @brief Start capture thread (IVisionSource interface)
   *        启动视觉采集线程
   */
  void start() override;

  /**
   * @brief Stop capture thread (IVisionSource interface)
   *        停止视觉采集线程
   */
  void stop() override;

  /**
   * @brief Register vision callback (IVisionSource interface)
   *        注册视觉回调
   * @param cb Detection result callback / 检测结果回调函数
   */
  void setCallback(VisionCallback cb) override;

  /**
   * @brief Clean up resources (IVisionSource interface)
   *        清理资源
   */
  void cleanup() override;

private:
  int width_;
  int height_;
  int smooth_window_;

  std::thread capture_thread_;
  std::mutex callback_mutex_;
  std::atomic<bool> running_;

  // Vision callback (event-driven core) / 视觉回调（事件驱动核心）
  VisionCallback callback_;

  // Smoothing filter history / 平滑滤波历史
  std::vector<double> history_x_;
  std::vector<double> history_y_;

  // HSV red thresholds (lowered S/V for red LED detection)
  // HSV红色阈值（降低S和V阈值以检测红光LED）
  static constexpr int RED_LOWER_1_H = 0, RED_LOWER_1_S = 50,
                       RED_LOWER_1_V = 50;
  static constexpr int RED_UPPER_1_H = 10, RED_UPPER_1_S = 255,
                       RED_UPPER_1_V = 255;
  static constexpr int RED_LOWER_2_H = 160, RED_LOWER_2_S = 50,
                       RED_LOWER_2_V = 50;
  static constexpr int RED_UPPER_2_H = 180, RED_UPPER_2_S = 255,
                       RED_UPPER_2_V = 255;

  // Min contour area (lowered to detect smaller light spots)
  // 最小轮廓面积（降低以检测更小的光点）
  static constexpr int MIN_CONTOUR_AREA = 100;

  /**
   * @brief Capture loop (runs in background thread)
   *        采集循环（在后台线程运行）
   */
  void captureLoop();

  /**
   * @brief Process a single frame, triggers callback when done
   *        处理单帧图像，处理完后触发回调
   * @param frame Input frame / 输入帧
   */
  void processFrame(const cv::Mat &frame);
};

#endif // VISION_SYSTEM_HPP
