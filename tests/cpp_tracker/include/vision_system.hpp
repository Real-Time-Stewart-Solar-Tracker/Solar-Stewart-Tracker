#ifndef VISION_SYSTEM_HPP
#define VISION_SYSTEM_HPP

#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

/**
 * @brief 视觉系统类
 *
 * 使用OpenCV进行HSV颜色检测和目标追踪。
 * 对应Python版本的VisionSystem类。
 */
class VisionSystem {
public:
  /**
   * @brief 构造函数
   * @param width 图像宽度
   * @param height 图像高度
   * @param smooth_window 平滑窗口大小
   */
  VisionSystem(int width = 640, int height = 480, int smooth_window = 5);

  /**
   * @brief 析构函数
   */
  ~VisionSystem();

  /**
   * @brief 启动视觉采集线程
   */
  void start();

  /**
   * @brief 停止视觉采集线程
   */
  void stop();

  /**
   * @brief 获取最新目标偏差
   * @return (found, dx, dy) found:是否检测到目标, dx/dy:归一化偏差[-1,1]
   */
  std::tuple<bool, double, double> getTarget();

  /**
   * @brief 清理资源
   */
  void cleanup();

private:
  int width, height;
  int smooth_window;

  std::thread capture_thread;
  std::mutex data_mutex;
  std::atomic<bool> running;

  // 目标数据（线程安全）
  bool target_found;
  double target_dx, target_dy;

  // 平滑滤波历史
  std::vector<double> history_x;
  std::vector<double> history_y;

  // HSV红色阈值（降低S和V阈值以检测红光LED）
  static constexpr int RED_LOWER_1_H = 0, RED_LOWER_1_S = 50,
                       RED_LOWER_1_V = 50;
  static constexpr int RED_UPPER_1_H = 10, RED_UPPER_1_S = 255,
                       RED_UPPER_1_V = 255;
  static constexpr int RED_LOWER_2_H = 160, RED_LOWER_2_S = 50,
                       RED_LOWER_2_V = 50;
  static constexpr int RED_UPPER_2_H = 180, RED_UPPER_2_S = 255,
                       RED_UPPER_2_V = 255;

  static constexpr int MIN_CONTOUR_AREA =
      100; // 最小轮廓面积（降低以检测更小的光点）

  /**
   * @brief 采集循环（在后台线程运行）
   */
  void captureLoop();

  /**
   * @brief 处理单帧图像
   * @param frame 输入帧
   */
  void processFrame(const cv::Mat &frame);
};

#endif // VISION_SYSTEM_HPP
