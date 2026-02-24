#ifndef VISION_SYSTEM_HPP
#define VISION_SYSTEM_HPP

#include "interfaces.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

/**
 * @brief Vision System / 视觉系统
 *
 * Uses rpicam-vid + MJPEG stream for camera capture on Raspberry Pi.
 * 使用 rpicam-vid + MJPEG 流在树莓派上采集摄像头画面。
 *
 * Implements IVisionSource interface; pushes detection results to
 * TrackerController via VisionCallback (event-driven).
 * 实现 IVisionSource 接口，通过 VisionCallback 推送检测结果（事件驱动）。
 */
class VisionSystem : public IVisionSource {
public:
  /**
   * @brief Detection mode / 检测模式
   */
  enum class DetectMode {
    BRIGHT_ONLY,    ///< Track high-brightness region only / 只追高亮区域（通用追光）
    BRIGHT_AND_RED  ///< Track bright+red region / 追"高亮且偏红"区域（适合红色手电筒，抑制白光/反光）
  };

  /**
   * @brief Target detection result / 目标检测结果
   */
  struct TargetInfo {
    bool found = false;

    double cx = 0.0;  ///< Filtered pixel X / 滤波后像素X坐标
    double cy = 0.0;  ///< Filtered pixel Y / 滤波后像素Y坐标

    /// Normalized error [-1, 1]; dx>0 means target is to the right
    /// 归一化误差：dx>0 代表目标在右侧；dy>0 代表目标在下侧（OpenCV y向下）
    double dx = 0.0;
    double dy = 0.0;

    /// dy flipped for control (up = positive) / 控制常用：上为正
    double dy_ctrl = 0.0;

    double area     = 0.0;  ///< Contour area / 轮廓面积
    double score    = 0.0;  ///< area × circularity score / 面积×圆度评分
    int mask_nonzero = 0;   ///< Mask white pixel count / 掩膜白像素数量

    std::chrono::steady_clock::time_point ts = std::chrono::steady_clock::now();
  };

public:
  /**
   * @brief Constructor / 构造函数
   * @param width  Image width / 图像宽度
   * @param height Image height / 图像高度
   * @param fps    Frame rate / 帧率
   */
  VisionSystem(int width = 640, int height = 480, int fps = 30);

  ~VisionSystem() override;

  // -------- IVisionSource interface / IVisionSource 接口 --------

  /**
   * @brief Start capture thread / 启动采集线程
   */
  void start() override;

  /**
   * @brief Stop capture thread / 停止采集线程
   */
  void stop() override;

  /**
   * @brief Register vision callback / 注册视觉回调
   * @param cb Called each frame with (found, dx, dy_ctrl) / 每帧调用，参数为(found, dx, dy_ctrl)
   */
  void setCallback(VisionCallback cb) override;

  /**
   * @brief Clean up resources (calls stop) / 清理资源（调用stop）
   */
  void cleanup() override;

  // -------- Extra interface: get latest target / 额外接口：获取最新目标 --------

  /**
   * @brief Thread-safe copy of latest detection result / 线程安全获取最新检测结果
   */
  TargetInfo getTarget() const;

  // -------- Runtime parameter setters / 运行时参数配置 --------

  void setDetectMode(DetectMode mode);

  /// Brightness threshold (V channel), recommended 220~245 / 追光亮度阈值（推荐220~245）
  void setBrightnessThreshold(int v_th);

  /// Saturation threshold for BRIGHT_AND_RED mode, recommended 40~80
  /// 饱和度阈值（仅 BRIGHT_AND_RED 使用，推荐40~80）
  void setSaturationThreshold(int s_th);

  /// Red hue ranges (default [0..10] & [160..180]) / 红色Hue范围
  void setRedHueRange(int h1_low, int h1_high, int h2_low, int h2_high);

  /// Area filter, prevents large reflections being treated as target
  /// 面积过滤，防止整面墙反光被当作目标
  void setAreaRange(double min_area, double max_area);

  /// EMA smoothing factor (smaller = smoother; larger = more responsive)
  /// EMA平滑系数（越小越稳，越大越灵敏）
  void setSmoothingAlpha(double alpha);

  /// Debug frame/mask save (headless-friendly); interval_frames<=0 to disable
  /// debug帧和掩膜保存（headless友好）；interval_frames<=0 表示不保存
  void setDebugSave(const std::string &frame_path,
                    const std::string &mask_path,
                    int interval_frames);

  /// Enable periodic stats print / 启用周期性统计打印
  void enableStatsPrint(bool enable, int period_seconds = 2);

private:
  void captureLoop();
  cv::Mat readOneMjpegFrame(FILE *pipe);
  void processFrame(const cv::Mat &frame);
  cv::Mat buildMask(const cv::Mat &bgr, int &mask_nonzero);

private:
  int width_;
  int height_;
  int fps_;

  DetectMode mode_ = DetectMode::BRIGHT_AND_RED;

  // Thresholds / 阈值
  int v_th_ = 230;
  int s_th_ = 60;

  // Red hue ranges / 红色Hue范围
  int h1_low_ = 0,   h1_high_ = 10;
  int h2_low_ = 160, h2_high_ = 180;

  // Area filter / 面积过滤
  double min_area_ = 50.0;
  double max_area_ = 50000.0;

  // EMA smoothing / EMA平滑
  double alpha_          = 0.2;
  bool   has_filter_state_ = false;
  double fx_ = 0.0, fy_ = 0.0;

  // Debug save / 调试保存
  std::string debug_frame_path_ = "/home/pi/debug_frame.jpg";
  std::string debug_mask_path_  = "/home/pi/debug_mask.jpg";
  int debug_interval_frames_    = 30;
  int frame_counter_            = 0;

  // Stats print / 统计打印
  bool stats_enable_    = true;
  int  stats_period_sec_ = 2;

  // Threading / 线程
  std::atomic<bool> running_{false};
  std::thread       worker_;

  mutable std::mutex mtx_;
  TargetInfo         latest_;

  // Vision callback (event-driven core) / 视觉回调（事件驱动核心）
  std::mutex    callback_mutex_;
  VisionCallback callback_;
};

#endif // VISION_SYSTEM_HPP
