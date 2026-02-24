/**
 * @file vision_system.cpp
 * @brief VisionSystem implementation / 视觉系统实现
 *
 * Camera capture via rpicam-vid + MJPEG pipe.
 * 通过 rpicam-vid 以 MJPEG 流采集摄像头画面。
 *
 * Implements IVisionSource; fires VisionCallback each frame (event-driven).
 * 实现 IVisionSource 接口；每帧触发 VisionCallback（事件驱动）。
 */
#include "vision_system.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

// ============================================================
// Constructor / Destructor
// ============================================================

VisionSystem::VisionSystem(int width, int height, int fps)
    : width_(width), height_(height), fps_(fps) {}

VisionSystem::~VisionSystem() { stop(); }

// ============================================================
// IVisionSource interface implementation
// ============================================================

void VisionSystem::start() {
  if (running_)
    return;
  running_ = true;
  worker_ = std::thread(&VisionSystem::captureLoop, this);
}

void VisionSystem::stop() {
  if (!running_)
    return;
  running_ = false;
  if (worker_.joinable())
    worker_.join();
}

void VisionSystem::setCallback(VisionCallback cb) {
  std::lock_guard<std::mutex> lk(callback_mutex_);
  callback_ = std::move(cb);
}

void VisionSystem::cleanup() { stop(); }

// ============================================================
// Extra: get latest target (thread-safe)
// ============================================================

VisionSystem::TargetInfo VisionSystem::getTarget() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return latest_;
}

// ============================================================
// Runtime parameter setters / 运行时参数配置
// ============================================================

void VisionSystem::setDetectMode(DetectMode mode) {
  std::lock_guard<std::mutex> lk(mtx_);
  mode_ = mode;
}

void VisionSystem::setBrightnessThreshold(int v_th) {
  std::lock_guard<std::mutex> lk(mtx_);
  v_th_ = std::max(0, std::min(255, v_th));
}

void VisionSystem::setSaturationThreshold(int s_th) {
  std::lock_guard<std::mutex> lk(mtx_);
  s_th_ = std::max(0, std::min(255, s_th));
}

void VisionSystem::setRedHueRange(int h1_low, int h1_high, int h2_low,
                                  int h2_high) {
  std::lock_guard<std::mutex> lk(mtx_);
  h1_low_ = h1_low;
  h1_high_ = h1_high;
  h2_low_ = h2_low;
  h2_high_ = h2_high;
}

void VisionSystem::setAreaRange(double min_area, double max_area) {
  std::lock_guard<std::mutex> lk(mtx_);
  min_area_ = std::max(0.0, min_area);
  max_area_ = std::max(min_area_, max_area);
}

void VisionSystem::setSmoothingAlpha(double alpha) {
  std::lock_guard<std::mutex> lk(mtx_);
  alpha_ = std::max(0.0, std::min(1.0, alpha));
}

void VisionSystem::setDebugSave(const std::string &frame_path,
                                const std::string &mask_path,
                                int interval_frames) {
  std::lock_guard<std::mutex> lk(mtx_);
  debug_frame_path_ = frame_path;
  debug_mask_path_ = mask_path;
  debug_interval_frames_ = interval_frames;
}

void VisionSystem::enableStatsPrint(bool enable, int period_seconds) {
  std::lock_guard<std::mutex> lk(mtx_);
  stats_enable_ = enable;
  stats_period_sec_ = std::max(1, period_seconds);
}

// ============================================================
// MJPEG frame reader / MJPEG帧读取器
// ============================================================

cv::Mat VisionSystem::readOneMjpegFrame(FILE *pipe) {
  // 1MB buffer avoids single-frame truncation / 1MB缓冲避免单帧截断
  const size_t MAX_JPEG_SIZE = 1024 * 1024;

  std::vector<uint8_t> buf;
  buf.reserve(MAX_JPEG_SIZE);

  bool in_frame = false;
  uint8_t byte = 0, prev = 0;

  while (running_) {
    if (fread(&byte, 1, 1, pipe) != 1) {
      return cv::Mat();
    }

    if (!in_frame) {
      // Detect JPEG SOI (0xFF 0xD8) / 检测JPEG帧头
      if (prev == 0xFF && byte == 0xD8) {
        in_frame = true;
        buf.clear();
        buf.push_back(0xFF);
        buf.push_back(0xD8);
      }
    } else {
      buf.push_back(byte);

      if (buf.size() > MAX_JPEG_SIZE) {
        // Frame too large — drop it / 帧过大，丢弃
        in_frame = false;
        buf.clear();
      } else if (prev == 0xFF && byte == 0xD9) {
        // JPEG EOI — decode / 检测JPEG帧尾，解码
        return cv::imdecode(buf, cv::IMREAD_COLOR);
      }
    }
    prev = byte;
  }

  return cv::Mat();
}

// ============================================================
// Mask builder / 检测掩膜构建
// ============================================================

cv::Mat VisionSystem::buildMask(const cv::Mat &bgr, int &mask_nonzero) {
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  std::vector<cv::Mat> ch;
  cv::split(hsv, ch);
  const cv::Mat &H = ch[0];
  const cv::Mat &S = ch[1];
  const cv::Mat &V = ch[2];

  // Snapshot parameters (avoid holding lock too long)
  // 参数快照（避免长时间持锁）
  DetectMode mode;
  int v_th, s_th;
  int h1l, h1h, h2l, h2h;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mode = mode_;
    v_th = v_th_;
    s_th = s_th_;
    h1l = h1_low_;
    h1h = h1_high_;
    h2l = h2_low_;
    h2h = h2_high_;
  }

  // Brightness mask / 亮度掩膜
  cv::Mat brightMask;
  cv::threshold(V, brightMask, v_th, 255, cv::THRESH_BINARY);

  cv::Mat mask;

  if (mode == DetectMode::BRIGHT_ONLY) {
    mask = brightMask;
  } else {
    // BRIGHT_AND_RED: brightness + red hue + saturation
    // 亮度 + 红色偏向 + 饱和度
    cv::Mat red1, red2, redHue, satMask;
    cv::inRange(H, h1l, h1h, red1);
    cv::inRange(H, h2l, h2h, red2);
    redHue = red1 | red2;

    cv::threshold(S, satMask, s_th, 255, cv::THRESH_BINARY);

    mask = brightMask & redHue & satMask;
  }

  // Morphological cleanup (small kernel suits light spot edges)
  // 形态学清理（小核更适合光斑边界）
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  mask_nonzero = cv::countNonZero(mask);
  return mask;
}

// ============================================================
// Frame processor / 单帧处理器
// ============================================================

void VisionSystem::processFrame(const cv::Mat &frame) {
  int nonzero = 0;
  cv::Mat mask = buildMask(frame, nonzero);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Parameter snapshot / 参数快照
  double min_area, max_area, alpha;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    min_area = min_area_;
    max_area = max_area_;
    alpha = alpha_;
  }

  bool found = false;
  double best_area = 0.0;
  double best_score = 0.0;
  cv::Point2f best_center(0, 0);

  // Strategy: score = area × circularity, favours round light spots
  // 追光策略：评分=面积×圆度，光斑通常较圆，抑制长条反光
  for (const auto &c : contours) {
    double area = cv::contourArea(c);
    if (area < min_area || area > max_area)
      continue;

    double peri = cv::arcLength(c, true);
    if (peri < 1e-6)
      continue;

    double circ = 4.0 * CV_PI * area / (peri * peri); // [0, 1]
    if (circ < 0.20)
      continue; // Too elongated, likely a reflection / 过长则可能是反光

    cv::Moments mu = cv::moments(c);
    if (mu.m00 < 1e-6)
      continue;

    cv::Point2f center((float)(mu.m10 / mu.m00), (float)(mu.m01 / mu.m00));

    double score = area * circ;
    if (score > best_score) {
      best_score = score;
      best_area = area;
      best_center = center;
      found = true;
    }
  }

  TargetInfo out;
  out.found = found;
  out.area = best_area;
  out.score = best_score;
  out.mask_nonzero = nonzero;
  out.ts = std::chrono::steady_clock::now();

  if (found) {
    // EMA low-pass filter (prevents servo jitter) / EMA低通滤波（防止舵机抖动）
    if (!has_filter_state_) {
      fx_ = best_center.x;
      fy_ = best_center.y;
      has_filter_state_ = true;
    } else {
      fx_ = alpha * best_center.x + (1.0 - alpha) * fx_;
      fy_ = alpha * best_center.y + (1.0 - alpha) * fy_;
    }

    out.cx = fx_;
    out.cy = fy_;

    const double w = static_cast<double>(frame.cols);
    const double h = static_cast<double>(frame.rows);

    out.dx = (out.cx - w / 2.0) / (w / 2.0);
    out.dy = (out.cy - h / 2.0) / (h / 2.0);

    // Flip dy for control (up = positive) / dy翻转为控制坐标系（上为正）
    out.dy_ctrl = -out.dy;

    // Dead zone / 死区
    if (std::abs(out.dx) < 0.03)
      out.dx = 0.0;
    if (std::abs(out.dy) < 0.03)
      out.dy = 0.0;
    if (std::abs(out.dy_ctrl) < 0.03)
      out.dy_ctrl = 0.0;
  }

  // Debug frame/mask save / 调试帧和掩膜保存
  frame_counter_++;
  int interval;
  std::string frame_path, mask_path;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    interval = debug_interval_frames_;
    frame_path = debug_frame_path_;
    mask_path = debug_mask_path_;
  }
  if (interval > 0 && (frame_counter_ % interval == 0)) {
    cv::imwrite(frame_path, frame);
    cv::imwrite(mask_path, mask);
    std::cout << "[Vision] [SAVE] " << frame_path << " | " << mask_path
              << std::endl;
  }

  // Update shared latest target / 更新共享的最新目标
  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_ = out;
  }

  // Fire event-driven callback → TrackerController::onVisionUpdate
  // 触发事件驱动回调 → TrackerController::onVisionUpdate
  VisionCallback cb;
  {
    std::lock_guard<std::mutex> lk(callback_mutex_);
    cb = callback_;
  }
  if (cb) {
    // Pass dy_ctrl (up=positive) as the dy parameter to the controller
    // 将dy_ctrl（上为正）作为dy参数传给控制器
    cb(out.found, out.dx, out.dy_ctrl);
  }
}

// ============================================================
// Capture loop (background thread) / 采集循环（后台线程）
// ============================================================

void VisionSystem::captureLoop() {
  // Build rpicam-vid command (MJPEG → stdout) /
  // 构造rpicam-vid命令（MJPEG输出到stdout）
  std::string cmd = "rpicam-vid -t 0 "
                    "--width " +
                    std::to_string(width_) +
                    " "
                    "--height " +
                    std::to_string(height_) +
                    " "
                    "--framerate " +
                    std::to_string(fps_) +
                    " "
                    "--codec mjpeg --inline --nopreview -o -";

  std::cout << "[Vision] Starting: " << cmd << std::endl;

  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    std::cerr << "[Vision] ERROR: popen() failed (rpicam-vid not found?)"
              << std::endl;
    running_ = false;
    return;
  }

  uint64_t decoded_ok = 0;
  uint64_t decoded_fail = 0;
  uint64_t found_cnt = 0;

  auto last_report = std::chrono::steady_clock::now();

  while (running_) {
    cv::Mat frame = readOneMjpegFrame(pipe);
    if (frame.empty()) {
      decoded_fail++;
      // Brief yield to avoid busy-spin / 轻微让步避免忙等
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    decoded_ok++;

    processFrame(frame);

    // Periodic stats print / 周期性统计打印
    bool stats_en;
    int period_sec;
    TargetInfo snap;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      stats_en = stats_enable_;
      period_sec = stats_period_sec_;
      snap = latest_;
    }
    if (snap.found)
      found_cnt++;

    auto now = std::chrono::steady_clock::now();
    if (stats_en && (now - last_report) > std::chrono::seconds(period_sec)) {
      last_report = now;
      std::cout << "[Vision] [STAT]"
                << " decoded_ok=" << decoded_ok
                << " decoded_fail=" << decoded_fail
                << " found_cnt=" << found_cnt
                << " mask_nonzero=" << snap.mask_nonzero << " dx=" << snap.dx
                << " dy_ctrl=" << snap.dy_ctrl << " area=" << snap.area
                << std::endl;
    }
  }

  pclose(pipe);
  std::cout << "[Vision] Stopped" << std::endl;
}
