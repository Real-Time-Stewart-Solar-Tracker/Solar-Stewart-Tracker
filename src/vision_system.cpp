#include "vision_system.hpp"
#include <cstdio>
#include <cstring>
#include <iostream>
#include <numeric>
#include <unistd.h>

VisionSystem::VisionSystem(int width, int height, int smooth_window)
    : width_(width), height_(height), smooth_window_(smooth_window),
      running_(false), callback_(nullptr) {}

VisionSystem::~VisionSystem() { cleanup(); }

void VisionSystem::start() {
  running_ = true;
  capture_thread_ = std::thread(&VisionSystem::captureLoop, this);
}

void VisionSystem::stop() {
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void VisionSystem::setCallback(VisionCallback cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callback_ = std::move(cb);
}

void VisionSystem::cleanup() {
  stop();
  cv::destroyAllWindows();
}

void VisionSystem::captureLoop() {
  // 通过 libcamera-vid 管道获取 JPEG 流
  char cmd[512];
  snprintf(cmd, sizeof(cmd),
           "libcamera-vid -t 0 --width %d --height %d --codec mjpeg "
           "--framerate 30 --inline -o - 2>/dev/null",
           width_, height_);

  std::cout << "[Vision] Starting libcamera pipe / 启动libcamera管道..."
            << std::endl;
  FILE *pipe = popen(cmd, "r");
  if (!pipe) {
    std::cerr << "[Vision] Failed to start libcamera-vid / 无法启动libcamera"
              << std::endl;
    return;
  }

  std::cout << "[Vision] Camera started / 摄像头已启动 " << width_ << "x"
            << height_ << " (libcamera)" << std::endl;

  // JPEG frame buffer (max 100KB) / JPEG帧缓冲区（最大100KB）
  const size_t MAX_JPEG_SIZE = 100 * 1024;
  std::vector<uint8_t> jpeg_buffer;
  jpeg_buffer.reserve(MAX_JPEG_SIZE);

  bool in_frame = false;
  uint8_t byte;
  uint8_t prev_byte = 0;

  while (running_) {
    // Read one byte (blocking I/O, driven by pipe data)
    // 读取一个字节（阻塞I/O，由管道数据驱动唤醒）
    if (fread(&byte, 1, 1, pipe) != 1) {
      // 管道数据不可用时短暂让出CPU
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // 检测到 JPEG 帧开始 (FF D8)
    if (prev_byte == 0xFF && byte == 0xD8) {
      in_frame = true;
      jpeg_buffer.clear();
      jpeg_buffer.push_back(0xFF);
      jpeg_buffer.push_back(0xD8);
      // 读取字节流，检测 JPEG 帧边界 (FFD8 / FFD9)结束
    } else if (in_frame && prev_byte == 0xFF && byte == 0xD9) {
      jpeg_buffer.push_back(0xFF);
      jpeg_buffer.push_back(0xD9);

      // 推送检测结果
      cv::Mat frame = cv::imdecode(jpeg_buffer, cv::IMREAD_COLOR);
      if (!frame.empty()) {
        processFrame(frame);
      }

      in_frame = false;
      jpeg_buffer.clear();
    }
    // 在帧内，继续累积
    else if (in_frame) {
      jpeg_buffer.push_back(byte);
      // Prevent buffer overflow
      if (jpeg_buffer.size() > MAX_JPEG_SIZE) {
        in_frame = false;
        jpeg_buffer.clear();
      }
    }

    prev_byte = byte;
  }

  pclose(pipe);
  std::cout << "[Vision] Camera closed" << std::endl;
}

void VisionSystem::processFrame(const cv::Mat &frame) {
  // 转换为 HSV
  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  // 红色掩膜（双区间合并）
  cv::Mat mask1, mask2, mask;
  cv::inRange(hsv, cv::Scalar(RED_LOWER_1_H, RED_LOWER_1_S, RED_LOWER_1_V),
              cv::Scalar(RED_UPPER_1_H, RED_UPPER_1_S, RED_UPPER_1_V), mask1);
  cv::inRange(hsv, cv::Scalar(RED_LOWER_2_H, RED_LOWER_2_S, RED_LOWER_2_V),
              cv::Scalar(RED_UPPER_2_H, RED_UPPER_2_S, RED_UPPER_2_V), mask2);
  cv::bitwise_or(mask1, mask2, mask);

  // 形态学去噪
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  // 查找轮廓，计算质心
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  bool found = false;
  double cx = width_ / 2.0;
  double cy = height_ / 2.0;

  if (!contours.empty()) {
    // HSV 颜色检测与目标定位
    auto biggest = std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });

    double area = cv::contourArea(*biggest);

    if (area > MIN_CONTOUR_AREA) {
      cv::Moments M = cv::moments(*biggest);
      if (M.m00 > 0) {
        cx = M.m10 / M.m00;
        cy = M.m01 / M.m00;
        found = true;
      }
    }
  }

  // 归一化偏差 [-1, 1]
  double raw_dx = (cx - width_ / 2.0) / (width_ / 2.0);
  double raw_dy = (cy - height_ / 2.0) / (height_ / 2.0);

  // 移动平均平滑滤波
  double smooth_dx = raw_dx;
  double smooth_dy = raw_dy;

  if (found) {
    history_x_.push_back(raw_dx);
    history_y_.push_back(raw_dy);

    if (history_x_.size() > static_cast<size_t>(smooth_window_)) {
      history_x_.erase(history_x_.begin());
      history_y_.erase(history_y_.begin());
    }
  }

  if (!history_x_.empty()) {
    smooth_dx = std::accumulate(history_x_.begin(), history_x_.end(), 0.0) /
                history_x_.size();
    smooth_dy = std::accumulate(history_y_.begin(), history_y_.end(), 0.0) /
                history_y_.size();
  }

  // 事件驱动：通过回调推送结果
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_) {
      callback_(found, smooth_dx, smooth_dy);
    }
  }
}
