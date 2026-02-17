#include "vision_system.hpp"
#include <cstdio>
#include <cstring>
#include <iostream>
#include <numeric>
#include <unistd.h>


VisionSystem::VisionSystem(int width, int height, int smooth_window)
    : width(width), height(height), smooth_window(smooth_window),
      running(false), target_found(false), target_dx(0.0), target_dy(0.0) {}

VisionSystem::~VisionSystem() { cleanup(); }

void VisionSystem::start() {
  running = true;
  capture_thread = std::thread(&VisionSystem::captureLoop, this);
}

void VisionSystem::stop() {
  running = false;
  if (capture_thread.joinable()) {
    capture_thread.join();
  }
}

std::tuple<bool, double, double> VisionSystem::getTarget() {
  std::lock_guard<std::mutex> lock(data_mutex);
  return std::make_tuple(target_found, target_dx, target_dy);
}

void VisionSystem::cleanup() {
  stop();
  cv::destroyAllWindows();
}

void VisionSystem::captureLoop() {
  // 使用libcamera-vid通过管道获取JPEG流
  // 这是纯C++方案，适用于树莓派5 CSI摄像头
  char cmd[512];
  snprintf(cmd, sizeof(cmd),
           "libcamera-vid -t 0 --width %d --height %d --codec mjpeg "
           "--framerate 30 --inline -o - 2>/dev/null",
           width, height);

  std::cout << "[Vision] 启动libcamera管道..." << std::endl;
  FILE *pipe = popen(cmd, "r");
  if (!pipe) {
    std::cerr << "[Vision] ⚠ 无法启动libcamera-vid" << std::endl;
    return;
  }

  std::cout << "[Vision] ✓ 摄像头已启动 " << width << "x" << height
            << " (libcamera)" << std::endl;

  // JPEG帧缓冲区（最大100KB）
  const size_t MAX_JPEG_SIZE = 100 * 1024;
  std::vector<uint8_t> jpeg_buffer;
  jpeg_buffer.reserve(MAX_JPEG_SIZE);

  // JPEG标记
  const uint8_t JPEG_SOI[2] = {0xFF, 0xD8}; // Start of Image
  const uint8_t JPEG_EOI[2] = {0xFF, 0xD9}; // End of Image

  bool in_frame = false;
  uint8_t byte;
  uint8_t prev_byte = 0;

  while (running) {
    // 读取一个字节
    if (fread(&byte, 1, 1, pipe) != 1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // 检测JPEG帧起始 (FF D8)
    if (prev_byte == 0xFF && byte == 0xD8) {
      in_frame = true;
      jpeg_buffer.clear();
      jpeg_buffer.push_back(0xFF);
      jpeg_buffer.push_back(0xD8);
    }
    // 检测JPEG帧结束 (FF D9)
    else if (in_frame && prev_byte == 0xFF && byte == 0xD9) {
      jpeg_buffer.push_back(0xFF);
      jpeg_buffer.push_back(0xD9);

      // 解码JPEG并处理
      cv::Mat frame = cv::imdecode(jpeg_buffer, cv::IMREAD_COLOR);
      if (!frame.empty()) {
        processFrame(frame);
      }

      in_frame = false;
      jpeg_buffer.clear();
    }
    // 在帧内，继续accumulate
    else if (in_frame) {
      jpeg_buffer.push_back(byte);
      // 防止缓冲区溢出
      if (jpeg_buffer.size() > MAX_JPEG_SIZE) {
        in_frame = false;
        jpeg_buffer.clear();
      }
    }

    prev_byte = byte;
  }

  pclose(pipe);
  std::cout << "[Vision] 摄像头已关闭" << std::endl;
}

void VisionSystem::processFrame(const cv::Mat &frame) {
  // 转换到HSV
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

  // 轮廓查找
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  bool found = false;
  double cx = width / 2.0;
  double cy = height / 2.0;

  if (!contours.empty()) {
    // 选择最大轮廓
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

  // 归一化偏差：以画面中心为原点，[-1, 1]
  double raw_dx = (cx - width / 2.0) / (width / 2.0);
  double raw_dy = (cy - height / 2.0) / (height / 2.0);

  // Moving Average 平滑滤波
  double smooth_dx = raw_dx;
  double smooth_dy = raw_dy;

  if (found) {
    history_x.push_back(raw_dx);
    history_y.push_back(raw_dy);

    if (history_x.size() > static_cast<size_t>(smooth_window)) {
      history_x.erase(history_x.begin());
      history_y.erase(history_y.begin());
    }
  }

  if (!history_x.empty()) {
    smooth_dx = std::accumulate(history_x.begin(), history_x.end(), 0.0) /
                history_x.size();
    smooth_dy = std::accumulate(history_y.begin(), history_y.end(), 0.0) /
                history_y.size();
  }

  // 更新共享状态
  {
    std::lock_guard<std::mutex> lock(data_mutex);
    target_found = found;
    target_dx = smooth_dx;
    target_dy = smooth_dy;
  }
}
