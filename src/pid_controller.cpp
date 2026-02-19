/**
 * @file pid_controller.cpp
 * @brief PIDController implementation / PID控制器实现
 */
#include "pid_controller.hpp"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd, double max_tilt)
    : kp_(kp), ki_(ki), kd_(kd), max_tilt_(max_tilt), integral_(0.0),
      last_error_(0.0), output_(0.0) {}

double PIDController::update(double error, double dt) {
  // PID 计算（对齐 sun211.m 第146~155行）
  integral_ += error * dt;

  double derivative = (dt > 0) ? (error - last_error_) / dt : 0.0;
  last_error_ = error;

  double out = kp_ * error + ki_ * integral_ + kd_ * derivative;
  output_ += out;

  // Clamp output / 限幅
  output_ = std::max(-max_tilt_, std::min(max_tilt_, output_));

  return output_;
}

void PIDController::reset() {
  integral_ = 0.0;
  last_error_ = 0.0;
  output_ = 0.0;
}
