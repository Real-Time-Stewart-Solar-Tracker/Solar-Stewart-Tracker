#include "pid_controller.hpp"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd, double max_tilt)
    : kp(kp), ki(ki), kd(kd), max_tilt(max_tilt), integral(0.0),
      last_error(0.0), output(0.0) {}

double PIDController::update(double error, double dt) {
  // 对齐sun211.m第146~155行
  integral += error * dt;

  double derivative = (dt > 0) ? (error - last_error) / dt : 0.0;
  last_error = error;

  double out = kp * error + ki * integral + kd * derivative;
  output += out;

  // 限幅
  output = std::max(-max_tilt, std::min(max_tilt, output));

  return output;
}

void PIDController::reset() {
  integral = 0.0;
  last_error = 0.0;
  output = 0.0;
}
