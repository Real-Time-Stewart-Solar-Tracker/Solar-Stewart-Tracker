#include "kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

RRSKinematics::RRSKinematics() {
  // Geometric parameters (aligned with sun211.m line 53)
  // 几何参数（对齐sun211.m第53行）
  Rb_ = 0.20; // Base platform radius / 静平台半径
  Rp_ = 0.12; // Moving platform radius / 动平台半径
  h_ = 0.18;  // Platform height / 平台高度
  L1_ = 0.10; // Active arm length / 主动臂长
  L2_ = 0.18; // Passive arm length / 从动臂长

  // Servo mapping parameters (aligned with sun211.m lines 14-16)
  // 舵机映射参数（对齐sun211.m第14~16行）
  servo_offset_ = {90, 90, 90}; // Servo neutral / 舵机中位
  servo_dir_ = {-1, -1, -1};    // Direction mapping / 方向映射
  servo_min_ = 0;
  servo_max_ = 180;

  // Base joint angles [0°, 120°, 240°] / 基座铰链角度
  base_angles_ = {0.0, 120.0, 240.0};
  for (auto &angle : base_angles_) {
    angle = angle * M_PI / 180.0; // Convert to radians / 转换为弧度
  }

  // Base joint points Bi (3x3 matrix) / 基座铰链点
  for (int i = 0; i < 3; i++) {
    B_(i, 0) = Rb_ * std::cos(base_angles_[i]);
    B_(i, 1) = Rb_ * std::sin(base_angles_[i]);
    B_(i, 2) = 0.0;
  }

  // Moving platform local coordinates P_local (3x3 matrix)
  // 动平台局部坐标
  for (int i = 0; i < 3; i++) {
    P_local_(i, 0) = Rp_ * std::cos(base_angles_[i]);
    P_local_(i, 1) = Rp_ * std::sin(base_angles_[i]);
    P_local_(i, 2) = 0.0;
  }

  // Platform center initial position / 平台中心初始位置
  p0_ << 0.0, 0.0, h_;

  // IK state initialization / 逆运动学状态初始化
  q2_prev_ = {0.0, 0.0, 0.0};
  last_valid_ = servo_offset_;
}

Eigen::Matrix3d RRSKinematics::eulZYX(double yaw_deg, double pitch_deg,
                                      double roll_deg) {
  // Convert to radians / 转换为弧度
  double yaw = yaw_deg * M_PI / 180.0;
  double pitch = pitch_deg * M_PI / 180.0;
  double roll = roll_deg * M_PI / 180.0;

  // Compute trig values / 计算三角函数值
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);

  // Build rotation matrix / 构造旋转矩阵
  Eigen::Matrix3d Rz, Ry, Rx;

  Rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;

  Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;

  Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;

  return Rz * Ry * Rx;
}

double RRSKinematics::wrapAngle(double x) {
  return std::fmod(x + M_PI, 2 * M_PI) - M_PI;
}

double RRSKinematics::chooseClosest(double q1, double q2, double q_prev) {
  q1 = wrapAngle(q1);
  q2 = wrapAngle(q2);
  q_prev = wrapAngle(q_prev);

  return (std::abs(q1 - q_prev) <= std::abs(q2 - q_prev)) ? q1 : q2;
}

double RRSKinematics::otherBranch(double q1, double q2, double q_cur) {
  q1 = wrapAngle(q1);
  q2 = wrapAngle(q2);
  q_cur = wrapAngle(q_cur);

  return (std::abs(q_cur - q1) < std::abs(q_cur - q2)) ? q2 : q1;
}

std::vector<int> RRSKinematics::solve(double pitch_deg, double roll_deg) {
  // Rotation matrix (yaw = 0) / 旋转矩阵
  Eigen::Matrix3d R = eulZYX(0, pitch_deg, roll_deg);

  // Moving platform joint positions in world frame
  // 动平台各铰链点在世界坐标系中的位置
  Eigen::Matrix3d P = R * P_local_.transpose();
  P.transposeInPlace();
  for (int i = 0; i < 3; i++) {
    P.row(i) += p0_.transpose();
  }

  // Fallback to last valid angles / 用上一次有效角度兜底
  std::vector<int> result = last_valid_;

  for (int i = 0; i < 3; i++) {
    Eigen::Vector3d Bi = B_.row(i).transpose();
    Eigen::Vector3d Pi = P.row(i).transpose();
    Eigen::Vector3d r_vec = Pi - Bi;

    // Local coordinate system (ex, ey, ez) / 局部坐标系
    double theta = std::atan2(Bi(1), Bi(0));
    Eigen::Vector3d ex(std::cos(theta), std::sin(theta), 0.0);
    Eigen::Vector3d ez(0.0, 0.0, 1.0);
    Eigen::Vector3d ey = ez.cross(ex);

    double x = r_vec.dot(ex);
    double z = r_vec(2);
    double yperp = r_vec.dot(ey);

    // Solve for joint angle q2 / 解算关节角 q2
    double C =
        (x * x + yperp * yperp + z * z + L1_ * L1_ - L2_ * L2_) / (2 * L1_);
    double Rxz = std::hypot(x, z);

    if (Rxz < 1e-6) {
      continue; // Use fallback angle / 使用兜底角度
    }

    double ratio = std::max(-1.0, std::min(1.0, C / Rxz));
    double a = std::acos(ratio);
    double gamma = std::atan2(z, x);

    // Choose nearest branch / 选择最近分支
    double q2c = chooseClosest(gamma + a, gamma - a, q2_prev_[i]);

    // Configuration check: dot < 0 (aligned with sun211.m lines 190-193)
    // 构型检查（对齐sun211.m第190~193行）
    Eigen::Vector3d Ki_joint =
        Bi + L1_ * (std::cos(q2c) * ex + std::sin(q2c) * ez);
    if ((Ki_joint - Bi).dot(ex) < 0) {
      q2c = otherBranch(gamma + a, gamma - a, q2c);
    }

    q2_prev_[i] = q2c;

    // Mechanism angle -> servo angle (aligned with sun211.m lines 197-198)
    // 机构角 → 舵机角（对齐sun211.m第197~198行）
    double servo_deg = servo_offset_[i] + servo_dir_[i] * q2c * 180.0 / M_PI;
    servo_deg = std::max(
        static_cast<double>(servo_min_),
        std::min(static_cast<double>(servo_max_), std::round(servo_deg)));
    result[i] = static_cast<int>(servo_deg);
  }

  last_valid_ = result;
  return result;
}
