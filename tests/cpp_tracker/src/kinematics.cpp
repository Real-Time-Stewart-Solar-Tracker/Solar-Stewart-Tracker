#include "kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>


RRSKinematics::RRSKinematics() {
  // 几何参数（对齐sun211.m第53行）
  Rb = 0.20; // 静平台半径
  Rp = 0.12; // 动平台半径
  h = 0.18;  // 平台高度
  L1 = 0.10; // 主动臂长
  L2 = 0.18; // 从动臂长

  // 舵机映射参数（对齐sun211.m第14~16行）
  servo_offset = {90, 90, 90}; // 舵机中位
  servo_dir = {-1, -1, -1};    // 方向映射
  servo_min = 0;
  servo_max = 180;

  // 基座铰链角度[0°, 120°, 240°]
  base_angles = {0.0, 120.0, 240.0};
  for (auto &angle : base_angles) {
    angle = angle * M_PI / 180.0; // 转换为弧度
  }

  // 基座铰链点 Bi (3x3矩阵)
  for (int i = 0; i < 3; i++) {
    B(i, 0) = Rb * std::cos(base_angles[i]);
    B(i, 1) = Rb * std::sin(base_angles[i]);
    B(i, 2) = 0.0;
  }

  // 动平台局部坐标 P_local (3x3矩阵)
  for (int i = 0; i < 3; i++) {
    P_local(i, 0) = Rp * std::cos(base_angles[i]);
    P_local(i, 1) = Rp * std::sin(base_angles[i]);
    P_local(i, 2) = 0.0;
  }

  // 平台中心初始位置
  p0 << 0.0, 0.0, h;

  // IK状态初始化
  q2_prev = {0.0, 0.0, 0.0};
  last_valid = servo_offset;
}

Eigen::Matrix3d RRSKinematics::eulZYX(double yaw_deg, double pitch_deg,
                                      double roll_deg) {
  // 转换为弧度
  double yaw = yaw_deg * M_PI / 180.0;
  double pitch = pitch_deg * M_PI / 180.0;
  double roll = roll_deg * M_PI / 180.0;

  // 计算三角函数值
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);

  // 构造旋转矩阵
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
  // 旋转矩阵（yaw = 0）
  Eigen::Matrix3d R = eulZYX(0, pitch_deg, roll_deg);

  // 动平台各铰链点在世界坐标系中的位置
  Eigen::Matrix3d P = R * P_local.transpose();
  P.transposeInPlace();
  for (int i = 0; i < 3; i++) {
    P.row(i) += p0.transpose();
  }

  // 用上一次有效角度兜底
  std::vector<int> result = last_valid;

  for (int i = 0; i < 3; i++) {
    Eigen::Vector3d Bi = B.row(i).transpose();
    Eigen::Vector3d Pi = P.row(i).transpose();
    Eigen::Vector3d r_vec = Pi - Bi;

    // 局部坐标系 (ex, ey, ez)
    double theta = std::atan2(Bi(1), Bi(0));
    Eigen::Vector3d ex(std::cos(theta), std::sin(theta), 0.0);
    Eigen::Vector3d ez(0.0, 0.0, 1.0);
    Eigen::Vector3d ey = ez.cross(ex);

    double x = r_vec.dot(ex);
    double z = r_vec(2);
    double yperp = r_vec.dot(ey);

    // 解算关节角 q2
    double C = (x * x + yperp * yperp + z * z + L1 * L1 - L2 * L2) / (2 * L1);
    double Rxz = std::hypot(x, z);

    if (Rxz < 1e-6) {
      continue; // 使用兜底角度
    }

    double ratio = std::max(-1.0, std::min(1.0, C / Rxz));
    double a = std::acos(ratio);
    double gamma = std::atan2(z, x);

    // 选择最近分支
    double q2c = chooseClosest(gamma + a, gamma - a, q2_prev[i]);

    // dot < 0 构型检查（对齐sun211.m第190~193行）
    Eigen::Vector3d Ki_joint =
        Bi + L1 * (std::cos(q2c) * ex + std::sin(q2c) * ez);
    if ((Ki_joint - Bi).dot(ex) < 0) {
      q2c = otherBranch(gamma + a, gamma - a, q2c);
    }

    q2_prev[i] = q2c;

    // 机构角 → 舵机角（对齐sun211.m第197~198行）
    double servo_deg = servo_offset[i] + servo_dir[i] * q2c * 180.0 / M_PI;
    servo_deg = std::max(
        static_cast<double>(servo_min),
        std::min(static_cast<double>(servo_max), std::round(servo_deg)));
    result[i] = static_cast<int>(servo_deg);
  }

  last_valid = result;
  return result;
}
