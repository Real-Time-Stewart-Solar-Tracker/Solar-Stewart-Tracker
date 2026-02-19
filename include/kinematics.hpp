#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "interfaces.hpp"
#include <Eigen/Dense>
#include <vector>

/**
 * @brief 3RRS parallel mechanism inverse kinematics solver
 *        3RRS并联机构逆运动学求解器
 *
 * Algorithm aligned with MATLAB sun211.m.
 * 算法对齐 MATLAB sun211.m。
 */
class RRSKinematics : public IKinematicsSolver {
public:
  /**
   * @brief Constructor, initializes geometric parameters
   *        构造函数，初始化几何参数
   */
  RRSKinematics();

  /**
   * @brief Inverse kinematics solve (IKinematicsSolver interface)
   *        逆运动学求解（IKinematicsSolver接口）
   * @param pitch_deg Pitch angle in degrees / 俯仰角（度）
   * @param roll_deg Roll angle in degrees / 滚转角（度）
   * @return Servo angles for 3 servos (degrees) / 三个舵机角度（度）
   */
  std::vector<int> solve(double pitch_deg, double roll_deg) override;

private:
  // Geometric parameters (aligned with sun211.m line 53)
  // 几何参数（对齐sun211.m第53行）
  double Rb_; // Base platform radius / 静平台半径
  double Rp_; // Moving platform radius / 动平台半径
  double h_;  // Platform height / 平台高度
  double L1_; // Active arm length / 主动臂长
  double L2_; // Passive arm length / 从动臂长

  // Servo mapping parameters (aligned with sun211.m lines 14-16)
  // 舵机映射参数（对齐sun211.m第14~16行）
  std::vector<int> servo_offset_; // Servo neutral position / 舵机中位
  std::vector<int> servo_dir_;    // Direction mapping / 方向映射
  int servo_min_;                 // Min servo angle / 最小舵机角度
  int servo_max_;                 // Max servo angle / 最大舵机角度

  // Base joint angles [0°, 120°, 240°] / 基座铰链角度
  std::vector<double> base_angles_;

  // Base joint points Bi (3x3 matrix) / 基座铰链点
  Eigen::Matrix3d B_;

  // Moving platform local coordinates P_local (3x3 matrix)
  // 动平台局部坐标
  Eigen::Matrix3d P_local_;

  // Platform center initial position / 平台中心初始位置
  Eigen::Vector3d p0_;

  // IK state / 逆运动学状态
  std::vector<double> q2_prev_; // Previous joint angles / 上一次关节角
  std::vector<int> last_valid_; // Last valid servo angles / 上次有效舵机角度

  /**
   * @brief ZYX Euler rotation matrix / ZYX欧拉旋转矩阵
   */
  Eigen::Matrix3d eulZYX(double yaw_deg, double pitch_deg, double roll_deg);

  /**
   * @brief Normalize angle to [-pi, pi] / 归一化角度到[-pi, pi]
   */
  double wrapAngle(double x);

  /**
   * @brief Choose branch closest to previous angle
   *        选择最接近上一次角度的分支
   */
  double chooseClosest(double q1, double q2, double q_prev);

  /**
   * @brief Get the other branch / 获取另一个分支
   */
  double otherBranch(double q1, double q2, double q_cur);
};

#endif // KINEMATICS_HPP
