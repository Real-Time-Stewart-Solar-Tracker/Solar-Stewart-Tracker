#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <Eigen/Dense>
#include <vector>

/**
 * @brief 3RRS并联机构逆运动学类
 *
 * 完全对应Python版本的RRSKinematics类和sun211.m的算法。
 */
class RRSKinematics {
public:
  /**
   * @brief 构造函数，初始化几何参数
   */
  RRSKinematics();

  /**
   * @brief 逆运动学求解
   * @param pitch_deg 俯仰角（度）
   * @param roll_deg 滚转角（度）
   * @return 三个舵机角度（度）
   */
  std::vector<int> solve(double pitch_deg, double roll_deg);

private:
  // 几何参数
  double Rb; // 静平台半径
  double Rp; // 动平台半径
  double h;  // 平台高度
  double L1; // 主动臂长
  double L2; // 从动臂长

  // 舵机映射参数
  std::vector<int> servo_offset; // 舵机中位
  std::vector<int> servo_dir;    // 方向映射
  int servo_min;
  int servo_max;

  // 基座铰链角度
  std::vector<double> base_angles;

  // 基座铰链点(3x3矩阵)
  Eigen::Matrix3d B;

  // 动平台局部坐标(3x3矩阵)
  Eigen::Matrix3d P_local;

  // 平台中心初始位置
  Eigen::Vector3d p0;

  // IK状态：前一步关节角
  std::vector<double> q2_prev;

  // 兜底角度
  std::vector<int> last_valid;

  /**
   * @brief ZYX欧拉角旋转矩阵
   * @param yaw_deg 偏航角（度）
   * @param pitch_deg 俯仰角（度）
   * @param roll_deg 滚转角（度）
   * @return 旋转矩阵
   */
  Eigen::Matrix3d eulZYX(double yaw_deg, double pitch_deg, double roll_deg);

  /**
   * @brief 选择最接近的分支
   * @param q1 分支1
   * @param q2 分支2
   * @param q_prev 前一步角度
   * @return 更接近的分支
   */
  double chooseClosest(double q1, double q2, double q_prev);

  /**
   * @brief 获取另一个分支
   * @param q1 分支1
   * @param q2 分支2
   * @param q_cur 当前分支
   * @return 另一个分支
   */
  double otherBranch(double q1, double q2, double q_cur);

  /**
   * @brief 角度归一化到[-π, π]
   * @param x 输入角度
   * @return 归一化后的角度
   */
  double wrapAngle(double x);
};

#endif // KINEMATICS_HPP
