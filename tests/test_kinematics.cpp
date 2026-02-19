/**
 * @file test_kinematics.cpp
 * @brief RRSKinematics Inverse Kinematics Unit Tests
 *        RRSKinematics 逆运动学单元测试
 *
 * Uses Google Test framework to verify 3RRS parallel mechanism IK correctness.
 * 使用 Google Test 框架验证3RRS并联机构逆运动学求解的正确性。
 */

#include "kinematics.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <memory>


class KinematicsTest : public ::testing::Test {
protected:
  void SetUp() override { ik = std::make_unique<RRSKinematics>(); }

  std::unique_ptr<RRSKinematics> ik;
};

// Test zero angle input (horizontal position) should return near-neutral angles
// 测试零角度输入（水平位置）应返回中位附近的角度
TEST_F(KinematicsTest, ZeroInputReturnsNeutral) {
  auto angles = ik->solve(0.0, 0.0);
  ASSERT_EQ(angles.size(), 3u);

  // Zero pose: servo angles should be in reasonable range (near 90° neutral)
  // 零姿态时舵机角度应在合理范围内（接近中位90°）
  for (int angle : angles) {
    EXPECT_GE(angle, 60);
    EXPECT_LE(angle, 120);
  }
}

// Test output angles are always within valid range [0, 180]
// 测试输出角度始终在有效范围 [0, 180] 内
TEST_F(KinematicsTest, OutputWithinServoRange) {
  // Test multiple typical inputs / 测试多组典型输入
  std::vector<std::pair<double, double>> test_cases = {
      {0, 0},   {10, 0},   {-10, 0},  {0, 10},    {0, -10},
      {15, 15}, {-15, 15}, {20, -20}, {-20, -20}, {30, 0},
  };

  for (auto &[pitch, roll] : test_cases) {
    auto angles = ik->solve(pitch, roll);
    ASSERT_EQ(angles.size(), 3u);
    for (int angle : angles) {
      EXPECT_GE(angle, 0) << "pitch=" << pitch << " roll=" << roll;
      EXPECT_LE(angle, 180) << "pitch=" << pitch << " roll=" << roll;
    }
  }
}

// Test should return exactly 3 servo angles
// 测试应返回正好3个舵机角度
TEST_F(KinematicsTest, ReturnsThreeAngles) {
  auto angles = ik->solve(5.0, 5.0);
  EXPECT_EQ(angles.size(), 3u);
}

// Test continuity: continuous input should give continuous output (no jumps)
// 测试连续性：连续输入的角度变化不应突变
TEST_F(KinematicsTest, ContinuousInputGivesContinuousOutput) {
  auto prev_angles = ik->solve(0.0, 0.0);

  for (double pitch = 1.0; pitch <= 20.0; pitch += 1.0) {
    auto angles = ik->solve(pitch, 0.0);
    for (size_t i = 0; i < 3; i++) {
      // Adjacent angle change should not exceed 15 degrees (prevent jumps)
      // 相邻角度变化不应超过15度（防止突变）
      EXPECT_LE(std::abs(angles[i] - prev_angles[i]), 15)
          << "Servo " << i << " jumped at pitch=" << pitch;
    }
    prev_angles = angles;
  }
}

// Test pitch symmetry: positive/negative pitch should deflect in opposite
// directions 测试pitch对称性：正负pitch应使角度向相反方向偏离中位
TEST_F(KinematicsTest, PitchSymmetry) {
  // Recreate for fresh initial state / 重新创建以从初始状态开始
  auto ik1 = std::make_unique<RRSKinematics>();
  auto ik2 = std::make_unique<RRSKinematics>();

  auto angles_pos = ik1->solve(15.0, 0.0);
  auto angles_neg = ik2->solve(-15.0, 0.0);

  // Positive/negative pitch: first servo angle should differ
  // 正负pitch时第一个舵机的角度应不同
  EXPECT_NE(angles_pos[0], angles_neg[0]);
}

// Test roll input works correctly / 测试roll输入正常
TEST_F(KinematicsTest, RollInputWorks) {
  auto angles_zero = ik->solve(0.0, 0.0);
  auto ik2 = std::make_unique<RRSKinematics>();
  auto angles_roll = ik2->solve(0.0, 15.0);

  // Roll input should change at least one servo angle
  // roll输入应改变至少一个舵机角度
  bool any_changed = false;
  for (size_t i = 0; i < 3; i++) {
    if (angles_roll[i] != angles_zero[i]) {
      any_changed = true;
    }
  }
  EXPECT_TRUE(any_changed);
}

// Test extreme angles don't crash / 测试极限角度不会崩溃
TEST_F(KinematicsTest, ExtremeAnglesDoNotCrash) {
  EXPECT_NO_THROW(ik->solve(35.0, 35.0));
  EXPECT_NO_THROW(ik->solve(-35.0, -35.0));
  EXPECT_NO_THROW(ik->solve(35.0, -35.0));
  EXPECT_NO_THROW(ik->solve(-35.0, 35.0));
}

// Test stability of repeated solves / 测试多次连续求解的稳定性
TEST_F(KinematicsTest, RepeatedSolveStable) {
  // Repeatedly solve same pose, results should converge
  // 反复求解同一姿态，结果应收敛一致
  for (int i = 0; i < 100; i++) {
    auto angles = ik->solve(10.0, 5.0);
    ASSERT_EQ(angles.size(), 3u);
    for (int angle : angles) {
      EXPECT_GE(angle, 0);
      EXPECT_LE(angle, 180);
    }
  }

  // Last two results should be identical (converged)
  // 最后两次结果应完全一致（收敛）
  auto a1 = ik->solve(10.0, 5.0);
  auto a2 = ik->solve(10.0, 5.0);
  EXPECT_EQ(a1, a2);
}
