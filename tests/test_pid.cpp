/**
 * @file test_pid.cpp
 * @brief PIDController 单元测试 / PIDController Unit Tests
 */

#include "pid_controller.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <memory>

class PIDControllerTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Default params: Kp=0.3, Ki=0.0, Kd=0.0, maxTilt=35.0
    // 默认参数
    pid = std::make_unique<PIDController>(0.3, 0.0, 0.0, 35.0);
  }

  std::unique_ptr<PIDController> pid;
};

// Test initial state / 测试初始状态
TEST_F(PIDControllerTest, InitialStateIsZero) {
  EXPECT_DOUBLE_EQ(pid->getOutput(), 0.0);
}

// Test proportional control: positive error -> positive output
// 测试比例控制：正误差应产生正输出
TEST_F(PIDControllerTest, ProportionalPositiveError) {
  double result = pid->update(10.0, 0.02);
  EXPECT_GT(result, 0.0);
}

// Test proportional control: negative error -> negative output
// 测试比例控制：负误差应产生负输出
TEST_F(PIDControllerTest, ProportionalNegativeError) {
  double result = pid->update(-10.0, 0.02);
  EXPECT_LT(result, 0.0);
}

// Test zero error input maintains stability
// 测试零误差输入保持稳定
TEST_F(PIDControllerTest, ZeroErrorMaintainsOutput) {
  double r1 = pid->update(0.0, 0.02);
  double r2 = pid->update(0.0, 0.02);
  EXPECT_DOUBLE_EQ(r1, 0.0);
  EXPECT_DOUBLE_EQ(r2, 0.0);
}

// Test output clamping: should not exceed maxTilt
// 测试输出限幅：不应超过 maxTilt
TEST_F(PIDControllerTest, OutputClampedToMaxTilt) {
  // Large positive errors should be clamped to 35 degrees
  // 大量正误差应被限幅到35度
  for (int i = 0; i < 1000; i++) {
    pid->update(100.0, 0.02);
  }
  EXPECT_LE(pid->getOutput(), 35.0);
  EXPECT_GE(pid->getOutput(), -35.0);
}

// Test output clamping: negative direction
// 测试输出限幅：负方向
TEST_F(PIDControllerTest, OutputClampedToNegMaxTilt) {
  for (int i = 0; i < 1000; i++) {
    pid->update(-100.0, 0.02);
  }
  EXPECT_GE(pid->getOutput(), -35.0);
}

// Test reset functionality / 测试 reset 功能
TEST_F(PIDControllerTest, ResetClearsState) {
  pid->update(50.0, 0.02);
  pid->update(50.0, 0.02);
  EXPECT_NE(pid->getOutput(), 0.0);

  pid->reset();
  EXPECT_DOUBLE_EQ(pid->getOutput(), 0.0);
}

// Test parameters are stored correctly / 测试参数正确存储
TEST_F(PIDControllerTest, ParametersStored) {
  EXPECT_DOUBLE_EQ(pid->getKp(), 0.3);
  EXPECT_DOUBLE_EQ(pid->getKi(), 0.0);
  EXPECT_DOUBLE_EQ(pid->getKd(), 0.0);
  EXPECT_DOUBLE_EQ(pid->getMaxTilt(), 35.0);
}

// Test integral control / 测试积分控制
TEST(PIDIntegralTest, IntegralAccumulates) {
  PIDController pid_i(0.0, 1.0, 0.0, 100.0);
  // Continuous error should cause integral accumulation and output growth
  // 持续施加误差，积分项应积累导致输出增长
  double prev = 0.0;
  for (int i = 0; i < 10; i++) {
    double out = pid_i.update(5.0, 0.02);
    EXPECT_GE(out, prev);
    prev = out;
  }
  EXPECT_GT(pid_i.getOutput(), 0.0);
}

// Test derivative control / 测试微分控制
TEST(PIDDerivativeTest, DerivativeRespondsToChange) {
  PIDController pid_d(0.0, 0.0, 1.0, 100.0);
  // First call (no previous error) / 第一次调用（无前一次误差）
  pid_d.update(0.0, 0.02);
  // Sudden error change should produce derivative response
  // 误差突变应产生微分响应
  double out = pid_d.update(10.0, 0.02);
  EXPECT_GT(out, 0.0);
}

// Test dt=0 does not cause division by zero
// 测试 dt=0 不会除零
TEST_F(PIDControllerTest, ZeroDtSafe) {
  EXPECT_NO_THROW(pid->update(10.0, 0.0));
}
