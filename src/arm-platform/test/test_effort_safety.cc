// 力矩通道安全兜底的回归测试（阶段 2 前置修复）。
//
// 背景：直接力矩通道超时后原本发 p=0, d=0, current=0 —— MIT 模式下
// 这同时杀掉位置环，机械臂在重力下直接垮落。而 0.05s 的超时会被扫描
// 程序里任何 >50ms 的间隙（日志/拟合/转移）常规触发：位置模式下超时
// = 保持位置（安全），力矩通道下超时 = 垮臂。兜底必须是位置保持。
//
// 另一半：串口心跳断了意味着关节反馈陈旧，基于模型算出的力矩不可信，
// 输出非零前馈之前必须检查 IsLinkAlive。

#include <gtest/gtest.h>

#include <manipulator/ros_nodes/student_arm_node.h>

#include <vector>

namespace {

using manipulator::StudentArmNode;
using dummy_interface::msg::MotorControl;

const std::vector<double> kPositions = {0.1, 1.3, -1.1, 0.0, 0.5, -0.5};
const std::vector<double> kP = {30, 30, 30, 5, 5, 5};
const std::vector<double> kD = {1, 1, 1, 0.1, 0.1, 0.1};

}  // namespace

TEST(EffortSafety, HoldCommandKeepsThePositionLoopAlive) {
  auto cmd = StudentArmNode::BuildHoldCommand(kPositions, kP, kD);
  ASSERT_EQ(cmd.p.size(), kPositions.size());
  for (size_t i = 0; i < cmd.p.size(); ++i) {
    EXPECT_GT(cmd.p[i], 0.0) << "joint" << i + 1
        << ": p=0 means the joint goes limp in MIT mode";
    EXPECT_GT(cmd.d[i], 0.0) << "joint" << i + 1;
  }
}

TEST(EffortSafety, HoldCommandHoldsTheGivenPositionsWithZeroFeedforward) {
  auto cmd = StudentArmNode::BuildHoldCommand(kPositions, kP, kD);
  ASSERT_EQ(cmd.position.size(), kPositions.size());
  for (size_t i = 0; i < kPositions.size(); ++i) {
    EXPECT_DOUBLE_EQ(cmd.position[i], kPositions[i]);
    EXPECT_DOUBLE_EQ(cmd.current[i], 0.0)
        << "the fallback must never output torque feedforward";
    EXPECT_DOUBLE_EQ(cmd.velocity[i], 0.0);
  }
}

TEST(EffortSafety, HoldCommandSurvivesEmptyOrShortGainVectors) {
  // 参数缺失时也绝不允许增益落到 0
  auto cmd = StudentArmNode::BuildHoldCommand(kPositions, {}, {30});
  for (size_t i = 0; i < cmd.p.size(); ++i) {
    EXPECT_GT(cmd.p[i], 0.0);
    EXPECT_GT(cmd.d[i], 0.0);
  }
}

TEST(EffortSafety, HoldCommandRejectsNonPositiveGains) {
  auto cmd = StudentArmNode::BuildHoldCommand(
      kPositions, {0, -1, 30, 5, 5, 5}, {0, 1, 1, 0.1, 0.1, 0.1});
  EXPECT_GT(cmd.p[0], 0.0);
  EXPECT_GT(cmd.p[1], 0.0);
  EXPECT_DOUBLE_EQ(cmd.p[2], 30.0);
  EXPECT_GT(cmd.d[0], 0.0);
}

TEST(EffortSafety, TorqueIsBlockedWhenHeartbeatIsStale) {
  MotorControl cmd;
  cmd.current = {0.0, 1.5, 0.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(StudentArmNode::ShouldBlockTorque(false, cmd))
      << "stale heartbeat + nonzero feedforward must be blocked";
  EXPECT_FALSE(StudentArmNode::ShouldBlockTorque(true, cmd))
      << "live link passes torque through";
}

TEST(EffortSafety, PureHoldPassesEvenWithStaleHeartbeat) {
  MotorControl cmd;
  cmd.current = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(StudentArmNode::ShouldBlockTorque(false, cmd))
      << "position-hold (zero feedforward) is always allowed; blocking it "
         "would leave the arm with no command at all";
}
