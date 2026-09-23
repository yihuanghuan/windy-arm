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

TEST(SpeedSafety, GuideWarningDurationResetAndIndependentAxes) {
  manipulator::SpeedSafety guard({.5,.5,.5,.8,.08,1.2});
  std::vector<double> v(6, 0.);
  v[0] = .6; guard.Update(v, true, 0.);
  EXPECT_TRUE(guard.warning[0]); EXPECT_FALSE(guard.fault);
  v[0] = .85; guard.Update(v, true, .01); guard.Update(v, true, .08);
  EXPECT_FALSE(guard.fault);
  v[0] = .8; v[1] = .85; guard.Update(v, true, .085);
  EXPECT_DOUBLE_EQ(guard.duration[0], 0.);
  v[0] = .85; guard.Update(v, true, .10); guard.Update(v, true, .16);
  EXPECT_FALSE(guard.fault);
  guard.Update(v, true, .166);
  EXPECT_TRUE(guard.fault); EXPECT_FALSE(guard.axes[0]); EXPECT_TRUE(guard.axes[1]);
  EXPECT_EQ(guard.type[1], "sustained");
  guard.Update(std::vector<double>(6, 0.), true, .17);
  EXPECT_TRUE(guard.fault);
}

TEST(SpeedSafety, InstantAndTransitionAreImmediate) {
  manipulator::SpeedSafety instant({.5,.5,.5,.8,.08,1.2});
  instant.Update(std::vector<double>(6, 1.25), true, 0.);
  EXPECT_TRUE(instant.fault); EXPECT_EQ(instant.type[5], "instantaneous");
  manipulator::SpeedSafety ramp({.5,.5,.5,.8,.08,1.2});
  ramp.Update(std::vector<double>(6, .51), false, 0.);
  EXPECT_TRUE(ramp.fault); EXPECT_EQ(ramp.type[0], "transition");
}

TEST(SpeedSafety, ManualProfileIsExplicitAndCannotRelaxTransition) {
  auto make=[]() {
    manipulator::SpeedSafety s({.5,.5,.5,1.,.08,1.2});
    s.SetManual(std::vector<double>(6,.8),std::vector<double>(6,1.5),std::vector<double>(6,2.),.12);
    return s;
  };
  auto s=make();std::vector<double> v(6,1.);
  s.Update(v,true,0.,true);EXPECT_FALSE(s.fault);EXPECT_TRUE(s.warning[0]);
  v[1]=1.6;s.Update(v,true,.01,true);s.Update(v,true,.06,true);EXPECT_FALSE(s.fault);
  v[1]=1.4;s.Update(v,true,.07,true);EXPECT_DOUBLE_EQ(s.duration[1],0.);
  v[1]=1.6;s.Update(v,true,.08,true);s.Update(v,true,.201,true);
  EXPECT_TRUE(s.fault);EXPECT_EQ(s.type[1],"sustained");EXPECT_FALSE(s.axes[0]);
  auto instant=make();instant.Update(std::vector<double>(6,2.1),true,0.,true);
  EXPECT_TRUE(instant.fault);EXPECT_EQ(instant.type[0],"instantaneous");
  auto ramp=make();ramp.Update(std::vector<double>(6,.51),false,0.,true);
  EXPECT_TRUE(ramp.fault);EXPECT_EQ(ramp.type[0],"transition");
  auto ordinary=make();ordinary.Update(std::vector<double>(6,1.3),true,0.);
  EXPECT_TRUE(ordinary.fault);EXPECT_EQ(ordinary.type[0],"instantaneous");
}

TEST(SpeedSafety, WiderManualProfileKeepsAbsoluteCapAndIndependentTimers) {
  for (size_t axis = 0; axis < 6; ++axis) {
    manipulator::SpeedSafety s({.5,.5,.5,1.,.08,1.2});
    s.SetManual(std::vector<double>(6,1.),std::vector<double>(6,1.8),std::vector<double>(6,2.),.12);
    std::vector<double> v(6,0.);v[axis]=1.7;
    s.Update(v,true,0.,true);s.Update(v,true,1.,true);
    EXPECT_FALSE(s.fault);EXPECT_TRUE(s.warning[axis]);EXPECT_DOUBLE_EQ(s.duration[axis],0.);
    v[axis]=1.9;s.Update(v,true,1.01,true);s.Update(v,true,1.06,true);
    EXPECT_FALSE(s.fault);
    v[axis]=1.8;s.Update(v,true,1.07,true);EXPECT_DOUBLE_EQ(s.duration[axis],0.);
    const size_t other=(axis+1)%6;
    v[axis]=1.9;s.Update(v,true,1.08,true);
    v[other]=1.9;s.Update(v,true,1.13,true);
    s.Update(v,true,1.201,true);
    EXPECT_TRUE(s.fault);EXPECT_TRUE(s.axes[axis]);EXPECT_FALSE(s.axes[other]);
    EXPECT_EQ(s.type[axis],"sustained");
    v.assign(6,0.);s.Update(v,true,2.,true);EXPECT_TRUE(s.fault);
    manipulator::SpeedSafety instant({.5,.5,.5,1.,.08,1.2});
    instant.SetManual(std::vector<double>(6,1.),std::vector<double>(6,1.8),std::vector<double>(6,2.),.12);
    v[axis]=2.1;instant.Update(v,true,0.,true);
    EXPECT_TRUE(instant.fault);EXPECT_EQ(instant.type[axis],"instantaneous");
  }
}

TEST(SpeedSafety, WristInstantExtensionRetainsSustainedAndTransitionLimits) {
  auto make=[] {
    manipulator::SpeedSafety s({.5,.5,.5,1.,.08,1.2});
    s.SetManual({1.,1.,1.,1.5,1.5,1.5},std::vector<double>(6,1.8),{2.,2.,2.,2.2,2.2,2.2},.12);
    return s;
  };
  for (size_t axis=0;axis<6;++axis) {
    auto s=make();std::vector<double> v(6,0.);v[axis]=1.2;
    s.Update(v,true,0.,true);EXPECT_FALSE(s.fault);EXPECT_EQ(s.warning[axis],axis<3);
    v[axis]=1.5;s.Update(v,true,.001,true);EXPECT_EQ(s.warning[axis],axis<3);
    v[axis]=1.51;s.Update(v,true,.002,true);EXPECT_TRUE(s.warning[axis]);EXPECT_FALSE(s.fault);
    v[axis]=2.1;s.Update(v,true,.01,true);
    if (axis<3) {
      EXPECT_TRUE(s.fault);EXPECT_EQ(s.type[axis],"instantaneous");
    } else {
      s.Update(v,true,.05,true);EXPECT_FALSE(s.fault);
      v[axis]=1.7;s.Update(v,true,.06,true);EXPECT_DOUBLE_EQ(s.duration[axis],0.);
      v[axis]=2.1;s.Update(v,true,.07,true);s.Update(v,true,.191,true);
      EXPECT_TRUE(s.fault);EXPECT_EQ(s.type[axis],"sustained");
    }
    auto cap=make();v[axis]=2.3;cap.Update(v,true,0.,true);
    EXPECT_TRUE(cap.fault);EXPECT_EQ(cap.type[axis],"instantaneous");
    auto transition=make();v[axis]=.51;transition.Update(v,false,0.,true);
    EXPECT_TRUE(transition.fault);EXPECT_EQ(transition.type[axis],"transition");
    auto guide=make();v[axis]=1.3;guide.Update(v,true,0.,false);
    EXPECT_TRUE(guide.fault);EXPECT_EQ(guide.type[axis],"instantaneous");
  }
}
