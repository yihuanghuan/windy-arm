#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <dummy_interface/msg/motor_control.hpp>
#include <manipulator/robotics/arm/torque_sim_arm.h>

namespace {

std::string SourcePath(const std::string& relative_path) {
  return std::string(MANIPULATOR_SOURCE_DIR) + "/" + relative_path;
}

TEST(TorqueSimArmTest, ClampsTorqueStepsAndPublishesFiniteFeedback) {
  manipulator::arm::TorqueSimArm arm;
  arm.SetModelPath(SourcePath("config/arm.urdf"));
  arm.SetInitialJointPositions(
      {0.418, -0.226, -0.364, -1.522, 0.789, 0.068, 0.042});
  arm.InitFromConfig(
      "", 0,
      SourcePath("config/motor_config.yaml"),
      SourcePath("config/arm_config.yaml"),
      "gamma");

  dummy_interface::msg::MotorControl command;
  command.enable = 1;
  command.current.assign(7, 100.0);
  ASSERT_TRUE(arm.SetMotorCommand(command));
  arm.Step(0.002);

  const auto& state = arm.GetJointStates();
  ASSERT_EQ(state.position.size(), 7U);
  ASSERT_EQ(state.velocity.size(), 7U);
  ASSERT_EQ(state.current.size(), 7U);
  // 被控对象模拟【硬件物理能力】，饱和在电机峰值力矩 (motor_config 的
  // max_torque)，不是额定值 (rated_torque)。额定值是连续热额定，属于
  // 软件【策略】上限，由 SMC 侧的 ActuatorAdapter 按 config 的
  // limits.torque 执行。
  //
  // 若这里改回额定值，就会形成【双重截断】：控制器按自己的上限限幅后，
  // 被控对象再截一次。后果是调高控制器配置里的力矩上限变成空操作 ——
  // 实测把 limits.torque 从 9 抬到 18 时仿真结果一字不变。
  const std::vector<double> expected_limits{
      27.0, 27.0, 27.0, 5.5, 5.5, 5.5, 5.5};
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_TRUE(std::isfinite(state.position[i]));
    EXPECT_TRUE(std::isfinite(state.velocity[i]));
    EXPECT_DOUBLE_EQ(state.current[i], expected_limits[i]);
  }

  command.enable = 0;
  ASSERT_TRUE(arm.SetMotorCommand(command));
  arm.Step(0.002);
  const auto& zero_state = arm.GetJointStates();
  for (double effort : zero_state.current) {
    EXPECT_DOUBLE_EQ(effort, 0.0);
  }
}

}  // namespace
