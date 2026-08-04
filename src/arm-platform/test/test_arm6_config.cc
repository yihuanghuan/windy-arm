// 6 自由度实机构型的配置回归测试。
//
// 实测固件按 7 通道上报 (dataLen=28), 但通道 7 是幽灵数据:
//   position -6.5231 rad / velocity 27.9731 rad/s / temperature 1
// 位置超出 ±4 rad 限位, 速度超 RS05 上限 5.6 倍。这些值一旦进入
// /joint_states 就会污染任何基于模型的计算 (科氏项按 q̇² 增长)。
//
// 屏蔽手段是只在 arm_config.yaml 的 a_l1_gamma6 里声明 6 个关节,
// 驱动层便不会去读第 7 通道。本测试锁住这个契约, 同时确认原
// 7 关节构型 a_l1_gamma 未被改动。

#include <gtest/gtest.h>

#include <manipulator/robotics/arm/sim_arm.h>
#include <manipulator/robotics/config/arm_config.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <string>
#include <vector>

namespace {

std::string MotorConfigPath() {
  return std::string(MANIPULATOR_SOURCE_DIR) + "/config/motor_config.yaml";
}

std::string ArmConfigPath() {
  return std::string(MANIPULATOR_SOURCE_DIR) + "/config/arm_config.yaml";
}

// SimArm is constructed directly rather than via ArmFactory: Create() does
// `std::move(it->second)`, so the factory hands out each arm exactly once and
// a second Create("sim") in the same process would return nullptr.
std::vector<std::string> JointNamesFor(const std::string& arm_version) {
  manipulator::arm::SimArm arm;
  arm.InitFromConfig("", 921600, MotorConfigPath(), ArmConfigPath(),
                     arm_version);
  return arm.GetJointNames();
}

}  // namespace

TEST(Arm6Config, SixJointVersionCreatesExactlySixMotors) {
  const auto names = JointNamesFor("gamma6");
  ASSERT_EQ(names.size(), 6u);
  EXPECT_EQ(std::count(names.begin(), names.end(), "joint7"), 0)
      << "joint7 must not exist: the physical arm has no motor there";
  for (int i = 1; i <= 6; ++i) {
    EXPECT_EQ(std::count(names.begin(), names.end(), "joint" + std::to_string(i)), 1);
  }
}

TEST(Arm6Config, SevenJointVersionIsUnchanged) {
  const auto names = JointNamesFor("gamma");
  ASSERT_EQ(names.size(), 7u);
  EXPECT_EQ(std::count(names.begin(), names.end(), "joint7"), 1);
}

TEST(Arm6Config, SixJointLimitsMatchTheSevenJointPrefix) {
  const YAML::Node yaml = YAML::LoadFile(ArmConfigPath());
  const auto seven =
      manipulator::config::ConfigLoader::LoadArmConfig(yaml, "a_l1_gamma");
  const auto six =
      manipulator::config::ConfigLoader::LoadArmConfig(yaml, "a_l1_gamma6");

  ASSERT_EQ(seven.joints.size(), 7u);
  ASSERT_EQ(six.joints.size(), 6u);

  // 6 关节构型必须是 7 关节构型的前缀: 同名、同 id、同限位、同电机型号。
  // 否则驱动层的 id 与固件通道号会错位。
  for (size_t i = 0; i < six.joints.size(); ++i) {
    EXPECT_EQ(six.joints[i].name, seven.joints[i].name) << "index " << i;
    EXPECT_EQ(six.joints[i].id, seven.joints[i].id) << "index " << i;
    EXPECT_EQ(six.joints[i].model, seven.joints[i].model) << "index " << i;
    EXPECT_DOUBLE_EQ(six.joints[i].lower_limit, seven.joints[i].lower_limit)
        << "index " << i;
    EXPECT_DOUBLE_EQ(six.joints[i].upper_limit, seven.joints[i].upper_limit)
        << "index " << i;
  }
}

TEST(Arm6Config, RatedTorqueLimitsComeFromTheMotorModels) {
  // 实测量化步长反解出的固件硬饱和是 ±28 Nm (DM4340) / ±6 Nm (RS05);
  // motor_config.yaml 的 rated_torque 必须安全地落在其内。
  const YAML::Node yaml = YAML::LoadFile(MotorConfigPath());
  const auto models = manipulator::config::ConfigLoader::LoadMotorModels(yaml);

  ASSERT_EQ(models.count("DM4340-48V"), 1u);
  ASSERT_EQ(models.count("RS05"), 1u);
  EXPECT_GT(models.at("DM4340-48V").rated_torque, 0.0);
  EXPECT_LT(models.at("DM4340-48V").rated_torque, 28.0);
  EXPECT_GT(models.at("RS05").rated_torque, 0.0);
  EXPECT_LT(models.at("RS05").rated_torque, 6.0);
}
