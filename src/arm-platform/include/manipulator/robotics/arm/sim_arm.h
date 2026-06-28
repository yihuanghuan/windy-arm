#pragma once

#include <manipulator/robotics/arm/abs_arm.h>

namespace manipulator::arm {

/**
 * @brief 虚拟机械臂（仿真用，不依赖任何真实硬件）
 *
 * 电机为理想伺服模型：位置指令经限位钳制后直接生效。
 * 上层 ArmPlatform / Controller / ROS 节点无需任何修改即可复用。
 */
class SimArm final : public AbsArm {
 public:
  SimArm();

  /**
   * @brief 初始化虚拟机械臂（port 和 baud 参数被忽略）
   */
  void Init(const std::string& port, uint32_t baud) override;

  /**
   * @brief 虚拟臂无需配置文件，直接转调 Init
   */
  void InitFromConfig(const std::string& port, uint32_t baud,
                      const std::string& /*motor_config_path*/,
                      const std::string& /*arm_config_path*/,
                      const std::string& /*arm_name*/) override {
    Init(port, baud);
  }
};

/**
 * @brief LL100-LH420 整机模型中的 6 轴机械臂虚拟控制对象。
 *
 * 只包含 A_Joint ~ F_Joint;4 个旋翼关节由独立 joint state publisher
 * 保持零位,用于整机 RViz TF 显示。
 */
class LL100SimArm final : public AbsArm {
 public:
  LL100SimArm();

  void Init(const std::string& port, uint32_t baud) override;

  void InitFromConfig(const std::string& port, uint32_t baud,
                      const std::string& /*motor_config_path*/,
                      const std::string& /*arm_config_path*/,
                      const std::string& /*arm_name*/) override {
    Init(port, baud);
  }
};

}  // namespace manipulator::arm
