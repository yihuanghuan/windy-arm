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
   *
   * 无配置文件时的 7 关节缺省构型，与 arm_config.yaml 的 a_l1_gamma 等价。
   */
  void Init(const std::string& port, uint32_t baud) override;

  /**
   * @brief 按 arm_config.yaml 构建虚拟臂，使仿真能覆盖 6 关节实机构型
   *
   * 对 arm_name="gamma" 产生的关节名与限位和 Init() 完全一致，因此原有
   * 7 关节仿真行为不变；arm_name="gamma6" 则得到 6 关节虚拟臂。
   * 配置缺失或读取失败时回退到 Init()。
   */
  void InitFromConfig(const std::string& port, uint32_t baud,
                      const std::string& motor_config_path,
                      const std::string& arm_config_path,
                      const std::string& arm_name) override;
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
