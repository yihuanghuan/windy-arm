#!/usr/bin/env python3
"""把 joint5 从腕部奇异挪到安全角度（位置模式，实物专用）。

为什么需要
----------
6D 任务下 j5 = 0 是**腕部奇异**：j4 与 j6 转轴共线（URDF 里 joint4/joint6 都是
`axis="1 0 0"`，joint5 是 `axis="0 0 1"` 夹在中间），6D 雅可比掉秩。

实物复位位姿恰好是 j4=j5=j6≈0。2026-08-03 仿真扫描（blend ∈ {10,20,30} s ×
起步 j5 ∈ {0.17°, 15°}，6 次）实测：

    起步 j5     σmin           j4/j6 峰值速度        拉入后姿态 RMS
     0.17°   1e-6 ~ 3e-8     2.72 ~ 4.60 rad/s     blend=20 那组发散到 15.84°
     15°     ~1.0e-2         0.30 ~ 0.98 rad/s     0.004°

j4/j6 峰值 4.60 rad/s 是看门狗包线 5.0 的 92%，只剩 8% 余量；而且这段互搏
**几乎不花力矩**（j4 峰值 0.286 Nm = 额定的 17.9%），发生在雅可比近零方向上，
末端看不见 —— 力矩饱和率恒为 0%，完全掩盖了这个风险。

⚠️ 机械臂**上电即锁初始位姿，手掰不动**，所以只能靠位置指令把它挪过去。

安全边界
--------
* 只动 joint5，其余五轴按【实测值】锁住不动。
* joint5 限位 ±114.6°，本程序硬上界 60°（再大没有收益：σmin 在 40° 就已越过
  sigma_warn=0.03；60° 是 0.0552）。
* 位置模式频率硬上限 100 Hz（`ArmPlatform::ExecuteControlCycle` 对 dt<0.01
  直接抛异常），这里用 50 Hz。
* 五次多项式速度型，两端速度/加速度为零，默认 10 s 走完（峰值 ~2.6°/s）。
* 结束后**读回实测值确认**。位置模式受静摩擦影响明显（2026-08-02 实测复位后
  j2 被卡在偏差 10.19° 处），不能假定指令发了就到位。

⚠️ 必须与 SMC 共用同一个驱动实例
--------------------------------
2026-08-03 实测：**驱动进程一停，电机立刻失去保持，机械臂软掉**。那次先用位置
模式把 j5 移到 18.94°，随后杀掉驱动、改起力矩模式驱动，机械臂在这个间隙里
整体下垂 89°（j2 从 84.09° 塌到 −4.95°），j5 回到 0.14° —— 就位白做，
第 14 次上机其实仍是从奇异点起步（实测 σmin 0.00018）。

所以本程序走【力矩话题】而不是位置话题：
* `student_arm_node.cc:313` 的 `ControlLoop` 在 `direct_effort_enabled_` 为真时
  **永远走力矩分支并 return**，位置执行路径根本不会被调用 —— 位置指令收得下、
  执行不了。
* 而 DM 电机是 MIT 模式 `τ = kp(p_des−p) + kd(v_des−v) + t_ff`，
  用 MotorControl 带非零 kp/kd 发位置目标，电机自己闭环。看门狗的保位
  （`whole_task_smc_ros_runner.py` 的 `msg.p=[30]*n, msg.d=[1]*n`）就是这么做的。
于是就位与 SMC 可以共用同一个驱动实例，中间不重启、机械臂不塌。

用法
----
起驱动（**要**开 enable_direct_effort_command，与 SMC 同一条命令）::

    ros2 launch manipulator student_arm6.launch.py arm_type:=a_l1 \\
        port_name:=/dev/ttyUSB0 control_rate_hz:=200.0 \\
        enable_direct_effort_command:=true max_velocity:=0.5 use_rviz:=false

先跑本程序就位，**不要关驱动**，再起 SMC::

    python3 task_space_smc/hw/wrist_prepose.py --target-deg 20
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from dummy_interface.msg import MotorControl
from sensor_msgs.msg import JointState

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
J5 = 4                      # joint5 的下标
MAX_TARGET_DEG = 60.0       # 硬上界，见 docstring
MIN_TARGET_DEG = 5.0
PUBLISH_HZ = 100.0          # 力矩话题无 100 Hz 限制，但就位不需要更快
# MIT 模式增益。取看门狗保位那一档（kp=30, kd=1）的量级：够把腕部推动，
# 又不至于在大臂上产生冲击。腕部惯量 ~0.011 kgm²，kp=30 折合 ~0.33 Nm/rad。
HOLD_KP = [30.0] * 6
HOLD_KD = [1.0] * 6


def quintic(u: float) -> float:
    """五次 smoothstep，两端一阶二阶导为零。"""
    u = min(max(u, 0.0), 1.0)
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


class WristPrepose(Node):
    def __init__(self, target_rad: float, duration: float, settle: float):
        super().__init__("wrist_prepose")
        self.target_rad = target_rad
        self.duration = duration
        self.settle = settle
        self.q = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        # 走力矩话题：ControlLoop 在力矩模式下不执行位置路径（见模块 docstring）
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)

    def _cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if any(n not in idx for n in JOINT_NAMES):
            return
        self.q = [msg.position[idx[n]] for n in JOINT_NAMES]

    def wait_for_feedback(self, timeout=10.0):
        t0 = time.time()
        while self.q is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None

    def run(self) -> int:
        if not self.wait_for_feedback():
            self.get_logger().error("收不到 /joint_states，驱动没起来？")
            return 1

        start = list(self.q)
        self.get_logger().info(
            "起始位形 (deg): %s" % [round(math.degrees(x), 2) for x in start])

        if abs(start[J5] - self.target_rad) < math.radians(0.5):
            self.get_logger().info("joint5 已在目标附近，无需移动")
            return 0

        # 只动 j5，其余按实测锁住
        self.get_logger().info(
            "joint5: %.2f° -> %.2f°，用时 %.1f s（峰值 %.2f °/s）"
            % (math.degrees(start[J5]), math.degrees(self.target_rad),
               self.duration,
               math.degrees(1.875 * abs(self.target_rad - start[J5]) / self.duration)))

        dt = 1.0 / PUBLISH_HZ
        t0 = time.time()
        while True:
            elapsed = time.time() - t0
            if elapsed > self.duration + self.settle:
                break
            phase = quintic(min(elapsed / self.duration, 1.0))
            cmd = list(start)
            cmd[J5] = start[J5] + phase * (self.target_rad - start[J5])
            msg = MotorControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = cmd
            msg.velocity = [0.0] * 6
            msg.current = [0.0] * 6          # 纯位置伺服，无前馈力矩
            msg.p = list(HOLD_KP)
            msg.d = list(HOLD_KD)
            msg.enable = 1
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        # 读回确认 —— 位置模式受静摩擦影响，不能假定到位
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.05)
        final = list(self.q)
        err = math.degrees(final[J5] - self.target_rad)
        moved = [round(math.degrees(final[i] - start[i]), 2) for i in range(6)]
        self.get_logger().info(
            "结束位形 (deg): %s" % [round(math.degrees(x), 2) for x in final])
        self.get_logger().info("各轴实际位移 (deg): %s" % moved)
        self.get_logger().info(
            "joint5 落点 %.2f°，距目标 %.2f°" % (math.degrees(final[J5]), err))

        drift = max(abs(moved[i]) for i in range(6) if i != J5)
        if drift > 2.0:
            self.get_logger().error("非目标关节漂移 %.2f° > 2°，异常" % drift)
            return 2
        if abs(err) > 3.0:
            self.get_logger().error(
                "joint5 未到位（差 %.2f° > 3°）—— 很可能被静摩擦卡住，"
                "不要据此上 SMC" % err)
            return 3
        self.get_logger().info("✅ joint5 就位")
        return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target-deg", type=float, default=20.0,
                    help="joint5 目标角度（度），%.0f~%.0f"
                         % (MIN_TARGET_DEG, MAX_TARGET_DEG))
    ap.add_argument("--duration", type=float, default=10.0, help="移动时长（秒）")
    ap.add_argument("--settle", type=float, default=2.0, help="到位后保持（秒）")
    args = ap.parse_args()

    if not (MIN_TARGET_DEG <= args.target_deg <= MAX_TARGET_DEG):
        print("target-deg 必须在 [%.0f, %.0f]：小了压不住奇异，"
              "大了没有收益且离开已验证区间" % (MIN_TARGET_DEG, MAX_TARGET_DEG),
              file=sys.stderr)
        return 2
    if args.duration < 3.0:
        print("duration 不得小于 3 s —— 位置模式下太快会甩", file=sys.stderr)
        return 2

    rclpy.init()
    node = WristPrepose(math.radians(args.target_deg), args.duration, args.settle)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
