#!/usr/bin/env python3
"""基于 Pinocchio IK 的末端轨迹演示脚本。

让机械臂末端 (link7) 在笛卡尔空间画一个圆, 每个周期用 IK 解出关节角,
发布到 /student/joint_command。

用法:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 move_arm_ik_demo.py
按 Ctrl+C 停止。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pinocchio_ik import PinocchioIK

JOINT_COUNT = 7
PUBLISH_RATE_HZ = 50.0
CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])  # 圆心 (base_link 系, m)
CIRCLE_RADIUS = 0.08                          # 半径 (m)
PERIOD_SEC = 8.0                              # 画一圈的周期 (s)


class MoveArmIkDemo(Node):
    def __init__(self):
        super().__init__('move_arm_ik_demo')
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.ik = PinocchioIK()
        self.t = 0.0
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.q_last = None  # 上次解, 用于热启动, 保证轨迹连续

        # 先解出起始点, 确认可达
        p0 = self._target(0.0)
        q0, ok = self.ik.solve(p0)
        if not ok:
            raise RuntimeError(f'起始点 {p0} IK 不可达, 请调整圆心/半径')
        self.q_last = q0

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'末端画圆: 圆心 {CIRCLE_CENTER.tolist()}, 半径 {CIRCLE_RADIUS} m, Ctrl+C 停止')

    def _target(self, t: float) -> np.ndarray:
        """t 时刻的目标位置: 在 y-z 平面画圆。"""
        w = 2.0 * math.pi / PERIOD_SEC
        return CIRCLE_CENTER + CIRCLE_RADIUS * np.array(
            [0.0, math.cos(w * t), math.sin(w * t)])

    def tick(self):
        self.t += self.dt
        q, ok = self.ik.solve(self._target(self.t), q_init=self.q_last)
        if not ok:
            self.get_logger().warn('IK 未收敛, 跳过本帧', throttle_duration_sec=2.0)
            return
        self.q_last = q

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        msg.position = q.tolist()
        msg.velocity = [0.0] * JOINT_COUNT
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MoveArmIkDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
