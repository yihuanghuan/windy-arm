#!/usr/bin/env python3
"""基于 Pinocchio IK 的末端直线往返演示脚本。

让机械臂末端 (link7) 在两点之间沿直线来回运动, 每帧用 IK 解出关节角,
发布到 /student/joint_command。插值使用余弦曲线, 端点处速度为零, 运动平滑。

用法:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 move_arm_line_demo.py
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
POINT_A = np.array([0.35, -0.15, 0.15])  # 起点 (base_link 系, m)
POINT_B = np.array([0.35, 0.15, 0.25])   # 终点 (base_link 系, m)
PERIOD_SEC = 6.0                          # 一个来回的周期 (s)


class MoveArmLineDemo(Node):
    def __init__(self):
        super().__init__('move_arm_line_demo')
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.ik = PinocchioIK()
        self.t = 0.0
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.q_last = None  # 上次解, 用于热启动, 保证轨迹连续

        # 先验证两个端点都可达
        for name, p in (('A', POINT_A), ('B', POINT_B)):
            q, ok = self.ik.solve(p, q_init=self.q_last)
            if not ok:
                raise RuntimeError(f'端点 {name} {p.tolist()} IK 不可达, 请调整坐标')
            self.q_last = q
        # 回到起点作为初始解
        self.q_last, _ = self.ik.solve(POINT_A)

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'末端直线往返: A={POINT_A.tolist()} <-> B={POINT_B.tolist()}, '
            f'周期 {PERIOD_SEC}s, Ctrl+C 停止')

    def _target(self, t: float) -> np.ndarray:
        """t 时刻的目标位置: A-B 之间余弦插值往返, 端点速度为零。"""
        # s 在 [0, 1] 之间往返: t=0 时 s=0 (A 点)
        s = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / PERIOD_SEC))
        return POINT_A + s * (POINT_B - POINT_A)

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
    node = MoveArmLineDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
