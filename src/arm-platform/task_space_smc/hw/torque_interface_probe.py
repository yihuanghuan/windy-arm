#!/usr/bin/env python3
"""Low-amplitude probe for the direct torque/feedforward command path."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    # 直接当脚本跑时把【包的父目录】放进 sys.path。2026-08-04 目录分层后
    # 模块下沉到 core/ sim/ hw/，原来写死的 package_dir.parent 会指到
    # task_space_smc/ 本身（少上溯一级），import task_space_smc.* 全部失败。
    # 改成逐级上溯直到走出带 __init__.py 的目录，对任意层深都成立。
    _here = Path(__file__).resolve().parent
    _root = _here
    while (_root / "__init__.py").exists():
        _root = _root.parent
    sys.path = [p for p in sys.path if Path(p or ".").resolve() != _here]
    sys.path.insert(0, str(_root))

from task_space_smc.core.model import DEFAULT_ARM_JOINT_NAMES

try:
    import rclpy
    from dummy_interface.msg import MotorControl
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except Exception:  # pragma: no cover
    rclpy = None
    Node = object


class TorqueInterfaceProbe(Node):
    def __init__(self):
        super().__init__("torque_interface_probe")
        self.declare_parameter("joint_index", 0)
        self.declare_parameter("amplitude_nm", 0.05)
        self.declare_parameter("frequency_hz", 0.25)
        self.declare_parameter("duration_sec", 8.0)
        self.declare_parameter("kp", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("enable", False)
        self.q = np.zeros(7)
        self.qd = np.zeros(7)
        self.start = self.get_clock().now().nanoseconds * 1e-9
        self.pub = self.create_publisher(MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.timer = self.create_timer(0.01, self._tick)
        self.get_logger().warn(
            "Torque probe publishes direct MotorControl. Start student_arm_node with enable_direct_effort_command:=true."
        )

    def _joint_state_cb(self, msg):
        by_name = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(DEFAULT_ARM_JOINT_NAMES):
            if name in by_name:
                i = by_name[name]
                self.q[j] = msg.position[i]
                self.qd[j] = msg.velocity[i] if i < len(msg.velocity) else 0.0

    def _zero(self):
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = self.q.tolist()
        msg.velocity = [0.0] * 7
        msg.current = [0.0] * 7
        msg.p = [0.0] * 7
        msg.d = [0.0] * 7
        msg.enable = 0
        self.pub.publish(msg)

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.start
        if t > float(self.get_parameter("duration_sec").value):
            self._zero()
            self.destroy_timer(self.timer)
            rclpy.shutdown()
            return
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = self.q.tolist()
        msg.velocity = [0.0] * 7
        msg.current = [0.0] * 7
        idx = int(self.get_parameter("joint_index").value)
        amp = float(self.get_parameter("amplitude_nm").value)
        freq = float(self.get_parameter("frequency_hz").value)
        if 0 <= idx < 7:
            msg.current[idx] = amp * math.sin(2.0 * math.pi * freq * t)
        msg.p = [float(self.get_parameter("kp").value)] * 7
        msg.d = [float(self.get_parameter("kd").value)] * 7
        msg.enable = 1 if bool(self.get_parameter("enable").value) else 0
        self.pub.publish(msg)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("rclpy is not available")
    rclpy.init(args=args)
    node = TorqueInterfaceProbe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
