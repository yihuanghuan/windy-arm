"""重力补偿手动摆位：用于把实机推到画圈起始姿态。

位置环关闭，只发送 candidate 重力前馈和少量速度阻尼。程序不会自动把
机械臂拉向某个目标姿态；操作者手动推动机械臂，Ctrl-C 后退出。
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dummy_interface.msg import MotorControl

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, JOINT_LOWER, JOINT_UPPER, N_ARM,
)
from task_space_smc.core.model import load_arm6_calibration  # noqa: E402


class ManualGuidance(Node):
    def __init__(self, calibration_yaml: str | None, damping: float,
                 rate_hz: float):
        super().__init__("manual_guidance")
        self.model = GravityModel()
        cal = load_arm6_calibration(calibration_yaml)
        if not cal.get("source"):
            raise RuntimeError("candidate calibration YAML not found")
        self.offset = np.asarray(cal["zero_offset_rad"], dtype=float).reshape(N_ARM)
        self.ratio = np.asarray(cal["first_moment_ratio"], dtype=float).reshape(N_ARM)
        self.damping = float(damping)
        self.period = 1.0 / float(rate_hz)
        self.q = None
        self.qd = None
        self.last_state = 0.0
        self.hold_target = None
        self._stdin_thread = threading.Thread(target=self._wait_for_capture,
                                               daemon=True)
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._state_cb, 20)
        self.get_logger().info("manual guidance calibration: %s" % cal["source"])
        self.get_logger().info(
            "zero_offset_rad=%s ratio=%s" %
            (np.round(self.offset, 6).tolist(), np.round(self.ratio, 6).tolist()))
        self.get_logger().warn(
            "手动摆位已启用：位置环关闭；到目标后按回车捕获当前位置并保持，"
            "Ctrl-C 退出")
        self._stdin_thread.start()

    def _wait_for_capture(self):
        try:
            input()
            if self.q is not None:
                self.hold_target = self.q.copy()
                self.get_logger().warn(
                    "已捕获保持姿态 q=%s" % np.round(self.hold_target, 5).tolist())
        except EOFError:
            pass

    def _state_cb(self, msg: JointState):
        index = {name: i for i, name in enumerate(msg.name)}
        names = ["joint%d" % (i + 1) for i in range(N_ARM)]
        if any(name not in index for name in names):
            return
        self.q = np.array([msg.position[index[name]] for name in names], dtype=float)
        if len(msg.velocity) >= len(msg.name):
            self.qd = np.array([msg.velocity[index[name]] for name in names], dtype=float)
        self.last_state = time.monotonic()

    def _publish(self):
        if self.q is None or time.monotonic() - self.last_state > 0.10:
            self.get_logger().error("/joint_states 超时，停止发送手动引导指令")
            return False
        if np.any(self.q <= np.asarray(JOINT_LOWER) + 0.05) or \
                np.any(self.q >= np.asarray(JOINT_UPPER) - 0.05):
            self.get_logger().error("关节接近硬限位，停止发送手动引导指令")
            return False
        if self.qd is not None and np.max(np.abs(self.qd)) > 1.5:
            self.get_logger().error("反馈速度 %.3f rad/s 过高，停止发送手动引导指令" %
                                    np.max(np.abs(self.qd)))
            return False

        q_model = self.q + self.offset
        gravity = self.model.gravity(q_model) * self.ratio
        velocity_damping = self.damping * self.qd if self.qd is not None else np.zeros(N_ARM)
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        holding = self.hold_target is not None
        msg.position = (self.hold_target if holding else self.q).tolist()
        msg.velocity = [0.0] * N_ARM
        msg.current = (gravity - velocity_damping).tolist()
        msg.p = ([15.0, 15.0, 15.0, 3.0, 3.0, 3.0]
                 if holding else [0.0] * N_ARM)
        msg.d = ([1.0, 1.0, 1.0, 0.2, 0.2, 0.2]
                 if holding else [self.damping] * N_ARM)
        msg.enable = 1
        self.pub.publish(msg)
        return True

    def run(self):
        # Allow the hardware node time to publish its first feedback frame.
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and self.q is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.q is None:
            self.get_logger().error("10 s 内未收到 /joint_states，确认 student_arm_node 正在运行")
            return 1
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self._publish():
                return 1
            time.sleep(self.period)
        return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration-yaml", required=True)
    ap.add_argument("--damping", type=float, default=0.20)
    ap.add_argument("--rate", type=float, default=100.0)
    args = ap.parse_args()
    rclpy.init()
    node = None
    try:
        node = ManualGuidance(args.calibration_yaml, args.damping, args.rate)
        return node.run()
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
