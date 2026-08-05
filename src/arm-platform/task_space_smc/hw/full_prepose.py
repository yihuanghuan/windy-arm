import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from dummy_interface.msg import MotorControl
from sensor_msgs.msg import JointState

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
PUBLISH_HZ = 100.0
HOLD_KP = [30.0] * 6
HOLD_KD = [1.0] * 6
LOWER = [-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0]
UPPER = [4.0, 2.0, 1.5708, 4.0, 2.0, 4.0]
MAX_STEP_RAD = 2.0


def quintic(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


class FullPrepose(Node):
    def __init__(self, target, duration, settle, kp=None):
        super().__init__("full_prepose")
        self.target = list(target)
        self.duration = duration
        self.settle = settle
        self.kp = list(kp) if kp else list(HOLD_KP)
        self.q = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
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
        delta = [self.target[i] - start[i] for i in range(6)]
        self.get_logger().info(
            "起始 (deg): %s" % [round(math.degrees(x), 2) for x in start])
        self.get_logger().info(
            "目标 (deg): %s" % [round(math.degrees(x), 2) for x in self.target])
        self.get_logger().info(
            "行程 (deg): %s" % [round(math.degrees(x), 2) for x in delta])

        for i, t in enumerate(self.target):
            if not (LOWER[i] < t < UPPER[i]):
                self.get_logger().error(
                    "joint%d 目标 %.4f 越限 [%.3f, %.3f]" % (i + 1, t, LOWER[i], UPPER[i]))
                return 2
        big = max(abs(x) for x in delta)
        if big > MAX_STEP_RAD:
            self.get_logger().error(
                "单轴行程 %.2f rad > %.2f，太大，分两段做" % (big, MAX_STEP_RAD))
            return 2
        peak = [1.875 * abs(x) / self.duration for x in delta]
        self.get_logger().info(
            "峰值速度 (rad/s): %s" % [round(v, 3) for v in peak])
        if max(peak) > 0.5:
            self.get_logger().error("峰值速度 %.3f rad/s 过快，加大 --duration" % max(peak))
            return 2

        dt = 1.0 / PUBLISH_HZ
        t0 = time.time()
        while True:
            elapsed = time.time() - t0
            if elapsed > self.duration + self.settle:
                break
            phase = quintic(min(elapsed / self.duration, 1.0))
            cmd = [start[i] + phase * delta[i] for i in range(6)]
            msg = MotorControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = cmd
            msg.velocity = [0.0] * 6
            msg.current = [0.0] * 6
            msg.p = list(self.kp)
            msg.d = list(HOLD_KD)
            msg.enable = 1
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.05)
        final = list(self.q)
        err = [math.degrees(final[i] - self.target[i]) for i in range(6)]
        self.get_logger().info(
            "落点 (deg): %s" % [round(math.degrees(x), 2) for x in final])
        self.get_logger().info("残差 (deg): %s" % [round(e, 2) for e in err])
        worst = max(abs(e) for e in err)
        if worst > 3.0:
            self.get_logger().error(
                "有轴未到位（最大 %.2f° > 3°）—— 多半被静摩擦卡住" % worst)
            return 3
        self.get_logger().info("✅ 整臂就位，最大残差 %.2f°" % worst)
        return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target-rad", type=str, required=True,
                    help="6 个目标角度，逗号分隔，弧度")
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--settle", type=float, default=3.0)
    ap.add_argument("--kp", type=str, default="",
                    help="6 个位置增益，逗号分隔；缺省 30")
    args = ap.parse_args()
    target = [float(x) for x in args.target_rad.split(",")]
    if len(target) != 6:
        print("需要 6 个角度")
        return 2
    rclpy.init()
    kp = [float(x) for x in args.kp.split(",")] if args.kp else None
    if kp is not None and len(kp) != 6:
        print("--kp 需要 6 个值"); return 2
    node = FullPrepose(target, args.duration, args.settle, kp)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
