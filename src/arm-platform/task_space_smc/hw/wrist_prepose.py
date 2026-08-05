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
J5 = 4
MAX_TARGET_DEG = 60.0
MIN_TARGET_DEG = 5.0
PUBLISH_HZ = 100.0


HOLD_KP = [30.0] * 6
HOLD_KD = [1.0] * 6


def quintic(u: float) -> float:
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
            msg.current = [0.0] * 6
            msg.p = list(HOLD_KP)
            msg.d = list(HOLD_KD)
            msg.enable = 1
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)


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
