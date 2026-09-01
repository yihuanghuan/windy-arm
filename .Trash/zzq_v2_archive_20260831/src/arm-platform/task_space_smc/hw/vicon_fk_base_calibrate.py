#!/usr/bin/env python3
"""静止位姿下的 Vicon/FK 标定，求 ``arm_base -> base_link``。

这是标定工具，不是任务空间控制器。臂在若干个低速位姿上停住（重力前馈已验证过），
记录关节反馈与两个 Vicon 刚体的中位数，再在现场约定 arm_ee == link6 下解固定变换

    V_T_E(q) = V_T_B · B_T_E(q)

把解出的 V_T_B 与实测的 V_T_Rb 比较即得 Rb_T_B，也就是要填进基座桥的安装变换。
样本都是静止的，所以这次几何标定不需要 Vicon 与编码器时钟对齐。

本工具【不】带桌面/围栏模型：它只用于清空过的 Vicon 标定区。关节硬限位、0.05 rad
限位余量、指令速度、实测速度与跟踪误差这几条中止判据仍然生效。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

try:
    import pinocchio as pin
    import rclpy
    from dummy_interface.msg import MotorControl
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError as exc:  # pragma: no cover - hardware dependency
    rclpy = None
    _IMPORT_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from task_space_smc.core.model import load_arm6_calibration
from task_space_smc.hw.fk_gravity_verify import GravityModel, N_ARM, JOINT_LOWER, JOINT_UPPER


RATE_HZ = 100.0
KP = np.array([30.0, 30.0, 30.0, 5.0, 5.0, 5.0])
KD = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
MIN_LIMIT_MARGIN = 0.05
MAX_MEASURED_SPEED = 0.50
MAX_TRACKING_ERROR = 0.18


def quintic(u: float) -> float:
    u = float(np.clip(u, 0.0, 1.0))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def quat_to_rotation(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(q_xyzw, dtype=float)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError("invalid Vicon quaternion")
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def rotation_angle(r: np.ndarray) -> float:
    return math.acos(float(np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0)))


def mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    if not rotations:
        raise ValueError("no rotations")
    u, _, vt = np.linalg.svd(sum(rotations))
    r = u @ vt
    if np.linalg.det(r) < 0.0:
        u[:, -1] *= -1.0
        r = u @ vt
    return r


def pose_median(samples: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
    if len(samples) < 5:
        raise ValueError("too few Vicon samples")
    p = np.median(np.asarray([x[0] for x in samples]), axis=0)
    r = mean_rotation([x[1] for x in samples])
    return p, r


class Calibrator(Node):
    def __init__(self, capture_sec: float):
        super().__init__("vicon_fk_base_calibrate")
        self.capture_sec = float(capture_sec)
        self.model = GravityModel()
        cal = load_arm6_calibration()
        self.zero_offset = np.asarray(cal["zero_offset_rad"], dtype=float)
        self.gravity_ratio = np.asarray(cal["first_moment_ratio"], dtype=float)
        self.q: np.ndarray | None = None
        self.qd: np.ndarray | None = None
        self.base_samples: deque[tuple[float, np.ndarray, np.ndarray]] = deque(maxlen=5000)
        self.ee_samples: deque[tuple[float, np.ndarray, np.ndarray]] = deque(maxlen=5000)
        self.pub = self.create_publisher(MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)
        self.create_subscription(PoseStamped, "/vrpn/arm_base/pose", self._base_cb, 100)
        self.create_subscription(PoseStamped, "/vrpn/arm_ee/pose", self._ee_cb, 100)

    def _joint_cb(self, msg: JointState) -> None:
        if len(msg.position) != N_ARM:
            return
        self.q = np.asarray(msg.position, dtype=float)
        self.qd = (np.asarray(msg.velocity, dtype=float) if len(msg.velocity) == N_ARM
                   else np.zeros(N_ARM))

    @staticmethod
    def _pose(msg: PoseStamped) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
            q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
            r = quat_to_rotation(q)
            return (p, r) if np.all(np.isfinite(p)) else None
        except (ValueError, FloatingPointError):
            return None

    def _base_cb(self, msg: PoseStamped) -> None:
        pose = self._pose(msg)
        if pose is not None:
            self.base_samples.append((time.monotonic(), *pose))

    def _ee_cb(self, msg: PoseStamped) -> None:
        pose = self._pose(msg)
        if pose is not None:
            self.ee_samples.append((time.monotonic(), *pose))

    def wait_ready(self, timeout: float = 10.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.q is not None and len(self.base_samples) >= 5 and len(self.ee_samples) >= 5:
                return True
        return False

    def gravity_ff(self) -> np.ndarray:
        if self.q is None:
            return np.zeros(N_ARM)
        return self.model.gravity(self.q + self.zero_offset) * self.gravity_ratio

    def command(self, q_ref: np.ndarray) -> None:
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(x) for x in q_ref]
        msg.velocity = [0.0] * N_ARM
        msg.current = [float(x) for x in self.gravity_ff()]
        msg.p = [float(x) for x in KP]
        msg.d = [float(x) for x in KD]
        msg.enable = 1
        self.pub.publish(msg)

    def validate_target(self, q: np.ndarray) -> str | None:
        q = np.asarray(q, dtype=float)
        margin = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)
        if not np.all(np.isfinite(q)) or np.min(margin) < MIN_LIMIT_MARGIN:
            return "target violates the 0.05 rad hard-limit margin"
        return None

    def guard(self, q_ref: np.ndarray, *, settled: bool) -> str | None:
        if self.q is None or self.qd is None:
            return "lost joint feedback"
        margin = np.minimum(self.q - JOINT_LOWER, JOINT_UPPER - self.q)
        if np.min(margin) < MIN_LIMIT_MARGIN:
            return "measured joint reached the 0.05 rad hard-limit margin"
        if np.max(np.abs(self.qd)) > MAX_MEASURED_SPEED:
            return f"measured speed {np.max(np.abs(self.qd)):.3f} rad/s exceeds limit"
        if settled and np.max(np.abs(self.q - q_ref)) > MAX_TRACKING_ERROR:
            return f"settled tracking error {np.max(np.abs(self.q-q_ref)):.3f} rad exceeds limit"
        return None

    def move_and_capture(self, q_target: np.ndarray, move_sec: float) -> dict:
        if self.q is None:
            raise RuntimeError("no joint feedback")
        why = self.validate_target(q_target)
        if why:
            raise RuntimeError(why)
        q_start = self.q.copy()
        t0, next_pub = time.monotonic(), time.monotonic()
        settle_sec = 2.0
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= move_sec + settle_sec + self.capture_sec:
                break
            phase = quintic(min(elapsed / move_sec, 1.0))
            q_ref = q_start + phase * (q_target - q_start)
            if now >= next_pub:
                self.command(q_ref)
                next_pub += 1.0 / RATE_HZ
            rclpy.spin_once(self, timeout_sec=0.002)
            why = self.guard(q_ref, settled=elapsed >= move_sec + settle_sec)
            if why:
                raise RuntimeError(why)

        # Static segment only: medians are valid without Vicon/encoder clock sync.
        cutoff = time.monotonic() - self.capture_sec
        base = [(p, r) for t, p, r in self.base_samples if t >= cutoff]
        ee = [(p, r) for t, p, r in self.ee_samples if t >= cutoff]
        if self.q is None:
            raise RuntimeError("lost joint feedback at capture")
        return dict(q_encoder=self.q.copy(), base=pose_median(base), ee=pose_median(ee),
                    base_count=len(base), ee_count=len(ee))

    def fk(self, q_encoder: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        q = np.asarray(q_encoder, dtype=float) + self.zero_offset
        pin.forwardKinematics(self.model.model, self.model.data, q)
        pin.updateFramePlacements(self.model.model, self.model.data)
        pose = self.model.data.oMf[self.model.ee_frame_id]
        return pose.translation.copy(), pose.rotation.copy()


def fit(samples: list[dict], node: Calibrator) -> dict:
    # For each static sample, V_T_B = V_T_E * inverse(B_T_E(q)).
    candidates = []
    for s in samples:
        p_e, r_e = s["ee"]
        p_be, r_be = node.fk(s["q_encoder"])
        candidates.append((p_e - r_e @ r_be.T @ p_be, r_e @ r_be.T))
    p_vb = np.median(np.asarray([p for p, _ in candidates]), axis=0)
    r_vb = mean_rotation([r for _, r in candidates])
    pos_err, rot_err = [], []
    rb_to_b = []
    for s in samples:
        p_be, r_be = node.fk(s["q_encoder"])
        p_pred = p_vb + r_vb @ p_be
        r_pred = r_vb @ r_be
        p_meas, r_meas = s["ee"]
        pos_err.append(float(np.linalg.norm(p_pred - p_meas)))
        rot_err.append(float(rotation_angle(r_meas.T @ r_pred)))
        p_rb, r_rb = s["base"]
        rb_to_b.append((r_rb.T @ (p_vb - p_rb), r_rb.T @ r_vb))
    p_rb_b = np.median(np.asarray([p for p, _ in rb_to_b]), axis=0)
    r_rb_b = mean_rotation([r for _, r in rb_to_b])
    return dict(
        vicon_from_base_link=dict(translation_m=p_vb.tolist(), rotation_matrix=r_vb.tolist()),
        arm_base_from_base_link=dict(translation_m=p_rb_b.tolist(), rotation_matrix=r_rb_b.tolist()),
        residual=dict(position_rms_m=float(np.sqrt(np.mean(np.square(pos_err)))),
                      position_max_m=float(max(pos_err)),
                      orientation_rms_deg=float(np.degrees(np.sqrt(np.mean(np.square(rot_err))))),
                      orientation_max_deg=float(np.degrees(max(rot_err)))),
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--move-sec", type=float, default=18.0)
    ap.add_argument("--capture-sec", type=float, default=2.0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--output-dir", default="")
    args = ap.parse_args()
    offsets = np.array([
        [0, 0, 0, 0, 0, 0],
        [0.18, -0.10, 0.10, 0, 0, 0],
        [-0.14, -0.17, 0.18, 0.16, 0.10, 0.12],
        [0.10, -0.25, 0.25, -0.18, -0.10, -0.16],
        [-0.20, -0.32, 0.30, 0.20, 0.16, 0.22],
    ])
    if args.dry_run:
        print("Five static poses; each offset is added to the measured start pose:")
        print(np.array2string(offsets, precision=3))
        return 0
    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2
    rclpy.init()
    node = Calibrator(args.capture_sec)
    try:
        if not node.wait_ready():
            raise RuntimeError("need /joint_states, /vrpn/arm_base/pose and /vrpn/arm_ee/pose")
        q0 = node.q.copy()
        targets = [q0 + off for off in offsets]
        for q in targets:
            why = node.validate_target(q)
            if why:
                raise RuntimeError(why)
        node.get_logger().info("starting static Vicon/FK calibration; no desk/fence guard by site agreement")
        samples = []
        for i, target in enumerate(targets, 1):
            node.get_logger().info("pose %d/%d target deg %s" %
                                   (i, len(targets), np.round(np.degrees(target), 1).tolist()))
            samples.append(node.move_and_capture(target, args.move_sec))
            node.get_logger().info("pose %d captured (%d base, %d ee frames)" %
                                   (i, samples[-1]["base_count"], samples[-1]["ee_count"]))
        result = fit(samples, node)
        result["assumptions"] = dict(arm_ee_equals_link6=True,
                                     q_fk="joint_encoder + existing arm6_calibration zero_offset_rad",
                                     vicon_axes="+y up; joint1 axis +x")
        result["samples"] = [dict(q_encoder_rad=s["q_encoder"].tolist(),
                                  arm_base_position_m=s["base"][0].tolist(),
                                  arm_ee_position_m=s["ee"][0].tolist(),
                                  arm_base_frames=s["base_count"], arm_ee_frames=s["ee_count"])
                             for s in samples]
        out_dir = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parent / "data" / "vicon_fk_calibration"
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / ("vicon_fk_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
        path.write_text(json.dumps(result, indent=2))
        print(json.dumps(result["residual"], indent=2))
        print("result -> %s" % path)
        return 0
    except Exception as exc:
        node.get_logger().error("ABORTED: %s" % exc)
        return 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
