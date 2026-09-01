"""Low-risk torque prepose: gravity compensation plus joint-space PD.

This is an approach tool, not the task-space SMC controller.  It generates a
quintic joint trajectory from the measured pose to a requested target and
publishes

    tau = gravity(q_encoder + zero_offset) + Kp(qd - q) + Kd(qd_dot - q_dot)

The command is torque-only (MotorControl p=d=0).  It is dry-run by default;
``--execute`` is required before any effort command is published.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from dummy_interface.msg import MotorControl
except ImportError as exc:  # pragma: no cover - reported by main
    rclpy = None
    _IMPORT_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.model import load_arm6_calibration
from task_space_smc.core.friction_feedforward import (
    FrictionFeedforwardConfig, stiction_feedforward,
)
from task_space_smc.hw.fk_gravity_verify import (
    GravityModel, JOINT_LOWER, JOINT_UPPER, N_ARM, check_setpoint,
)


JOINT_NAMES = [f"joint{i}" for i in range(1, N_ARM + 1)]
TORQUE_LIMIT = np.array([9.0, 9.0, 9.0, 1.6, 1.6, 1.6])
SPEED_LIMIT = 0.35
FEEDBACK_TIMEOUT = 0.15
RATE_HZ = 100.0

# Conservative outer-loop gains.  The inner motor position gains are zero in
# the MotorControl message; these values are the only position correction.
DEFAULT_KP = np.array([2.0, 2.0, 2.0, 0.45, 0.45, 0.45])
DEFAULT_KD = np.array([0.35, 0.35, 0.35, 0.12, 0.12, 0.12])
DEFAULT_TAU_RATE = np.array([3.0, 3.0, 3.0, 0.8, 0.8, 0.8])


def quintic(u: float) -> tuple[float, float]:
    u = min(max(float(u), 0.0), 1.0)
    s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    ds_du = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
    return s, ds_du


def parse_vec(text: str, size: int, name: str) -> np.ndarray:
    values = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(values) != size:
        raise ValueError(f"{name} expects {size} comma-separated values")
    result = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains NaN/Inf")
    return result


class PdGravityPrepose(Node):
    def __init__(self, calibration_yaml: str, kp: np.ndarray, kd: np.ndarray,
                 friction_scale: float, friction_epsilon: float,
                 friction_error_gain: float, execute: bool):
        super().__init__("pd_gravity_prepose")
        self.model = GravityModel()
        cal = load_arm6_calibration(calibration_yaml)
        if cal["source"] is None:
            raise RuntimeError("calibration YAML not found")
        self.cal_source = cal["source"]
        self.zero = np.asarray(cal["zero_offset_rad"], dtype=float)
        self.ratio = np.asarray(cal["first_moment_ratio"], dtype=float)
        self.stiction = np.asarray(cal["stiction_nm"], dtype=float)
        raw_pos = cal.get("kinetic_friction_pos_nm") or []
        raw_neg = cal.get("kinetic_friction_neg_nm") or []
        # Match the runner: an invalid directional value falls back only for
        # that joint; valid joints must keep their measured F+/F-.
        pos = np.array([
            float(raw_pos[i]) if i < len(raw_pos)
            and raw_pos[i] is not None
            and np.isfinite(float(raw_pos[i])) and float(raw_pos[i]) >= 0.0
            else self.stiction[i]
            for i in range(N_ARM)])
        neg = np.array([
            float(raw_neg[i]) if i < len(raw_neg)
            and raw_neg[i] is not None
            and np.isfinite(float(raw_neg[i])) and float(raw_neg[i]) >= 0.0
            else self.stiction[i]
            for i in range(N_ARM)])
        self.friction = FrictionFeedforwardConfig(
            self.stiction, positive_nm=pos, negative_nm=neg,
            enabled=friction_scale > 0.0, scale=friction_scale,
            velocity_epsilon=friction_epsilon, error_gain=friction_error_gain)
        self.kp = np.asarray(kp, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.friction_scale = float(friction_scale)
        self.execute = bool(execute)
        self.q = None
        self.qd = None
        self.last_feedback = None
        self.last_tau = np.zeros(N_ARM)
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)
        self.get_logger().info("calibration YAML: %s" % self.cal_source)
        self.get_logger().info(
            "PD gravity prepose: execute=%s zero_deg=%s ratio=%s"
            % (self.execute, np.round(np.degrees(self.zero), 3).tolist(),
               np.round(self.ratio, 5).tolist()))
        self.get_logger().info(
            "directional friction: F+=%s Nm F-=%s Nm scale=%.3f"
            % (np.round(self.friction.positive_nm, 5).tolist(),
               np.round(self.friction.negative_nm, 5).tolist(),
               self.friction.scale))

    def _joint_cb(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if any(name not in index for name in JOINT_NAMES):
            return
        self.q = np.array([msg.position[index[name]] for name in JOINT_NAMES],
                          dtype=float)
        self.qd = np.array([
            msg.velocity[index[name]] if index[name] < len(msg.velocity) else 0.0
            for name in JOINT_NAMES
        ], dtype=float)
        self.last_feedback = time.monotonic()

    def gravity(self, q_encoder: np.ndarray) -> np.ndarray:
        q_model = np.asarray(q_encoder, dtype=float) + self.zero
        return self.model.gravity(q_model) * self.ratio

    def publish_tau(self, tau: np.ndarray) -> None:
        tau = np.asarray(tau, dtype=float)
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Torque-only mode: inner MIT position/velocity gains are disabled.
        msg.position = self.q.tolist()
        msg.velocity = [0.0] * N_ARM
        msg.current = tau.tolist()
        msg.p = [0.0] * N_ARM
        msg.d = [0.0] * N_ARM
        msg.enable = 1
        self.pub.publish(msg)

    def stop(self) -> None:
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = self.q.tolist() if self.q is not None else [0.0] * N_ARM
        msg.velocity = [0.0] * N_ARM
        msg.current = [0.0] * N_ARM
        msg.p = [0.0] * N_ARM
        msg.d = [0.0] * N_ARM
        msg.enable = 0
        self.pub.publish(msg)

    def effort_path_enabled(self) -> bool:
        from rcl_interfaces.srv import GetParameters
        client = self.create_client(GetParameters,
                                    "/student_arm_node/get_parameters")
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        request = GetParameters.Request()
        request.names = ["enable_direct_effort_command"]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return bool(future.result() and future.result().values
                    and future.result().values[0].bool_value)


def plan(target: np.ndarray, cal_path: str, kp: np.ndarray, kd: np.ndarray,
         duration: float, friction_scale: float) -> int:
    model = GravityModel()
    cal = load_arm6_calibration(cal_path)
    report = check_setpoint(target, model)
    if not report.ok:
        print("REFUSED target: " + "; ".join(report.reasons), file=sys.stderr)
        return 2
    print("PD gravity prepose plan (dry run; no command published)")
    print(f"calibration: {cal['source']}")
    print(f"target_rad: {np.round(target, 6).tolist()}")
    print(f"kp: {np.round(kp, 4).tolist()}")
    print(f"kd: {np.round(kd, 4).tolist()}")
    print(f"duration_sec: {duration:.1f}, friction_scale: {friction_scale:.3f}")
    print(f"torque_limit_nm: {TORQUE_LIMIT.tolist()}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target-rad",
        default="-0.169219241,0.325749286,-0.545772770,-1.162966483,0.485442256,0.884270641",
        help="six encoder-space target angles, comma separated")
    parser.add_argument("--calibration-yaml", required=True)
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--kp", default=",".join(map(str, DEFAULT_KP)))
    parser.add_argument("--kd", default=",".join(map(str, DEFAULT_KD)))
    parser.add_argument("--friction-scale", type=float, default=0.0,
                        help="candidate stiction feedforward scale; default 0")
    parser.add_argument("--friction-epsilon", type=float, default=0.02)
    parser.add_argument("--friction-error-gain", type=float, default=10.0)
    parser.add_argument("--execute", action="store_true",
                        help="publish torque commands; otherwise only print plan")
    args = parser.parse_args()
    try:
        target = parse_vec(args.target_rad, N_ARM, "--target-rad")
        kp = parse_vec(args.kp, N_ARM, "--kp")
        kd = parse_vec(args.kd, N_ARM, "--kd")
        if args.duration <= 0.0 or args.duration > 600.0:
            raise ValueError("--duration must be in (0, 600]")
    except ValueError as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    if not args.execute:
        return plan(target, args.calibration_yaml, kp, kd,
                    args.duration, args.friction_scale)
    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    rclpy.init()
    node = PdGravityPrepose(
        args.calibration_yaml, kp, kd, args.friction_scale,
        args.friction_epsilon, args.friction_error_gain, args.execute)
    try:
        deadline = time.monotonic() + 10.0
        while node.q is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.q is None:
            print("REFUSED: no /joint_states", file=sys.stderr)
            return 3
        if not node.effort_path_enabled():
            print("REFUSED: student_arm_node direct effort is disabled; "
                  "no torque will be sent", file=sys.stderr)
            return 3
        if node.last_feedback is None:
            print("REFUSED: no fresh joint feedback", file=sys.stderr)
            return 3
        if not np.all(np.isfinite(node.q)):
            print("REFUSED: invalid joint feedback", file=sys.stderr)
            return 3
        # The large displacement is intentionally divided into a quintic
        # trajectory; check the target here and enforce limits on every tick.
        report = check_setpoint(target, node.model)
        if not report.ok:
            print("REFUSED target: " + "; ".join(report.reasons), file=sys.stderr)
            return 4
        start = node.q.copy()
        print("EXECUTE: torque prepose started from "
              f"{np.round(start, 6).tolist()}", flush=True)
        start_time = time.monotonic()
        next_tick = start_time
        while True:
            now = time.monotonic()
            elapsed = now - start_time
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.last_feedback is None or now - node.last_feedback > FEEDBACK_TIMEOUT:
                raise RuntimeError("joint feedback stale")
            if node.qd is not None and np.max(np.abs(node.qd)) > SPEED_LIMIT:
                raise RuntimeError("joint speed exceeded %.3f rad/s" % SPEED_LIMIT)
            if np.any(node.q < JOINT_LOWER) or np.any(node.q > JOINT_UPPER):
                raise RuntimeError("joint limit exceeded")
            if elapsed >= args.duration:
                break
            s, ds_du = quintic(elapsed / args.duration)
            q_des = start + s * (target - start)
            qd_des = ds_du * (target - start) / args.duration
            qd_actual = node.qd if node.qd is not None else np.zeros(N_ARM)
            tau = (node.gravity(node.q)
                   + node.kp * (q_des - node.q)
                   + node.kd * (qd_des - qd_actual))
            if node.friction.enabled:
                tau += stiction_feedforward(
                    node.friction, qd_des, q_des - node.q)
            tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
            max_step = DEFAULT_TAU_RATE / RATE_HZ
            tau = node.last_tau + np.clip(tau - node.last_tau, -max_step, max_step)
            node.last_tau = tau
            node.publish_tau(tau)
            while next_tick <= now:
                next_tick += 1.0 / RATE_HZ
            time.sleep(max(0.0, next_tick - time.monotonic()))
        # Hold the final pose with gravity + PD for a short validation window.
        hold_until = time.monotonic() + 5.0
        while time.monotonic() < hold_until:
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.last_feedback is None or time.monotonic() - node.last_feedback > FEEDBACK_TIMEOUT:
                raise RuntimeError("joint feedback stale during hold")
            qd_actual = node.qd if node.qd is not None else np.zeros(N_ARM)
            tau = node.gravity(node.q) + node.kp * (target - node.q) - node.kd * qd_actual
            if node.friction.enabled:
                tau += stiction_feedforward(
                    node.friction, np.zeros(N_ARM), target - node.q)
            tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
            node.publish_tau(tau)
            time.sleep(1.0 / RATE_HZ)
        error_deg = np.degrees(node.q - target)
        print("DONE: q_deg=%s error_deg=%s max_error=%.2f"
              % (np.round(np.degrees(node.q), 2).tolist(),
                 np.round(error_deg, 2).tolist(),
                 float(np.max(np.abs(error_deg)))), flush=True)
        return 0 if np.max(np.abs(error_deg)) < 3.0 else 5
    except KeyboardInterrupt:
        print("STOPPED by Ctrl-C", file=sys.stderr)
        return 130
    except RuntimeError as exc:
        print(f"ABORTED: {exc}", file=sys.stderr)
        return 6
    finally:
        if node.execute:
            node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
