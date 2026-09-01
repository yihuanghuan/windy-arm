#!/usr/bin/env python3
"""Low-speed bidirectional constant-velocity residual test for selected joints.

This freezes the supplied arm6 calibration YAML.  It commands slow +/- constant
velocity sweeps around one fixed posture, records q/qdot/effort, computes
Pinocchio RNEA with the frozen zero and gravity ratio, and reports
tau_measured - tau_rnea by direction.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from dummy_interface.msg import MotorControl
except ImportError as exc:
    rclpy = None
    _IMPORT_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.model import load_arm6_calibration
from task_space_smc.hw.fk_gravity_verify import (
    GravityModel, N_ARM, JOINT_LOWER, JOINT_UPPER, check_setpoint,
)
from task_space_smc.hw.phase4_gravity_compensation import (
    BASE_D, BASE_P, desk_margin,
)

OUT = Path(__file__).resolve().parent / "data" / "phase5_constvel"
DEFAULT_POSTURE = "0.0387,0.8207,-0.6804,2.1650,1.3398,1.4196"
DESK_MARGIN_MIN = 0.10
PUBLISH_RATE = 100.0


def parse_vec(text: str, n: int = N_ARM) -> np.ndarray:
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != n:
        raise ValueError(f"expected {n} comma separated values")
    return np.array(vals, dtype=float)


def rnea_scaled(model: GravityModel, q_model: np.ndarray, qd: np.ndarray,
                qdd: np.ndarray, ratio: np.ndarray) -> np.ndarray:
    tau = pin.rnea(model.model, model.data, q_model, qd, qdd).copy()
    g = model.gravity(q_model)
    return (tau - g) + ratio * g


class ConstVelNode(Node):
    def __init__(self, calibration_yaml: str):
        super().__init__("phase5_constant_velocity_residual")
        self.model = GravityModel()
        cal = load_arm6_calibration(calibration_yaml)
        self.offset = np.asarray(cal["zero_offset_rad"], dtype=float)
        self.ratio = np.asarray(cal["first_moment_ratio"], dtype=float)
        self.cal_source = cal["source"]
        self.q = None
        self.v = None
        self.tau = None
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb, 50)
        self.get_logger().info(f"calibration YAML: {self.cal_source}")
        self.get_logger().info(
            f"zero_offset_deg: {np.round(np.degrees(self.offset), 3).tolist()}")
        self.get_logger().info(
            f"first_moment_ratio: {np.round(self.ratio, 4).tolist()}")

    def _cb(self, msg):
        if len(msg.position) == N_ARM:
            self.q = np.asarray(msg.position, dtype=float)
        if len(msg.velocity) == N_ARM:
            self.v = np.asarray(msg.velocity, dtype=float)
        if len(msg.effort) == N_ARM:
            self.tau = np.asarray(msg.effort, dtype=float)

    def feedforward(self, q):
        return self.model.gravity(np.asarray(q, dtype=float) + self.offset) * self.ratio

    def send(self, q_cmd, qd_cmd=None, kp=None, kd=None, tau_ff=None):
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(x) for x in q_cmd]
        msg.velocity = [0.0] * N_ARM if qd_cmd is None else [float(x) for x in qd_cmd]
        msg.current = [float(x) for x in (self.feedforward(self.q if self.q is not None else q_cmd)
                                          if tau_ff is None else tau_ff)]
        msg.p = [float(x) for x in (BASE_P if kp is None else kp)]
        msg.d = [float(x) for x in (BASE_D if kd is None else kd)]
        msg.enable = 1
        self.pub.publish(msg)

    def wait_state(self, timeout=10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and (self.q is None or self.v is None or self.tau is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None and self.v is not None and self.tau is not None

    def check_effort_enabled(self, timeout=5.0) -> str | None:
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters, "/student_arm_node/get_parameters")
        if not cli.wait_for_service(timeout_sec=timeout):
            return "student_arm_node parameter service unavailable"
        req = GetParameters.Request()
        req.names = ["enable_direct_effort_command"]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None or not fut.result().values:
            return "could not read enable_direct_effort_command"
        if not fut.result().values[0].bool_value:
            return "student_arm_node is not in direct effort mode"
        return None

    def safety_reason(self, q) -> str | None:
        q = np.asarray(q, dtype=float)
        rep = check_setpoint(q, self.model)
        if not rep.ok:
            return "; ".join(rep.reasons)
        mg = desk_margin(self.model, q)
        if mg < DESK_MARGIN_MIN:
            return f"desk/fence margin {mg:+.4f}"
        lim = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)
        if lim.min() < 0.08:
            return f"joint{int(np.argmin(lim)) + 1} within 0.08 rad of a limit"
        return None

    def hard_safety_reason(self, q) -> str | None:
        q = np.asarray(q, dtype=float)
        mg = desk_margin(self.model, q)
        if mg < 0.0:
            return f"desk/fence margin {mg:+.4f}"
        lim = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)
        if lim.min() < 0.05:
            return f"joint{int(np.argmin(lim)) + 1} within 0.05 rad of a limit"
        return None

    def move_to(self, q_target, timeout=40.0) -> str | None:
        q_target = np.asarray(q_target, dtype=float)
        why = self.safety_reason(q_target)
        if why:
            return f"target unsafe: {why}"
        if self.q is None:
            return "no current state"
        q_start = self.q.copy()
        dist = float(np.max(np.abs(q_target - q_start)))
        ramp_sec = max(4.0, dist / 0.16)
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < timeout:
            now = time.time()
            alpha = min(1.0, (now - t0) / ramp_sec)
            q_cmd = q_start + alpha * (q_target - q_start)
            if now >= nxt:
                self.send(q_cmd)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)
            if self.q is None:
                continue
            why = self.hard_safety_reason(self.q)
            if why:
                return why
            if np.max(np.abs(self.q - q_target)) < 0.06:
                return None
        return f"move_to timed out {np.max(np.abs(q_target - self.q)):.3f} rad away"

    def hold(self, q_hold, seconds=2.0):
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= nxt:
                self.send(q_hold)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)

    def sweep(self, center: np.ndarray, joint: int, direction: int,
              speed: float, amplitude: float, profile: str) -> dict:
        idx = joint - 1
        start = center.copy()
        end = center.copy()
        start[idx] -= direction * amplitude * 0.5
        end[idx] += direction * amplitude * 0.5
        for q in (start, end):
            why = self.safety_reason(q)
            if why:
                return dict(ok=False, joint=joint, direction=direction,
                            speed=speed, profile=profile, reason=f"unsafe endpoint: {why}")
        why = self.move_to(start)
        if why:
            return dict(ok=False, joint=joint, direction=direction,
                        speed=speed, profile=profile, reason=f"move_to start failed: {why}")
        self.hold(start, 1.5)

        duration = amplitude / max(abs(speed), 1e-6)
        t0 = time.time()
        nxt = t0
        rows = []
        while time.time() - t0 < duration:
            now = time.time()
            s = min(1.0, (now - t0) / duration)
            q_cmd = start + s * (end - start)
            qd_cmd = np.zeros(N_ARM)
            qd_cmd[idx] = direction * abs(speed)
            if now >= nxt:
                self.send(q_cmd, qd_cmd=qd_cmd,
                          kp=np.asarray(BASE_P) * 0.8,
                          kd=np.asarray(BASE_D) * 1.0)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.001)
            if self.q is None or self.v is None or self.tau is None:
                continue
            why = self.hard_safety_reason(self.q)
            if why:
                return dict(ok=False, joint=joint, direction=direction,
                            speed=speed, profile=profile, reason=f"aborted: {why}", rows=rows)
            rows.append(dict(t=float(now - t0),
                             q=self.q.tolist(),
                             qd=self.v.tolist(),
                             tau=self.tau.tolist(),
                             q_cmd=q_cmd.tolist(),
                             qd_cmd=qd_cmd.tolist()))
        self.hold(end, 1.0)
        return dict(ok=True, joint=joint, direction=direction, speed=speed,
                    profile=profile, amplitude=amplitude, rows=rows)


def analyze_run(model: GravityModel, offset: np.ndarray, ratio: np.ndarray,
                run: dict, trim_frac: float) -> dict:
    rows = run.get("rows", [])
    n = len(rows)
    if n < 8:
        return dict(ok=False, reason=f"only {n} rows")
    q = np.asarray([r["q"] for r in rows], dtype=float)
    qd = np.asarray([r["qd"] for r in rows], dtype=float)
    tau = np.asarray([r["tau"] for r in rows], dtype=float)
    t = np.asarray([r["t"] for r in rows], dtype=float)
    qdd = np.gradient(qd, t, axis=0, edge_order=1)
    k0 = int(trim_frac * n)
    k1 = max(k0 + 3, int((1.0 - trim_frac) * n))
    idx = int(run["joint"]) - 1
    residuals = []
    for qi, vi, ai, yi in zip(q[k0:k1], qd[k0:k1], qdd[k0:k1], tau[k0:k1]):
        pred = rnea_scaled(model, qi + offset, vi, ai, ratio)
        residuals.append(yi - pred)
    residuals = np.asarray(residuals)
    return dict(ok=True,
                n_total=n,
                n_used=int(k1 - k0),
                qd_mean=float(np.mean(qd[k0:k1, idx])),
                tau_meas_mean=float(np.mean(tau[k0:k1, idx])),
                residual_mean=float(np.mean(residuals[:, idx])),
                residual_std=float(np.std(residuals[:, idx])),
                residual_abs_max=float(np.max(np.abs(residuals[:, idx]))),
                residual_all_mean=residuals.mean(axis=0).tolist())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--calibration-yaml", default=str(Path(__file__).resolve().parents[2] / "config" / "arm6_calibration_candidate.yaml"))
    ap.add_argument("--posture", default=DEFAULT_POSTURE)
    ap.add_argument("--joints", default="2,4")
    ap.add_argument("--speeds", default="0.025,0.045")
    ap.add_argument("--amplitude", type=float, default=0.08)
    ap.add_argument("--trim-frac", type=float, default=0.25)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2
    joints = [int(x) for x in args.joints.split(",") if x.strip()]
    speeds = [float(x) for x in args.speeds.split(",") if x.strip()]
    center = parse_vec(args.posture)

    model = GravityModel()
    cal = load_arm6_calibration(args.calibration_yaml)
    offset = np.asarray(cal["zero_offset_rad"], dtype=float)
    ratio = np.asarray(cal["first_moment_ratio"], dtype=float)
    for j in joints:
        for direction in (-1, 1):
            q0 = center.copy()
            q1 = center.copy()
            q0[j - 1] -= direction * args.amplitude * 0.5
            q1[j - 1] += direction * args.amplitude * 0.5
            for q in (q0, q1):
                rep = check_setpoint(q, model)
                if not rep.ok or desk_margin(model, q) < DESK_MARGIN_MIN:
                    print(f"REFUSED: joint{j} endpoint unsafe {np.round(q, 4).tolist()} {rep.reasons} margin={desk_margin(model, q):+.4f}", file=sys.stderr)
                    return 3
    if args.dry_run:
        print("PLAN only")
        print(f"calibration: {cal['source']}")
        print(f"posture: {center.tolist()}")
        print(f"joints: {joints}, speeds: {speeds}, amplitude: {args.amplitude}")
        return 0

    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    rclpy.init()
    node = ConstVelNode(args.calibration_yaml)
    runs = []
    try:
        if not node.wait_state():
            print("no /joint_states", file=sys.stderr)
            return 4
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 5
        why = node.move_to(center)
        if why:
            print(f"REFUSED: could not reach center: {why}", file=sys.stderr)
            return 6
        node.hold(center, 2.0)
        for si, speed in enumerate(speeds):
            for joint in joints:
                for direction in (1, -1):
                    node.get_logger().info(
                        f"===== profile{si} joint{joint} dir {direction:+d} speed {speed:.3f} =====")
                    run = node.sweep(center, joint, direction, speed, args.amplitude,
                                     profile=f"speed{si}")
                    run["analysis"] = analyze_run(model, offset, ratio, run, args.trim_frac) if run.get("ok") else {}
                    runs.append(run)
    finally:
        node.hold(center, 1.0)
        node.destroy_node()
        rclpy.shutdown()

    summary = []
    for run in runs:
        a = run.get("analysis", {})
        summary.append(dict(joint=run.get("joint"), direction=run.get("direction"),
                            profile=run.get("profile"), speed=run.get("speed"),
                            ok=run.get("ok"), reason=run.get("reason"),
                            residual_mean=a.get("residual_mean"),
                            residual_std=a.get("residual_std"),
                            qd_mean=a.get("qd_mean"),
                            tau_meas_mean=a.get("tau_meas_mean"),
                            n_used=a.get("n_used")))
    path = OUT / f"constvel_j24_{stamp}.json"
    path.write_text(json.dumps(dict(calibration=args.calibration_yaml,
                                    posture=center.tolist(),
                                    zero_offset_rad=offset.tolist(),
                                    first_moment_ratio=ratio.tolist(),
                                    runs=runs,
                                    summary=summary),
                               indent=2, ensure_ascii=False))
    csv_path = OUT / f"constvel_j24_{stamp}_summary.csv"
    with csv_path.open("w", newline="") as fh:
        fields = ["joint", "direction", "profile", "speed", "ok", "reason",
                  "qd_mean", "tau_meas_mean", "residual_mean", "residual_std", "n_used"]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary)
    print(f"\nresults -> {path}")
    print(f"summary_csv -> {csv_path}")
    print("\njoint dir profile speed qd_mean residual_mean residual_std")
    for row in summary:
        print(f"J{row['joint']} {row['direction']:+d} {row['profile']} {row['speed']:.3f} "
              f"{row.get('qd_mean')} {row.get('residual_mean')} {row.get('residual_std')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
