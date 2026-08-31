#!/usr/bin/env python3
"""J5 LESO observer capture and offline parameter tune.

Scope is intentionally narrow:

  * J1/J2/J3 calibration values are frozen from the selected YAML.
  * Only J5 zero offset, J5 gravity ratio, and J5 directional friction F+/F-
    are fitted offline.
  * The existing state_estimator / JointLESO stack is used, but the LESO
    disturbance estimate is logged only.  It is never fed back as compensation.
  * Hardware motion is J5-only dynamic sweeps at several safe upstream
    postures.

The script never writes candidate or official YAML.
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
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from dummy_interface.msg import MotorControl
except ImportError as exc:
    rclpy = None
    _RCLPY_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.model import load_arm6_calibration  # noqa: E402
from task_space_smc.core.state_estimator import resolve_estimator  # noqa: E402
from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, JOINT_LOWER, JOINT_UPPER, N_ARM, RATED_TORQUE, SAFE_WAYPOINT,
    check_setpoint, interpolate,
)
from task_space_smc.hw.phase3_dynamic_zero_calibration import safety_reasons  # noqa: E402


HERE = Path(__file__).resolve()
DEFAULT_CANDIDATE = HERE.parents[2] / "config" / "arm6_calibration_candidate.yaml"
OUT = HERE.parent / "data" / "phase3"

J5 = 4
PUBLISH_RATE = 100.0
DEFAULT_FIT_RATE = 100.0
DEFAULT_DURATION = 8.0
DEFAULT_HOLD_SEC = 1.5

DEFAULT_POSTURES = (
    "-0.16,0.72,0.18,-0.55,0.0,0.50;"
    "-0.16,0.88,0.30,-0.55,0.0,0.50;"
    "-0.16,1.04,0.42,-0.55,0.0,0.50"
)

J5_SWEEP_LO = -0.70
J5_SWEEP_HI = 0.70

TRACK_P = np.array([24.0, 24.0, 24.0, 5.0, 5.0, 5.0])
TRACK_D = np.array([1.1, 1.1, 1.1, 0.20, 0.20, 0.20])
GOTO_P = np.array([30.0, 30.0, 30.0, 5.0, 5.0, 5.0])
GOTO_D = np.array([1.1, 1.1, 1.1, 0.20, 0.20, 0.20])

MAX_CMD_VEL = np.array([0.18, 0.18, 0.18, 0.20, 0.38, 0.20])
MAX_CMD_ACC = np.array([0.50, 0.50, 0.50, 0.50, 0.28, 0.50])
MAX_MEAS_SPEED = np.array([0.70, 0.90, 0.90, 0.60, 0.80, 0.80])
MAX_TRACK_ERR = 0.28
GOTO_ACCEPT_ERR = 0.08
SPEED_ABORT_DWELL_SEC = 0.15
TRANSIT_MAX_VEL = np.array([0.18, 0.18, 0.18, 0.20, 0.18, 0.20])
TRANSIT_MIN_SEC = 4.0
MIN_LIMIT_MARGIN = 0.12


def parse_vec(text: str) -> np.ndarray:
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != N_ARM:
        raise ValueError(f"expected {N_ARM} comma-separated values")
    return np.asarray(vals, dtype=float)


def parse_postures(text: str) -> list[np.ndarray]:
    return [parse_vec(chunk) for chunk in text.split(";") if chunk.strip()]


def apply_armature(model: GravityModel, cal: dict) -> None:
    arm = cal.get("armature_kgm2")
    if arm is None:
        return
    model.model.armature[:] = np.asarray(
        [0.0 if x is None else float(x) for x in arm], dtype=float)
    model.data = model.model.createData()


def rnea_scaled(model: GravityModel, q_model: np.ndarray, qd: np.ndarray,
                qdd: np.ndarray, ratio: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    tau = pin.rnea(model.model, model.data, q_model, qd, qdd).copy()
    g = model.gravity(q_model)
    return (tau - g) + ratio * g, g


def gravity_scaled(model: GravityModel, q_encoder: np.ndarray,
                   zero: np.ndarray, ratio: np.ndarray) -> np.ndarray:
    return model.gravity(np.asarray(q_encoder, dtype=float) + zero) * ratio


def hard_safety_reasons(model: GravityModel, q: np.ndarray) -> list[str]:
    reasons = safety_reasons(model, q)
    lim = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)
    if np.min(lim) < MIN_LIMIT_MARGIN:
        reasons.append(
            f"joint{int(np.argmin(lim)) + 1} limit margin {np.min(lim):.3f} rad")
    return reasons


def sweep_trajectory(center: np.ndarray, duration: float, rate: float,
                     direction: str, lo: float, hi: float) -> dict:
    dt = 1.0 / rate
    t = np.arange(0.0, duration + 0.5 * dt, dt)
    x = np.clip(t / duration, 0.0, 1.0)
    h = x * x * x * (10.0 + x * (-15.0 + 6.0 * x))
    hd = 30.0 * x * x * (1.0 - x) * (1.0 - x) / duration
    hdd = (60.0 * x * (1.0 - x) * (1.0 - 2.0 * x)) / (duration * duration)
    q0, q1 = (lo, hi) if direction == "forward" else (hi, lo)
    dq = q1 - q0
    q = np.tile(center, (t.size, 1))
    q[:, J5] = q0 + dq * h
    qd = np.zeros_like(q)
    qdd = np.zeros_like(q)
    qd[:, J5] = dq * hd
    qdd[:, J5] = dq * hdd
    qd[0, :] = 0.0
    qd[-1, :] = 0.0
    qdd[0, :] = 0.0
    qdd[-1, :] = 0.0
    return dict(t=t, q=q, qd=qd, qdd=qdd)


def check_path(model: GravityModel, qa: np.ndarray, qb: np.ndarray) -> list[str]:
    for q in interpolate(qa, qb):
        reasons = hard_safety_reasons(model, q)
        if reasons:
            return reasons
    return []


def check_traj(model: GravityModel, tr: dict) -> dict:
    bad = []
    for i, q in enumerate(tr["q"]):
        reasons = hard_safety_reasons(model, q)
        if reasons:
            bad.append(dict(index=i, t=float(tr["t"][i]), reasons=reasons))
        v_over = np.abs(tr["qd"][i]) - MAX_CMD_VEL
        if np.max(v_over) > 1e-9:
            j = int(np.argmax(v_over))
            bad.append(dict(index=i, t=float(tr["t"][i]),
                            reasons=[f"cmd velocity J{j + 1} "
                                     f"{abs(tr['qd'][i, j]):.3f} rad/s"]))
        a_over = np.abs(tr["qdd"][i]) - MAX_CMD_ACC
        if np.max(a_over) > 1e-9:
            j = int(np.argmax(a_over))
            bad.append(dict(index=i, t=float(tr["t"][i]),
                            reasons=[f"cmd acceleration J{j + 1} "
                                     f"{abs(tr['qdd'][i, j]):.3f} rad/s^2"]))
    return dict(ok=not bad, violation_count=len(bad), violations=bad[:20])


def plan_summary(model: GravityModel, postures: list[np.ndarray],
                 duration: float, rate: float, repeats: int,
                 lo: float, hi: float) -> dict:
    ok = True
    segments = []
    transit_bad = []
    all_q = []
    all_qd = []
    all_qdd = []
    prev = SAFE_WAYPOINT.copy()
    for idx, p in enumerate(postures):
        p = p.copy()
        p[J5] = lo
        reasons = check_path(model, prev, p)
        if reasons:
            ok = False
            transit_bad.append(dict(to_posture=idx, reasons=reasons))
        prev = p
    for rep in range(repeats):
        for idx, center in enumerate(postures):
            for direction in ("forward", "reverse"):
                tr = sweep_trajectory(center, duration, rate, direction, lo, hi)
                rep_safety = check_traj(model, tr)
                ok = ok and rep_safety["ok"]
                all_q.append(tr["q"])
                all_qd.append(tr["qd"])
                all_qdd.append(tr["qdd"])
                segments.append(dict(
                    repeat=rep,
                    posture_index=idx,
                    direction=direction,
                    safety=rep_safety,
                    q_min_rad=np.min(tr["q"], axis=0).tolist(),
                    q_max_rad=np.max(tr["q"], axis=0).tolist(),
                    max_abs_cmd_vel=np.max(np.abs(tr["qd"]), axis=0).tolist(),
                    max_abs_cmd_acc=np.max(np.abs(tr["qdd"]), axis=0).tolist(),
                ))
    q = np.vstack(all_q)
    qd = np.vstack(all_qd)
    qdd = np.vstack(all_qdd)
    nseg = repeats * len(postures) * 2
    return dict(
        ok=ok,
        writes_yaml=False,
        motion="J5 only forward/reverse sweeps",
        fitted_parameters="J5 zero_offset, first_moment_ratio, F+, F- only",
        frozen_parameters="J1/J2/J3 zero and ratio from calibration YAML",
        repeats=int(repeats),
        posture_count=len(postures),
        dynamic_segment_count=int(nseg),
        dynamic_sec=float(nseg * duration),
        estimated_total_minutes=float(
            (nseg * duration + (nseg + len(postures)) * DEFAULT_HOLD_SEC + 60.0) / 60.0),
        transit_safety=dict(ok=not transit_bad, violations=transit_bad[:20]),
        q_min_rad=np.min(q, axis=0).tolist(),
        q_max_rad=np.max(q, axis=0).tolist(),
        max_abs_cmd_vel=np.max(np.abs(qd), axis=0).tolist(),
        max_abs_cmd_acc=np.max(np.abs(qdd), axis=0).tolist(),
        max_abs_rated_torque_fraction=np.max(np.abs([
            model.gravity(qi) / RATED_TORQUE
            for qi in q[::max(1, len(q) // 200)]
        ]), axis=0).tolist(),
        segments=segments,
    )


class J5LesoNode(Node):
    def __init__(self, calibration_yaml: str, estimator_cfg: dict,
                 control_period: float, control_zero: np.ndarray,
                 control_ratio: np.ndarray, control_friction_pos: float,
                 control_friction_neg: float, control_friction_eps: float,
                 apply_control_friction: bool):
        super().__init__("phase3_j5_leso_observer_tune")
        self.model = GravityModel()
        self.cal = load_arm6_calibration(calibration_yaml)
        apply_armature(self.model, self.cal)
        self.zero = np.asarray(control_zero, dtype=float)
        self.ratio = np.asarray(control_ratio, dtype=float)
        self.control_friction_pos = float(control_friction_pos)
        self.control_friction_neg = float(control_friction_neg)
        self.control_friction_eps = float(control_friction_eps)
        self.apply_control_friction = bool(apply_control_friction)
        plan = resolve_estimator(
            estimator_cfg, N_ARM, control_period, mode="hardware",
            feedback_delay_enabled=False)
        self.estimator = plan.estimator
        self.estimator_messages = list(plan.messages)
        self.estimator_warnings = list(plan.warnings)
        self.q = None
        self.qd = None
        self.tau = None
        self.records: list[dict] = []
        self.segment = None
        self.t0_segment = None
        self.q_cmd = None
        self.qd_cmd = None
        self.qdd_cmd = None
        self.speed_violation_since = None
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb, 100)
        self.get_logger().info(f"calibration YAML: {self.cal['source']}")
        for msg in self.estimator_messages:
            self.get_logger().info(msg)
        for msg in self.estimator_warnings:
            self.get_logger().warning(msg)

    def _cb(self, msg: JointState):
        now = time.time()
        if len(msg.position) == N_ARM:
            self.q = np.asarray(msg.position, dtype=float)
        if len(msg.velocity) == N_ARM:
            self.qd = np.asarray(msg.velocity, dtype=float)
        if len(msg.effort) == N_ARM:
            self.tau = np.asarray(msg.effort, dtype=float)
        if self.q is None or self.qd is None:
            return
        q_est, qd_est = self.estimator(self.q, self.qd, 1.0 / PUBLISH_RATE)
        if self.segment and self.tau is not None:
            self.records.append(dict(
                segment=self.segment,
                t_wall=now,
                t_segment=now - float(self.t0_segment),
                q=self.q.tolist(),
                qdot_msg=self.qd.tolist(),
                tau_meas=self.tau.tolist(),
                q_est=np.asarray(q_est, dtype=float).tolist(),
                qdot_est=np.asarray(qd_est, dtype=float).tolist(),
                leso_position=self.estimator.leso.position_estimate.tolist(),
                leso_velocity=self.estimator.leso.velocity_estimate.tolist(),
                leso_disturbance=self.estimator.leso.disturbance_estimate.tolist(),
                q_cmd=(self.q_cmd.tolist() if self.q_cmd is not None else None),
                qdot_cmd=(self.qd_cmd.tolist() if self.qd_cmd is not None else None),
                qddot_cmd=(self.qdd_cmd.tolist() if self.qdd_cmd is not None else None),
            ))

    def wait_state(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and (
                self.q is None or self.qd is None or self.tau is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None and self.qd is not None and self.tau is not None

    def check_effort_enabled(self, timeout: float = 5.0) -> str | None:
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
            return "student_arm_node direct effort command is disabled"
        return None

    def send(self, q: np.ndarray, qd: np.ndarray | None = None,
             qdd: np.ndarray | None = None, p=None, d=None):
        q = np.asarray(q, dtype=float)
        qd = np.zeros(N_ARM) if qd is None else np.asarray(qd, dtype=float)
        qdd = np.zeros(N_ARM) if qdd is None else np.asarray(qdd, dtype=float)
        self.q_cmd = q.copy()
        self.qd_cmd = qd.copy()
        self.qdd_cmd = qdd.copy()
        self.estimator.push_command(qdd, 1.0 / PUBLISH_RATE)
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in q]
        msg.velocity = [float(v) for v in qd]
        current = gravity_scaled(self.model, q, self.zero, self.ratio)
        if self.apply_control_friction:
            mag = self.control_friction_pos if qd[J5] >= 0.0 else self.control_friction_neg
            current[J5] += mag * np.tanh(qd[J5] / self.control_friction_eps)
        msg.current = [float(v) for v in current]
        msg.p = [float(v) for v in (TRACK_P if p is None else p)]
        msg.d = [float(v) for v in (TRACK_D if d is None else d)]
        msg.enable = 1
        self.pub.publish(msg)

    def guard(self, q_cmd=None) -> str | None:
        if self.q is None:
            return None
        reasons = hard_safety_reasons(self.model, self.q)
        if reasons:
            return "measured pose unsafe: " + reasons[0]
        if self.qd is not None:
            over = np.abs(self.qd) - MAX_MEAS_SPEED
            if np.max(over) > 0.0:
                now = time.time()
                if self.speed_violation_since is None:
                    self.speed_violation_since = now
                if now - self.speed_violation_since >= SPEED_ABORT_DWELL_SEC:
                    j = int(np.argmax(over))
                    return (
                        f"measured speed joint{j + 1} "
                        f"{abs(self.qd[j]):.3f} rad/s exceeded for "
                        f"{now - self.speed_violation_since:.2f}s")
            else:
                self.speed_violation_since = None
        if q_cmd is not None:
            err = np.abs(self.q - np.asarray(q_cmd, dtype=float))
            if np.max(err) > MAX_TRACK_ERR:
                return (
                    f"tracking error joint{int(np.argmax(err)) + 1} "
                    f"{np.max(err):.3f} rad exceeded")
        return None

    def goto(self, q_target: np.ndarray, timeout: float = 90.0) -> str | None:
        q_target = np.asarray(q_target, dtype=float)
        if self.q is None:
            return "no measured state"
        q_start = self.q.copy()
        span = np.abs(q_target - q_start)
        travel_sec = max(TRANSIT_MIN_SEC, float(np.max(span / TRANSIT_MAX_VEL)))
        travel_sec = min(travel_sec, timeout)
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < travel_sec:
            now = time.time()
            s = np.clip((now - t0) / travel_sec, 0.0, 1.0)
            h = s * s * (3.0 - 2.0 * s)
            hd = 6.0 * s * (1.0 - s) / travel_sec
            q_cmd = q_start + (q_target - q_start) * h
            qd_cmd = (q_target - q_start) * hd
            why = self.guard(q_cmd)
            if why:
                return why
            if now >= nxt:
                reasons = hard_safety_reasons(self.model, q_cmd)
                if reasons:
                    return "commanded transit unsafe: " + reasons[0]
                self.send(q_cmd, qd_cmd, np.zeros(N_ARM), GOTO_P, GOTO_D)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)
        t_end = time.time()
        while time.time() - t_end < 2.0:
            self.send(q_target, np.zeros(N_ARM), np.zeros(N_ARM), GOTO_P, GOTO_D)
            rclpy.spin_once(self, timeout_sec=0.002)
            if self.q is not None and np.max(np.abs(self.q - q_target)) < GOTO_ACCEPT_ERR:
                return None
        return f"goto timed out {np.max(np.abs(q_target - self.q)):.3f} rad away"

    def hold(self, q_hold: np.ndarray, seconds: float):
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= nxt:
                self.send(q_hold, np.zeros(N_ARM), np.zeros(N_ARM), GOTO_P, GOTO_D)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)

    def run_segment(self, name: str, tr: dict) -> str | None:
        self.segment = name
        self.t0_segment = time.time()
        t0 = time.time()
        t_plan = tr["t"]
        q_plan = tr["q"]
        qd_plan = tr["qd"]
        qdd_plan = tr["qdd"]
        k = 0
        while k < len(t_plan):
            now = time.time()
            while k + 1 < len(t_plan) and t_plan[k + 1] <= now - t0:
                k += 1
            q_cmd = q_plan[k]
            why = self.guard(q_cmd)
            if why:
                self.segment = None
                return why
            self.send(q_cmd, qd_plan[k], qdd_plan[k], TRACK_P, TRACK_D)
            rclpy.spin_once(self, timeout_sec=0.001)
            delay = t_plan[k] - (time.time() - t0)
            if delay > 0.0:
                time.sleep(min(delay, 0.003))
            if now - t0 > t_plan[-1] + 1.0:
                break
            if k == len(t_plan) - 1 and now - t0 >= t_plan[-1]:
                break
        self.segment = None
        return None


def parse_segment(name: str) -> tuple[int, int, str]:
    parts = name.split("_")
    return int(parts[0].removeprefix("r")), int(parts[1].removeprefix("p")), parts[2]


def process_records(records: list[dict], fit_rate: float,
                    smooth_window_sec: float, trim_sec: float) -> list[dict]:
    grouped: dict[str, list[dict]] = {}
    for row in records:
        grouped.setdefault(row["segment"], []).append(row)
    out = []
    for name, rows in grouped.items():
        rows = sorted(rows, key=lambda r: r["t_wall"])
        if len(rows) < 30:
            continue
        t_raw = np.asarray([r["t_wall"] for r in rows], dtype=float)
        t_raw = t_raw - t_raw[0]
        q_raw = np.asarray([r["q"] for r in rows], dtype=float)
        tau_raw = np.asarray([r["tau_meas"] for r in rows], dtype=float)
        qcmd_raw = np.asarray([r["q_cmd"] for r in rows], dtype=float)
        qdcmd_raw = np.asarray([r["qdot_cmd"] for r in rows], dtype=float)
        qddcmd_raw = np.asarray([r["qddot_cmd"] for r in rows], dtype=float)
        leso_raw = np.asarray([r["leso_disturbance"] for r in rows], dtype=float)
        qd_est_raw = np.asarray([r["qdot_est"] for r in rows], dtype=float)
        unique = np.r_[True, np.diff(t_raw) > 1e-6]
        t_raw = t_raw[unique]
        q_raw = q_raw[unique]
        tau_raw = tau_raw[unique]
        qcmd_raw = qcmd_raw[unique]
        qdcmd_raw = qdcmd_raw[unique]
        qddcmd_raw = qddcmd_raw[unique]
        leso_raw = leso_raw[unique]
        qd_est_raw = qd_est_raw[unique]
        dt = 1.0 / fit_rate
        t = np.arange(t_raw[0], t_raw[-1], dt)
        if t.size < 30:
            continue
        def interp(arr):
            return np.column_stack([
                np.interp(t, t_raw, arr[:, j]) for j in range(N_ARM)])
        q_interp = interp(q_raw)
        tau = interp(tau_raw)
        qcmd = interp(qcmd_raw)
        qdcmd = interp(qdcmd_raw)
        qddcmd = interp(qddcmd_raw)
        leso = interp(leso_raw)
        qd_est = interp(qd_est_raw)
        win = max(7, int(round(smooth_window_sec * fit_rate)))
        if win % 2 == 0:
            win += 1
        win = min(win, len(t) - (1 - len(t) % 2))
        if win < 7:
            continue
        q = savgol_filter(q_interp, win, 3, axis=0, mode="interp")
        qd = savgol_filter(q_interp, win, 3, deriv=1, delta=dt, axis=0, mode="interp")
        qdd = savgol_filter(q_interp, win, 3, deriv=2, delta=dt, axis=0, mode="interp")
        mask = (t >= trim_sec) & (t <= t[-1] - trim_sec)
        rep, posture, direction = parse_segment(name)
        for i in np.flatnonzero(mask):
            out.append(dict(
                segment=name,
                repeat=rep,
                posture_index=posture,
                direction=direction,
                t=float(t[i]),
                q=q[i],
                qdot=qd[i],
                qddot=qdd[i],
                tau=tau[i],
                q_cmd=qcmd[i],
                qdot_cmd=qdcmd[i],
                qddot_cmd=qddcmd[i],
                leso_disturbance=leso[i],
                qdot_est=qd_est[i],
            ))
    return out


def _pack_json(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    raise TypeError(type(obj).__name__)


def fit_j5(model: GravityModel, rows: list[dict], zero: np.ndarray,
           ratio: np.ndarray, friction_eps: float) -> dict:
    if not rows:
        return dict(ok=False, reason="no processed rows")
    tau_all = np.asarray([r["tau"] for r in rows], dtype=float)
    if np.max(np.abs(tau_all[:, J5])) < 1e-6:
        return dict(ok=False, reason="/joint_states.effort J5 is zero")

    def predict(row, q5_zero, ratio5, fp, fn):
        q0 = zero.copy()
        rr = ratio.copy()
        q0[J5] = q5_zero
        rr[J5] = ratio5
        pred, _ = rnea_scaled(model, row["q"] + q0, row["qdot"], row["qddot"], rr)
        vel = float(row["qdot"][J5])
        mag = fp if vel >= 0.0 else fn
        pred[J5] += mag * np.tanh(vel / friction_eps)
        return pred[J5]

    def residual(x):
        q5_zero, ratio5, fp, fn = x
        out = []
        for r in rows:
            out.append(predict(r, q5_zero, ratio5, fp, fn) - r["tau"][J5])
        return np.asarray(out, dtype=float) / 0.02

    x0 = np.array([zero[J5], ratio[J5], 0.075, 0.075], dtype=float)
    lower = np.array([np.radians(-60.0), 0.30, 0.0, 0.0])
    upper = np.array([np.radians(45.0), 1.80, 0.60, 0.60])
    sol = least_squares(
        residual, np.clip(x0, lower, upper), bounds=(lower, upper),
        loss="soft_l1", f_scale=1.0, max_nfev=240)

    def eval_x(x):
        raw = residual(x) * 0.02
        return dict(
            rms_nm=float(np.sqrt(np.mean(raw ** 2))),
            max_abs_nm=float(np.max(np.abs(raw))),
        )

    fit = eval_x(sol.x)
    base = eval_x(x0)
    leso = np.asarray([r["leso_disturbance"][J5] for r in rows], dtype=float)
    track = np.asarray([r["q"][J5] - r["q_cmd"][J5] for r in rows], dtype=float)
    by_group = {}
    for key in ("direction", "posture_index"):
        groups = {}
        for r in rows:
            groups.setdefault(str(r[key]), []).append(r)
        by_group[key] = {}
        for g, rs in groups.items():
            raw = np.asarray([
                predict(r, *sol.x) - r["tau"][J5] for r in rs], dtype=float)
            tr = np.asarray([r["q"][J5] - r["q_cmd"][J5] for r in rs], dtype=float)
            z3 = np.asarray([r["leso_disturbance"][J5] for r in rs], dtype=float)
            by_group[key][g] = dict(
                n=len(rs),
                residual_rms_nm=float(np.sqrt(np.mean(raw ** 2))),
                tracking_rms_rad=float(np.sqrt(np.mean(tr ** 2))),
                leso_z3_rms_rad_s2=float(np.sqrt(np.mean(z3 ** 2))),
            )
    return dict(
        ok=bool(sol.success),
        message=str(sol.message),
        n_samples=len(rows),
        baseline=dict(
            q5_zero_deg=float(np.degrees(x0[0])),
            ratio5=float(x0[1]),
            friction_pos_nm=float(x0[2]),
            friction_neg_nm=float(x0[3]),
            **base,
        ),
        fit=dict(
            q5_zero_rad=float(sol.x[0]),
            q5_zero_deg=float(np.degrees(sol.x[0])),
            ratio5=float(sol.x[1]),
            friction_pos_nm=float(sol.x[2]),
            friction_neg_nm=float(sol.x[3]),
            **fit,
        ),
        improvement_ratio=float(1.0 - fit["rms_nm"] / max(base["rms_nm"], 1e-12)),
        tracking_rms_rad=float(np.sqrt(np.mean(track ** 2))),
        tracking_max_abs_rad=float(np.max(np.abs(track))),
        leso_z3_rms_rad_s2=float(np.sqrt(np.mean(leso ** 2))),
        leso_z3_median_rad_s2=float(np.median(leso)),
        cost=float(sol.cost),
        nfev=int(sol.nfev),
        by_group=by_group,
    )


def write_processed_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        header = ["segment", "repeat", "posture_index", "direction", "t"]
        for prefix in ("q", "qdot", "qddot", "tau", "q_cmd", "qdot_cmd",
                       "qddot_cmd", "qdot_est", "leso_disturbance"):
            header.extend([f"{prefix}_J{j}" for j in range(1, N_ARM + 1)])
        writer.writerow(header)
        for r in rows:
            writer.writerow(
                [r["segment"], r["repeat"], r["posture_index"], r["direction"], r["t"]]
                + r["q"].tolist()
                + r["qdot"].tolist()
                + r["qddot"].tolist()
                + r["tau"].tolist()
                + r["q_cmd"].tolist()
                + r["qdot_cmd"].tolist()
                + r["qddot_cmd"].tolist()
                + r["qdot_est"].tolist()
                + r["leso_disturbance"].tolist()
            )


def run_capture(args, postures, estimator_cfg, control_zero,
                control_ratio) -> tuple[list[dict], dict, list[str], list[str]]:
    if rclpy is None:
        raise RuntimeError(f"rclpy unavailable: {_RCLPY_ERROR}")
    rclpy.init()
    node = J5LesoNode(
        str(args.calibration_yaml), estimator_cfg, 1.0 / PUBLISH_RATE,
        control_zero, control_ratio,
        args.control_j5_friction_pos_nm, args.control_j5_friction_neg_nm,
        args.friction_eps, args.apply_control_j5_friction)
    try:
        if not node.wait_state():
            raise RuntimeError("no /joint_states position/velocity/effort")
        why = node.check_effort_enabled()
        if why:
            raise RuntimeError(why)
        for rep in range(args.repeats):
            for idx, center in enumerate(postures):
                start = center.copy()
                start[J5] = args.j5_lo
                node.get_logger().info(
                    f"J5 LESO repeat {rep} posture {idx}: transit")
                why = node.goto(start)
                if why:
                    raise RuntimeError(f"posture {idx} transit failed: {why}")
                node.hold(start, args.hold_sec)
                for direction in ("forward", "reverse"):
                    name = f"r{rep}_p{idx}_{direction}"
                    node.get_logger().info(f"J5 LESO segment: {name}")
                    tr = sweep_trajectory(
                        center, args.duration, args.rate, direction,
                        args.j5_lo, args.j5_hi)
                    why = node.run_segment(name, tr)
                    if why:
                        raise RuntimeError(f"{name} aborted: {why}")
                    end = center.copy()
                    end[J5] = args.j5_hi if direction == "forward" else args.j5_lo
                    node.hold(end, args.hold_sec)
        raw = list(node.records)
        meta = dict(
            estimator_messages=node.estimator_messages,
            estimator_warnings=node.estimator_warnings,
        )
    finally:
        node.segment = None
        node.destroy_node()
        rclpy.shutdown()
    return raw, meta, [], []


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--calibration-yaml", type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument("--postures", default=DEFAULT_POSTURES)
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    ap.add_argument("--rate", type=float, default=PUBLISH_RATE)
    ap.add_argument("--fit-rate", type=float, default=DEFAULT_FIT_RATE)
    ap.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    ap.add_argument("--smooth-window-sec", type=float, default=0.31)
    ap.add_argument("--trim-sec", type=float, default=0.50)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--j5-lo", type=float, default=J5_SWEEP_LO)
    ap.add_argument("--j5-hi", type=float, default=J5_SWEEP_HI)
    ap.add_argument("--leso-bandwidth-hz", type=float, default=32.0)
    ap.add_argument("--leso-input", choices=["none", "commanded_acceleration"],
                    default="commanded_acceleration")
    ap.add_argument("--leso-input-delay-sec", type=float, default=0.010)
    ap.add_argument("--leso-b0-j5", type=float, default=1.0)
    ap.add_argument("--friction-eps", type=float, default=0.03)
    ap.add_argument("--control-j5-zero-deg", type=float, default=None)
    ap.add_argument("--control-j5-ratio", type=float, default=None)
    ap.add_argument("--control-j5-friction-pos-nm", type=float, default=0.0)
    ap.add_argument("--control-j5-friction-neg-nm", type=float, default=0.0)
    ap.add_argument("--apply-control-j5-friction", action="store_true")
    ap.add_argument("--plan-only", action="store_true")
    args = ap.parse_args()

    postures = parse_postures(args.postures)
    cal = load_arm6_calibration(str(args.calibration_yaml))
    zero = np.asarray(cal["zero_offset_rad"], dtype=float)
    ratio = np.asarray(cal["first_moment_ratio"], dtype=float)
    control_zero = zero.copy()
    control_ratio = ratio.copy()
    if args.control_j5_zero_deg is not None:
        control_zero[J5] = np.radians(float(args.control_j5_zero_deg))
    if args.control_j5_ratio is not None:
        control_ratio[J5] = float(args.control_j5_ratio)
    model = GravityModel()
    apply_armature(model, cal)
    plan = plan_summary(
        model, postures, args.duration, args.rate, args.repeats,
        args.j5_lo, args.j5_hi)
    source = ["motor"] * N_ARM
    source[J5] = "leso"
    bandwidth = [0.0] * N_ARM
    bandwidth[J5] = args.leso_bandwidth_hz
    b0 = [0.0] * N_ARM
    b0[J5] = args.leso_b0_j5
    estimator_cfg = dict(
        source=source,
        position_source="measured",
        predict_velocity_sec=0.0,
        predict_position_sec=0.0,
        leso=dict(
            bandwidth_hz=bandwidth,
            input=args.leso_input,
            b0=b0,
            input_delay_sec=args.leso_input_delay_sec,
        ),
    )
    plan["calibration_yaml"] = str(args.calibration_yaml)
    plan["frozen_zero_offset_deg"] = np.degrees(zero).tolist()
    plan["frozen_first_moment_ratio"] = ratio.tolist()
    plan["control_zero_offset_deg"] = np.degrees(control_zero).tolist()
    plan["control_first_moment_ratio"] = control_ratio.tolist()
    plan["control_j5_friction"] = dict(
        enabled=bool(args.apply_control_j5_friction),
        positive_nm=float(args.control_j5_friction_pos_nm),
        negative_nm=float(args.control_j5_friction_neg_nm),
        epsilon=float(args.friction_eps),
    )
    plan["estimator_cfg"] = estimator_cfg
    plan["armature_kgm2"] = model.model.armature.tolist()
    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return 0 if plan["ok"] else 1
    if not plan["ok"]:
        print("REFUSED: plan is unsafe; run --plan-only for details", file=sys.stderr)
        return 2

    stamp = time.strftime("%Y%m%d_%H%M%S")
    try:
        raw_records, est_meta, _, _ = run_capture(
            args, postures, estimator_cfg, control_zero, control_ratio)
    except RuntimeError as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 3
    processed = process_records(
        raw_records, args.fit_rate, args.smooth_window_sec, args.trim_sec)
    fit = fit_j5(model, processed, zero, ratio, args.friction_eps)

    OUT.mkdir(parents=True, exist_ok=True)
    json_path = OUT / f"phase3_j5_leso_observer_tune_{stamp}.json"
    csv_path = OUT / f"phase3_j5_leso_observer_tune_processed_{stamp}.csv"
    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            writes_yaml=False,
            leso_compensation_closed_loop=False,
            note="LESO disturbance estimate is logged only; MotorControl current uses YAML gravity feedforward plus optional in-script J5 overrides.",
            control_j5_override=dict(
                zero_deg=(None if args.control_j5_zero_deg is None else float(args.control_j5_zero_deg)),
                ratio=args.control_j5_ratio,
                friction_enabled=bool(args.apply_control_j5_friction),
                friction_pos_nm=float(args.control_j5_friction_pos_nm),
                friction_neg_nm=float(args.control_j5_friction_neg_nm),
            ),
            **est_meta,
        ),
        plan=plan,
        raw_records=raw_records,
        processed_sample_count=len(processed),
        fit=fit,
    )
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False, default=_pack_json))
    write_processed_csv(csv_path, processed)
    print(json.dumps(dict(
        result=str(json_path),
        processed_csv=str(csv_path),
        processed_sample_count=len(processed),
        fit=fit,
        writes_yaml=False,
    ), indent=2, ensure_ascii=False, default=_pack_json))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
