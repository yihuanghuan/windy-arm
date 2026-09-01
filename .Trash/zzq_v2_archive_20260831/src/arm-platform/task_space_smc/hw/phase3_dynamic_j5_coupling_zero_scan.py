#!/usr/bin/env python3
"""J5 dynamic coupling zero-offset scan.

This replaces the low-observability J5 Phase3 gravity sweep with a coupled
dynamic experiment:

  * J1/J2/J3 zero offsets and gravity ratios are frozen from a calibration YAML.
  * J5 is held at several safe angles and does not perform large own motion.
  * At each J5 hold angle, J2/J3 run the same forward and reverse dynamic path.
  * The recorded q/qdot/effort is processed into qddot and compared with
    Pinocchio RNEA while scanning only J5 zero offset.

The script never writes YAML.  Use --plan-only before hardware motion and
--input-json to re-scan an existing capture offline.
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
from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, JOINT_LOWER, JOINT_UPPER, N_ARM, RATED_TORQUE, SAFE_WAYPOINT,
    TAU_NOISE, interpolate,
)
from task_space_smc.hw.phase3_dynamic_zero_calibration import (  # noqa: E402
    DEFAULT_FIT_RATE, DEFAULT_HOLD_SEC, DEFAULT_RATE, safety_reasons,
)
HERE = Path(__file__).resolve()
DEFAULT_CANDIDATE = HERE.parents[2] / "config" / "arm6_calibration_candidate.yaml"
OUT = HERE.parent / "data" / "phase3"

J5 = 4
MOVING = np.array([1, 2])
EVAL = np.array([1, 2, 3, 4])

BASE_CENTER = np.array([-0.18, 0.88, 0.30, SAFE_WAYPOINT[3], 0.0, SAFE_WAYPOINT[5]])
DEFAULT_J5_HOLDS = "-0.72,-0.36,0.0,0.36,0.72"
DEFAULT_DURATION = 18.0
EXCITE_AMP = np.array([0.0, 0.28, 0.34, 0.0, 0.0, 0.0])
EXCITE_CYCLES = np.array([0.0, 1.4, 2.3, 0.0, 0.0, 0.0])
EXCITE_PHASE = np.array([0.0, 0.3, 1.6, 0.0, 0.0, 0.0])
EXCITE_PHASE2 = np.array([0.0, 1.9, 0.8, 0.0, 0.0, 0.0])
SECOND_HARMONIC = 0.25

TRACK_P = np.array([22.0, 22.0, 22.0, 4.0, 4.0, 4.0])
TRACK_D = np.array([1.0, 1.0, 1.0, 0.16, 0.16, 0.16])
GOTO_P = np.array([30.0, 30.0, 30.0, 5.0, 5.0, 5.0])
GOTO_D = np.array([1.0, 1.0, 1.0, 0.18, 0.18, 0.18])

MAX_CMD_VEL = np.array([0.20, 0.42, 0.50, 0.08, 0.08, 0.08])
MAX_CMD_ACC = np.array([0.35, 0.95, 1.20, 0.15, 0.15, 0.15])
MAX_MEAS_SPEED = np.array([0.60, 0.90, 0.90, 0.35, 0.35, 0.50])
MIN_LIMIT_MARGIN = 0.12
MAX_TRACK_ERR = 0.24
GOTO_ACCEPT_ERR = 0.08
SPEED_ABORT_DWELL_SEC = 0.15
PUBLISH_RATE = 100.0
TRANSIT_MAX_VEL = np.array([0.18, 0.18, 0.18, 0.20, 0.20, 0.20])
TRANSIT_MIN_SEC = 4.0


def parse_floats(text: str) -> list[float]:
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def parse_scan_deg(text: str) -> np.ndarray:
    parts = [float(x.strip()) for x in text.replace(",", ":").split(":") if x.strip()]
    if len(parts) != 3:
        raise ValueError("--scan-deg must be lo:hi:step")
    lo, hi, step = parts
    if step <= 0.0 or hi < lo:
        raise ValueError("--scan-deg requires hi >= lo and step > 0")
    n = int(np.floor((hi - lo) / step + 0.5))
    vals = lo + step * np.arange(n + 1)
    vals = vals[vals <= hi + 1e-9]
    return np.radians(vals)


def apply_armature(model: GravityModel, cal: dict) -> None:
    arm = cal.get("armature_kgm2")
    if arm is None:
        return
    model.model.armature[:] = np.asarray(
        [0.0 if x is None else float(x) for x in arm], dtype=float)
    model.data = model.model.createData()


def rnea_scaled(model: GravityModel, q_model: np.ndarray, qd: np.ndarray,
                qdd: np.ndarray, ratio: np.ndarray) -> np.ndarray:
    tau = pin.rnea(model.model, model.data, q_model, qd, qdd).copy()
    g = model.gravity(q_model)
    return (tau - g) + ratio * g


def center_for_j5(j5_hold: float, j4_hold: float, j6_hold: float) -> np.ndarray:
    q = BASE_CENTER.copy()
    q[3] = float(j4_hold)
    q[J5] = float(j5_hold)
    q[5] = float(j6_hold)
    return q


def trajectory(center: np.ndarray, duration: float, rate: float,
               reverse: bool = False) -> dict:
    dt = 1.0 / rate
    t = np.arange(0.0, duration + 0.5 * dt, dt)
    x = t / duration
    env = np.sin(np.pi * x) ** 2
    q = np.tile(center, (t.size, 1))
    for j in MOVING:
        s1 = np.sin(2.0 * np.pi * EXCITE_CYCLES[j] * x + EXCITE_PHASE[j])
        s2 = np.sin(2.0 * np.pi * (EXCITE_CYCLES[j] + 1.0) * x
                    + EXCITE_PHASE2[j])
        q[:, j] += EXCITE_AMP[j] * env * (
            s1 + SECOND_HARMONIC * s2) / (1.0 + SECOND_HARMONIC)
    q[:, 0] = center[0]
    q[:, 3] = center[3]
    q[:, 4] = center[4]
    q[:, 5] = center[5]
    qd = np.gradient(q, dt, axis=0, edge_order=2)
    qdd = np.gradient(qd, dt, axis=0, edge_order=2)
    qd[0, :] = 0.0
    qd[-1, :] = 0.0
    qdd[0, :] = 0.0
    qdd[-1, :] = 0.0
    if reverse:
        q = q[::-1].copy()
        qd = -qd[::-1].copy()
        qdd = qdd[::-1].copy()
    return dict(t=t, q=q, qd=qd, qdd=qdd)


def hard_safety_reasons(model: GravityModel, q: np.ndarray) -> list[str]:
    reasons = safety_reasons(model, q)
    lim = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)
    if np.min(lim) < MIN_LIMIT_MARGIN:
        reasons.append(
            f"joint{int(np.argmin(lim)) + 1} limit margin {np.min(lim):.3f} rad")
    return reasons


def check_path(model: GravityModel, qa: np.ndarray, qb: np.ndarray) -> list[str]:
    for q in interpolate(qa, qb):
        reasons = hard_safety_reasons(model, q)
        if reasons:
            return reasons
    return []


def check_trajectory(model: GravityModel, tr: dict) -> dict:
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


def plan_summary(model: GravityModel, j5_holds: list[float],
                 duration: float, rate: float, repeats: int,
                 j4_hold: float, j6_hold: float) -> dict:
    segments = []
    ok = True
    transit_bad = []
    all_q = []
    all_qd = []
    all_qdd = []
    centers = [center_for_j5(v, j4_hold, j6_hold) for v in j5_holds]
    prev = SAFE_WAYPOINT
    for idx, center in enumerate(centers):
        reasons = check_path(model, prev, center)
        if reasons:
            ok = False
            transit_bad.append(dict(to_posture=idx, reasons=reasons))
        prev = center
    for idx, center in enumerate(centers):
        for direction, reverse in (("forward", False), ("reverse", True)):
            tr = trajectory(center, duration, rate, reverse)
            rep = check_trajectory(model, tr)
            ok = ok and rep["ok"]
            all_q.append(tr["q"])
            all_qd.append(tr["qd"])
            all_qdd.append(tr["qdd"])
            segments.append(dict(
                posture_index=idx,
                direction=direction,
                j5_hold_rad=float(center[J5]),
                j5_hold_deg=float(np.degrees(center[J5])),
                safety=rep,
                q_min_rad=np.min(tr["q"], axis=0).tolist(),
                q_max_rad=np.max(tr["q"], axis=0).tolist(),
                max_abs_cmd_vel=np.max(np.abs(tr["qd"]), axis=0).tolist(),
                max_abs_cmd_acc=np.max(np.abs(tr["qdd"]), axis=0).tolist(),
            ))
    q = np.vstack(all_q)
    qd = np.vstack(all_qd)
    qdd = np.vstack(all_qdd)
    total_segments = repeats * len(j5_holds) * 2
    return dict(
        ok=ok,
        writes_yaml=False,
        frozen_zero_joints="J1-J3, J4/J6 baseline only",
        scanned_zero_joint="J5 only",
        gravity_ratio="frozen from calibration YAML; not optimized",
        moving_joints="J2-J3",
        j4_hold_rad=float(j4_hold),
        j4_hold_deg=float(np.degrees(j4_hold)),
        j6_hold_rad=float(j6_hold),
        j6_hold_deg=float(np.degrees(j6_hold)),
        j5_hold_rad=[float(x) for x in j5_holds],
        j5_hold_deg=[float(np.degrees(x)) for x in j5_holds],
        repeats=int(repeats),
        dynamic_segment_count=int(total_segments),
        dynamic_sec=float(total_segments * duration),
        estimated_total_minutes=float(
            (total_segments * duration
             + total_segments * DEFAULT_HOLD_SEC
             + len(j5_holds) * repeats * DEFAULT_HOLD_SEC
             + 70.0) / 60.0),
        transit_safety=dict(ok=not transit_bad, violations=transit_bad[:20]),
        q_min_rad=np.min(q, axis=0).tolist(),
        q_max_rad=np.max(q, axis=0).tolist(),
        max_abs_cmd_vel=np.max(np.abs(qd), axis=0).tolist(),
        max_abs_cmd_acc=np.max(np.abs(qdd), axis=0).tolist(),
        max_abs_gravity_nm=np.max(np.abs([
            model.gravity(qi) for qi in q[::max(1, len(q) // 200)]
        ]), axis=0).tolist(),
        max_abs_rated_torque_fraction=np.max(np.abs([
            model.gravity(qi) / RATED_TORQUE
            for qi in q[::max(1, len(q) // 200)]
        ]), axis=0).tolist(),
        segments=segments,
    )


class J5CouplingNode(Node):
    def __init__(self, calibration_yaml: str):
        super().__init__("phase3_dynamic_j5_coupling_zero_scan")
        self.model = GravityModel()
        self.cal = load_arm6_calibration(calibration_yaml)
        apply_armature(self.model, self.cal)
        self.zero = np.asarray(self.cal["zero_offset_rad"], dtype=float)
        self.ratio = np.asarray(self.cal["first_moment_ratio"], dtype=float)
        self.q = None
        self.qd = None
        self.tau = None
        self.records: list[dict] = []
        self.segment: str | None = None
        self.t0_segment: float | None = None
        self.speed_violation_since: float | None = None
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb, 100)
        self.get_logger().info(f"calibration YAML: {self.cal['source']}")
        self.get_logger().info(
            f"frozen zero_offset_deg: {np.round(np.degrees(self.zero), 4).tolist()}")
        self.get_logger().info(
            f"frozen first_moment_ratio: {np.round(self.ratio, 5).tolist()}")
        self.get_logger().info(
            f"armature_kgm2: {self.model.model.armature.tolist()}")

    def _cb(self, msg: JointState):
        now = time.time()
        if len(msg.position) == N_ARM:
            self.q = np.asarray(msg.position, dtype=float)
        if len(msg.velocity) == N_ARM:
            self.qd = np.asarray(msg.velocity, dtype=float)
        if len(msg.effort) == N_ARM:
            self.tau = np.asarray(msg.effort, dtype=float)
        if self.segment and self.q is not None and self.qd is not None and self.tau is not None:
            self.records.append(dict(
                segment=self.segment,
                t_wall=now,
                t_segment=now - float(self.t0_segment),
                q=self.q.tolist(),
                qdot_msg=self.qd.tolist(),
                tau_meas=self.tau.tolist(),
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

    def gravity_scaled(self, q_encoder: np.ndarray) -> np.ndarray:
        return self.model.gravity(np.asarray(q_encoder, dtype=float) + self.zero) * self.ratio

    def send(self, q: np.ndarray, qd: np.ndarray | None = None,
             p: np.ndarray | None = None, d: np.ndarray | None = None):
        q = np.asarray(q, dtype=float)
        qd = np.zeros(N_ARM) if qd is None else np.asarray(qd, dtype=float)
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in q]
        msg.velocity = [float(v) for v in qd]
        msg.current = [float(v) for v in self.gravity_scaled(q)]
        msg.p = [float(v) for v in (TRACK_P if p is None else p)]
        msg.d = [float(v) for v in (TRACK_D if d is None else d)]
        msg.enable = 1
        self.pub.publish(msg)

    def guard(self, q_cmd: np.ndarray | None = None) -> str | None:
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
            err[[3, 4, 5]] *= 0.6
            if np.max(err) > MAX_TRACK_ERR:
                return (
                    f"tracking error joint{int(np.argmax(err)) + 1} "
                    f"{np.max(err):.3f} rad exceeded")
        return None

    def goto(self, q_target: np.ndarray, timeout: float = 90.0) -> str | None:
        q_target = np.asarray(q_target, dtype=float)
        while self.q is None and timeout > 0.0:
            rclpy.spin_once(self, timeout_sec=0.002)
            timeout -= 0.002
        if self.q is None:
            return "no measured state"
        q_start = self.q.copy()
        span = np.abs(q_target - q_start)
        travel_sec = max(
            TRANSIT_MIN_SEC,
            float(np.max(span / TRANSIT_MAX_VEL)),
        )
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
                self.send(q_cmd, qd_cmd, GOTO_P, GOTO_D)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)
        t_end = time.time()
        while time.time() - t_end < 2.0:
            self.send(q_target, np.zeros(N_ARM), GOTO_P, GOTO_D)
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
                self.send(q_hold, np.zeros(N_ARM), GOTO_P, GOTO_D)
                nxt = now + 1.0 / PUBLISH_RATE
            rclpy.spin_once(self, timeout_sec=0.002)

    def run_segment(self, name: str, tr: dict) -> str | None:
        self.segment = name
        self.t0_segment = time.time()
        t0 = time.time()
        t_plan = tr["t"]
        q_plan = tr["q"]
        qd_plan = tr["qd"]
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
            self.send(q_cmd, qd_plan[k], TRACK_P, TRACK_D)
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
    # r0_p3_forward
    parts = name.split("_")
    repeat = int(parts[0].removeprefix("r"))
    posture = int(parts[1].removeprefix("p"))
    direction = parts[2]
    return repeat, posture, direction


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
        qd_msg_raw = np.asarray([r["qdot_msg"] for r in rows], dtype=float)
        tau_raw = np.asarray([r["tau_meas"] for r in rows], dtype=float)
        unique = np.r_[True, np.diff(t_raw) > 1e-6]
        t_raw = t_raw[unique]
        q_raw = q_raw[unique]
        qd_msg_raw = qd_msg_raw[unique]
        tau_raw = tau_raw[unique]
        dt = 1.0 / fit_rate
        t = np.arange(t_raw[0], t_raw[-1], dt)
        if t.size < 30:
            continue
        q_interp = np.column_stack([
            np.interp(t, t_raw, q_raw[:, j]) for j in range(N_ARM)])
        qd_msg = np.column_stack([
            np.interp(t, t_raw, qd_msg_raw[:, j]) for j in range(N_ARM)])
        tau = np.column_stack([
            np.interp(t, t_raw, tau_raw[:, j]) for j in range(N_ARM)])
        win = max(7, int(round(smooth_window_sec * fit_rate)))
        if win % 2 == 0:
            win += 1
        win = min(win, len(t) - (1 - len(t) % 2))
        if win < 7:
            continue
        q = savgol_filter(q_interp, win, 3, axis=0, mode="interp")
        qd = savgol_filter(q_interp, win, 3, deriv=1, delta=dt,
                           axis=0, mode="interp")
        qdd = savgol_filter(q_interp, win, 3, deriv=2, delta=dt,
                            axis=0, mode="interp")
        mask = (t >= trim_sec) & (t <= t[-1] - trim_sec)
        repeat, posture, direction = parse_segment(name)
        for i in np.flatnonzero(mask):
            out.append(dict(
                segment=name,
                repeat=repeat,
                posture_index=posture,
                direction=direction,
                t=float(t[i]),
                q=q[i],
                qdot=qd[i],
                qdot_msg=qd_msg[i],
                qddot=qdd[i],
                tau=tau[i],
            ))
    return out


def group_key(row: dict, mode: str):
    if mode == "combined":
        return "combined"
    if mode == "posture":
        return f"p{row['posture_index']}"
    if mode == "direction":
        return row["direction"]
    if mode == "posture_direction":
        return f"p{row['posture_index']}_{row['direction']}"
    if mode == "repeat":
        return f"r{row['repeat']}"
    raise ValueError(mode)


def residual_matrix(model: GravityModel, rows: list[dict], zero: np.ndarray,
                    ratio: np.ndarray, q5_zero: float,
                    demean_by_segment: bool) -> np.ndarray:
    q0 = zero.copy()
    q0[J5] = float(q5_zero)
    raw = []
    segs = []
    for r in rows:
        pred = rnea_scaled(model, r["q"] + q0, r["qdot"], r["qddot"], ratio)
        raw.append(pred[EVAL] - r["tau"][EVAL])
        segs.append(r["segment"])
    arr = np.asarray(raw, dtype=float)
    if demean_by_segment:
        segs_arr = np.asarray(segs)
        for seg in sorted(set(segs)):
            mask = segs_arr == seg
            arr[mask] -= np.mean(arr[mask], axis=0)
    return arr


def scan_rows(model: GravityModel, rows: list[dict], zero: np.ndarray,
              ratio: np.ndarray, scan_rad: np.ndarray,
              demean_by_segment: bool) -> dict:
    weights = 1.0 / np.maximum(TAU_NOISE[EVAL], 0.02)
    costs = []
    raw_rms = []
    for z in scan_rad:
        resid = residual_matrix(model, rows, zero, ratio, z, demean_by_segment)
        weighted = resid * weights
        costs.append(float(np.sqrt(np.mean(weighted ** 2))))
        raw_rms.append(float(np.sqrt(np.mean(resid ** 2))))
    costs_arr = np.asarray(costs)
    i = int(np.argmin(costs_arr))
    edge = i == 0 or i == len(scan_rad) - 1
    return dict(
        n_samples=int(len(rows)),
        best_q5_zero_rad=float(scan_rad[i]),
        best_q5_zero_deg=float(np.degrees(scan_rad[i])),
        best_weighted_rms=float(costs_arr[i]),
        best_raw_rms_nm=float(raw_rms[i]),
        edge_minimum=bool(edge),
        scan_q5_zero_deg=np.degrees(scan_rad).tolist(),
        weighted_rms=costs,
        raw_rms_nm=raw_rms,
    )


def analyze_scan(model: GravityModel, rows: list[dict], zero: np.ndarray,
                 ratio: np.ndarray, scan_rad: np.ndarray,
                 demean_by_segment: bool) -> dict:
    by_mode = {}
    for mode in ("combined", "posture", "direction", "posture_direction", "repeat"):
        groups: dict[str, list[dict]] = {}
        for r in rows:
            groups.setdefault(group_key(r, mode), []).append(r)
        by_mode[mode] = {
            k: scan_rows(model, v, zero, ratio, scan_rad, demean_by_segment)
            for k, v in sorted(groups.items())
        }
    posture_bests = [
        v["best_q5_zero_deg"] for v in by_mode["posture"].values()
        if v["n_samples"] > 0
    ]
    direction_bests = [
        v["best_q5_zero_deg"] for v in by_mode["direction"].values()
        if v["n_samples"] > 0
    ]
    repeat_bests = [
        v["best_q5_zero_deg"] for v in by_mode["repeat"].values()
        if v["n_samples"] > 0
    ]
    combined = by_mode["combined"]["combined"]
    edge_any = combined["edge_minimum"] or any(
        v["edge_minimum"] for group in by_mode.values() for v in group.values())
    posture_span = (
        max(posture_bests) - min(posture_bests) if posture_bests else float("nan"))
    direction_span = (
        max(direction_bests) - min(direction_bests) if direction_bests else float("nan"))
    repeat_span = (
        max(repeat_bests) - min(repeat_bests) if repeat_bests else 0.0)
    stable = (
        not edge_any
        and posture_span <= 4.0
        and direction_span <= 4.0
        and repeat_span <= 3.0
    )
    return dict(
        scanned_joint="J5",
        baseline_q5_zero_deg=float(np.degrees(zero[J5])),
        gravity_ratio_frozen=True,
        demean_by_segment=bool(demean_by_segment),
        stability=dict(
            stable=bool(stable),
            posture_best_span_deg=float(posture_span),
            direction_best_span_deg=float(direction_span),
            repeat_best_span_deg=float(repeat_span),
            any_edge_minimum=bool(edge_any),
            rule="PASS only if minima are not on scan edge and posture/direction spans <= 4 deg",
        ),
        scans=by_mode,
    )


def write_processed_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        header = ["segment", "repeat", "posture_index", "direction", "t"]
        for prefix in ("q", "qdot", "qdot_msg", "qddot", "tau_meas"):
            header.extend([f"{prefix}_J{j}" for j in range(1, N_ARM + 1)])
        writer.writerow(header)
        for r in rows:
            writer.writerow(
                [r["segment"], r["repeat"], r["posture_index"], r["direction"], r["t"]]
                + r["q"].tolist()
                + r["qdot"].tolist()
                + r["qdot_msg"].tolist()
                + r["qddot"].tolist()
                + r["tau"].tolist()
            )


def run_capture(args, j5_holds: list[float]) -> tuple[list[dict], list[dict]]:
    if rclpy is None:
        raise RuntimeError(f"rclpy unavailable: {_RCLPY_ERROR}")
    rclpy.init()
    node = J5CouplingNode(str(args.calibration_yaml))
    try:
        if not node.wait_state():
            raise RuntimeError("no /joint_states position/velocity/effort")
        why = node.check_effort_enabled()
        if why:
            raise RuntimeError(why)
        for rep in range(args.repeats):
            for idx, j5_hold in enumerate(j5_holds):
                center = center_for_j5(j5_hold, args.j4_hold_rad, args.j6_hold_rad)
                node.get_logger().info(
                    f"J5 coupling repeat {rep} posture {idx}: J5={np.degrees(j5_hold):+.2f}deg")
                why = node.goto(center)
                if why:
                    raise RuntimeError(f"posture {idx} transit failed: {why}")
                node.hold(center, args.hold_sec)
                for direction, reverse in (("forward", False), ("reverse", True)):
                    name = f"r{rep}_p{idx}_{direction}"
                    node.get_logger().info(f"J5 coupling segment: {name}")
                    why = node.run_segment(
                        name, trajectory(center, args.duration, args.rate, reverse))
                    if why:
                        raise RuntimeError(f"{name} aborted: {why}")
                    node.hold(center, args.hold_sec)
        records = list(node.records)
    finally:
        node.segment = None
        node.destroy_node()
        rclpy.shutdown()
    processed = process_records(
        records, args.fit_rate, args.smooth_window_sec, args.trim_sec)
    return records, processed


def load_existing_capture(path: Path) -> tuple[dict, list[dict]]:
    data = json.loads(path.read_text())
    raw = data.get("raw_records", [])
    return data, raw


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--calibration-yaml", type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument("--j5-holds", default=DEFAULT_J5_HOLDS)
    ap.add_argument("--j4-hold-rad", type=float, default=float(BASE_CENTER[3]))
    ap.add_argument("--j6-hold-rad", type=float, default=float(BASE_CENTER[5]))
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE)
    ap.add_argument("--fit-rate", type=float, default=DEFAULT_FIT_RATE)
    ap.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    ap.add_argument("--smooth-window-sec", type=float, default=0.31)
    ap.add_argument("--trim-sec", type=float, default=0.50)
    ap.add_argument("--scan-deg", default="-35:20:1")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--no-demean", action="store_true")
    ap.add_argument("--plan-only", action="store_true")
    ap.add_argument("--input-json", type=Path, default=None)
    args = ap.parse_args()

    j5_holds = parse_floats(args.j5_holds)
    scan_rad = parse_scan_deg(args.scan_deg)
    cal = load_arm6_calibration(str(args.calibration_yaml))
    zero = np.asarray(cal["zero_offset_rad"], dtype=float)
    ratio = np.asarray(cal["first_moment_ratio"], dtype=float)
    model = GravityModel()
    apply_armature(model, cal)
    plan = plan_summary(
        model, j5_holds, args.duration, args.rate, args.repeats,
        args.j4_hold_rad, args.j6_hold_rad)
    plan["calibration_yaml"] = str(args.calibration_yaml)
    plan["frozen_zero_offset_deg"] = np.degrees(zero).tolist()
    plan["frozen_first_moment_ratio"] = ratio.tolist()
    plan["armature_kgm2"] = model.model.armature.tolist()
    plan["scan_q5_zero_deg"] = np.degrees(scan_rad).tolist()

    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return 0 if plan["ok"] else 1
    if not plan["ok"]:
        print("REFUSED: plan is unsafe; run --plan-only for details", file=sys.stderr)
        return 2

    stamp = time.strftime("%Y%m%d_%H%M%S")
    if args.input_json is not None:
        loaded, raw_records = load_existing_capture(args.input_json)
        processed = process_records(
            raw_records, args.fit_rate, args.smooth_window_sec, args.trim_sec)
        mode = "rawbias" if args.no_demean else "demean"
        stamp = time.strftime("%Y%m%d_%H%M%S") + f"_rescan_{mode}"
        source = str(args.input_json)
        original_metadata = loaded.get("metadata", {})
    else:
        try:
            raw_records, processed = run_capture(args, j5_holds)
        except RuntimeError as exc:
            print(f"REFUSED: {exc}", file=sys.stderr)
            return 3
        source = None
        original_metadata = None

    scan = analyze_scan(
        model, processed, zero, ratio, scan_rad, demean_by_segment=not args.no_demean)
    OUT.mkdir(parents=True, exist_ok=True)
    json_path = OUT / f"phase3_dynamic_j5_coupling_zero_scan_{stamp}.json"
    csv_path = OUT / f"phase3_dynamic_j5_coupling_zero_scan_processed_{stamp}.csv"
    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            writes_yaml=False,
            source_capture=source,
            original_metadata=original_metadata,
            calibration_yaml=str(args.calibration_yaml),
            method="J5 q0 scan with frozen J1-J3 zero and frozen gravity ratio",
            rnea="Pinocchio RNEA with optional armature from calibration YAML",
            residual="predicted RNEA minus measured effort on J2-J5",
        ),
        plan=plan,
        raw_records=raw_records,
        processed_sample_count=len(processed),
        scan=scan,
    )
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    write_processed_csv(csv_path, processed)
    combined = scan["scans"]["combined"]["combined"]
    print(json.dumps(dict(
        result=str(json_path),
        processed_csv=str(csv_path),
        processed_sample_count=len(processed),
        combined_best_q5_zero_deg=combined["best_q5_zero_deg"],
        combined_best_weighted_rms=combined["best_weighted_rms"],
        stability=scan["stability"],
        writes_yaml=False,
    ), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
