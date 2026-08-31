"""动态零位标定（实物）：多关节耦合往返轨迹 + Pinocchio RNEA。

该脚本用于替代多姿态静态力矩方案：

  1. J1-J5 同时走低速、多频耦合轨迹，J6 固定不标定。
  2. 正向和反向时间轨迹都执行，降低库仑摩擦对零位的单向偏置。
  3. 连续记录 q、qdot、由 q 平滑微分得到的 qddot，以及实测力矩。
  4. 用 Pinocchio RNEA 计算 tau_model(q_encoder + q0, qdot, qddot)。
  5. 只优化 J1-J5 zero offset；J6 固定为 0。

不自动写 arm6_calibration.yaml。正式运行只保存 JSON/CSV。
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
import yaml
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

from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, N_ARM, SAFE_WAYPOINT, TAU_NOISE, check_setpoint,
    interpolate, JOINT_LOWER, JOINT_UPPER,
)
from task_space_smc.hw.phase3_gravity_calibration import (  # noqa: E402
    RATIO, desk_margin,
)


OUT = Path(__file__).resolve().parent / "data" / "phase3"
ACTIVE = np.arange(5)

DEFAULT_DURATION = 16.0
DEFAULT_RATE = 100.0
DEFAULT_FIT_RATE = 100.0
DEFAULT_HOLD_SEC = 2.0

CENTER = np.array([-0.18, 0.88, 0.30, 0.15, -0.22, SAFE_WAYPOINT[5]])
AMPLITUDE = np.array([0.36, 0.34, 0.42, 0.58, 0.46, 0.0])
CYCLES = np.array([1.0, 1.7, 2.3, 3.1, 3.7, 0.0])
PHASE = np.array([0.0, 0.7, 1.4, 2.1, 2.8, 0.0])
PHASE2 = np.array([1.1, 2.0, 2.8, 0.4, 1.7, 0.0])
SECOND_HARMONIC = 0.32

TRACK_P = np.array([22.0, 22.0, 22.0, 4.0, 4.0, 4.0])
TRACK_D = np.array([1.0, 1.0, 1.0, 0.16, 0.16, 0.16])
GOTO_P = np.array([30.0, 30.0, 30.0, 5.0, 5.0, 5.0])
GOTO_D = np.array([1.0, 1.0, 1.0, 0.18, 0.18, 0.18])

MAX_CMD_VEL = np.array([0.45, 0.45, 0.45, 0.65, 0.65, 0.20])
MAX_CMD_ACC = np.array([0.95, 0.95, 0.95, 1.55, 1.55, 0.50])
MAX_MEAS_SPEED = np.array([0.90, 0.90, 0.90, 1.20, 1.20, 0.70])
MIN_LIMIT_MARGIN = 0.12
MAX_TRACK_ERR = 0.22
GOTO_ACCEPT_ERR = 0.08
SPEED_ABORT_DWELL_SEC = 0.15
PUBLISH_RATE = 100.0
TRANSIT_MAX_VEL = np.array([0.18, 0.18, 0.18, 0.25, 0.25, 0.20])
TRANSIT_MIN_SEC = 4.0


def load_existing_zero() -> np.ndarray:
    path = Path(__file__).resolve().parents[2] / "config" / "arm6_calibration.yaml"
    data = yaml.safe_load(path.read_text())
    return np.asarray(data.get("zero_offset_rad", [0.0] * N_ARM), dtype=float)


def gravity_ff(model: GravityModel, q: np.ndarray, q0_ff: np.ndarray) -> np.ndarray:
    tau = model.gravity(np.asarray(q, dtype=float) + q0_ff)
    ratio = np.array([RATIO.get(i + 1, 1.0) for i in range(N_ARM)], dtype=float)
    return tau * ratio


def rnea_tau(model: GravityModel, q: np.ndarray,
             qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
    return pin.rnea(
        model.model,
        model.data,
        np.asarray(q, dtype=float),
        np.asarray(qd, dtype=float),
        np.asarray(qdd, dtype=float),
    ).copy()


def trajectory(duration: float, rate: float, reverse: bool = False) -> dict:
    dt = 1.0 / rate
    t = np.arange(0.0, duration + 0.5 * dt, dt)
    x = t / duration
    env = np.sin(np.pi * x) ** 2
    q = np.tile(CENTER, (t.size, 1))
    for j in ACTIVE:
        s1 = np.sin(2.0 * np.pi * CYCLES[j] * x + PHASE[j])
        s2 = np.sin(2.0 * np.pi * (CYCLES[j] + 1.0) * x + PHASE2[j])
        q[:, j] += AMPLITUDE[j] * env * (s1 + SECOND_HARMONIC * s2) / (
            1.0 + SECOND_HARMONIC)
    q[:, 5] = CENTER[5]
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


def safety_reasons(model: GravityModel, q: np.ndarray) -> list[str]:
    rep = check_setpoint(q, model)
    reasons = list(rep.reasons)
    margin = desk_margin(model, q)
    if margin < 0.0 and not any("desk" in r.lower() for r in reasons):
        reasons.append(f"desk clearance margin {margin:.4f} m")
    lim = np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)
    if np.min(lim) < MIN_LIMIT_MARGIN:
        reasons.append(
            f"joint{int(np.argmin(lim)) + 1} limit margin {np.min(lim):.3f} rad")
    return reasons


def check_trajectory_plan(model: GravityModel, tr: dict) -> dict:
    bad = []
    q = tr["q"]
    qd = tr["qd"]
    qdd = tr["qdd"]
    for i in range(q.shape[0]):
        reasons = safety_reasons(model, q[i])
        if reasons:
            bad.append(dict(index=i, t=float(tr["t"][i]), reasons=reasons))
        v_over = np.abs(qd[i]) - MAX_CMD_VEL
        if np.max(v_over) > 1e-9:
            bad.append(dict(
                index=i, t=float(tr["t"][i]),
                reasons=[f"command velocity joint{int(np.argmax(v_over)) + 1} "
                         f"{np.max(np.abs(qd[i])):.3f} rad/s"]))
        a_over = np.abs(qdd[i]) - MAX_CMD_ACC
        if np.max(a_over) > 1e-9:
            bad.append(dict(
                index=i, t=float(tr["t"][i]),
                reasons=[f"command acceleration joint{int(np.argmax(a_over)) + 1} "
                         f"{np.max(np.abs(qdd[i])):.3f} rad/s^2"]))
    return dict(ok=not bad, violation_count=len(bad), violations=bad[:20])


def plan_summary(model: GravityModel, duration: float, rate: float) -> dict:
    fwd = trajectory(duration, rate, reverse=False)
    rev = trajectory(duration, rate, reverse=True)
    q_all = np.vstack([fwd["q"], rev["q"]])
    qd_all = np.vstack([fwd["qd"], rev["qd"]])
    qdd_all = np.vstack([fwd["qdd"], rev["qdd"]])
    tau_dyn = np.asarray([
        rnea_tau(model, q, qd, qdd) for q, qd, qdd in zip(q_all, qd_all, qdd_all)
    ])
    path_bad = []
    for step in interpolate(SAFE_WAYPOINT, CENTER):
        reasons = safety_reasons(model, step)
        if reasons:
            path_bad.append(reasons[0])
            break
    return dict(
        center_rad=CENTER.tolist(),
        amplitude_rad=AMPLITUDE.tolist(),
        duration_sec=duration,
        rate_hz=rate,
        total_dynamic_sec=2.0 * duration,
        estimated_total_minutes=(2.0 * duration + 3.0 * DEFAULT_HOLD_SEC + 25.0) / 60.0,
        q_min_rad=np.min(q_all, axis=0).tolist(),
        q_max_rad=np.max(q_all, axis=0).tolist(),
        max_abs_cmd_vel=np.max(np.abs(qd_all), axis=0).tolist(),
        max_abs_cmd_acc=np.max(np.abs(qdd_all), axis=0).tolist(),
        max_abs_model_rnea_nm=np.max(np.abs(tau_dyn), axis=0).tolist(),
        safe_waypoint_to_center_ok=not path_bad,
        safe_waypoint_to_center_reasons=path_bad,
        forward_safety=check_trajectory_plan(model, fwd),
        reverse_safety=check_trajectory_plan(model, rev),
    )


class DynamicZeroNode(Node):
    def __init__(self, q0_ff: np.ndarray):
        super().__init__("phase3_dynamic_zero_calibration")
        self.model = GravityModel()
        self.q0_ff = np.asarray(q0_ff, dtype=float)
        self.q = None
        self.qd = None
        self.tau = None
        self.records: list[dict] = []
        self.segment = None
        self.t0_segment = None
        self.speed_violation_since = None
        self.pub = self.create_publisher(
            MotorControl, "/student/joint_effort_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb, 100)

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
                t_segment=now - self.t0_segment,
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

    def send(self, q: np.ndarray, qd: np.ndarray | None = None,
             p: np.ndarray | None = None, d: np.ndarray | None = None):
        q = np.asarray(q, dtype=float)
        qd = np.zeros(N_ARM) if qd is None else np.asarray(qd, dtype=float)
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in q]
        msg.velocity = [float(v) for v in qd]
        msg.current = [float(v) for v in gravity_ff(self.model, q, self.q0_ff)]
        msg.p = [float(v) for v in (TRACK_P if p is None else p)]
        msg.d = [float(v) for v in (TRACK_D if d is None else d)]
        msg.enable = 1
        self.pub.publish(msg)

    def guard(self, q_cmd: np.ndarray | None = None) -> str | None:
        if self.q is None:
            return None
        reasons = safety_reasons(self.model, self.q)
        if reasons:
            return "measured pose unsafe: " + reasons[0]
        if self.qd is not None:
            over = np.zeros(N_ARM)
            over[ACTIVE] = np.abs(self.qd[ACTIVE]) - MAX_MEAS_SPEED[ACTIVE]
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
            err[5] = 0.0
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
            float(np.max(span[ACTIVE] / TRANSIT_MAX_VEL[ACTIVE])),
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
                reasons = safety_reasons(self.model, q_cmd)
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


def resample_and_differentiate(records: list[dict], fit_rate: float,
                               smooth_window_sec: float) -> dict:
    if len(records) < 20:
        raise RuntimeError(f"need at least 20 samples, got {len(records)}")
    rows = sorted(records, key=lambda r: r["t_wall"])
    t_raw = np.asarray([r["t_wall"] for r in rows], dtype=float)
    t_raw = t_raw - t_raw[0]
    q_raw = np.asarray([r["q"] for r in rows], dtype=float)
    qdot_msg_raw = np.asarray([r["qdot_msg"] for r in rows], dtype=float)
    tau_raw = np.asarray([r["tau_meas"] for r in rows], dtype=float)
    seg_raw = [r["segment"] for r in rows]

    unique = np.r_[True, np.diff(t_raw) > 1e-6]
    t_raw = t_raw[unique]
    q_raw = q_raw[unique]
    qdot_msg_raw = qdot_msg_raw[unique]
    tau_raw = tau_raw[unique]
    seg_raw = [s for s, u in zip(seg_raw, unique) if u]

    dt = 1.0 / fit_rate
    t = np.arange(t_raw[0], t_raw[-1], dt)
    q = np.column_stack([np.interp(t, t_raw, q_raw[:, j]) for j in range(N_ARM)])
    qdot_msg = np.column_stack([
        np.interp(t, t_raw, qdot_msg_raw[:, j]) for j in range(N_ARM)])
    tau = np.column_stack([
        np.interp(t, t_raw, tau_raw[:, j]) for j in range(N_ARM)])
    seg = []
    idx = np.searchsorted(t_raw, t, side="right") - 1
    idx = np.clip(idx, 0, len(seg_raw) - 1)
    seg = [seg_raw[i] for i in idx]

    win = max(7, int(round(smooth_window_sec * fit_rate)))
    if win % 2 == 0:
        win += 1
    win = min(win, len(t) - (1 - len(t) % 2))
    if win < 7:
        raise RuntimeError("not enough samples for smoothing")
    q_smooth = savgol_filter(q, win, 3, axis=0, mode="interp")
    qdot = savgol_filter(q, win, 3, deriv=1, delta=dt, axis=0, mode="interp")
    qddot = savgol_filter(q, win, 3, deriv=2, delta=dt, axis=0, mode="interp")
    margin = int(0.5 * fit_rate)
    mask = np.ones(len(t), dtype=bool)
    if len(t) > 2 * margin:
        mask[:margin] = False
        mask[-margin:] = False
    return dict(
        t=t,
        q=q_smooth,
        q_raw_interp=q,
        qdot=qdot,
        qdot_msg=qdot_msg,
        qddot=qddot,
        tau=tau,
        segment=np.asarray(seg),
        fit_mask=mask,
        raw_rows=rows,
    )


def fit_zero_offsets(model: GravityModel, data: dict,
                     initial: np.ndarray, segment: str | None = None) -> dict:
    mask = data["fit_mask"].copy()
    if segment is not None:
        mask &= data["segment"] == segment
    q = data["q"][mask]
    qd = data["qdot"][mask]
    qdd = data["qddot"][mask]
    tau = data["tau"][mask]
    weights = 1.0 / np.maximum(TAU_NOISE[ACTIVE], 0.02)

    def residual(z: np.ndarray) -> np.ndarray:
        q0 = np.zeros(N_ARM)
        q0[ACTIVE] = z
        out = []
        for qi, vi, ai, yi in zip(q, qd, qdd, tau):
            pred = rnea_tau(model, qi + q0, vi, ai)
            out.append((pred[ACTIVE] - yi[ACTIVE]) * weights)
        return np.concatenate(out)

    x0 = np.asarray(initial, dtype=float)[ACTIVE]
    sol = least_squares(
        residual,
        x0,
        bounds=(np.full(5, -0.75), np.full(5, 0.75)),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=300,
    )
    q0 = np.zeros(N_ARM)
    q0[ACTIVE] = sol.x
    raw = []
    for qi, vi, ai, yi in zip(q, qd, qdd, tau):
        raw.append(rnea_tau(model, qi + q0, vi, ai)[ACTIVE] - yi[ACTIVE])
    raw = np.asarray(raw)
    return dict(
        segment="combined" if segment is None else segment,
        ok=bool(sol.success),
        message=str(sol.message),
        n_samples=int(q.shape[0]),
        zero_offset_rad=q0.tolist(),
        zero_offset_deg=np.degrees(q0).tolist(),
        residual_all_rms_nm=float(np.sqrt(np.mean(raw ** 2))),
        residual_rms_nm=np.sqrt(np.mean(raw ** 2, axis=0)).tolist(),
        residual_max_abs_nm=np.max(np.abs(raw), axis=0).tolist(),
        cost=float(sol.cost),
        nfev=int(sol.nfev),
    )


def write_processed_csv(path: Path, data: dict) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        header = ["t", "segment"]
        for prefix in ("q", "qdot", "qddot", "tau_meas"):
            header.extend([f"{prefix}_J{j}" for j in range(1, N_ARM + 1)])
        writer.writerow(header)
        for i in range(len(data["t"])):
            writer.writerow(
                [float(data["t"][i]), str(data["segment"][i])]
                + data["q"][i].tolist()
                + data["qdot"][i].tolist()
                + data["qddot"][i].tolist()
                + data["tau"][i].tolist()
            )


def run_plan_only(args) -> int:
    model = GravityModel()
    plan = plan_summary(model, args.duration, args.rate)
    print(json.dumps(plan, indent=2, ensure_ascii=False))
    if not plan["forward_safety"]["ok"] or not plan["reverse_safety"]["ok"] \
            or not plan["safe_waypoint_to_center_ok"]:
        return 1
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE)
    ap.add_argument("--fit-rate", type=float, default=DEFAULT_FIT_RATE)
    ap.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    ap.add_argument("--smooth-window-sec", type=float, default=0.31)
    ap.add_argument("--plan-only", action="store_true")
    args = ap.parse_args()

    if args.plan_only:
        return run_plan_only(args)
    if rclpy is None:
        print(f"rclpy unavailable: {_RCLPY_ERROR}", file=sys.stderr)
        return 2

    model = GravityModel()
    plan = plan_summary(model, args.duration, args.rate)
    if not plan["forward_safety"]["ok"] or not plan["reverse_safety"]["ok"] \
            or not plan["safe_waypoint_to_center_ok"]:
        print("REFUSED: plan is unsafe; run --plan-only for details", file=sys.stderr)
        return 3

    q0_existing = load_existing_zero()
    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    rclpy.init()
    node = DynamicZeroNode(q0_existing)
    try:
        if not node.wait_state():
            print("no /joint_states position/velocity/effort", file=sys.stderr)
            return 4
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 5
        node.get_logger().info("dynamic zero calibration: transit to center")
        why = node.goto(CENTER)
        if why:
            print(f"REFUSED: center transit failed: {why}", file=sys.stderr)
            return 6
        node.hold(CENTER, args.hold_sec)

        for name, rev in (("forward", False), ("reverse", True)):
            node.get_logger().info(f"dynamic segment: {name}")
            why = node.run_segment(name, trajectory(args.duration, args.rate, rev))
            if why:
                print(f"REFUSED: {name} segment aborted: {why}", file=sys.stderr)
                return 7
            node.hold(CENTER, args.hold_sec)
    finally:
        node.segment = None
        node.destroy_node()
        rclpy.shutdown()

    processed = resample_and_differentiate(
        node.records, args.fit_rate, args.smooth_window_sec)
    fits = [
        fit_zero_offsets(model, processed, q0_existing, None),
        fit_zero_offsets(model, processed, q0_existing, "forward"),
        fit_zero_offsets(model, processed, q0_existing, "reverse"),
    ]
    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            writes_yaml=False,
            fitted_joints="J1-J5",
            j6="not fitted",
            qddot_source="Savitzky-Golay second derivative of measured q",
            command_feedforward="existing YAML zero offset used only for gravity feedforward",
        ),
        plan=plan,
        existing_zero_offset_rad=q0_existing.tolist(),
        fits=fits,
        raw_records=node.records,
    )
    json_path = OUT / f"phase3_dynamic_zero_{stamp}.json"
    csv_path = OUT / f"phase3_dynamic_zero_processed_{stamp}.csv"
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    write_processed_csv(csv_path, processed)
    print(json.dumps(dict(
        result=str(json_path),
        processed_csv=str(csv_path),
        fits=fits,
    ), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
