"""J4/J5 专用动态耦合零位标定（实物）。

实验设计：
  - J4/J5 不做大幅自身动态运动，只固定在多个不同角度。
  - 每个腕部固定角度下，由 J2/J3 执行耦合往返轨迹。
  - 用全关节 J1-J5 的 RNEA 残差辨识 J4/J5 zero offset。
  - J1-J3 zero 固定；J6 不标定。

拟合时允许一个 RNEA scale、J1-J5 固定 bias，以及 J2/J3 的 Coulomb/粘性项
吸收主要主动关节摩擦/闭环误差，避免 J4/J5 自身摩擦和回差污染 zero。

脚本不自动写 arm6_calibration.yaml。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

try:
    import rclpy
except ImportError as exc:
    rclpy = None
    _RCLPY_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, N_ARM, SAFE_WAYPOINT, TAU_NOISE, interpolate,
)
from task_space_smc.hw.phase3_dynamic_zero_calibration import (  # noqa: E402
    ACTIVE, CENTER, DEFAULT_FIT_RATE, DEFAULT_HOLD_SEC, DEFAULT_RATE,
    DynamicZeroNode, load_existing_zero, rnea_tau, safety_reasons,
)


OUT = Path(__file__).resolve().parent / "data" / "phase3"
FIT = np.array([3, 4])
MOVING = np.array([1, 2])
EVAL = np.arange(5)

DEFAULT_DURATION = 18.0
EXCITE_AMP = np.array([0.0, 0.28, 0.34, 0.0, 0.0, 0.0])
EXCITE_CYCLES = np.array([0.0, 1.4, 2.3, 0.0, 0.0, 0.0])
EXCITE_PHASE = np.array([0.0, 0.3, 1.6, 0.0, 0.0, 0.0])
EXCITE_PHASE2 = np.array([0.0, 1.9, 0.8, 0.0, 0.0, 0.0])
SECOND_HARMONIC = 0.25

WRIST_POSTURES = [
    (-0.36, -0.42),
    (-0.36, 0.08),
    (0.02, -0.42),
    (0.02, 0.08),
    (0.38, -0.32),
    (0.38, 0.16),
]

MAX_CMD_VEL = np.array([0.20, 0.42, 0.50, 0.08, 0.08, 0.08])
MAX_CMD_ACC = np.array([0.35, 0.95, 1.20, 0.15, 0.15, 0.15])


def posture_center(j4: float, j5: float) -> np.ndarray:
    q = CENTER.copy()
    q[3] = j4
    q[4] = j5
    q[5] = SAFE_WAYPOINT[5]
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
    q[:, 3] = center[3]
    q[:, 4] = center[4]
    q[:, 5] = SAFE_WAYPOINT[5]
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


def check_tr(model: GravityModel, tr: dict) -> dict:
    bad = []
    for i, q in enumerate(tr["q"]):
        reasons = safety_reasons(model, q)
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


def check_path(model: GravityModel, qa: np.ndarray, qb: np.ndarray) -> list[str]:
    for q in interpolate(qa, qb):
        reasons = safety_reasons(model, q)
        if reasons:
            return reasons
    return []


def check_plan(model: GravityModel, duration: float, rate: float) -> dict:
    segments = []
    ok = True
    all_q = []
    all_qd = []
    all_qdd = []
    centers = [posture_center(j4, j5) for j4, j5 in WRIST_POSTURES]
    transit_bad = []
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
            rep = check_tr(model, tr)
            ok = ok and rep["ok"]
            all_q.append(tr["q"])
            all_qd.append(tr["qd"])
            all_qdd.append(tr["qdd"])
            segments.append(dict(
                name=f"p{idx}_{direction}",
                posture_index=idx,
                j4_hold_rad=float(center[3]),
                j5_hold_rad=float(center[4]),
                j4_hold_deg=float(np.degrees(center[3])),
                j5_hold_deg=float(np.degrees(center[4])),
                duration_sec=duration,
                direction=direction,
                safety=rep,
                q_min_rad=np.min(tr["q"], axis=0).tolist(),
                q_max_rad=np.max(tr["q"], axis=0).tolist(),
                max_abs_cmd_vel=np.max(np.abs(tr["qd"]), axis=0).tolist(),
                max_abs_cmd_acc=np.max(np.abs(tr["qdd"]), axis=0).tolist(),
            ))
    q = np.vstack(all_q)
    qd = np.vstack(all_qd)
    qdd = np.vstack(all_qdd)
    return dict(
        ok=ok,
        writes_yaml=False,
        fitted_zero_joints="J4-J5",
        moving_joints="J2-J3",
        posture_count=len(centers),
        dynamic_segment_count=2 * len(centers),
        dynamic_sec=float(2 * len(centers) * duration),
        estimated_total_minutes=float(
            (2 * len(centers) * duration
             + (2 * len(centers) + len(centers)) * DEFAULT_HOLD_SEC
             + 70.0) / 60.0),
        transit_safety=dict(ok=not transit_bad, violations=transit_bad[:20]),
        q_min_rad=np.min(q, axis=0).tolist(),
        q_max_rad=np.max(q, axis=0).tolist(),
        max_abs_cmd_vel=np.max(np.abs(qd), axis=0).tolist(),
        max_abs_cmd_acc=np.max(np.abs(qdd), axis=0).tolist(),
        segments=segments,
    )


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
        posture_s, direction = name.split("_", 1)
        posture_index = int(posture_s.removeprefix("p"))
        for i in np.flatnonzero(mask):
            out.append(dict(
                segment=name,
                posture_index=posture_index,
                direction=direction,
                t=float(t[i]),
                q=q[i],
                qdot=qd[i],
                qdot_msg=qd_msg[i],
                qddot=qdd[i],
                tau=tau[i],
            ))
    return out


def pack(q0: np.ndarray, scale: float, bias: np.ndarray,
         fc: np.ndarray, visc: np.ndarray) -> np.ndarray:
    return np.r_[q0[FIT], scale, bias[EVAL], fc[MOVING], visc[MOVING]]


def unpack(x: np.ndarray, fixed_zero: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, np.ndarray, np.ndarray]:
    q0 = np.asarray(fixed_zero, dtype=float).copy()
    q0[FIT] = x[:2]
    scale = float(x[2])
    bias = np.zeros(N_ARM)
    fc = np.zeros(N_ARM)
    visc = np.zeros(N_ARM)
    bias[EVAL] = x[3:8]
    fc[MOVING] = x[8:10]
    visc[MOVING] = x[10:12]
    return q0, scale, bias, fc, visc


def fit_model(model: GravityModel, rows: list[dict], fixed_zero: np.ndarray,
              v_eps: float) -> dict:
    weights = 1.0 / np.maximum(TAU_NOISE[EVAL], 0.02)
    x0 = pack(fixed_zero, 1.0, np.zeros(N_ARM), np.zeros(N_ARM), np.zeros(N_ARM))
    lower = np.r_[np.full(2, -0.75), 0.65, np.full(5, -3.0),
                  np.full(2, -4.0), np.full(2, -8.0)]
    upper = np.r_[np.full(2, 0.75), 1.35, np.full(5, 3.0),
                  np.full(2, 4.0), np.full(2, 8.0)]

    def residual(x: np.ndarray) -> np.ndarray:
        q0, scale, bias, fc, visc = unpack(x, fixed_zero)
        raw = []
        for r in rows:
            pred = scale * rnea_tau(model, r["q"] + q0, r["qdot"], r["qddot"])
            pred = pred + bias + fc * np.tanh(r["qdot"] / v_eps) + visc * r["qdot"]
            raw.append((pred[EVAL] - r["tau"][EVAL]) * weights)
        return np.asarray(raw).ravel()

    sol = least_squares(
        residual,
        np.clip(x0, lower, upper),
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=420,
    )
    q0, scale, bias, fc, visc = unpack(sol.x, fixed_zero)
    raw = []
    for r in rows:
        pred = scale * rnea_tau(model, r["q"] + q0, r["qdot"], r["qddot"])
        pred = pred + bias + fc * np.tanh(r["qdot"] / v_eps) + visc * r["qdot"]
        raw.append(pred[EVAL] - r["tau"][EVAL])
    raw = np.asarray(raw)
    return dict(
        ok=bool(sol.success),
        message=str(sol.message),
        n_samples=int(len(rows)),
        zero_offset_rad=q0.tolist(),
        zero_offset_deg=np.degrees(q0).tolist(),
        rnea_scale=scale,
        bias_nm=bias.tolist(),
        coulomb_nm=fc.tolist(),
        viscous_nm_per_rad_s=visc.tolist(),
        residual_all_rms_nm=float(np.sqrt(np.mean(raw ** 2))),
        residual_rms_nm=np.sqrt(np.mean(raw ** 2, axis=0)).tolist(),
        residual_max_abs_nm=np.max(np.abs(raw), axis=0).tolist(),
        cost=float(sol.cost),
        nfev=int(sol.nfev),
    )


def write_processed_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        header = ["segment", "posture_index", "direction", "t"]
        for prefix in ("q", "qdot", "qdot_msg", "qddot", "tau_meas"):
            header.extend([f"{prefix}_J{j}" for j in range(1, N_ARM + 1)])
        writer.writerow(header)
        for r in rows:
            writer.writerow(
                [r["segment"], r["posture_index"], r["direction"], r["t"]]
                + r["q"].tolist()
                + r["qdot"].tolist()
                + r["qdot_msg"].tolist()
                + r["qddot"].tolist()
                + r["tau"].tolist()
            )


def parse_fixed_j123(args, existing: np.ndarray) -> np.ndarray:
    q0 = existing.copy()
    if args.fixed_j123_rad is not None:
        q0[:3] = np.asarray(args.fixed_j123_rad, dtype=float)
    if args.fixed_j123_deg is not None:
        q0[:3] = np.radians(np.asarray(args.fixed_j123_deg, dtype=float))
    return q0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE)
    ap.add_argument("--fit-rate", type=float, default=DEFAULT_FIT_RATE)
    ap.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    ap.add_argument("--smooth-window-sec", type=float, default=0.31)
    ap.add_argument("--trim-sec", type=float, default=0.50)
    ap.add_argument("--v-eps", type=float, default=0.03)
    ap.add_argument("--fixed-j123-rad", nargs=3, type=float, default=None)
    ap.add_argument("--fixed-j123-deg", nargs=3, type=float, default=None)
    ap.add_argument("--plan-only", action="store_true")
    args = ap.parse_args()

    model = GravityModel()
    plan = check_plan(model, args.duration, args.rate)
    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return 0 if plan["ok"] else 1
    if not plan["ok"]:
        print("REFUSED: plan is unsafe; run --plan-only", file=sys.stderr)
        return 2
    if rclpy is None:
        print(f"rclpy unavailable: {_RCLPY_ERROR}", file=sys.stderr)
        return 3

    existing = load_existing_zero()
    fixed_zero = parse_fixed_j123(args, existing)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    rclpy.init()
    node = DynamicZeroNode(existing)
    try:
        if not node.wait_state():
            print("no /joint_states position/velocity/effort", file=sys.stderr)
            return 4
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 5
        for idx, (j4, j5) in enumerate(WRIST_POSTURES):
            center = posture_center(j4, j5)
            node.get_logger().info(f"J45 coupling posture {idx}: transit")
            why = node.goto(center)
            if why:
                print(f"REFUSED: posture {idx} transit failed: {why}",
                      file=sys.stderr)
                return 6
            node.hold(center, args.hold_sec)
            for direction, reverse in (("forward", False), ("reverse", True)):
                name = f"p{idx}_{direction}"
                node.get_logger().info(f"J45 coupling segment: {name}")
                why = node.run_segment(
                    name, trajectory(center, args.duration, args.rate, reverse))
                if why:
                    print(f"REFUSED: {name} aborted: {why}", file=sys.stderr)
                    return 7
                node.hold(center, args.hold_sec)
    finally:
        node.segment = None
        node.destroy_node()
        rclpy.shutdown()

    rows = process_records(
        node.records, args.fit_rate, args.smooth_window_sec, args.trim_sec)
    fit = fit_model(model, rows, fixed_zero, args.v_eps)

    OUT.mkdir(parents=True, exist_ok=True)
    json_path = OUT / f"phase3_dynamic_j45_coupling_zero_{stamp}.json"
    csv_path = OUT / f"phase3_dynamic_j45_coupling_zero_processed_{stamp}.csv"
    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            writes_yaml=False,
            fitted_zero_joints="J4-J5",
            fixed_zero_joints="J1-J3 from args/existing config",
            moving_joints="J2-J3",
            model="scale*RNEA(q+q0,qdot,qddot)+bias+J2/J3 Coulomb+B",
            v_eps=args.v_eps,
        ),
        plan=plan,
        existing_zero_offset_rad=existing.tolist(),
        fixed_zero_used_rad=fixed_zero.tolist(),
        raw_records=node.records,
        fit=fit,
    )
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    write_processed_csv(csv_path, rows)
    print(json.dumps(dict(
        result=str(json_path),
        processed_csv=str(csv_path),
        fixed_j1_j3_zero_deg=np.degrees(fixed_zero[:3]).tolist(),
        fit=fit,
    ), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
