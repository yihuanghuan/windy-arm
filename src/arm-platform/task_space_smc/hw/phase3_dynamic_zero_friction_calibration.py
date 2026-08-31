"""双速度动态零位标定（实物）：RNEA + Coulomb 摩擦 + 粘性阻尼。

采集设计：
  - J1-J5 联动，J6 固定不标定。
  - 两条不同频率/幅值轨迹：A、B。
  - 每条轨迹做 slow/fast 双速度，每个速度做 forward/reverse。

拟合模型：

    tau_meas[j] = RNEA(q_encoder + q0, qdot, qddot)[j]
                  + Fc[j] * tanh(qdot[j] / v_eps)
                  + B[j] * qdot[j]

其中 j=J1..J5。脚本只保存 JSON/CSV，不写 arm6_calibration.yaml。
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
    DynamicZeroNode, load_existing_zero, plan_summary, rnea_tau,
    safety_reasons, trajectory as trajectory_a,
)


OUT = Path(__file__).resolve().parent / "data" / "phase3"

SPEEDS = {
    "slow": 24.0,
    "fast": 16.0,
}

TRAJECTORY_B_AMPLITUDE = np.array([0.30, 0.27, 0.34, 0.42, 0.34, 0.0])
TRAJECTORY_B_CYCLES = np.array([1.3, 2.0, 2.7, 3.5, 4.2, 0.0])
TRAJECTORY_B_PHASE = np.array([0.4, 1.2, 2.4, 0.9, 2.9, 0.0])
TRAJECTORY_B_PHASE2 = np.array([2.2, 0.6, 1.6, 2.7, 0.2, 0.0])
SECOND_HARMONIC_B = 0.24


def trajectory_b(duration: float, rate: float, reverse: bool = False) -> dict:
    dt = 1.0 / rate
    t = np.arange(0.0, duration + 0.5 * dt, dt)
    x = t / duration
    env = np.sin(np.pi * x) ** 2
    q = np.tile(CENTER, (t.size, 1))
    for j in ACTIVE:
        s1 = np.sin(2.0 * np.pi * TRAJECTORY_B_CYCLES[j] * x
                    + TRAJECTORY_B_PHASE[j])
        s2 = np.sin(2.0 * np.pi * (TRAJECTORY_B_CYCLES[j] + 1.0) * x
                    + TRAJECTORY_B_PHASE2[j])
        q[:, j] += TRAJECTORY_B_AMPLITUDE[j] * env * (
            s1 + SECOND_HARMONIC_B * s2) / (1.0 + SECOND_HARMONIC_B)
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


def get_trajectory(family: str, duration: float, rate: float,
                   reverse: bool = False) -> dict:
    if family == "A":
        return trajectory_a(duration, rate, reverse)
    if family == "B":
        return trajectory_b(duration, rate, reverse)
    raise ValueError(f"unknown trajectory family {family}")


def check_plan(model: GravityModel, rate: float) -> dict:
    segments = []
    ok = True
    q_all = []
    qd_all = []
    qdd_all = []
    path_bad = []
    for step in interpolate(SAFE_WAYPOINT, CENTER):
        reasons = safety_reasons(model, step)
        if reasons:
            path_bad.append(reasons[0])
            ok = False
            break
    for family in ("A", "B"):
        for speed, duration in SPEEDS.items():
            for direction, reverse in (("forward", False), ("reverse", True)):
                tr = get_trajectory(family, duration, rate, reverse)
                # Reuse the single-trajectory checker through plan_summary for A,
                # and explicitly use safety_reasons for B below.
                bad = []
                for i, q in enumerate(tr["q"]):
                    reasons = safety_reasons(model, q)
                    if reasons:
                        bad.append(dict(index=i, t=float(tr["t"][i]),
                                        reasons=reasons))
                seg_ok = not bad
                ok = ok and seg_ok
                q_all.append(tr["q"])
                qd_all.append(tr["qd"])
                qdd_all.append(tr["qdd"])
                segments.append(dict(
                    name=f"{family}_{speed}_{direction}",
                    family=family,
                    speed=speed,
                    direction=direction,
                    duration_sec=duration,
                    safety=dict(ok=seg_ok, violation_count=len(bad),
                                violations=bad[:20]),
                    q_min_rad=np.min(tr["q"], axis=0).tolist(),
                    q_max_rad=np.max(tr["q"], axis=0).tolist(),
                    max_abs_cmd_vel=np.max(np.abs(tr["qd"]), axis=0).tolist(),
                    max_abs_cmd_acc=np.max(np.abs(tr["qdd"]), axis=0).tolist(),
                ))
    q_all = np.vstack(q_all)
    qd_all = np.vstack(qd_all)
    qdd_all = np.vstack(qdd_all)
    return dict(
        ok=ok,
        rate_hz=rate,
        center_rad=CENTER.tolist(),
        safe_waypoint_to_center_ok=not path_bad,
        safe_waypoint_to_center_reasons=path_bad,
        estimated_dynamic_sec=float(sum(2.0 * d for d in SPEEDS.values()) * 2.0),
        estimated_total_minutes=float(
            (sum(2.0 * d for d in SPEEDS.values()) * 2.0
             + len(segments) * DEFAULT_HOLD_SEC + 30.0) / 60.0),
        q_min_rad=np.min(q_all, axis=0).tolist(),
        q_max_rad=np.max(q_all, axis=0).tolist(),
        max_abs_cmd_vel=np.max(np.abs(qd_all), axis=0).tolist(),
        max_abs_cmd_acc=np.max(np.abs(qdd_all), axis=0).tolist(),
        segments=segments,
        reference_A=plan_summary(model, SPEEDS["fast"], rate),
    )


def segment_info(name: str) -> dict:
    family, speed, direction = name.split("_", 2)
    return dict(family=family, speed=speed, direction=direction)


def process_segments(records: list[dict], fit_rate: float,
                     smooth_window_sec: float, trim_sec: float) -> dict:
    grouped: dict[str, list[dict]] = {}
    for row in records:
        grouped.setdefault(row["segment"], []).append(row)
    out_rows = []
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
        info = segment_info(name)
        for i in np.flatnonzero(mask):
            out_rows.append(dict(
                segment=name,
                family=info["family"],
                speed=info["speed"],
                direction=info["direction"],
                t=float(t[i]),
                q=q[i],
                qdot=qd[i],
                qdot_msg=qd_msg[i],
                qddot=qdd[i],
                tau=tau[i],
            ))
    return dict(rows=out_rows, segment_names=sorted(grouped))


def rows_to_arrays(rows: list[dict]) -> dict:
    return dict(
        q=np.asarray([r["q"] for r in rows], dtype=float),
        qd=np.asarray([r["qdot"] for r in rows], dtype=float),
        qdd=np.asarray([r["qddot"] for r in rows], dtype=float),
        tau=np.asarray([r["tau"] for r in rows], dtype=float),
    )


def pack(q0: np.ndarray, fc: np.ndarray, visc: np.ndarray) -> np.ndarray:
    return np.r_[q0[ACTIVE], fc[ACTIVE], visc[ACTIVE]]


def unpack(x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q0 = np.zeros(N_ARM)
    fc = np.zeros(N_ARM)
    visc = np.zeros(N_ARM)
    q0[ACTIVE] = x[:5]
    fc[ACTIVE] = x[5:10]
    visc[ACTIVE] = x[10:15]
    return q0, fc, visc


def fit_model(model: GravityModel, rows: list[dict],
              initial_zero: np.ndarray, v_eps: float) -> dict:
    arr = rows_to_arrays(rows)
    weights = 1.0 / np.maximum(TAU_NOISE[ACTIVE], 0.02)
    x0 = pack(initial_zero, np.zeros(N_ARM), np.zeros(N_ARM))
    lower = np.r_[np.full(5, -0.75), np.full(5, -4.0), np.full(5, -8.0)]
    upper = np.r_[np.full(5, 0.75), np.full(5, 4.0), np.full(5, 8.0)]

    def residual(x: np.ndarray) -> np.ndarray:
        q0, fc, visc = unpack(x)
        out = []
        for q, qd, qdd, tau in zip(
                arr["q"], arr["qd"], arr["qdd"], arr["tau"]):
            pred = rnea_tau(model, q + q0, qd, qdd)
            pred = pred + fc * np.tanh(qd / v_eps) + visc * qd
            out.append((pred[ACTIVE] - tau[ACTIVE]) * weights)
        return np.concatenate(out)

    sol = least_squares(
        residual, np.clip(x0, lower, upper), bounds=(lower, upper),
        loss="soft_l1", f_scale=1.0, max_nfev=350)
    q0, fc, visc = unpack(sol.x)
    raw = []
    for q, qd, qdd, tau in zip(arr["q"], arr["qd"], arr["qdd"], arr["tau"]):
        pred = rnea_tau(model, q + q0, qd, qdd)
        pred = pred + fc * np.tanh(qd / v_eps) + visc * qd
        raw.append(pred[ACTIVE] - tau[ACTIVE])
    raw = np.asarray(raw)
    return dict(
        ok=bool(sol.success),
        message=str(sol.message),
        n_samples=int(len(rows)),
        zero_offset_rad=q0.tolist(),
        zero_offset_deg=np.degrees(q0).tolist(),
        coulomb_nm=fc.tolist(),
        viscous_nm_per_rad_s=visc.tolist(),
        residual_all_rms_nm=float(np.sqrt(np.mean(raw ** 2))),
        residual_rms_nm=np.sqrt(np.mean(raw ** 2, axis=0)).tolist(),
        residual_max_abs_nm=np.max(np.abs(raw), axis=0).tolist(),
        cost=float(sol.cost),
        nfev=int(sol.nfev),
    )


def subset(rows: list[dict], *, family: str | None = None,
           speed: str | None = None, direction: str | None = None) -> list[dict]:
    out = rows
    if family is not None:
        out = [r for r in out if r["family"] == family]
    if speed is not None:
        out = [r for r in out if r["speed"] == speed]
    if direction is not None:
        out = [r for r in out if r["direction"] == direction]
    return out


def fit_all_subsets(model: GravityModel, rows: list[dict],
                    initial_zero: np.ndarray, v_eps: float) -> dict:
    specs = [("combined", {})]
    specs.extend((f"family_{f}", dict(family=f)) for f in ("A", "B"))
    specs.extend((f"speed_{s}", dict(speed=s)) for s in ("slow", "fast"))
    specs.extend((f"direction_{d}", dict(direction=d))
                 for d in ("forward", "reverse"))
    for f in ("A", "B"):
        for s in ("slow", "fast"):
            specs.append((f"{f}_{s}", dict(family=f, speed=s)))
    fits = {}
    for name, filt in specs:
        rr = subset(rows, **filt)
        if len(rr) < 50:
            fits[name] = dict(ok=False, reason=f"only {len(rr)} samples")
            continue
        fits[name] = fit_model(model, rr, initial_zero, v_eps)
    return fits


def stability_summary(fits: dict) -> dict:
    pairs = [
        ("family_A", "family_B"),
        ("speed_slow", "speed_fast"),
        ("direction_forward", "direction_reverse"),
        ("A_slow", "A_fast"),
        ("B_slow", "B_fast"),
    ]
    out = {}
    for a, b in pairs:
        if not fits.get(a, {}).get("ok") or not fits.get(b, {}).get("ok"):
            continue
        qa = np.asarray(fits[a]["zero_offset_rad"], dtype=float)
        qb = np.asarray(fits[b]["zero_offset_rad"], dtype=float)
        delta = qb - qa
        out[f"{a}_vs_{b}"] = dict(
            delta_rad=delta.tolist(),
            delta_deg=np.degrees(delta).tolist(),
            max_abs_delta_deg=float(np.max(np.abs(np.degrees(delta[ACTIVE])))),
            j4_delta_deg=float(np.degrees(delta[3])),
            j5_delta_deg=float(np.degrees(delta[4])),
        )
    if fits.get("combined", {}).get("ok"):
        q = np.asarray(fits["combined"]["zero_offset_rad"], dtype=float)
        out["combined_zero_offset_deg"] = np.degrees(q).tolist()
    return out


def write_processed_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        header = ["segment", "family", "speed", "direction", "t"]
        for prefix in ("q", "qdot", "qdot_msg", "qddot", "tau_meas"):
            header.extend([f"{prefix}_J{j}" for j in range(1, N_ARM + 1)])
        writer.writerow(header)
        for r in rows:
            writer.writerow(
                [r["segment"], r["family"], r["speed"], r["direction"], r["t"]]
                + r["q"].tolist()
                + r["qdot"].tolist()
                + r["qdot_msg"].tolist()
                + r["qddot"].tolist()
                + r["tau"].tolist()
            )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE)
    ap.add_argument("--fit-rate", type=float, default=DEFAULT_FIT_RATE)
    ap.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    ap.add_argument("--smooth-window-sec", type=float, default=0.31)
    ap.add_argument("--trim-sec", type=float, default=0.50)
    ap.add_argument("--v-eps", type=float, default=0.03)
    ap.add_argument("--plan-only", action="store_true")
    args = ap.parse_args()

    model = GravityModel()
    plan = check_plan(model, args.rate)
    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return 0 if plan["ok"] else 1
    if not plan["ok"]:
        print("REFUSED: plan is unsafe; run --plan-only", file=sys.stderr)
        return 2
    if rclpy is None:
        print(f"rclpy unavailable: {_RCLPY_ERROR}", file=sys.stderr)
        return 3

    q0_existing = load_existing_zero()
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
        node.get_logger().info("dual-speed dynamic zero: transit to center")
        why = node.goto(CENTER)
        if why:
            print(f"REFUSED: center transit failed: {why}", file=sys.stderr)
            return 6
        node.hold(CENTER, args.hold_sec)
        for family in ("A", "B"):
            for speed, duration in SPEEDS.items():
                for direction, reverse in (("forward", False), ("reverse", True)):
                    name = f"{family}_{speed}_{direction}"
                    node.get_logger().info(f"dynamic segment: {name}")
                    tr = get_trajectory(family, duration, args.rate, reverse)
                    why = node.run_segment(name, tr)
                    if why:
                        print(f"REFUSED: {name} aborted: {why}", file=sys.stderr)
                        return 7
                    node.hold(CENTER, args.hold_sec)
    finally:
        node.segment = None
        node.destroy_node()
        rclpy.shutdown()

    processed = process_segments(
        node.records, args.fit_rate, args.smooth_window_sec, args.trim_sec)
    fits = fit_all_subsets(model, processed["rows"], q0_existing, args.v_eps)
    stability = stability_summary(fits)

    OUT.mkdir(parents=True, exist_ok=True)
    json_path = OUT / f"phase3_dynamic_zero_friction_{stamp}.json"
    csv_path = OUT / f"phase3_dynamic_zero_friction_processed_{stamp}.csv"
    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            writes_yaml=False,
            fitted_joints="J1-J5",
            j6="not fitted",
            model="RNEA(q+q0, qdot, qddot) + Fc*tanh(qdot/v_eps) + B*qdot",
            v_eps=args.v_eps,
        ),
        plan=plan,
        existing_zero_offset_rad=q0_existing.tolist(),
        raw_records=node.records,
        processed_segments=processed["segment_names"],
        fits=fits,
        stability=stability,
    )
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    write_processed_csv(csv_path, processed["rows"])
    print(json.dumps(dict(
        result=str(json_path),
        processed_csv=str(csv_path),
        combined=fits.get("combined"),
        stability=stability,
    ), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
