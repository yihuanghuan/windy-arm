"""多姿态零位标定（实物）：整臂随机姿态 + 正反接近平均 + 联合优化。

该脚本不写 arm6_calibration.yaml，只把原始采样和拟合结果保存为 JSON。

模型：

    tau_meas[j] = scale[j] * g_urdf(q_encoder + q0)[j] + bias[j]

其中 j=1..5，J6 不参与标定。每个姿态从正、反两个方向接近并在同一目标处静态
采样，取两次力矩和关节角均值，以降低库仑摩擦与回程效应对静态力矩的污染。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

try:
    from scipy.optimize import least_squares
except ImportError as exc:
    least_squares = None
    _SCIPY_ERROR = exc

try:
    import rclpy
except ImportError as exc:
    rclpy = None
    _RCLPY_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, N_ARM, SAFE_WAYPOINT, check_setpoint, interpolate, TAU_NOISE,
)
from task_space_smc.hw.phase3_gravity_calibration import (  # noqa: E402
    Calibrator, OUT, RATIO, desk_margin,
)


ACTIVE = np.arange(5)
DEFAULT_POINTS = 40
DEFAULT_DWELL = 2.0
DEFAULT_APPROACH_DELTA = 0.12
DEFAULT_SEED = 420


# 随机整臂姿态的保守范围。比硬限位窄，避开桌面、围栏和高重力力矩区域。
SAMPLE_LOWER = np.array([-0.80, 0.15, -1.25, -1.60, -1.20, 0.0])
SAMPLE_UPPER = np.array([0.80, 1.65, 1.15, 1.60, 1.20, 0.0])


def load_existing_zero() -> np.ndarray:
    path = Path(__file__).resolve().parents[2] / "config" / "arm6_calibration.yaml"
    data = yaml.safe_load(path.read_text())
    return np.asarray(data["zero_offset_rad"], dtype=float)


def safety_reasons(model: GravityModel, q: np.ndarray) -> list[str]:
    rep = check_setpoint(q, model)
    reasons = list(rep.reasons)
    margin = desk_margin(model, q)
    if margin < 0.0 and not any("desk" in r.lower() for r in reasons):
        reasons.append(f"desk clearance margin {margin:.4f} m")
    return reasons


def safe_target_with_approach(model: GravityModel, q: np.ndarray,
                              approach_delta: float) -> tuple[bool, str]:
    for label, candidate in [("target", q)] + [
            (f"approach_{k}", a)
            for k, a in enumerate(approach_candidates(q, approach_delta), 1)]:
        reasons = safety_reasons(model, candidate)
        if reasons:
            return False, f"{label}: {reasons[0]}"
    return True, ""


def sample_safe_poses(model: GravityModel, n: int, seed: int,
                      q6: float, max_attempts: int,
                      approach_delta: float) -> tuple[np.ndarray, dict]:
    rng = np.random.default_rng(seed)
    poses: list[np.ndarray] = []
    reject_reasons: dict[str, int] = {}
    attempts = 0
    prev = None
    while len(poses) < n and attempts < max_attempts:
        attempts += 1
        q = rng.uniform(SAMPLE_LOWER, SAMPLE_UPPER)
        q[5] = q6
        ok, reason = safe_target_with_approach(model, q, approach_delta)
        if ok and prev is not None:
            for step in interpolate(prev, q):
                reasons = safety_reasons(model, step)
                if reasons:
                    ok = False
                    reason = f"path: {reasons[0]}"
                    break
        if not ok:
            reject_reasons[reason or "unsafe"] = reject_reasons.get(
                reason or "unsafe", 0) + 1
            continue
        poses.append(q)
        prev = q
    if len(poses) < n:
        raise RuntimeError(
            f"only generated {len(poses)}/{n} safe poses after {attempts} attempts")
    return order_poses(np.asarray(poses), SAFE_WAYPOINT), dict(
        attempts=attempts, rejects=reject_reasons)


def order_poses(poses: np.ndarray, start: np.ndarray) -> np.ndarray:
    remaining = [p.copy() for p in np.asarray(poses)]
    ordered = []
    q = np.asarray(start, dtype=float).copy()
    q[5] = poses[0, 5]
    while remaining:
        k = min(range(len(remaining)),
                key=lambda i: float(np.max(np.abs(remaining[i] - q))))
        q = remaining.pop(k)
        ordered.append(q)
    return np.asarray(ordered)


def approach_candidates(target: np.ndarray, delta: float) -> list[np.ndarray]:
    out = []
    for sign in (1.0, -1.0):
        q = np.asarray(target, dtype=float).copy()
        q[ACTIVE] += sign * delta
        out.append(q)
    return out


def check_plan_safety(model: GravityModel, poses: np.ndarray,
                      approach_delta: float) -> dict:
    bad = []
    prev = None
    for i, q in enumerate(poses, 1):
        for label, candidate in [("target", q)] + [
                (f"approach_{k}", a)
                for k, a in enumerate(approach_candidates(q, approach_delta), 1)]:
            reasons = safety_reasons(model, candidate)
            if reasons:
                bad.append(dict(index=i, label=label, reasons=reasons))
        if prev is not None:
            for step in interpolate(prev, q):
                reasons = safety_reasons(model, step)
                if reasons:
                    bad.append(dict(index=i, label="path_from_previous",
                                    reasons=reasons))
                    break
        prev = q
    return dict(ok=not bad, violations=bad[:20], violation_count=len(bad))


def approach_pose(node: Calibrator, target: np.ndarray,
                  sign: float, delta: float) -> str | None:
    for scale in (1.0, 0.5, 0.25):
        q_try = target.copy()
        q_try[ACTIVE] += float(sign) * float(delta) * scale
        rep = check_setpoint(q_try, node.model)
        if rep.ok and desk_margin(node.model, q_try) >= 0.0:
            why = node.goto(q_try, 0, timeout=90.0, check_drift=False)
            if why:
                return why
            return node.goto(target, 0, timeout=60.0, check_drift=False)
    return node.goto(target, 0, timeout=90.0, check_drift=False)


def capture_pose(node: Calibrator, target: np.ndarray,
                 dwell: float, approach_delta: float) -> dict:
    passes = []
    for label, sign in (("positive", 1.0), ("negative", -1.0)):
        why = approach_pose(node, target, sign, approach_delta)
        if why:
            raise RuntimeError(f"{label} approach failed: {why}")
        rec = node.dwell(target, 0, dwell)
        if rec is None:
            raise RuntimeError(f"{label} dwell got no samples")
        passes.append(dict(
            direction=label,
            q_mean=rec["q_mean"].tolist(),
            q_std=rec["q_std"].tolist(),
            tau_mean=rec["tau_mean"].tolist(),
            tau_std=rec["tau_std"].tolist(),
            n=int(rec["n"])))
    q_avg = 0.5 * (np.asarray(passes[0]["q_mean"]) + np.asarray(passes[1]["q_mean"]))
    tau_avg = 0.5 * (
        np.asarray(passes[0]["tau_mean"]) + np.asarray(passes[1]["tau_mean"]))
    tau_hyst = np.asarray(passes[0]["tau_mean"]) - np.asarray(passes[1]["tau_mean"])
    return dict(
        target_rad=target.tolist(),
        q_encoder_rad=q_avg.tolist(),
        tau_avg_nm=tau_avg.tolist(),
        tau_hysteresis_nm=tau_hyst.tolist(),
        tau_hysteresis_abs_mean_nm=float(np.mean(np.abs(tau_hyst[ACTIVE]))),
        passes=passes)


def pack_params(q0: np.ndarray, scale: np.ndarray, bias: np.ndarray) -> np.ndarray:
    return np.r_[q0[ACTIVE], scale[ACTIVE], bias[ACTIVE]]


def unpack_params(x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q0 = np.zeros(N_ARM)
    scale = np.ones(N_ARM)
    bias = np.zeros(N_ARM)
    q0[ACTIVE] = x[:5]
    scale[ACTIVE] = x[5:10]
    bias[ACTIVE] = x[10:15]
    return q0, scale, bias


def fit_offsets(model: GravityModel, samples: list[dict],
                initial_zero: np.ndarray) -> dict:
    if least_squares is None:
        raise RuntimeError(f"scipy.optimize.least_squares unavailable: {_SCIPY_ERROR}")

    q_enc = np.asarray([s["q_encoder_rad"] for s in samples], dtype=float)
    tau = np.asarray([s["tau_avg_nm"] for s in samples], dtype=float)
    weights = 1.0 / np.maximum(TAU_NOISE[ACTIVE], 0.01)

    def residual(x: np.ndarray) -> np.ndarray:
        q0, scale, bias = unpack_params(x)
        rows = []
        for q, y in zip(q_enc, tau):
            pred = scale * model.gravity(q + q0) + bias
            rows.append((pred[ACTIVE] - y[ACTIVE]) * weights)
        return np.concatenate(rows)

    x0 = pack_params(initial_zero, np.array(
        [RATIO.get(i + 1, 1.0) for i in range(N_ARM)], dtype=float), np.zeros(N_ARM))
    lower = np.r_[np.full(5, -0.75), np.full(5, 0.2), np.full(5, -2.0)]
    upper = np.r_[np.full(5, 0.75), np.full(5, 3.0), np.full(5, 2.0)]
    sol = least_squares(
        residual, x0, bounds=(lower, upper), loss="soft_l1",
        f_scale=1.0, max_nfev=400)
    q0, scale, bias = unpack_params(sol.x)
    raw_res = []
    for q, y in zip(q_enc, tau):
        pred = scale * model.gravity(q + q0) + bias
        raw_res.append(pred[ACTIVE] - y[ACTIVE])
    raw_res = np.asarray(raw_res)
    return dict(
        ok=bool(sol.success),
        message=str(sol.message),
        cost=float(sol.cost),
        nfev=int(sol.nfev),
        zero_offset_rad=q0.tolist(),
        zero_offset_deg=np.degrees(q0).tolist(),
        scale=scale.tolist(),
        bias_nm=bias.tolist(),
        residual_rms_nm=np.sqrt(np.mean(raw_res ** 2, axis=0)).tolist(),
        residual_max_abs_nm=np.max(np.abs(raw_res), axis=0).tolist(),
        residual_all_rms_nm=float(np.sqrt(np.mean(raw_res ** 2))))


def describe_plan(model: GravityModel, poses: np.ndarray,
                  dwell: float, approach_delta: float) -> dict:
    spans = np.max(np.abs(np.diff(poses, axis=0)), axis=1) if len(poses) > 1 else []
    travel_seconds = float(np.sum(np.maximum(3.0, np.asarray(spans) / 0.20)))
    capture_minutes = float(len(poses) * 2 * dwell / 60.0)
    approach_minutes = float(len(poses) * 2 * 3.0 / 60.0)
    return dict(
        n_poses=int(len(poses)),
        q_min_rad=np.min(poses, axis=0).tolist(),
        q_max_rad=np.max(poses, axis=0).tolist(),
        max_segment_span_rad=float(np.max(spans)) if len(poses) > 1 else 0.0,
        estimated_capture_minutes=capture_minutes,
        estimated_approach_minutes=approach_minutes,
        estimated_total_minutes=capture_minutes + approach_minutes +
        travel_seconds / 60.0,
        safety=check_plan_safety(model, poses, approach_delta))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--points", type=int, default=DEFAULT_POINTS)
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED)
    ap.add_argument("--dwell", type=float, default=DEFAULT_DWELL)
    ap.add_argument("--approach-delta", type=float, default=DEFAULT_APPROACH_DELTA)
    ap.add_argument("--q6", type=float, default=float(SAFE_WAYPOINT[5]))
    ap.add_argument("--max-attempts", type=int, default=10000)
    ap.add_argument("--plan-only", action="store_true")
    args = ap.parse_args()

    model = GravityModel()
    poses, gen = sample_safe_poses(
        model, args.points, args.seed, args.q6, args.max_attempts,
        args.approach_delta)
    plan = describe_plan(model, poses, args.dwell, args.approach_delta)
    plan["generation"] = gen
    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return 0
    if rclpy is None:
        print(f"rclpy unavailable: {_RCLPY_ERROR}", file=sys.stderr)
        return 2

    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    rclpy.init()
    node = Calibrator()
    samples = []
    try:
        if not node.wait_state():
            print("no /joint_states or torque feedback", file=sys.stderr)
            return 3
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 4
        node.get_logger().info(
            f"multi-pose zero calibration: {len(poses)} poses, "
            f"dwell={args.dwell:.2f}s, approach_delta={args.approach_delta:.3f}")
        for i, target in enumerate(poses, 1):
            node.get_logger().info(f"pose {i}/{len(poses)}")
            sample = capture_pose(node, target, args.dwell, args.approach_delta)
            samples.append(sample)
            node.get_logger().info(
                "  hysteresis mean %.4f Nm" %
                sample["tau_hysteresis_abs_mean_nm"])
        fit = fit_offsets(model, samples, load_existing_zero())
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            q_model="q_encoder + zero_offset",
            fitted_joints="J1-J5",
            j6="not fitted",
            writes_yaml=False),
        plan=plan,
        initial_zero_rad=load_existing_zero().tolist(),
        samples=samples,
        fit=fit)
    path = OUT / f"phase3_multi_pose_zero_{stamp}.json"
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps(dict(result=str(path), fit=fit), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
