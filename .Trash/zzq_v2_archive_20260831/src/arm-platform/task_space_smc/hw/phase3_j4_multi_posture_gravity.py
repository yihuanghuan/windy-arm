#!/usr/bin/env python3
"""J4 Phase 3 multi-posture bidirectional gravity sweep.

This is a diagnostic wrapper around the existing Phase 3 algorithm.  It does
not change the fitting algorithm and does not write any YAML.  For each safe
upstream posture it runs repeated forward/reverse J4 sweeps, then compares
A/stiction, resid/A, zero offset and ratio repeatability.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

try:
    import rclpy
except ImportError as exc:
    rclpy = None
    _IMPORT_ERROR = exc

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw import phase3_gravity_calibration as phase3
from task_space_smc.hw.fk_gravity_verify import (
    GravityModel, N_ARM, check_setpoint, fit_sine,
)


OUT = Path(__file__).resolve().parent / "data" / "phase3_j4_multi_posture"
DEFAULT_CANDIDATE = Path(__file__).resolve().parents[2] / "config" / "arm6_calibration_candidate.yaml"
DEFAULT_POSTURES = (
    "0.0400,0.3500,-0.3500,0.0000,1.5500,1.4200;"
    "0.0400,0.6500,-0.6500,0.0000,1.5500,1.4200;"
    "0.0400,0.8500,-0.8500,0.0000,1.5500,1.4200"
)
J4 = 4


def parse_vec(text: str) -> np.ndarray:
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != N_ARM:
        raise ValueError(f"expected {N_ARM} values")
    return np.asarray(vals, dtype=float)


def parse_postures(text: str) -> list[np.ndarray]:
    return [parse_vec(chunk) for chunk in text.split(";") if chunk.strip()]


def model_amplitude(model: GravityModel, posture: np.ndarray,
                    lo: float, hi: float, points: int) -> dict:
    qs = np.linspace(lo, hi, points)
    tau = []
    margins = []
    unsafe = []
    for q4 in qs:
        q = posture.copy()
        q[J4 - 1] = q4
        rep = check_setpoint(q, model)
        margin = phase3.desk_margin(model, q)
        if not rep.ok or margin < 0.10:
            unsafe.append(dict(q4=float(q4), margin=float(margin),
                               reasons=list(rep.reasons)))
        tau.append(float(model.gravity(q)[J4 - 1]))
        margins.append(float(margin))
    fit = fit_sine(qs, np.asarray(tau, dtype=float))
    stiction = phase3.STICTION[J4]
    return dict(
        safe=not unsafe,
        unsafe=unsafe,
        A_model=float(fit["amplitude"]),
        A_model_over_stiction=float(fit["amplitude"] / stiction) if stiction else None,
        model_resid_over_A=float(fit["rms_residual"] / fit["amplitude"]) if fit["amplitude"] else None,
        min_margin=float(min(margins)),
        lo=float(lo),
        hi=float(hi),
        points=int(points),
    )


def set_j4_plan(posture: np.ndarray, lo: float, hi: float,
                points: int, dwell: float) -> None:
    phase3.SWEEP_PLAN[J4] = dict(
        posture=posture.tolist(),
        lo=float(lo),
        hi=float(hi),
        amp=0.0,
        swing=0.0,
        points=int(points),
        dwell=float(dwell),
        sigma=0.0348,
    )
    phase3.POINTS[J4] = int(points)
    phase3.DWELL[J4] = float(dwell)


def summarize_repeats(repeats: list[dict], tol_deg: float) -> dict:
    ok = [r for r in repeats if r.get("ok")]
    if not ok:
        return dict(ok=False, reason="no valid repeats")
    zero = np.asarray([r["zero_offset_deg"] for r in ok], dtype=float)
    ratio = np.asarray([r["ratio"] for r in ok], dtype=float)
    a = np.asarray([r["A"] for r in ok], dtype=float)
    resid = np.asarray([r["resid_over_A"] for r in ok], dtype=float)
    stiction = phase3.STICTION[J4]
    offset_span = float(np.max(zero) - np.min(zero)) if zero.size else float("nan")
    ratio_cv = (
        float(np.std(ratio, ddof=1) / abs(np.mean(ratio)))
        if ratio.size > 1 and abs(float(np.mean(ratio))) > 1e-9 else 0.0)
    converged = bool(offset_span <= tol_deg)
    accepted = bool(
        converged
        and float(np.mean(a) / stiction) >= phase3.MIN_A_OVER_STICTION
        and float(np.max(resid)) <= phase3.MAX_RESID_OVER_A
        and ratio_cv <= phase3.MAX_RATIO_CV
    )
    reasons = []
    if not converged:
        reasons.append(f"zero span={offset_span:.3f}deg > {tol_deg:.3f}deg")
    if float(np.mean(a) / stiction) < phase3.MIN_A_OVER_STICTION:
        reasons.append(
            f"A/stiction={float(np.mean(a) / stiction):.2f} < "
            f"{phase3.MIN_A_OVER_STICTION:.2f}")
    if float(np.max(resid)) > phase3.MAX_RESID_OVER_A:
        reasons.append(
            f"max resid/A={float(np.max(resid)):.3f} > "
            f"{phase3.MAX_RESID_OVER_A:.3f}")
    if ratio_cv > phase3.MAX_RATIO_CV:
        reasons.append(f"ratio CV={ratio_cv:.3f} > {phase3.MAX_RATIO_CV:.3f}")
    return dict(
        ok=True,
        accepted=accepted,
        reasons=reasons,
        n=len(ok),
        zero_offset_deg_mean=float(np.mean(zero)),
        zero_offset_deg_std=float(np.std(zero, ddof=1)) if zero.size > 1 else 0.0,
        zero_offset_deg_span=offset_span,
        ratio_mean=float(np.mean(ratio)),
        ratio_std=float(np.std(ratio, ddof=1)) if ratio.size > 1 else 0.0,
        ratio_cv=ratio_cv,
        A_mean=float(np.mean(a)),
        A_over_stiction_mean=float(np.mean(a) / stiction),
        resid_over_A_max=float(np.max(resid)),
        resid_over_A_mean=float(np.mean(resid)),
    )


def print_plan(postures: list[np.ndarray], lo: float, hi: float,
               points: int, dwell: float, repeats: int) -> int:
    model = GravityModel()
    print("=" * 88)
    print("PLAN only: J4 multi-posture Phase3 bidirectional sweep")
    print(f"candidate ratio/stiction: ratio={phase3.RATIO[J4]:.4f}, "
          f"stiction={phase3.STICTION[J4]:.4f} Nm")
    total = 0.0
    refused = False
    for i, posture in enumerate(postures):
        amp = model_amplitude(model, posture, lo, hi, points)
        total += repeats * 2 * points * (dwell + 3.0) / 60.0
        print(f"posture{i}: {np.round(posture, 4).tolist()} "
              f"model A/stiction={amp['A_model_over_stiction']:.2f} "
              f"A={amp['A_model']:.4f} min_margin={amp['min_margin']:+.4f} "
              f"safe={amp['safe']}")
        if not amp["safe"]:
            refused = True
            for u in amp["unsafe"][:5]:
                print(f"  unsafe q4={u['q4']:+.3f} margin={u['margin']:+.4f} "
                      f"{u['reasons']}")
    print(f"J4 sweep: [{lo:+.2f}, {hi:+.2f}] rad, {points} points, "
          f"forward+reverse, repeats={repeats}, dwell={dwell:.1f}s")
    print(f"estimated dwell/move time: {total:.1f} min")
    print("official YAML will not be written")
    print("=" * 88)
    return 3 if refused else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--calibration-yaml", default=str(DEFAULT_CANDIDATE))
    ap.add_argument("--postures", default=DEFAULT_POSTURES)
    ap.add_argument("--lo", type=float, default=-3.0)
    ap.add_argument("--hi", type=float, default=3.0)
    ap.add_argument("--points", type=int, default=13)
    ap.add_argument("--dwell", type=float, default=7.5)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--tol-deg", type=float, default=0.4)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    phase3.load_candidate_globals(args.calibration_yaml)
    postures = parse_postures(args.postures)
    if args.dry_run:
        return print_plan(postures, args.lo, args.hi,
                          args.points, args.dwell, args.repeats)
    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    OUT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    rclpy.init()
    node = phase3.Calibrator()
    results = []
    try:
        if not node.wait_state():
            print("no /joint_states or torque feedback", file=sys.stderr)
            return 4
        why = node.check_effort_enabled()
        if why:
            print(f"REFUSED: {why}", file=sys.stderr)
            return 5
        for pi, posture in enumerate(postures):
            set_j4_plan(posture, args.lo, args.hi, args.points, args.dwell)
            amp = model_amplitude(node.model, posture, args.lo, args.hi,
                                  args.points)
            if not amp["safe"]:
                results.append(dict(posture_index=pi, posture=posture.tolist(),
                                    ok=False, reason="unsafe plan",
                                    model_signal=amp, repeats=[]))
                break
            reps = []
            node.get_logger().info(
                f"===== J4 posture{pi}: {np.round(posture, 4).tolist()} =====")
            for ri in range(args.repeats):
                node.get_logger().info(
                    f"--- J4 posture{pi} repeat {ri + 1}/{args.repeats} ---")
                r = phase3.run_sweep(node, J4, {}, args.points, args.dwell,
                                     bidirectional=True)
                r["repeat"] = ri + 1
                reps.append(r)
                if not r.get("ok"):
                    node.get_logger().error(
                        f"J4 posture{pi} repeat{ri + 1}: {r.get('reason')}")
                    break
                node.get_logger().info(
                    f"J4 posture{pi} repeat{ri + 1}: "
                    f"A/stiction={r['A'] / phase3.STICTION[J4]:.2f} "
                    f"ratio={r['ratio']:.3f} "
                    f"offset={r['zero_offset_deg']:+.3f}deg "
                    f"resid/A={r['resid_over_A']:.3f}")
            results.append(dict(
                posture_index=pi,
                posture=posture.tolist(),
                model_signal=amp,
                repeats=reps,
                summary=summarize_repeats(reps, args.tol_deg),
            ))
            if reps and not reps[-1].get("ok"):
                break
    finally:
        path = OUT / f"phase3_j4_multi_posture_{stamp}.json"
        path.write_text(json.dumps(
            dict(calibration_yaml=args.calibration_yaml,
                 lo=args.lo, hi=args.hi, points=args.points, dwell=args.dwell,
                 repeats=args.repeats, results=results),
            indent=2, ensure_ascii=False, default=float))
        print(f"\nresults -> {path}")
        print("\nposture accepted A/stiction zero_mean zero_span ratio_mean ratio_cv resid/A_max")
        for r in results:
            s = r.get("summary") or {}
            print(f"p{r['posture_index']} {s.get('accepted')} "
                  f"{s.get('A_over_stiction_mean')} "
                  f"{s.get('zero_offset_deg_mean')} "
                  f"{s.get('zero_offset_deg_span')} "
                  f"{s.get('ratio_mean')} {s.get('ratio_cv')} "
                  f"{s.get('resid_over_A_max')}")
            if s.get("reasons"):
                print("  reject:", "; ".join(s["reasons"]))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
