#!/usr/bin/env python3
"""Pure-numpy B0--B4 proxy evaluation for the analytical uncertainty stack.

This script validates estimator/decision behavior without ROS, Pinocchio or
ACADOS.  The reported error is a disturbance-compensation proxy rather than an
arm closed-loop tracking result; the ROS experiment remains the final gate.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

DEMO_DIR = Path(__file__).resolve().parent
if str(DEMO_DIR) not in sys.path:
    sys.path.insert(0, str(DEMO_DIR))

from base_trust_estimator import (  # noqa: E402
    BaseTrustEstimator,
    TrustFeedforwardGate,
    UncertaintyAwareFeedforwardDecision,
)

DT = 0.02
MODES = ("b0", "b1", "b2", "b3", "b4")


def raised_cosine(t: float, center: float, width: float) -> float:
    x = abs(float(t) - float(center))
    if x >= 0.5 * width:
        return 0.0
    return 0.5 * (1.0 + np.cos(2.0 * np.pi * x / width))


def disturbance(t: float) -> np.ndarray:
    """Low-frequency drift + short impulses + bounded persistent bias."""
    frequencies = np.exp(np.linspace(np.log(0.15), np.log(1.0), 5))
    background = np.array([
        0.004 * np.sin(2.0 * np.pi * frequencies[0] * t),
        0.003 * np.cos(2.0 * np.pi * frequencies[2] * t + 0.2),
        0.002 * np.sin(2.0 * np.pi * frequencies[3] * t + 0.4),
    ])
    gust = (
        raised_cosine(t, 8.0, 0.6) * np.array([0.010, -0.005, 0.004])
        + raised_cosine(t, 23.0, 0.8) * np.array([-0.008, 0.006, 0.003])
    )
    persistent = (
        np.array([0.010, -0.004, 0.003])
        if 12.0 <= t < 18.0 else np.zeros(3))
    return background + gust + persistent


def make_estimator() -> BaseTrustEstimator:
    return BaseTrustEstimator(
        n_freq=5,
        fmin_hz=0.15,
        fmax_hz=1.0,
        n_peaks=5,
        buffer_sec=20.0,
        min_ident_sec=100.0,  # isolate online RLS + fast regime behavior
        lam=0.995,
        sigma0=0.002,
        meas_delay_sec=0.04,
        dt_nominal=DT,
        regime_enabled=True,
        cal_horizon_steps=5,
        n_cal=100,
    )


def run_mode(mode: str, seed: int, duration: float) -> dict:
    estimator = make_estimator()
    legacy_gate = TrustFeedforwardGate()
    decision = UncertaintyAwareFeedforwardDecision(risk_budget_m=0.012)
    rng = np.random.default_rng(seed)
    errors = []
    loop_cost_ms = []
    gains = []
    speed_scales = []
    persistent_probabilities = []
    coverages = []
    warmup = 6.0
    for index in range(int(round(duration / DT))):
        t = index * DT
        measurement = (
            disturbance(t - 0.04) + rng.normal(0.0, 0.002, 3))
        started = time.perf_counter()
        diagnostic = estimator.step(t, measurement)
        stats = estimator.predict_horizon_stats([t + DT])
        harmonic = stats["harmonic_mean"]
        fast = stats["fast_mean"]
        full = stats["mean"]
        if mode == "b0":
            prediction = np.zeros_like(full)
            gain = 0.0
            speed_scale = 1.0
        elif mode == "b1":
            prediction = harmonic
            gain = 1.0
            speed_scale = 1.0
        elif mode == "b2":
            gain = legacy_gate.update(diagnostic["trust"], DT)
            prediction = gain * harmonic
            speed_scale = 1.0
        elif mode == "b3":
            prediction = full
            gain = 1.0
            speed_scale = 1.0
        elif mode == "b4":
            decision_diagnostic = decision.update(
                harmonic, fast, stats["covariance"], stats["delta"],
                diagnostic["parameter_applicability"], DT)
            prediction = decision_diagnostic["compensated_prediction"]
            gain = decision_diagnostic["feedforward_gain"]
            speed_scale = decision_diagnostic["speed_scale"]
        else:
            raise ValueError(f"unsupported mode: {mode}")
        loop_cost_ms.append(1.0e3 * (time.perf_counter() - started))
        if t < warmup:
            continue
        truth = disturbance(t + DT)
        error = truth - prediction[0]
        errors.append(error)
        gains.append(gain)
        speed_scales.append(speed_scale)
        persistent_probabilities.append(
            diagnostic["persistent_probability"])
        coverages.append(float(np.max(np.abs(truth - full[0]))
                               <= stats["delta"]))
    errors = np.asarray(errors)
    return {
        "mode": mode,
        "seed": int(seed),
        "proxy_axis_rmse_mm": float(
            1.0e3 * np.sqrt(np.mean(np.square(errors)))),
        "proxy_vector_peak_mm": float(
            1.0e3 * np.max(np.linalg.norm(errors, axis=1))),
        "mean_feedforward_gain": float(np.mean(gains)),
        "mean_speed_scale": float(np.mean(speed_scales)),
        "max_persistent_probability": float(
            np.max(persistent_probabilities)),
        "empirical_one_step_coverage": float(np.mean(coverages)),
        "estimator_decision_p99_ms": float(
            np.percentile(loop_cost_ms, 99)),
    }


def aggregate(rows: list[dict]) -> dict[str, dict]:
    result = {}
    numeric_keys = [
        key for key in rows[0]
        if key not in ("mode", "seed")
    ]
    for mode in MODES:
        subset = [row for row in rows if row["mode"] == mode]
        result[mode] = {
            key: float(np.mean([row[key] for row in subset]))
            for key in numeric_keys
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    seeds = [int(item) for item in args.seeds.split(",") if item.strip()]
    if not seeds or args.duration <= 18.0:
        raise ValueError("need at least one seed and duration > 18 s")
    rows = [
        run_mode(mode, seed, args.duration)
        for seed in seeds for mode in MODES
    ]
    summary = aggregate(rows)
    acceptance = {
        "b3_beats_b1_proxy_rmse": (
            summary["b3"]["proxy_axis_rmse_mm"]
            < summary["b1"]["proxy_axis_rmse_mm"]),
        "b4_beats_b2_proxy_rmse": (
            summary["b4"]["proxy_axis_rmse_mm"]
            < summary["b2"]["proxy_axis_rmse_mm"]),
        "b4_peak_not_worse_than_b2": (
            summary["b4"]["proxy_vector_peak_mm"]
            <= summary["b2"]["proxy_vector_peak_mm"]),
        "persistent_mode_activates": (
            summary["b4"]["max_persistent_probability"] > 0.8),
        "fast_path_p99_under_5ms": (
            summary["b4"]["estimator_decision_p99_ms"] < 5.0),
    }
    report = {
        "scope": (
            "pure-numpy disturbance-compensation proxy; "
            "not a ROS arm closed-loop result"),
        "dt_sec": DT,
        "duration_sec": args.duration,
        "seeds": seeds,
        "summary": summary,
        "acceptance": acceptance,
        "rows": rows,
    }
    text = json.dumps(report, ensure_ascii=False, indent=2)
    print(text)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    return 0 if all(acceptance.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
