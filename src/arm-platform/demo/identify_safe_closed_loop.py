#!/usr/bin/env python3
"""从已经安全完成的闭环运行中，被动辨识执行器闭环动态。

模型（逐关节、逐运行）:
    q_meas[k+1] = a*q_meas[k] + b*q_pub[k-d] + c + e[k]

只接受稳定、正增益且直流增益合理的模型。该脚本不发布命令，不产生额外激励；
因此适合先估计命令到实测关节的等效延迟/带宽。基座耦合必须依赖真实 IMU/基座
状态列；没有这些列时明确报告 unavailable，不把解析轨迹当作实测响应。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable

import numpy as np


BASE_RESPONSE_COLUMNS = (
    "base_meas_x", "base_meas_y", "base_meas_z",
    "base_meas_roll", "base_meas_pitch", "base_meas_yaw",
)


@dataclass
class JointFit:
    joint: int
    run: str
    sample_time_sec: float
    extra_delay_steps: int
    extra_delay_sec: float
    a: float
    b: float
    intercept: float
    dc_gain: float
    time_constant_sec: float
    bandwidth_hz: float
    rmse_rad: float
    fit_percent: float
    regressor_condition: float


def _finite_rows(rows: list[dict], names: Iterable[str]) -> np.ndarray:
    return np.array([
        all(np.isfinite(float(row[name])) for name in names)
        for row in rows
    ], dtype=bool)


def load_run(path: Path, start_sec: float, end_sec: float | None):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fields = tuple(reader.fieldnames or ())
    required = ["t_solve"] + [
        f"{prefix}_{joint}"
        for joint in range(1, 8)
        for prefix in ("q_meas", "q_pub")
    ]
    missing = [name for name in required if name not in fields]
    if missing:
        raise ValueError(f"{path}: 缺少列 {missing}")
    valid = _finite_rows(rows, required)
    time = np.array([float(row["t_solve"]) for row in rows])
    valid &= time >= start_sec
    if end_sec is not None:
        valid &= time <= end_sec
    if np.count_nonzero(valid) < 50:
        raise ValueError(f"{path}: 有效稳态样本不足 50")
    selected = [row for row, keep in zip(rows, valid) if keep]
    t = np.array([float(row["t_solve"]) for row in selected])
    dt = float(np.median(np.diff(t)))
    if not 0.005 <= dt <= 0.1:
        raise ValueError(f"{path}: 异常采样周期 {dt:.6f}s")
    return selected, fields, dt


def fit_joint(rows: list[dict], run: str, joint: int, dt: float,
              max_delay_steps: int) -> JointFit:
    y = np.array([float(row[f"q_meas_{joint}"]) for row in rows])
    u = np.array([float(row[f"q_pub_{joint}"]) for row in rows])
    candidates = []
    for delay in range(max_delay_steps + 1):
        count = len(y) - delay - 1
        if count < 30:
            continue
        target = y[delay + 1:]
        phi = np.column_stack((y[delay:-1], u[:count], np.ones(count)))
        theta, *_ = np.linalg.lstsq(phi, target, rcond=None)
        a, b, intercept = map(float, theta)
        if not (0.0 < a < 0.9999 and b > 0.0):
            continue
        dc_gain = b / (1.0 - a)
        if not 0.5 <= dc_gain <= 1.5:
            continue
        residual = target - phi @ theta
        rss = max(float(residual @ residual), np.finfo(float).tiny)
        # 延迟越大有效样本越少；用 BIC 而不是直接比较训练 RMSE。
        bic = count * math.log(rss / count) + 3 * math.log(count)
        candidates.append((bic, delay, a, b, intercept, target, phi, residual))
    if not candidates:
        raise RuntimeError(f"{run}: joint{joint} 没有满足稳定/正增益约束的模型")
    _, delay, a, b, intercept, target, phi, residual = min(candidates, key=lambda x: x[0])
    tau = -dt / math.log(a)
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    scale = float(np.sqrt(np.mean((target - np.mean(target)) ** 2)))
    fit_percent = 100.0 * (1.0 - rmse / max(scale, 1e-12))
    return JointFit(
        joint=joint,
        run=run,
        sample_time_sec=dt,
        extra_delay_steps=delay,
        extra_delay_sec=delay * dt,
        a=a,
        b=b,
        intercept=intercept,
        dc_gain=b / (1.0 - a),
        time_constant_sec=tau,
        bandwidth_hz=1.0 / (2.0 * math.pi * tau),
        rmse_rad=rmse,
        fit_percent=fit_percent,
        regressor_condition=float(np.linalg.cond(phi)),
    )


def safety_audit(rows: list[dict], dt: float) -> dict:
    q_pub = np.array([
        [float(row[f"q_pub_{joint}"]) for joint in range(1, 8)]
        for row in rows
    ])
    result = {
        "samples": len(rows),
        "max_abs_published_step_rad": float(np.max(np.abs(np.diff(q_pub, axis=0)))),
        "max_abs_published_velocity_rad_s": float(
            np.max(np.abs(np.diff(q_pub, axis=0))) / dt),
    }
    for name in ("guard_block", "solver_status"):
        if name in rows[0]:
            values = np.array([float(row[name]) for row in rows])
            result[f"nonzero_{name}_count"] = int(np.count_nonzero(values))
    return result


def summarize(fits: list[JointFit]) -> list[dict]:
    output = []
    for joint in range(1, 8):
        group = [fit for fit in fits if fit.joint == joint]
        output.append({
            "joint": joint,
            "runs": len(group),
            "extra_delay_sec_median": float(np.median([x.extra_delay_sec for x in group])),
            "time_constant_sec_median": float(np.median([x.time_constant_sec for x in group])),
            "time_constant_sec_range": [
                float(np.min([x.time_constant_sec for x in group])),
                float(np.max([x.time_constant_sec for x in group])),
            ],
            "bandwidth_hz_median": float(np.median([x.bandwidth_hz for x in group])),
            "dc_gain_median": float(np.median([x.dc_gain for x in group])),
            "fit_percent_median": float(np.median([x.fit_percent for x in group])),
            "rmse_rad_median": float(np.median([x.rmse_rad for x in group])),
        })
    return output


def parse_args():
    parser = argparse.ArgumentParser(description="安全运行数据的被动闭环辨识")
    parser.add_argument("loop_csv", nargs="+", type=Path)
    parser.add_argument("--start-sec", type=float, default=4.0)
    parser.add_argument("--end-sec", type=float, default=40.0)
    parser.add_argument("--max-delay-steps", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    fits: list[JointFit] = []
    audits = []
    has_base_response = True
    for path in args.loop_csv:
        rows, fields, dt = load_run(path, args.start_sec, args.end_sec)
        audits.append({"run": str(path), **safety_audit(rows, dt)})
        has_base_response &= all(name in fields for name in BASE_RESPONSE_COLUMNS)
        for joint in range(1, 8):
            fits.append(fit_joint(rows, path.name, joint, dt, args.max_delay_steps))

    result = {
        "model": "q_meas[k+1] = a*q_meas[k] + b*q_pub[k-d] + c",
        "interpretation": {
            "extra_delay": "d is extra whole-sample transport delay; d=0 still predicts the next sample",
            "bandwidth": "equivalent closed-loop -3 dB bandwidth of the fitted first-order pole",
        },
        "passive_identification": True,
        "safety_audit": audits,
        "joint_summary": summarize(fits),
        "per_run_joint_fits": [asdict(fit) for fit in fits],
        "base_coupling": (
            {"status": "columns_available_but_estimator_not_implemented"}
            if has_base_response else
            {
                "status": "unavailable",
                "reason": "loop CSV has no measured base/IMU response columns; analytic base trajectory is not identification data",
                "required_columns": list(BASE_RESPONSE_COLUMNS),
            }
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"结果: {args.output}")
    for item in result["joint_summary"]:
        print(
            f"joint{item['joint']}: delay={item['extra_delay_sec_median']*1000:.1f}ms "
            f"tau={item['time_constant_sec_median']*1000:.1f}ms "
            f"bw={item['bandwidth_hz_median']:.2f}Hz "
            f"fit={item['fit_percent_median']:.1f}%"
        )
    print(f"base coupling: {result['base_coupling']['status']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
