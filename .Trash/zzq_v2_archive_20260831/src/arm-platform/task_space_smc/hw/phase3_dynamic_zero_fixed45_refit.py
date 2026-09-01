"""离线重拟合：固定 J4/J5 zero，只验证 J1-J3 动态零位稳定性。

输入使用 phase3_dynamic_zero_friction_calibration.py 已采集的双速度、
双轨迹 CSV/JSON。该脚本不跑实机、不写 YAML。

拟合模型：

    tau_meas[j] = RNEA(q_encoder + q0, qdot, qddot)[j]
                  + Fc[j] * tanh(qdot[j] / v_eps)
                  + B[j] * qdot[j]

其中只优化 J1-J3 的 q0/Fc/B；J4/J5 zero 固定为现有配置值，
J6 不参与。额外比较 B 自由与 B>=0 两种结果，用于判断当前负 B
是物理摩擦还是闭环/滤波/未建模项被吸收。
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, N_ARM, TAU_NOISE,
)
from task_space_smc.hw.phase3_dynamic_zero_calibration import (  # noqa: E402
    load_existing_zero, rnea_tau,
)


OUT = Path(__file__).resolve().parent / "data" / "phase3"
FIT = np.arange(3)
EVAL = np.arange(5)


def latest_pair() -> tuple[Path, Path | None]:
    csvs = sorted(OUT.glob("phase3_dynamic_zero_friction_processed_*.csv"))
    if not csvs:
        raise FileNotFoundError(f"no processed friction CSV under {OUT}")
    csv_path = csvs[-1]
    stamp = csv_path.name.removeprefix(
        "phase3_dynamic_zero_friction_processed_").removesuffix(".csv")
    json_path = OUT / f"phase3_dynamic_zero_friction_{stamp}.json"
    return csv_path, json_path if json_path.exists() else None


def load_rows(path: Path, stride: int) -> list[dict]:
    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for idx, raw in enumerate(reader):
            if idx % stride:
                continue
            row = dict(
                segment=raw["segment"],
                family=raw["family"],
                speed=raw["speed"],
                direction=raw["direction"],
                t=float(raw["t"]),
                q=np.asarray([float(raw[f"q_J{i}"]) for i in range(1, N_ARM + 1)]),
                qdot=np.asarray([
                    float(raw[f"qdot_J{i}"]) for i in range(1, N_ARM + 1)]),
                qddot=np.asarray([
                    float(raw[f"qddot_J{i}"]) for i in range(1, N_ARM + 1)]),
                tau=np.asarray([
                    float(raw[f"tau_meas_J{i}"]) for i in range(1, N_ARM + 1)]),
            )
            rows.append(row)
    return rows


def subset(rows: list[dict], *, family: str | None = None,
           speed: str | None = None,
           direction: str | None = None,
           segment: str | None = None) -> list[dict]:
    out = rows
    if family is not None:
        out = [r for r in out if r["family"] == family]
    if speed is not None:
        out = [r for r in out if r["speed"] == speed]
    if direction is not None:
        out = [r for r in out if r["direction"] == direction]
    if segment is not None:
        out = [r for r in out if r["segment"] == segment]
    return out


def pack(q0: np.ndarray, fc: np.ndarray, visc: np.ndarray) -> np.ndarray:
    return np.r_[q0[FIT], fc[FIT], visc[FIT]]


def unpack(x: np.ndarray, fixed_zero: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q0 = np.asarray(fixed_zero, dtype=float).copy()
    fc = np.zeros(N_ARM)
    visc = np.zeros(N_ARM)
    q0[FIT] = x[:3]
    fc[FIT] = x[3:6]
    visc[FIT] = x[6:9]
    return q0, fc, visc


def residual_array(model: GravityModel, rows: list[dict], q0: np.ndarray,
                   fc: np.ndarray, visc: np.ndarray, v_eps: float,
                   joints: np.ndarray) -> np.ndarray:
    raw = []
    for r in rows:
        pred = rnea_tau(model, r["q"] + q0, r["qdot"], r["qddot"])
        pred = pred + fc * np.tanh(r["qdot"] / v_eps) + visc * r["qdot"]
        raw.append(pred[joints] - r["tau"][joints])
    return np.asarray(raw)


def fit_model(model: GravityModel, rows: list[dict], fixed_zero: np.ndarray,
              v_eps: float, nonnegative_b: bool) -> dict:
    if len(rows) < 50:
        return dict(ok=False, reason=f"only {len(rows)} samples")

    weights = 1.0 / np.maximum(TAU_NOISE[FIT], 0.02)
    x0 = pack(fixed_zero, np.zeros(N_ARM), np.zeros(N_ARM))
    lower_b = 0.0 if nonnegative_b else -8.0
    lower = np.r_[np.full(3, -0.75), np.full(3, -4.0), np.full(3, lower_b)]
    upper = np.r_[np.full(3, 0.75), np.full(3, 4.0), np.full(3, 8.0)]

    def residual(x: np.ndarray) -> np.ndarray:
        q0, fc, visc = unpack(x, fixed_zero)
        raw = residual_array(model, rows, q0, fc, visc, v_eps, FIT)
        return (raw * weights).ravel()

    sol = least_squares(
        residual,
        np.clip(x0, lower, upper),
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=320,
    )
    q0, fc, visc = unpack(sol.x, fixed_zero)
    raw_fit = residual_array(model, rows, q0, fc, visc, v_eps, FIT)
    raw_eval = residual_array(model, rows, q0, fc, visc, v_eps, EVAL)
    return dict(
        ok=bool(sol.success),
        message=str(sol.message),
        n_samples=int(len(rows)),
        nonnegative_b=bool(nonnegative_b),
        zero_offset_rad=q0.tolist(),
        zero_offset_deg=np.degrees(q0).tolist(),
        coulomb_nm=fc.tolist(),
        viscous_nm_per_rad_s=visc.tolist(),
        fit_joints="J1-J3",
        residual_fit_all_rms_nm=float(np.sqrt(np.mean(raw_fit ** 2))),
        residual_fit_rms_nm=np.sqrt(np.mean(raw_fit ** 2, axis=0)).tolist(),
        residual_fit_max_abs_nm=np.max(np.abs(raw_fit), axis=0).tolist(),
        residual_eval_j1_j5_all_rms_nm=float(np.sqrt(np.mean(raw_eval ** 2))),
        residual_eval_j1_j5_rms_nm=np.sqrt(np.mean(raw_eval ** 2, axis=0)).tolist(),
        residual_eval_j1_j5_max_abs_nm=np.max(np.abs(raw_eval), axis=0).tolist(),
        cost=float(sol.cost),
        nfev=int(sol.nfev),
    )


def slope_diagnostic(model: GravityModel, rows: list[dict], fit: dict,
                     fixed_zero: np.ndarray, v_eps: float) -> dict:
    if not fit.get("ok"):
        return dict(ok=False, reason="fit failed")
    q0 = np.asarray(fit["zero_offset_rad"], dtype=float)
    fc = np.asarray(fit["coulomb_nm"], dtype=float)
    fc_only = np.zeros(N_ARM)
    fc_only[FIT] = fc[FIT]
    zero_visc = np.zeros(N_ARM)

    raw_no_b = []
    qd = []
    for r in rows:
        pred = rnea_tau(model, r["q"] + q0, r["qdot"], r["qddot"])
        pred = pred + fc_only * np.tanh(r["qdot"] / v_eps)
        raw_no_b.append(r["tau"][FIT] - pred[FIT])
        qd.append(r["qdot"][FIT])
    raw_no_b = np.asarray(raw_no_b)
    qd = np.asarray(qd)

    slopes = []
    corr = []
    for k in range(len(FIT)):
        x = qd[:, k]
        y = raw_no_b[:, k]
        denom = float(np.dot(x, x))
        slopes.append(float(np.dot(x, y) / denom) if denom > 1e-12 else 0.0)
        if np.std(x) > 1e-12 and np.std(y) > 1e-12:
            corr.append(float(np.corrcoef(x, y)[0, 1]))
        else:
            corr.append(0.0)

    return dict(
        ok=True,
        meaning="tau_meas - (RNEA + Coulomb) regressed against qdot; negative slope drives negative B",
        slope_nm_per_rad_s=slopes,
        corr_with_qdot=corr,
        qdot_rms_rad_s=np.sqrt(np.mean(qd ** 2, axis=0)).tolist(),
        residual_without_b_rms_nm=np.sqrt(np.mean(raw_no_b ** 2, axis=0)).tolist(),
        fixed_j4_j5_zero_deg=np.degrees(fixed_zero[3:5]).tolist(),
        fixed_zero_source="existing YAML or source JSON",
        sanity_fixed_zero_rad=fixed_zero.tolist(),
    )


def fit_all(model: GravityModel, rows: list[dict], fixed_zero: np.ndarray,
            v_eps: float, nonnegative_b: bool, include_segments: bool) -> dict:
    specs: list[tuple[str, dict]] = [("combined", {})]
    specs.extend((f"family_{f}", dict(family=f)) for f in ("A", "B"))
    specs.extend((f"speed_{s}", dict(speed=s)) for s in ("slow", "fast"))
    specs.extend((f"direction_{d}", dict(direction=d))
                 for d in ("forward", "reverse"))
    for f in ("A", "B"):
        for s in ("slow", "fast"):
            specs.append((f"{f}_{s}", dict(family=f, speed=s)))
    if include_segments:
        for seg in sorted({r["segment"] for r in rows}):
            specs.append((f"segment_{seg}", dict(segment=seg)))

    fits = {}
    for name, filt in specs:
        rr = subset(rows, **filt)
        fit = fit_model(model, rr, fixed_zero, v_eps, nonnegative_b)
        if name == "combined" and fit.get("ok"):
            fit["negative_b_diagnostic"] = slope_diagnostic(
                model, rr, fit, fixed_zero, v_eps)
        fits[name] = fit
    return fits


def stability(fits: dict) -> dict:
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
            delta_deg=np.degrees(delta).tolist(),
            max_abs_j1_j3_delta_deg=float(np.max(np.abs(np.degrees(delta[FIT])))),
            j1_j3_delta_deg=np.degrees(delta[FIT]).tolist(),
            j4_j5_fixed_delta_deg=np.degrees(delta[3:5]).tolist(),
        )
    if fits.get("combined", {}).get("ok"):
        q = np.asarray(fits["combined"]["zero_offset_rad"], dtype=float)
        out["combined_zero_offset_j1_j3_deg"] = np.degrees(q[FIT]).tolist()
    return out


def source_existing_zero(json_path: Path | None) -> np.ndarray:
    if json_path is not None:
        data = json.loads(json_path.read_text())
        q0 = data.get("existing_zero_offset_rad")
        if q0 is not None and len(q0) == N_ARM:
            return np.asarray(q0, dtype=float)
    return load_existing_zero()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", type=Path, default=None)
    ap.add_argument("--source-json", type=Path, default=None)
    ap.add_argument("--v-eps", type=float, default=0.03)
    ap.add_argument("--stride", type=int, default=5,
                    help="deterministic row decimation for faster offline RNEA fits")
    ap.add_argument("--include-segments", action="store_true",
                    help="also fit every individual segment; slower")
    args = ap.parse_args()
    if args.stride < 1:
        raise ValueError("--stride must be >= 1")

    csv_path, auto_json = latest_pair()
    if args.csv is not None:
        csv_path = args.csv
    json_path = args.source_json if args.source_json is not None else auto_json
    rows = load_rows(csv_path, args.stride)
    fixed_zero = source_existing_zero(json_path)
    model = GravityModel()

    free = fit_all(model, rows, fixed_zero, args.v_eps,
                   nonnegative_b=False, include_segments=args.include_segments)
    nonnegative = fit_all(model, rows, fixed_zero, args.v_eps,
                          nonnegative_b=True,
                          include_segments=args.include_segments)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = OUT / f"phase3_dynamic_zero_fixed45_refit_{stamp}.json"
    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            input_csv=str(csv_path),
            source_json=str(json_path) if json_path is not None else None,
            writes_yaml=False,
            fitted_zero_joints="J1-J3",
            fixed_zero_joints="J4-J5",
            j6="not fitted",
            model="RNEA(q+q0, qdot, qddot) + Fc*tanh(qdot/v_eps) + B*qdot",
            v_eps=args.v_eps,
            stride=args.stride,
            n_rows=len(rows),
        ),
        fixed_zero_offset_rad=fixed_zero.tolist(),
        fixed_zero_offset_deg=np.degrees(fixed_zero).tolist(),
        fits=dict(free_B=free, nonnegative_B=nonnegative),
        stability=dict(free_B=stability(free),
                       nonnegative_B=stability(nonnegative)),
    )
    out_path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps(dict(
        result=str(out_path),
        fixed_j4_j5_zero_deg=np.degrees(fixed_zero[3:5]).tolist(),
        free_B_combined=free["combined"],
        nonnegative_B_combined=nonnegative["combined"],
        free_B_stability=stability(free),
        nonnegative_B_stability=stability(nonnegative),
    ), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
