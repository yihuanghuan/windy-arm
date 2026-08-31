"""动态零位离线重拟合：正反轨迹配对，分离偶对称动力学和奇对称摩擦。

输入 phase3_dynamic_zero_processed_*.csv：

  forward(u) 与 reverse(1-u) 配对：

    tau_even = 0.5 * (tau_fwd + tau_rev)
    tau_odd  = 0.5 * (tau_fwd - tau_rev)

偶对称项由重力、惯性、Coriolis 组成，用 Pinocchio RNEA 拟合 J1-J5 zero offset。
奇对称项作为摩擦/阻尼诊断，用 tanh(v/v_eps)、v 和常数项回归，不参与写 YAML。
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

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, N_ARM, TAU_NOISE,
)


ACTIVE = np.arange(5)
OUT = Path("src/arm-platform/task_space_smc/hw/data/phase3")
DEFAULT_JSON = (
    "src/arm-platform/task_space_smc/hw/data/phase3/"
    "phase3_dynamic_zero_20260812_163431.json")
DEFAULT_CSV = (
    "src/arm-platform/task_space_smc/hw/data/phase3/"
    "phase3_dynamic_zero_processed_20260812_163431.csv")
STATIC_RUN1 = np.array([
    0.03897077608327311,
    0.08921582544793427,
    0.0017947995480055174,
    0.05389154486890846,
    -0.06592826105455603,
    0.0,
])
STATIC_RUN2 = np.array([
    0.030832876069075605,
    0.0887499133939055,
    0.0231952129303072,
    0.10281641162071224,
    -0.06506409890097394,
    0.0,
])


def rnea_tau(model: GravityModel, q: np.ndarray,
             qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
    return pin.rnea(
        model.model,
        model.data,
        np.asarray(q, dtype=float),
        np.asarray(qd, dtype=float),
        np.asarray(qdd, dtype=float),
    ).copy()


def read_processed(path: Path) -> dict:
    rows = list(csv.DictReader(path.open()))
    out = {}
    for segment in ("forward", "reverse"):
        rr = [r for r in rows if r["segment"] == segment]
        out[segment] = dict(
            t=np.asarray([float(r["t"]) for r in rr], dtype=float),
            q=np.asarray([[float(r[f"q_J{j}"]) for j in range(1, N_ARM + 1)]
                          for r in rr], dtype=float),
            qd=np.asarray([[float(r[f"qdot_J{j}"]) for j in range(1, N_ARM + 1)]
                           for r in rr], dtype=float),
            qdd=np.asarray([[float(r[f"qddot_J{j}"]) for j in range(1, N_ARM + 1)]
                            for r in rr], dtype=float),
            tau=np.asarray([[float(r[f"tau_meas_J{j}"])
                             for j in range(1, N_ARM + 1)]
                            for r in rr], dtype=float),
        )
        out[segment]["t"] -= out[segment]["t"][0]
    return out


def interp_rows(seg: dict, u: np.ndarray) -> dict:
    t = seg["t"]
    phase = t / max(t[-1], 1e-9)
    return dict(
        q=np.column_stack([
            np.interp(u, phase, seg["q"][:, j]) for j in range(N_ARM)]),
        qd=np.column_stack([
            np.interp(u, phase, seg["qd"][:, j]) for j in range(N_ARM)]),
        qdd=np.column_stack([
            np.interp(u, phase, seg["qdd"][:, j]) for j in range(N_ARM)]),
        tau=np.column_stack([
            np.interp(u, phase, seg["tau"][:, j]) for j in range(N_ARM)]),
    )


def make_pairs(data: dict, n_pairs: int, trim_phase: float) -> dict:
    u = np.linspace(trim_phase, 1.0 - trim_phase, n_pairs)
    f = interp_rows(data["forward"], u)
    r = interp_rows(data["reverse"], 1.0 - u)
    tau_even = 0.5 * (f["tau"] + r["tau"])
    tau_odd = 0.5 * (f["tau"] - r["tau"])
    q_even = 0.5 * (f["q"] + r["q"])
    qd_pair = 0.5 * (f["qd"] - r["qd"])
    qdd_even = 0.5 * (f["qdd"] + r["qdd"])
    return dict(
        u=u,
        forward=f,
        reverse=r,
        q_even=q_even,
        qd_pair=qd_pair,
        qdd_even=qdd_even,
        tau_even=tau_even,
        tau_odd=tau_odd,
        pair_mismatch=dict(
            q_rms_rad=np.sqrt(np.mean((f["q"] - r["q"]) ** 2, axis=0)).tolist(),
            q_max_abs_rad=np.max(np.abs(f["q"] - r["q"]), axis=0).tolist(),
            qd_sum_rms=np.sqrt(np.mean((f["qd"] + r["qd"]) ** 2, axis=0)).tolist(),
            qdd_diff_rms=np.sqrt(np.mean((f["qdd"] - r["qdd"]) ** 2, axis=0)).tolist(),
        ),
    )


def residual_even(model: GravityModel, pairs: dict, q0: np.ndarray) -> np.ndarray:
    raw = []
    f = pairs["forward"]
    r = pairs["reverse"]
    for i in range(len(pairs["u"])):
        tau_f = rnea_tau(model, f["q"][i] + q0, f["qd"][i], f["qdd"][i])
        tau_r = rnea_tau(model, r["q"][i] + q0, r["qd"][i], r["qdd"][i])
        pred_even = 0.5 * (tau_f + tau_r)
        raw.append(pred_even[ACTIVE] - pairs["tau_even"][i, ACTIVE])
    return np.asarray(raw)


def fit_even_zero(model: GravityModel, pairs: dict,
                  initial: np.ndarray) -> dict:
    weights = 1.0 / np.maximum(TAU_NOISE[ACTIVE], 0.02)

    def fun(z: np.ndarray) -> np.ndarray:
        q0 = np.zeros(N_ARM)
        q0[ACTIVE] = z
        return (residual_even(model, pairs, q0) * weights).reshape(-1)

    sol = least_squares(
        fun,
        np.asarray(initial, dtype=float)[ACTIVE],
        bounds=(np.full(5, -0.75), np.full(5, 0.75)),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=300,
    )
    q0 = np.zeros(N_ARM)
    q0[ACTIVE] = sol.x
    raw = residual_even(model, pairs, q0)
    return dict(
        ok=bool(sol.success),
        message=str(sol.message),
        n_pairs=int(len(pairs["u"])),
        zero_offset_rad=q0.tolist(),
        zero_offset_deg=np.degrees(q0).tolist(),
        residual_all_rms_nm=float(np.sqrt(np.mean(raw ** 2))),
        residual_rms_nm=np.sqrt(np.mean(raw ** 2, axis=0)).tolist(),
        residual_max_abs_nm=np.max(np.abs(raw), axis=0).tolist(),
        cost=float(sol.cost),
        nfev=int(sol.nfev),
    )


def eval_even_candidate(model: GravityModel, pairs: dict,
                        q0: np.ndarray) -> dict:
    raw = residual_even(model, pairs, np.asarray(q0, dtype=float))
    return dict(
        zero_offset_rad=np.asarray(q0, dtype=float).tolist(),
        zero_offset_deg=np.degrees(np.asarray(q0, dtype=float)).tolist(),
        residual_all_rms_nm=float(np.sqrt(np.mean(raw ** 2))),
        residual_rms_nm=np.sqrt(np.mean(raw ** 2, axis=0)).tolist(),
        residual_max_abs_nm=np.max(np.abs(raw), axis=0).tolist(),
    )


def fit_odd_friction(model: GravityModel, pairs: dict,
                     q0: np.ndarray, v_eps: float) -> dict:
    f = pairs["forward"]
    r = pairs["reverse"]
    model_odd = []
    for i in range(len(pairs["u"])):
        tau_f = rnea_tau(model, f["q"][i] + q0, f["qd"][i], f["qdd"][i])
        tau_r = rnea_tau(model, r["q"][i] + q0, r["qd"][i], r["qdd"][i])
        model_odd.append(0.5 * (tau_f - tau_r))
    model_odd = np.asarray(model_odd)
    odd_resid = pairs["tau_odd"][:, ACTIVE] - model_odd[:, ACTIVE]
    v = pairs["qd_pair"][:, ACTIVE]
    joints = []
    for j in range(len(ACTIVE)):
        A = np.column_stack([
            np.tanh(v[:, j] / v_eps),
            v[:, j],
            np.ones_like(v[:, j]),
        ])
        coef, *_ = np.linalg.lstsq(A, odd_resid[:, j], rcond=None)
        pred = A @ coef
        res = odd_resid[:, j] - pred
        joints.append(dict(
            joint=int(j + 1),
            coulomb_nm=float(coef[0]),
            viscous_nm_per_rad_s=float(coef[1]),
            bias_odd_nm=float(coef[2]),
            odd_raw_rms_nm=float(np.sqrt(np.mean(odd_resid[:, j] ** 2))),
            odd_fit_rms_nm=float(np.sqrt(np.mean(res ** 2))),
            odd_max_abs_nm=float(np.max(np.abs(odd_resid[:, j]))),
        ))
    return dict(v_eps=v_eps, joints=joints)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--csv", default=DEFAULT_CSV)
    ap.add_argument("--n-pairs", type=int, default=1400)
    ap.add_argument("--trim-phase", type=float, default=0.04)
    ap.add_argument("--v-eps", type=float, default=0.03)
    args = ap.parse_args()

    source = json.loads(Path(args.json).read_text())
    data = read_processed(Path(args.csv))
    pairs = make_pairs(data, args.n_pairs, args.trim_phase)
    model = GravityModel()

    initial = np.asarray(source["existing_zero_offset_rad"], dtype=float)
    even_fit = fit_even_zero(model, pairs, initial)
    q0_even = np.asarray(even_fit["zero_offset_rad"], dtype=float)
    candidates = {
        "old_yaml": initial,
        "static_run1": STATIC_RUN1,
        "static_run2": STATIC_RUN2,
        "dynamic_even_fit": q0_even,
    }
    for fit in source.get("fits", []):
        candidates[f"dynamic_{fit['segment']}"] = np.asarray(
            fit["zero_offset_rad"], dtype=float)
    evals = {
        name: eval_even_candidate(model, pairs, q0)
        for name, q0 in candidates.items()
    }
    odd = fit_odd_friction(model, pairs, q0_even, args.v_eps)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"phase3_dynamic_even_zero_{stamp}.json"
    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            source_json=args.json,
            source_csv=args.csv,
            writes_yaml=False,
            method="pair forward(u) with reverse(1-u); fit zero on tau_even only",
        ),
        pair_mismatch=pairs["pair_mismatch"],
        even_fit=even_fit,
        even_candidate_eval=evals,
        odd_friction_fit=odd,
    )
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps(dict(result=str(path), **result), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
