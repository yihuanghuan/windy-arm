#!/usr/bin/env python3
"""Constrained offline fit for q0, gravity ratio and directional friction.

Inputs are JSON files from phase5_constant_velocity_residual.py.  Inertia stays
fixed inside the Pinocchio model; only selected joints' q0, gravity ratio, F+
and F- are optimized.  Friction is constrained non-negative.  Validation is
done on an independent profile when possible.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.core.model import load_arm6_calibration
from task_space_smc.hw.fk_gravity_verify import GravityModel, N_ARM


def rnea_scaled(model: GravityModel, q_model: np.ndarray, qd: np.ndarray,
                qdd: np.ndarray, ratio: np.ndarray) -> np.ndarray:
    tau = pin.rnea(model.model, model.data, q_model, qd, qdd).copy()
    g = model.gravity(q_model)
    return (tau - g) + ratio * g


def rows_from_files(paths: list[Path], trim_frac: float) -> list[dict]:
    rows = []
    for path in paths:
        data = json.loads(path.read_text())
        for run_id, run in enumerate(data.get("runs", [])):
            if not run.get("ok"):
                continue
            raw = run.get("rows", [])
            n = len(raw)
            if n < 10:
                continue
            k0 = int(trim_frac * n)
            k1 = max(k0 + 3, int((1.0 - trim_frac) * n))
            q = np.asarray([r["q"] for r in raw], dtype=float)
            qd = np.asarray([r["qd"] for r in raw], dtype=float)
            tau = np.asarray([r["tau"] for r in raw], dtype=float)
            t = np.asarray([r["t"] for r in raw], dtype=float)
            qdd = np.gradient(qd, t, axis=0, edge_order=1)
            for i in range(k0, k1):
                rows.append(dict(source=str(path), run_id=run_id,
                                 profile=run.get("profile"),
                                 joint=int(run.get("joint")),
                                 direction=int(run.get("direction")),
                                 q=q[i].tolist(), qd=qd[i].tolist(),
                                 qdd=qdd[i].tolist(), tau=tau[i].tolist()))
    return rows


def split_train_val(rows: list[dict], validation_profile: str | None) -> tuple[list[dict], list[dict]]:
    profiles = sorted({r["profile"] for r in rows})
    if validation_profile is None and len(profiles) > 1:
        validation_profile = profiles[-1]
    if validation_profile is None:
        train = [r for i, r in enumerate(rows) if i % 2 == 0]
        val = [r for i, r in enumerate(rows) if i % 2 == 1]
    else:
        train = [r for r in rows if r["profile"] != validation_profile]
        val = [r for r in rows if r["profile"] == validation_profile]
    return train, val


class FitPack:
    def __init__(self, active: list[int], q0_init: np.ndarray, ratio_init: np.ndarray):
        self.active = active
        self.q0_init = q0_init.copy()
        self.ratio_init = ratio_init.copy()
        self.n = len(active)

    def pack0(self) -> np.ndarray:
        return np.r_[
            self.q0_init[self.active],
            self.ratio_init[self.active],
            np.full(self.n, 0.1),
            np.full(self.n, 0.1),
        ]

    def bounds(self, q0_window_rad: float) -> tuple[np.ndarray, np.ndarray]:
        q0_lo = self.q0_init[self.active] - q0_window_rad
        q0_hi = self.q0_init[self.active] + q0_window_rad
        ratio_lo = np.full(self.n, 0.5)
        ratio_hi = np.full(self.n, 1.6)
        f_lo = np.zeros(self.n)
        f_hi = np.full(self.n, 3.0)
        return (np.r_[q0_lo, ratio_lo, f_lo, f_lo],
                np.r_[q0_hi, ratio_hi, f_hi, f_hi])

    def unpack(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        q0 = self.q0_init.copy()
        ratio = self.ratio_init.copy()
        fpos = np.zeros(N_ARM)
        fneg = np.zeros(N_ARM)
        q0[self.active] = x[:self.n]
        ratio[self.active] = x[self.n:2 * self.n]
        fpos[self.active] = x[2 * self.n:3 * self.n]
        fneg[self.active] = x[3 * self.n:4 * self.n]
        return q0, ratio, fpos, fneg


def residual_vector(model: GravityModel, rows: list[dict], pack: FitPack,
                    x: np.ndarray, weights: np.ndarray, v_eps: float) -> np.ndarray:
    q0, ratio, fpos, fneg = pack.unpack(x)
    out = []
    for r in rows:
        q = np.asarray(r["q"], dtype=float)
        qd = np.asarray(r["qd"], dtype=float)
        qdd = np.asarray(r["qdd"], dtype=float)
        tau = np.asarray(r["tau"], dtype=float)
        pred = rnea_scaled(model, q + q0, qd, qdd, ratio)
        for j in pack.active:
            if qd[j] > v_eps:
                pred[j] += fpos[j]
            elif qd[j] < -v_eps:
                pred[j] -= fneg[j]
        out.extend(((pred[pack.active] - tau[pack.active]) * weights).tolist())
    return np.asarray(out)


def metrics(model: GravityModel, rows: list[dict], pack: FitPack, x: np.ndarray,
            v_eps: float) -> dict:
    if not rows:
        return dict(n_samples=0)
    raw = residual_vector(model, rows, pack, x, np.ones(len(pack.active)), v_eps)
    arr = raw.reshape((-1, len(pack.active)))
    return dict(n_samples=len(rows),
                rms_all=float(np.sqrt(np.mean(arr ** 2))),
                rms_by_joint=np.sqrt(np.mean(arr ** 2, axis=0)).tolist(),
                mean_by_joint=np.mean(arr, axis=0).tolist(),
                max_abs_by_joint=np.max(np.abs(arr), axis=0).tolist())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("json", nargs="+", type=Path)
    ap.add_argument("--calibration-yaml", default=str(Path(__file__).resolve().parents[2] / "config" / "arm6_calibration_candidate.yaml"))
    ap.add_argument("--joints", default="2,4")
    ap.add_argument("--validation-profile", default=None)
    ap.add_argument("--trim-frac", type=float, default=0.25)
    ap.add_argument("--q0-window-deg", type=float, default=5.0)
    ap.add_argument("--v-eps", type=float, default=0.005)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    active = [int(x) - 1 for x in args.joints.split(",") if x.strip()]
    cal = load_arm6_calibration(args.calibration_yaml)
    q0_init = np.asarray(cal["zero_offset_rad"], dtype=float)
    ratio_init = np.asarray(cal["first_moment_ratio"], dtype=float)
    rows = rows_from_files(args.json, args.trim_frac)
    train, val = split_train_val(rows, args.validation_profile)
    model = GravityModel()
    pack = FitPack(active, q0_init, ratio_init)
    weights = np.ones(len(active))
    lo, hi = pack.bounds(np.radians(args.q0_window_deg))
    x0 = np.clip(pack.pack0(), lo, hi)
    sol = least_squares(
        lambda x: residual_vector(model, train, pack, x, weights, args.v_eps),
        x0, bounds=(lo, hi), loss="soft_l1", f_scale=0.2, max_nfev=300)
    q0, ratio, fpos, fneg = pack.unpack(sol.x)
    result = dict(
        ok=bool(sol.success),
        message=str(sol.message),
        calibration=args.calibration_yaml,
        active_joints=[j + 1 for j in active],
        train_sources=[str(p) for p in args.json],
        validation_profile=args.validation_profile,
        n_train=len(train),
        n_validation=len(val),
        q0_initial_deg=np.degrees(q0_init).tolist(),
        q0_fit_deg=np.degrees(q0).tolist(),
        ratio_initial=ratio_init.tolist(),
        ratio_fit=ratio.tolist(),
        friction_pos_nm=fpos.tolist(),
        friction_neg_nm=fneg.tolist(),
        train_metrics=metrics(model, train, pack, sol.x, args.v_eps),
        validation_metrics=metrics(model, val, pack, sol.x, args.v_eps),
        cost=float(sol.cost),
        nfev=int(sol.nfev),
        writes_yaml=False,
    )
    out = args.out or (Path(__file__).resolve().parent / "data" / "phase5_constvel" /
                       ("constrained_fit_" + Path(args.json[0]).stem + ".json"))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"results -> {out}")
    for j in active:
        print(f"J{j+1}: q0 {np.degrees(q0_init[j]):+.3f} -> {np.degrees(q0[j]):+.3f} deg, "
              f"ratio {ratio_init[j]:.4f} -> {ratio[j]:.4f}, "
              f"F+ {fpos[j]:.3f}, F- {fneg[j]:.3f}")
    print("train rms:", result["train_metrics"])
    print("validation rms:", result["validation_metrics"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
