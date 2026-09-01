"""多姿态零位标定结果的离线可观测性诊断。

输入 phase3_multi_pose_zero_*.json，复用同一个 URDF/Pinocchio 重力模型：

    tau_meas[j] = scale[j] * g_urdf(q_encoder + q0)[j] + bias[j]

诊断内容：
  1. 计算 q0 / gravity scale / bias 的加权残差 Jacobian 列相关性。
  2. 固定指定关节 q0，重新优化其余 q0、scale、bias，得到 profile RMS 曲线。

该脚本只读采样 JSON，只写诊断 JSON/CSV/PNG，不写 arm6_calibration.yaml。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_space_smc.hw.fk_gravity_verify import (  # noqa: E402
    GravityModel, N_ARM, TAU_NOISE,
)


ACTIVE = np.arange(5)
DEFAULT_RUNS = [
    "src/arm-platform/task_space_smc/hw/data/phase3/"
    "phase3_multi_pose_zero_20260812_140426.json",
    "src/arm-platform/task_space_smc/hw/data/phase3/"
    "phase3_multi_pose_zero_20260812_141959.json",
]
OUT_DIR = Path("src/arm-platform/task_space_smc/hw/data/phase3")
Q0_BOUNDS = (-0.75, 0.75)
SCALE_BOUNDS = (0.2, 3.0)
BIAS_BOUNDS = (-2.0, 2.0)
FLAT_ABS_RMS_NM = 0.005
FLAT_SPAN_RAD = 0.10


def load_run(path: Path) -> dict:
    data = json.loads(path.read_text())
    samples = data["samples"]
    return dict(
        name=path.stem,
        path=str(path),
        q=np.asarray([s["q_encoder_rad"] for s in samples], dtype=float),
        tau=np.asarray([s["tau_avg_nm"] for s in samples], dtype=float),
        fit=data["fit"],
        initial_zero=np.asarray(data["initial_zero_rad"], dtype=float),
    )


def pack(q0: np.ndarray, scale: np.ndarray, bias: np.ndarray) -> np.ndarray:
    return np.r_[q0[ACTIVE], scale[ACTIVE], bias[ACTIVE]]


def unpack(x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q0 = np.zeros(N_ARM)
    scale = np.ones(N_ARM)
    bias = np.zeros(N_ARM)
    q0[ACTIVE] = x[:5]
    scale[ACTIVE] = x[5:10]
    bias[ACTIVE] = x[10:15]
    return q0, scale, bias


def param_name(k: int) -> str:
    if k < 5:
        return f"q0_J{k + 1}"
    if k < 10:
        return f"scale_J{k - 4}"
    return f"bias_J{k - 9}"


def seed_from_fit(fit: dict) -> np.ndarray:
    return pack(
        np.asarray(fit["zero_offset_rad"], dtype=float),
        np.asarray(fit["scale"], dtype=float),
        np.asarray(fit["bias_nm"], dtype=float),
    )


def raw_residual_matrix(model: GravityModel, q: np.ndarray,
                        tau: np.ndarray, x: np.ndarray) -> np.ndarray:
    q0, scale, bias = unpack(x)
    pred = []
    for qi in q:
        pred.append(scale * model.gravity(qi + q0) + bias)
    return np.asarray(pred)[:, ACTIVE] - tau[:, ACTIVE]


def weighted_residual(model: GravityModel, q: np.ndarray,
                      tau: np.ndarray, x: np.ndarray) -> np.ndarray:
    raw = raw_residual_matrix(model, q, tau, x)
    return (raw / np.maximum(TAU_NOISE[ACTIVE], 0.01)).reshape(-1)


def raw_rms(model: GravityModel, q: np.ndarray,
            tau: np.ndarray, x: np.ndarray) -> tuple[float, np.ndarray]:
    raw = raw_residual_matrix(model, q, tau, x)
    return float(np.sqrt(np.mean(raw ** 2))), np.sqrt(np.mean(raw ** 2, axis=0))


def bounds() -> tuple[np.ndarray, np.ndarray]:
    lower = np.r_[np.full(5, Q0_BOUNDS[0]), np.full(5, SCALE_BOUNDS[0]),
                  np.full(5, BIAS_BOUNDS[0])]
    upper = np.r_[np.full(5, Q0_BOUNDS[1]), np.full(5, SCALE_BOUNDS[1]),
                  np.full(5, BIAS_BOUNDS[1])]
    return lower, upper


def fit_all(model: GravityModel, q: np.ndarray,
            tau: np.ndarray, x0: np.ndarray) -> np.ndarray:
    lower, upper = bounds()

    def fun(x: np.ndarray) -> np.ndarray:
        return weighted_residual(model, q, tau, x)

    sol = least_squares(
        fun, np.clip(x0, lower, upper), bounds=(lower, upper), max_nfev=400)
    return sol.x


def fit_with_fixed(model: GravityModel, q: np.ndarray, tau: np.ndarray,
                   x_seed: np.ndarray, fixed: dict[int, float]) -> np.ndarray:
    lower, upper = bounds()
    fixed_idx = sorted(fixed)
    free_idx = [i for i in range(len(x_seed)) if i not in fixed_idx]
    x_base = np.clip(np.asarray(x_seed, dtype=float).copy(), lower, upper)
    for i, value in fixed.items():
        x_base[i] = float(value)
    z0 = x_base[free_idx]

    def merge(z: np.ndarray) -> np.ndarray:
        x = x_base.copy()
        x[free_idx] = z
        for i, value in fixed.items():
            x[i] = float(value)
        return x

    def fun(z: np.ndarray) -> np.ndarray:
        return weighted_residual(model, q, tau, merge(z))

    sol = least_squares(
        fun, z0, bounds=(lower[free_idx], upper[free_idx]), max_nfev=250)
    return merge(sol.x)


def numerical_jacobian(model: GravityModel, q: np.ndarray,
                       tau: np.ndarray, x: np.ndarray) -> np.ndarray:
    f0 = weighted_residual(model, q, tau, x)
    jac = np.empty((f0.size, x.size))
    steps = np.r_[np.full(5, 1e-5), np.full(5, 1e-5), np.full(5, 1e-5)]
    for k, h in enumerate(steps):
        xp = x.copy()
        xm = x.copy()
        xp[k] += h
        xm[k] -= h
        jac[:, k] = (
            weighted_residual(model, q, tau, xp)
            - weighted_residual(model, q, tau, xm)
        ) / (2.0 * h)
    return jac


def jacobian_summary(jac: np.ndarray) -> dict:
    norms = np.linalg.norm(jac, axis=0)
    denom = np.outer(norms, norms)
    corr = np.divide(jac.T @ jac, denom, out=np.zeros_like(denom), where=denom > 0)
    sv = np.linalg.svd(jac, compute_uv=False)
    interesting = {}
    for joint in (3, 4):
        q0_i = joint - 1
        entries = []
        for k in range(corr.shape[1]):
            if k == q0_i:
                continue
            entries.append(dict(
                with_param=param_name(k),
                corr=float(corr[q0_i, k]),
                abs_corr=float(abs(corr[q0_i, k])),
            ))
        entries.sort(key=lambda r: r["abs_corr"], reverse=True)
        interesting[f"q0_J{joint}"] = entries[:8]
    q0_scale_bias = []
    for joint in range(1, 6):
        q0_i = joint - 1
        scale_i = 5 + joint - 1
        bias_i = 10 + joint - 1
        q0_scale_bias.append(dict(
            joint=joint,
            corr_q0_scale=float(corr[q0_i, scale_i]),
            corr_q0_bias=float(corr[q0_i, bias_i]),
            q0_column_norm=float(norms[q0_i]),
            scale_column_norm=float(norms[scale_i]),
            bias_column_norm=float(norms[bias_i]),
        ))
    return dict(
        condition_number=float(sv[0] / sv[-1]) if sv[-1] > 0 else float("inf"),
        smallest_singular_value=float(sv[-1]),
        column_norms={param_name(i): float(v) for i, v in enumerate(norms)},
        q0_scale_bias=q0_scale_bias,
        top_q0_correlations=interesting,
    )


def profile_one(model: GravityModel, q: np.ndarray, tau: np.ndarray,
                x_fit: np.ndarray, joint: int, half_width: float,
                points: int) -> list[dict]:
    idx = joint - 1
    lo, hi = bounds()
    center = float(x_fit[idx])
    grid = np.linspace(
        max(lo[idx], center - half_width),
        min(hi[idx], center + half_width),
        points,
    )
    rows = []
    x_seed = x_fit.copy()
    for value in grid:
        x = fit_with_fixed(model, q, tau, x_seed, {idx: float(value)})
        all_rms, per_rms = raw_rms(model, q, tau, x)
        q0, scale, bias = unpack(x)
        rows.append(dict(
            joint=joint,
            fixed_q0_rad=float(value),
            fixed_q0_deg=float(np.degrees(value)),
            rms_nm=all_rms,
            per_joint_rms_nm=per_rms.tolist(),
            scale=scale.tolist(),
            bias_nm=bias.tolist(),
            zero_offset_rad=q0.tolist(),
        ))
        x_seed = x
    return rows


def profile_summary(rows: list[dict], fit_q0: float) -> dict:
    rms = np.asarray([r["rms_nm"] for r in rows], dtype=float)
    q0 = np.asarray([r["fixed_q0_rad"] for r in rows], dtype=float)
    k = int(np.argmin(rms))
    min_rms = float(rms[k])
    near = q0[rms <= min_rms + FLAT_ABS_RMS_NM]
    center_mask = np.abs(q0 - fit_q0) <= FLAT_SPAN_RAD / 2.0
    center_delta = float(np.max(rms[center_mask]) - np.min(rms[center_mask])) \
        if np.any(center_mask) else float("nan")
    return dict(
        min_q0_rad=float(q0[k]),
        min_q0_deg=float(np.degrees(q0[k])),
        min_rms_nm=min_rms,
        rms_at_fit_nearest_nm=float(rms[np.argmin(np.abs(q0 - fit_q0))]),
        flat_band_threshold_nm=FLAT_ABS_RMS_NM,
        flat_band_width_rad=float(np.max(near) - np.min(near)) if near.size else 0.0,
        flat_band_width_deg=float(np.degrees(np.max(near) - np.min(near)))
        if near.size else 0.0,
        rms_span_over_pm_0p05_rad_nm=center_delta,
        judged_flat=bool(center_delta <= FLAT_ABS_RMS_NM),
    )


def write_profile_csv(path: Path, profiles: dict) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "dataset", "joint", "fixed_q0_rad", "fixed_q0_deg", "rms_nm",
            "scale_J3", "scale_J4", "bias_J3_nm", "bias_J4_nm",
            "zero_J3_rad", "zero_J4_rad",
        ])
        for dataset, by_joint in profiles.items():
            for joint, rows in by_joint.items():
                for row in rows:
                    scale = row["scale"]
                    bias = row["bias_nm"]
                    zero = row["zero_offset_rad"]
                    writer.writerow([
                        dataset, joint, row["fixed_q0_rad"], row["fixed_q0_deg"],
                        row["rms_nm"], scale[2], scale[3], bias[2], bias[3],
                        zero[2], zero[3],
                    ])


def plot_profiles(path: Path, profiles: dict, summaries: dict) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    for ax, joint in zip(axes, (3, 4)):
        for dataset, by_joint in profiles.items():
            rows = by_joint[str(joint)]
            x = np.asarray([r["fixed_q0_deg"] for r in rows], dtype=float)
            y = np.asarray([r["rms_nm"] for r in rows], dtype=float)
            ax.plot(x, y, label=dataset)
            s = summaries[dataset][f"J{joint}"]
            ax.axvline(s["min_q0_deg"], linestyle="--", linewidth=0.8)
        ax.set_title(f"J{joint} fixed q0 profile")
        ax.set_xlabel("fixed q0 (deg)")
        ax.set_ylabel("optimized raw RMS (Nm)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs", nargs="*", default=DEFAULT_RUNS)
    ap.add_argument("--scan-half-width", type=float, default=0.18)
    ap.add_argument("--scan-points", type=int, default=91)
    args = ap.parse_args()

    model = GravityModel()
    run_data = [load_run(Path(p)) for p in args.runs]
    combined = dict(
        name="combined_80_pose",
        path=" + ".join(r["path"] for r in run_data),
        q=np.vstack([r["q"] for r in run_data]),
        tau=np.vstack([r["tau"] for r in run_data]),
        fit=None,
    )
    datasets = run_data + [combined]

    x0_combined = np.mean([seed_from_fit(r["fit"]) for r in run_data], axis=0)
    fits = {}
    jac = {}
    profiles = {}
    summaries = {}
    for data in datasets:
        if data["fit"] is None:
            x_fit = fit_all(model, data["q"], data["tau"], x0_combined)
        else:
            x_fit = fit_all(model, data["q"], data["tau"], seed_from_fit(data["fit"]))
        fits[data["name"]] = x_fit
        jac[data["name"]] = jacobian_summary(
            numerical_jacobian(model, data["q"], data["tau"], x_fit))
        profiles[data["name"]] = {}
        summaries[data["name"]] = {}
        for joint in (3, 4):
            rows = profile_one(
                model, data["q"], data["tau"], x_fit, joint,
                args.scan_half_width, args.scan_points)
            profiles[data["name"]][str(joint)] = rows
            summaries[data["name"]][f"J{joint}"] = profile_summary(
                rows, float(x_fit[joint - 1]))

    stamp = time.strftime("%Y%m%d_%H%M%S")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    json_path = OUT_DIR / f"phase3_zero_observability_{stamp}.json"
    csv_path = OUT_DIR / f"phase3_zero_observability_profile_{stamp}.csv"
    png_path = OUT_DIR / f"phase3_zero_observability_profile_{stamp}.png"

    fit_summary = {}
    for name, x in fits.items():
        q0, scale, bias = unpack(x)
        all_rms, per_rms = raw_rms(
            model,
            next(d for d in datasets if d["name"] == name)["q"],
            next(d for d in datasets if d["name"] == name)["tau"],
            x,
        )
        fit_summary[name] = dict(
            zero_offset_rad=q0.tolist(),
            zero_offset_deg=np.degrees(q0).tolist(),
            scale=scale.tolist(),
            bias_nm=bias.tolist(),
            residual_all_rms_nm=all_rms,
            residual_rms_nm=per_rms.tolist(),
        )

    result = dict(
        metadata=dict(
            script=Path(__file__).name,
            runs=[r["path"] for r in run_data],
            writes_yaml=False,
            optimizer="ordinary least squares on weighted residuals",
            profile_method=(
                "fix one q0, re-optimize all other J1-J5 q0 plus all scale/bias"),
            flat_rule=(
                f"flat if RMS span within +/-{FLAT_SPAN_RAD/2:.3f} rad "
                f"is <= {FLAT_ABS_RMS_NM:.3f} Nm"),
        ),
        fit=fit_summary,
        jacobian=jac,
        profile_summary=summaries,
        profiles=profiles,
    )
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    write_profile_csv(csv_path, profiles)
    plot_profiles(png_path, profiles, summaries)
    print(json.dumps(dict(
        result=str(json_path),
        profile_csv=str(csv_path),
        profile_png=str(png_path),
        fit=fit_summary,
        profile_summary=summaries,
        jacobian_focus={
            name: jac[name]["q0_scale_bias"] for name in jac
        },
    ), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
