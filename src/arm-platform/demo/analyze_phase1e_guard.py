#!/usr/bin/env python3
"""阶段1E: 用阶段1D 真实 D0(gain=0.7)原始数据重算 expected-residual 守卫量并推导阈值。

  python3 analyze_phase1e_guard.py \
      --metrics data/ab_runs_1d/metrics_p1d_r1_d0.json \
                data/ab_runs_1d/metrics_p1d_r4_d0.json \
                data/ab_runs_1d/metrics_p1d_r5_d0.json \
      [--out data/ab_runs_1e/phase1e_d0_residual_analysis.json]

输入为**显式**的三轮 D0 metrics JSON(不做"最新文件"选择);loop CSV / 参数快照路径、
轨迹配置、URDF 均从 metrics 的冻结引用中读取。

新守卫定义(与 move_base_circle_mpc_ik_demo guard_mode=expected_residual 一致):
  t_ref[k]   = t_solve[k] + reference_lead_steps*dt   (CSV 已记录 t_reference)
  p_cmd^W[k] = p_base^W(t_ref[k]) + R_base^W(t_ref[k]) @ FK_base(q_pub[k])
  p_ref^W[k] = p_desired^W(t_ref[k])
  e[k]       = p_cmd^W[k] - p_ref^W[k]
  r_pos[k]   = ||e[k] - e[k-1]||   ==   ||Δp_cmd - Δp_ref||   (逐拍核对两种算法相等)

阈值形式: r_allow = r_abs + r_rel*||Δp_ref|| + r_acc*||a_ref||*dt² (+ uncertainty, demo=0)
推导口径(全部打印来源,不反向放宽,也不重复计入同一条尾巴):
  r_abs = ceil_0.1mm( max(SAFETY_ABS × P99.9(r_pos), SAFETY_MAX × max(r_pos), ABS_FLOOR_M) )
          —— 覆盖 D0 实测残差全部尾巴(D0 是单一速度工作点,corr(r_pos,v_ref)≈0.06,
          残差与速度基本无关,故尾巴全部归入 abs 项,不摊给 rel 项重复计数)。
  r_rel = R_REL_FIXED(建模裕量): **D0 只有一个运行速度,速度比例项不可由该数据辨识**;
          取 0.5(=允许残差随期望位移额外增长 50%)作为比 D0 更快合法运动的余量,
          将来有多速度数据时再按数据重推。D0 工况贡献 ≈ 0.5×2.5mm=1.26mm。
  r_acc = 1.0(解析曲率覆盖项;D0 贡献 ||a_ref||dt²≈0.08mm,量级报告)。
验证: 用推导出的阈值回放三轮 D0 → 误拦截必须为 0,并报告最小裕量;
       同时报告 20mm 异常跳变在 D0 工况下必被拦截(max r_allow 至少 2×余量)。
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from analyze_moving_base_run import TrajectoryModel, FkModel, _load_csv, _col

SAFETY_ABS = 2.0        # r_abs 相对 P99.9 的安全系数(尾巴全部归 abs,不与 rel 重复计)
SAFETY_MAX = 1.5        # r_abs 同时不低于 1.5×max(数据最坏值也留 50% 裕量)
ABS_FLOOR_M = 0.0005    # 最小数值裕量 0.5mm(防 P99.9 过小导致门贴地)
R_REL_FIXED = 0.5       # 速度比例项: D0 单速点不可辨识,取建模裕量(见 docstring)
R_ACC_GAIN = 1.0        # 解析曲率覆盖项


def _pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def _stats(a):
    a = np.asarray(a, dtype=float)
    return {
        "count": int(a.size), "mean": float(np.mean(a)), "rms": float(np.sqrt(np.mean(a * a))),
        "p95": _pct(a, 95), "p99": _pct(a, 99), "p999": _pct(a, 99.9),
        "max": float(np.max(a)),
    }


def _phase_bin_stats(phase01, values, nbins=8):
    out = []
    for b in range(nbins):
        m = (phase01 >= b / nbins) & (phase01 < (b + 1) / nbins)
        out.append({
            "bin": f"[{360.0 * b / nbins:.0f},{360.0 * (b + 1) / nbins:.0f})deg",
            "count": int(np.sum(m)),
            "p99": _pct(values[m], 99) if np.any(m) else float("nan"),
            "max": float(np.max(values[m])) if np.any(m) else float("nan"),
        })
    return out


def desired_accel(tm: TrajectoryModel, t: float, eps: float = 1e-3) -> np.ndarray:
    p_p = tm.ee_target_world(t + eps)
    p_0 = tm.ee_target_world(t)
    p_m = tm.ee_target_world(t - eps)
    return (p_p - 2.0 * p_0 + p_m) / (eps * eps)


def analyze_run(metrics_path: Path):
    meta = json.loads(metrics_path.read_text())
    base = metrics_path.resolve().parent.parent.parent  # .../demo
    loop_csv = base / meta["files"]["loop_csv"].replace("data/", "data/", 1)
    if not loop_csv.exists():
        loop_csv = (metrics_path.parent / Path(meta["files"]["loop_csv"]).name)
    snap_tc = meta["config_fingerprint"]["trajectory_config"]
    urdf = meta["config_fingerprint"]["urdf_path"]
    ee_frame = meta["config_fingerprint"]["ee_frame"]
    dt = float(meta["control_dt"])
    tm = TrajectoryModel(snap_tc)
    fk = FkModel(urdf, ee_frame)

    h, d = _load_csv(loop_csv)
    t_ref = _col(h, d, "t_reference")
    guard_block = _col(h, d, "guard_block")
    legacy_step = _col(h, d, "command_fk_step_m")
    q_pub = np.column_stack([_col(h, d, f"q_pub_{i + 1}") for i in range(7)])

    n = len(t_ref)
    p_cmd = np.empty((n, 3))
    p_ref = np.empty((n, 3))
    for k in range(n):
        p_cmd[k] = tm.base_pose(t_ref[k]) + tm.base_rotation(t_ref[k]) @ fk.fk_base(q_pub[k])
        p_ref[k] = tm.ee_target_world(t_ref[k])
    e = p_cmd - p_ref

    # 两种算法逐拍相等核对: ||e_new - e_last|| == ||Δp_cmd - Δp_ref||
    r_from_e = np.linalg.norm(np.diff(e, axis=0), axis=1)
    r_from_dp = np.linalg.norm(np.diff(p_cmd, axis=0) - np.diff(p_ref, axis=0), axis=1)
    equiv_max = float(np.max(np.abs(r_from_e - r_from_dp))) if n > 1 else 0.0

    dp_ref = np.linalg.norm(np.diff(p_ref, axis=0), axis=1)
    a_ref = np.array([np.linalg.norm(desired_accel(tm, t)) for t in t_ref[1:]])
    v_ref = dp_ref / dt

    k1 = np.arange(1, n)
    return {
        "metrics": str(metrics_path), "loop_csv": str(loop_csv), "ticks": int(n),
        "dt": dt, "guard_block_total": int(np.sum(guard_block)),
        "equiv_max_abs_diff_m": equiv_max,
        "r_pos": r_from_e, "dp_ref": dp_ref, "a_ref_norm": a_ref, "v_ref": v_ref,
        "legacy_step": legacy_step[k1], "t_ref": t_ref[k1],
        "ee_period": tm.ee_period, "base_period": tm.base_period,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--metrics", nargs=3, required=True,
                    help="三轮 D0 metrics JSON(显式路径)")
    ap.add_argument("--out", default="data/ab_runs_1e/phase1e_d0_residual_analysis.json")
    args = ap.parse_args()

    runs = [analyze_run(Path(p)) for p in args.metrics]
    for r in runs:
        if r["guard_block_total"] != 0:
            raise SystemExit(f"{r['metrics']}: D0 数据含守卫拦截({r['guard_block_total']}),"
                             "不是干净基线,拒绝作为阈值推导依据")
        if r["equiv_max_abs_diff_m"] > 1e-12:
            raise SystemExit(f"{r['metrics']}: 两种 r_pos 算法不一致 "
                             f"({r['equiv_max_abs_diff_m']:.3e}m)")

    pool_r = np.concatenate([r["r_pos"] for r in runs])
    pool_dp = np.concatenate([r["dp_ref"] for r in runs])
    pool_a = np.concatenate([r["a_ref_norm"] for r in runs])
    pool_legacy = np.concatenate([r["legacy_step"] for r in runs])
    pool_t = np.concatenate([r["t_ref"] for r in runs])
    dt = runs[0]["dt"]
    ee_period = runs[0]["ee_period"]
    base_period = runs[0]["base_period"]

    ratio = pool_r / np.maximum(pool_dp, 1e-9)

    # ---- 阈值推导(来源全部打印;不以让测试通过为目的调整) ----
    r999 = _pct(pool_r, 99.9)
    rmax = float(np.max(pool_r))
    r_abs_raw = max(SAFETY_ABS * r999, SAFETY_MAX * rmax, ABS_FLOOR_M)
    r_abs = math.ceil(r_abs_raw * 10000.0) / 10000.0          # 向上取整到 0.1mm
    ratio999 = _pct(ratio, 99.9)                              # 仅报告,不用于推导(见 docstring)
    r_rel = R_REL_FIXED
    r_acc = R_ACC_GAIN
    acc_term = float(np.mean(pool_a)) * dt * dt

    # ---- 验证: 回放三轮 D0,推导阈值下误拦截必须为 0 ----
    allow = r_abs + r_rel * pool_dp + r_acc * pool_a * dt * dt
    false_blocks = int(np.sum(pool_r > allow))
    margin = allow - pool_r

    # 拦截能力: 20mm 非期望跳变 => r_pos 增量 ~0.020m,对比最大 r_allow
    max_allow = float(np.max(allow))
    if 0.020 < 2.0 * max_allow:
        print(f"警告: 20mm 注入跳变对 max(r_allow)={max_allow*1000.0:.2f}mm 余量不足 2x")

    corr_v = float(np.corrcoef(pool_r, np.concatenate([r["v_ref"] for r in runs]))[0, 1])
    corr_a = float(np.corrcoef(pool_r, pool_a)[0, 1])

    report = {
        "created_from": [r["metrics"] for r in runs],
        "definition": "r_pos=||e_new-e_last||, e=p_cmd^W(t_ref)-p_desired^W(t_ref), "
                      "t_ref=t_solve+lead*dt; 锚点=上一被接受命令及其参考时刻",
        "equivalence_check_max_abs_diff_m": max(r["equiv_max_abs_diff_m"] for r in runs),
        "per_run": [{k: r[k] for k in ("metrics", "loop_csv", "ticks", "guard_block_total")}
                    | {"r_pos_stats_mm": {k2: v * 1000.0 for k2, v in _stats(r["r_pos"]).items()
                                          if k2 != "count"}}
                    for r in runs],
        "pooled": {
            "r_pos_mm": {k: (v * 1000.0 if k != "count" else v)
                         for k, v in _stats(pool_r).items()},
            "dp_ref_mm": {k: (v * 1000.0 if k != "count" else v)
                          for k, v in _stats(pool_dp).items()},
            "legacy_command_fk_step_mm": {k: (v * 1000.0 if k != "count" else v)
                                          for k, v in _stats(pool_legacy).items()},
            "ratio_r_over_dpref": _stats(ratio),
            "corr_r_pos_vs_v_ref": corr_v,
            "corr_r_pos_vs_a_ref": corr_a,
            "by_ee_phase": _phase_bin_stats((pool_t % ee_period) / ee_period,
                                            pool_r * 1000.0),
            "by_base_phase": _phase_bin_stats((pool_t % base_period) / base_period,
                                              pool_r * 1000.0),
        },
        "threshold_derivation": {
            "formula": "r_allow = r_abs + r_rel*||dp_ref|| + r_acc*||a_ref||*dt^2 + uncertainty(demo=0)",
            "inputs": {"p999_r_pos_mm": r999 * 1000.0, "max_r_pos_mm": rmax * 1000.0,
                       "p999_ratio_report_only": ratio999,
                       "safety_abs": SAFETY_ABS, "safety_max": SAFETY_MAX,
                       "r_rel_fixed_modeling_allowance": R_REL_FIXED,
                       "r_rel_note": "D0 单速点不可辨识速度比例项(corr(r_pos,v)≈0.06),"
                                     "r_rel 为建模裕量而非数据拟合,多速度数据后重推",
                       "abs_floor_mm": ABS_FLOOR_M * 1000.0},
            "derived": {"guard_residual_abs_m": r_abs,
                        "guard_residual_relative_gain": r_rel,
                        "guard_residual_accel_gain": r_acc,
                        "acc_term_at_d0_mm": acc_term * 1000.0},
        },
        "verification": {
            "false_blocks_on_d0_with_derived_thresholds": false_blocks,
            "min_margin_mm": float(np.min(margin)) * 1000.0,
            "max_r_allow_on_d0_mm": max_allow * 1000.0,
            "note_20mm_jump": f"20mm 非期望跳变 >> max(r_allow)={max_allow*1000.0:.2f}mm,必拦",
        },
    }

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False))

    p = report["pooled"]
    print(f"pooled r_pos mm: mean={p['r_pos_mm']['mean']:.4f} rms={p['r_pos_mm']['rms']:.4f} "
          f"p95={p['r_pos_mm']['p95']:.4f} p99={p['r_pos_mm']['p99']:.4f} "
          f"p99.9={p['r_pos_mm']['p999']:.4f} max={p['r_pos_mm']['max']:.4f}")
    print(f"dp_ref mm:      mean={p['dp_ref_mm']['mean']:.4f} max={p['dp_ref_mm']['max']:.4f}")
    print(f"legacy step mm: mean={p['legacy_command_fk_step_mm']['mean']:.4f} "
          f"max={p['legacy_command_fk_step_mm']['max']:.4f} (6mm 门限)")
    print(f"corr(r_pos, v_ref)={corr_v:.3f}  corr(r_pos, a_ref)={corr_a:.3f}")
    d = report["threshold_derivation"]["derived"]
    print(f"推导阈值: r_abs={d['guard_residual_abs_m']*1000.0:.2f}mm "
          f"r_rel={d['guard_residual_relative_gain']} r_acc={d['guard_residual_accel_gain']} "
          f"(acc 项@D0={d['acc_term_at_d0_mm']:.4f}mm)")
    v = report["verification"]
    print(f"验证: D0 误拦截={v['false_blocks_on_d0_with_derived_thresholds']} "
          f"最小裕量={v['min_margin_mm']:.3f}mm max(r_allow)={v['max_r_allow_on_d0_mm']:.3f}mm")
    print(f"报告: {out}")
    if v["false_blocks_on_d0_with_derived_thresholds"] != 0:
        raise SystemExit("推导阈值在 D0 上出现误拦截,推导失败(不得反向放宽,须重新审视模型)")


if __name__ == "__main__":
    main()
