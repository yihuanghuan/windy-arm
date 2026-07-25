#!/usr/bin/env python3
"""调参器可信度自动对照:T0 离线指标 vs 阶段1D 真实 D0 基准。

标准用法(干净 checkout 可复现,基准来自入库 fixture):
  python3 check_tuner_credibility.py \
      --offline data/tuner_t0/tuner_t0_metrics_gain0.7_<stamp>.json \
      [--offline-gain1 data/tuner_t0/tuner_t0_metrics_gain1_<stamp>.json] \
      [--reference testdata/tuner_phase1d_d0_reference.json]

本机附加交叉核对(原始三轮 metrics 在 gitignored 的 data/ab_runs_1d/):
  python3 check_tuner_credibility.py --offline ... --manifest data/tuner_t0/phase1d_d0_manifest.json

重新生成 fixture(仅在真实 D0 基准变更时;需本机原始数据):
  python3 check_tuner_credibility.py --make-fixture --manifest data/tuner_t0/phase1d_d0_manifest.json

输入路径**显式**(fixture 为入库默认),不做"目录里最新文件"这类不稳定选择。
判定规则(每项输出 offline/real/ratio 与状态):
  - PASS     : ratio ∈ RATIO_PASS_BAND;
  - ADVISORY : ratio ∈ RATIO_FAIL_BAND 之内但出 PASS 带(可接受的模型偏置);
  - FAIL     : ratio 出 RATIO_FAIL_BAND(量级严重不一致)→ 非零退出;
  - guard    : 离线与真实 D0 **都必须为 0**(D0 基线本就无守卫拦截);任一侧非零、
               或两侧同时非零,一律 FAIL —— "两边都有拦截"不是一致性,是两套异常。
  - 元数据校验: offline 指标必须与 fixture 冻结的语义/参数/轨迹工况一致
               (plant_gain=0.7、lead/dq_prev/RTI、h/step/ow/tow、轨迹配置逐字段),
               不一致 → VALIDATION_FAIL 非零退出(比较器不再默认相信调用者传对了文件)。
容差依据: 真实 D0 三轮 run-to-run 波动约 ±10-15%,离线理想化(无反馈时序抖动/DDS 抖动/
测量噪声)系统性偏优约 20-40% —— PASS 带取 (0.5,1.5) 覆盖两者叠加;FAIL 带 (0.2,5.0)
之外意味着量级级别的不一致(定性冲突)。
gain=1.0 输入只输出"模型效度外/不能复现真实失稳",**不参与可用性判定**——
离线调参器不得用于执行器参数评估(阶段1D 已由真实 A/B 裁定)。
"""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

RATIO_PASS_BAND = (0.5, 1.5)
RATIO_FAIL_BAND = (0.2, 5.0)
VEL_PEAK_PASS_BAND = (0.85, 1.15)

DEFAULT_FIXTURE = Path(__file__).resolve().parent / "testdata" / "tuner_phase1d_d0_reference.json"


# ---------------------------------------------------------------- fixture
def extract_round_summary(real: dict, path: str = "") -> dict:
    """从完整 D0 metrics JSON 抽取 fixture 关键指标(字段缺失即抛错,不静默)。"""
    return {
        "label": real["label"],
        "file": path,
        "velocity_feedforward_gain": float(real["velocity_feedforward_gain"]),
        "metrics": {
            "actual_rms_mm": float(real["actual_error"]["steady"]["rms_mm"]),
            "actual_p95_mm": float(real["actual_error"]["steady"]["p95_mm"]),
            "exec_gap_cart_rms_mm":
                float(real["phase1d_executor_match"]["exec_gap_cartesian_mm"]["rms"]),
            "base_1hz_resid_rms_mm":
                float(real["actual_sine_steady"]["base_freq"]["combined_rms_mm"]),
            "vel_peak_rad_s":
                float(real["measured_vel_saturation_steady"]["abs_peak"]),
            "guard_block_count": int(real["guard"]["block_count"]),
        },
    }


def load_manifest_reals(manifest_path: Path):
    manifest = json.loads(manifest_path.read_text())
    real_paths = manifest["real_d0_metrics"]
    if len(real_paths) != 3:
        raise SystemExit(f"manifest 必须给出 3 个真实 D0 metrics 路径,当前 {len(real_paths)}")
    base = manifest_path.resolve().parent
    out = []
    for p in real_paths:
        pth = Path(p)
        if not pth.is_absolute():
            pth = (base / p).resolve()
        out.append((pth, json.loads(pth.read_text())))
    return out


def make_fixture(manifest_path: Path, out_path: Path) -> dict:
    """从本机原始三轮 D0 metrics 生成入库 fixture(关键指标+SHA+工况/语义快照)。"""
    reals = load_manifest_reals(manifest_path)
    tc0 = reals[0][1]["config_fingerprint"]["trajectory_config"]
    ex0 = reals[0][1]["executor_params"]
    rounds = []
    for pth, real in reals:
        if real["config_fingerprint"]["trajectory_config"] != tc0:
            raise SystemExit(f"{pth.name}: trajectory_config 与首轮不一致,拒绝生成 fixture")
        if real["executor_params"] != ex0:
            raise SystemExit(f"{pth.name}: executor_params 与首轮不一致,拒绝生成 fixture")
        if not str(real["label"]).endswith("_d0"):
            raise SystemExit(f"{pth.name}: label={real['label']} 不是 D0 轮,拒绝")
        if abs(float(real["velocity_feedforward_gain"]) - 0.7) > 1e-12:
            raise SystemExit(f"{pth.name}: gain={real['velocity_feedforward_gain']} != 0.7,拒绝")
        summary = extract_round_summary(real, path=str(pth))
        summary["sha256"] = hashlib.sha256(pth.read_bytes()).hexdigest()
        rounds.append(summary)
    fixture = {
        "description": ("阶段1D 真实 D0(gain=0.7)三轮关键指标冻结基准。入库 fixture,"
                        "供 check_tuner_credibility.py 在干净 checkout 上复现自动对照;"
                        "完整原始 metrics 在本机 demo/data/ab_runs_1d/(gitignored),"
                        "sha256 用于本机交叉核对。"),
        "created": "2026-07-23",
        "source": ("由 check_tuner_credibility.py --make-fixture 从 "
                   "data/tuner_t0/phase1d_d0_manifest.json 引用的三轮原始文件抽取"),
        "semantics": {
            "reference_lead_steps": int(reals[0][1]["reference_lead_steps"]),
            "control_dt": float(reals[0][1]["control_dt"]),
            "dq_prev_mode": str(reals[0][1]["dq_prev_mode"]),
            "rti_iteration_mode": str(reals[0][1]["rti_iteration_mode"]),
            "acados_rti_iterations":
                int(reals[0][1]["config_fingerprint"]["ros_params"]["acados_rti_iterations"]),
            "velocity_feedforward_gain": 0.7,
        },
        "production_params": {
            "horizon_steps":
                int(reals[0][1]["config_fingerprint"]["ros_params"]["horizon_steps"]),
            "max_joint_step":
                float(reals[0][1]["config_fingerprint"]["ros_params"]["max_joint_step"]),
            "orientation_weight":
                float(reals[0][1]["config_fingerprint"]["ros_params"]["orientation_weight"]),
            "terminal_orientation_weight":
                float(reals[0][1]["config_fingerprint"]["ros_params"]
                      ["terminal_orientation_weight"]),
        },
        "trajectory_config": tc0,
        "executor_params": ex0,
        "rounds": rounds,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(fixture, indent=2, ensure_ascii=False))
    print(f"fixture 已写入: {out_path}")
    return fixture


# ---------------------------------------------------------------- 元数据校验
def _isclose(a, b) -> bool:
    try:
        return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=1e-9)
    except (TypeError, ValueError):
        return a == b


def validate_offline_metadata(offline: dict, fixture: dict):
    """offline 指标 JSON 必须与 fixture 冻结的语义/参数/工况一致。返回不一致项列表。"""
    fails = []
    sem_f = fixture["semantics"]
    sem_o = offline.get("semantics", {})
    if not _isclose(offline.get("plant_gain"), sem_f["velocity_feedforward_gain"]):
        fails.append(f"plant_gain: offline={offline.get('plant_gain')} != "
                     f"{sem_f['velocity_feedforward_gain']}")
    for key in ("reference_lead_steps", "dq_prev_mode", "rti_iteration_mode",
                "acados_rti_iterations"):
        if sem_o.get(key) != sem_f[key]:
            fails.append(f"semantics.{key}: offline={sem_o.get(key)} != {sem_f[key]}")
    par_f = fixture["production_params"]
    par_o = offline.get("params", {})
    for key, want in par_f.items():
        if not _isclose(par_o.get(key), want):
            fails.append(f"params.{key}: offline={par_o.get(key)} != {want}")
    # plant/守卫阈值/统计窗口也必须与 fixture 冻结值一致(不再是死数据)。
    plant_o = sem_o.get("plant", {}) if isinstance(sem_o.get("plant"), dict) else {}
    ex_f = fixture.get("executor_params", {})
    for off_key, f_val, name in (
            ("fb_tau", ex_f.get("position_feedback_tau"), "plant.fb_tau"),
            ("fb_vel_limit", ex_f.get("feedback_velocity_limit"), "plant.fb_vel_limit"),
            ("max_velocity", ex_f.get("max_velocity"), "plant.max_velocity"),
            ("max_accel", ex_f.get("max_acceleration"), "plant.max_accel")):
        if f_val is not None and not _isclose(plant_o.get(off_key), f_val):
            fails.append(f"semantics.{name}: offline={plant_o.get(off_key)} != {f_val}")
    if not _isclose(sem_o.get("guard_step_warn_m"), 0.006):
        fails.append(f"semantics.guard_step_warn_m: offline={sem_o.get('guard_step_warn_m')} != 0.006")
    if not _isclose(offline.get("warmup_periods"), 2.0):
        fails.append(f"warmup_periods: offline={offline.get('warmup_periods')} != 2.0")
    if not _isclose(offline.get("steady_periods"), 4.0):
        fails.append(f"steady_periods: offline={offline.get('steady_periods')} != 4.0")
    tc_f = fixture["trajectory_config"]
    tc_o = offline.get("trajectory_config") or {}
    for key, want in tc_f.items():
        got = tc_o.get(key)
        if isinstance(want, list):
            same = (isinstance(got, list) and len(got) == len(want)
                    and all(_isclose(a, b) for a, b in zip(got, want)))
        else:
            same = got is not None and _isclose(got, want)
        if not same:
            fails.append(f"trajectory_config.{key}: offline={got} != {want}")
    return fails


def crosscheck_fixture_vs_manifest(fixture: dict, manifest_path: Path):
    """本机模式: 核对 fixture 的 SHA 与关键指标确实来自 manifest 指向的原始文件。"""
    issues = []
    reals = load_manifest_reals(manifest_path)
    if len(reals) != len(fixture["rounds"]):
        return [f"轮数不一致: manifest={len(reals)} fixture={len(fixture['rounds'])}"]
    # fixture 头部快照(驱动元数据门控)也必须与原始文件一致,防手改 fixture 绕过校验。
    r0 = reals[0][1]
    if fixture.get("trajectory_config") != r0["config_fingerprint"]["trajectory_config"]:
        issues.append("fixture.trajectory_config 与原始 config_fingerprint 不一致")
    if fixture.get("executor_params") != r0["executor_params"]:
        issues.append("fixture.executor_params 与原始文件不一致")
    rp = r0["config_fingerprint"]["ros_params"]
    for k, want in (("reference_lead_steps", int(r0["reference_lead_steps"])),
                    ("control_dt", float(r0["control_dt"])),
                    ("dq_prev_mode", str(r0["dq_prev_mode"])),
                    ("rti_iteration_mode", str(r0["rti_iteration_mode"])),
                    ("acados_rti_iterations", int(rp["acados_rti_iterations"]))):
        if fixture.get("semantics", {}).get(k) != want:
            issues.append(f"fixture.semantics.{k}={fixture.get('semantics', {}).get(k)} != 原始 {want}")
    for k in ("horizon_steps", "max_joint_step", "orientation_weight",
              "terminal_orientation_weight"):
        if not _isclose(fixture.get("production_params", {}).get(k), rp[k]):
            issues.append(f"fixture.production_params.{k} != 原始 {rp[k]}")
    for (pth, real), snap in zip(reals, fixture["rounds"]):
        sha = hashlib.sha256(pth.read_bytes()).hexdigest()
        if sha != snap["sha256"]:
            issues.append(f"{pth.name}: sha256 与 fixture 不一致(原始数据被改动?)")
            continue
        fresh = extract_round_summary(real)["metrics"]
        for k, v in snap["metrics"].items():
            if not _isclose(fresh[k], v):
                issues.append(f"{pth.name}.{k}: 原始={fresh[k]} fixture={v}")
    return issues


# ---------------------------------------------------------------- 核心比较
def compare_credibility(offline: dict, fixture: dict, offline_gain1: dict = None):
    """核心比较器(可单元测试)。返回 (items, gain1_note, overall_fail)。

    offline: T0 gain0.7 指标 dict;fixture: 入库 D0 基准(含 rounds 关键指标与工况快照)。
    """
    items = []
    overall_fail = False

    # ---- 元数据校验(不再默认相信调用者传对了文件) ----
    for msg in validate_offline_metadata(offline, fixture):
        items.append({"name": "metadata", "offline": float("nan"), "real": float("nan"),
                      "ratio": float("nan"), "status": "VALIDATION_FAIL", "detail": msg})
        overall_fail = True

    rounds = [r["metrics"] for r in fixture["rounds"]]

    def real_mean(key):
        return float(np.mean([r[key] for r in rounds]))

    def judge(name, off, real, pass_band, fail_band):
        nonlocal overall_fail
        ratio = off / real if real else float("inf")
        if pass_band[0] <= ratio <= pass_band[1]:
            status = "PASS"
        elif fail_band[0] <= ratio <= fail_band[1]:
            status = "ADVISORY"
        else:
            status = "FAIL"
            overall_fail = True
        items.append({"name": name, "offline": off, "real": real,
                      "ratio": ratio, "status": status})

    judge("actual_rms_mm", offline["steady_pos_err_rms"] * 1000.0,
          real_mean("actual_rms_mm"), RATIO_PASS_BAND, RATIO_FAIL_BAND)
    judge("actual_p95_mm", offline["steady_pos_err_p95"] * 1000.0,
          real_mean("actual_p95_mm"), RATIO_PASS_BAND, RATIO_FAIL_BAND)
    judge("exec_gap_cart_rms_mm", offline["steady_exec_gap_cart_rms"] * 1000.0,
          real_mean("exec_gap_cart_rms_mm"), RATIO_PASS_BAND, RATIO_FAIL_BAND)
    judge("base_1hz_resid_rms_mm", offline["steady_base_freq_resid_rms"] * 1000.0,
          real_mean("base_1hz_resid_rms_mm"), RATIO_PASS_BAND, RATIO_FAIL_BAND)
    judge("vel_peak_rad_s", offline["vel_peak"],
          real_mean("vel_peak_rad_s"), VEL_PEAK_PASS_BAND, RATIO_FAIL_BAND)

    # guard: D0 基线要求两侧都为 0。任一侧非零(含两侧同时非零)都是 FAIL ——
    # "两边都有拦截"不是可信度证据,而是两套各自异常的行为。
    off_guard = int(offline["guard_count"])
    real_guards = [int(r["guard_block_count"]) for r in rounds]
    guard_ok = off_guard == 0 and all(g == 0 for g in real_guards)
    items.append({"name": "guard_zero_both", "offline": off_guard,
                  "real": max(real_guards),
                  "ratio": float("nan"),
                  "status": "PASS" if guard_ok else "FAIL"})
    if not guard_ok:
        overall_fail = True

    gain1_note = None
    if offline_gain1 is not None:
        # 效度边界: 只输出说明,不参与判定,更不得据此宣称可评估执行器参数。
        gain1_note = ("OUT_OF_VALIDITY: gain=1.0 离线(guard=%d, p95=%.1fmm)不能复现真实 "
                      "D1 失稳(真实 guard 1000+/P95 130-170mm);执行器参数在离线调参器"
                      "效度范围之外,只能真实 A/B 评估。"
                      % (int(offline_gain1["guard_count"]),
                         offline_gain1["steady_pos_err_p95"] * 1000.0))
    return items, gain1_note, overall_fail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reference", type=Path, default=DEFAULT_FIXTURE,
                    help=f"入库 D0 基准 fixture(默认 {DEFAULT_FIXTURE.name})")
    ap.add_argument("--offline", type=Path, default=None,
                    help="T0 gain0.7 指标 JSON(显式路径)")
    ap.add_argument("--offline-gain1", type=Path, default=None,
                    help="T0 gain1.0 指标 JSON(可选,只作效度外说明)")
    ap.add_argument("--manifest", type=Path, default=None,
                    help="本机原始三轮 manifest(可选: 交叉核对 fixture SHA/数值)")
    ap.add_argument("--make-fixture", action="store_true",
                    help="从 --manifest 原始数据重新生成 fixture 后退出")
    args = ap.parse_args()

    if args.make_fixture:
        if args.manifest is None:
            ap.error("--make-fixture 需要 --manifest(本机原始数据)")
        make_fixture(args.manifest, args.reference)
        return

    if args.offline is None:
        ap.error("--offline 必填(或使用 --make-fixture)")
    fixture = json.loads(args.reference.read_text())

    if args.manifest is not None:
        issues = crosscheck_fixture_vs_manifest(fixture, args.manifest)
        if issues:
            print("fixture 与本机原始数据交叉核对失败:")
            for msg in issues:
                print(f"  {msg}")
            sys.exit(3)
        print("fixture 与本机原始数据交叉核对: 一致(SHA+数值)")

    offline = json.loads(args.offline.read_text())
    gain1 = (json.loads(args.offline_gain1.read_text())
             if args.offline_gain1 else None)

    items, gain1_note, overall_fail = compare_credibility(offline, fixture, gain1)
    print(f'{"item":26s} {"offline":>10s} {"real":>10s} {"ratio":>7s}  status')
    for it in items:
        print(f'{it["name"]:26s} {it["offline"]:10.3f} {it["real"]:10.3f} '
              f'{it["ratio"]:7.3f}  {it["status"]}'
              + (f'  [{it["detail"]}]' if "detail" in it else ""))
    if gain1_note:
        print(f"\n[gain=1.0] {gain1_note}")
    print(f'\n判定: {"FAIL(元数据/量级/守卫冲突)" if overall_fail else "OK(全部 PASS/ADVISORY)"}')
    sys.exit(1 if overall_fail else 0)


if __name__ == "__main__":
    main()
