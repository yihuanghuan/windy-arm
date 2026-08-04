#!/usr/bin/env python3
"""合并多次 fk_gravity_verify 运行的结果，输出统一判决表与零位标定向量。

扫描按关节分次运行时（例如先单独验证 joint1、再跑 joint2~5），每次都会
写出独立的 JSON。本工具把它们按关节合并，同一关节以最新一次为准，并给出
可直接写入模型的零位偏差向量。

用法:
    ros2 run manipulator merge_fk_verify.py
    ros2 run manipulator merge_fk_verify.py --data-dir <dir> --json out.json
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

N_ARM = 6
# 与 fk_gravity_verify 保持一致
SHAPE_RESIDUAL_LIMIT = 0.05
ENGINEERING_THRESHOLD_DEG = 0.5


def default_data_dir() -> Path:
    return Path(__file__).resolve().parent / "data" / "fk_verify"


def load_runs(data_dir: Path) -> list[dict]:
    runs = []
    for path in sorted(data_dir.glob("fk_verify_*.json")):
        try:
            runs.append((path, json.loads(path.read_text())))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"  skipping {path.name}: {exc}", file=sys.stderr)
    return runs


def ingest_log(path: Path) -> dict | None:
    """从 runner 日志重建扫描结果。

    JSON 只在运行结束时写出，被中断的运行什么也留不下 —— 实测发生过：
    joint2 完整跑完了 26 个点，但进程被杀，结果只存在于日志里。日志是逐行
    写穿的，所以它是最可靠的恢复源。
    """
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from task_space_smc.hw.fk_gravity_verify import (
        SWEEP_PLAN, GravityModel, fit_sine, model_reference,
        offset_threshold_deg, zero_offset,
    )

    joint = direction = None
    data: dict[int, dict[str, list]] = {}
    for line in path.read_text(errors="ignore").splitlines():
        mj = re.search(r"=== joint(\d+) sweep", line)
        if mj:
            joint, direction = int(mj.group(1)), None
            continue
        if "sweeping fwd" in line:
            direction = "fwd"
            continue
        if "sweeping rev" in line:
            direction = "rev"
            continue
        mp = re.search(r"q=([-+]?\d+\.\d+)\s+tau=([-+]?\d+\.\d+)\s+\+/-\s+(\d+\.\d+)",
                       line)
        if mp and joint and direction:
            data.setdefault(joint, {}).setdefault(direction, []).append(
                (float(mp.group(1)), float(mp.group(2))))

    if not data:
        return None
    model = GravityModel()
    sweeps = []
    for j, dd in sorted(data.items()):
        want = SWEEP_PLAN[j]["points"]
        if len(dd.get("fwd", [])) != want or len(dd.get("rev", [])) != want:
            continue                       # 该关节没跑完，丢弃而不是半截拟合
        fwd = np.array(dd["fwd"])
        rev = np.array(dd["rev"])[::-1]
        q = 0.5 * (fwd[:, 0] + rev[:, 0])
        tau = 0.5 * (fwd[:, 1] + rev[:, 1])
        stic = 0.5 * (fwd[:, 1] - rev[:, 1])
        meas = fit_sine(q, tau)
        ref = model_reference(model, j, list(SWEEP_PLAN[j]["posture"]), q)
        sweeps.append(dict(
            joint=j, ok=True, posture=list(SWEEP_PLAN[j]["posture"]),
            points=want, dwell=SWEEP_PLAN[j]["dwell"],
            expected_sigma_deg=SWEEP_PLAN[j]["sigma"],
            threshold_deg=offset_threshold_deg(j),
            measured=meas, model=ref,
            amplitude_ratio=meas["amplitude"] / ref["amplitude"],
            zero_offset_rad=zero_offset(meas["phi"], ref["phi"]),
            zero_offset_deg=float(np.degrees(zero_offset(meas["phi"], ref["phi"]))),
            stiction_mean=float(np.mean(np.abs(stic))),
            stiction_max=float(np.max(np.abs(stic))),
            recovered_from_log=str(path),
        ))
    if not sweeps:
        return None
    return dict(timestamp=f"log:{path.name}", sweeps=sweeps)


def collect(runs) -> tuple[dict, dict, list[str]]:
    """按关节取最新一次成功的扫描与方向校验。"""
    sweeps, directions, notes = {}, {}, []
    for path, run in runs:
        stamp = run.get("timestamp", path.stem)
        for r in run.get("direction_check", []):
            directions[r["joint"]] = (stamp, r)
        for s in run.get("sweeps", []):
            if s.get("warmup"):
                # 热身扫描的数据保留在文件里作为证据，但不能进拟合：
                # 到达新位姿后的首扫会被污染，joint2 的四次首扫散布在
                # -7.9 ~ +5.6 度，而热身后的两遍吻合到 0.007 度。
                continue
            if s.get("ok"):
                sweeps[s["joint"]] = (stamp, s)
            elif s["joint"] not in sweeps:
                notes.append(f"joint{s['joint']} 无有效扫描: {s.get('reason')}")
        if run.get("aborted_at_joint"):
            notes.append(
                f"{stamp} 因安全检查在 joint{run['aborted_at_joint']} 中止")
    return sweeps, directions, notes


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", default=None)
    ap.add_argument("--json", default=None, help="把合并结果另存为 JSON")
    ap.add_argument("--ingest-log", action="append", default=[],
                    help="从 runner 日志重建被中断运行的结果，可多次指定")
    args = ap.parse_args()

    data_dir = Path(args.data_dir) if args.data_dir else default_data_dir()
    if not data_dir.is_dir():
        print(f"目录不存在: {data_dir}", file=sys.stderr)
        return 2

    runs = load_runs(data_dir)
    for lp in args.ingest_log:
        rec = ingest_log(Path(lp))
        if rec:
            joints = [s["joint"] for s in rec["sweeps"]]
            print(f"  从日志恢复 {Path(lp).name}: joint{joints}")
            runs.append((Path(lp), rec))
        else:
            print(f"  {Path(lp).name} 没有可恢复的完整扫描", file=sys.stderr)
    if not runs:
        print(f"{data_dir} 下没有 fk_verify_*.json", file=sys.stderr)
        return 2

    sweeps, directions, notes = collect(runs)

    print("=" * 92)
    print(f"合并 {len(runs)} 次运行 ({data_dir})")
    print("=" * 92)

    if directions:
        print("\n方向 / 标度校验（验证命令->固件->编码器往返符号一致；"
              "注意它无法发现相对 URDF 的符号约定错误，那要看扫描的相位）")
        for j in sorted(directions):
            stamp, r = directions[j]
            if r.get("ok"):
                print(f"  joint{j}: OK   命令 {r['commanded']:+.4f} -> "
                      f"实测 {r['measured']:+.4f} rad  (ratio {r.get('scale_ratio', float('nan')):+.3f})")
            else:
                print(f"  joint{j}: FAIL {r.get('reason', 'sign/scale mismatch')}")

    print(f"\n{'关节':<7} {'A实测':>9} {'A模型':>9} {'ratio':>7} "
          f"{'零位偏差':>11} {'±1σ':>8} {'resid/A':>8} {'静摩擦':>9}  判定")
    offsets = np.zeros(N_ARM)
    geometry_fault = []
    for j in range(1, N_ARM + 1):
        if j not in sweeps:
            reason = "无重力信号（link6 绕自身轴对称）" if j == N_ARM else "未扫描"
            print(f"joint{j:<3} {'—':>9} {'—':>9} {'—':>7} {'—':>11} "
                  f"{'—':>8} {'—':>8} {'—':>9}  {reason}")
            continue
        stamp, s = sweeps[j]
        m = s["measured"]
        rel = m["rms_residual"] / m["amplitude"] if m["amplitude"] > 1e-9 else float("inf")
        off = s.get("zero_offset_deg", float("nan"))
        sig = s.get("expected_sigma_deg", float("nan"))
        if rel > SHAPE_RESIDUAL_LIMIT:
            verdict = "*** 几何错（形状被破坏）***"
            geometry_fault.append(j)
        elif abs(off) < 3.0 * sig:
            verdict = "零位偏差不显著"
        else:
            verdict = "零位偏差 -> 作为标定值"
            offsets[j - 1] = np.radians(off)
        print(f"joint{j:<3} {m['amplitude']:>9.4f} {s['model']['amplitude']:>9.4f} "
              f"{s.get('amplitude_ratio', float('nan')):>7.3f} {off:>8.3f}deg "
              f"{sig:>8.4f} {rel:>8.4f} {s['stiction_mean']:>9.4f}  {verdict}")

    print("\n" + "-" * 92)
    if geometry_fault:
        print(f"结论: 检出几何错 —— joint{geometry_fault}。"
              f"正弦形状被破坏说明连杆长度或轴向与实物不符，")
        print("      必须先查清再进入阶段 2 的质量标定，否则标定建在错误的几何上。")
    else:
        print("结论: 所有已扫描关节的 resid/A 均在阈值内 —— 正弦形状完好，")
        print("      模型的连杆长度与轴向结构正确，可以进入阶段 2 的质量标定。")

    ratios = [sweeps[j][1].get("amplitude_ratio") for j in sweeps
              if sweeps[j][1].get("amplitude_ratio")]
    if ratios:
        print(f"\n幅值比 ratio: {np.round(ratios, 3).tolist()}  "
              f"中位数 {np.median(ratios):.3f}")
        print("  ratio > 1 表示实测重力力矩大于模型 -> URDF 质量偏轻，阶段 2 标定的目标")

    print(f"\n零位偏差向量（实际角 = 命令角 + offset，rad）:")
    print(f"  {np.round(offsets, 6).tolist()}")
    print(f"  角度: {np.round(np.degrees(offsets), 3).tolist()} deg")
    if abs(offsets).max() > 0:
        print("  用法: 模型计算前把测得的关节角减去该向量，或在 URDF 里加对应的关节零位")

    for n in notes:
        print(f"\n注意: {n}")

    if args.json:
        out = dict(
            data_dir=str(data_dir), runs=len(runs),
            zero_offsets_rad=offsets.tolist(),
            zero_offsets_deg=np.degrees(offsets).tolist(),
            geometry_fault_joints=geometry_fault,
            per_joint={str(j): sweeps[j][1] for j in sweeps},
            direction_check={str(j): directions[j][1] for j in directions},
            notes=notes,
        )
        Path(args.json).write_text(json.dumps(out, indent=2, default=float))
        print(f"\n合并结果写入 {args.json}")
    return 1 if geometry_fault else 0


if __name__ == "__main__":
    sys.exit(main())
