#!/usr/bin/env python3
"""不确定性实验一键试验台 (harness)。

把"MPC→plant→传感→MPC"整个闭环在纯 Python 里仿真(不开 ROS),
你只管选工况参数, 它负责跑完并打分, 结果自动追加到
data/uncertainty_harness_results.csv 方便横向比较。

常用命令:
  python3 uncertainty_harness.py --preset b1          # 第1级: MPC 不知道扰动
  python3 uncertainty_harness.py --preset b2          # 对照: MPC 全知扰动
  python3 uncertainty_harness.py --preset b3          # 第2级: 模拟状态估计
  python3 uncertainty_harness.py --preset baseline    # 无扰动基线
  python3 uncertainty_harness.py --list               # 看所有预设

自定义(想调哪个就给哪个, 其余用预设值):
  python3 uncertainty_harness.py --preset b3 --noise-mm 5 --delay-ms 100
  python3 uncertainty_harness.py --belief nominal --amp-mm 20 --seeds 0,1,2

跑之前记得 source 环境:
  source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
"""
import argparse
import csv
from datetime import datetime
from pathlib import Path

import numpy as np

import hifi_offline_loop as H

# 生产等效配置(与实机 demo 默认一致: 放开限幅+延迟补偿+Smith+逆变换)
PROD = dict(plant_vmax=3.0, plant_amax=20.0, mpc_vmax=2.0, mpc_amax=15.0,
            latency_comp=True, act_delay_ms=5.0, smith=True, invert=True)

PRESETS = {
    'baseline': dict(help='无扰动基线(装置自检, 预期~0.5mm)',
                     dist_amp=0.0, belief='truth'),
    'b1': dict(help='第1级·无知下界: 扰动10mm, MPC只知名义轨迹(预期~5mm)',
               dist_amp=0.010, belief='nominal'),
    'b2': dict(help='全知上界: 扰动10mm, MPC精确知道扰动(预期~0.5mm)',
               dist_amp=0.010, belief='truth'),
    'b3': dict(help='第2级·模拟状态估计: σ=2mm/延迟40ms/低通10Hz(预期~4.3mm)',
               dist_amp=0.010, belief='estimated',
               est_noise_m=0.002, est_delay_ms=40.0, est_lpf_hz=10.0),
}

RESULT_CSV = Path(__file__).resolve().parent / 'data' / 'uncertainty_harness_results.csv'
COLS = ['stamp', 'preset', 'belief', 'amp_mm', 'noise_mm', 'delay_ms',
        'lpf_hz', 'seed', 'steady_sec', 'total_rms_mm', 'p95_mm', 'max_mm',
        'rms_2hz_mm', 'resid_mm', 'label']


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--preset', choices=sorted(PRESETS), default=None,
                    help='工况预设(先取预设值, 再被下面的显式参数覆盖)')
    ap.add_argument('--list', action='store_true', help='列出预设并退出')
    ap.add_argument('--belief', choices=('nominal', 'truth', 'estimated'),
                    help='MPC 对扰动知道多少')
    ap.add_argument('--amp-mm', type=float, help='扰动幅值界(每轴, mm)')
    ap.add_argument('--noise-mm', type=float, help='估计噪声σ(mm, 仅 estimated)')
    ap.add_argument('--delay-ms', type=float, help='估计延迟(ms, 仅 estimated)')
    ap.add_argument('--lpf-hz', type=float, help='估计低通带宽(Hz, 仅 estimated)')
    ap.add_argument('--seeds', default='0', help='扰动种子, 逗号分隔如 0,1,2')
    ap.add_argument('--steady-sec', type=float, default=10.0, help='统计时长(s)')
    ap.add_argument('--label', default='', help='备注, 写进结果表')
    args = ap.parse_args()

    if args.list:
        for name, p in PRESETS.items():
            print(f'  {name:9s} {p["help"]}')
        return

    cfg = dict(PRESETS[args.preset or 'b3'])
    cfg.pop('help')
    if args.belief:
        cfg['belief'] = args.belief
    if args.amp_mm is not None:
        cfg['dist_amp'] = args.amp_mm * 1e-3
    if args.noise_mm is not None:
        cfg['est_noise_m'] = args.noise_mm * 1e-3
    if args.delay_ms is not None:
        cfg['est_delay_ms'] = args.delay_ms
    if args.lpf_hz is not None:
        cfg['est_lpf_hz'] = args.lpf_hz

    seeds = [int(s) for s in args.seeds.split(',') if s.strip() != '']
    rows, totals = [], []
    for seed in seeds:
        name = (f"{args.preset or 'custom'} belief={cfg['belief']} "
                f"amp={cfg.get('dist_amp', 0.0) * 1000:.0f}mm seed={seed}")
        d = H.run(name, dist_seed=seed, steady_sec=args.steady_sec,
                  **cfg, **PROD)
        totals.append(d['total'])
        rows.append([
            datetime.now().strftime('%Y%m%d_%H%M%S'), args.preset or 'custom',
            cfg['belief'], round(cfg.get('dist_amp', 0.0) * 1000, 3),
            round(cfg.get('est_noise_m', 0.0) * 1000, 3),
            cfg.get('est_delay_ms', 0.0), cfg.get('est_lpf_hz', 0.0), seed,
            args.steady_sec, round(d['total'], 4), round(d.get('p95', float('nan')), 4)
            if 'p95' in d else '', round(d.get('max', float('nan')), 4)
            if 'max' in d else '', round(d['f1'], 4), round(d['resid'], 4),
            args.label,
        ])

    new_file = not RESULT_CSV.exists()
    RESULT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with RESULT_CSV.open('a', newline='') as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(COLS)
        w.writerows(rows)
    t = np.asarray(totals)
    print(f'\n== 汇总 ({len(seeds)} seeds): total RMS mean={t.mean():.3f} '
          f'std={t.std():.3f} mm | 已追加 {len(rows)} 行 -> {RESULT_CSV}')


if __name__ == '__main__':
    main()
