#!/usr/bin/env python3
"""幅值-频率二维可行包络扫描(真机,二阶生产配置 gain=1.0)。

每个 (频率, 幅值) 网格点跑一轮 run_ab_experiment.sh(水平半径与 z 幅值同设为 A,
经 MB_* 环境变量注入共享轨迹配置),analyze 后记录稳态 RMS/P95/max/守卫/fault。
结果增量存 results.json(断点续跑),最后画 envelope_2d.png:
横轴频率、纵轴幅值,两幅面分别为 RMS 与最大误差,fault 点标 X,
并拟合可行边界趋势线 A = c / f^2(恒加速度需求曲线)。
"""

import itertools
import json
import re
import os
import subprocess
from pathlib import Path

import numpy as np

DEMO = Path(__file__).resolve().parent
OUT = DEMO / 'data' / 'envelope_sphere'
OUT.mkdir(parents=True, exist_ok=True)
RES = OUT / 'results.json'

FREQS = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5]          # Hz
AMPS = [0.005, 0.0075, 0.01, 0.015, 0.02, 0.03, 0.04]  # m = 球半径 R(基座约束球)
RMS_FEASIBLE_MM = 5.0                                 # 边界趋势线判据: RMS<=5mm 且无 fault

ENV_BASE = dict(
    os.environ, TRACK_SEC='20', CONTROL_RATE_HZ='50.0', MAX_JOINT_STEP='0.02',
    SECOND_ORDER_HORIZON_STEPS='10', SECOND_ORDER_POSITION_WEIGHT='5000.0',
    SECOND_ORDER_ACCELERATION_WEIGHT='0.0', SECOND_ORDER_DQ_FF_COMPENSATION='1.0',
    SECOND_ORDER_VELOCITY_SOURCE='commanded', SECOND_ORDER_REFERENCE_LEAD='-1',
    SECOND_ORDER_STATE_PREDICTION='false')


def run_point(freq, amp):
    key = f'f{freq:g}_A{amp * 1000:g}mm'
    lbl = ('env_' + key).replace('.', 'p')
    env = dict(ENV_BASE, MB_BASE_PERIOD=str(1.0 / freq),
               MB_BASE_SHAPE='sphere', MB_BASE_SPHERE_R=str(amp))
    o = OUT / lbl
    o.mkdir(exist_ok=True)
    rec = {'freq_hz': freq, 'amp_m': amp, 'label': lbl}
    for attempt in (1, 2):
        try:
            subprocess.run(
                ['bash', 'run_ab_experiment.sh', lbl, '1', str(o), 'measured_delta',
                 'fixed_iterations', '1.0', 'expected_residual', '2', '0.25',
                 'reset_nominal', 'second_order', '1.0', '1.0'],
                cwd=DEMO, env=env, capture_output=True, timeout=420)
        except subprocess.TimeoutExpired:
            continue
        log_p = o / f'demo_{lbl}.log'
        if not log_p.exists():
            continue
        log = log_p.read_text(errors='ignore')
        loops = re.findall(r'控制环诊断 CSV 开始记录: (\S+)', log)
        if loops:
            rec['fault'] = 'guard_fault 安全停止' in log
            mg = re.findall(r'守卫拦截总次数: (\d+)', log)
            rec['guard_blocks'] = int(mg[-1]) if mg else None
            loop = loops[-1]
            params = loop.replace('mpc_loop_', 'mpc_params_')[:-4] + '.json'
            mfile = o / 'metrics.json'
            r = subprocess.run(
                ['python3', 'analyze_moving_base_run.py', 'analyze', '--loop-csv', loop,
                 '--params-json', params, '--label', lbl, '--out', str(mfile)],
                cwd=DEMO, env=env, capture_output=True, timeout=300)
            if r.returncode == 0 and mfile.exists():
                m = json.loads(mfile.read_text())
                s = m['actual_error']['steady']
                rec.update(rms_mm=s['rms_mm'], p95_mm=s['p95_mm'], max_mm=s['max_mm'],
                           guard=m['guard']['block_count'],
                           vpk=m['measured_vel_saturation_steady']['abs_peak'],
                           apk=m['measured_accel_saturation_steady']['abs_peak'])
            else:
                rec['analyze_failed'] = True   # 早停(fault)导致无稳态窗
            return rec
    rec['run_failed'] = True
    return rec


def plot(results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fs = sorted({r['freq_hz'] for r in results.values()})
    As = sorted({r['amp_m'] for r in results.values()})
    grids = {}
    for name in ('rms_mm', 'max_mm'):
        G = np.full((len(As), len(fs)), np.nan)
        for r in results.values():
            i, j = As.index(r['amp_m']), fs.index(r['freq_hz'])
            G[i, j] = r.get(name, np.nan)
        grids[name] = G
    fault = np.zeros((len(As), len(fs)), dtype=bool)
    for r in results.values():
        if r.get('fault') or r.get('analyze_failed') or r.get('run_failed'):
            fault[As.index(r['amp_m']), fs.index(r['freq_hz'])] = True

    # 可行边界: 每个频率下 RMS<=阈值且无 fault 的最大幅值 -> 拟合 A = c/f^2
    bx, by = [], []
    for j, f in enumerate(fs):
        ok = [As[i] for i in range(len(As))
              if not fault[i, j] and np.isfinite(grids['rms_mm'][i, j])
              and grids['rms_mm'][i, j] <= RMS_FEASIBLE_MM]
        if ok:
            bx.append(f)
            by.append(max(ok))
    c = float(np.median([a * f * f for f, a in zip(bx, by)])) if bx else None

    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    titles = {'rms_mm': 'steady RMS error (mm)', 'max_mm': 'steady max error (mm)'}
    for ax, name in zip(axes, ('rms_mm', 'max_mm')):
        G = grids[name]
        pc = ax.pcolormesh(fs, np.array(As) * 1000, G, shading='nearest', cmap='viridis')
        fig.colorbar(pc, ax=ax, label='mm')
        for i in range(len(As)):
            for j in range(len(fs)):
                if fault[i, j]:
                    ax.plot(fs[j], As[i] * 1000, 'rx', ms=12, mew=3)
                elif np.isfinite(G[i, j]):
                    ax.text(fs[j], As[i] * 1000, f'{G[i, j]:.1f}', ha='center',
                            va='center', color='w', fontsize=8)
        if c is not None:
            ff = np.linspace(min(fs), max(fs), 100)
            ax.plot(ff, np.minimum(np.array([c / (f * f) for f in ff]),
                                   max(As)) * 1000, 'w--', lw=2,
                    label=f'feasible boundary  A·f² ≈ {c * 1000:.1f} mm·Hz²')
            ax.legend(loc='upper right', fontsize=8)
        ax.set_xlabel('base frequency (Hz)')
        ax.set_title(titles[name] + '   (X = guard fault / infeasible)')
    axes[0].set_ylabel('base amplitude: circle radius = z amp (mm)')
    fig.suptitle('Base-in-sphere feasibility envelope (|p|=R tilted great circle) — second-order NMPC, gain=1.0')
    fig.tight_layout()
    out = OUT / 'envelope_2d.png'
    fig.savefig(out, dpi=140)
    print(f'plot: {out}  boundary c={c}')


def main():
    results = json.loads(RES.read_text()) if RES.exists() else {}
    points = list(itertools.product(FREQS, AMPS))
    for n, (f, a) in enumerate(points, 1):
        key = f'f{f:g}_A{a * 1000:g}mm'
        if key in results:
            continue
        print(f'[{n}/{len(points)}] {key}', flush=True)
        results[key] = run_point(f, a)
        RES.write_text(json.dumps(results, indent=1, ensure_ascii=False))
    plot(results)


if __name__ == '__main__':
    main()
