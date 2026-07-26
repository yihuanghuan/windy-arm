#!/usr/bin/env python3
"""幅值-频率二维可行包络扫描(ROS student_arm 闭环仿真,二阶生产配置 gain=1.0)。

每个 (频率, 幅值) 网格点跑一轮 run_ab_experiment.sh(基座沿倾斜球面大圆运动,
恒有 |p_base-center|=R=A,经 MB_* 环境变量注入共享轨迹配置),analyze 后记录
稳态 RMS/P95/max/守卫/fault。
结果增量存 results.json(断点续跑),最后画 envelope_2d.png:
横轴频率、纵轴幅值,两幅面分别为 RMS 与最大误差,fault 点标 X,
并分别画出 RMS<=5mm、MAX<=5mm 与 MAX<=10mm 的离散采样包络。
"""

import itertools
import csv
import json
import re
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

DEMO = Path(__file__).resolve().parent
OUT = DEMO / 'data' / 'envelope_sphere'
OUT.mkdir(parents=True, exist_ok=True)
RES = OUT / 'results.json'

FREQS = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5]          # Hz
AMPS = [0.005, 0.0075, 0.01, 0.015, 0.02, 0.03, 0.04]  # m = 球半径 R(基座约束球)
RMS_FEASIBLE_MM = 5.0                                 # 边界趋势线判据: RMS<=5mm 且无 fault
MAX_FEASIBLE_MM = 5.0
MAX_RELAXED_MM = 10.0

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


def _faulted(record):
    return bool(record.get('fault') or record.get('analyze_failed')
                or record.get('run_failed'))


def _max_feasible_amplitude(results, frequencies, metric, threshold_mm):
    boundary = []
    for freq in frequencies:
        feasible = [
            float(r['amp_m']) * 1000.0
            for r in results.values()
            if float(r['freq_hz']) == float(freq)
            and not _faulted(r)
            and np.isfinite(float(r.get(metric, np.nan)))
            and float(r[metric]) <= threshold_mm
        ]
        boundary.append(max(feasible) if feasible else np.nan)
    return np.asarray(boundary, dtype=float)


def write_tables(results):
    """Write a flat point table, sampled envelope table, and run manifest."""
    fs = sorted({float(r['freq_hz']) for r in results.values()})
    rows = sorted(results.values(), key=lambda r: (r['freq_hz'], r['amp_m']))
    point_csv = OUT / 'envelope_points.csv'
    fieldnames = [
        'frequency_hz', 'period_sec', 'amplitude_mm',
        'base_speed_peak_m_s', 'base_accel_peak_m_s2',
        'rms_mm', 'p95_mm', 'max_mm', 'guard_blocks', 'fault',
        'run_failed', 'analyze_failed', 'joint_velocity_peak_rad_s',
        'joint_accel_peak_rad_s2',
    ]
    with point_csv.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            freq = float(r['freq_hz'])
            amp = float(r['amp_m'])
            writer.writerow({
                'frequency_hz': freq,
                'period_sec': 1.0 / freq,
                'amplitude_mm': 1000.0 * amp,
                'base_speed_peak_m_s': 2.0 * np.pi * freq * amp,
                'base_accel_peak_m_s2': (2.0 * np.pi * freq) ** 2 * amp,
                'rms_mm': r.get('rms_mm', ''),
                'p95_mm': r.get('p95_mm', ''),
                'max_mm': r.get('max_mm', ''),
                'guard_blocks': r.get('guard', r.get('guard_blocks', '')),
                'fault': _faulted(r),
                'run_failed': bool(r.get('run_failed')),
                'analyze_failed': bool(r.get('analyze_failed')),
                'joint_velocity_peak_rad_s': r.get('vpk', ''),
                'joint_accel_peak_rad_s2': r.get('apk', ''),
            })

    envelope_csv = OUT / 'envelope_boundary.csv'
    rms5 = _max_feasible_amplitude(results, fs, 'rms_mm', RMS_FEASIBLE_MM)
    max5 = _max_feasible_amplitude(results, fs, 'max_mm', MAX_FEASIBLE_MM)
    max10 = _max_feasible_amplitude(results, fs, 'max_mm', MAX_RELAXED_MM)
    with envelope_csv.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'frequency_hz', 'rms_le_5mm_max_amplitude_mm',
            'max_le_5mm_max_amplitude_mm', 'max_le_10mm_max_amplitude_mm',
        ])
        for values in zip(fs, rms5, max5, max10):
            writer.writerow(values)

    def git_state(cwd):
        head = subprocess.run(
            ['git', 'rev-parse', 'HEAD'], cwd=cwd, capture_output=True,
            text=True, check=True).stdout.strip()
        branch = subprocess.run(
            ['git', 'branch', '--show-current'], cwd=cwd, capture_output=True,
            text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(
            ['git', 'status', '--porcelain'], cwd=cwd, capture_output=True,
            text=True, check=True).stdout.strip())
        return {'root': str(cwd), 'head': head, 'branch': branch, 'dirty': dirty}

    run_git_states = set()
    provenance_found = 0
    for r in rows:
        provenance = OUT.parent / f"provenance_{r['label']}.json"
        if not provenance.exists():
            continue
        recorded = json.loads(provenance.read_text()).get('git', {})
        outer = recorded.get('outer_ws', {})
        inner = recorded.get('inner_pkg', {})
        run_git_states.add((
            outer.get('head'), outer.get('branch'), bool(outer.get('dirty')),
            inner.get('head'), inner.get('branch'), bool(inner.get('dirty')),
        ))
        provenance_found += 1

    manifest = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'render_git_state': {
            'outer_ws': git_state(DEMO.parents[2]),
            'inner_pkg': git_state(DEMO.parent),
        },
        'point_run_provenance': {
            'records_found': provenance_found,
            'unique_git_states': [
                {
                    'outer_ws': {'head': state[0], 'branch': state[1], 'dirty': state[2]},
                    'inner_pkg': {'head': state[3], 'branch': state[4], 'dirty': state[5]},
                }
                for state in sorted(run_git_states, key=str)
            ],
        },
        'controller': 'position-only second-order NMPC',
        'plant': 'ROS student_arm closed-loop simulator',
        'base_motion': 'sphere: tilted great circle with constant |p_base-center|=R',
        'frequencies_hz': fs,
        'amplitudes_mm': sorted({1000.0 * float(r['amp_m']) for r in rows}),
        'point_count': len(rows),
        'finite_metric_count': int(sum(
            np.isfinite(float(r.get('rms_mm', np.nan)))
            and np.isfinite(float(r.get('max_mm', np.nan)))
            for r in rows)),
        'fault_count': int(sum(_faulted(r) for r in rows)),
        'metric_window': 'analyze_moving_base_run.py actual_error.steady',
        'thresholds_mm': {
            'rms_feasible': RMS_FEASIBLE_MM,
            'max_strict': MAX_FEASIBLE_MM,
            'max_relaxed': MAX_RELAXED_MM,
        },
        'fixed_configuration': {
            'control_rate_hz': 50.0,
            'track_sec': 20.0,
            'solver_model': 'second_order',
            'horizon_steps': 10,
            'position_weight': 5000.0,
            'terminal_position_weight': 500.0,
            'joint_velocity_weight': 0.02,
            'joint_acceleration_weight': 0.0,
            'max_joint_velocity_rad_s': 1.0,
            'max_joint_acceleration_rad_s2': 5.0,
            'velocity_feedforward_gain': 1.0,
            'second_order_velocity_source': 'commanded',
            'guard_mode': 'expected_residual',
        },
        'files': {
            'raw_results': str(RES),
            'point_table': str(point_csv),
            'sampled_boundary': str(envelope_csv),
            'plot': str(OUT / 'envelope_2d.png'),
        },
    }
    (OUT / 'experiment_manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return point_csv, envelope_csv


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
        if _faulted(r):
            fault[As.index(r['amp_m']), fs.index(r['freq_hz'])] = True

    rms_boundary = _max_feasible_amplitude(
        results, fs, 'rms_mm', RMS_FEASIBLE_MM)
    max_boundary = _max_feasible_amplitude(
        results, fs, 'max_mm', MAX_FEASIBLE_MM)
    max_relaxed_boundary = _max_feasible_amplitude(
        results, fs, 'max_mm', MAX_RELAXED_MM)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    titles = {'rms_mm': 'steady RMS error (mm)', 'max_mm': 'steady max error (mm)'}
    for ax, name in zip(axes, ('rms_mm', 'max_mm')):
        G = grids[name]
        ax.set_facecolor('#eeeeee')
        pc = ax.pcolormesh(
            fs, np.array(As) * 1000, np.ma.masked_invalid(G),
            shading='nearest', cmap='viridis')
        fig.colorbar(pc, ax=ax, label='mm')
        for i in range(len(As)):
            for j in range(len(fs)):
                if fault[i, j]:
                    ax.plot(fs[j], As[i] * 1000, 'rx', ms=12, mew=3)
                elif np.isfinite(G[i, j]):
                    ax.text(fs[j], As[i] * 1000, f'{G[i, j]:.1f}', ha='center',
                            va='center', color='w', fontsize=8)
        if name == 'rms_mm':
            ax.plot(fs, rms_boundary, 'wo-', lw=2, ms=5,
                    label=f'sampled envelope: RMS ≤ {RMS_FEASIBLE_MM:g} mm')
        else:
            ax.plot(fs, max_boundary, 'wo-', lw=2, ms=5,
                    label=f'sampled envelope: MAX ≤ {MAX_FEASIBLE_MM:g} mm')
            ax.plot(fs, max_relaxed_boundary, 'w^--', lw=1.8, ms=5,
                    label=f'sampled envelope: MAX ≤ {MAX_RELAXED_MM:g} mm')
        ax.plot(2.0, 10.0, marker='*', ms=14, color='#ffcc00',
                markeredgecolor='black', label='nominal f/A: 2 Hz, 10 mm')
        ax.legend(loc='upper right', fontsize=8)
        ax.set_xlabel('base frequency (Hz)')
        ax.set_title(titles[name] + '   (X = guard fault / infeasible)')
    axes[0].set_ylabel('base displacement amplitude R (mm),  |p_base-center|=R')
    fig.suptitle(
        'Position-only second-order NMPC base-motion envelope — '
        'sphere great-circle sweep, gain=1.0')
    fig.tight_layout()
    out = OUT / 'envelope_2d.png'
    fig.savefig(out, dpi=140)
    print(f'plot: {out}')


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
    points, boundary = write_tables(results)
    plot(results)
    print(f'point table: {points}')
    print(f'boundary table: {boundary}')


if __name__ == '__main__':
    main()
