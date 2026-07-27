#!/usr/bin/env python3
"""moving_base_circle 实验自动指标分析(阶段0 时钟修复版)。

数据源:loop CSV 现为自洽真值源——每行含真实 clock 的 t_solve/t_publish/t_reference/
t_next、q_pub、q_meas、dq_meas、js_header_stamp_rel、时钟诊断。分析器据此 + 参数快照
(轨迹/URDF/lead)重建全部指标。visualizer CSV 仅作一致性交叉核对,不进门槛。

时间语义(base/wall clock 与 path clock 均相对首个 track tick 求解起点):
- 基座平移/浮动态度扰动用真实 base/wall clock；目标相位与 whole 工作面朝向用 path clock。
- cmd@publish       : FK_world(q_pub, base=t_publish,path=path(t_publish))
                      vs target(path(t_publish))。publish-lag 由此测。
- cmd@reference_time: FK_world(q_pub, base=t_reference,path=path_target_time)
                      vs target(path_target_time),t_reference=t_solve+lead*dt。
                      **命名不叫 effective**:lead=0 不代表命令能在发布瞬间生效。lead 相关,报告项。
- cmd@next_tick     : FK_world(q_pub,base=t_publish+dt,path=path(t_publish+dt))
                      vs target(path(t_publish+dt))。固定+1物理拍,lead 无关,KPI。
- exec_gap(real)    : 执行器跟随误差,按真实 publish wall 对齐:
                      joint = q_actual(t_publish+dt) - q_pub(t_publish);
                      cart  = FK_world(q_actual(t_publish+dt), t_publish+dt) - FK_world(q_pub, t_publish+dt)。
- actual_error      : actual_world(base=t_js,path=path(t_js))-target(path(t_js)),
                      t_js=js_header_stamp_rel(真实测量时刻)。真值跟踪误差。
旧 loop/visualizer CSV 无 path 列时保持 path_time=wall_time 的历史语义。
统一统计窗口:full(全交叠)与 steady(丢弃启动过程的完整周期窗口,默认[4,40]s);门槛用 steady。
误差单位 mm;RMS=sqrt(mean(e^2))。正弦"幅值A"与"RMS"严格区分。lag>0 表示信号相对解析参考延迟。
异步序列一律按真实时间戳 np.interp 对齐,mask 到交叠区间防端点外推。

用法:
  python3 analyze_moving_base_run.py analyze --loop-csv L.csv --params-json P.json \
      [--prefix VIZ_PREFIX] --label NAME --out M.json
  python3 analyze_moving_base_run.py compare baseline.json phase1a.json
"""

import argparse
import csv
import json
import math
import sys
from fractions import Fraction
from pathlib import Path

import numpy as np
import pinocchio as pin

import moving_base_trajectory_config as moving_traj

U0_ACTIVE_THRESHOLD_RAD = 0.0195
SATURATION_FRACTION = 0.98
STEADY_WINDOW = (4.0, 40.0)
LEGACY_ACCEL_LIMIT = 5.0
LEGACY_VEL_LIMIT = 1.0


# ================================================================= 轨迹/FK 重建

class TrajectoryModel:
    def __init__(self, cfg, trajectory_time_scale=1.0, base_motion_time_scale=1.0):
        self.center = np.array(cfg['BASE_CIRCLE_CENTER'], dtype=float)
        self.radius = float(cfg['BASE_CIRCLE_RADIUS'])
        self.base_period = float(cfg['BASE_PERIOD_SEC'])
        self.height = float(cfg['BASE_HEIGHT'])
        self.z_amp = float(cfg.get('BASE_Z_AMPLITUDE', 0.0) or 0.0)
        self.z_phase = float(cfg.get('BASE_Z_PHASE', 0.0) or 0.0)
        self.base_motion_shape = str(
            cfg.get('BASE_MOTION_SHAPE', 'circle_z') or 'circle_z')
        self.sphere_r = float(cfg.get('BASE_SPHERE_RADIUS', 0.0) or 0.0)
        _deg = cfg.get('BASE_ATTITUDE_AMPLITUDE_DEG')
        _fb = math.radians(float(_deg)) if _deg is not None else None
        self.roll_amp = float(cfg.get('BASE_ROLL_AMPLITUDE', _fb))
        self.pitch_amp = float(cfg.get('BASE_PITCH_AMPLITUDE', _fb))
        self.yaw_amp = float(cfg.get('BASE_YAW_AMPLITUDE', _fb))
        self.ee_center = np.array(cfg['EE_WORLD_CIRCLE_CENTER'], dtype=float)
        self.ee_radius = float(cfg['EE_CIRCLE_RADIUS'])
        self.ee_period = float(cfg['EE_PERIOD_SEC'])
        self.trajectory_shape = str(
            cfg.get('EE_TRAJECTORY_SHAPE', 'circle') or 'circle'
        ).strip().lower()
        self.trajectory_time_scale = float(trajectory_time_scale)
        self.base_motion_time_scale = float(base_motion_time_scale)
        if self.trajectory_time_scale <= 0.0 or self.base_motion_time_scale <= 0.0:
            raise ValueError('trajectory_time_scale/base_motion_time_scale must be > 0')
        # New snapshots store EE_TRAJECTORY_PERIOD_SEC already scaled.  Older
        # snapshots omit it, so reconstruct the scaled period exactly once.
        period_from_snapshot = cfg.get('EE_TRAJECTORY_PERIOD_SEC')
        if period_from_snapshot is not None:
            self.ee_trajectory_period = float(period_from_snapshot)
        else:
            unscaled_period = (
                self.ee_period
                if self.trajectory_shape == 'circle'
                else moving_traj.ee_trajectory_period(
                    self.trajectory_shape)
            )
            self.ee_trajectory_period = (
                float(unscaled_period) * self.trajectory_time_scale)

    def base_pose(self, t):
        w = 2.0 * math.pi / (self.base_period * self.base_motion_time_scale)
        if self.base_motion_shape == 'sphere':
            c = np.array([self.center[0], self.center[1], self.height])
            u = np.array([1.0, 0.0, 0.0])
            v = np.array([0.0, 1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0)])
            return c + self.sphere_r * (math.cos(w * t) * u + math.sin(w * t) * v)
        return np.array([self.center[0] + self.radius * math.cos(w * t),
                         self.center[1] + self.radius * math.sin(w * t),
                         self.height + self.z_amp * math.sin(w * t + self.z_phase)])

    def base_rotation(self, base_t, path_t=None):
        """Return world-from-base rotation using the two production clocks.

        Floating attitude disturbance follows physical/base time.  Only the
        nominal task-station heading used by ``whole`` follows path time.
        ``path_t=None`` preserves the one-clock behaviour of historical CSVs.
        """
        if path_t is None:
            path_t = base_t
        w = 2.0 * math.pi / (
            self.base_period * self.base_motion_time_scale)
        roll = self.roll_amp * math.sin(w * base_t)
        pitch = self.pitch_amp * math.sin(
            w * base_t + 2.0 * math.pi / 3.0)
        yaw = self.yaw_amp * math.sin(
            w * base_t + 4.0 * math.pi / 3.0)
        disturbance = pin.rpy.rpyToMatrix(roll, pitch, yaw)
        if self.trajectory_shape != 'whole':
            return disturbance
        # moving_traj consumes unscaled trajectory seconds.  The snapshot
        # scale is applied exactly once here.
        nominal_yaw = moving_traj.base_nominal_yaw(
            float(path_t) / self.trajectory_time_scale,
            self.trajectory_shape)
        c, s = math.cos(nominal_yaw), math.sin(nominal_yaw)
        nominal = np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ])
        return nominal @ disturbance

    def ee_target_world(self, t):
        if self.trajectory_shape != 'circle':
            # Non-circle geometry is source-defined and the source file is
            # part of the experiment fingerprint.  Snapshot time scaling is
            # nevertheless authoritative and is applied exactly once.
            return moving_traj.ee_target_world(
                float(t) / self.trajectory_time_scale,
                self.trajectory_shape)
        w = 2.0 * math.pi / (self.ee_period * self.trajectory_time_scale)
        return self.ee_center + self.ee_radius * np.array(
            [math.cos(w * t), math.sin(w * t), 0.0])

    def world_from_base_point(self, p_base, base_t, path_t=None):
        if path_t is None:
            path_t = base_t
        return (
            self.base_pose(base_t)
            + self.base_rotation(base_t, path_t) @ p_base
        )

    def repetition_period(self):
        ee = Fraction(str(self.ee_trajectory_period))
        base = Fraction(str(self.base_period * self.base_motion_time_scale))
        num = np.lcm(ee.numerator * base.denominator, base.numerator * ee.denominator)
        return float(Fraction(int(num), ee.denominator * base.denominator))


class FkModel:
    def __init__(self, urdf_path, ee_frame):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(ee_frame)

    def fk_base(self, q):
        pin.framesForwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        return self.data.oMf[self.frame_id].translation.copy()


# ================================================================= CSV/统计

def _load_csv(path):
    with open(path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        model_col = header.index('solver_model') if 'solver_model' in header else -1
        model_codes = {'first_order': 0.0, 'second_order': 1.0, 'shadow': 2.0}
        rows = []
        for row in reader:
            if not row:
                continue
            if model_col >= 0 and row[model_col] in model_codes:
                row[model_col] = model_codes[row[model_col]]
            rows.append([float(x) for x in row])
    return header, np.array(rows, dtype=float)


def _cols(h, d, names):
    return np.column_stack([d[:, h.index(n)] for n in names])


def _col(h, d, name):
    return d[:, h.index(name)]


def _interp_path_clock(query_t, base_t, path_t, path_scale):
    """Interpolate recorded path progress at physical/base timestamps.

    Interior samples use the recorded ``path_time`` timeline, so a scale
    change on the following controller tick is represented exactly instead
    of being projected forever from the preceding row.  The logged local
    scale is only used for the small boundary extrapolation.
    """
    query_raw = np.asarray(query_t, dtype=float)
    scalar_query = query_raw.ndim == 0
    query = np.atleast_1d(query_raw)
    base = np.asarray(base_t, dtype=float)
    path = np.asarray(path_t, dtype=float)
    scale = np.asarray(path_scale, dtype=float)
    if base.ndim != 1 or len(base) == 0:
        raise ValueError('base/path clock arrays must be non-empty 1-D arrays')
    out = np.interp(query, base, path)
    left = query < base[0]
    right = query > base[-1]
    if np.any(left):
        out[left] = path[0] + scale[0] * (query[left] - base[0])
    if np.any(right):
        out[right] = path[-1] + scale[-1] * (query[right] - base[-1])
    return float(out[0]) if scalar_query else out


def _loop_path_clocks(h, d):
    """Return path-time arrays corresponding to each physical loop clock.

    Schema <=4 did not record a separate path clock.  In that case every
    path axis intentionally aliases its historical wall/base-time axis.
    """
    t_solve = _col(h, d, 't_solve')
    t_publish = _col(h, d, 't_publish')
    t_reference = _col(h, d, 't_reference')
    t_next = _col(h, d, 't_next')
    js_rel = _col(h, d, 'js_header_stamp_rel')
    has_path_clock = (
        'path_time' in h
        and np.any(np.isfinite(_col(h, d, 'path_time')))
    )
    if not has_path_clock:
        return {
            'mode': 'legacy_wall',
            'solve': t_solve.copy(),
            'publish': t_publish.copy(),
            'reference': t_reference.copy(),
            'next': t_next.copy(),
            'js': js_rel.copy(),
            'scale': np.ones_like(t_solve),
        }

    path_solve_raw = _col(h, d, 'path_time')
    path_solve = np.where(
        np.isfinite(path_solve_raw), path_solve_raw, t_solve)
    if 'path_scale' in h:
        scale_raw = _col(h, d, 'path_scale')
        path_scale = np.where(
            np.isfinite(scale_raw) & (scale_raw >= 0.0),
            scale_raw, 1.0)
    else:
        path_scale = np.ones_like(t_solve)

    path_publish = _interp_path_clock(
        t_publish, t_solve, path_solve, path_scale)
    path_next = _interp_path_clock(
        t_next, t_solve, path_solve, path_scale)
    path_js = _interp_path_clock(
        js_rel, t_solve, path_solve, path_scale)
    path_reference = _interp_path_clock(
        t_reference, t_solve, path_solve, path_scale)
    if 'path_target_time' in h:
        target_raw = _col(h, d, 'path_target_time')
        path_reference = np.where(
            np.isfinite(target_raw), target_raw, path_reference)
    return {
        'mode': 'recorded_path',
        'solve': path_solve,
        'publish': path_publish,
        'reference': path_reference,
        'next': path_next,
        'js': path_js,
        'scale': path_scale,
    }


def _visualizer_path_axis(header, data):
    """Prefer the synchronized visualizer path axis, with old CSV fallback."""
    return _col(
        header, data,
        'path_time_sec' if 'path_time_sec' in header else 'time_sec')


def _win(t, lo, hi):
    return (t >= lo) & (t <= hi)


def _num(v, default):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _norm_stats_mm(err_xyz, mask=None):
    e = np.linalg.norm(err_xyz, axis=1) * 1000.0
    if mask is not None:
        e = e[mask]
    if e.size == 0:
        return {'error': 'empty window'}
    return {'mean_mm': float(np.mean(e)), 'rms_mm': float(np.sqrt(np.mean(e ** 2))),
            'p95_mm': float(np.percentile(e, 95)), 'max_mm': float(np.max(e)), 'n': int(e.size)}


def _full_steady(t_axis, err_xyz):
    return {'full': _norm_stats_mm(err_xyz),
            'steady': _norm_stats_mm(err_xyz, _win(t_axis, *STEADY_WINDOW))}


def _quadrature_fit(t, y, freq_hz):
    w = 2.0 * math.pi * freq_hz
    design = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(design, y, rcond=None)
    return math.hypot(float(coef[0]), float(coef[1])), math.atan2(float(coef[1]), float(coef[0]))


def _sine(t, err_xyz, freq_hz, mask=None):
    if mask is not None:
        t, err_xyz = t[mask], err_xyz[mask]
    amps = [_quadrature_fit(t, err_xyz[:, ax], freq_hz)[0] * 1000.0 for ax in range(3)]
    return {'freq_hz': freq_hz, 'amplitude_mm': dict(zip('xyz', amps)),
            'combined_rms_mm': math.sqrt(sum(a * a for a in amps) / 2.0)}


def _wrap_pi(x):
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def _lag_ms(t, ref, sig, freq_hz, mask=None):
    if mask is not None:
        t, ref, sig = t[mask], ref[mask], sig[mask]
    w = 2.0 * math.pi * freq_hz
    _, pr = _quadrature_fit(t, ref, freq_hz)
    _, ps = _quadrature_fit(t, sig, freq_hz)
    return _wrap_pi(pr - ps) / w * 1000.0


def _phase_bins(t, err_xyz, period, mask=None, bin_deg=30):
    if mask is not None:
        t, err_xyz = t[mask], err_xyz[mask]
    phase = (t % period) / period * 360.0
    e = np.linalg.norm(err_xyz, axis=1) * 1000.0
    bins = []
    for start in range(0, 360, bin_deg):
        m = (phase >= start) & (phase < start + bin_deg)
        bins.append({'phase_deg': [start, start + bin_deg],
                     'mean_mm': float(np.mean(e[m])) if np.any(m) else None,
                     'n': int(np.sum(m))})
    valid = [b for b in bins if b['n'] > 0]
    worst = max(valid, key=lambda b: b['mean_mm'])
    best = min(valid, key=lambda b: b['mean_mm'])
    return {'bins': bins, 'worst_bin_mean_mm': worst['mean_mm'],
            'worst_bin_phase_deg': worst['phase_deg'],
            'best_bin_mean_mm': best['mean_mm']}


# ================================================================= 漂移(不外推)

def _same_phase_drift(times, q, period, discard_cycles=0):
    times = np.asarray(times, dtype=float)
    t0, t_end = float(times[0]), float(times[-1])
    K = int(math.floor((t_end - t0) / period + 1e-9))
    if K < 1:
        return {'error': 'recording shorter than one repetition period', 'n_cycles': 0}
    if discard_cycles >= K:
        return {'error': f'discard({discard_cycles}) >= available cycles({K})', 'n_cycles': K}
    anchors = [t0 + k * period for k in range(discard_cycles, K + 1)]
    assert anchors[-1] <= t_end + 1e-6, 'drift anchor out of bounds'
    qa = np.column_stack([np.interp(anchors, times, q[:, j]) for j in range(q.shape[1])])
    total = qa[-1] - qa[0]
    return {'repetition_period_sec': period, 'discard_initial_cycles': discard_cycles,
            'anchor_time_range_sec': [anchors[0], anchors[-1]], 'n_cycles_used': len(anchors) - 1,
            'total_drift_rad': {f'joint{j + 1}': float(total[j]) for j in range(q.shape[1])},
            'max_abs_total_drift_rad': float(np.max(np.abs(total)))}


# ================================================================= 激活/饱和

def _activation_by_sector(t_phase, u0, ee_period, threshold, mask=None, bin_deg=30):
    if mask is not None:
        t_phase, u0 = t_phase[mask], u0[mask]
    active = np.abs(u0) >= threshold
    n_per_frame = np.sum(active, axis=1)
    phase = (t_phase % ee_period) / ee_period * 360.0
    sectors = []
    for start in range(0, 360, bin_deg):
        m = (phase >= start) & (phase < start + bin_deg)
        if not np.any(m):
            sectors.append({'phase_deg': [start, start + bin_deg], 'n': 0})
            continue
        sectors.append({'phase_deg': [start, start + bin_deg], 'n': int(np.sum(m)),
                        'any_joint_active_ratio': float(np.mean(np.any(active[m], axis=1))),
                        'per_joint_active_ratio': {f'joint{j + 1}': float(np.mean(active[m, j]))
                                                   for j in range(u0.shape[1])},
                        'mean_active_joint_count': float(np.mean(n_per_frame[m]))})
    return {'threshold_rad': threshold,
            'overall_any_joint_active_ratio': float(np.mean(np.any(active, axis=1))),
            'overall_per_joint_active_ratio': {f'joint{j + 1}': float(np.mean(active[:, j]))
                                               for j in range(u0.shape[1])},
            'active_joint_count_hist': {int(k): int(v) for k, v in
                                        zip(*np.unique(n_per_frame, return_counts=True))},
            'by_sector': sectors,
            'caveat': ('激活率高只说明步长约束在该扇区广泛活跃;无对偶变量/KKT/放宽约束反事实实验,'
                       '只能说"步长约束可能参与",不能断定其主导 P95 尖峰。')}


def _accel_saturation(t_js, dq_meas, a_limit, mask=None):
    """实测加速度饱和: a=diff(dq_meas)/diff(js_header_stamp)。排除非正/异常/跨丢帧 dt。"""
    t = np.asarray(t_js, dtype=float)
    dt = np.diff(t)
    finite_dq = np.all(np.isfinite(dq_meas), axis=1)
    pair_ok = finite_dq[:-1] & finite_dq[1:]
    if not np.any(pair_ok & (dt > 0)):
        return {'error': 'no usable dq/timestamp pairs'}
    med = float(np.median(dt[dt > 0]))
    good = pair_ok & (dt > 0.5 * med) & (dt < 2.0 * med)   # 排除非正/跨丢帧/异常
    a = np.diff(dq_meas, axis=0)[good] / dt[good][:, None]
    if mask is not None:
        # mask 对齐到 diff 的左端点
        a = np.diff(dq_meas, axis=0)[good & mask[:-1]] / dt[good & mask[:-1]][:, None]
    if a.shape[0] == 0:
        return {'error': 'no accel samples in window'}
    peak = np.max(np.abs(a), axis=1)
    return {'a_limit': a_limit, 'nominal_dt_ms': med * 1000.0, 'n': int(a.shape[0]),
            'overall_near_limit_ratio': float(np.mean(peak >= SATURATION_FRACTION * a_limit)),
            'per_joint_near_limit_ratio': {f'joint{j + 1}':
                                           float(np.mean(np.abs(a[:, j]) >= SATURATION_FRACTION * a_limit))
                                           for j in range(a.shape[1])},
            'abs_p95': float(np.percentile(np.abs(a), 95)), 'abs_peak': float(np.max(np.abs(a)))}


def _vel_saturation(dq_meas, v_limit, mask=None):
    v = dq_meas[mask] if mask is not None else dq_meas
    ok = np.all(np.isfinite(v), axis=1)
    v = v[ok]
    if v.size == 0:
        return {'error': 'no finite velocity'}
    peak = np.max(np.abs(v), axis=1)
    return {'v_limit': v_limit, 'near_limit_ratio': float(np.mean(peak >= SATURATION_FRACTION * v_limit)),
            'abs_p95': float(np.percentile(np.abs(v), 95)), 'abs_peak': float(np.max(np.abs(v))), 'n': int(v.shape[0])}


# ================================================================= 时钟健康

def _clock_health(
        t_solve, t_publish, timer_interval_ms, feedback_age_ms,
        solve_to_publish_ms, dt, feedback_delta_t_sec=None):
    dt_ms = dt * 1000.0
    fin = np.isfinite(timer_interval_ms)
    interval = timer_interval_ms[fin]
    idx = np.arange(len(t_solve))
    dev = t_solve - idx * dt
    dev_detrended_ms = (dev - np.median(dev)) * 1000.0
    fa = feedback_age_ms[np.isfinite(feedback_age_ms)]
    if feedback_delta_t_sec is None:
        feedback_delta_ms = np.zeros(0)
    else:
        feedback_delta_t_sec = np.asarray(feedback_delta_t_sec, dtype=float)
        feedback_delta_ms = (
            1000.0 * feedback_delta_t_sec[
                np.isfinite(feedback_delta_t_sec)
                & (feedback_delta_t_sec >= 0.0)])
    # steady 子集(门槛用):排除启动冷路径,只看稳态控制是否干净。
    steady = _win(t_solve, *STEADY_WINDOW)
    itv_s = timer_interval_ms[fin & steady]
    dev_s = dev_detrended_ms[steady]

    def stats(x):
        return {'mean': float(np.mean(x)), 'p95': float(np.percentile(x, 95)),
                'min': float(np.min(x)), 'max': float(np.max(x))} if x.size else {}
    return {
        'dt_ms': dt_ms, 'n_ticks': int(len(t_solve)),
        'timer_interval_ms_full': stats(interval),
        'timer_interval_ms_steady': stats(itv_s),
        # 门槛字段(steady):补偿 tick 数、nominal-real 偏差 P95。
        'catch_up_tick_count': int(np.sum(itv_s < 0.5 * dt_ms)),
        'catch_up_tick_count_full': int(np.sum(interval < 0.5 * dt_ms)),
        'long_tick_count': int(np.sum(itv_s > 1.5 * dt_ms)),
        'long_tick_count_full': int(np.sum(interval > 1.5 * dt_ms)),
        'nominal_real_dev_p95_ms': float(np.percentile(np.abs(dev_s), 95)) if dev_s.size else None,
        'nominal_real_dev_max_ms_full': float(np.max(np.abs(dev_detrended_ms))),
        # 永久合成偏移检测:median(dev) 在 run 前/后半是否稳定(应≈0)。
        'median_dev_drift_ms': float((np.median(dev[t_solve > np.median(t_solve)])
                                     - np.median(dev[t_solve <= np.median(t_solve)])) * 1000.0),
        'solve_to_publish_ms': {'mean': float(np.nanmean(solve_to_publish_ms)),
                                'p95': float(np.nanpercentile(solve_to_publish_ms, 95)),
                                'max': float(np.nanmax(solve_to_publish_ms))},
        'feedback_age_ms': {'first': float(feedback_age_ms[0]) if len(feedback_age_ms) else None,
                            'mean': float(np.mean(fa)) if fa.size else None,
                            'p95': float(np.percentile(fa, 95)) if fa.size else None,
                            'max': float(np.max(fa)) if fa.size else None},
        # 50Hz NMPC 从 100Hz feedback 读取“最新值”时，正常相邻样本时间差应接近
        # 20ms。长期约 10ms 表明 executor 正在逐条消费旧队列；偶发 70--110ms
        # 跳变表明队列溢出后批量丢弃，是 20260725 螺旋跟踪的直接时序证据。
        'feedback_delta_t_ms': {
            **stats(feedback_delta_ms),
            'duplicate_count': int(np.sum(feedback_delta_ms <= 1e-6)),
            'backlog_10ms_ratio': float(np.mean(
                feedback_delta_ms < 0.75 * dt_ms))
                if feedback_delta_ms.size else None,
            'skip_over_30ms_count': int(np.sum(
                feedback_delta_ms > 1.5 * dt_ms)),
        },
    }


# ================================================================= 指纹

FINGERPRINT_ROS_PARAMS = (
    'solver_backend', 'solver_model', 'horizon_steps', 'max_joint_step',
    'terminal_tolerance',
    'orientation_weight', 'terminal_orientation_weight', 'acados_rti_iterations',
    'position_weight', 'terminal_position_weight', 'joint_delta_weight',
    'joint_delta_change_weight', 'current_posture_weight', 'center_posture_weight',
    'command_fk_step_warn_m',
    # 阶段1E: 守卫参数进指纹(guard_mode 是 1E 实验变量;其余守卫参数两组必须一致)。
    'guard_mode', 'guard_residual_abs_m', 'guard_residual_relative_gain',
    'guard_residual_accel_gain', 'guard_projection_enabled',
    'guard_projection_min_lambda', 'guard_projection_max_iterations',
    'guard_max_consecutive_blocks',
    # Executor-model and OCP limits are fixed configuration, not hidden
    # experimental variables. Feed-forward gain is checked as a paired
    # Phase-1D variable separately.
    'plant_position_feedback_tau', 'plant_feedback_velocity_limit',
    'plant_max_velocity', 'plant_max_acceleration',
    'second_order_max_velocity', 'second_order_max_acceleration',
)
FINGERPRINT_FILES = (
    'demo/move_base_circle_mpc_ik_demo.py', 'demo/mpc_pinocchio_ik.py', 'demo/pinocchio_ik.py',
    'demo/moving_base_trajectory_config.py', 'demo/trajectory_visualizer_demo.py',
    'demo/analyze_moving_base_run.py', 'launch/student_arm.launch.py',
    'config/arm.urdf', 'config/motor_config.yaml', 'config/arm_config.yaml',
    'build/student_arm_node', 'install/arm.urdf',
)


# 阶段1D:执行器关键参数。gain 是 1D 实验变量(单独核对,不进指纹);其余六项必须逐值相同。
EXECUTOR_KEY_PARAMS = (
    'max_velocity', 'enable_velocity_feedforward', 'velocity_feedforward_gain',
    'position_feedback_tau', 'feedback_velocity_limit',
    'enable_acceleration_limit', 'max_acceleration',
)
EXECUTOR_PARAMS_EXCEPT_GAIN = tuple(p for p in EXECUTOR_KEY_PARAMS if p != 'velocity_feedforward_gain')


def _student_arm_key_params(snap):
    """从 provenance 结构化取 /student_arm_node 执行器关键参数(优先 key_params,回退解析 yaml)。"""
    nodep = (snap.get('provenance', {}) or {}).get('node_params', {}) or {}
    v = nodep.get('/student_arm_node')
    if isinstance(v, dict) and isinstance(v.get('key_params'), dict):
        return v['key_params']
    if isinstance(v, dict) and isinstance(v.get('yaml'), str):
        parsed = {}
        lines = v['yaml'].splitlines()
        for key in EXECUTOR_KEY_PARAMS:
            prefix = f'{key}:'
            matches = [
                line.strip()[len(prefix):].strip()
                for line in lines
                if line.strip().startswith(prefix)
            ]
            if not matches:
                parsed[key] = None
                continue
            raw = matches[-1]
            if raw.lower() in ('true', 'false'):
                parsed[key] = raw.lower() == 'true'
                continue
            try:
                parsed[key] = float(raw)
            except ValueError:
                parsed[key] = raw
        return parsed
    return {k: None for k in EXECUTOR_KEY_PARAMS}


def _actual_executor_limits(snap):
    """Resolve measured saturation thresholds without conflating model limits.

    Actual ``student_arm_node`` provenance is authoritative. Schema-7
    snapshots may also carry an explicit executor-limits record copied from
    that provenance. Historical snapshots retain their top-level acceleration
    field and 1/5 defaults for backwards-compatible analysis.
    """
    key_params = _student_arm_key_params(snap)
    recorded = snap.get('executor_limits', {}) or {}
    try:
        schema = int(snap.get('schema_version', 0))
    except (TypeError, ValueError):
        schema = 0

    def positive(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value > 0.0 else None

    def resolve(key_name, recorded_name, legacy_name, legacy_default):
        value = positive(key_params.get(key_name))
        if value is not None:
            return value, 'provenance:/student_arm_node:key_params'
        value = positive(recorded.get(recorded_name))
        if value is not None:
            return value, 'snapshot:executor_limits'
        value = positive(snap.get(legacy_name))
        if value is not None:
            return value, f'snapshot:{legacy_name}:legacy'
        if schema < 7:
            return float(legacy_default), 'legacy-default'
        return None, 'unavailable'

    velocity, velocity_source = resolve(
        'max_velocity', 'velocity_rad_s', 'velocity_limit_rad_s',
        LEGACY_VEL_LIMIT)
    acceleration, acceleration_source = resolve(
        'max_acceleration', 'acceleration_rad_s2',
        'accel_limit_rad_s2', LEGACY_ACCEL_LIMIT)
    enabled = key_params.get('enable_acceleration_limit')
    if not isinstance(enabled, bool):
        enabled = recorded.get('acceleration_limit_enabled')
    if not isinstance(enabled, bool):
        enabled = True if schema < 7 else None
    return {
        'velocity_rad_s': velocity,
        'velocity_source': velocity_source,
        'acceleration_rad_s2': acceleration,
        'acceleration_source': acceleration_source,
        'acceleration_limit_enabled': enabled,
    }


def _plant_model_consistency(snap):
    """Check controller Smith/inversion parameters against actual executor."""
    actual = _student_arm_key_params(snap)
    model = snap.get('ros_params', {}) or {}
    pairs = (
        ('velocity_feedforward_gain',
         'plant_velocity_feedforward_gain'),
        ('position_feedback_tau', 'plant_position_feedback_tau'),
        ('feedback_velocity_limit', 'plant_feedback_velocity_limit'),
        ('max_velocity', 'plant_max_velocity'),
        ('max_acceleration', 'plant_max_acceleration'),
    )
    checked = {}
    mismatches = []
    for actual_name, model_name in pairs:
        actual_value = _num(actual.get(actual_name), None)
        model_value = _num(model.get(model_name), None)
        checked[actual_name] = {
            'actual': actual_value,
            'controller_model': model_value,
        }
        if actual_value is None or model_value is None:
            continue
        if not math.isclose(
                actual_value, model_value, rel_tol=0.0, abs_tol=1e-9):
            mismatches.append(
                f'{actual_name}: actual={actual_value}, '
                f'model={model_value}')

    limits = _actual_executor_limits(snap)
    solver_v = _num(model.get('second_order_max_velocity'), None)
    solver_a = _num(model.get('second_order_max_acceleration'), None)
    if (
        limits['velocity_rad_s'] is not None
        and solver_v is not None
        and solver_v > limits['velocity_rad_s'] + 1e-9
    ):
        mismatches.append(
            f'solver velocity={solver_v} exceeds executor '
            f'{limits["velocity_rad_s"]}')
    if (
        limits['acceleration_limit_enabled'] is True
        and limits['acceleration_rad_s2'] is not None
        and solver_a is not None
        and solver_a > limits['acceleration_rad_s2'] + 1e-9
    ):
        mismatches.append(
            f'solver acceleration={solver_a} exceeds executor '
            f'{limits["acceleration_rad_s2"]}')
    any_actual = any(
        values['actual'] is not None for values in checked.values())
    return {
        'valid': (not mismatches) if any_actual else None,
        'mismatches': mismatches,
        'checked': checked,
        'note': (
            'actual is provenance /student_arm_node; model is demo plant_*'),
    }


def _config_fingerprint(snap):
    prov = snap.get('provenance', {}) or {}
    files = prov.get('files_sha256', {}) or {}
    nodep = prov.get('node_params', {}) or {}
    git = prov.get('git', {}) or {}
    key_params = _student_arm_key_params(snap)

    def nodesha(name):
        # /student_arm_node 用结构化 key_params(除 gain)代替整包 dump SHA,
        #   使 gain(1D 变量)不进指纹、其余六项仍逐值核对;其他节点仍用整包 SHA。
        if name.rstrip('/').endswith('student_arm_node'):
            return None
        v = nodep.get(name)
        return v.get('sha256') if isinstance(v, dict) else str(v)
    return {
        'trajectory_config': snap.get('trajectory_config'),
        'control_dt': snap.get('control_dt'),
        'urdf_path': snap.get('urdf_path'),
        'ee_frame': snap.get('ee_frame'),
        'ros_params': {k: snap.get('ros_params', {}).get(k) for k in FINGERPRINT_ROS_PARAMS},
        'file_sha256': {k: (files.get(k, {}) or {}).get('sha256') for k in FINGERPRINT_FILES},
        'node_params_sha256': {n: nodesha(n) for n in sorted(nodep)},
        # 执行器六项(不含 gain)进指纹,任一不同即硬门失败;gain 单列供 1D 变量识别。
        'student_arm_key_params_except_gain': {k: key_params.get(k) for k in EXECUTOR_PARAMS_EXCEPT_GAIN},
        'git_inner_head': (git.get('inner_pkg', {}) or {}).get('head'),
        'git_outer_head': (git.get('outer_ws', {}) or {}).get('head'),
    }


# ================================================================= analyze

def analyze(args):
    snap = json.loads(Path(args.params_json).read_text())
    ros_params = snap.get('ros_params', {})
    traj = TrajectoryModel(
        snap['trajectory_config'],
        trajectory_time_scale=ros_params.get('trajectory_time_scale', 1.0),
        base_motion_time_scale=ros_params.get('base_motion_time_scale', 1.0),
    )
    fk = FkModel(snap['urdf_path'], snap['ee_frame'])
    dt = float(snap['control_dt'])
    lead = int(snap.get('reference_lead_steps',
                        snap.get('ros_params', {}).get('reference_lead_steps', 0)))
    ee_period = traj.ee_trajectory_period
    base_period = traj.base_period * traj.base_motion_time_scale
    ee_freq, base_freq = 1.0 / ee_period, 1.0 / base_period

    h, d = _load_csv(Path(args.loop_csv))
    t_solve = _col(h, d, 't_solve')
    t_publish = _col(h, d, 't_publish')
    t_reference = _col(h, d, 't_reference')
    t_next = _col(h, d, 't_next')
    js_rel = _col(h, d, 'js_header_stamp_rel')
    path_clock = _loop_path_clocks(h, d)
    path_solve = path_clock['solve']
    path_publish = path_clock['publish']
    path_reference = path_clock['reference']
    path_next = path_clock['next']
    path_js = path_clock['js']
    q_pub = _cols(h, d, [f'q_pub_{i + 1}' for i in range(7)])
    q_meas = _cols(h, d, [f'q_meas_{i + 1}' for i in range(7)])
    dq_meas = _cols(h, d, [f'dq_meas_{i + 1}' for i in range(7)])
    dq_pub = _cols(h, d, [f'dq_pub_{i + 1}' for i in range(7)]) if 'dq_pub_1' in h else None
    u0 = _cols(h, d, [f'u0_{i + 1}' for i in range(7)])
    n = len(t_solve)

    fk_pub = np.array([fk.fk_base(q_pub[i]) for i in range(n)])
    fk_meas = np.array([fk.fk_base(q_meas[i]) for i in range(n)])

    def wcmd(fkb, base_tarr, path_tarr=None):
        if path_tarr is None:
            path_tarr = base_tarr
        return np.array([
            traj.world_from_base_point(
                fkb[i], float(base_tarr[i]), float(path_tarr[i]))
            for i in range(len(base_tarr))
        ])

    def wtarget(tarr):
        return np.array([traj.ee_target_world(float(t)) for t in tarr])

    # ---- command metrics
    cmd_pub_w = wcmd(fk_pub, t_publish, path_publish)
    cmd_ref_w = wcmd(fk_pub, t_reference, path_reference)
    cmd_next_w = wcmd(fk_pub, t_next, path_next)
    tgt_pub = wtarget(path_publish)
    tgt_ref = wtarget(path_reference)
    tgt_next = wtarget(path_next)

    # ---- actual world (from loop q_meas at real js time)
    actual_w = wcmd(fk_meas, js_rel, path_js)
    tgt_js = wtarget(path_js)
    err_actual = actual_w - tgt_js

    # ---- executor gap (real publish wall): q_actual interp at t_next
    lo = max(float(js_rel.min()), float(t_next.min()))
    hi = min(float(js_rel.max()), float(t_next.max()))
    exec_mask = _win(t_next, lo, hi)
    q_act_next = np.column_stack([np.interp(t_next, js_rel, q_meas[:, j]) for j in range(7)])
    fk_act_next = np.array([fk.fk_base(q_act_next[i]) for i in range(n)])
    exec_joint = q_act_next - q_pub
    exec_cart_w = np.array([
        traj.world_from_base_point(
            fk_act_next[i], float(t_next[i]), float(path_next[i]))
        for i in range(n)
    ]) - cmd_next_w

    dq_prev_mode = snap.get('dq_prev_mode',
                            snap.get('ros_params', {}).get('dq_prev_mode', 'zero'))
    rti_iteration_mode = snap.get('rti_iteration_mode',
                                  snap.get('ros_params', {}).get('rti_iteration_mode',
                                                                 'legacy_early_exit'))
    exec_params = _student_arm_key_params(snap)
    executor_limits = _actual_executor_limits(snap)
    plant_model_consistency = _plant_model_consistency(snap)
    vff_gain = exec_params.get('velocity_feedforward_gain')
    result = {
        'label': args.label,
        'reference_lead_steps': lead, 'control_dt': dt,
        'path_clock_mode': path_clock['mode'],
        'dq_prev_mode': dq_prev_mode,
        'rti_iteration_mode': rti_iteration_mode,
        'velocity_feedforward_gain': vff_gain,
        'executor_params': exec_params,
        'executor_limits': executor_limits,
        'plant_model_consistency': plant_model_consistency,
        'experiment_valid': bool(snap.get('experiment_valid', False)),
        'joint_states_publisher_count': snap.get('graph', {}).get('joint_states_publisher_count'),
        'joint_states_publisher_nodes': snap.get('graph', {}).get('joint_states_publisher_nodes'),
        'config_fingerprint': _config_fingerprint(snap),
        'steady_window_sec': list(STEADY_WINDOW),
        'files': {'loop_csv': str(args.loop_csv), 'params_json': str(args.params_json)},
        'definitions': {
            'dual_clock': (
                'base translation/floating attitude use physical base time; '
                'target and whole station heading use recorded path time'),
            'cmd@publish': (
                'FK_world(q_pub,base=t_publish,path=path(t_publish)) '
                'vs target(path(t_publish))'),
            'cmd@reference_time': (
                f'FK_world(q_pub,base=t_reference=t_solve+{lead}*dt,'
                'path=path_target_time) vs target(path_target_time); '
                'lead 相关,非物理生效'),
            'cmd@next_tick': (
                'FK_world(q_pub,base=t_publish+dt,path=path(t_publish+dt)) '
                'vs target(path(t_publish+dt)); 固定+1拍,lead 无关,KPI'),
            'exec_gap': 'q_actual(t_publish+dt)-q_pub / FK 差,按真实 publish wall 对齐',
            'actual_error': (
                'actual_world(base=t_js,path=path(t_js))'
                '-target(path(t_js)),真值跟踪误差'),
            'windows': f'full=全交叠;steady={STEADY_WINDOW}s',
        },
        # ---- 误差指标(full + steady)
        'actual_error': _full_steady(js_rel, err_actual),
        'cmd_error_at_publish': _full_steady(t_publish, cmd_pub_w - tgt_pub),
        'cmd_error_at_reference_time': _full_steady(t_reference, cmd_ref_w - tgt_ref),
        'cmd_error_at_next_tick': _full_steady(t_next, cmd_next_w - tgt_next),
        # full 也用 exec_mask 限制到 js_rel/t_next 交叠区,避免 np.interp 端点钳制外推泄漏。
        'exec_gap_cartesian': {'full': _norm_stats_mm(exec_cart_w, exec_mask),
                               'steady': _norm_stats_mm(exec_cart_w, _win(t_next, *STEADY_WINDOW) & exec_mask)},
        'exec_gap_joint_rad': {
            'steady_rms': float(np.sqrt(np.mean(exec_joint[_win(t_next, *STEADY_WINDOW) & exec_mask] ** 2))),
            'steady_max_abs': float(np.max(np.abs(exec_joint[_win(t_next, *STEADY_WINDOW) & exec_mask])))},
    }
    # ---- 阶段1D 执行匹配(闭环外部证据:q_pub/dq_pub 命令 vs q_measured/dq_measured 实测,按真实时间对齐)
    esm = _win(t_next, *STEADY_WINDOW) & exec_mask
    ej = exec_joint[esm]
    ejn = np.linalg.norm(exec_joint, axis=1)[esm]      # 每 tick 关节空间执行位置误差范数
    execm = {
        'note': 'q_pub/dq_pub(命令) vs q_measured/dq_measured(实测),按真实 publish wall 时间戳插值对齐;'
                'NMPC 闭环下两组 command 本就可能不同,这是外部执行端匹配证据,非控制器内部 v_ff/v_fb。',
        'exec_gap_joint_rad': {'rms': float(np.sqrt(np.mean(ej ** 2))),
                               'p95_norm': float(np.percentile(ejn, 95)),
                               'max_abs': float(np.max(np.abs(ej)))},
        'exec_gap_cartesian_mm': {
            'rms': _norm_stats_mm(exec_cart_w, esm).get('rms_mm'),
            'p95': _norm_stats_mm(exec_cart_w, esm).get('p95_mm'),
            'max': _norm_stats_mm(exec_cart_w, esm).get('max_mm')},
    }
    if dq_pub is not None:
        # 实测速度插值到命令有效时刻 t_next,与发布速度 dq_pub 比;各关节速度跟踪误差。
        dqm_next = np.column_stack([np.interp(t_next, js_rel, dq_meas[:, j]) for j in range(7)])
        vgap = (dqm_next - dq_pub)[esm]
        execm['vel_gap_dqpub_vs_dqmeas_rad_s'] = {
            'rms': float(np.sqrt(np.mean(vgap ** 2))),
            'p95_norm': float(np.percentile(np.linalg.norm(dqm_next - dq_pub, axis=1)[esm], 95)),
            'per_joint_rms': {f'joint{j + 1}': float(np.sqrt(np.mean(vgap[:, j] ** 2))) for j in range(7)}}
        dqp = dq_pub[esm]
        vmax = float(_num(exec_params.get('max_velocity'), 1.0))
        execm['dq_pub_near_max_velocity_ratio'] = float(np.mean(np.max(np.abs(dqp), axis=1) >= 0.98 * vmax))
        execm['dq_pub_abs_max_rad_s'] = float(np.max(np.abs(dqp)))
    result['phase1d_executor_match'] = execm
    sm = _win(js_rel, *STEADY_WINDOW)
    result['actual_sine_steady'] = {
        'base_freq': _sine(js_rel, err_actual, base_freq, sm),
        'ee_freq': _sine(path_js, err_actual, ee_freq, sm)}
    result['actual_lag_ms_steady'] = {
        'quad_x': _lag_ms(
            path_js, tgt_js[:, 0], actual_w[:, 0], ee_freq, sm),
        'quad_y': _lag_ms(
            path_js, tgt_js[:, 1], actual_w[:, 1], ee_freq, sm),
        'axis': 'path_time_sec'}
    result['cmd_lag_ms_steady'] = {
        'publish_x': _lag_ms(
            path_publish, tgt_pub[:, 0], cmd_pub_w[:, 0], ee_freq,
            _win(t_publish, *STEADY_WINDOW)),
        'publish_y': _lag_ms(
            path_publish, tgt_pub[:, 1], cmd_pub_w[:, 1], ee_freq,
            _win(t_publish, *STEADY_WINDOW)),
        'reference_x': _lag_ms(
            path_reference, tgt_ref[:, 0], cmd_ref_w[:, 0], ee_freq,
            _win(t_reference, *STEADY_WINDOW)),
        'reference_y': _lag_ms(
            path_reference, tgt_ref[:, 1], cmd_ref_w[:, 1], ee_freq,
            _win(t_reference, *STEADY_WINDOW)),
        'axis': 'path_time_sec',
        'note': 'publish=命令vs发布时刻目标;reference=命令vs参考路径相位目标'}
    result['actual_phase_bins_steady'] = _phase_bins(
        path_js, err_actual, ee_period, sm)
    result['u0_activation_steady'] = _activation_by_sector(
        path_solve, u0, ee_period, U0_ACTIVE_THRESHOLD_RAD, sm)
    accel_limit = executor_limits['acceleration_rad_s2']
    if executor_limits['acceleration_limit_enabled'] is False:
        result['measured_accel_saturation_steady'] = {
            'error': 'executor acceleration limit disabled',
            'a_limit': accel_limit,
            'source': executor_limits['acceleration_source'],
        }
    elif accel_limit is None:
        result['measured_accel_saturation_steady'] = {
            'error': 'actual executor acceleration limit unavailable',
            'a_limit': None,
            'source': executor_limits['acceleration_source'],
        }
    else:
        result['measured_accel_saturation_steady'] = _accel_saturation(
            js_rel, dq_meas, accel_limit, sm)
        result['measured_accel_saturation_steady']['source'] = (
            executor_limits['acceleration_source'])
    velocity_limit = executor_limits['velocity_rad_s']
    if velocity_limit is None:
        result['measured_vel_saturation_steady'] = {
            'error': 'actual executor velocity limit unavailable',
            'v_limit': None,
            'source': executor_limits['velocity_source'],
        }
    else:
        result['measured_vel_saturation_steady'] = _vel_saturation(
            dq_meas, velocity_limit, sm)
        result['measured_vel_saturation_steady']['source'] = (
            executor_limits['velocity_source'])

    # ---- 时钟健康
    result['clock_health'] = _clock_health(
        t_solve, t_publish, _col(h, d, 'timer_interval_ms'), _col(h, d, 'feedback_age_ms'),
        _col(h, d, 'solve_to_publish_ms'), dt,
        (_col(h, d, 'feedback_delta_t_sec')
         if 'feedback_delta_t_sec' in h else None))

    # ---- solver
    status = _col(h, d, 'solver_status').astype(int)
    st = _col(h, d, 'solve_time_ms'); st = st[np.isfinite(st)]
    result['solver'] = {
        'nonzero_status_count': int(np.sum(status != 0)),
        'command_usable_ratio': float(np.mean(_col(h, d, 'command_usable'))),
        'solve_time_ms': {'p95': float(np.percentile(st, 95)), 'p99': float(np.percentile(st, 99)),
                          'max': float(np.max(st))}}
    result['guard'] = {'block_count': int(np.sum(_col(h, d, 'guard_block'))), 'ticks': n}
    qpub_step = np.abs(np.diff(q_pub, axis=0))
    result['command_discontinuity'] = {'max_tick_dq_pub_rad': float(np.max(qpub_step))}

    # ---- 阶段1E 守卫诊断(loop CSV 有这些列时)
    guard_mode = snap.get('guard_mode',
                          snap.get('ros_params', {}).get('guard_mode', 'legacy_fixed_step'))
    result['guard_mode'] = guard_mode
    if 'guard_r_pos_m' in h:
        rpos = _col(h, d, 'guard_r_pos_m')
        rall = _col(h, d, 'guard_r_allow_m')
        lam = _col(h, d, 'guard_lambda')
        prj = _col(h, d, 'guard_projected').astype(int)
        consec = _col(h, d, 'guard_consecutive_blocks').astype(int)
        fault = _col(h, d, 'guard_fault').astype(int)
        ssv = _col(h, d, 'solver_step_violation').astype(int)
        rori = _col(h, d, 'guard_r_ori_rad')
        jnc = _col(h, d, 'jstep_max_next_current')
        fin = np.isfinite(rpos) & np.isfinite(rall)
        margin = rall[fin] - rpos[fin]
        rp = rpos[fin]
        rof = rori[np.isfinite(rori)]
        jf = jnc[np.isfinite(jnc)]
        # 投影事件逐条(不能只报总数): tick/t_solve/λ/r_pos/r_allow。
        projection_events = [
            {'tick': int(i), 't_solve': float(t_solve[i]), 'lambda': float(lam[i]),
             'r_pos_mm': float(rpos[i] * 1000.0), 'r_allow_mm': float(rall[i] * 1000.0)}
            for i in np.nonzero(prj)[0]]
        result['guard_1e'] = {
            'mode': guard_mode,
            'evaluated_ticks': int(np.sum(fin)),
            'r_pos_mm': ({'mean': float(np.mean(rp)) * 1000.0,
                          'p99': float(np.percentile(rp, 99)) * 1000.0,
                          'p999': float(np.percentile(rp, 99.9)) * 1000.0,
                          'max': float(np.max(rp)) * 1000.0} if rp.size else None),
            'margin_min_mm': float(np.min(margin)) * 1000.0 if rp.size else None,
            'would_exceed_count': int(np.sum(rp > rall[fin])) if rp.size else 0,
            'projection_count': int(np.sum(prj)),
            'projection_events': projection_events,
            'max_consecutive_blocks': int(np.max(consec)) if len(consec) else 0,
            'fault': bool(np.any(fault)),
            'solver_step_violation_count': int(np.sum(ssv)),
            'r_ori_max_rad': float(np.max(rof)) if rof.size else None,
            'jstep_max_next_current_max_rad': float(np.max(jf)) if jf.size else None,
        }

    # ---- 阶段1B dq_prev 诊断(loop CSV 有这些列时)
    if 'dq_prev_source' in h:
        src = _col(h, d, 'dq_prev_source').astype(int)
        fb = _col(h, d, 'dq_prev_fallback').astype(int)
        dqp_in = _cols(h, d, [f'dq_prev_input_{i + 1}' for i in range(7)])
        u0md = _cols(h, d, [f'u0_minus_dq_prev_{i + 1}' for i in range(7)])
        nu0md = _col(h, d, 'norm_u0_minus_dq_prev')
        ndu0 = _col(h, d, 'norm_delta_u0')
        sm = _win(t_solve, *STEADY_WINDOW)
        # 离线重算实际执行增量(与 demo 同公式),核对 dq_prev_input(仅 measured 模式有意义)
        js_hdr = _col(h, d, 'js_header_stamp_sec')
        recon = np.zeros((n, 7))
        for k in range(1, n):
            dtf = js_hdr[k] - js_hdr[k - 1]
            if np.isfinite(dtf) and 0.0 < dtf <= 5.0 * dt:
                recon[k] = np.clip((q_meas[k] - q_meas[k - 1]) * (dt / dtf), -0.02, 0.02)
        # 仅在 source==1(measured 成功)的行核对
        okm = (src == 1)
        recon_diff = (float(np.max(np.abs(dqp_in[okm] - recon[okm]))) if np.any(okm) else 0.0)
        ndu0f = ndu0[np.isfinite(ndu0) & sm]
        result['dq_prev'] = {
            'mode': dq_prev_mode,
            'fallback_ratio': float(np.mean(fb != 0)),
            'fallback_count': int(np.sum(fb != 0)), 'ticks': n,
            'fallback_reason_hist': {int(k): int(v) for k, v in zip(*np.unique(fb, return_counts=True))},
            'source_hist': {int(k): int(v) for k, v in zip(*np.unique(src, return_counts=True))},
            'all_zero': bool(np.all(dqp_in == 0.0)),
            'dq_prev_input_abs_max': float(np.max(np.abs(dqp_in))),
            'recompute_vs_logged_max_rad': recon_diff,
            'u0_minus_dq_prev_rms_mm_na': None,  # rad, 见下
            'u0_minus_dq_prev_norm_rad': {'rms': float(np.sqrt(np.mean(nu0md[sm] ** 2))),
                                          'p95': float(np.percentile(nu0md[sm], 95))},
            'u0_minus_dq_prev_per_joint_rms_rad': {
                f'joint{j + 1}': float(np.sqrt(np.mean(u0md[sm, j] ** 2))) for j in range(7)},
            'delta_u0_norm_rad': ({'rms': float(np.sqrt(np.mean(ndu0f ** 2))),
                                   'p95': float(np.percentile(ndu0f, 95))} if ndu0f.size else {}),
        }

    # ---- 阶段1C RTI 诊断(loop CSV 有这些列时)
    if 'rti_nit_actual' in h:
        nit = _col(h, d, 'rti_nit_actual').astype(int)
        conf = _col(h, d, 'rti_configured').astype(int)
        early = _col(h, d, 'rti_early_exit').astype(int)
        fstatus = _col(h, d, 'solver_status').astype(int)   # 最终 status
        smr = _win(t_solve, *STEADY_WINDOW)
        success = (fstatus == 0)
        succ_steady = success & smr
        n_succ = int(np.sum(succ_steady))
        conf_med = int(np.median(conf)) if len(conf) else -1
        result['rti'] = {
            'mode': rti_iteration_mode,
            'configured_iters': conf_med,
            'nit_hist_steady': {int(k): int(v) for k, v in
                                zip(*np.unique(nit[smr], return_counts=True))},
            'success_ticks_steady': n_succ,
            # treatment 机制门:成功 tick 中实际 nit==configured 的比例(steady)。
            'nit_eq_configured_ratio_success_steady': (
                float(np.mean(nit[succ_steady] == conf[succ_steady])) if n_succ else None),
            'mean_nit_success_steady': (float(np.mean(nit[succ_steady])) if n_succ else None),
            # baseline 机制存在性:早退比例/次数(steady)。
            'early_exit_ratio_steady': float(np.mean(early[smr])) if np.any(smr) else None,
            'early_exit_count_steady': int(np.sum(early[smr])),
        }

    # ---- 漂移(measured q over js time; commanded q_pub over t_publish)
    period = traj.repetition_period()
    result['drift_measured'] = {
        'with_convergence': _same_phase_drift(js_rel, q_meas, period, 0),
        'steady': _same_phase_drift(js_rel, q_meas, period, args.discard_initial_cycles)}
    result['drift_commanded'] = {
        'with_convergence': _same_phase_drift(t_publish, q_pub, period, 0),
        'steady': _same_phase_drift(t_publish, q_pub, period, args.discard_initial_cycles)}

    # ---- 一致性交叉核对(visualizer CSV,可选)
    cons = {}
    if args.prefix:
        vp = Path(f'{args.prefix}.csv')
        if vp.exists():
            vh, vd = _load_csv(vp)
            tv = _visualizer_path_axis(vh, vd)
            exp = _cols(vh, vd, ['expected_x', 'expected_y', 'expected_z'])
            ana = wtarget(tv)
            cons['viz_expected_vs_snapshot_analytic_max_m'] = float(np.max(np.linalg.norm(exp - ana, axis=1)))
            cons['viz_expected_time_axis'] = (
                'path_time_sec'
                if 'path_time_sec' in vh else 'time_sec_legacy')
        cp = Path(f'{args.prefix}_command_fk.csv')
        if cp.exists():
            ch, cd = _load_csv(cp)
            tc = _col(ch, cd, 'time_sec')
            ptc = _visualizer_path_axis(ch, cd)
            if all(name in ch for name in (
                    'expected_x', 'expected_y', 'expected_z')):
                cexp = _cols(
                    ch, cd,
                    ['expected_x', 'expected_y', 'expected_z'])
                cons['viz_command_expected_vs_snapshot_analytic_max_m'] = (
                    float(np.max(np.linalg.norm(
                        cexp - wtarget(ptc), axis=1)))
                )
            fkc = _cols(ch, cd, ['command_fk_x', 'command_fk_y', 'command_fk_z'])
            recomp = np.column_stack([np.interp(tc, t_publish, cmd_pub_w[:, i]) for i in range(3)])
            cons['recomputed_vs_viz_cmdfk_max_m'] = float(np.max(np.linalg.norm(recomp - fkc, axis=1)))
    result['consistency'] = cons

    out = Path(args.out) if args.out else Path(str(args.loop_csv).replace('.csv', '_metrics.json'))
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(f'[analyze] metrics saved: {out}\n')
    _print_summary(result)
    return result


def _s(st):
    return (f'mean {st["mean_mm"]:6.2f} rms {st["rms_mm"]:6.2f} p95 {st["p95_mm"]:6.2f} '
            f'max {st["max_mm"]:6.2f} (n={st["n"]})') if st and 'mean_mm' in st else '(n/a)'


def _print_summary(r):
    print(f'== {r["label"]} (lead={r["reference_lead_steps"]}, valid={r["experiment_valid"]}, '
          f'pub={r["joint_states_publisher_count"]}) ==  steady window {STEADY_WINDOW}s')
    plant_check = r.get('plant_model_consistency', {})
    if plant_check.get('valid') is False:
        print('  PLANT MODEL MISMATCH: '
              + '; '.join(plant_check.get('mismatches', ())))
    elif plant_check.get('valid') is None:
        print('  plant model consistency: unavailable (no executor provenance)')
    for key, lbl in (('actual_error', 'actual err'), ('cmd_error_at_publish', 'cmd@publish'),
                     ('cmd_error_at_reference_time', 'cmd@reference'), ('cmd_error_at_next_tick', 'cmd@next_tick')):
        print(f'  {lbl:14s} steady {_s(r[key]["steady"])}')
    print(f'  exec_gap cart   steady {_s(r["exec_gap_cartesian"]["steady"])}  '
          f'joint rms {r["exec_gap_joint_rad"]["steady_rms"]:.4f} rad')
    cl = r['cmd_lag_ms_steady']
    print(f'  cmd lag(ms) steady publish x/y {cl["publish_x"]:6.1f}/{cl["publish_y"]:6.1f}  '
          f'reference x/y {cl["reference_x"]:6.1f}/{cl["reference_y"]:6.1f}')
    al = r['actual_lag_ms_steady']
    print(f'  actual lag(ms) steady x/y {al["quad_x"]:6.1f}/{al["quad_y"]:6.1f}')
    s = r['actual_sine_steady']
    print(f'  actual 1Hz comb RMS {s["base_freq"]["combined_rms_mm"]:.2f}  0.25Hz {s["ee_freq"]["combined_rms_mm"]:.2f} mm')
    pb = r['actual_phase_bins_steady']
    print(f'  worst phase bin {pb["worst_bin_mean_mm"]:.2f} mm @{pb["worst_bin_phase_deg"]}')
    ac = r['measured_accel_saturation_steady']
    if 'overall_near_limit_ratio' in ac:
        print(f'  accel |a|>=0.98*{ac["a_limit"]}: '
              f'{100 * ac["overall_near_limit_ratio"]:.1f}% frames, '
              f'p95 {ac["abs_p95"]:.2f} peak {ac["abs_peak"]:.2f} rad/s^2')
    else:
        print(f'  accel saturation n/a: {ac.get("error", "unknown")}')
    ch = r['clock_health']
    tip = ch.get('timer_interval_ms_steady', {}).get('p95')
    print(f'  clock(steady): catch_up {ch["catch_up_tick_count"]}, dev_p95 {ch["nominal_real_dev_p95_ms"]:.3f}ms, '
          f'median_dev_drift {ch["median_dev_drift_ms"]:.3f}ms, timer_int p95 {tip:.2f}ms, '
          f'feedback_age first {ch["feedback_age_ms"]["first"]:.1f} p95 {ch["feedback_age_ms"]["p95"]:.1f}ms '
          f'(full catch_up {ch["catch_up_tick_count_full"]})')
    print(f'  solver nonzero {r["solver"]["nonzero_status_count"]}, p99 {r["solver"]["solve_time_ms"]["p99"]:.2f}ms, '
          f'guard {r["guard"]["block_count"]}, u0 active {100 * r["u0_activation_steady"]["overall_any_joint_active_ratio"]:.1f}%')
    dm = r['drift_measured']['steady']
    if 'error' not in dm:
        print(f'  drift meas steady max|tot| {dm["max_abs_total_drift_rad"]:.3f} rad (discard={dm["discard_initial_cycles"]})')
    print(f'  consistency {r.get("consistency")}\n')


# ================================================================= compare (hard gate)

def _get(d, path, default=None):
    for k in path.split('.'):
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def _fp_diffs(a, b):
    fa, fb = a.get('config_fingerprint', {}), b.get('config_fingerprint', {})
    diffs = []

    def rec(pa, pb, prefix=''):
        for k in sorted(set(pa or {}) | set(pb or {})):
            va, vb = (pa or {}).get(k), (pb or {}).get(k)
            if isinstance(va, dict) or isinstance(vb, dict):
                rec(va or {}, vb or {}, f'{prefix}{k}.')
            elif va != vb:
                diffs.append((f'{prefix}{k}', va, vb))
    rec(fa, fb)
    return diffs


def compare(args):
    a = json.loads(Path(args.baseline).read_text())
    b = json.loads(Path(args.after).read_text())
    la, lb = a.get('label', 'A'), b.get('label', 'B')
    dt_ms = float(a.get('control_dt', 0.02)) * 1000.0
    hard_fail = []
    lead_a, lead_b = a.get('reference_lead_steps'), b.get('reference_lead_steps')
    mode_a, mode_b = a.get('dq_prev_mode', 'zero'), b.get('dq_prev_mode', 'zero')
    rti_a, rti_b = a.get('rti_iteration_mode', 'legacy_early_exit'), b.get('rti_iteration_mode', 'legacy_early_exit')
    gain_a, gain_b = a.get('velocity_feedforward_gain'), b.get('velocity_feedforward_gain')
    gm_a = a.get('guard_mode', 'legacy_fixed_step')
    gm_b = b.get('guard_mode', 'legacy_fixed_step')
    # 实验变量识别:reference_lead_steps / dq_prev_mode / rti_iteration_mode /
    # velocity_feedforward_gain / guard_mode 恰好一个不同 -> Phase1A/1B/1C/1D/1E。
    diffs_which = [('1A', lead_a != lead_b), ('1B', mode_a != mode_b), ('1C', rti_a != rti_b),
                   ('1D', gain_a != gain_b), ('1E', gm_a != gm_b)]
    changed = [p for p, d0 in diffs_which if d0]
    if len(changed) == 1:
        phase = changed[0]
    else:
        phase = '?'
        hard_fail.append(f'实验变量不唯一(必须恰好一个不同): lead {lead_a}->{lead_b}, '
                         f'dq_prev_mode {mode_a}->{mode_b}, rti_mode {rti_a}->{rti_b}, '
                         f'gain {gain_a}->{gain_b}, guard_mode {gm_a}->{gm_b}')
    print(f'compare[Phase {phase}]: {la}(lead={lead_a},dq_prev={mode_a},rti={rti_a},'
          f'vff_gain={gain_a},guard={gm_a}) vs '
          f'{lb}(lead={lead_b},dq_prev={mode_b},rti={rti_b},vff_gain={gain_b},guard={gm_b})\n')

    # ---------------- 有效性硬门 ----------------
    print('== 有效性硬门 ==')
    for lbl, r in ((la, a), (lb, b)):
        if not r.get('experiment_valid'):
            hard_fail.append(f'{lbl} experiment_valid=false')
        if r.get('joint_states_publisher_count') != 1:
            hard_fail.append(f'{lbl} publisher_count={r.get("joint_states_publisher_count")} != 1')
        plant_check = r.get('plant_model_consistency', {})
        if plant_check.get('valid') is False:
            hard_fail.append(
                f'{lbl} controller plant model 与实际执行器不一致: '
                + '; '.join(plant_check.get('mismatches', ())))
    if phase == '1A':
        if lead_a != 0:
            hard_fail.append(f'baseline lead={lead_a} != 0')
        if lead_b != 1:
            hard_fail.append(f'after lead={lead_b} != 1')
    elif phase == '1B':
        # Phase1B:两组 lead 都固定=1,只允许 dq_prev_mode 不同(zero->measured_delta)。
        if lead_a != 1 or lead_b != 1:
            hard_fail.append(f'Phase1B 要求两组 lead==1(实际 {lead_a}/{lead_b})')
        if mode_a != 'zero':
            hard_fail.append(f'Phase1B baseline dq_prev_mode={mode_a} != zero')
        if mode_b != 'measured_delta':
            hard_fail.append(f'Phase1B after dq_prev_mode={mode_b} != measured_delta')
        # fallback 比例硬门 ≤0.1%
        for lbl, r in ((la, a), (lb, b)):
            fr = _get(r, 'dq_prev.fallback_ratio')
            if fr is not None and fr > 0.001:
                hard_fail.append(f'{lbl} dq_prev fallback 比例 {100 * fr:.3f}% > 0.1%(该轮无效)')
    elif phase == '1C':
        # Phase1C:两组固定 lead==1 且 dq_prev_mode==measured_delta,只允许 rti_iteration_mode 不同。
        if lead_a != 1 or lead_b != 1:
            hard_fail.append(f'Phase1C 要求两组 lead==1(实际 {lead_a}/{lead_b})')
        if mode_a != 'measured_delta' or mode_b != 'measured_delta':
            hard_fail.append(f'Phase1C 要求两组 dq_prev_mode==measured_delta(实际 {mode_a}/{mode_b})')
        if rti_a != 'legacy_early_exit':
            hard_fail.append(f'Phase1C baseline rti_mode={rti_a} != legacy_early_exit')
        if rti_b != 'fixed_iterations':
            hard_fail.append(f'Phase1C after rti_mode={rti_b} != fixed_iterations')
        # dq_prev fallback ≤0.1%(两组都是 measured)
        for lbl, r in ((la, a), (lb, b)):
            fr = _get(r, 'dq_prev.fallback_ratio')
            if fr is not None and fr > 0.001:
                hard_fail.append(f'{lbl} dq_prev fallback 比例 {100 * fr:.3f}% > 0.1%(该轮无效)')
    elif phase == '1D':
        # Phase1D:两组固定 lead==1、dq_prev==measured_delta、rti==fixed_iterations,
        #   只允许 velocity_feedforward_gain 不同(0.7->1.0)。
        if lead_a != 1 or lead_b != 1:
            hard_fail.append(f'Phase1D 要求两组 lead==1(实际 {lead_a}/{lead_b})')
        if mode_a != 'measured_delta' or mode_b != 'measured_delta':
            hard_fail.append(f'Phase1D 要求两组 dq_prev_mode==measured_delta(实际 {mode_a}/{mode_b})')
        if rti_a != 'fixed_iterations' or rti_b != 'fixed_iterations':
            hard_fail.append(f'Phase1D 要求两组 rti_mode==fixed_iterations(实际 {rti_a}/{rti_b})')
        if _num(gain_a, -1) != 0.7:
            hard_fail.append(f'Phase1D baseline velocity_feedforward_gain={gain_a} != 0.7')
        if _num(gain_b, -1) != 1.0:
            hard_fail.append(f'Phase1D after velocity_feedforward_gain={gain_b} != 1.0')
        if gain_a == gain_b:
            hard_fail.append(f'Phase1D gain 未真正改变({gain_a}=={gain_b})')
        # 执行器其余六项(除 gain)必须逐值相同(结构化,不靠整包 dump SHA)。
        ka = _get(a, 'config_fingerprint.student_arm_key_params_except_gain') or {}
        kb = _get(b, 'config_fingerprint.student_arm_key_params_except_gain') or {}
        for p in EXECUTOR_PARAMS_EXCEPT_GAIN:
            if ka.get(p) != kb.get(p):
                hard_fail.append(f'Phase1D 执行器参数 {p} 不同: {ka.get(p)} != {kb.get(p)}(只允许 gain 变)')
        # dq_prev fallback ≤0.1%
        for lbl, r in ((la, a), (lb, b)):
            fr = _get(r, 'dq_prev.fallback_ratio')
            if fr is not None and fr > 0.001:
                hard_fail.append(f'{lbl} dq_prev fallback 比例 {100 * fr:.3f}% > 0.1%(该轮无效)')
    elif phase == '1E':
        # Phase1E:两组固定 lead==1、measured_delta、fixed_iterations、gain==0.7,
        #   只允许 guard_mode 不同(legacy_fixed_step -> expected_residual)。
        if lead_a != 1 or lead_b != 1:
            hard_fail.append(f'Phase1E 要求两组 lead==1(实际 {lead_a}/{lead_b})')
        if mode_a != 'measured_delta' or mode_b != 'measured_delta':
            hard_fail.append(f'Phase1E 要求两组 dq_prev_mode==measured_delta(实际 {mode_a}/{mode_b})')
        if rti_a != 'fixed_iterations' or rti_b != 'fixed_iterations':
            hard_fail.append(f'Phase1E 要求两组 rti_mode==fixed_iterations(实际 {rti_a}/{rti_b})')
        if _num(gain_a, -1) != 0.7 or _num(gain_b, -1) != 0.7:
            hard_fail.append(f'Phase1E 要求两组 gain==0.7(实际 {gain_a}/{gain_b})')
        if gm_a != 'legacy_fixed_step':
            hard_fail.append(f'Phase1E baseline guard_mode={gm_a} != legacy_fixed_step')
        if gm_b != 'expected_residual':
            hard_fail.append(f'Phase1E after guard_mode={gm_b} != expected_residual')
        # 执行器七项参数(含 gain)必须逐值相同。
        ea = a.get('executor_params') or {}
        eb = b.get('executor_params') or {}
        for p in EXECUTOR_KEY_PARAMS:
            if ea.get(p) != eb.get(p):
                hard_fail.append(f'Phase1E 执行器参数 {p} 不同: {ea.get(p)} != {eb.get(p)}')
        # 守卫阈值/投影/fault 参数(除 guard_mode 外)必须一致。
        ra = _get(a, 'config_fingerprint.ros_params') or {}
        rb = _get(b, 'config_fingerprint.ros_params') or {}
        for p in ('guard_residual_abs_m', 'guard_residual_relative_gain',
                  'guard_residual_accel_gain', 'guard_projection_enabled',
                  'guard_projection_min_lambda', 'guard_projection_max_iterations',
                  'guard_max_consecutive_blocks'):
            if ra.get(p) != rb.get(p):
                hard_fail.append(f'Phase1E 守卫参数 {p} 不同: {ra.get(p)} != {rb.get(p)}')
        for lbl, r in ((la, a), (lb, b)):
            fr = _get(r, 'dq_prev.fallback_ratio')
            if fr is not None and fr > 0.001:
                hard_fail.append(f'{lbl} dq_prev fallback 比例 {100 * fr:.3f}% > 0.1%(该轮无效)')
    # provenance 必须存在(否则 SHA 指纹全 None,指纹门会 vacuously 通过)。
    for lbl, r in ((la, a), (lb, b)):
        fsha = _get(r, 'config_fingerprint.file_sha256') or {}
        if not any(v for v in fsha.values()):
            hard_fail.append(f'{lbl} 缺 provenance file_sha256(无法核对源码同一性)')
    diffs = _fp_diffs(a, b)
    if phase == '1E':
        # guard_mode 是 1E 的实验变量,允许指纹中 ros_params.guard_mode 不同(其余仍必须一致)。
        diffs = [t for t in diffs if t[0] != 'ros_params.guard_mode']
    if diffs:
        for name, va, vb in diffs:
            hard_fail.append(f'指纹不一致 {name}: {va!r} != {vb!r}')
    # 求解健康硬门:命令可用率过低 / solver 非零占比过高 = 稳态破损运行,即使 A/B 一致也拒绝。
    for lbl, r in ((la, a), (lb, b)):
        cur = _get(r, 'solver.command_usable_ratio')
        if cur is not None and cur < 0.98:
            hard_fail.append(f'{lbl} command_usable_ratio={cur:.3f} < 0.98(运行破损)')
        nz = _get(r, 'solver.nonzero_status_count'); nt = _get(r, 'guard.ticks')
        if nz is not None and nt:
            if nz / nt > 0.05:
                hard_fail.append(f'{lbl} solver 非零占比 {100 * nz / nt:.1f}% > 5%(运行破损)')
    # 一致性(expected vs analytic 必须 ~0;命令 FK 交叉核对宽容)
    for lbl, r in ((la, a), (lb, b)):
        ev = _get(r, 'consistency.viz_expected_vs_snapshot_analytic_max_m')
        if ev is not None and ev > 1e-6:
            hard_fail.append(f'{lbl} expected-vs-analytic={ev:.2e} m > 1e-6(快照与录制配置不一致)')
        cv = _get(r, 'consistency.recomputed_vs_viz_cmdfk_max_m')
        if cv is not None and cv > 5e-3:
            hard_fail.append(f'{lbl} recomputed-vs-viz cmdFK={cv * 1000:.2f} mm > 5 mm')
    # 时钟健康门 + 配对 feedback-age
    for lbl, r in ((la, a), (lb, b)):
        ch = r.get('clock_health', {})
        if ch.get('catch_up_tick_count', 0) > 0:
            hard_fail.append(f'{lbl} steady catch_up_tick_count={ch.get("catch_up_tick_count")} > 0')
        if (ch.get('nominal_real_dev_p95_ms') or 99) >= 2.0:
            hard_fail.append(f'{lbl} nominal_real_dev_p95={ch.get("nominal_real_dev_p95_ms"):.2f}ms >= 2ms')
        if abs(ch.get('median_dev_drift_ms', 99)) >= 2.0:
            hard_fail.append(f'{lbl} median_dev_drift={ch.get("median_dev_drift_ms"):.2f}ms >= 2ms(疑永久合成偏移)')
    fa_a = _get(a, 'clock_health.feedback_age_ms.p95')
    fa_b = _get(b, 'clock_health.feedback_age_ms.p95')
    # 硬门:任一轮 feedback_age P95 病态偏大(旧 snapshot 阻塞 bug 会到秒级)-> 无效。
    for lbl, fa in ((la, fa_a), (lb, fa_b)):
        if fa is not None and fa > 50.0:
            hard_fail.append(f'{lbl} feedback_age P95={fa:.1f}ms > 50ms(反馈病态陈旧)')
    # 配对匹配作**报告级 advisory**(非硬门):100Hz发布 vs 50Hz求解相位抖动使
    # p95 有 ±数 ms 的运行间散布,2ms 严格匹配对该硬件不可靠,故用 3 配对聚合排除偶然性。
    fa_advisory = (fa_a is not None and fa_b is not None and abs(fa_a - fa_b) > 2.0)
    # 有效共同窗口
    for lbl, r in ((la, a), (lb, b)):
        stw = _get(r, 'actual_error.steady')
        if not stw or 'mean_mm' not in stw or stw.get('n', 0) < 50:
            hard_fail.append(f'{lbl} steady 窗口无有效数据')
    for f in hard_fail:
        print(f'  [HARD-FAIL] {f}')
    if fa_advisory:
        print(f'  [ADVISORY] 配对 feedback_age P95 相差 {abs(fa_a - fa_b):.2f}ms > 2ms '
              f'({fa_a:.1f} vs {fa_b:.1f}ms;100/50Hz 相位抖动,非病态,靠 3 配对聚合排除)')
    if not hard_fail:
        print('  [OK] 全部有效性硬门通过(单发布者/同源码SHA/参数/时钟/窗口)')

    # ---------------- 指标表(steady) ----------------
    print('\n== 指标对照(steady window) ==')
    rows = [
        ('actual err mean', 'actual_error.steady.mean_mm'),
        ('actual err p95', 'actual_error.steady.p95_mm'),
        ('cmd@publish p95', 'cmd_error_at_publish.steady.p95_mm'),
        ('cmd@reference p95', 'cmd_error_at_reference_time.steady.p95_mm'),
        ('cmd@next_tick mean', 'cmd_error_at_next_tick.steady.mean_mm'),
        ('cmd@next_tick p95', 'cmd_error_at_next_tick.steady.p95_mm'),
        ('exec_gap cart rms', 'exec_gap_cartesian.steady.rms_mm'),
        ('cmd lag publish x', 'cmd_lag_ms_steady.publish_x'),
        ('cmd lag publish y', 'cmd_lag_ms_steady.publish_y'),
        ('actual lag x', 'actual_lag_ms_steady.quad_x'),
        ('actual lag y', 'actual_lag_ms_steady.quad_y'),
        ('actual 1Hz RMS', 'actual_sine_steady.base_freq.combined_rms_mm'),
        ('worst phase bin', 'actual_phase_bins_steady.worst_bin_mean_mm'),
        ('accel near-limit %', 'measured_accel_saturation_steady.overall_near_limit_ratio'),
        ('solver nonzero', 'solver.nonzero_status_count'),
        ('solve p99 ms', 'solver.solve_time_ms.p99'),
        ('guard blocks', 'guard.block_count'),
        ('max dq_pub rad', 'command_discontinuity.max_tick_dq_pub_rad'),
        ('drift meas steady', 'drift_measured.steady.max_abs_total_drift_rad'),
    ]
    print(f'{"metric":24s} {la:>12s} {lb:>12s} {"delta":>10s}')
    for name, path in rows:
        va, vb = _get(a, path), _get(b, path)
        if va is None or vb is None:
            print(f'{name:24s} {str(va):>12s} {str(vb):>12s} {"n/a":>10s}')
        else:
            print(f'{name:24s} {va:12.3f} {vb:12.3f} {vb - va:+10.3f}')

    func = []
    kpi_ok = None
    if phase == '1A':
        print('\n== 1A 功能门(参考前移约一拍相位变化;lead 无关口径)==')
        la_pub = np.nanmean([_get(a, 'cmd_lag_ms_steady.publish_x', np.nan),
                             _get(a, 'cmd_lag_ms_steady.publish_y', np.nan)])
        lb_pub = np.nanmean([_get(b, 'cmd_lag_ms_steady.publish_x', np.nan),
                             _get(b, 'cmd_lag_ms_steady.publish_y', np.nan)])
        func.append((f'命令 publish-lag 减少约一拍 ({la_pub:.1f}->{lb_pub:.1f} ms, 减 {la_pub - lb_pub:.1f}, 一拍={dt_ms:.0f})',
                     (la_pub - lb_pub) >= 0.6 * dt_ms))
        aea, aeb = _get(a, 'actual_error.steady.p95_mm'), _get(b, 'actual_error.steady.p95_mm')
        if aea and aeb:
            func.append((f'真值 actual_error P95 不恶化 ({aea:.2f}->{aeb:.2f} mm)', aeb <= aea * 1.02 + 0.1))
        na, nb = _get(a, 'solver.nonzero_status_count'), _get(b, 'solver.nonzero_status_count')
        func.append((f'solver 非零不增加 ({na}->{nb})', (nb or 0) <= (na or 0)))
        da, db = _get(a, 'command_discontinuity.max_tick_dq_pub_rad'), _get(b, 'command_discontinuity.max_tick_dq_pub_rad')
        if da is not None and db is not None:
            func.append((f'命令不连续性不恶化 ({da:.4f}->{db:.4f})', db <= da * 1.02 + 1e-4))
        for t, ok in func:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')
        print('\n== 性能 KPI(cmd@next_tick P95 下降>=25%;lead 无关)==')
        pa, pb = _get(a, 'cmd_error_at_next_tick.steady.p95_mm'), _get(b, 'cmd_error_at_next_tick.steady.p95_mm')
        if pa and pb:
            drop = (pa - pb) / pa * 100.0
            kpi_ok = drop >= 25.0
            print(f'  [{"PASS" if kpi_ok else "FAIL"}] cmd@next_tick P95 {pa:.2f}->{pb:.2f} mm ({drop:+.1f}%); '
                  f'即使 FAIL 也如实输出,不解释为 1A 逻辑错误')

    elif phase == '1B':
        # 求解健康硬门(Phase1B 更严):command_usable>=99.9%,solver nonzero=0,solve_time P99<20ms,未重新codegen由测试保证。
        for lbl, r in ((la, a), (lb, b)):
            cur = _get(r, 'solver.command_usable_ratio')
            if cur is not None and cur < 0.999:
                hard_fail.append(f'{lbl} command_usable_ratio={cur:.4f} < 0.999(Phase1B)')
            if (_get(r, 'solver.nonzero_status_count') or 0) != 0:
                hard_fail.append(f'{lbl} solver nonzero != 0(Phase1B)')
            p99 = _get(r, 'solver.solve_time_ms.p99')
            if p99 is not None and p99 >= dt_ms:
                hard_fail.append(f'{lbl} solve_time P99={p99:.2f} >= {dt_ms:.0f}ms')
        for f2 in hard_fail:
            if 'Phase1B' in f2 or 'solve_time P99' in f2:
                print(f'  [HARD-FAIL] {f2}')

        print('\n== 1B 功能门(dq_prev 真的进入求解器且正确)==')
        # zero 模式 dq_prev 全零;measured 模式与离线重算一致且非零
        za, zb = _get(a, 'dq_prev.all_zero'), _get(b, 'dq_prev.all_zero')
        func.append((f'{la}(zero) dq_prev 全为零', za is True))
        func.append((f'{lb}(measured) dq_prev 非零(确进入求解器)',
                     _get(b, 'dq_prev.dq_prev_input_abs_max', 0) > 1e-9))
        rc = _get(b, 'dq_prev.recompute_vs_logged_max_rad')
        func.append((f'{lb}(measured) dq_prev_input 与离线重算一致 (max {rc if rc is not None else float("nan"):.2e} rad)',
                     rc is not None and rc < 1e-6))
        for t, ok in func:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')

        print('\n== 1B 安全门(dq_prev 不得使跟踪/命令连续性/求解显著恶化)==')
        safe = []
        aea, aeb = _get(a, 'actual_error.steady.p95_mm'), _get(b, 'actual_error.steady.p95_mm')
        if aea and aeb:
            safe.append((f'actual_error P95 恶化<=2% ({aea:.2f}->{aeb:.2f} mm, {(aeb-aea)/aea*100:+.1f}%)',
                         aeb <= aea * 1.02))
        da, db = _get(a, 'command_discontinuity.max_tick_dq_pub_rad'), _get(b, 'command_discontinuity.max_tick_dq_pub_rad')
        if da is not None and db is not None:
            safe.append((f'最大命令跳变恶化<=5% ({da:.4f}->{db:.4f} rad, {(db-da)/da*100:+.1f}%)',
                         db <= da * 1.05))
        na, nb = _get(a, 'solver.nonzero_status_count'), _get(b, 'solver.nonzero_status_count')
        safe.append((f'solver 失败不增加 ({na}->{nb})', (nb or 0) <= (na or 0)))
        ga, gb = _get(a, 'guard.block_count'), _get(b, 'guard.block_count')
        safe.append((f'守卫拦截不增加 ({ga}->{gb})', (gb or 0) <= (ga or 0)))
        # exec gap 如实报告(非假指标)
        ea, eb = _get(a, 'exec_gap_cartesian.steady.rms_mm'), _get(b, 'exec_gap_cartesian.steady.rms_mm')
        print(f'  [报告] exec_gap cart rms {ea:.3f}->{eb:.3f} mm; '
              f'joint rms {_get(a,"exec_gap_joint_rad.steady_rms"):.4f}->{_get(b,"exec_gap_joint_rad.steady_rms"):.4f} rad '
              f'(如实报告,非假指标)')
        safe_fail = [t for t, ok in safe if not ok]
        for t, ok in safe:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')
        func += safe
        print('\n== 1B 性能评估(仅报告,无 -25% 门槛;改善/无效/退化据实)==')
        for name, path in [('actual mean', 'actual_error.steady.mean_mm'),
                           ('actual P95', 'actual_error.steady.p95_mm'),
                           ('cmd@next_tick mean', 'cmd_error_at_next_tick.steady.mean_mm'),
                           ('cmd@next_tick P95', 'cmd_error_at_next_tick.steady.p95_mm'),
                           ('publish-lag x', 'cmd_lag_ms_steady.publish_x'),
                           ('||u0-dq_prev|| RMS', 'dq_prev.u0_minus_dq_prev_norm_rad.rms'),
                           ('delta_u0 RMS', 'dq_prev.delta_u0_norm_rad.rms'),
                           ('exec_gap cart rms', 'exec_gap_cartesian.steady.rms_mm')]:
            va, vb = _get(a, path), _get(b, path)
            if va is not None and vb is not None:
                print(f'  {name:22s} {va:10.4f} -> {vb:10.4f}  ({vb - va:+.4f})')

    elif phase == '1C':
        # 求解健康硬门:command_usable>=99.9%,solver nonzero=0,solve_time P99<20ms。
        for lbl, r in ((la, a), (lb, b)):
            cur = _get(r, 'solver.command_usable_ratio')
            if cur is not None and cur < 0.999:
                hard_fail.append(f'{lbl} command_usable_ratio={cur:.4f} < 0.999(Phase1C)')
            if (_get(r, 'solver.nonzero_status_count') or 0) != 0:
                hard_fail.append(f'{lbl} solver nonzero != 0(Phase1C)')
            p99 = _get(r, 'solver.solve_time_ms.p99')
            if p99 is not None and p99 >= dt_ms:
                hard_fail.append(f'{lbl} solve_time P99={p99:.2f} >= {dt_ms:.0f}ms(Phase1C)')
        for f2 in hard_fail:
            if 'Phase1C' in f2:
                print(f'  [HARD-FAIL] {f2}')

        print('\n== 1C 机制门(RTI 早退解耦生效且干预真实存在)==')
        # treatment(fixed):成功 tick 中实际 nit==configured 的比例 >=99.5%
        conf_b = _get(b, 'rti.configured_iters')
        ratio_b = _get(b, 'rti.nit_eq_configured_ratio_success_steady')
        func.append((f'{lb}(fixed) 成功 tick nit==configured({conf_b}) 比例>=99.5% '
                     f'(实测 {100 * ratio_b:.2f}%)' if ratio_b is not None else f'{lb} 无 rti 数据',
                     ratio_b is not None and ratio_b >= 0.995))
        # baseline(legacy):必须确实存在早退(否则本轮 A/B 无有效干预)
        ee_a = _get(a, 'rti.early_exit_count_steady')
        eer_a = _get(a, 'rti.early_exit_ratio_steady')
        func.append((f'{la}(legacy) 确实存在提前退出 (early_exit {ee_a} 次, '
                     f'{100 * eer_a:.1f}%)' if eer_a is not None else f'{la} 无 rti 数据',
                     ee_a is not None and ee_a > 0))
        # treatment 早退应显著少于 baseline(fixed 只在 status!=0 早退,应≈0)
        eer_b = _get(b, 'rti.early_exit_ratio_steady')
        if eer_b is not None:
            print(f'  [报告] early_exit_ratio: {la}(legacy) {100 * eer_a:.1f}% vs '
                  f'{lb}(fixed) {100 * eer_b:.2f}%;mean_nit success: '
                  f'{_get(a,"rti.mean_nit_success_steady"):.3f} -> {_get(b,"rti.mean_nit_success_steady"):.3f}')
        for t, ok in func:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')

        print('\n== 1C 安全门(解耦不得使跟踪/命令/求解显著恶化)==')
        safe = []
        aea, aeb = _get(a, 'actual_error.steady.p95_mm'), _get(b, 'actual_error.steady.p95_mm')
        if aea and aeb:
            safe.append((f'actual_error P95 恶化<=2% ({aea:.2f}->{aeb:.2f} mm, {(aeb-aea)/aea*100:+.1f}%)',
                         aeb <= aea * 1.02))
        da, db = _get(a, 'command_discontinuity.max_tick_dq_pub_rad'), _get(b, 'command_discontinuity.max_tick_dq_pub_rad')
        if da is not None and db is not None:
            safe.append((f'最大命令跳变恶化<=5% ({da:.4f}->{db:.4f} rad, {(db-da)/da*100:+.1f}%)',
                         db <= da * 1.05))
        na, nb = _get(a, 'solver.nonzero_status_count'), _get(b, 'solver.nonzero_status_count')
        safe.append((f'solver 失败不增加 ({na}->{nb})', (nb or 0) <= (na or 0)))
        ga, gb = _get(a, 'guard.block_count'), _get(b, 'guard.block_count')
        safe.append((f'守卫拦截不增加 ({ga}->{gb})', (gb or 0) <= (ga or 0)))
        safe_fail = [t for t, ok in safe if not ok]
        for t, ok in safe:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')
        func += safe

        print('\n== 1C 性能评估(仅报告,不设 -25% 门槛)==')
        for name, path in [('actual mean', 'actual_error.steady.mean_mm'),
                           ('actual P95', 'actual_error.steady.p95_mm'),
                           ('cmd@next_tick mean', 'cmd_error_at_next_tick.steady.mean_mm'),
                           ('cmd@next_tick P95', 'cmd_error_at_next_tick.steady.p95_mm'),
                           ('actual 1Hz RMS', 'actual_sine_steady.base_freq.combined_rms_mm'),
                           ('exec_gap cart rms', 'exec_gap_cartesian.steady.rms_mm'),
                           ('solve_time p99 ms', 'solver.solve_time_ms.p99'),
                           ('accel near-limit', 'measured_accel_saturation_steady.overall_near_limit_ratio')]:
            va, vb = _get(a, path), _get(b, path)
            if va is not None and vb is not None:
                print(f'  {name:22s} {va:10.4f} -> {vb:10.4f}  ({vb - va:+.4f})')

    elif phase == '1D':
        # 求解健康硬门(与 1C 同):command_usable>=99.9%,solver nonzero=0,solve_time P99<20ms;
        # fixed 成功 tick nit==configured 比例>=99.5%(两组都是 fixed)。
        for lbl, r in ((la, a), (lb, b)):
            cur = _get(r, 'solver.command_usable_ratio')
            if cur is not None and cur < 0.999:
                hard_fail.append(f'{lbl} command_usable_ratio={cur:.4f} < 0.999(Phase1D)')
            if (_get(r, 'solver.nonzero_status_count') or 0) != 0:
                hard_fail.append(f'{lbl} solver nonzero != 0(Phase1D)')
            p99 = _get(r, 'solver.solve_time_ms.p99')
            if p99 is not None and p99 >= dt_ms:
                hard_fail.append(f'{lbl} solve_time P99={p99:.2f} >= {dt_ms:.0f}ms(Phase1D)')
            nr = _get(r, 'rti.nit_eq_configured_ratio_success_steady')
            if nr is not None and nr < 0.995:
                hard_fail.append(f'{lbl} fixed nit==configured 比例 {100 * nr:.2f}% < 99.5%(Phase1D)')
        for f2 in hard_fail:
            if 'Phase1D' in f2:
                print(f'  [HARD-FAIL] {f2}')

        print('\n== 1D 机制门(gain 生效且为唯一变量)==')
        func.append((f'唯一变量是 velocity_feedforward_gain ({gain_a}->{gain_b})',
                     gain_a != gain_b and _num(gain_a, -1) == 0.7 and _num(gain_b, -1) == 1.0))
        for t, ok in func:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')

        print('\n== 1D 安全门(gain 变化不得使跟踪/命令/速度/加速度显著恶化)==')
        safe = []
        aea, aeb = _get(a, 'actual_error.steady.p95_mm'), _get(b, 'actual_error.steady.p95_mm')
        if aea and aeb:
            safe.append((f'actual_error P95 恶化<=2% ({aea:.2f}->{aeb:.2f} mm, {(aeb-aea)/aea*100:+.1f}%)',
                         aeb <= aea * 1.02))
        da, db = _get(a, 'command_discontinuity.max_tick_dq_pub_rad'), _get(b, 'command_discontinuity.max_tick_dq_pub_rad')
        if da is not None and db is not None:
            safe.append((f'最大关节跳变恶化<=5% ({da:.4f}->{db:.4f} rad, {(db-da)/da*100:+.1f}%)', db <= da * 1.05))
        # 实测速度不得超 1.0 rad/s(留合理容差 3%)
        vpk = _get(b, 'measured_vel_saturation_steady.abs_peak')
        if vpk is not None:
            safe.append((f'{lb} 实测关节速度峰值 <=1.03 rad/s (实测 {vpk:.3f})', vpk <= 1.03))
        # 实测加速度不得明显超 5.0(留 10% 容差)
        apk = _get(b, 'measured_accel_saturation_steady.abs_peak')
        if apk is not None:
            safe.append((f'{lb} 实测加速度峰值 <=5.5 rad/s^2 (实测 {apk:.3f})', apk <= 5.5))
        na, nb = _get(a, 'solver.nonzero_status_count'), _get(b, 'solver.nonzero_status_count')
        safe.append((f'solver 失败不增加 ({na}->{nb})', (nb or 0) <= (na or 0)))
        ga, gb = _get(a, 'guard.block_count'), _get(b, 'guard.block_count')
        safe.append((f'守卫拦截不增加 ({ga}->{gb})', (gb or 0) <= (ga or 0)))
        for t, ok in safe:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')
        func += safe

        print('\n== 1D 执行匹配(机制核心,据实报告)+ 末端/预算 ==')
        for name, path in [
                ('exec_gap joint rms', 'phase1d_executor_match.exec_gap_joint_rad.rms'),
                ('exec_gap joint p95norm', 'phase1d_executor_match.exec_gap_joint_rad.p95_norm'),
                ('exec_gap cart rms mm', 'phase1d_executor_match.exec_gap_cartesian_mm.rms'),
                ('exec_gap cart p95 mm', 'phase1d_executor_match.exec_gap_cartesian_mm.p95'),
                ('vel_gap dqpub-dqmeas rms', 'phase1d_executor_match.vel_gap_dqpub_vs_dqmeas_rad_s.rms'),
                ('vel_gap p95norm', 'phase1d_executor_match.vel_gap_dqpub_vs_dqmeas_rad_s.p95_norm'),
                ('dq_pub near vmax ratio', 'phase1d_executor_match.dq_pub_near_max_velocity_ratio'),
                ('dq_pub abs max', 'phase1d_executor_match.dq_pub_abs_max_rad_s'),
                ('actual mean mm', 'actual_error.steady.mean_mm'),
                ('actual P95 mm', 'actual_error.steady.p95_mm'),
                ('cmd@next_tick P95 mm', 'cmd_error_at_next_tick.steady.p95_mm'),
                ('actual 1Hz RMS mm', 'actual_sine_steady.base_freq.combined_rms_mm'),
                ('worst phase bin mm', 'actual_phase_bins_steady.worst_bin_mean_mm'),
                ('actual lag x ms', 'actual_lag_ms_steady.quad_x'),
                ('meas vel peak', 'measured_vel_saturation_steady.abs_peak'),
                ('meas accel peak', 'measured_accel_saturation_steady.abs_peak'),
                ('accel near-limit ratio', 'measured_accel_saturation_steady.overall_near_limit_ratio'),
                ('solve_time p99 ms', 'solver.solve_time_ms.p99')]:
            va, vb = _get(a, path), _get(b, path)
            if va is not None and vb is not None:
                print(f'  {name:26s} {va:10.4f} -> {vb:10.4f}  ({vb - va:+.4f})')

    elif phase == '1E':
        # 求解健康硬门(与 1D 同): command_usable>=99.9%, solver nonzero=0, P99<20ms, nit 比例。
        for lbl, r in ((la, a), (lb, b)):
            cur = _get(r, 'solver.command_usable_ratio')
            if cur is not None and cur < 0.999:
                hard_fail.append(f'{lbl} command_usable_ratio={cur:.4f} < 0.999(Phase1E)')
            if (_get(r, 'solver.nonzero_status_count') or 0) != 0:
                hard_fail.append(f'{lbl} solver nonzero != 0(Phase1E)')
            p99 = _get(r, 'solver.solve_time_ms.p99')
            if p99 is not None and p99 >= dt_ms:
                hard_fail.append(f'{lbl} solve_time P99={p99:.2f} >= {dt_ms:.0f}ms(Phase1E)')
            nr = _get(r, 'rti.nit_eq_configured_ratio_success_steady')
            if nr is not None and nr < 0.995:
                hard_fail.append(f'{lbl} fixed nit==configured 比例 {100 * nr:.2f}% < 99.5%(Phase1E)')
        for f2 in hard_fail:
            if 'Phase1E' in f2:
                print(f'  [HARD-FAIL] {f2}')

        print('\n== 1E 机制门(guard_mode 是唯一变量)==')
        func.append((f'唯一变量是 guard_mode ({gm_a}->{gm_b})',
                     gm_a == 'legacy_fixed_step' and gm_b == 'expected_residual'))
        for t, ok in func:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')

        print('\n== 1E 安全门(新守卫不得破坏正常闭环)==')
        safe = []
        aea, aeb = _get(a, 'actual_error.steady.p95_mm'), _get(b, 'actual_error.steady.p95_mm')
        if aea and aeb:
            safe.append((f'actual_error P95 恶化<=2% ({aea:.2f}->{aeb:.2f} mm, '
                         f'{(aeb - aea) / aea * 100:+.1f}%)', aeb <= aea * 1.02))
        da = _get(a, 'command_discontinuity.max_tick_dq_pub_rad')
        db = _get(b, 'command_discontinuity.max_tick_dq_pub_rad')
        if da is not None and db is not None:
            safe.append((f'最大关节命令跳变恶化<=5% ({da:.4f}->{db:.4f} rad, '
                         f'{(db - da) / da * 100:+.1f}%)', db <= da * 1.05))
        for lbl, r in ((la, a), (lb, b)):
            vpk = _get(r, 'measured_vel_saturation_steady.abs_peak')
            if vpk is not None:
                safe.append((f'{lbl} 实测速度峰值 <=1.03 rad/s (实测 {vpk:.3f})', vpk <= 1.03))
            apk = _get(r, 'measured_accel_saturation_steady.abs_peak')
            if apk is not None:
                safe.append((f'{lbl} 实测加速度峰值 <=5.5 rad/s^2 (实测 {apk:.3f})', apk <= 5.5))
        na, nb = _get(a, 'solver.nonzero_status_count'), _get(b, 'solver.nonzero_status_count')
        safe.append((f'solver 失败不增加 ({na}->{nb})', (nb or 0) <= (na or 0)))
        # 正常生产工况: treatment 误拦截必须为 0;guard_fault=0;无锁死;契约违反=0。
        gb = _get(b, 'guard.block_count')
        safe.append((f'{lb} 正常工况误拦截==0 (实际 {gb})', (gb or 0) == 0))
        ga = _get(a, 'guard.block_count')
        safe.append((f'{la} baseline 守卫拦截==0 (实际 {ga})', (ga or 0) == 0))
        for lbl, r in ((la, a), (lb, b)):
            g1e = r.get('guard_1e') or {}
            if r.get('guard_mode') == 'expected_residual':
                safe.append((f'{lbl} guard_fault==0', not g1e.get('fault', False)))
                mcb = g1e.get('max_consecutive_blocks', 0)
                safe.append((f'{lbl} 无锁死(max_consecutive={mcb})', mcb <= 2))
                safe.append((f'{lbl} solver 步长契约违反==0 '
                             f'(实际 {g1e.get("solver_step_violation_count", 0)})',
                             g1e.get('solver_step_violation_count', 0) == 0))
        for t, ok in safe:
            print(f'  [{"PASS" if ok else "FAIL"}] {t}')
        func += safe

        # 投影必须逐条报告(不能只报总数);正常工况应为 0,若发生逐条列 λ 与残差。
        print('\n== 1E 守卫诊断(残差分布/裕量/投影逐条)==')
        for lbl, r in ((la, a), (lb, b)):
            g1e = r.get('guard_1e') or {}
            if g1e and g1e.get('r_pos_mm'):
                rp = g1e['r_pos_mm']
                mm = g1e.get('margin_min_mm')
                mm = float('nan') if mm is None else mm
                print(f'  {lbl}: mode={g1e.get("mode")} eval_ticks={g1e.get("evaluated_ticks")} '
                      f'r_pos p99/p99.9/max={rp.get("p99", float("nan")):.3f}/'
                      f'{rp.get("p999", float("nan")):.3f}/{rp.get("max", float("nan")):.3f}mm '
                      f'margin_min={mm:.3f}mm '
                      f'would_exceed={g1e.get("would_exceed_count")} '
                      f'projections={g1e.get("projection_count")}')
                for evp in g1e.get('projection_events', []):
                    print(f'    [PROJECTION] tick={evp["tick"]} t={evp["t_solve"]:.3f}s '
                          f'λ={evp["lambda"]:.3f} r_pos={evp["r_pos_mm"]:.2f}mm '
                          f'r_allow={evp["r_allow_mm"]:.2f}mm')
            else:
                print(f'  {lbl}: (legacy 模式,无 expected 残差诊断)')
        print('\n== 1E 性能对照(据实报告,无改善门)==')
        for name, path in [
                ('actual mean mm', 'actual_error.steady.mean_mm'),
                ('actual P95 mm', 'actual_error.steady.p95_mm'),
                ('cmd@next_tick P95 mm', 'cmd_error_at_next_tick.steady.p95_mm'),
                ('exec_gap cart rms mm', 'phase1d_executor_match.exec_gap_cartesian_mm.rms'),
                ('actual 1Hz RMS mm', 'actual_sine_steady.base_freq.combined_rms_mm'),
                ('meas vel peak', 'measured_vel_saturation_steady.abs_peak'),
                ('meas accel peak', 'measured_accel_saturation_steady.abs_peak'),
                ('solve_time p99 ms', 'solver.solve_time_ms.p99')]:
            va, vb = _get(a, path), _get(b, path)
            if va is not None and vb is not None:
                print(f'  {name:26s} {va:10.4f} -> {vb:10.4f}  ({vb - va:+.4f})')

    func_fail = [t for t, ok in func if not ok]

    # ---------------- 总判决 ----------------
    overall_fail = bool(hard_fail) or bool(func_fail) or (kpi_ok is False)
    print('\n== 总判决 ==')
    if hard_fail:
        print(f'  实验无效/硬门失败 {len(hard_fail)} 项 -> 数据不可作正式 A/B 证据')
    if func_fail:
        print(f'  功能/安全门失败 {len(func_fail)} 项')
    if kpi_ok is False:
        print('  Phase1A 性能 KPI(P95 -25%)未达标(如实标 FAIL)')
    if phase == '1B':
        print('  Phase1B 无 -25% 门槛;核心是验证真实 dq_prev 的模型连续性/执行匹配,结果据实。')
    if phase == '1C':
        print('  Phase1C 无 -25% 门槛;核心是验证 RTI 早退解耦"机制生效且安全",性能据实报告。')
    if phase == '1D':
        print('  Phase1D 无改善门槛;核心是执行匹配(exec_gap)与安全预算。是否提默认见汇总(需 3 配对中位改善)。')
    if phase == '1E':
        print('  Phase1E 无改善门槛;核心是"新守卫不破坏正常闭环"(基线 guard 本就=0),'
              '异常拦截能力由故障注入单测证明。是否提默认见 3 配对汇总。')
    print(f'  COMPARE 结果: {"FAIL" if overall_fail else "PASS"}  -> 停止等待审核')
    sys.exit(1 if overall_fail else 0)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='mode', required=True)
    pa = sub.add_parser('analyze')
    pa.add_argument('--loop-csv', required=True)
    pa.add_argument('--params-json', required=True)
    pa.add_argument('--prefix', default=None, help='visualizer CSV 前缀(一致性交叉核对,可选)')
    pa.add_argument('--label', default='run')
    pa.add_argument('--out', default=None)
    pa.add_argument('--discard-initial-cycles', type=int, default=2)
    pa.set_defaults(func=analyze)
    pc = sub.add_parser('compare')
    pc.add_argument('baseline')
    pc.add_argument('after')
    pc.set_defaults(func=compare)
    args = p.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
