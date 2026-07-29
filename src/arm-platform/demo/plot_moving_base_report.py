#!/usr/bin/env python3
"""Generate four clean uncertainty-run figures from recorded CSV data.

The trajectory figures intentionally contain only expected and actual
end-effector values.  Command-FK remains a separate diagnostic artifact and
is never loaded here.  Base truth for the spectrum figure is reconstructed
from the parameter snapshot and the shared disturbance model because the
``base_pos_*`` columns in a model-belief loop CSV are controller belief, not
physical truth.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import re
from typing import Iterable, Sequence

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
from scipy.signal import welch

import moving_base_trajectory_config as moving_traj


_NOTO_CJK_PATH = Path(
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if _NOTO_CJK_PATH.is_file():
    # Matplotlib 3.5 does not discover every face in a TTC through fontconfig.
    # Registering the collection explicitly selects its first face, which
    # contains Chinese, Latin letters, digits, and scientific unit symbols.
    font_manager.fontManager.addfont(str(_NOTO_CJK_PATH))
    _REPORT_FONT_FAMILY = font_manager.FontProperties(
        fname=str(_NOTO_CJK_PATH)).get_name()
else:
    _REPORT_FONT_FAMILY = 'DejaVu Sans'

plt.rcParams.update({
    'font.family': _REPORT_FONT_FAMILY,
    'mathtext.fontset': 'dejavusans',
    'axes.unicode_minus': False,
})

EXPECTED_COLOR = '#2ca02c'
ACTUAL_COLOR = '#d62728'
DISTURBANCE_COLOR = '#a61e9f'
SECOND_GUST_COLOR = '#6f0878'
AXIS_ERROR_COLORS = ('#1f77b4', '#ff7f0e', '#2ca02c')
STAMP_RE = re.compile(r'(\d{8}_\d{6})')
MAIN_TRAJECTORY_RE = re.compile(
    r'.+_trajectory_(\d{8}_\d{6})\.csv$')


@dataclass(frozen=True)
class RunFiles:
    trajectory_csv: Path
    expected_path_csv: Path | None
    loop_csv: Path
    params_json: Path
    trajectory_stamp: str
    controller_stamp: str


@dataclass(frozen=True)
class DisturbanceEvent:
    number: int
    start_sec: float
    peak_sec: float
    end_sec: float
    label: str


def _stamp(path: Path) -> str:
    match = STAMP_RE.search(path.name)
    if match is None:
        raise ValueError(f'No YYYYMMDD_HHMMSS timestamp in {path}')
    return match.group(1)


def _latest_not_after(paths: Iterable[Path], reference_stamp: str) -> Path:
    candidates = sorted(paths, key=_stamp)
    if not candidates:
        raise FileNotFoundError('No matching run files found')
    eligible = [path for path in candidates if _stamp(path) <= reference_stamp]
    return eligible[-1] if eligible else candidates[-1]


def discover_run(
        data_dir: Path,
        trajectory_csv: Path | None = None,
        loop_csv: Path | None = None,
        params_json: Path | None = None) -> RunFiles:
    """Resolve the newest complete trajectory/controller file set."""
    data_dir = Path(data_dir).expanduser().resolve()
    if trajectory_csv is None:
        trajectory_candidates = [
            path for path in data_dir.glob('*_trajectory_*.csv')
            if MAIN_TRAJECTORY_RE.fullmatch(path.name)
        ]
        if not trajectory_candidates:
            raise FileNotFoundError(
                f'No main trajectory CSV found in {data_dir}')
        trajectory_csv = max(trajectory_candidates, key=_stamp)
    else:
        trajectory_csv = Path(trajectory_csv).expanduser().resolve()
    trajectory_stamp = _stamp(trajectory_csv)

    if params_json is None:
        params_json = _latest_not_after(
            data_dir.glob('moving_base_mpc_params_*.json'),
            trajectory_stamp,
        )
    else:
        params_json = Path(params_json).expanduser().resolve()
    controller_stamp = _stamp(params_json)

    if loop_csv is None:
        exact_loop = data_dir / (
            f'moving_base_mpc_loop_{controller_stamp}.csv')
        loop_csv = (
            exact_loop if exact_loop.exists()
            else _latest_not_after(
                data_dir.glob('moving_base_mpc_loop_*.csv'),
                trajectory_stamp,
            )
        )
    else:
        loop_csv = Path(loop_csv).expanduser().resolve()

    expected_path_csv = trajectory_csv.with_name(
        trajectory_csv.stem + '_expected_path.csv')
    if not expected_path_csv.exists():
        expected_path_csv = None
    return RunFiles(
        trajectory_csv=trajectory_csv,
        expected_path_csv=expected_path_csv,
        loop_csv=loop_csv,
        params_json=params_json,
        trajectory_stamp=trajectory_stamp,
        controller_stamp=controller_stamp,
    )


def _load_csv_columns(path: Path, columns: Sequence[str]) -> dict[str, np.ndarray]:
    values = {name: [] for name in columns}
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or ())
        missing = [name for name in columns if name not in fieldnames]
        if missing:
            raise ValueError(f'{path} is missing columns: {missing}')
        for row in reader:
            for name in columns:
                values[name].append(float(row[name]))
    return {
        name: np.asarray(column_values, dtype=float)
        for name, column_values in values.items()
    }


def load_trajectory(path: Path) -> dict[str, np.ndarray]:
    columns = [
        'time_sec',
        'path_time_sec',
        'expected_x', 'expected_y', 'expected_z',
        'actual_x', 'actual_y', 'actual_z',
        'pen_down', 'stroke_id',
    ]
    values = _load_csv_columns(path, columns)
    expected = np.column_stack([
        values['expected_x'], values['expected_y'], values['expected_z']])
    actual = np.column_stack([
        values['actual_x'], values['actual_y'], values['actual_z']])
    finite = (
        np.isfinite(values['time_sec'])
        & np.all(np.isfinite(expected), axis=1)
        & np.all(np.isfinite(actual), axis=1)
    )
    if not np.any(finite):
        raise ValueError(f'No finite trajectory rows in {path}')
    return {
        'time_sec': values['time_sec'][finite],
        'path_time_sec': values['path_time_sec'][finite],
        'expected': expected[finite],
        'actual': actual[finite],
        'error': actual[finite] - expected[finite],
        'pen_down': values['pen_down'][finite].astype(bool),
        'stroke_id': values['stroke_id'][finite].astype(int),
    }


def trajectory_segments(
        points: np.ndarray,
        pen_down: np.ndarray,
        stroke_id: np.ndarray) -> list[np.ndarray]:
    """Split points at every pen-up interval or stroke-id transition."""
    segments: list[np.ndarray] = []
    current: list[np.ndarray] = []
    current_stroke: int | None = None
    for point, is_down, next_stroke in zip(points, pen_down, stroke_id):
        next_stroke = int(next_stroke)
        if not bool(is_down) or not np.all(np.isfinite(point)):
            if len(current) >= 2:
                segments.append(np.asarray(current, dtype=float))
            current = []
            current_stroke = None
            continue
        if current_stroke is not None and next_stroke != current_stroke:
            if len(current) >= 2:
                segments.append(np.asarray(current, dtype=float))
            current = []
        current.append(np.asarray(point, dtype=float))
        current_stroke = next_stroke
    if len(current) >= 2:
        segments.append(np.asarray(current, dtype=float))
    return segments


def load_expected_path_segments(path: Path | None) -> list[np.ndarray]:
    if path is None:
        return []
    values = _load_csv_columns(
        path,
        ['stroke_id', 'index', 'expected_x', 'expected_y', 'expected_z'],
    )
    stroke_ids = values['stroke_id'].astype(int)
    indices = values['index'].astype(int)
    points = np.column_stack([
        values['expected_x'], values['expected_y'], values['expected_z']])
    segments = []
    for stroke_id in dict.fromkeys(stroke_ids.tolist()):
        mask = stroke_ids == stroke_id
        order = np.argsort(indices[mask])
        segment = points[mask][order]
        if len(segment) >= 2:
            segments.append(segment)
    return segments


def smooth_segments(
        segments: Sequence[np.ndarray], window: int) -> list[np.ndarray]:
    """Apply display-only triangular smoothing within each segment."""
    window = int(window)
    if window < 1 or window % 2 == 0:
        raise ValueError('smoothing window must be a positive odd integer')
    if window == 1:
        return [np.asarray(segment, dtype=float).copy()
                for segment in segments]
    half = window // 2
    ramp = np.arange(1, half + 2, dtype=float)
    kernel = np.concatenate([ramp, ramp[-2::-1]])
    kernel /= kernel.sum()
    output = []
    for segment in segments:
        points = np.asarray(segment, dtype=float)
        if len(points) < window:
            output.append(points.copy())
            continue
        padded = np.pad(points, ((half, half), (0, 0)), mode='edge')
        smoothed = np.column_stack([
            np.convolve(padded[:, axis], kernel, mode='valid')
            for axis in range(3)
        ])
        smoothed[0] = points[0]
        smoothed[-1] = points[-1]
        output.append(smoothed)
    return output


def disturbance_events(
        profile: str,
        start_sec: float,
        end_sec: float,
        base_motion_time_scale: float) -> list[DisturbanceEvent]:
    """Return visible event windows while preserving global event numbering."""
    profile = str(profile).strip().lower()
    scale = float(base_motion_time_scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError('base_motion_time_scale must be positive')
    if profile == 'gust':
        period = moving_traj.BASE_DISTURBANCE_GUST_PERIOD_SEC * scale
        width = moving_traj.BASE_DISTURBANCE_GUST_WIDTH_SEC * scale
        first_k = max(0, int(math.floor((start_sec - width) / period)))
        last_k = int(math.floor(end_sec / period))
        events = []
        for k in range(first_k, last_k + 1):
            event_start = k * period
            event_end = event_start + width
            if event_end < start_sec or event_start > end_sec:
                continue
            events.append(DisturbanceEvent(
                number=k + 1,
                start_sec=event_start,
                peak_sec=event_start + 0.5 * width,
                end_sec=event_end,
                label='gust',
            ))
        return events
    if profile == 'fault':
        event_start = moving_traj.BASE_DISTURBANCE_FAULT_START_SEC * scale
        event_end = moving_traj.BASE_DISTURBANCE_FAULT_END_SEC * scale
        if event_end < start_sec or event_start > end_sec:
            return []
        return [DisturbanceEvent(
            number=1,
            start_sec=event_start,
            peak_sec=event_start,
            end_sec=event_end,
            label='fault bias',
        )]
    return []


def _plot_segments(axis, segments, label: str | None = None, **kwargs):
    for index, segment in enumerate(segments):
        axis.plot(
            segment[:, 0], segment[:, 1], segment[:, 2],
            label=label if index == 0 else None,
            **kwargs,
        )


def _equal_3d_axes(axis, arrays: Sequence[np.ndarray]):
    finite_arrays = [
        np.asarray(array, dtype=float)[
            np.all(np.isfinite(np.asarray(array, dtype=float)), axis=1)]
        for array in arrays
        if np.asarray(array).size
    ]
    if not finite_arrays:
        return
    points = np.vstack(finite_arrays)
    low = points.min(axis=0)
    high = points.max(axis=0)
    center = 0.5 * (low + high)
    radius = max(0.05, 0.53 * float(np.max(high - low)))
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)
    axis.set_box_aspect((1.0, 1.0, 1.0))


def _apply_common_axis_style(axis):
    axis.grid(True, color='#d5d9df', linewidth=0.7, alpha=0.8)
    axis.spines['top'].set_visible(False)
    axis.spines['right'].set_visible(False)


def _shade_events(axis, events: Sequence[DisturbanceEvent]):
    for event in events:
        axis.axvspan(
            event.start_sec, event.end_sec,
            color=DISTURBANCE_COLOR, alpha=0.075, linewidth=0.0,
            zorder=0,
        )
        axis.axvline(
            event.start_sec, color=DISTURBANCE_COLOR,
            linestyle=':', linewidth=0.65, alpha=0.50, zorder=1,
        )


def _nearest_value(times: np.ndarray, values: np.ndarray, time_sec: float) -> float:
    index = int(np.argmin(np.abs(times - float(time_sec))))
    return float(values[index])


def _add_event_arrows(
        axis,
        times: np.ndarray,
        actual_axis: np.ndarray,
        events: Sequence[DisturbanceEvent],
        add_second_gust_label: bool):
    y_low, y_high = axis.get_ylim()
    y_span = max(y_high - y_low, 1e-6)
    y_mid = 0.5 * (y_low + y_high)
    for event in events:
        tip_y = _nearest_value(times, actual_axis, event.start_sec)
        direction = -1.0 if tip_y > y_mid else 1.0
        color = (
            SECOND_GUST_COLOR
            if event.label == 'gust' and event.number == 2
            else DISTURBANCE_COLOR
        )
        width = 1.35 if event.number == 2 else 0.85
        axis.annotate(
            '',
            xy=(event.start_sec, tip_y),
            xytext=(event.start_sec, tip_y + direction * 0.075 * y_span),
            arrowprops={
                'arrowstyle': '-|>',
                'color': color,
                'linewidth': width,
                'mutation_scale': 9 if event.number == 2 else 7,
            },
            zorder=6,
        )
        peak_y = _nearest_value(times, actual_axis, event.peak_sec)
        axis.plot(
            event.peak_sec, peak_y, marker='o', markersize=3.2,
            markerfacecolor=color, markeredgewidth=0.0, color=color,
            zorder=7,
        )
        if (
            add_second_gust_label
            and event.label == 'gust'
            and event.number == 2
        ):
            axis.annotate(
                '全局第 2 次阵风\n8.0 s 加入，8.4 s 达峰',
                xy=(event.peak_sec, peak_y),
                xytext=(event.peak_sec + 5.0, y_high - 0.13 * y_span),
                fontsize=9,
                color=SECOND_GUST_COLOR,
                ha='left',
                va='top',
                bbox={
                    'boxstyle': 'round,pad=0.3',
                    'facecolor': 'white',
                    'edgecolor': SECOND_GUST_COLOR,
                    'alpha': 0.92,
                },
                arrowprops={
                    'arrowstyle': '->',
                    'color': SECOND_GUST_COLOR,
                    'linewidth': 1.2,
                },
                zorder=8,
            )


def plot_trajectory_3d(
        path: Path,
        expected_segments: Sequence[np.ndarray],
        actual_segments: Sequence[np.ndarray],
        run_label: str,
        dpi: int):
    fig = plt.figure(figsize=(10.5, 8.5))
    axis = fig.add_subplot(111, projection='3d')
    _plot_segments(
        axis, expected_segments, label='期望值',
        color=EXPECTED_COLOR, linestyle='--', linewidth=1.9, alpha=0.95,
    )
    _plot_segments(
        axis, actual_segments, label='实际值',
        color=ACTUAL_COLOR, linewidth=1.15, alpha=0.95,
    )
    axis.set_xlabel('世界坐标 x（m）', labelpad=8)
    axis.set_ylabel('世界坐标 y（m）', labelpad=8)
    axis.set_zlabel('世界坐标 z（m）', labelpad=8)
    axis.set_title('末端三维轨迹', pad=16, fontweight='bold')
    axis.view_init(elev=25, azim=-58)
    axis.legend(loc='upper right', frameon=True, framealpha=0.95)
    _equal_3d_axes(axis, [*expected_segments, *actual_segments])
    fig.suptitle(run_label, fontsize=11, y=0.965)
    fig.text(
        0.5, 0.018,
        '轨迹在每次抬笔、笔画切换和绘图面切换处断开。',
        ha='center', fontsize=9, color='#4b5563',
    )
    fig.savefig(path, dpi=dpi, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def plot_axis_tracking(
        path: Path,
        times: np.ndarray,
        expected: np.ndarray,
        actual: np.ndarray,
        events: Sequence[DisturbanceEvent],
        run_label: str,
        dpi: int):
    fig, axes = plt.subplots(
        3, 1, figsize=(14.0, 9.0), sharex=True,
        gridspec_kw={'hspace': 0.10},
    )
    axis_names = ('x', 'y', 'z')
    for index, axis in enumerate(axes):
        axis.plot(
            times, expected[:, index], color=EXPECTED_COLOR,
            linestyle='--', linewidth=1.45, label='期望值', zorder=2,
        )
        axis.plot(
            times, actual[:, index], color=ACTUAL_COLOR,
            linewidth=1.0, label='实际值', zorder=3,
        )
        _shade_events(axis, events)
        axis.relim()
        axis.autoscale_view()
        _add_event_arrows(
            axis, times, actual[:, index], events,
            add_second_gust_label=(index == 0),
        )
        axis.set_ylabel(f'{axis_names[index]} 轴位置（m）')
        _apply_common_axis_style(axis)
    axes[0].legend(
        handles=[
            Line2D([], [], color=EXPECTED_COLOR, linestyle='--',
                   linewidth=1.6, label='期望值'),
            Line2D([], [], color=ACTUAL_COLOR, linewidth=1.2,
                   label='实际值'),
            Patch(facecolor=DISTURBANCE_COLOR, alpha=0.12,
                  label='阵风作用区间'),
            Line2D([], [], color=DISTURBANCE_COLOR, marker='o',
                   linestyle=':', linewidth=0.8, markersize=4,
                   label='加入箭头 / 峰值圆点'),
        ],
        loc='upper right', ncol=2, framealpha=0.95,
    )
    axes[-1].set_xlabel('物理基座时间（s）')
    axes[-1].set_xlim(float(times[0]), float(times[-1]))
    fig.suptitle(
        f'三轴轨迹跟踪与扰动时序\n{run_label}',
        fontsize=13, fontweight='bold', y=0.985,
    )
    if events and events[0].label == 'gust' and events[0].number == 2:
        fig.text(
            0.5, 0.018,
            '第 1 次阵风发生在预热阶段的 0.0–0.8 s；'
            '本次记录中的第一个完整事件是全局第 2 次阵风。',
            ha='center', fontsize=9, color='#4b5563',
        )
    fig.subplots_adjust(top=0.90, bottom=0.09, left=0.08, right=0.98)
    fig.savefig(path, dpi=dpi, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def _moving_rms(values: np.ndarray, sample_count: int) -> np.ndarray:
    sample_count = max(1, int(sample_count))
    kernel = np.ones(sample_count, dtype=float) / sample_count
    return np.sqrt(np.convolve(values * values, kernel, mode='same'))


def plot_tracking_error(
        path: Path,
        times: np.ndarray,
        error: np.ndarray,
        events: Sequence[DisturbanceEvent],
        run_label: str,
        dpi: int) -> dict[str, float]:
    error_mm = 1000.0 * np.asarray(error, dtype=float)
    error_norm_mm = np.linalg.norm(error_mm, axis=1)
    rms_mm = float(np.sqrt(np.mean(error_norm_mm ** 2)))
    p95_mm = float(np.percentile(error_norm_mm, 95.0))
    max_index = int(np.argmax(error_norm_mm))
    max_mm = float(error_norm_mm[max_index])
    max_time_sec = float(times[max_index])
    median_dt = float(np.median(np.diff(times)))
    rolling_count = max(1, int(round(0.5 / median_dt)))
    rolling_rms = _moving_rms(error_norm_mm, rolling_count)

    fig, axes = plt.subplots(
        2, 1, figsize=(14.0, 8.2), sharex=True,
        gridspec_kw={'height_ratios': [1.15, 1.0], 'hspace': 0.10},
    )
    for axis_index, (axis_name, color) in enumerate(
            zip(('x', 'y', 'z'), AXIS_ERROR_COLORS)):
        axes[0].plot(
            times, error_mm[:, axis_index], color=color,
            linewidth=0.85, alpha=0.85,
            label=f'{axis_name} 轴误差',
        )
    axes[0].axhline(0.0, color='#4b5563', linewidth=0.8)
    axes[0].set_ylabel('有符号误差（mm）')
    axes[0].legend(loc='upper right', ncol=3, framealpha=0.95)

    axes[1].plot(
        times, error_norm_mm, color=ACTUAL_COLOR, linewidth=0.75,
        alpha=0.58, label='原始误差范数',
    )
    axes[1].plot(
        times, rolling_rms, color='#111827', linewidth=1.35,
        label='0.5 s 滑动均方根',
    )
    axes[1].axhline(
        rms_mm, color='#2563eb', linestyle='--', linewidth=1.0,
        label=f'全局均方根 = {rms_mm:.3f} mm',
    )
    axes[1].axhline(
        p95_mm, color='#7c3aed', linestyle=':', linewidth=1.0,
        label=f'95% 分位值 = {p95_mm:.3f} mm',
    )
    axes[1].scatter(
        [max_time_sec], [max_mm], color=SECOND_GUST_COLOR,
        s=28, zorder=6,
    )
    axes[1].annotate(
        f'最大值 {max_mm:.3f} mm\n时刻 {max_time_sec:.3f} s',
        xy=(max_time_sec, max_mm),
        xytext=(max_time_sec + 5.0, max_mm * 0.83),
        color=SECOND_GUST_COLOR,
        fontsize=9,
        arrowprops={
            'arrowstyle': '->',
            'color': SECOND_GUST_COLOR,
            'linewidth': 1.1,
        },
        bbox={
            'boxstyle': 'round,pad=0.25',
            'facecolor': 'white',
            'edgecolor': SECOND_GUST_COLOR,
            'alpha': 0.92,
        },
    )
    axes[1].set_ylabel('误差范数（mm）')
    axes[1].set_xlabel('物理基座时间（s）')
    axes[1].legend(loc='upper right', ncol=2, framealpha=0.95)

    for axis in axes:
        _shade_events(axis, events)
        _apply_common_axis_style(axis)
    axes[-1].set_xlim(float(times[0]), float(times[-1]))
    fig.suptitle(
        f'实际值 − 期望值跟踪误差\n{run_label}',
        fontsize=13, fontweight='bold', y=0.985,
    )
    fig.subplots_adjust(top=0.88, bottom=0.09, left=0.08, right=0.98)
    fig.savefig(path, dpi=dpi, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return {
        'rms_mm': rms_mm,
        'p95_mm': p95_mm,
        'max_mm': max_mm,
        'max_time_sec': max_time_sec,
    }


def _validate_snapshot_against_model(params: dict):
    config = params.get('trajectory_config', {})
    comparisons = {
        'BASE_CIRCLE_RADIUS': moving_traj.BASE_CIRCLE_RADIUS,
        'BASE_PERIOD_SEC': moving_traj.BASE_PERIOD_SEC,
        'BASE_HEIGHT': moving_traj.BASE_HEIGHT,
        'BASE_Z_AMPLITUDE': moving_traj.BASE_Z_AMPLITUDE,
        'BASE_Z_PHASE': moving_traj.BASE_Z_PHASE,
    }
    mismatches = []
    for name, current_value in comparisons.items():
        if name not in config:
            continue
        if not math.isclose(
                float(config[name]), float(current_value),
                rel_tol=1e-12, abs_tol=1e-12):
            mismatches.append(
                f'{name}: snapshot={config[name]}, current={current_value}')
    if mismatches:
        raise ValueError(
            'Trajectory snapshot does not match the current reconstruction '
            f'model: {mismatches}')


def reconstruct_base_truth(
        start_sec: float,
        end_sec: float,
        params: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    ros_params = params['ros_params']
    rate_hz = float(params.get(
        'publish_rate_hz', ros_params.get('control_rate_hz', 50.0)))
    scale = float(ros_params['base_motion_time_scale'])
    amplitude = float(ros_params['base_disturbance_amplitude_m'])
    seed = int(ros_params['base_disturbance_seed'])
    profile = str(ros_params['base_disturbance_profile']).strip().lower()
    sample_count = int(math.floor((end_sec - start_sec) * rate_hz)) + 1
    times = start_sec + np.arange(sample_count, dtype=float) / rate_hz
    scaled_times = times / scale
    nominal = np.asarray([
        moving_traj.base_pose(time_value) for time_value in scaled_times
    ])
    disturbance = np.asarray([
        moving_traj.base_disturbance_sample(
            time_value, amplitude, seed, profile=profile).total_offset
        for time_value in scaled_times
    ])
    return times, nominal, nominal + disturbance, disturbance


def _welch_asd(
        values_mm: np.ndarray,
        rate_hz: float,
        window_sec: float = 32.0) -> tuple[np.ndarray, np.ndarray]:
    nperseg = min(
        len(values_mm),
        max(64, int(round(float(window_sec) * float(rate_hz)))),
    )
    frequencies, density = welch(
        values_mm,
        fs=float(rate_hz),
        window='hann',
        nperseg=nperseg,
        noverlap=nperseg // 2,
        detrend='constant',
        scaling='density',
    )
    return frequencies, np.sqrt(np.maximum(density, 0.0))


def plot_base_spectrum(
        path: Path,
        base_times: np.ndarray,
        nominal: np.ndarray,
        truth: np.ndarray,
        disturbance: np.ndarray,
        profile: str,
        run_label: str,
        dpi: int) -> dict[str, float]:
    rate_hz = 1.0 / float(np.median(np.diff(base_times)))
    fig, axes = plt.subplots(
        3, 1, figsize=(14.0, 9.2), sharex=True,
        gridspec_kw={'hspace': 0.10},
    )
    axis_names = ('x', 'y', 'z')
    base_frequency_hz = 1.0 / moving_traj.BASE_PERIOD_SEC
    gust_frequency_hz = (
        1.0 / moving_traj.BASE_DISTURBANCE_GUST_PERIOD_SEC)
    peak_frequency_by_axis = {}
    for index, axis in enumerate(axes):
        f_nominal, asd_nominal = _welch_asd(
            1000.0 * nominal[:, index], rate_hz)
        f_truth, asd_truth = _welch_asd(
            1000.0 * truth[:, index], rate_hz)
        f_disturbance, asd_disturbance = _welch_asd(
            1000.0 * disturbance[:, index], rate_hz)
        frequency_mask = (
            (f_disturbance >= 0.05) & (f_disturbance <= 5.0))
        peak_index = int(np.argmax(asd_disturbance[frequency_mask]))
        peak_frequency_by_axis[axis_names[index]] = float(
            f_disturbance[frequency_mask][peak_index])

        floor = 1e-5
        nominal_visible = np.where(
            asd_nominal >= 5e-5, asd_nominal, np.nan)
        axis.semilogy(
            f_nominal, nominal_visible,
            color=EXPECTED_COLOR, linestyle='--', linewidth=1.55,
            label='名义基座（期望）', zorder=6,
        )
        axis.semilogy(
            f_truth, np.maximum(asd_truth, floor),
            color=ACTUAL_COLOR, linewidth=1.55,
            label='受扰物理基座（实际）', zorder=4,
        )
        axis.semilogy(
            f_disturbance, np.maximum(asd_disturbance, floor),
            color=DISTURBANCE_COLOR, linestyle=(0, (3, 2)),
            linewidth=1.05, alpha=0.95, zorder=5,
            label='扰动分量 Δp（实际 − 名义）',
        )
        axis.axvspan(
            moving_traj.BASE_DISTURBANCE_FMIN_HZ,
            moving_traj.BASE_DISTURBANCE_FMAX_HZ,
            color=DISTURBANCE_COLOR, alpha=0.055, linewidth=0.0,
        )
        axis.axvline(
            base_frequency_hz, color='#111827', linestyle='--',
            linewidth=0.9, alpha=0.75,
        )
        axis.axvline(
            gust_frequency_hz, color=SECOND_GUST_COLOR, linestyle=':',
            linewidth=1.0, alpha=0.85,
        )
        axis.set_ylabel(
            f'{axis_names[index]} 轴幅值谱密度\n'
            f'（mm/√Hz）')
        axis.grid(
            True, which='both', color='#d5d9df',
            linewidth=0.65, alpha=0.72,
        )
        axis.spines['top'].set_visible(False)
        axis.spines['right'].set_visible(False)
    axes[0].legend(loc='upper right', framealpha=0.95)
    axes[0].text(
        base_frequency_hz + 0.05, axes[0].get_ylim()[1] / 1.7,
        '名义基座主频 2 Hz',
        fontsize=8.5, color='#111827', ha='left', va='top',
    )
    axes[0].text(
        gust_frequency_hz + 0.05, axes[0].get_ylim()[1] / 4.0,
        '阵风重复频率 0.125 Hz',
        fontsize=8.5, color=SECOND_GUST_COLOR, ha='left', va='top',
    )
    axes[-1].set_xlabel('频率（Hz）')
    axes[-1].set_xlim(0.05, 5.0)
    fig.suptitle(
        f'基座运动频谱与扰动干扰（{_profile_label(profile)}）\n'
        f'{run_label}',
        fontsize=13, fontweight='bold', y=0.985,
    )
    fig.text(
        0.5, 0.018,
        '物理真值由本次实验参数快照重建；'
        '控制器的模型信念列不会被误用为实际基座运动。',
        ha='center', fontsize=9, color='#4b5563',
    )
    fig.subplots_adjust(top=0.89, bottom=0.10, left=0.09, right=0.98)
    fig.savefig(path, dpi=dpi, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return {
        'sample_rate_hz': rate_hz,
        'nominal_base_frequency_hz': base_frequency_hz,
        'gust_repetition_frequency_hz': gust_frequency_hz,
        **{
            f'disturbance_peak_frequency_{axis}_hz': frequency
            for axis, frequency in peak_frequency_by_axis.items()
        },
    }


def _profile_label(profile: str) -> str:
    return {
        'harmonic': '谐波',
        'gust': '阵风',
        'fault': '偏置故障',
        'wideband': '宽带扰动',
    }.get(str(profile).strip().lower(), str(profile))


def _shape_label(shape: str) -> str:
    return {
        'whole': '全任务',
        'circle': '圆形',
        'triangle': '三角形',
        'square': '正方形',
        'sine': '正弦曲线',
        'windylab': 'windylab 字样',
    }.get(str(shape).strip().lower(), str(shape))


def _belief_label(mode: str) -> str:
    return {
        'nominal': '名义信念',
        'truth': '真值信念',
        'estimated': '估计信念',
        'model': '模型信念',
    }.get(str(mode).strip().lower(), str(mode))


def _run_label(params: dict, trajectory_stamp: str) -> str:
    ros_params = params['ros_params']
    date_text = (
        f'{trajectory_stamp[:4]}-{trajectory_stamp[4:6]}-'
        f'{trajectory_stamp[6:8]} {trajectory_stamp[9:11]}:'
        f'{trajectory_stamp[11:13]}:{trajectory_stamp[13:15]}')
    return (
        f'最新实验 {date_text} | '
        f"{_shape_label(ros_params['trajectory_shape'])} / "
        f"{_profile_label(ros_params['base_disturbance_profile'])} / "
        f"{_belief_label(ros_params['base_belief_mode'])}")


def generate_report(
        run_files: RunFiles,
        output_dir: Path,
        dpi: int = 180,
        three_d_smoothing_window: int = 7) -> dict:
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    params = json.loads(run_files.params_json.read_text(encoding='utf-8'))
    _validate_snapshot_against_model(params)
    trajectory = load_trajectory(run_files.trajectory_csv)
    times = trajectory['time_sec']
    expected = trajectory['expected']
    actual = trajectory['actual']
    error = trajectory['error']
    expected_segments = load_expected_path_segments(
        run_files.expected_path_csv)
    if not expected_segments:
        expected_segments = trajectory_segments(
            expected, trajectory['pen_down'], trajectory['stroke_id'])
    actual_segments_raw = trajectory_segments(
        actual, trajectory['pen_down'], trajectory['stroke_id'])
    actual_segments_display = smooth_segments(
        actual_segments_raw, three_d_smoothing_window)

    ros_params = params['ros_params']
    profile = str(ros_params['base_disturbance_profile']).strip().lower()
    events = disturbance_events(
        profile,
        float(times[0]),
        float(times[-1]),
        float(ros_params['base_motion_time_scale']),
    )
    run_label = _run_label(params, run_files.trajectory_stamp)
    paths = {
        'trajectory_3d': output_dir / '01_trajectory_3d.png',
        'axis_tracking': output_dir / (
            '02_axis_tracking_with_disturbance.png'),
        'tracking_error': output_dir / '03_tracking_error.png',
        'base_spectrum': output_dir / '04_base_spectrum.png',
    }
    plot_trajectory_3d(
        paths['trajectory_3d'],
        expected_segments,
        actual_segments_display,
        run_label,
        dpi,
    )
    plot_axis_tracking(
        paths['axis_tracking'],
        times,
        expected,
        actual,
        events,
        run_label,
        dpi,
    )
    error_metrics = plot_tracking_error(
        paths['tracking_error'],
        times,
        error,
        events,
        run_label,
        dpi,
    )
    base_times, base_nominal, base_truth, base_disturbance = (
        reconstruct_base_truth(
            float(times[0]), float(times[-1]), params))
    spectrum_metrics = plot_base_spectrum(
        paths['base_spectrum'],
        base_times,
        base_nominal,
        base_truth,
        base_disturbance,
        profile,
        run_label,
        dpi,
    )

    second_gust = next(
        (event for event in events
         if event.label == 'gust' and event.number == 2),
        None,
    )
    metadata = {
        'run_files': {
            key: str(value) if value is not None else None
            for key, value in asdict(run_files).items()
        },
        'profile': profile,
        'belief_mode': ros_params['base_belief_mode'],
        'trajectory_shape': ros_params['trajectory_shape'],
        'disturbance_amplitude_m': float(
            ros_params['base_disturbance_amplitude_m']),
        'trajectory_time_range_sec': [
            float(times[0]), float(times[-1])],
        'path_time_range_sec': [
            float(trajectory['path_time_sec'][0]),
            float(trajectory['path_time_sec'][-1]),
        ],
        'sample_count': int(len(times)),
        'stroke_count': int(len(actual_segments_raw)),
        'three_d_actual_display_smoothing_window': int(
            three_d_smoothing_window),
        'error_metrics': error_metrics,
        'second_global_gust': (
            asdict(second_gust) if second_gust is not None else None),
        'visible_events': [asdict(event) for event in events],
        'spectrum': spectrum_metrics,
        'figures': {key: str(value) for key, value in paths.items()},
        'command_fk_loaded': False,
    }
    metadata_path = output_dir / 'plot_metadata.json'
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    metadata['metadata_json'] = str(metadata_path)
    return metadata


def _parser() -> argparse.ArgumentParser:
    ros_home = Path(os.environ.get(
        'ROS_HOME', str(Path.home() / '.ros')))
    default_data_dir = ros_home / 'moving_base_uncertainty'
    parser = argparse.ArgumentParser(
        description=(
            'Plot four expected-vs-actual moving-base uncertainty figures.'))
    parser.add_argument(
        '--data-dir', type=Path, default=default_data_dir,
        help='Directory containing trajectory/loop/parameter run files.')
    parser.add_argument(
        '--trajectory-csv', type=Path,
        help='Main trajectory CSV; defaults to the newest complete main CSV.')
    parser.add_argument(
        '--loop-csv', type=Path,
        help='Controller loop CSV; defaults to the matching earlier stamp.')
    parser.add_argument(
        '--params-json', type=Path,
        help='Controller parameter snapshot; defaults to the matching stamp.')
    parser.add_argument(
        '--output-dir', type=Path,
        help='Output directory; defaults to DATA_DIR/figures_<trajectory stamp>.')
    parser.add_argument('--dpi', type=int, default=180)
    parser.add_argument(
        '--three-d-smoothing-window', type=int, default=7,
        help=(
            'Odd display-only smoothing window for the 3D actual line. '
            'Figures 2/3 and all metrics always use raw data.'))
    return parser


def main() -> int:
    args = _parser().parse_args()
    run_files = discover_run(
        args.data_dir,
        trajectory_csv=args.trajectory_csv,
        loop_csv=args.loop_csv,
        params_json=args.params_json,
    )
    output_dir = (
        args.output_dir
        if args.output_dir is not None
        else Path(args.data_dir) / f'figures_{run_files.trajectory_stamp}'
    )
    metadata = generate_report(
        run_files,
        output_dir,
        dpi=args.dpi,
        three_d_smoothing_window=args.three_d_smoothing_window,
    )
    print(json.dumps({
        'figures': metadata['figures'],
        'metadata_json': metadata['metadata_json'],
        'error_metrics': metadata['error_metrics'],
        'second_global_gust': metadata['second_global_gust'],
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
