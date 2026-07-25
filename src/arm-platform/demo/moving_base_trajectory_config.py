#!/usr/bin/env python3
"""Shared moving-base trajectory configuration.

The IK demo, MPC demo, and visualizer import this file so base/world transforms
stay consistent. Edit this file when switching between fixed-base and
moving-base experiments.
"""

import math
import os
from typing import NamedTuple

import numpy as np
import pinocchio as pin


WORLD_FRAME = 'world'
BASE_FRAME = 'base_link'

# Moving-base experiment: small base motion around the world-frame origin.
BASE_CIRCLE_CENTER = np.array([0.0, 0.0, 0.0])
BASE_CIRCLE_RADIUS = float(os.environ.get('MB_BASE_RADIUS', 0.01))  # 水平运动半径(可用环境变量覆盖,供扫描脚本用)
BASE_PERIOD_SEC = float(os.environ.get('MB_BASE_PERIOD', 0.5))  # 基座平移和姿态周期,改小频率增大
BASE_HEIGHT = 0.01 # 高度中心
BASE_Z_AMPLITUDE = float(os.environ.get('MB_BASE_Z_AMP', 0.01))  # z 轴浮动幅值(m);0=关闭
BASE_Z_PHASE = math.pi / 3.0   # z 相对水平圆的相位偏移(与 x/y 去相关)
# 基座运动形状: 'circle_z' = 水平圆+z正弦(历史默认);
# 'sphere' = 倾斜大圆,|p-中心|恒等于 BASE_SPHERE_RADIUS(球面最坏情况:速度/加速度持续顶满)
BASE_MOTION_SHAPE = os.environ.get('MB_BASE_SHAPE', 'circle_z')
BASE_SPHERE_RADIUS = float(os.environ.get('MB_BASE_SPHERE_R', 0.02))  # 球半径(m,sphere 模式)
# 倾斜大圆的正交基(单位向量): u=x̂, v=(ŷ+ẑ)/√2 -> 圆平面与水平面成45°,三轴都有持续分量
_SPHERE_U = np.array([1.0, 0.0, 0.0])
_SPHERE_V = np.array([0.0, 1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0)])
BASE_ATTITUDE_AMPLITUDE_DEG = 0.1  # deg

BASE_ROLL_AMPLITUDE = math.radians(BASE_ATTITUDE_AMPLITUDE_DEG)
BASE_PITCH_AMPLITUDE = math.radians(BASE_ATTITUDE_AMPLITUDE_DEG)
BASE_YAW_AMPLITUDE = math.radians(BASE_ATTITUDE_AMPLITUDE_DEG)

EE_WORLD_CIRCLE_CENTER = np.array([0.3, 0.01, 0.1])
EE_CIRCLE_RADIUS = 0.08
EE_PERIOD_SEC = 4.0

# World-frame drawing trajectory.  ``circle`` remains the compatibility
# default; the MPC node also exposes this as the ROS parameter
# ``trajectory_shape``.
SUPPORTED_EE_TRAJECTORIES = (
    'circle', 'triangle', 'square', 'sine', 'windylab', 'whole')
EE_TRAJECTORY_SHAPE = os.environ.get('MB_EE_SHAPE', 'circle').strip().lower()
EE_DRAW_PLANE_Z = float(os.environ.get('MB_EE_DRAW_Z', EE_WORLD_CIRCLE_CENTER[2]))
EE_PEN_LIFT_HEIGHT = float(os.environ.get('MB_EE_PEN_LIFT', 0.03))
EE_DRAW_SPEED = float(os.environ.get('MB_EE_DRAW_SPEED', 0.04))
EE_TRAVEL_SPEED = float(os.environ.get('MB_EE_TRAVEL_SPEED', 0.08))
EE_WINDYLAB_LETTER_WIDTH = 0.016
# 原字间距 6 mm，再增加 10 mm，避免相邻字母视觉上挤在一起。
EE_WINDYLAB_LETTER_GAP = 0.016
EE_WINDYLAB_LETTER_HEIGHT = 0.050
EE_WINDYLAB_MIRROR_X = True

_TRIANGLE_PERIOD_SEC = 6.0
_SQUARE_PERIOD_SEC = 8.0
_SINE_DRAW_SEC = 5.0
_SINE_LIFT_SEC = 0.5
_SINE_TRANSFER_SEC = 2.0
_SINE_LOWER_SEC = 0.5


class EeTrajectorySample(NamedTuple):
    """One world-frame trajectory sample with explicit pen semantics."""

    position: np.ndarray
    pen_down: bool
    stroke_id: int
    rotation: np.ndarray = None

TARGET_YAW = math.radians(90.0)
TARGET_PITCH = math.radians(90.0)
TARGET_ROLL = 0.0
TARGET_ROTATION = pin.rpy.rpyToMatrix(TARGET_ROLL, TARGET_PITCH, TARGET_YAW)

# Standalone tasks keep the historical drawing plane at
# ``EE_WORLD_CIRCLE_CENTER``.  The canonical whole-task order requested by the
# operator maps those same local primitives onto five different work surfaces:
#   front circle -> left triangle -> back windylab -> right sine -> top square
_SIDE_STATION_RADIUS = 0.30
_SIDE_STATION_HEIGHT = 0.12
# The top board is above and slightly in front of the arm rather than directly
# over joint1.  This keeps the complete 12 cm square inside the validated
# neutral-seed IK basin while the tool points down.
_TOP_STATION_FORWARD = 0.22
_TOP_STATION_HEIGHT = 0.30
_WHOLE_TASKS = (
    ('circle', 'front'),
    ('triangle', 'left'),
    ('windylab', 'back'),
    ('sine', 'right'),
    ('square', 'top'),
)
_STATION_AZIMUTH = {
    'front': 0.0,
    'left': 0.5 * math.pi,
    'back': math.pi,
    'right': 1.5 * math.pi,
    'top': 2.0 * math.pi,
}
_WHOLE_LIFT_SEC = 0.6
_WHOLE_TRANSFER_SEC = 2.4
_WHOLE_LOWER_SEC = 0.6
_WHOLE_TRANSITION_SEC = (
    _WHOLE_LIFT_SEC + _WHOLE_TRANSFER_SEC + _WHOLE_LOWER_SEC)


def _validate_ee_shape(shape: str = None) -> str:
    value = EE_TRAJECTORY_SHAPE if shape is None else str(shape)
    value = value.strip().lower()
    if value not in SUPPORTED_EE_TRAJECTORIES:
        raise ValueError(
            f"unsupported EE trajectory '{value}'; expected one of "
            f"{SUPPORTED_EE_TRAJECTORIES}")
    return value


def _smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def _line_sample(start: np.ndarray, end: np.ndarray, phase: float) -> np.ndarray:
    return np.asarray(start, dtype=float) + _smoothstep(phase) * (
        np.asarray(end, dtype=float) - np.asarray(start, dtype=float))


def _closed_polyline_sample(
        t: float, vertices: np.ndarray, period: float) -> EeTrajectorySample:
    """Time-parameterize a closed polygon, stopping smoothly at each corner."""
    cycle = math.floor(float(t) / period)
    local = float(t) - cycle * period
    edge_count = len(vertices) - 1
    edge_phase = local / period * edge_count
    edge_index = min(int(edge_phase), edge_count - 1)
    position = _line_sample(
        vertices[edge_index],
        vertices[edge_index + 1],
        edge_phase - edge_index,
    )
    return EeTrajectorySample(position, True, int(cycle))


def _letter_strokes():
    """Return separated lowercase-like vector strokes spelling ``windylab``.

    Coordinates are normalized per character.  Separate arrays are separate
    physical strokes, so ``i`` has a detached dot and ``y`` has a detached
    descender.  Every stroke and every letter is separated by an actual
    lift/travel/lower motion in :func:`_build_windylab_timeline`.
    """
    glyphs = {
        'w': [
            [(0.00, 0.72), (0.20, 0.00), (0.50, 0.48),
             (0.80, 0.00), (1.00, 0.72)],
        ],
        'i': [
            [(0.50, 0.00), (0.50, 0.70)],
            [(0.50, 0.88), (0.50, 1.00)],
        ],
        'n': [
            [(0.00, 0.00), (0.00, 0.70), (0.45, 0.70),
             (0.82, 0.55), (1.00, 0.00)],
        ],
        'd': [
            [(1.00, 0.00), (0.28, 0.00), (0.00, 0.20),
             (0.00, 0.52), (0.28, 0.70), (1.00, 0.70),
             (1.00, 0.00), (1.00, 1.00)],
        ],
        'y': [
            [(0.00, 0.70), (0.50, 0.20), (1.00, 0.70)],
            [(0.50, 0.20), (0.50, -0.25)],
        ],
        'l': [
            [(0.50, 1.00), (0.50, 0.00)],
        ],
        'a': [
            [(1.00, 0.00), (0.28, 0.00), (0.00, 0.20),
             (0.00, 0.52), (0.28, 0.70), (1.00, 0.70),
             (1.00, 0.00)],
        ],
        'b': [
            [(0.00, 1.00), (0.00, 0.00), (0.68, 0.00),
             (1.00, 0.20), (1.00, 0.50), (0.68, 0.70),
             (0.00, 0.70)],
        ],
    }
    word = 'windylab'
    letter_width = EE_WINDYLAB_LETTER_WIDTH
    letter_gap = EE_WINDYLAB_LETTER_GAP
    letter_height = EE_WINDYLAB_LETTER_HEIGHT
    total_width = len(word) * letter_width + (len(word) - 1) * letter_gap
    x_origin = EE_WORLD_CIRCLE_CENTER[0] - 0.5 * total_width
    baseline_y = EE_WORLD_CIRCLE_CENTER[1] - 0.022

    strokes = []
    for letter_index, letter in enumerate(word):
        letter_x = x_origin + letter_index * (letter_width + letter_gap)
        for points in glyphs[letter]:
            stroke = np.array([
                [
                    letter_x + letter_width * u,
                    baseline_y + letter_height * v,
                    EE_DRAW_PLANE_Z,
                ]
                for u, v in points
            ], dtype=float)
            if EE_WINDYLAB_MIRROR_X:
                # Mirror the complete word around its horizontal center.  This
                # intentionally mirrors both glyph geometry and letter order.
                stroke[:, 0] = (
                    2.0 * EE_WORLD_CIRCLE_CENTER[0] - stroke[:, 0])
            strokes.append(stroke)
    return strokes


def _motion_duration(start: np.ndarray, end: np.ndarray, speed: float) -> float:
    distance = float(np.linalg.norm(np.asarray(end) - np.asarray(start)))
    return max(distance / max(float(speed), 1e-6), 0.12)


def _build_windylab_timeline():
    strokes = _letter_strokes()
    segments = []

    def add(start, end, speed, pen_down, stroke_id):
        segments.append({
            'start': np.asarray(start, dtype=float),
            'end': np.asarray(end, dtype=float),
            'duration': _motion_duration(start, end, speed),
            'pen_down': bool(pen_down),
            'stroke_id': int(stroke_id),
        })

    for stroke_id, stroke in enumerate(strokes):
        for index in range(len(stroke) - 1):
            add(
                stroke[index], stroke[index + 1],
                EE_DRAW_SPEED, True, stroke_id)

        next_stroke = strokes[(stroke_id + 1) % len(strokes)]
        current = stroke[-1]
        next_start = next_stroke[0]
        lifted_current = current + np.array([0.0, 0.0, EE_PEN_LIFT_HEIGHT])
        lifted_next = next_start + np.array([0.0, 0.0, EE_PEN_LIFT_HEIGHT])
        # These three segments are always pen-up.  This includes the transition
        # between the final "b" and the next cycle's "w".
        add(current, lifted_current, EE_TRAVEL_SPEED, False, -1)
        add(lifted_current, lifted_next, EE_TRAVEL_SPEED, False, -1)
        add(lifted_next, next_start, EE_TRAVEL_SPEED, False, -1)

    cumulative = np.cumsum([0.0] + [s['duration'] for s in segments])
    return strokes, segments, cumulative


_WINDYLAB_STROKES, _WINDYLAB_SEGMENTS, _WINDYLAB_CUMULATIVE = (
    _build_windylab_timeline())
_WINDYLAB_PERIOD_SEC = float(_WINDYLAB_CUMULATIVE[-1])


def _local_trajectory_period(shape: str) -> float:
    if shape == 'whole':
        raise ValueError('whole has no single local primitive period')
    shape = _validate_ee_shape(shape)
    return {
        'circle': EE_PERIOD_SEC,
        'triangle': _TRIANGLE_PERIOD_SEC,
        'square': _SQUARE_PERIOD_SEC,
        'sine': (
            _SINE_DRAW_SEC + _SINE_LIFT_SEC
            + _SINE_TRANSFER_SEC + _SINE_LOWER_SEC
        ),
        'windylab': _WINDYLAB_PERIOD_SEC,
    }[shape]


def ee_trajectory_period(shape: str = None) -> float:
    shape = _validate_ee_shape(shape)
    if shape != 'whole':
        return _local_trajectory_period(shape)
    return (
        sum(_local_trajectory_period(task_shape)
            for task_shape, _ in _WHOLE_TASKS)
        + len(_WHOLE_TASKS) * _WHOLE_TRANSITION_SEC
    )


def _windylab_sample(t: float) -> EeTrajectorySample:
    period = _WINDYLAB_PERIOD_SEC
    cycle = math.floor(float(t) / period)
    local = float(t) - cycle * period
    index = int(np.searchsorted(_WINDYLAB_CUMULATIVE, local, side='right') - 1)
    index = int(np.clip(index, 0, len(_WINDYLAB_SEGMENTS) - 1))
    segment = _WINDYLAB_SEGMENTS[index]
    phase = (
        (local - _WINDYLAB_CUMULATIVE[index])
        / segment['duration']
    )
    stroke_id = segment['stroke_id']
    if stroke_id >= 0:
        stroke_id += int(cycle) * len(_WINDYLAB_STROKES)
    return EeTrajectorySample(
        _line_sample(segment['start'], segment['end'], phase),
        segment['pen_down'],
        stroke_id,
    )


def _local_trajectory_sample(t: float, shape: str) -> EeTrajectorySample:
    """Sample a primitive in the canonical local drawing plane."""
    if shape == 'whole':
        raise ValueError('whole is not a local drawing primitive')
    t = float(t)
    center = EE_WORLD_CIRCLE_CENTER

    if shape == 'circle':
        period = EE_PERIOD_SEC
        cycle = math.floor(t / period)
        angle = 2.0 * math.pi * (t - cycle * period) / period
        position = center + EE_CIRCLE_RADIUS * np.array([
            math.cos(angle), math.sin(angle), 0.0])
        return EeTrajectorySample(position, True, int(cycle))

    if shape == 'triangle':
        vertices = center + np.array([
            [-0.070, -0.050, 0.0],
            [0.000, 0.070, 0.0],
            [0.070, -0.050, 0.0],
            [-0.070, -0.050, 0.0],
        ])
        return _closed_polyline_sample(t, vertices, _TRIANGLE_PERIOD_SEC)

    if shape == 'square':
        vertices = center + np.array([
            [-0.060, -0.060, 0.0],
            [-0.060, 0.060, 0.0],
            [0.060, 0.060, 0.0],
            [0.060, -0.060, 0.0],
            [-0.060, -0.060, 0.0],
        ])
        return _closed_polyline_sample(t, vertices, _SQUARE_PERIOD_SEC)

    if shape == 'sine':
        period = _local_trajectory_period('sine')
        cycle = math.floor(t / period)
        local = t - cycle * period
        start = center + np.array([-0.080, 0.0, 0.0])
        end = center + np.array([0.080, 0.0, 0.0])
        lift = np.array([0.0, 0.0, EE_PEN_LIFT_HEIGHT])
        if local < _SINE_DRAW_SEC:
            progress = _smoothstep(local / _SINE_DRAW_SEC)
            position = np.array([
                start[0] + (end[0] - start[0]) * progress,
                center[1] + 0.050 * math.sin(2.0 * math.pi * progress),
                EE_DRAW_PLANE_Z,
            ])
            return EeTrajectorySample(position, True, int(cycle))
        local -= _SINE_DRAW_SEC
        if local < _SINE_LIFT_SEC:
            return EeTrajectorySample(
                _line_sample(end, end + lift, local / _SINE_LIFT_SEC),
                False, -1)
        local -= _SINE_LIFT_SEC
        if local < _SINE_TRANSFER_SEC:
            return EeTrajectorySample(
                _line_sample(
                    end + lift, start + lift,
                    local / _SINE_TRANSFER_SEC),
                False, -1)
        local -= _SINE_TRANSFER_SEC
        return EeTrajectorySample(
            _line_sample(
                start + lift, start,
                local / _SINE_LOWER_SEC),
            False, -1)

    return _windylab_sample(t)


def _rz(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _station_geometry(station: str):
    """Return center, drawing u/v axes, lift direction, and tool rotation."""
    if station == 'top':
        center = np.array([
            _TOP_STATION_FORWARD, 0.0, _TOP_STATION_HEIGHT])
        axis_u = np.array([1.0, 0.0, 0.0])
        axis_v = np.array([0.0, 1.0, 0.0])
        lift_direction = np.array([0.0, 0.0, 1.0])
        # Keep link7's y-axis normal to the top drawing plane.
        rotation = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
        ])
        return center, axis_u, axis_v, lift_direction, rotation

    azimuth = _STATION_AZIMUTH[station]
    radial = np.array([math.cos(azimuth), math.sin(azimuth), 0.0])
    # Horizontal drawing coordinate as seen by the robot facing the board.
    # This is the robot's right direction (negative local y); using local left
    # here would mirror asymmetric glyphs such as "windylab".
    axis_u = np.array([math.sin(azimuth), -math.cos(azimuth), 0.0])
    axis_v = np.array([0.0, 0.0, 1.0])
    center = (
        _SIDE_STATION_RADIUS * radial
        + np.array([0.0, 0.0, _SIDE_STATION_HEIGHT])
    )
    # Move toward the robot to lift away from a vertical drawing surface.
    lift_direction = -radial
    rotation = _rz(azimuth) @ TARGET_ROTATION
    return center, axis_u, axis_v, lift_direction, rotation


def _place_local_sample(
        sample: EeTrajectorySample, station: str) -> EeTrajectorySample:
    center, axis_u, axis_v, lift_direction, rotation = (
        _station_geometry(station))
    local_offset = np.asarray(sample.position) - EE_WORLD_CIRCLE_CENTER
    position = (
        center
        + local_offset[0] * axis_u
        + local_offset[1] * axis_v
        + local_offset[2] * lift_direction
    )
    return EeTrajectorySample(
        position,
        sample.pen_down,
        sample.stroke_id,
        rotation.copy(),
    )


def _interpolate_rotation(
        rotation_start: np.ndarray, rotation_end: np.ndarray,
        phase: float) -> np.ndarray:
    phase = _smoothstep(phase)
    delta = pin.log3(rotation_start.T @ rotation_end)
    return rotation_start @ pin.exp3(phase * delta)


def _whole_trajectory_sample(t: float) -> EeTrajectorySample:
    period = ee_trajectory_period('whole')
    cycle = math.floor(float(t) / period)
    local_time = float(t) - cycle * period
    task_count = len(_WHOLE_TASKS)

    for task_index, (task_shape, station) in enumerate(_WHOLE_TASKS):
        draw_period = _local_trajectory_period(task_shape)
        if local_time < draw_period:
            placed = _place_local_sample(
                _local_trajectory_sample(local_time, task_shape),
                station,
            )
            stroke_id = placed.stroke_id
            if stroke_id >= 0:
                stroke_id += int(cycle) * 10000 + task_index * 100
            return EeTrajectorySample(
                placed.position, placed.pen_down, stroke_id, placed.rotation)
        local_time -= draw_period
        if local_time >= _WHOLE_TRANSITION_SEC:
            local_time -= _WHOLE_TRANSITION_SEC
            continue

        next_shape, next_station = _WHOLE_TASKS[(task_index + 1) % task_count]
        current_end = _place_local_sample(
            _local_trajectory_sample(draw_period - 1e-9, task_shape),
            station,
        )
        next_start = _place_local_sample(
            _local_trajectory_sample(0.0, next_shape),
            next_station,
        )
        current_lift = _station_geometry(station)[3]
        next_lift = _station_geometry(next_station)[3]
        lifted_current = (
            current_end.position + EE_PEN_LIFT_HEIGHT * current_lift)
        lifted_next = (
            next_start.position + EE_PEN_LIFT_HEIGHT * next_lift)

        if local_time < _WHOLE_LIFT_SEC:
            phase = local_time / _WHOLE_LIFT_SEC
            position = _line_sample(
                current_end.position, lifted_current, phase)
            rotation = current_end.rotation
        elif local_time < _WHOLE_LIFT_SEC + _WHOLE_TRANSFER_SEC:
            phase = (
                (local_time - _WHOLE_LIFT_SEC) / _WHOLE_TRANSFER_SEC)
            position = _line_sample(lifted_current, lifted_next, phase)
            rotation = _interpolate_rotation(
                current_end.rotation, next_start.rotation, phase)
        else:
            phase = (
                local_time - _WHOLE_LIFT_SEC - _WHOLE_TRANSFER_SEC
            ) / _WHOLE_LOWER_SEC
            position = _line_sample(lifted_next, next_start.position, phase)
            rotation = next_start.rotation
        return EeTrajectorySample(position, False, -1, rotation.copy())

    # Floating-point fall-through at the exact period boundary.
    return _whole_trajectory_sample(0.0)


def base_nominal_yaw(t: float, shape: str = None) -> float:
    """Nominal base heading that keeps the active side station in front.

    Small floating-base attitude disturbances are composed on top of this
    heading in :func:`base_rotation`.
    """
    shape = _validate_ee_shape(shape)
    if shape != 'whole':
        # Preserve the original moving-base demo for every standalone shape:
        # the base only has its small configured floating disturbance.
        return 0.0

    period = ee_trajectory_period('whole')
    cycle = math.floor(float(t) / period)
    local_time = float(t) - cycle * period
    for task_index, (task_shape, station) in enumerate(_WHOLE_TASKS):
        draw_period = _local_trajectory_period(task_shape)
        current_yaw = _STATION_AZIMUTH[station]
        if local_time < draw_period:
            return current_yaw
        local_time -= draw_period

        next_station = _WHOLE_TASKS[
            (task_index + 1) % len(_WHOLE_TASKS)][1]
        next_yaw = _STATION_AZIMUTH[next_station]
        if task_index == len(_WHOLE_TASKS) - 1:
            next_yaw = 2.0 * math.pi
        if local_time < _WHOLE_TRANSITION_SEC:
            phase = _smoothstep(local_time / _WHOLE_TRANSITION_SEC)
            return current_yaw + phase * (next_yaw - current_yaw)
        local_time -= _WHOLE_TRANSITION_SEC
    return 2.0 * math.pi


def ee_trajectory_sample(t: float, shape: str = None) -> EeTrajectorySample:
    """Return world position, world rotation, and explicit pen state."""
    shape = _validate_ee_shape(shape)
    if shape == 'whole':
        return _whole_trajectory_sample(t)
    local = _local_trajectory_sample(t, shape)
    return EeTrajectorySample(
        local.position.copy(),
        local.pen_down,
        local.stroke_id,
        TARGET_ROTATION.copy(),
    )


def ee_pen_down(t: float, shape: str = None) -> bool:
    return bool(ee_trajectory_sample(t, shape).pen_down)


def ee_stroke_id(t: float, shape: str = None) -> int:
    return int(ee_trajectory_sample(t, shape).stroke_id)


def sample_ee_drawing_segments(
        shape: str = None, sample_count: int = 1000):
    """Sample only visible pen-down strokes, split for RViz/Matplotlib.

    Pen-up travel is intentionally absent and stroke changes always start a new
    array, so consumers cannot accidentally draw connections between letters.
    """
    shape = _validate_ee_shape(shape)
    sample_count = max(int(sample_count), 2)
    if shape == 'windylab':
        sample_count = max(sample_count, 1000)
    elif shape == 'whole':
        # The whole cycle is much longer and includes short strokes such as the
        # dot over "i".  A dense minimum prevents RViz's static preview from
        # silently omitting them when expected_path_points keeps its old value.
        sample_count = max(sample_count, 3000)
    period = ee_trajectory_period(shape)
    times = np.linspace(0.0, period, sample_count, endpoint=False)
    segments = []
    current = []
    current_stroke = None
    for time_value in times:
        sample = ee_trajectory_sample(float(time_value), shape)
        if not sample.pen_down:
            if len(current) >= 2:
                segments.append(np.asarray(current))
            current = []
            current_stroke = None
            continue
        if current_stroke is None or sample.stroke_id != current_stroke:
            if len(current) >= 2:
                segments.append(np.asarray(current))
            current = [sample.position]
            current_stroke = sample.stroke_id
        else:
            current.append(sample.position)
    if len(current) >= 2:
        segments.append(np.asarray(current))

    if shape in ('circle', 'triangle', 'square') and segments:
        segments[0] = np.vstack([segments[0], segments[0][0]])
    return segments


# 基座运动形状
def base_pose(t: float) -> np.ndarray:
    """Return base_link origin position in world frame."""
    w = 2.0 * math.pi / BASE_PERIOD_SEC
    angle = w * t
    if BASE_MOTION_SHAPE == 'sphere':
        center = np.array([BASE_CIRCLE_CENTER[0], BASE_CIRCLE_CENTER[1], BASE_HEIGHT])
        return center + BASE_SPHERE_RADIUS * (
            math.cos(angle) * _SPHERE_U + math.sin(angle) * _SPHERE_V)
    return np.array([
        BASE_CIRCLE_CENTER[0] + BASE_CIRCLE_RADIUS * math.cos(angle),
        BASE_CIRCLE_CENTER[1] + BASE_CIRCLE_RADIUS * math.sin(angle),
        BASE_HEIGHT + BASE_Z_AMPLITUDE * math.sin(angle + BASE_Z_PHASE),
    ])


def base_rotation(
        t: float, shape: str = None, path_t: float = None) -> np.ndarray:
    """Return base_link rotation matrix in world frame.

    ``t`` drives the floating jitter. ``path_t`` drives the task-station
    heading and may be time-scaled independently.
    """
    w = 2.0 * math.pi / BASE_PERIOD_SEC
    roll = BASE_ROLL_AMPLITUDE * math.sin(w * t)
    pitch = BASE_PITCH_AMPLITUDE * math.sin(w * t + 2.0 * math.pi / 3.0)
    yaw = BASE_YAW_AMPLITUDE * math.sin(w * t + 4.0 * math.pi / 3.0)
    disturbance = pin.rpy.rpyToMatrix(roll, pitch, yaw)
    if shape is None:
        return disturbance
    path_t = t if path_t is None else float(path_t)
    return _rz(base_nominal_yaw(path_t, shape)) @ disturbance


def ee_target_world(t: float, shape: str = None) -> np.ndarray:
    """Return desired end-effector position in world frame."""
    return ee_trajectory_sample(t, shape).position.copy()


def target_rotation_world(
        t: float = 0.0, shape: str = None) -> np.ndarray:
    """Return desired end-effector rotation matrix in world frame."""
    return ee_trajectory_sample(t, shape).rotation.copy()
