#!/usr/bin/env python3
"""Visualize desired and actual end-effector trajectories.

This node is intended to run alongside move_arm_ik_demo.py,
move_arm_line_demo.py, or move_arm_mpc_ik_demo.py. It publishes RViz markers
and, when available, opens a Matplotlib window with a 3D trajectory plot plus
x/y/z tracking errors.
"""

import math
import csv
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

import moving_base_trajectory_config as moving_traj
from pinocchio_ik import PinocchioIK


JOINT_COUNT = 7

CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
CIRCLE_RADIUS = 0.08
CIRCLE_PERIOD_SEC = 8.0

HORIZONTAL_CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
HORIZONTAL_CIRCLE_RADIUS = 0.08
HORIZONTAL_CIRCLE_PERIOD_SEC = 8.0

POINT_A = np.array([0.35, -0.15, 0.15])
POINT_B = np.array([0.35, 0.15, 0.25])
LINE_PERIOD_SEC = 6.0

def to_point(p: np.ndarray) -> Point:
    point = Point()
    point.x = float(p[0])
    point.y = float(p[1])
    point.z = float(p[2])
    return point


class MatplotlibPlotter:
    def __init__(self, logger, expected_path_segments):
        self._logger = logger
        self._expected_path_segments = [
            np.asarray(segment, dtype=float)
            for segment in expected_path_segments
        ]
        self._expected_path = np.vstack(self._expected_path_segments)
        self._times = deque(maxlen=2000)
        self._expected = deque(maxlen=2000)
        self._actual = deque(maxlen=2000)
        self._errors = deque(maxlen=2000)
        self._command_times = deque(maxlen=2000)
        self._command_expected = deque(maxlen=2000)
        self._command_fk = deque(maxlen=2000)
        self._command_errors = deque(maxlen=2000)
        self._expected_draw = deque(maxlen=2200)
        self._actual_draw = deque(maxlen=2200)
        self._command_draw = deque(maxlen=2200)
        self._draw_stroke = None
        self._command_draw_stroke = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    @staticmethod
    def _append_draw_point(
            draw_points, point: np.ndarray,
            pen_down: bool, stroke_id: int, previous_stroke):
        if not pen_down:
            if draw_points and np.all(np.isfinite(draw_points[-1])):
                draw_points.append(np.full(3, np.nan))
            return None
        if previous_stroke is not None and previous_stroke != int(stroke_id):
            draw_points.append(np.full(3, np.nan))
        draw_points.append(np.asarray(point, dtype=float).copy())
        return int(stroke_id)

    def add_sample(
            self, t: float, expected: np.ndarray, actual: np.ndarray,
            pen_down: bool, stroke_id: int):
        with self._lock:
            self._times.append(float(t))
            self._expected.append(expected.copy())
            self._actual.append(actual.copy())
            self._errors.append(actual - expected)
            previous = self._draw_stroke
            self._draw_stroke = self._append_draw_point(
                self._expected_draw, expected, pen_down, stroke_id, previous)
            self._append_draw_point(
                self._actual_draw, actual, pen_down, stroke_id, previous)

    def add_command_fk_sample(
            self, t: float, expected: np.ndarray, command_fk: np.ndarray,
            pen_down: bool, stroke_id: int):
        with self._lock:
            self._command_times.append(float(t))
            self._command_expected.append(expected.copy())
            self._command_fk.append(command_fk.copy())
            self._command_errors.append(command_fk - expected)
            self._command_draw_stroke = self._append_draw_point(
                self._command_draw,
                command_fk,
                pen_down,
                stroke_id,
                self._command_draw_stroke,
            )

    def reset(self):
        with self._lock:
            self._times.clear()
            self._expected.clear()
            self._actual.clear()
            self._errors.clear()
            self._command_times.clear()
            self._command_expected.clear()
            self._command_fk.clear()
            self._command_errors.clear()
            self._expected_draw.clear()
            self._actual_draw.clear()
            self._command_draw.clear()
            self._draw_stroke = None
            self._command_draw_stroke = None

    def _snapshot(self):
        with self._lock:
            times = np.array(self._times, dtype=float)
            expected = np.array(self._expected, dtype=float)
            actual = np.array(self._actual, dtype=float)
            errors = np.array(self._errors, dtype=float)
            command_times = np.array(self._command_times, dtype=float)
            command_expected = np.array(self._command_expected, dtype=float)
            command_fk = np.array(self._command_fk, dtype=float)
            command_errors = np.array(self._command_errors, dtype=float)
            expected_draw = np.array(self._expected_draw, dtype=float)
            actual_draw = np.array(self._actual_draw, dtype=float)
            command_draw = np.array(self._command_draw, dtype=float)
        return (
            times, expected, actual, errors,
            command_times, command_expected, command_fk, command_errors,
            expected_draw, actual_draw, command_draw,
        )

    def _run(self):
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover - depends on desktop env
            self._logger.warn(f'Matplotlib unavailable, RViz markers continue: {exc}')
            return

        try:
            plt.ion()
            fig = plt.figure(figsize=(12, 10))
            grid = fig.add_gridspec(3, 3, height_ratios=[2.0, 1.0, 1.0])
            ax_3d = fig.add_subplot(grid[0, :], projection='3d')
            ax_track = [fig.add_subplot(grid[1, i]) for i in range(3)]
            ax_err = [fig.add_subplot(grid[2, i]) for i in range(3)]
            fig.suptitle('End-effector trajectory tracking')

            for index, segment in enumerate(self._expected_path_segments):
                ax_3d.plot(
                    segment[:, 0],
                    segment[:, 1],
                    segment[:, 2],
                    color='tab:green',
                    linestyle='--',
                    label='expected path' if index == 0 else None)
            expected_line, = ax_3d.plot([], [], [], color='limegreen', label='expected')
            command_line, = ax_3d.plot([], [], [], color='tab:orange', label='command FK')
            actual_line, = ax_3d.plot([], [], [], color='tab:red', label='actual')
            ax_3d.set_xlabel('x (m)')
            ax_3d.set_ylabel('y (m)')
            ax_3d.set_zlabel('z (m)')
            ax_3d.legend(loc='upper right')
            self._set_equal_ish_3d(ax_3d, self._expected_path)

            track_lines = []
            error_lines = []
            axis_names = ['x', 'y', 'z']
            for axis, name in zip(ax_track, axis_names):
                expected_axis, = axis.plot([], [], color='tab:green', linestyle='--', label='expected')
                command_axis, = axis.plot([], [], color='tab:orange', label='command FK')
                actual_axis, = axis.plot([], [], color='tab:red', label='actual')
                axis.set_xlabel('time (s)')
                axis.set_ylabel(f'{name} position (m)')
                axis.grid(True)
                axis.legend(loc='upper right')
                track_lines.append((expected_axis, command_axis, actual_axis))

            for axis, name in zip(ax_err, axis_names):
                line, = axis.plot([], [], color='tab:blue')
                axis.axhline(0.0, color='0.4', linewidth=0.8)
                axis.set_xlabel('time (s)')
                axis.set_ylabel(f'{name} error (m)')
                axis.grid(True)
                error_lines.append(line)

            while not self._stop.is_set():
                (
                    times, expected, actual, errors,
                    command_times, command_expected, command_fk, command_errors,
                    expected_draw, actual_draw, command_draw,
                ) = self._snapshot()
                if actual.size:
                    if expected_draw.size:
                        expected_line.set_data(
                            expected_draw[:, 0], expected_draw[:, 1])
                        expected_line.set_3d_properties(expected_draw[:, 2])
                    else:
                        expected_line.set_data([], [])
                        expected_line.set_3d_properties([])
                    if actual_draw.size:
                        actual_line.set_data(
                            actual_draw[:, 0], actual_draw[:, 1])
                        actual_line.set_3d_properties(actual_draw[:, 2])
                    else:
                        actual_line.set_data([], [])
                        actual_line.set_3d_properties([])

                    all_points = np.vstack([self._expected_path, actual])
                    if command_draw.size:
                        command_line.set_data(
                            command_draw[:, 0], command_draw[:, 1])
                        command_line.set_3d_properties(command_draw[:, 2])
                        all_points = np.vstack([all_points, command_fk])
                    self._set_equal_ish_3d(ax_3d, all_points)

                    for i, (expected_axis, command_axis, actual_axis) in enumerate(track_lines):
                        expected_axis.set_data(times, expected[:, i])
                        if command_fk.size:
                            command_axis.set_data(command_times, command_fk[:, i])
                        actual_axis.set_data(times, actual[:, i])
                        axis = ax_track[i]
                        axis.relim()
                        axis.autoscale_view()

                    for i, line in enumerate(error_lines):
                        line.set_data(times, errors[:, i])
                        axis = ax_err[i]
                        axis.relim()
                        axis.autoscale_view()

                fig.canvas.draw_idle()
                plt.pause(0.05)

            plt.close(fig)
        except Exception as exc:  # pragma: no cover - depends on desktop env
            self._logger.warn(f'Matplotlib plotting stopped, RViz markers continue: {exc}')

    @staticmethod
    def _set_equal_ish_3d(axis, points: np.ndarray):
        points = np.asarray(points, dtype=float)
        points = points[np.all(np.isfinite(points), axis=1)]
        if not len(points):
            return
        center = points.mean(axis=0)
        span = np.ptp(points, axis=0)
        radius = max(float(span.max()) * 0.6, 0.05)
        axis.set_xlim(center[0] - radius, center[0] + radius)
        axis.set_ylim(center[1] - radius, center[1] + radius)
        axis.set_zlim(center[2] - radius, center[2] + radius)


class TrajectoryVisualizerDemo(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer_demo')

        self.trajectory_mode = self.declare_parameter('trajectory_mode', 'circle').value
        self.trajectory_shape = str(self.declare_parameter(
            'trajectory_shape', moving_traj.EE_TRAJECTORY_SHAPE).value
        ).strip().lower()
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.actual_topic = self.declare_parameter('actual_topic', '/joint_states').value
        self.command_topic = self.declare_parameter(
            'command_topic', '/student/joint_command').value
        self.control_signal_topic = self.declare_parameter(
            'control_signal_topic', self.command_topic).value
        self.record_control_signal = bool(
            self.declare_parameter('record_control_signal', False).value)
        self.record_command_fk = bool(
            self.declare_parameter('record_command_fk', True).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 10.0).value)
        # Keep at least one complete high-rate ``whole`` cycle visible in RViz.
        # The previous 2000-point deque discarded early red strokes mid-cycle.
        self.max_actual_points = int(
            self.declare_parameter('max_actual_points', 12000).value)
        self.expected_path_points = int(
            self.declare_parameter('expected_path_points', 240).value)
        self.show_matplotlib = bool(self.declare_parameter('show_matplotlib', True).value)
        self.start_on_command = bool(self.declare_parameter('start_on_command', True).value)
        self.use_message_stamp = bool(self.declare_parameter('use_message_stamp', True).value)
        self.reset_gap_sec = float(self.declare_parameter('reset_gap_sec', 2.0).value)
        self.stale_command_timeout_sec = float(
            self.declare_parameter('stale_command_timeout_sec', 0.2).value)
        self.sample_only_on_actual_change = bool(
            self.declare_parameter('sample_only_on_actual_change', True).value)
        self.actual_change_epsilon = float(
            self.declare_parameter('actual_change_epsilon', 1e-9).value)
        self.control_signal_change_epsilon = float(
            self.declare_parameter('control_signal_change_epsilon', 1e-12).value)
        self.ignore_initial_sec = float(
            self.declare_parameter('ignore_initial_sec', 1.0).value)
        self.trajectory_time_scale = float(
            self.declare_parameter('trajectory_time_scale', 1.0).value)
        self.base_motion_time_scale = float(
            self.declare_parameter('base_motion_time_scale', 1.0).value)
        # 基座扰动真值通道(不确定性实验): actual world EE 用 名义+扰动 计算。
        # 与 MPC demo 的 base_disturbance_* 参数必须一致(同幅值同种子),
        # MPC 端另有 base_belief_mode 决定它"知道"多少。0=关闭(现状)。
        self.base_disturbance_amplitude_m = float(
            self.declare_parameter('base_disturbance_amplitude_m', 0.0).value)
        self.base_disturbance_seed = int(
            self.declare_parameter('base_disturbance_seed', 0).value)
        self.data_dir = Path(str(self.declare_parameter(
            'data_dir',
            '/home/windylab/code/windylab_ws/src/arm-platform/demo/data').value))

        if self.trajectory_mode not in (
                'circle', 'horizontal_circle', 'line',
                'moving_base', 'moving_base_circle'):
            raise ValueError(
                "trajectory_mode must be 'circle', 'horizontal_circle', 'line' "
                "'moving_base' or 'moving_base_circle'")
        if self.trajectory_shape not in moving_traj.SUPPORTED_EE_TRAJECTORIES:
            raise ValueError(
                f"trajectory_shape must be one of "
                f"{moving_traj.SUPPORTED_EE_TRAJECTORIES}")
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        if self.ignore_initial_sec < 0.0:
            raise ValueError('ignore_initial_sec must be non-negative')
        if self.trajectory_time_scale <= 0.0 or self.base_motion_time_scale <= 0.0:
            raise ValueError('trajectory_time_scale/base_motion_time_scale must be > 0')
        if self._is_moving_base_mode() and self.frame_id == 'base_link':
            self.frame_id = 'world'
            self.get_logger().info(
                "moving-base tasks use world-frame positions; "
                "marker frame_id changed to 'world'")

        self.ik = PinocchioIK()
        self.expected_path_segments = self._make_expected_path_segments()
        self.expected_path = np.vstack(self.expected_path_segments)
        self.actual_points = deque(maxlen=self.max_actual_points)
        self.expected_points = deque(maxlen=self.max_actual_points)
        self.command_fk_points = deque(maxlen=self.max_actual_points)
        self.current_actual = None
        self.current_expected = self._target(0.0)
        self.start_time = None
        self.last_command_time = None
        self.last_recorded_actual = None
        self.last_recorded_control = None
        self.last_recorded_command_fk = None
        self.samples = []
        self.control_samples = []
        self.command_fk_samples = []
        self.warmup_reported = False
        self.stale_reported = False

        marker_qos = QoSProfile(depth=10)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.expected_pub = self.create_publisher(
            Marker, '/trajectory/expected', marker_qos)
        self.actual_pub = self.create_publisher(
            Marker, '/trajectory/actual', marker_qos)
        self.command_fk_pub = self.create_publisher(
            Marker, '/trajectory/command_fk', marker_qos)
        self.current_target_pub = self.create_publisher(
            Marker, '/trajectory/current_target', marker_qos)
        self.current_actual_pub = self.create_publisher(
            Marker, '/trajectory/current_actual', marker_qos)

        self.create_subscription(JointState, self.actual_topic, self._joint_state_cb, 10)
        self.create_subscription(JointState, self.command_topic, self._command_cb, 10)
        if self.record_control_signal and self.control_signal_topic != self.command_topic:
            self.create_subscription(JointState, self.control_signal_topic, self._control_signal_cb, 10)

        self.plotter = None
        if self.show_matplotlib:
            self.plotter = MatplotlibPlotter(
                self.get_logger(), self.expected_path_segments)
            self.plotter.start()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_markers)
        self.status_timer = self.create_timer(2.0, self._print_status)
        self.get_logger().info(
            f'Trajectory visualizer started: mode={self.trajectory_mode}, '
            f'shape={self.trajectory_shape}, '
            f'actual_topic={self.actual_topic}, command_topic={self.command_topic}')
        if self.record_control_signal:
            self.get_logger().info(
                f'Control signal recording enabled: topic={self.control_signal_topic}')
        if self.record_command_fk:
            self.get_logger().info(
                f'Command FK recording enabled: topic={self.command_topic}')

    def destroy_node(self):
        self._save_results()
        if self.plotter:
            self.plotter.stop()
        super().destroy_node()

    def _command_cb(self, msg: JointState):
        if self._ignore_command_for_mode(msg):
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        msg_time = self._message_time_sec(msg)
        should_reset = self.start_time is None
        if self.last_command_time is not None and now - self.last_command_time > self.reset_gap_sec:
            should_reset = True
        self.last_command_time = now

        if should_reset:
            self._reset_recording(msg_time)
            self.get_logger().info('Trajectory recording started from /student/joint_command')

        if self.record_command_fk:
            self._record_command_fk(msg)

        if not self.record_control_signal:
            return
        if self.control_signal_topic == self.command_topic:
            self._record_control_signal(msg)

    def _ignore_command_for_mode(self, msg: JointState) -> bool:
        return (
            self._is_moving_base_mode()
            and msg.header.frame_id
            and msg.header.frame_id != 'moving_base_track'
        )

    def _is_moving_base_mode(self) -> bool:
        return self.trajectory_mode in ('moving_base', 'moving_base_circle')

    @staticmethod
    def _append_visible_point(
            records, point: np.ndarray, pen_down: bool, stroke_id: int):
        if not pen_down:
            if records and records[-1][0] is not None:
                records.append((None, None))
            return
        if (
            records
            and records[-1][0] is not None
            and records[-1][1] != int(stroke_id)
        ):
            records.append((None, None))
        records.append((
            np.asarray(point, dtype=float).copy(),
            int(stroke_id),
        ))

    @staticmethod
    def _records_to_segments(records):
        segments = []
        current = []
        current_stroke = None
        for point, stroke_id in records:
            if point is None:
                if len(current) >= 2:
                    segments.append(np.asarray(current))
                current = []
                current_stroke = None
                continue
            if current_stroke is None or stroke_id == current_stroke:
                current.append(point)
            else:
                if len(current) >= 2:
                    segments.append(np.asarray(current))
                current = [point]
            current_stroke = stroke_id
        if len(current) >= 2:
            segments.append(np.asarray(current))
        return segments

    def _control_signal_cb(self, msg: JointState):
        self._record_control_signal(msg)

    def _record_command_fk(self, msg: JointState):
        q = self._extract_q(msg)
        if q is None:
            self.get_logger().warn(
                f'Ignoring command FK JointState: expected {JOINT_COUNT} joint positions',
                throttle_duration_sec=2.0)
            return

        sample_time = self._message_time_sec(msg)
        if self.start_time is None:
            if self.start_on_command:
                return
            self._reset_recording(sample_time)

        elapsed = sample_time - self.start_time
        if elapsed < self.ignore_initial_sec:
            return

        command_fk = self._actual_position(q, elapsed)
        expected = self._target(elapsed)
        error = command_fk - expected
        pen_down, stroke_id = self._drawing_state(elapsed)

        if (
            self.last_recorded_command_fk is not None
            and np.linalg.norm(command_fk - self.last_recorded_command_fk) <= self.actual_change_epsilon
        ):
            return

        self.command_fk_samples.append((
            elapsed, expected.copy(), command_fk.copy(), error.copy(),
            pen_down, stroke_id,
        ))
        self._append_visible_point(
            self.command_fk_points, command_fk, pen_down, stroke_id)
        self.last_recorded_command_fk = command_fk.copy()
        if self.plotter:
            self.plotter.add_command_fk_sample(
                elapsed, expected, command_fk, pen_down, stroke_id)

    def _record_control_signal(self, msg: JointState):
        if not self.record_control_signal:
            return
        q = self._extract_q(msg)
        if q is None:
            self.get_logger().warn(
                f'Ignoring control JointState: expected {JOINT_COUNT} joint positions',
                throttle_duration_sec=2.0)
            return

        sample_time = self._message_time_sec(msg)
        if self.start_time is None:
            if self.start_on_command:
                return
            self._reset_recording(sample_time)

        elapsed = sample_time - self.start_time
        if elapsed < self.ignore_initial_sec:
            return

        if (
            self.last_recorded_control is not None
            and np.linalg.norm(q - self.last_recorded_control) <= self.control_signal_change_epsilon
        ):
            return

        self.control_samples.append((elapsed, q.copy()))
        self.last_recorded_control = q.copy()

    def _joint_state_cb(self, msg: JointState):
        q = self._extract_q(msg)
        if q is None:
            self.get_logger().warn(
                f'Ignoring JointState: expected {JOINT_COUNT} joint positions',
                throttle_duration_sec=2.0)
            return

        sample_time = self._message_time_sec(msg)

        if self.start_time is None:
            if self.start_on_command:
                if self.actual_topic != self.command_topic:
                    return
                self._reset_recording(sample_time)
            else:
                self._reset_recording(sample_time)

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = sample_time - self.start_time
        if elapsed < 0.0:
            return
        expected = self._target(elapsed)
        actual = self._actual_position(q, elapsed)
        pen_down, stroke_id = self._drawing_state(elapsed)
        self.current_expected = expected
        self.current_actual = actual

        if (
            self.start_on_command and self.last_command_time is not None
            and now - self.last_command_time > self.stale_command_timeout_sec
        ):
            if not self.stale_reported:
                self.get_logger().warn(
                    'Command topic is stale; pause trajectory error recording until new command arrives')
                self.stale_reported = True
            return
        self.stale_reported = False

        if elapsed < self.ignore_initial_sec:
            if not self.warmup_reported:
                self.get_logger().info(
                    f'Ignoring the first {self.ignore_initial_sec:.2f}s before recording '
                    'trajectory error')
                self.warmup_reported = True
            return

        # Keep the original trajectory time in CSV/plots. This makes
        # time_sec, expected values, and the target formula share one clock.
        recorded_t = elapsed
        if (
            self.sample_only_on_actual_change and self.last_recorded_actual is not None
            and np.linalg.norm(actual - self.last_recorded_actual) <= self.actual_change_epsilon
        ):
            return

        self._append_visible_point(
            self.actual_points, actual, pen_down, stroke_id)
        self._append_visible_point(
            self.expected_points, expected, pen_down, stroke_id)
        self.samples.append((
            recorded_t, expected.copy(), actual.copy(), actual - expected,
            pen_down, stroke_id,
        ))
        self.last_recorded_actual = actual.copy()

        if self.plotter:
            self.plotter.add_sample(
                recorded_t, expected, actual, pen_down, stroke_id)

    def _reset_recording(self, start_time: float):
        self.start_time = start_time
        self.actual_points.clear()
        self.expected_points.clear()
        self.command_fk_points.clear()
        self.samples.clear()
        self.control_samples.clear()
        self.command_fk_samples.clear()
        self.last_recorded_actual = None
        self.last_recorded_control = None
        self.last_recorded_command_fk = None
        self.warmup_reported = False
        self.stale_reported = False
        if self.plotter:
            self.plotter.reset()

    def _message_time_sec(self, msg: JointState) -> float:
        stamp = msg.header.stamp
        if self.use_message_stamp and (stamp.sec != 0 or stamp.nanosec != 0):
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return self.get_clock().now().nanoseconds * 1e-9

    def _extract_q(self, msg: JointState):
        if len(msg.name) >= JOINT_COUNT:
            named_positions = dict(zip(msg.name, msg.position))
            names = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
            if all(name in named_positions for name in names):
                return np.array([named_positions[name] for name in names], dtype=float)

        if len(msg.position) >= JOINT_COUNT:
            return np.array(msg.position[:JOINT_COUNT], dtype=float)

        return None

    def _target(self, t: float) -> np.ndarray:
        if self.trajectory_mode == 'circle':
            w = 2.0 * math.pi / CIRCLE_PERIOD_SEC
            return CIRCLE_CENTER + CIRCLE_RADIUS * np.array(
                [0.0, math.cos(w * t), math.sin(w * t)])
        if self.trajectory_mode == 'horizontal_circle':
            w = 2.0 * math.pi / HORIZONTAL_CIRCLE_PERIOD_SEC
            return HORIZONTAL_CIRCLE_CENTER + HORIZONTAL_CIRCLE_RADIUS * np.array(
                [math.cos(w * t), math.sin(w * t), 0.0])
        if self._is_moving_base_mode():
            return self._moving_ee_target_world(t)

        s = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / LINE_PERIOD_SEC))
        return POINT_A + s * (POINT_B - POINT_A)

    def _actual_position(self, q: np.ndarray, t: float) -> np.ndarray:
        ee_pos_base, _ = self.ik.forward(q)
        if not self._is_moving_base_mode():
            return ee_pos_base

        base_pos_world = self._moving_base_pose(t)
        base_rot_world = self._moving_base_rotation(t)
        return base_pos_world + base_rot_world @ ee_pos_base

    def _moving_base_pose(self, t: float) -> np.ndarray:
        scaled_t = t / self.base_motion_time_scale
        pose = moving_traj.base_pose(scaled_t)
        if self.base_disturbance_amplitude_m > 0.0:
            pose = pose + moving_traj.base_disturbance_offset(
                scaled_t, self.base_disturbance_amplitude_m,
                self.base_disturbance_seed)
        return pose

    def _moving_base_rotation(self, t: float) -> np.ndarray:
        return moving_traj.base_rotation(
            t / self.base_motion_time_scale,
            self.trajectory_shape,
            t / self.trajectory_time_scale,
        )

    def _moving_ee_target_world(self, t: float) -> np.ndarray:
        return moving_traj.ee_target_world(
            t / self.trajectory_time_scale,
            self.trajectory_shape,
        )

    def _drawing_state(self, t: float):
        if not self._is_moving_base_mode():
            return True, 0
        sample = moving_traj.ee_trajectory_sample(
            t / self.trajectory_time_scale,
            self.trajectory_shape,
        )
        return sample.pen_down, sample.stroke_id

    def _make_expected_path_segments(self):
        if self._is_moving_base_mode():
            return moving_traj.sample_ee_drawing_segments(
                self.trajectory_shape,
                self.expected_path_points,
            )
        periods = {
            'circle': CIRCLE_PERIOD_SEC,
            'horizontal_circle': HORIZONTAL_CIRCLE_PERIOD_SEC,
            'line': LINE_PERIOD_SEC,
        }
        period = periods[self.trajectory_mode]
        ts = np.linspace(0.0, period, self.expected_path_points + 1)
        return [np.array([self._target(t) for t in ts])]

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.expected_pub.publish(self._line_marker(
            marker_id=1,
            ns='trajectory_expected',
            segments=self.expected_path_segments,
            stamp=now,
            color=(0.1, 1.0, 0.1, 1.0),
            width=0.008))

        if self.actual_points:
            actual_segments = self._records_to_segments(self.actual_points)
            self.actual_pub.publish(self._line_marker(
                marker_id=2,
                ns='trajectory_actual',
                segments=actual_segments,
                stamp=now,
                color=(1.0, 0.1, 0.1, 1.0),
                width=0.01))

        if self.command_fk_points:
            command_segments = self._records_to_segments(
                self.command_fk_points)
            self.command_fk_pub.publish(self._line_marker(
                marker_id=5,
                ns='trajectory_command_fk',
                segments=command_segments,
                stamp=now,
                color=(1.0, 0.55, 0.0, 1.0),
                width=0.009))

        self.current_target_pub.publish(self._sphere_marker(
            marker_id=3,
            ns='trajectory_current_target',
            point=self.current_expected,
            stamp=now,
            color=(0.1, 1.0, 0.1, 1.0),
            scale=0.025))

        if self.current_actual is not None:
            self.current_actual_pub.publish(self._sphere_marker(
                marker_id=4,
                ns='trajectory_current_actual',
                point=self.current_actual,
                stamp=now,
                color=(1.0, 0.1, 0.1, 1.0),
                scale=0.025))

    def _print_status(self):
        self.get_logger().info(
            'Published expected path; '
            f'recorded actual samples={len(self.samples)}, '
            f'command FK samples={len(self.command_fk_samples)}',
            throttle_duration_sec=2.0)

    def _base_marker(self, marker_id: int, ns: str, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _line_marker(self, marker_id: int, ns: str, segments, stamp,
                     color, width: float):
        marker = self._base_marker(marker_id, ns, stamp)
        marker.type = Marker.LINE_LIST
        marker.scale.x = float(width)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = []
        for segment in segments:
            for index in range(len(segment) - 1):
                marker.points.extend([
                    to_point(segment[index]),
                    to_point(segment[index + 1]),
                ])
        return marker

    def _sphere_marker(self, marker_id: int, ns: str, point: np.ndarray, stamp,
                       color, scale: float):
        marker = self._base_marker(marker_id, ns, stamp)
        marker.type = Marker.SPHERE
        marker.scale.x = float(scale)
        marker.scale.y = float(scale)
        marker.scale.z = float(scale)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.pose.position = to_point(point)
        return marker

    def _save_results(self):
        self.data_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        mode_name = (
            f'{self.trajectory_mode}_{self.trajectory_shape}'
            if self._is_moving_base_mode() else self.trajectory_mode
        )
        prefix = self.data_dir / f'{mode_name}_trajectory_{stamp}'

        expected_path_csv = prefix.with_name(prefix.name + '_expected_path.csv')
        with expected_path_csv.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'stroke_id', 'index',
                'expected_x', 'expected_y', 'expected_z'])
            for stroke_id, segment in enumerate(self.expected_path_segments):
                for index, point in enumerate(segment):
                    writer.writerow([
                        stroke_id, index,
                        float(point[0]), float(point[1]), float(point[2])])

        if not self.samples and not self.control_samples and not self.command_fk_samples:
            self.get_logger().info(
                f'No actual trajectory samples recorded. Expected path saved to {expected_path_csv}')
            return

        if not self.samples:
            self.get_logger().info(
                f'No actual trajectory samples recorded. Expected path saved to {expected_path_csv}')
            self._save_command_fk_results(prefix)
            self._save_control_results(prefix)
            return

        samples_csv = prefix.with_suffix('.csv')
        with samples_csv.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_sec',
                'expected_x', 'expected_y', 'expected_z',
                'actual_x', 'actual_y', 'actual_z',
                'error_x', 'error_y', 'error_z',
                'pen_down', 'stroke_id'])
            for t, expected, actual, error, pen_down, stroke_id in self.samples:
                writer.writerow([
                    float(t),
                    float(expected[0]), float(expected[1]), float(expected[2]),
                    float(actual[0]), float(actual[1]), float(actual[2]),
                    float(error[0]), float(error[1]), float(error[2]),
                    int(pen_down), int(stroke_id)])

        plot_png = prefix.with_suffix('.png')
        try:
            self._save_final_plot(plot_png)
            self.get_logger().info(
                f'Trajectory data saved: {samples_csv}; figure saved: {plot_png}')
        except Exception as exc:
            self.get_logger().warn(
                f'Trajectory CSV saved to {samples_csv}, but final plot failed: {exc}')

        self._save_command_fk_results(prefix)
        self._save_control_results(prefix)

    def _save_command_fk_results(self, prefix: Path):
        if not self.record_command_fk or not self.command_fk_samples:
            return

        command_fk_csv = prefix.with_name(prefix.name + '_command_fk.csv')
        with command_fk_csv.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_sec',
                'expected_x', 'expected_y', 'expected_z',
                'command_fk_x', 'command_fk_y', 'command_fk_z',
                'command_error_x', 'command_error_y', 'command_error_z',
                'pen_down', 'stroke_id'])
            for (
                    t, expected, command_fk, error,
                    pen_down, stroke_id) in self.command_fk_samples:
                writer.writerow([
                    float(t),
                    float(expected[0]), float(expected[1]), float(expected[2]),
                    float(command_fk[0]), float(command_fk[1]), float(command_fk[2]),
                    float(error[0]), float(error[1]), float(error[2]),
                    int(pen_down), int(stroke_id)])

        command_fk_png = prefix.with_name(prefix.name + '_command_fk.png')
        try:
            self._save_command_fk_plot(command_fk_png)
            self.get_logger().info(
                f'Command FK data saved: {command_fk_csv}; figure saved: {command_fk_png}')
        except Exception as exc:
            self.get_logger().warn(
                f'Command FK CSV saved to {command_fk_csv}, but plot failed: {exc}')

    def _save_control_results(self, prefix: Path):
        if not self.record_control_signal or not self.control_samples:
            return

        control_csv = prefix.with_name(prefix.name + '_control.csv')
        with control_csv.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec'] + [f'joint{i + 1}' for i in range(JOINT_COUNT)])
            for t, q in self.control_samples:
                writer.writerow([float(t)] + [float(v) for v in q])

        control_png = prefix.with_name(prefix.name + '_control.png')
        try:
            self._save_control_plot(control_png)
            self.get_logger().info(
                f'Control signal data saved: {control_csv}; figure saved: {control_png}')
        except Exception as exc:
            self.get_logger().warn(
                f'Control signal CSV saved to {control_csv}, but plot failed: {exc}')

    def _save_final_plot(self, path: Path):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        times = np.array([s[0] for s in self.samples], dtype=float)
        expected = np.array([s[1] for s in self.samples], dtype=float)
        actual = np.array([s[2] for s in self.samples], dtype=float)
        error = np.array([s[3] for s in self.samples], dtype=float)
        command_times = np.array([s[0] for s in self.command_fk_samples], dtype=float)
        command_fk = np.array([s[2] for s in self.command_fk_samples], dtype=float)
        command_error = np.array([s[3] for s in self.command_fk_samples], dtype=float)
        expected_draw = self._sample_segments(self.samples, 1)
        actual_draw = self._sample_segments(self.samples, 2)
        command_draw = self._sample_segments(self.command_fk_samples, 2)

        fig = Figure(figsize=(12, 10), constrained_layout=True)
        FigureCanvasAgg(fig)
        grid = fig.add_gridspec(3, 3, height_ratios=[2.0, 1.0, 1.0])
        ax_3d = fig.add_subplot(grid[0, :], projection='3d')
        ax_track = [fig.add_subplot(grid[1, i]) for i in range(3)]
        ax_err = [fig.add_subplot(grid[2, i]) for i in range(3)]

        self._plot_3d_segments(
            ax_3d, self.expected_path_segments,
            color='tab:green', linestyle='--', label='expected full path')
        self._plot_3d_segments(
            ax_3d, expected_draw,
            color='limegreen', label='expected samples')
        self._plot_3d_segments(
            ax_3d, actual_draw,
            color='tab:red', label='actual')
        if command_draw:
            self._plot_3d_segments(
                ax_3d, command_draw,
                color='tab:orange', label='command FK')
        ax_3d.set_xlabel('x (m)')
        ax_3d.set_ylabel('y (m)')
        ax_3d.set_zlabel('z (m)')
        ax_3d.set_title('End-effector 3D trajectory')
        ax_3d.legend(loc='upper right')
        all_points = np.vstack([self.expected_path, actual])
        if command_fk.size:
            all_points = np.vstack([all_points, command_fk])
        MatplotlibPlotter._set_equal_ish_3d(ax_3d, all_points)

        axis_names = ['x', 'y', 'z']
        for i, axis in enumerate(ax_track):
            axis.plot(times, expected[:, i], color='tab:green', linestyle='--', label='expected')
            if command_fk.size:
                axis.plot(command_times, command_fk[:, i], color='tab:orange', label='command FK')
            axis.plot(times, actual[:, i], color='tab:red', label='actual')
            axis.set_xlabel('time (s)')
            axis.set_ylabel(f'{axis_names[i]} position (m)')
            axis.grid(True)
            axis.legend(loc='upper right')

        for i, axis in enumerate(ax_err):
            axis.plot(times, error[:, i], color='tab:blue')
            if command_error.size:
                axis.plot(
                    command_times,
                    command_error[:, i],
                    color='tab:orange',
                    alpha=0.7,
                    label='command FK error')
            axis.axhline(0.0, color='0.4', linewidth=0.8)
            axis.set_xlabel('time (s)')
            axis.set_ylabel(f'{axis_names[i]} error (m)')
            axis.grid(True)
            if command_error.size:
                axis.legend(loc='upper right')

        mode_title = (
            f'{self.trajectory_mode}/{self.trajectory_shape}'
            if self._is_moving_base_mode() else self.trajectory_mode
        )
        fig.suptitle(f'{mode_title} trajectory tracking')
        fig.savefig(path, dpi=150)

    def _save_command_fk_plot(self, path: Path):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        times = np.array([s[0] for s in self.command_fk_samples], dtype=float)
        expected = np.array([s[1] for s in self.command_fk_samples], dtype=float)
        command_fk = np.array([s[2] for s in self.command_fk_samples], dtype=float)
        error = np.array([s[3] for s in self.command_fk_samples], dtype=float)
        expected_draw = self._sample_segments(self.command_fk_samples, 1)
        command_draw = self._sample_segments(self.command_fk_samples, 2)

        fig = Figure(figsize=(12, 10), constrained_layout=True)
        FigureCanvasAgg(fig)
        grid = fig.add_gridspec(3, 3, height_ratios=[2.0, 1.0, 1.0])
        ax_3d = fig.add_subplot(grid[0, :], projection='3d')
        ax_track = [fig.add_subplot(grid[1, i]) for i in range(3)]
        ax_err = [fig.add_subplot(grid[2, i]) for i in range(3)]

        self._plot_3d_segments(
            ax_3d, self.expected_path_segments,
            color='tab:green', linestyle='--', label='expected full path')
        self._plot_3d_segments(
            ax_3d, expected_draw,
            color='limegreen', label='expected samples')
        self._plot_3d_segments(
            ax_3d, command_draw,
            color='tab:orange', label='command FK')
        ax_3d.set_xlabel('x (m)')
        ax_3d.set_ylabel('y (m)')
        ax_3d.set_zlabel('z (m)')
        ax_3d.set_title('Command end-effector FK trajectory')
        ax_3d.legend(loc='upper right')
        MatplotlibPlotter._set_equal_ish_3d(ax_3d, np.vstack([self.expected_path, command_fk]))

        axis_names = ['x', 'y', 'z']
        for i, axis in enumerate(ax_track):
            axis.plot(times, expected[:, i], color='tab:green', linestyle='--', label='expected')
            axis.plot(times, command_fk[:, i], color='tab:orange', label='command FK')
            axis.set_xlabel('time (s)')
            axis.set_ylabel(f'{axis_names[i]} position (m)')
            axis.grid(True)
            axis.legend(loc='upper right')

        for i, axis in enumerate(ax_err):
            axis.plot(times, error[:, i], color='tab:orange')
            axis.axhline(0.0, color='0.4', linewidth=0.8)
            axis.set_xlabel('time (s)')
            axis.set_ylabel(f'{axis_names[i]} command error (m)')
            axis.grid(True)

        mode_title = (
            f'{self.trajectory_mode}/{self.trajectory_shape}'
            if self._is_moving_base_mode() else self.trajectory_mode
        )
        fig.suptitle(f'{mode_title} command FK')
        fig.savefig(path, dpi=150)

    @staticmethod
    def _sample_segments(samples, point_index: int):
        records = deque()
        for sample in samples:
            TrajectoryVisualizerDemo._append_visible_point(
                records,
                sample[point_index],
                bool(sample[4]),
                int(sample[5]),
            )
        return TrajectoryVisualizerDemo._records_to_segments(records)

    @staticmethod
    def _plot_3d_segments(axis, segments, label=None, **kwargs):
        for index, segment in enumerate(segments):
            axis.plot(
                segment[:, 0],
                segment[:, 1],
                segment[:, 2],
                label=label if index == 0 else None,
                **kwargs,
            )

    def _save_control_plot(self, path: Path):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        times = np.array([s[0] for s in self.control_samples], dtype=float)
        commands = np.array([s[1] for s in self.control_samples], dtype=float)

        fig = Figure(figsize=(12, 10), constrained_layout=True)
        FigureCanvasAgg(fig)
        axes = fig.subplots(4, 2, sharex=True)
        axes_flat = axes.reshape(-1)

        for i in range(JOINT_COUNT):
            axis = axes_flat[i]
            axis.plot(times, commands[:, i], color='tab:red', linewidth=1.0)
            axis.set_ylabel(f'joint{i + 1} (rad)')
            axis.grid(True)

        axes_flat[-1].axis('off')
        for axis in axes_flat[:JOINT_COUNT]:
            axis.set_xlabel('time (s)')

        fig.suptitle(f'{self.trajectory_mode} control signal')
        fig.savefig(path, dpi=150)


def main():
    rclpy.init()
    node = TrajectoryVisualizerDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
