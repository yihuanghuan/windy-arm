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
    def __init__(self, logger, expected_path: np.ndarray):
        self._logger = logger
        self._expected_path = expected_path
        self._times = deque(maxlen=2000)
        self._expected = deque(maxlen=2000)
        self._actual = deque(maxlen=2000)
        self._errors = deque(maxlen=2000)
        self._command_times = deque(maxlen=2000)
        self._command_expected = deque(maxlen=2000)
        self._command_fk = deque(maxlen=2000)
        self._command_errors = deque(maxlen=2000)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def add_sample(self, t: float, expected: np.ndarray, actual: np.ndarray):
        with self._lock:
            self._times.append(float(t))
            self._expected.append(expected.copy())
            self._actual.append(actual.copy())
            self._errors.append(actual - expected)

    def add_command_fk_sample(self, t: float, expected: np.ndarray, command_fk: np.ndarray):
        with self._lock:
            self._command_times.append(float(t))
            self._command_expected.append(expected.copy())
            self._command_fk.append(command_fk.copy())
            self._command_errors.append(command_fk - expected)

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
        return times, expected, actual, errors, command_times, command_expected, command_fk, command_errors

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

            expected_full, = ax_3d.plot(
                self._expected_path[:, 0],
                self._expected_path[:, 1],
                self._expected_path[:, 2],
                color='tab:green',
                linestyle='--',
                label='expected path')
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
                ) = self._snapshot()
                if actual.size:
                    expected_line.set_data(expected[:, 0], expected[:, 1])
                    expected_line.set_3d_properties(expected[:, 2])
                    actual_line.set_data(actual[:, 0], actual[:, 1])
                    actual_line.set_3d_properties(actual[:, 2])

                    all_points = np.vstack([self._expected_path, actual])
                    if command_fk.size:
                        command_line.set_data(command_fk[:, 0], command_fk[:, 1])
                        command_line.set_3d_properties(command_fk[:, 2])
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
        self.max_actual_points = int(self.declare_parameter('max_actual_points', 2000).value)
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
        self.data_dir = Path(str(self.declare_parameter(
            'data_dir',
            '/home/windylab/code/windylab_ws/src/arm-platform/demo/data').value))

        if self.trajectory_mode not in ('circle', 'horizontal_circle', 'line', 'moving_base_circle'):
            raise ValueError(
                "trajectory_mode must be 'circle', 'horizontal_circle', 'line' "
                "or 'moving_base_circle'")
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        if self.ignore_initial_sec < 0.0:
            raise ValueError('ignore_initial_sec must be non-negative')
        if self.trajectory_mode == 'moving_base_circle' and self.frame_id == 'base_link':
            self.frame_id = 'world'
            self.get_logger().info(
                "moving_base_circle uses world-frame positions; marker frame_id changed to 'world'")

        self.ik = PinocchioIK()
        self.expected_path = self._make_expected_path()
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
            self.plotter = MatplotlibPlotter(self.get_logger(), self.expected_path)
            self.plotter.start()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_markers)
        self.status_timer = self.create_timer(2.0, self._print_status)
        self.get_logger().info(
            f'Trajectory visualizer started: mode={self.trajectory_mode}, '
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
            self.trajectory_mode == 'moving_base_circle'
            and msg.header.frame_id
            and msg.header.frame_id != 'moving_base_track'
        )

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

        if (
            self.last_recorded_command_fk is not None
            and np.linalg.norm(command_fk - self.last_recorded_command_fk) <= self.actual_change_epsilon
        ):
            return

        self.command_fk_samples.append((elapsed, expected.copy(), command_fk.copy(), error.copy()))
        self.command_fk_points.append(command_fk)
        self.last_recorded_command_fk = command_fk.copy()
        if self.plotter:
            self.plotter.add_command_fk_sample(elapsed, expected, command_fk)

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

        self.actual_points.append(actual)
        self.expected_points.append(expected)
        self.samples.append((recorded_t, expected.copy(), actual.copy(), actual - expected))
        self.last_recorded_actual = actual.copy()

        if self.plotter:
            self.plotter.add_sample(recorded_t, expected, actual)

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
        if self.trajectory_mode == 'moving_base_circle':
            return self._moving_ee_target_world(t)

        s = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / LINE_PERIOD_SEC))
        return POINT_A + s * (POINT_B - POINT_A)

    def _actual_position(self, q: np.ndarray, t: float) -> np.ndarray:
        ee_pos_base, _ = self.ik.forward(q)
        if self.trajectory_mode != 'moving_base_circle':
            return ee_pos_base

        base_pos_world = self._moving_base_pose(t)
        base_rot_world = self._moving_base_rotation(t)
        return base_pos_world + base_rot_world @ ee_pos_base

    @staticmethod
    def _moving_base_pose(t: float) -> np.ndarray:
        return moving_traj.base_pose(t)

    @staticmethod
    def _moving_base_rotation(t: float) -> np.ndarray:
        return moving_traj.base_rotation(t)

    @staticmethod
    def _moving_ee_target_world(t: float) -> np.ndarray:
        return moving_traj.ee_target_world(t)

    def _make_expected_path(self) -> np.ndarray:
        periods = {
            'circle': CIRCLE_PERIOD_SEC,
            'horizontal_circle': HORIZONTAL_CIRCLE_PERIOD_SEC,
            'line': LINE_PERIOD_SEC,
            'moving_base_circle': moving_traj.EE_PERIOD_SEC,
        }
        period = periods[self.trajectory_mode]
        ts = np.linspace(0.0, period, self.expected_path_points + 1)
        return np.array([self._target(t) for t in ts])

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.expected_pub.publish(self._line_marker(
            marker_id=1,
            ns='trajectory_expected',
            points=self.expected_path,
            stamp=now,
            color=(0.1, 1.0, 0.1, 1.0),
            width=0.008))

        if self.actual_points:
            self.actual_pub.publish(self._line_marker(
                marker_id=2,
                ns='trajectory_actual',
                points=np.array(self.actual_points),
                stamp=now,
                color=(1.0, 0.1, 0.1, 1.0),
                width=0.01))

        if self.command_fk_points:
            self.command_fk_pub.publish(self._line_marker(
                marker_id=5,
                ns='trajectory_command_fk',
                points=np.array(self.command_fk_points),
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

    def _line_marker(self, marker_id: int, ns: str, points: np.ndarray, stamp,
                     color, width: float):
        marker = self._base_marker(marker_id, ns, stamp)
        marker.type = Marker.LINE_STRIP
        marker.scale.x = float(width)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [to_point(p) for p in points]
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
        prefix = self.data_dir / f'{self.trajectory_mode}_trajectory_{stamp}'

        expected_path_csv = prefix.with_name(prefix.name + '_expected_path.csv')
        with expected_path_csv.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 'expected_x', 'expected_y', 'expected_z'])
            for i, p in enumerate(self.expected_path):
                writer.writerow([i, float(p[0]), float(p[1]), float(p[2])])

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
                'error_x', 'error_y', 'error_z'])
            for t, expected, actual, error in self.samples:
                writer.writerow([
                    float(t),
                    float(expected[0]), float(expected[1]), float(expected[2]),
                    float(actual[0]), float(actual[1]), float(actual[2]),
                    float(error[0]), float(error[1]), float(error[2])])

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
                'command_error_x', 'command_error_y', 'command_error_z'])
            for t, expected, command_fk, error in self.command_fk_samples:
                writer.writerow([
                    float(t),
                    float(expected[0]), float(expected[1]), float(expected[2]),
                    float(command_fk[0]), float(command_fk[1]), float(command_fk[2]),
                    float(error[0]), float(error[1]), float(error[2])])

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

        fig = Figure(figsize=(12, 10), constrained_layout=True)
        FigureCanvasAgg(fig)
        grid = fig.add_gridspec(3, 3, height_ratios=[2.0, 1.0, 1.0])
        ax_3d = fig.add_subplot(grid[0, :], projection='3d')
        ax_track = [fig.add_subplot(grid[1, i]) for i in range(3)]
        ax_err = [fig.add_subplot(grid[2, i]) for i in range(3)]

        ax_3d.plot(
            self.expected_path[:, 0],
            self.expected_path[:, 1],
            self.expected_path[:, 2],
            color='tab:green',
            linestyle='--',
            label='expected full path')
        ax_3d.plot(
            expected[:, 0], expected[:, 1], expected[:, 2],
            color='limegreen',
            label='expected samples')
        ax_3d.plot(
            actual[:, 0], actual[:, 1], actual[:, 2],
            color='tab:red',
            label='actual')
        if command_fk.size:
            ax_3d.plot(
                command_fk[:, 0], command_fk[:, 1], command_fk[:, 2],
                color='tab:orange',
                label='command FK')
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

        fig.suptitle(f'{self.trajectory_mode} trajectory tracking')
        fig.savefig(path, dpi=150)

    def _save_command_fk_plot(self, path: Path):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        times = np.array([s[0] for s in self.command_fk_samples], dtype=float)
        expected = np.array([s[1] for s in self.command_fk_samples], dtype=float)
        command_fk = np.array([s[2] for s in self.command_fk_samples], dtype=float)
        error = np.array([s[3] for s in self.command_fk_samples], dtype=float)

        fig = Figure(figsize=(12, 10), constrained_layout=True)
        FigureCanvasAgg(fig)
        grid = fig.add_gridspec(3, 3, height_ratios=[2.0, 1.0, 1.0])
        ax_3d = fig.add_subplot(grid[0, :], projection='3d')
        ax_track = [fig.add_subplot(grid[1, i]) for i in range(3)]
        ax_err = [fig.add_subplot(grid[2, i]) for i in range(3)]

        ax_3d.plot(
            self.expected_path[:, 0],
            self.expected_path[:, 1],
            self.expected_path[:, 2],
            color='tab:green',
            linestyle='--',
            label='expected full path')
        ax_3d.plot(
            expected[:, 0], expected[:, 1], expected[:, 2],
            color='limegreen',
            label='expected samples')
        ax_3d.plot(
            command_fk[:, 0], command_fk[:, 1], command_fk[:, 2],
            color='tab:orange',
            label='command FK')
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

        fig.suptitle(f'{self.trajectory_mode} command FK')
        fig.savefig(path, dpi=150)

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
