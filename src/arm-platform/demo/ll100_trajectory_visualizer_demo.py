#!/usr/bin/env python3
"""LL100-LH420 trajectory visualizer.

This node reuses trajectory_visualizer_demo.MatplotlibPlotter, but switches the
FK model, joint names, planned paths, and obstacle markers to the LL100 six-axis
arm.
"""

import csv
import math
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from ll100_traj_pub import SUPPORTED_PATH_MODES, _build_path
from move_ll100_lh420_nmpc_demo import (
    BASE_FRAME,
    DEFAULT_LL100_BODY_OBSTACLE_BOXES,
    JOINT_NAMES,
    LL100_NMPC_URDF,
    _quat_from_z_axis,
)
from mpc_pinocchio_ik import DEFAULT_LL100_URDF, MpcIkConfig, MpcPinocchioIK
from trajectory_visualizer_demo import MatplotlibPlotter


JOINT_COUNT = 6
PUBLISH_RATE_HZ = 10.0


def to_point(p: np.ndarray) -> Point:
    point = Point()
    point.x = float(p[0])
    point.y = float(p[1])
    point.z = float(p[2])
    return point


class LL100TrajectoryVisualizerDemo(Node):
    def __init__(self):
        super().__init__("ll100_trajectory_visualizer_demo")

        self.trajectory_mode = str(self.declare_parameter("trajectory_mode", "lower_obstacle_circle").value)
        self.period_sec = float(self.declare_parameter("period_sec", 60.0).value)
        self.expected_path_points = int(self.declare_parameter("expected_path_points", 240).value)
        self.max_actual_points = int(self.declare_parameter("max_actual_points", 2000).value)
        self.show_matplotlib = bool(self.declare_parameter("show_matplotlib", True).value)
        self.start_on_command = bool(self.declare_parameter("start_on_command", True).value)
        self.use_message_stamp = bool(self.declare_parameter("use_message_stamp", True).value)
        self.reset_gap_sec = float(self.declare_parameter("reset_gap_sec", 2.0).value)
        self.stale_command_timeout_sec = float(self.declare_parameter("stale_command_timeout_sec", 0.2).value)
        self.ignore_initial_sec = float(self.declare_parameter("ignore_initial_sec", 1.0).value)
        self.sample_only_on_actual_change = bool(self.declare_parameter("sample_only_on_actual_change", True).value)
        self.actual_change_epsilon = float(self.declare_parameter("actual_change_epsilon", 1e-9).value)
        self.record_command_fk = bool(self.declare_parameter("record_command_fk", True).value)
        self.obstacle_safety_margin = float(self.declare_parameter("obstacle_safety_margin", 0.04).value)
        self.obstacle_disk_half_thickness = float(self.declare_parameter("obstacle_disk_half_thickness", 0.02).value)
        self.body_obstacle_margin = float(self.declare_parameter("body_obstacle_margin", 0.03).value)
        self.capsule_radius = float(self.declare_parameter("capsule_radius", 0.045).value)
        self.data_dir = Path(str(self.declare_parameter(
            "data_dir",
            "/home/windylab/code/windylab_ws/src/arm-platform/demo/data").value))

        if self.trajectory_mode not in SUPPORTED_PATH_MODES:
            raise ValueError(f"trajectory_mode 必须是 {SUPPORTED_PATH_MODES}")
        if self.period_sec <= 0.0:
            raise ValueError("period_sec 必须 > 0")
        if self.expected_path_points < 2:
            raise ValueError("expected_path_points 必须 >= 2")
        if self.ignore_initial_sec < 0.0:
            raise ValueError("ignore_initial_sec 必须 >= 0")

        cfg = MpcIkConfig(
            solver_backend="acados",
            obstacle_avoidance_enabled=True,
            obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
            obstacle_coordinate_mode="ll100_base",
            obstacle_keypoint_frames=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
            body_obstacle_enabled=True,
            body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
            capsule_radius=self.capsule_radius,
            capsule_frame_sequence=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
            obstacle_safety_margin=self.obstacle_safety_margin,
            obstacle_disk_half_thickness=self.obstacle_disk_half_thickness,
            body_obstacle_margin=self.body_obstacle_margin,
        )
        self.mpc_ik = MpcPinocchioIK(urdf_path=LL100_NMPC_URDF, ee_frame="F_Link", config=cfg)
        self.current_q = np.clip(pin.neutral(self.mpc_ik.model), self.mpc_ik.q_lower, self.mpc_ik.q_upper)

        # Avoid circular dependency: _target() indexes this raw path; expected_path
        # is then generated from _target().
        self._raw_path = _build_path(self.trajectory_mode, self.expected_path_points)
        self.expected_path = self._make_expected_path()
        self.actual_points = deque(maxlen=self.max_actual_points)
        self.expected_points = deque(maxlen=self.max_actual_points)
        self.command_fk_points = deque(maxlen=self.max_actual_points)
        self.current_actual = None
        self.current_expected = self._target(0.0)
        self.start_time = None
        self.last_command_time = None
        self.last_recorded_actual = None
        self.last_recorded_command_fk = None
        self.samples = []
        self.command_fk_samples = []
        self.warmup_reported = False
        self.stale_reported = False

        self.plotter = None
        if self.show_matplotlib:
            self.plotter = MatplotlibPlotter(self.get_logger(), self.expected_path)
            self.plotter.start()

        marker_qos = QoSProfile(depth=10)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.expected_pub = self.create_publisher(Marker, "/ll100_trajectory/expected", marker_qos)
        self.actual_pub = self.create_publisher(Marker, "/ll100_trajectory/actual", marker_qos)
        self.command_fk_pub = self.create_publisher(Marker, "/ll100_trajectory/command_fk", marker_qos)
        self.current_target_pub = self.create_publisher(Marker, "/ll100_trajectory/current_target", marker_qos)
        self.current_actual_pub = self.create_publisher(Marker, "/ll100_trajectory/current_actual", marker_qos)
        self.obstacle_pub = self.create_publisher(MarkerArray, "/ll100_trajectory/obstacles", marker_qos)
        self.markers_pub = self.create_publisher(MarkerArray, "/ll100_trajectory/markers", marker_qos)

        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.create_subscription(JointState, "/student/joint_command", self._command_cb, 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_markers)
        self.status_timer = self.create_timer(2.0, self._print_status)

        self.get_logger().info(
            "LL100 trajectory visualizer started: "
            f"mode={self.trajectory_mode}, expected_points={len(self.expected_path)}, "
            "actual_topic=/joint_states, command_topic=/student/joint_command"
        )

    def destroy_node(self):
        self._save_results()
        if self.plotter:
            self.plotter.stop()
        super().destroy_node()

    def _command_cb(self, msg: JointState):
        now = self.get_clock().now().nanoseconds * 1e-9
        msg_time = self._message_time_sec(msg)
        should_reset = self.start_time is None
        if self.last_command_time is not None and now - self.last_command_time > self.reset_gap_sec:
            should_reset = True
        self.last_command_time = now

        if should_reset:
            self._reset_recording(msg_time)
            self.get_logger().info("LL100 trajectory recording started from /student/joint_command")

        if self.record_command_fk:
            self._record_command_fk(msg)

    def _record_command_fk(self, msg: JointState):
        q = self._extract_q(msg)
        if q is None:
            self.get_logger().warn(
                f"Ignoring command FK JointState: expected {JOINT_COUNT} LL100 joint positions",
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

        command_fk = self._actual_position(q)
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

    def _joint_state_cb(self, msg: JointState):
        q = self._extract_q(msg)
        if q is None:
            return
        self.current_q = q.copy()

        sample_time = self._message_time_sec(msg)

        if self.start_time is None:
            if self.start_on_command:
                return
            self._reset_recording(sample_time)

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = sample_time - self.start_time
        if elapsed < 0.0:
            return

        expected = self._target(elapsed)
        actual = self._actual_position(q)
        self.current_expected = expected
        self.current_actual = actual

        if (
            self.start_on_command and self.last_command_time is not None
            and now - self.last_command_time > self.stale_command_timeout_sec
        ):
            if not self.stale_reported:
                self.get_logger().warn(
                    "Command topic is stale; pause LL100 trajectory error recording until new command arrives")
                self.stale_reported = True
            return
        self.stale_reported = False

        if elapsed < self.ignore_initial_sec:
            if not self.warmup_reported:
                self.get_logger().info(
                    f"Ignoring the first {self.ignore_initial_sec:.2f}s before recording trajectory error")
                self.warmup_reported = True
            return

        if (
            self.sample_only_on_actual_change and self.last_recorded_actual is not None
            and np.linalg.norm(actual - self.last_recorded_actual) <= self.actual_change_epsilon
        ):
            return

        self.actual_points.append(actual)
        self.expected_points.append(expected)
        self.samples.append((elapsed, expected.copy(), actual.copy(), actual - expected))
        self.last_recorded_actual = actual.copy()

        if self.plotter:
            self.plotter.add_sample(elapsed, expected, actual)

    def _reset_recording(self, start_time: float):
        self.start_time = start_time
        self.actual_points.clear()
        self.expected_points.clear()
        self.command_fk_points.clear()
        self.samples.clear()
        self.command_fk_samples.clear()
        self.last_recorded_actual = None
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
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(name in name_to_pos for name in JOINT_NAMES):
            q = np.array([name_to_pos[name] for name in JOINT_NAMES], dtype=float)
            if np.all(np.isfinite(q)):
                return q
        if len(msg.position) >= JOINT_COUNT:
            q = np.array(msg.position[:JOINT_COUNT], dtype=float)
            if np.all(np.isfinite(q)):
                return q
        return None

    def _target(self, t: float) -> np.ndarray:
        idx = int((t % self.period_sec) / self.period_sec * len(self._raw_path)) % len(self._raw_path)
        return self._raw_path[idx].copy()

    def _actual_position(self, q: np.ndarray) -> np.ndarray:
        pos, _ = self.mpc_ik.forward(q)
        return pos

    def _make_expected_path(self) -> np.ndarray:
        ts = np.linspace(0.0, self.period_sec, self.expected_path_points + 1)
        return np.array([self._target(t) for t in ts], dtype=float)

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        combined = MarkerArray()

        expected_marker = self._line_marker(
            1, "ll100_expected", self.expected_path, now, (0.1, 1.0, 0.1, 1.0), 0.008)
        self.expected_pub.publish(expected_marker)
        combined.markers.append(expected_marker)

        if self.actual_points:
            actual_marker = self._line_marker(
                2, "ll100_actual", np.array(self.actual_points), now, (1.0, 0.1, 0.1, 1.0), 0.01)
            self.actual_pub.publish(actual_marker)
            combined.markers.append(actual_marker)

        if self.command_fk_points:
            command_fk_marker = self._line_marker(
                5, "ll100_command_fk", np.array(self.command_fk_points), now, (1.0, 0.55, 0.0, 1.0), 0.009)
            self.command_fk_pub.publish(command_fk_marker)
            combined.markers.append(command_fk_marker)

        current_target_marker = self._sphere_marker(
            3, "ll100_current_target", self.current_expected, now, (0.1, 1.0, 0.1, 1.0), 0.025)
        self.current_target_pub.publish(current_target_marker)
        combined.markers.append(current_target_marker)

        if self.current_actual is not None:
            current_actual_marker = self._sphere_marker(
                4, "ll100_current_actual", self.current_actual, now, (1.0, 0.1, 0.1, 1.0), 0.025)
            self.current_actual_pub.publish(current_actual_marker)
            combined.markers.append(current_actual_marker)

        obstacle_markers = self._obstacle_markers(now)
        self.obstacle_pub.publish(obstacle_markers)
        combined.markers.extend(obstacle_markers.markers)
        self.markers_pub.publish(combined)

    def _publish_obstacle_markers(self, stamp):
        self.obstacle_pub.publish(self._obstacle_markers(stamp))

    def _obstacle_markers(self, stamp):
        markers = MarkerArray()

        for i, disk in enumerate(self.mpc_ik.obstacle_disks()):
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.header.stamp = stamp
            marker.ns = "front_disks"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(disk[0])
            marker.pose.position.y = float(disk[1])
            marker.pose.position.z = float(disk[2])
            qx, qy, qz, qw = _quat_from_z_axis(disk[3:6])
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            diameter = 2.0 * float(disk[6] + self.obstacle_safety_margin)
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 2.0 * self.obstacle_disk_half_thickness
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.05
            marker.color.a = 0.30
            markers.markers.append(marker)

        for i, box in enumerate(self.mpc_ik.body_obstacle_boxes()):
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.header.stamp = stamp
            marker.ns = "body_boxes"
            marker.id = 100 + i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2])
            marker.pose.orientation.w = 1.0
            half = box[3:] + self.body_obstacle_margin + self.capsule_radius
            marker.scale.x = float(2.0 * half[0])
            marker.scale.y = float(2.0 * half[1])
            marker.scale.z = float(2.0 * half[2])
            marker.color.r = 0.7
            marker.color.g = 0.15
            marker.color.b = 1.0
            marker.color.a = 0.22
            markers.markers.append(marker)

        capsule_samples = self.mpc_ik.capsule_sample_positions(self.current_q)
        if len(capsule_samples) > 0:
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.header.stamp = stamp
            marker.ns = "capsule_samples"
            marker.id = 200
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(2.0 * self.capsule_radius)
            marker.scale.y = float(2.0 * self.capsule_radius)
            marker.scale.z = float(2.0 * self.capsule_radius)
            marker.color.r = 0.1
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.25
            marker.points = [to_point(p) for p in capsule_samples]
            markers.markers.append(marker)

        return markers

    def _print_status(self):
        self.get_logger().info(
            "Published LL100 expected path; "
            f"recorded actual samples={len(self.samples)}, "
            f"command FK samples={len(self.command_fk_samples)}",
            throttle_duration_sec=2.0)

    def _base_marker(self, marker_id: int, ns: str, stamp):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _line_marker(self, marker_id: int, ns: str, points: np.ndarray, stamp, color, width: float):
        marker = self._base_marker(marker_id, ns, stamp)
        marker.type = Marker.LINE_STRIP
        marker.scale.x = float(width)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [to_point(p) for p in points]
        return marker

    def _sphere_marker(self, marker_id: int, ns: str, point: np.ndarray, stamp, color, scale: float):
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
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = self.data_dir / f"ll100_{self.trajectory_mode}_trajectory_{stamp}"

        expected_path_csv = prefix.with_name(prefix.name + "_expected_path.csv")
        with expected_path_csv.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "expected_x", "expected_y", "expected_z"])
            for i, point in enumerate(self.expected_path):
                writer.writerow([i, float(point[0]), float(point[1]), float(point[2])])

        if not self.samples and not self.command_fk_samples:
            self.get_logger().info(f"No LL100 trajectory samples recorded. Expected path saved to {expected_path_csv}")
            return

        if self.samples:
            samples_csv = prefix.with_suffix(".csv")
            with samples_csv.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "time_sec",
                    "expected_x", "expected_y", "expected_z",
                    "actual_x", "actual_y", "actual_z",
                    "error_x", "error_y", "error_z"])
                for t, expected, actual, error in self.samples:
                    writer.writerow([
                        float(t),
                        float(expected[0]), float(expected[1]), float(expected[2]),
                        float(actual[0]), float(actual[1]), float(actual[2]),
                        float(error[0]), float(error[1]), float(error[2])])

            plot_png = prefix.with_suffix(".png")
            try:
                self._save_final_plot(plot_png)
                self.get_logger().info(f"LL100 trajectory data saved: {samples_csv}; figure saved: {plot_png}")
            except Exception as exc:
                self.get_logger().warn(f"LL100 trajectory CSV saved to {samples_csv}, but final plot failed: {exc}")

        self._save_command_fk_results(prefix)

    def _save_command_fk_results(self, prefix: Path):
        if not self.record_command_fk or not self.command_fk_samples:
            return

        command_fk_csv = prefix.with_name(prefix.name + "_command_fk.csv")
        with command_fk_csv.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_sec",
                "expected_x", "expected_y", "expected_z",
                "command_fk_x", "command_fk_y", "command_fk_z",
                "command_error_x", "command_error_y", "command_error_z"])
            for t, expected, command_fk, error in self.command_fk_samples:
                writer.writerow([
                    float(t),
                    float(expected[0]), float(expected[1]), float(expected[2]),
                    float(command_fk[0]), float(command_fk[1]), float(command_fk[2]),
                    float(error[0]), float(error[1]), float(error[2])])

        command_fk_png = prefix.with_name(prefix.name + "_command_fk.png")
        try:
            self._save_command_fk_plot(command_fk_png)
            self.get_logger().info(f"LL100 command FK data saved: {command_fk_csv}; figure saved: {command_fk_png}")
        except Exception as exc:
            self.get_logger().warn(f"LL100 command FK CSV saved to {command_fk_csv}, but plot failed: {exc}")

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
        ax_3d = fig.add_subplot(grid[0, :], projection="3d")
        ax_track = [fig.add_subplot(grid[1, i]) for i in range(3)]
        ax_err = [fig.add_subplot(grid[2, i]) for i in range(3)]

        ax_3d.plot(self.expected_path[:, 0], self.expected_path[:, 1], self.expected_path[:, 2],
                   color="tab:green", linestyle="--", label="expected full path")
        ax_3d.plot(expected[:, 0], expected[:, 1], expected[:, 2], color="limegreen", label="expected samples")
        ax_3d.plot(actual[:, 0], actual[:, 1], actual[:, 2], color="tab:red", label="actual")
        if command_fk.size:
            ax_3d.plot(command_fk[:, 0], command_fk[:, 1], command_fk[:, 2], color="tab:orange", label="command FK")
        ax_3d.set_xlabel("x (m)")
        ax_3d.set_ylabel("y (m)")
        ax_3d.set_zlabel("z (m)")
        ax_3d.set_title("LL100 end-effector 3D trajectory")
        ax_3d.legend(loc="upper right")
        all_points = np.vstack([self.expected_path, actual])
        if command_fk.size:
            all_points = np.vstack([all_points, command_fk])
        MatplotlibPlotter._set_equal_ish_3d(ax_3d, all_points)

        axis_names = ["x", "y", "z"]
        for i, axis in enumerate(ax_track):
            axis.plot(times, expected[:, i], color="tab:green", linestyle="--", label="expected")
            if command_fk.size:
                axis.plot(command_times, command_fk[:, i], color="tab:orange", label="command FK")
            axis.plot(times, actual[:, i], color="tab:red", label="actual")
            axis.set_xlabel("time (s)")
            axis.set_ylabel(f"{axis_names[i]} position (m)")
            axis.grid(True)
            axis.legend(loc="upper right")

        for i, axis in enumerate(ax_err):
            axis.plot(times, error[:, i], color="tab:blue")
            if command_error.size:
                axis.plot(command_times, command_error[:, i], color="tab:orange", alpha=0.7, label="command FK error")
            axis.axhline(0.0, color="0.4", linewidth=0.8)
            axis.set_xlabel("time (s)")
            axis.set_ylabel(f"{axis_names[i]} error (m)")
            axis.grid(True)
            if command_error.size:
                axis.legend(loc="upper right")

        fig.suptitle(f"LL100 {self.trajectory_mode} trajectory tracking")
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
        ax_3d = fig.add_subplot(grid[0, :], projection="3d")
        ax_track = [fig.add_subplot(grid[1, i]) for i in range(3)]
        ax_err = [fig.add_subplot(grid[2, i]) for i in range(3)]

        ax_3d.plot(self.expected_path[:, 0], self.expected_path[:, 1], self.expected_path[:, 2],
                   color="tab:green", linestyle="--", label="expected full path")
        ax_3d.plot(expected[:, 0], expected[:, 1], expected[:, 2], color="limegreen", label="expected samples")
        ax_3d.plot(command_fk[:, 0], command_fk[:, 1], command_fk[:, 2], color="tab:orange", label="command FK")
        ax_3d.set_xlabel("x (m)")
        ax_3d.set_ylabel("y (m)")
        ax_3d.set_zlabel("z (m)")
        ax_3d.set_title("LL100 command end-effector FK trajectory")
        ax_3d.legend(loc="upper right")
        MatplotlibPlotter._set_equal_ish_3d(ax_3d, np.vstack([self.expected_path, command_fk]))

        axis_names = ["x", "y", "z"]
        for i, axis in enumerate(ax_track):
            axis.plot(times, expected[:, i], color="tab:green", linestyle="--", label="expected")
            axis.plot(times, command_fk[:, i], color="tab:orange", label="command FK")
            axis.set_xlabel("time (s)")
            axis.set_ylabel(f"{axis_names[i]} position (m)")
            axis.grid(True)
            axis.legend(loc="upper right")

        for i, axis in enumerate(ax_err):
            axis.plot(times, error[:, i], color="tab:orange")
            axis.axhline(0.0, color="0.4", linewidth=0.8)
            axis.set_xlabel("time (s)")
            axis.set_ylabel(f"{axis_names[i]} command error (m)")
            axis.grid(True)

        fig.suptitle(f"LL100 {self.trajectory_mode} command FK")
        fig.savefig(path, dpi=150)


def main():
    rclpy.init()
    node = LL100TrajectoryVisualizerDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
