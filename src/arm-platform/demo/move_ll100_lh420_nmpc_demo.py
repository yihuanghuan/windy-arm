#!/usr/bin/env python3
"""LL100-LH420 NMPC path demo with front-propeller and body obstacle avoidance."""

import math
from pathlib import Path

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from ll100_traj_pub import SUPPORTED_PATH_MODES, _build_path as _build_ll100_path
from mpc_pinocchio_ik import (
    DEFAULT_LL100_URDF,
    MpcIkConfig,
    MpcPinocchioIK,
    PACKAGE_ROOT,
)


JOINT_NAMES = ["A_Joint", "B_Joint", "C_Joint", "D_Joint", "E_Joint", "F_Joint"]
BASE_FRAME = "base_link"
LL100_NMPC_URDF = str(PACKAGE_ROOT / "config" / "ll100_lh420_nmpc.urdf")
DEFAULT_LL100_BODY_OBSTACLE_BOXES = (
    # [cx, cy, cz, hx, hy, hz], all in LL100 base_link frame.
    (0.05, 0.0, -0.12, 0.30, 0.24, 0.09),   # lower deck / central body
    (-0.05, 0.0, 0.00, 0.24, 0.18, 0.08),    # rear upper body volume
    (0.31, 0.0, -0.13, 0.09, 0.12, 0.05),    # arm mount support block
)


def _quat_from_z_axis(normal: np.ndarray):
    n = np.asarray(normal, dtype=float)
    norm = np.linalg.norm(n)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    n = n / norm
    z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    dot = float(np.clip(z_axis @ n, -1.0, 1.0))
    if dot > 1.0 - 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    if dot < -1.0 + 1e-9:
        return 1.0, 0.0, 0.0, 0.0
    axis = np.cross(z_axis, n)
    axis = axis / np.linalg.norm(axis)
    angle = math.acos(dot)
    s = math.sin(0.5 * angle)
    return float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), float(math.cos(0.5 * angle))


PUBLISH_RATE_HZ = 50.0
MAX_TRAIL_POINTS = 2500
DEFAULT_MAX_JOINT_STEP = 0.01
DEFAULT_PATH_PREVIEW_STRIDE = 4


def _lerp(a: np.ndarray, b: np.ndarray, alpha: float) -> np.ndarray:
    return (1.0 - alpha) * a + alpha * b


def _resample_polyline(points, count: int) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return np.repeat(pts[:1], count, axis=0)
    segment_lengths = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    total = float(np.sum(segment_lengths))
    if total < 1e-12:
        return np.repeat(pts[:1], count, axis=0)
    distances = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    samples = np.linspace(0.0, total, count, endpoint=False)
    result = []
    seg = 0
    for distance in samples:
        while seg + 1 < len(segment_lengths) and distance > distances[seg + 1]:
            seg += 1
        length = max(segment_lengths[seg], 1e-12)
        alpha = (distance - distances[seg]) / length
        result.append(_lerp(pts[seg], pts[seg + 1], alpha))
    return np.array(result, dtype=float)


def _stroke_text_points(text: str, x: float, y0: float, z0: float, height: float, spacing: float):
    glyphs = {
        "w": [(0, 1), (0.18, 0), (0.36, 0.55), (0.54, 0), (0.72, 1)],
        "i": [(0.36, 0), (0.36, 0.72), (0.36, 0.95), (0.36, 1.0)],
        "n": [(0, 0), (0, 1), (0.55, 1), (0.55, 0)],
        "d": [(0.55, 0), (0.55, 1), (0.05, 1), (0, 0.75), (0, 0.25), (0.05, 0), (0.55, 0)],
        "y": [(0, 1), (0.28, 0.45), (0.55, 1), (0.28, 0.45), (0.20, 0)],
        "l": [(0.28, 1), (0.28, 0), (0.55, 0)],
        "a": [(0, 0), (0.0, 0.85), (0.55, 0.85), (0.55, 0), (0.55, 0.45), (0.0, 0.45)],
        "b": [(0, 0), (0, 1), (0.48, 1), (0.55, 0.75), (0.48, 0.55), (0, 0.55),
              (0.48, 0.55), (0.55, 0.25), (0.48, 0), (0, 0)],
    }
    width = height * 0.55
    pen_y = y0
    points = []
    last = None
    for char in text:
        raw = glyphs.get(char.lower())
        if raw is None:
            pen_y += width + spacing
            continue
        char_points = [
            np.array([x, pen_y + width * px, z0 + height * pz], dtype=float)
            for px, pz in raw
        ]
        if last is not None:
            points.append(last)
            points.append(char_points[0])
        points.extend(char_points)
        last = char_points[-1]
        pen_y += width + spacing
    return points


def _build_path(
    mode: str,
    period: float,
    dt: float,
    obstacle_disks: np.ndarray = None,
    single_target: np.ndarray = None,
) -> np.ndarray:
    if mode == "single_target":
        if single_target is None:
            raise ValueError("single_target 模式需要提供 single_target_position")
        point = np.asarray(single_target, dtype=float).reshape(3)
        # 单点路径:整个时域参考退化成同一个固定目标点,到点后保持。
        return point.reshape(1, 3)
    count = max(20, int(round(period / dt)))
    if mode in ("circle", "lower_obstacle_circle", "windylab", "avoidance_sweep", "front_wing_arc"):
        return _build_ll100_path(mode, count)
    if mode == "front_propeller_line":
        if obstacle_disks is None or len(obstacle_disks) < 2:
            raise ValueError("front_propeller_line 需要先生成前桨避障圆盘")
        sorted_disks = obstacle_disks[np.argsort(obstacle_disks[:, 1])]
        left_center = sorted_disks[0, :3]
        right_center = sorted_disks[-1, :3]
        prop_center = 0.5 * (left_center + right_center)
        # Do not place the demo line at the absolute front of the safety disks:
        # that drives this 6-DoF arm close to a straight-arm singularity.
        # These offsets keep the line outside the front-propeller disks while
        # leaving enough elbow bend for a stable visual demo.
        x = float(prop_center[0] + 0.315)
        z = float(prop_center[2] + 0.22)
        y_extension = 0.02
        start = np.array([x, left_center[1] - y_extension, z], dtype=float)
        end = np.array([x, right_center[1] + y_extension, z], dtype=float)
        return _resample_polyline([start, end, start], count)
    raise ValueError(
        f"path_mode 必须是 {SUPPORTED_PATH_MODES} 或 'single_target'"
    )


class MoveLL100LH420NmpcDemo(Node):
    def __init__(self):
        super().__init__("move_ll100_lh420_nmpc_demo")
        self.declare_parameter("path_mode", "lower_obstacle_circle")
        self.declare_parameter("single_target_position", [0.50, 0.0, 0.10])
        self.declare_parameter("period_sec", 60.0)
        self.declare_parameter("terminal_tolerance", 0.025)
        self.declare_parameter("horizon_steps", 8)
        self.declare_parameter("max_joint_step", DEFAULT_MAX_JOINT_STEP)
        self.declare_parameter("path_preview_stride", DEFAULT_PATH_PREVIEW_STRIDE)
        self.declare_parameter("position_weight", 120.0)
        self.declare_parameter("terminal_position_weight", 1200.0)
        self.declare_parameter("lock_orientation", True)
        self.declare_parameter("orientation_target", "vertical_down")
        self.declare_parameter("orientation_weight", 0.3)
        self.declare_parameter("terminal_orientation_weight", 3.0)
        self.declare_parameter("obstacle_avoidance_enabled", True)
        self.declare_parameter("obstacle_safety_margin", 0.04)
        self.declare_parameter("obstacle_disk_half_thickness", 0.02)
        self.declare_parameter("body_obstacle_enabled", True)
        self.declare_parameter("body_obstacle_margin", 0.015)
        self.declare_parameter("capsule_radius", 0.018)
        self.declare_parameter("capsule_samples_per_segment", 1)
        self.declare_parameter("advance_on_feedback", False)
        self.declare_parameter("start_from_nearest_path_point", True)
        self.declare_parameter("hold_start_until_reached", True)
        self.declare_parameter("target_switch_tolerance", 0.025)
        self.declare_parameter("stuck_error_epsilon", 0.002)
        self.declare_parameter("stuck_steps_before_advance", 120)
        self.declare_parameter("stuck_advance_points", 10)
        self.declare_parameter("current_posture_weight", 0.005)
        self.declare_parameter("center_posture_weight", 0.01)
        self.declare_parameter("metrics_log_sec", 0.0)

        self.path_mode = str(self.get_parameter("path_mode").value)
        self.single_target_position = np.array(
            self.get_parameter("single_target_position").value, dtype=float
        )
        self.period_sec = float(self.get_parameter("period_sec").value)
        self.terminal_tolerance = float(self.get_parameter("terminal_tolerance").value)
        self.horizon_steps = int(self.get_parameter("horizon_steps").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.path_preview_stride = int(self.get_parameter("path_preview_stride").value)
        self.position_weight = float(self.get_parameter("position_weight").value)
        self.terminal_position_weight = float(self.get_parameter("terminal_position_weight").value)
        self.lock_orientation = bool(self.get_parameter("lock_orientation").value)
        self.orientation_target = str(self.get_parameter("orientation_target").value)
        self.orientation_weight = float(self.get_parameter("orientation_weight").value)
        self.terminal_orientation_weight = float(self.get_parameter("terminal_orientation_weight").value)
        self.obstacle_avoidance_enabled = bool(self.get_parameter("obstacle_avoidance_enabled").value)
        self.obstacle_safety_margin = float(self.get_parameter("obstacle_safety_margin").value)
        self.obstacle_disk_half_thickness = float(self.get_parameter("obstacle_disk_half_thickness").value)
        self.body_obstacle_enabled = bool(self.get_parameter("body_obstacle_enabled").value)
        self.body_obstacle_margin = float(self.get_parameter("body_obstacle_margin").value)
        self.capsule_radius = float(self.get_parameter("capsule_radius").value)
        self.capsule_samples_per_segment = int(self.get_parameter("capsule_samples_per_segment").value)
        self.advance_on_feedback = bool(self.get_parameter("advance_on_feedback").value)
        self.start_from_nearest_path_point = bool(self.get_parameter("start_from_nearest_path_point").value)
        self.hold_start_until_reached = bool(self.get_parameter("hold_start_until_reached").value)
        self.target_switch_tolerance = float(self.get_parameter("target_switch_tolerance").value)
        self.stuck_error_epsilon = float(self.get_parameter("stuck_error_epsilon").value)
        self.stuck_steps_before_advance = int(self.get_parameter("stuck_steps_before_advance").value)
        self.stuck_advance_points = int(self.get_parameter("stuck_advance_points").value)
        self.current_posture_weight = float(self.get_parameter("current_posture_weight").value)
        self.center_posture_weight = float(self.get_parameter("center_posture_weight").value)
        self.metrics_log_sec = float(self.get_parameter("metrics_log_sec").value)

        if self.period_sec <= 0.0:
            raise ValueError("period_sec 必须 > 0")
        if self.horizon_steps < 1:
            raise ValueError("horizon_steps 必须 >= 1")
        if self.max_joint_step <= 0.0:
            raise ValueError("max_joint_step 必须 > 0")
        if self.path_preview_stride < 1:
            raise ValueError("path_preview_stride 必须 >= 1")
        if self.orientation_target not in ("initial", "vertical_down"):
            raise ValueError("orientation_target 必须是 'initial' 或 'vertical_down'")
        if self.path_mode == "single_target" and self.single_target_position.shape != (3,):
            raise ValueError("single_target_position 必须是长度为 3 的数组,例如 [0.50, 0.0, 0.10]")
        if not Path(LL100_NMPC_URDF).exists():
            raise FileNotFoundError(f"找不到 LL100 NMPC URDF: {LL100_NMPC_URDF}")

        self.dt = 1.0 / PUBLISH_RATE_HZ
        cfg = MpcIkConfig(
            horizon_steps=self.horizon_steps,
            control_dt=self.dt,
            max_joint_step=self.max_joint_step,
            success_tolerance=self.terminal_tolerance,
            solver_backend="acados",
            acados_build_dir=f"/tmp/windylab_acados_ll100_lh420_path_{self.path_mode}",
            position_weight=self.position_weight,
            terminal_position_weight=self.terminal_position_weight,
            orientation_weight=self.orientation_weight if self.lock_orientation else 0.0,
            terminal_orientation_weight=self.terminal_orientation_weight if self.lock_orientation else 0.0,
            obstacle_avoidance_enabled=self.obstacle_avoidance_enabled,
            obstacle_hard_constraint=True,  # 安全第一:桨叶避障只用硬约束,不保留软约束
            obstacle_safety_margin=self.obstacle_safety_margin,
            obstacle_disk_half_thickness=self.obstacle_disk_half_thickness,
            obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
            obstacle_coordinate_mode="ll100_base",
            obstacle_keypoint_frames=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
            body_obstacle_enabled=self.body_obstacle_enabled,
            body_obstacle_hard_constraint=True,  # 安全第一:机身避障只用硬约束,不保留软约束
            body_obstacle_margin=self.body_obstacle_margin,
            body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
            capsule_radius=self.capsule_radius,
            capsule_samples_per_segment=self.capsule_samples_per_segment,
            capsule_frame_sequence=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
            current_posture_weight=self.current_posture_weight,
            center_posture_weight=self.center_posture_weight,
        )
        self.mpc_ik = MpcPinocchioIK(urdf_path=LL100_NMPC_URDF, ee_frame="F_Link", config=cfg)
        self.path = _build_path(
            self.path_mode,
            self.period_sec,
            self.dt,
            self.mpc_ik.obstacle_disks(),
            single_target=self.single_target_position,
        )
        self.q_last = np.clip(pin.neutral(self.mpc_ik.model), self.mpc_ik.q_lower, self.mpc_ik.q_upper)
        self.q_feedback = None
        self.path_index_initialized = False
        self.locked_rotation = self._orientation_target_rotation(self.q_last)
        self.warm_start = None
        self.t = 0.0
        self.path_index = 0
        self.path_time_offset_index = 0
        self.path_time_tracking_started = not self.hold_start_until_reached
        self.best_current_target_error = float("inf")
        self.stuck_steps = 0
        self.trajectory_start_time_sec = None
        self.actual_trail = []

        self.command_pub = self.create_publisher(JointState, "/student/joint_command", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ll100_nmpc_path/markers", 10)
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            "LL100 路径避障 NMPC 启动: "
            f"path_mode={self.path_mode}, period={self.period_sec:.2f}s, "
            f"horizon={self.horizon_steps}, max_joint_step={self.max_joint_step:.4f}, "
            f"path_preview_stride={self.path_preview_stride}, "
            f"lock_orientation={self.lock_orientation}, orientation_target={self.orientation_target}, "
            f"advance_on_feedback={self.advance_on_feedback}, start_nearest={self.start_from_nearest_path_point}, "
            f"hold_start={self.hold_start_until_reached}, "
            f"front_obstacle={self.obstacle_avoidance_enabled}(hard-only), "
            f"body_obstacle={self.body_obstacle_enabled}(hard-only)"
        )

    def _orientation_target_rotation(self, q0: np.ndarray) -> np.ndarray:
        if self.orientation_target == "vertical_down":
            return np.array(
                [
                    [0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                ],
                dtype=float,
            )
        _, current_rotation = self.mpc_ik.forward(q0)
        return current_rotation

    def _joint_state_callback(self, msg: JointState):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in name_to_index for name in JOINT_NAMES):
            # LL100 rotor publisher also publishes /joint_states with only rot1~rot4.
            # Ignore those partial messages; the student_arm_node message has A~F.
            return
        q = np.array([msg.position[name_to_index[name]] for name in JOINT_NAMES], dtype=float)
        if q.shape == (len(JOINT_NAMES),) and np.all(np.isfinite(q)):
            self.q_feedback = np.clip(q, self.mpc_ik.q_lower, self.mpc_ik.q_upper)

    def _target_sequence(self) -> np.ndarray:
        offsets = self.path_preview_stride * np.arange(self.horizon_steps)
        if self.advance_on_feedback:
            indices = (self.path_index + offsets) % len(self.path)
            return self.path[indices]
        if not self.path_time_tracking_started:
            return np.repeat(self.path[self.path_index].reshape(1, 3), self.horizon_steps, axis=0)
        start = (self._time_path_index() + self.path_time_offset_index) % len(self.path)
        indices = (start + offsets) % len(self.path)
        return self.path[indices]

    def _time_path_index(self) -> int:
        return int(round((self.t % self.period_sec) / self.dt)) % len(self.path)

    def _initialize_path_index(self, actual_pos: np.ndarray):
        if self.start_from_nearest_path_point:
            distances = np.linalg.norm(self.path - actual_pos.reshape(1, 3), axis=1)
            nearest_index = int(np.argmin(distances))
            self.path_index = nearest_index
            if self.path_mode == "front_wing_arc" and distances[nearest_index] > self.target_switch_tolerance:
                center_candidates = np.array([len(self.path) // 4, 3 * len(self.path) // 4], dtype=int)
                candidate_distances = distances[center_candidates]
                self.path_index = int(center_candidates[int(np.argmin(candidate_distances))])
            self.path_time_offset_index = (self.path_index - self._time_path_index()) % len(self.path)
            self.best_current_target_error = float(distances[self.path_index])
            self.get_logger().info(
                "初始化弧线路径并入点: "
                f"path_index={self.path_index}, nearest_index={nearest_index}, "
                f"error={self.best_current_target_error:.4f}, "
                f"time_offset={self.path_time_offset_index}"
            )
        self.path_index_initialized = True

    def _advance_to_next_path_index(self, steps: int = 1):
        next_index = (self.path_index + max(1, int(steps))) % len(self.path)
        if next_index < self.path_index:
            self.actual_trail.clear()
        self.path_index = next_index
        self.best_current_target_error = float("inf")
        self.stuck_steps = 0

    def _advance_path_index_if_ready(self, actual_pos: np.ndarray):
        if not self.path_index_initialized:
            self._initialize_path_index(actual_pos)
        if not self.advance_on_feedback:
            if not self.path_time_tracking_started:
                target = self.path[self.path_index]
                error = float(np.linalg.norm(actual_pos - target))
                if not self.hold_start_until_reached or error <= self.target_switch_tolerance:
                    self.path_time_offset_index = (self.path_index - self._time_path_index()) % len(self.path)
                    self.path_time_tracking_started = True
                    self.get_logger().info(
                        "已进入弧线并启动按时间推进: "
                        f"path_index={self.path_index}, error={error:.4f}, "
                        f"time_offset={self.path_time_offset_index}"
                    )
            return
        target = self.path[self.path_index]
        error = float(np.linalg.norm(actual_pos - target))
        if error <= self.target_switch_tolerance:
            self._advance_to_next_path_index()
            return
        if error < self.best_current_target_error - self.stuck_error_epsilon:
            self.best_current_target_error = error
            self.stuck_steps = 0
            return
        self.stuck_steps += 1
        if self.stuck_steps >= self.stuck_steps_before_advance:
            self.get_logger().warn(
                "当前路径点误差长时间没有继续下降,自动推进到下一个点: "
                f"path_index={self.path_index}, error={error:.4f}, "
                f"best={self.best_current_target_error:.4f}",
                throttle_duration_sec=2.0,
            )
            self._advance_to_next_path_index(self.stuck_advance_points)

    def tick(self):
        if self.q_feedback is None:
            self.get_logger().warn("尚未收到 /joint_states 的 A_Joint~F_Joint 反馈,暂不发布路径命令", throttle_duration_sec=2.0)
            return
        q_current = self.q_feedback.copy()
        actual_pos, _ = self.mpc_ik.forward(q_current)
        self._advance_path_index_if_ready(actual_pos)
        targets = self._target_sequence()
        target_rotation = self.locked_rotation if self.lock_orientation else None
        q_next, ok, info = self.mpc_ik.step(
            targets,
            q_current,
            warm_start=self.warm_start,
            target_rotation=target_rotation,
        )
        self.warm_start = info["trajectory"]

        command_usable = bool(info.get("command_usable", ok))
        dq_cmd = np.zeros(len(JOINT_NAMES))
        if command_usable:
            self.q_last = q_next
            dq_cmd = (q_next - q_current) / self.dt
        else:
            self.get_logger().warn(
                "路径 NMPC 命令不可用,保持上一帧: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"body_margin={info.get('min_body_obstacle_margin')}, "
                f"status={info['status']}, message={info['message']}",
                throttle_duration_sec=1.0,
            )

        if command_usable and not bool(info.get("tracking_ok", ok)):
            self.get_logger().warn(
                "路径 NMPC 正在滚动跟踪: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"body_margin={info.get('min_body_obstacle_margin')}",
                throttle_duration_sec=2.0,
            )

        msg = JointState()
        msg.header.stamp = self._trajectory_stamp(self.t)
        msg.name = JOINT_NAMES
        msg.position = self.q_last.tolist()
        msg.velocity = dq_cmd.tolist()
        self.command_pub.publish(msg)

        self.actual_trail.append(actual_pos.copy())
        if len(self.actual_trail) > MAX_TRAIL_POINTS:
            self.actual_trail.pop(0)
        self._publish_markers(targets[0], actual_pos, q_current)
        if self.metrics_log_sec > 0.0:
            self.get_logger().info(
                f"[metrics] mode={self.path_mode} "
                f"terminal_error={info['terminal_error']:.4f} "
                f"min_margin={info.get('min_obstacle_margin')} "
                f"body_margin={info.get('min_body_obstacle_margin')} "
                f"violation={info.get('obstacle_violation_max')} "
                f"usable={command_usable} tracking_ok={info.get('tracking_ok')}",
                throttle_duration_sec=self.metrics_log_sec,
            )
        if self.advance_on_feedback or self.path_time_tracking_started or not self.hold_start_until_reached:
            self.t += self.dt

    def _publish_markers(self, current_target: np.ndarray, actual_pos: np.ndarray, q_actual: np.ndarray):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for i, disk in enumerate(self.mpc_ik.obstacle_disks()):
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.header.stamp = now
            marker.ns = "front_propeller_disks"
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
            marker.scale.x = marker.scale.y = diameter
            marker.scale.z = 2.0 * float(self.obstacle_disk_half_thickness)
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.05
            marker.color.a = 0.25
            markers.markers.append(marker)

        for i, box in enumerate(self.mpc_ik.body_obstacle_boxes()):
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.header.stamp = now
            marker.ns = "body_obstacle_boxes"
            marker.id = 500 + i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(box[0])
            marker.pose.position.y = float(box[1])
            marker.pose.position.z = float(box[2])
            marker.pose.orientation.w = 1.0
            expanded_half = box[3:] + self.body_obstacle_margin + self.capsule_radius
            marker.scale.x = float(2.0 * expanded_half[0])
            marker.scale.y = float(2.0 * expanded_half[1])
            marker.scale.z = float(2.0 * expanded_half[2])
            marker.color.r = 0.7
            marker.color.g = 0.15
            marker.color.b = 1.0
            marker.color.a = 0.20
            markers.markers.append(marker)

        markers.markers.append(self._line_marker("expected_path", 1000, self.path, (0.0, 0.9, 0.15, 0.9), 0.01, now))
        markers.markers.append(self._line_marker("actual_path", 1001, self.actual_trail, (0.1, 0.45, 1.0, 0.95), 0.012, now))
        markers.markers.append(self._sphere_marker("current_target", 1100, current_target, (0.0, 1.0, 0.2, 1.0), 0.045, now))
        markers.markers.append(self._sphere_marker("current_actual", 1101, actual_pos, (0.1, 0.45, 1.0, 1.0), 0.04, now))

        capsule_samples = self.mpc_ik.capsule_sample_positions(q_actual)
        capsule_marker = Marker()
        capsule_marker.header.frame_id = BASE_FRAME
        capsule_marker.header.stamp = now
        capsule_marker.ns = "capsule_samples"
        capsule_marker.id = 1500
        capsule_marker.type = Marker.SPHERE_LIST
        capsule_marker.action = Marker.ADD
        capsule_marker.pose.orientation.w = 1.0
        capsule_marker.scale.x = capsule_marker.scale.y = capsule_marker.scale.z = float(2.0 * self.capsule_radius)
        capsule_marker.color.r = 0.1
        capsule_marker.color.g = 0.8
        capsule_marker.color.b = 1.0
        capsule_marker.color.a = 0.25
        capsule_marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in capsule_samples]
        markers.markers.append(capsule_marker)
        self.marker_pub.publish(markers)

    def _line_marker(self, ns, marker_id, points, color, width, stamp):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(width)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        return marker

    def _sphere_marker(self, ns, marker_id, point, color, diameter, stamp):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = float(diameter)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _trajectory_stamp(self, trajectory_time_sec: float) -> Time:
        if self.trajectory_start_time_sec is None:
            self.trajectory_start_time_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_sec = self.trajectory_start_time_sec + trajectory_time_sec
        sec = math.floor(stamp_sec)
        nanosec = int(round((stamp_sec - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Time(sec=int(sec), nanosec=nanosec)


def main():
    rclpy.init()
    node = MoveLL100LH420NmpcDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
