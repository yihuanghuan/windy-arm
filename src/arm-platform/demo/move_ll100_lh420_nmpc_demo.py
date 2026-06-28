#!/usr/bin/env python3
"""LL100-LH420 六轴机械臂 ACADOS NMPC demo."""

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

from mpc_pinocchio_ik import (
    DEFAULT_LL100_URDF,
    MpcIkConfig,
    MpcPinocchioIK,
    PACKAGE_ROOT,
)


JOINT_NAMES = ["A_Joint", "B_Joint", "C_Joint", "D_Joint", "E_Joint", "F_Joint"]
PUBLISH_RATE_HZ = 50.0
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


def _disk_signed_distance_np(point: np.ndarray, disk: np.ndarray) -> float:
    rel = np.asarray(point, dtype=float) - disk[:3]
    normal = disk[3:6]
    d_axial = float(np.dot(normal, rel))
    lateral = rel - d_axial * normal
    excess = max(float(np.linalg.norm(lateral)) - float(disk[6]), 0.0)
    return float(math.sqrt(excess ** 2 + d_axial ** 2))


class MoveLL100LH420NmpcDemo(Node):
    def __init__(self):
        super().__init__("move_ll100_lh420_nmpc_demo")
        self.declare_parameter("target_position", [0.50, 0.0, 0.10])
        self.declare_parameter("terminal_tolerance", 0.02)
        self.declare_parameter("horizon_steps", 8)
        self.declare_parameter("max_joint_step", 0.08)
        self.declare_parameter("lock_orientation", False)
        self.declare_parameter("orientation_target", "initial")
        self.declare_parameter("orientation_weight", 2.0)
        self.declare_parameter("terminal_orientation_weight", 20.0)
        self.declare_parameter("obstacle_weight", 2e4)
        self.declare_parameter("obstacle_safety_margin", 0.04)
        self.declare_parameter("obstacle_propeller_radius", 0.27)
        self.declare_parameter("obstacle_disk_half_thickness", 0.02)
        self.declare_parameter("body_obstacle_enabled", True)
        self.declare_parameter("body_obstacle_weight", 3e4)
        self.declare_parameter("body_obstacle_margin", 0.01)
        self.declare_parameter("capsule_radius", 0.018)
        self.declare_parameter("capsule_samples_per_segment", 3)

        self.target_position = np.array(self.get_parameter("target_position").value, dtype=float)
        if self.target_position.shape != (3,):
            raise ValueError("target_position 必须是长度为 3 的数组,例如 [0.50, 0.0, 0.10]")
        self.terminal_tolerance = float(self.get_parameter("terminal_tolerance").value)
        self.horizon_steps = int(self.get_parameter("horizon_steps").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.lock_orientation = bool(self.get_parameter("lock_orientation").value)
        self.orientation_target = self.get_parameter("orientation_target").value
        self.orientation_weight = float(self.get_parameter("orientation_weight").value)
        self.terminal_orientation_weight = float(self.get_parameter("terminal_orientation_weight").value)
        self.obstacle_weight = float(self.get_parameter("obstacle_weight").value)
        self.obstacle_safety_margin = float(self.get_parameter("obstacle_safety_margin").value)
        self.obstacle_propeller_radius = float(self.get_parameter("obstacle_propeller_radius").value)
        self.obstacle_disk_half_thickness = float(self.get_parameter("obstacle_disk_half_thickness").value)
        self.body_obstacle_enabled = bool(self.get_parameter("body_obstacle_enabled").value)
        self.body_obstacle_weight = float(self.get_parameter("body_obstacle_weight").value)
        self.body_obstacle_margin = float(self.get_parameter("body_obstacle_margin").value)
        self.capsule_radius = float(self.get_parameter("capsule_radius").value)
        self.capsule_samples_per_segment = int(self.get_parameter("capsule_samples_per_segment").value)

        if self.horizon_steps < 1:
            raise ValueError("horizon_steps 必须 >= 1")
        if self.max_joint_step <= 0.0:
            raise ValueError("max_joint_step 必须 > 0")
        if self.capsule_radius <= 0.0:
            raise ValueError("capsule_radius 必须 > 0")
        if self.capsule_samples_per_segment < 1:
            raise ValueError("capsule_samples_per_segment 必须 >= 1")
        if self.orientation_target not in ("initial", "vertical_down"):
            raise ValueError("orientation_target 必须是 'initial' 或 'vertical_down'")
        if not Path(LL100_NMPC_URDF).exists():
            raise FileNotFoundError(f"找不到 LL100 NMPC URDF: {LL100_NMPC_URDF}")

        self.dt = 1.0 / PUBLISH_RATE_HZ
        cfg = MpcIkConfig(
            horizon_steps=self.horizon_steps,
            control_dt=self.dt,
            max_joint_step=self.max_joint_step,
            success_tolerance=self.terminal_tolerance,
            solver_backend="acados",
            acados_build_dir="/tmp/windylab_acados_ll100_lh420_nmpc",
            orientation_weight=self.orientation_weight if self.lock_orientation else 0.0,
            terminal_orientation_weight=self.terminal_orientation_weight if self.lock_orientation else 0.0,
            obstacle_avoidance_enabled=True,
            obstacle_weight=self.obstacle_weight,
            obstacle_safety_margin=self.obstacle_safety_margin,
            obstacle_propeller_radius=self.obstacle_propeller_radius,
            obstacle_disk_half_thickness=self.obstacle_disk_half_thickness,
            obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
            obstacle_coordinate_mode="ll100_base",
            obstacle_keypoint_frames=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
            body_obstacle_enabled=self.body_obstacle_enabled,
            body_obstacle_weight=self.body_obstacle_weight,
            body_obstacle_margin=self.body_obstacle_margin,
            body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
            capsule_radius=self.capsule_radius,
            capsule_samples_per_segment=self.capsule_samples_per_segment,
            capsule_frame_sequence=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
        )
        self.mpc_ik = MpcPinocchioIK(
            urdf_path=LL100_NMPC_URDF,
            ee_frame="F_Link",
            config=cfg,
        )
        self.q_last = np.clip(pin.neutral(self.mpc_ik.model), self.mpc_ik.q_lower, self.mpc_ik.q_upper)
        self.locked_rotation = self._orientation_target_rotation(self.q_last)
        self.warm_start = None
        self.t = 0.0
        self.trajectory_start_time_sec = None

        self.command_pub = self.create_publisher(JointState, "/student/joint_command", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ll100_nmpc/markers", 10)
        self.timer = self.create_timer(self.dt, self.tick)
        self._warn_if_target_inside_obstacle()

        self.get_logger().info(
            "LL100-LH420 六轴 NMPC 启动: "
            f"target(base_link)={self.target_position.tolist()}, horizon={self.horizon_steps}, "
            f"max_joint_step={self.max_joint_step:.3f}, obstacle_disks={len(self.mpc_ik.obstacle_disks())}, "
            f"body_obstacle={self.body_obstacle_enabled}, body_boxes={len(self.mpc_ik.body_obstacle_boxes())}, "
            f"lock_orientation={self.lock_orientation}, orientation_target={self.orientation_target}"
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

    def _warn_if_target_inside_obstacle(self):
        min_margin = min(
            _disk_signed_distance_np(self.target_position, disk) - self.obstacle_safety_margin
            for disk in self.mpc_ik.obstacle_disks()
        )
        if min_margin < 0.0:
            self.get_logger().warn(
                "目标点位于前桨圆盘安全距离内, NMPC 会在到达目标和避障之间折中: "
                f"target_margin={min_margin:.4f} m"
            )
        else:
            self.get_logger().info(f"目标点前桨安全裕度: {min_margin:.4f} m")

    def tick(self):
        q_current = self.q_last.copy()
        target_rotation = self.locked_rotation if self.lock_orientation else None
        q_next, ok, info = self.mpc_ik.step(
            self.target_position,
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
                "LL100 NMPC 命令不可用,保持上一帧: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"body_margin={info.get('min_body_obstacle_margin')}, "
                f"violation={info.get('obstacle_violation_max')}, "
                f"status={info['status']}, message={info['message']}",
                throttle_duration_sec=1.0,
            )

        if command_usable and not bool(info.get("tracking_ok", ok)):
            self.get_logger().warn(
                "LL100 NMPC 尚未到达目标,继续滚动靠近: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"body_margin={info.get('min_body_obstacle_margin')}, "
                f"violation={info.get('obstacle_violation_max')}",
                throttle_duration_sec=2.0,
            )

        msg = JointState()
        msg.header.stamp = self._trajectory_stamp(self.t)
        msg.name = JOINT_NAMES
        msg.position = self.q_last.tolist()
        msg.velocity = dq_cmd.tolist()
        self.command_pub.publish(msg)
        self._publish_markers(self.q_last)
        self.t += self.dt

    def _publish_markers(self, q: np.ndarray):
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
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 2.0 * float(self.obstacle_disk_half_thickness)
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.05
            marker.color.a = 0.28
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
            marker.color.a = 0.22
            markers.markers.append(marker)

        keypoints = self.mpc_ik.obstacle_keypoint_positions(q)
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = now
        marker.ns = "arm_keypoints"
        marker.id = 1000
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        marker.color.r = 0.1
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in keypoints.values()]
        markers.markers.append(marker)

        capsule_samples = self.mpc_ik.capsule_sample_positions(q)
        capsule_marker = Marker()
        capsule_marker.header.frame_id = BASE_FRAME
        capsule_marker.header.stamp = now
        capsule_marker.ns = "capsule_samples"
        capsule_marker.id = 1500
        capsule_marker.type = Marker.SPHERE_LIST
        capsule_marker.action = Marker.ADD
        capsule_marker.pose.orientation.w = 1.0
        capsule_marker.scale.x = float(2.0 * self.capsule_radius)
        capsule_marker.scale.y = float(2.0 * self.capsule_radius)
        capsule_marker.scale.z = float(2.0 * self.capsule_radius)
        capsule_marker.color.r = 0.1
        capsule_marker.color.g = 0.8
        capsule_marker.color.b = 1.0
        capsule_marker.color.a = 0.25
        capsule_marker.points = [
            Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            for p in capsule_samples
        ]
        markers.markers.append(capsule_marker)

        target = Marker()
        target.header.frame_id = BASE_FRAME
        target.header.stamp = now
        target.ns = "target"
        target.id = 2000
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose.position.x = float(self.target_position[0])
        target.pose.position.y = float(self.target_position[1])
        target.pose.position.z = float(self.target_position[2])
        target.pose.orientation.w = 1.0
        target.scale.x = 0.05
        target.scale.y = 0.05
        target.scale.z = 0.05
        target.color.r = 0.0
        target.color.g = 0.85
        target.color.b = 0.2
        target.color.a = 1.0
        markers.markers.append(target)
        self.marker_pub.publish(markers)

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
