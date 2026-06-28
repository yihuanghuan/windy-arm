#!/usr/bin/env python3
"""LL100-LH420 NMPC real-time goal following node.

The node only consumes /goal_pose and joint feedback. It publishes one
JointState command per control tick and sends /ll100_nmpc_goal/reached when the
actual end effector reaches the current target.
"""

import math
from collections import deque

from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from move_ll100_lh420_nmpc_demo import (
    BASE_FRAME,
    DEFAULT_LL100_BODY_OBSTACLE_BOXES,
    JOINT_NAMES,
    LL100_NMPC_URDF,
    _quat_from_z_axis,
)
from mpc_pinocchio_ik import DEFAULT_LL100_URDF, MpcIkConfig, MpcPinocchioIK


PUBLISH_RATE_HZ = 50.0


def _quat_to_rotation_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


class MoveLL100NmpcGoalDemo(Node):
    def __init__(self):
        super().__init__("move_ll100_lh420_nmpc_goal_demo")
        self.declare_parameter("horizon_steps", 6)
        self.declare_parameter("max_joint_step", 0.08)
        self.declare_parameter("success_tolerance", 0.015)
        self.declare_parameter("lock_orientation", False)
        self.declare_parameter("orientation_weight", 1.0)
        self.declare_parameter("terminal_orientation_weight", 10.0)
        self.declare_parameter("obstacle_safety_margin", 0.04)
        self.declare_parameter("obstacle_weight", 2e4)
        self.declare_parameter("obstacle_disk_half_thickness", 0.02)
        self.declare_parameter("body_obstacle_enabled", True)
        self.declare_parameter("body_obstacle_weight", 3e4)
        self.declare_parameter("body_obstacle_margin", 0.01)
        self.declare_parameter("capsule_radius", 0.018)
        self.declare_parameter("capsule_samples_per_segment", 1)

        horizon_steps = int(self.get_parameter("horizon_steps").value)
        max_joint_step = float(self.get_parameter("max_joint_step").value)
        success_tolerance = float(self.get_parameter("success_tolerance").value)
        self.lock_orientation = bool(self.get_parameter("lock_orientation").value)
        orientation_weight = float(self.get_parameter("orientation_weight").value)
        terminal_orientation_weight = float(self.get_parameter("terminal_orientation_weight").value)
        self.obstacle_safety_margin = float(self.get_parameter("obstacle_safety_margin").value)
        obstacle_weight = float(self.get_parameter("obstacle_weight").value)
        self.obstacle_disk_half_thickness = float(self.get_parameter("obstacle_disk_half_thickness").value)
        self.body_obstacle_enabled = bool(self.get_parameter("body_obstacle_enabled").value)
        self.body_obstacle_weight = float(self.get_parameter("body_obstacle_weight").value)
        self.body_obstacle_margin = float(self.get_parameter("body_obstacle_margin").value)
        self.capsule_radius = float(self.get_parameter("capsule_radius").value)
        capsule_samples = int(self.get_parameter("capsule_samples_per_segment").value)

        if horizon_steps < 1:
            raise ValueError("horizon_steps 必须 >= 1")
        if max_joint_step <= 0.0:
            raise ValueError("max_joint_step 必须 > 0")
        if success_tolerance <= 0.0:
            raise ValueError("success_tolerance 必须 > 0")
        if capsule_samples < 1:
            raise ValueError("capsule_samples_per_segment 必须 >= 1")

        self.dt = 1.0 / PUBLISH_RATE_HZ
        cfg = MpcIkConfig(
            horizon_steps=horizon_steps,
            control_dt=self.dt,
            max_joint_step=max_joint_step,
            success_tolerance=success_tolerance,
            solver_backend="acados",
            acados_build_dir="/tmp/windylab_acados_ll100_goal",
            orientation_weight=orientation_weight if self.lock_orientation else 0.0,
            terminal_orientation_weight=terminal_orientation_weight if self.lock_orientation else 0.0,
            obstacle_avoidance_enabled=True,
            obstacle_weight=obstacle_weight,
            obstacle_safety_margin=self.obstacle_safety_margin,
            obstacle_disk_half_thickness=self.obstacle_disk_half_thickness,
            obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
            obstacle_coordinate_mode="ll100_base",
            obstacle_keypoint_frames=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
            body_obstacle_enabled=self.body_obstacle_enabled,
            body_obstacle_weight=self.body_obstacle_weight,
            body_obstacle_margin=self.body_obstacle_margin,
            body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
            capsule_radius=self.capsule_radius,
            capsule_samples_per_segment=capsule_samples,
            capsule_frame_sequence=("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link"),
        )
        self.mpc_ik = MpcPinocchioIK(urdf_path=LL100_NMPC_URDF, ee_frame="F_Link", config=cfg)
        self.q_current = np.clip(pin.neutral(self.mpc_ik.model), self.mpc_ik.q_lower, self.mpc_ik.q_upper)
        self.q_feedback_received = False
        self.target_position = None
        self.target_rotation = None
        self.goal_reached = False
        self.warm_start = None
        self.last_info = None
        self.actual_trail = deque(maxlen=500)

        self.command_pub = self.create_publisher(JointState, "/student/joint_command", 10)
        self.reached_pub = self.create_publisher(Bool, "/ll100_nmpc_goal/reached", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ll100_nmpc_goal/markers", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_callback, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            "LL100 目标跟随 NMPC 启动: "
            f"horizon={horizon_steps}, max_joint_step={max_joint_step:.3f}, "
            f"success_tolerance={success_tolerance:.3f}, lock_orientation={self.lock_orientation}, "
            f"front_disks={len(self.mpc_ik.obstacle_disks())}, body_obstacle={self.body_obstacle_enabled}"
        )

    def _goal_callback(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != BASE_FRAME:
            self.get_logger().warn(
                f"/goal_pose frame_id='{msg.header.frame_id}', 当前节点按 {BASE_FRAME} 坐标直接使用",
                throttle_duration_sec=2.0,
        )
        p = msg.pose.position
        target_position = np.array([p.x, p.y, p.z], dtype=float)
        target_rotation = None
        if self.lock_orientation:
            q = msg.pose.orientation
            target_rotation = _quat_to_rotation_matrix(q.w, q.x, q.y, q.z)
        same_position = (
            self.target_position is not None
            and np.linalg.norm(target_position - self.target_position) < 1e-9
        )
        same_rotation = (
            not self.lock_orientation
            or (
                self.target_rotation is not None
                and target_rotation is not None
                and np.linalg.norm(target_rotation - self.target_rotation) < 1e-9
            )
        )
        if same_position and same_rotation:
            return
        self.target_position = target_position
        self.goal_reached = False
        self.warm_start = None
        self.target_rotation = target_rotation
        self.reached_pub.publish(Bool(data=False))
        self.get_logger().info(f"收到新目标: pos={self.target_position.tolist()}")

    def _joint_state_callback(self, msg: JointState):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in name_to_index for name in JOINT_NAMES):
            return
        q = np.array([msg.position[name_to_index[name]] for name in JOINT_NAMES], dtype=float)
        if q.shape == (len(JOINT_NAMES),) and np.all(np.isfinite(q)):
            self.q_current = np.clip(q, self.mpc_ik.q_lower, self.mpc_ik.q_upper)
            self.q_feedback_received = True

    def tick(self):
        if self.target_position is None:
            self._publish_markers()
            return
        if self.goal_reached:
            self._publish_markers()
            return
        if not self.q_feedback_received:
            self.get_logger().warn("等待 /joint_states 中的 A_Joint~F_Joint 反馈...", throttle_duration_sec=2.0)
            return

        q_next, ok, info = self.mpc_ik.step(
            self.target_position,
            self.q_current,
            warm_start=self.warm_start,
            target_rotation=self.target_rotation,
        )
        self.warm_start = info["trajectory"]
        self.last_info = info

        command_usable = bool(info.get("command_usable", ok))
        if command_usable:
            dq_cmd = (q_next - self.q_current) / self.dt
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = JOINT_NAMES
            cmd.position = q_next.tolist()
            cmd.velocity = dq_cmd.tolist()
            self.command_pub.publish(cmd)
        else:
            self.get_logger().warn(
                "目标跟随 NMPC 命令不可用: "
                f"terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"status={info['status']}, message={info['message']}",
                throttle_duration_sec=1.0,
            )

        actual_pos, _ = self.mpc_ik.forward(self.q_current)
        self.actual_trail.append(actual_pos.copy())
        dist = float(np.linalg.norm(actual_pos - self.target_position))
        if dist < self.mpc_ik.config.success_tolerance:
            self.goal_reached = True
            self.reached_pub.publish(Bool(data=True))
            self.get_logger().info(f"已到达目标: dist={dist:.4f} m")
        elif command_usable and not bool(info.get("tracking_ok", ok)):
            self.get_logger().warn(
                "目标跟随 NMPC 正在滚动靠近: "
                f"actual_error={dist:.4f}, terminal_error={info['terminal_error']:.4f}, "
                f"min_margin={info.get('min_obstacle_margin')}, "
                f"body_margin={info.get('min_body_obstacle_margin')}",
                throttle_duration_sec=2.0,
            )

        self._publish_markers()

    def _publish_markers(self):
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

        if self.q_feedback_received:
            keypoints = self.mpc_ik.obstacle_keypoint_positions(self.q_current)
            marker = Marker()
            marker.header.frame_id = BASE_FRAME
            marker.header.stamp = now
            marker.ns = "arm_keypoints"
            marker.id = 1000
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.035
            marker.color.r = 0.1
            marker.color.g = 0.45
            marker.color.b = 1.0
            marker.color.a = 0.95
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in keypoints.values()]
            markers.markers.append(marker)

            actual_pos, _ = self.mpc_ik.forward(self.q_current)
            markers.markers.append(self._sphere_marker("actual_ee", 1100, actual_pos, (0.1, 0.45, 1.0, 1.0), 0.04, now))

            capsule_samples = self.mpc_ik.capsule_sample_positions(self.q_current)
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

        if self.target_position is not None:
            color = (0.0, 1.0, 0.2, 1.0) if not self.goal_reached else (0.0, 0.85, 1.0, 1.0)
            markers.markers.append(self._sphere_marker("goal", 2000, self.target_position, color, 0.05, now))

        if len(self.actual_trail) >= 2:
            trail = Marker()
            trail.header.frame_id = BASE_FRAME
            trail.header.stamp = now
            trail.ns = "actual_trail"
            trail.id = 3000
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.pose.orientation.w = 1.0
            trail.scale.x = 0.005
            trail.color.g = 0.4
            trail.color.b = 1.0
            trail.color.a = 0.9
            trail.points = [
                Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                for p in self.actual_trail
            ]
            markers.markers.append(trail)

        self.marker_pub.publish(markers)

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


def main(args=None):
    rclpy.init(args=args)
    node = MoveLL100NmpcGoalDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
