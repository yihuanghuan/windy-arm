#!/usr/bin/env python3
"""世界坐标系下基座绕圆，同时机械臂末端在世界坐标系下带姿态约束绕圆的演示脚本。"""

import math

import numpy as np
import pinocchio as pin
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

import moving_base_trajectory_config as traj
from pinocchio_ik import PinocchioIK

JOINT_COUNT = 7
PUBLISH_RATE_HZ = 50.0
JOINT_LIMIT_WARN_MARGIN_DEG = 5.0
MAX_MARKER_POINTS = 2000
RESET_TOLERANCE_RAD = math.radians(2.0)
RESET_HOLD_SEC = 1.0
MAX_PUBLISHED_JOINT_SPEED_RAD_S = 1.0

WORLD_FRAME = traj.WORLD_FRAME
BASE_FRAME = traj.BASE_FRAME
ROT_WEIGHT = 0.2


class MoveBaseCircleVisualDemo(Node):
    def __init__(self):
        super().__init__('move_base_circle_visual_demo')
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.actual_marker_pub = self.create_publisher(Marker, '/ee_traj_marker', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/ee_target_traj_marker', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)
        self.ik = PinocchioIK()
        self.t = 0.0
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.q_last = None
        self.q_measured = None
        self.last_limit_warn = None
        self.last_joint_print_time = -1.0
        self.last_feedback_warn_time = 0.0
        self.phase = 'reset'
        self.reset_hold_time = 0.0
        self.trajectory_start_time_sec = None
        self.last_tracking_command_time_sec = None
        self.ee_traj_marker = self._create_marker(
            topic_ns='ee_traj', marker_id=0, color=(1.0, 0.2, 0.2), scale=0.005)
        self.ee_target_traj_marker = self._create_marker(
            topic_ns='ee_target_traj', marker_id=1, color=(0.1, 0.8, 0.1), scale=0.004)

        p0 = self._ee_target_base(0.0)
        q0, ok = self.ik.solve(
            p0,
            target_rotation=self._target_rotation_base(0.0),
            q_init=None,
            rot_weight=ROT_WEIGHT,
        )
        if not ok:
            raise RuntimeError(
                f'末端起始点 {p0.tolist()} 在当前姿态约束下 IK 不可达，请调整基座/末端圆参数或姿态')
        self.q_last = q0
        self.reset_q_target = q0.copy()

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            '启动后先复位到轨迹起始点，复位完成后再开始轨迹跟踪；'
            '真实末端轨迹来自 /joint_states -> /ee_traj_marker，'
            '期望轨迹话题: /ee_target_traj_marker，'
            f'复位容差={math.degrees(RESET_TOLERANCE_RAD):.2f} deg, 保持时间={RESET_HOLD_SEC:.1f}s，'
            '数据记录和曲线绘制请使用 trajectory_visualizer_demo.py，Ctrl+C 停止')

    def _joint_states_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        joint_names = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        if not all(name in name_to_pos for name in joint_names):
            return
        self.q_measured = np.array([name_to_pos[name] for name in joint_names], dtype=float)

    def _create_marker(self, topic_ns: str, marker_id: int, color, scale: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.ns = topic_ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.color.a = 1.0
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.pose.orientation.w = 1.0
        return marker

    def _append_point_and_publish(self, marker: Marker, pub, pos_world: np.ndarray):
        pt = Point()
        pt.x = float(pos_world[0])
        pt.y = float(pos_world[1])
        pt.z = float(pos_world[2])
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.points.append(pt)
        if len(marker.points) > MAX_MARKER_POINTS:
            marker.points.pop(0)
        pub.publish(marker)

    def _base_pose(self, t: float) -> np.ndarray:
        return traj.base_pose(t)

    def _base_rotation(self, t: float) -> np.ndarray:
        return traj.base_rotation(t)

    def _ee_target_world(self, t: float) -> np.ndarray:
        return traj.ee_target_world(t)

    def _ee_target_base(self, t: float) -> np.ndarray:
        base_pos_world = self._base_pose(t)
        base_rot_world = self._base_rotation(t)
        ee_pos_world = self._ee_target_world(t)
        return base_rot_world.T @ (ee_pos_world - base_pos_world)

    def _target_rotation_base(self, t: float) -> np.ndarray:
        base_rot_world = self._base_rotation(t)
        return base_rot_world.T @ traj.target_rotation_world()

    def _publish_base_tf(self):
        x, y, z = self._base_pose(self.t)
        base_rot = self._base_rotation(self.t)
        quat = pin.Quaternion(base_rot)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = WORLD_FRAME
        tf_msg.child_frame_id = BASE_FRAME
        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = float(z)
        tf_msg.transform.rotation.x = float(quat.x)
        tf_msg.transform.rotation.y = float(quat.y)
        tf_msg.transform.rotation.z = float(quat.z)
        tf_msg.transform.rotation.w = float(quat.w)
        self.br.sendTransform(tf_msg)

    def _publish_world_markers(self):
        if self.q_measured is None:
            return

        base_pos_world = self._base_pose(self.t)
        base_rot_world = self._base_rotation(self.t)

        ee_pos_base, _ = self.ik.forward(self.q_measured)
        ee_pos_world = base_pos_world + base_rot_world @ ee_pos_base
        ee_target_pos_world = self._ee_target_world(self.t)

        self._append_point_and_publish(self.ee_traj_marker, self.actual_marker_pub, ee_pos_world)
        self._append_point_and_publish(self.ee_target_traj_marker, self.target_marker_pub, ee_target_pos_world)

    def _warn_joint_limits(self, q: np.ndarray):
        margin = np.deg2rad(JOINT_LIMIT_WARN_MARGIN_DEG)
        lower_gap = q - self.ik.q_lower
        upper_gap = self.ik.q_upper - q
        warn_msgs = []

        for idx, (low, high) in enumerate(zip(lower_gap, upper_gap), start=1):
            if low <= margin:
                warn_msgs.append(
                    f'joint{idx} 接近下限，剩余 {math.degrees(max(low, 0.0)):.2f} deg')
            elif high <= margin:
                warn_msgs.append(
                    f'joint{idx} 接近上限，剩余 {math.degrees(max(high, 0.0)):.2f} deg')

        warn_text = '; '.join(warn_msgs) if warn_msgs else None
        if warn_text and warn_text != self.last_limit_warn:
            self.get_logger().warn(f'检测到关节限位风险: {warn_text}')
        self.last_limit_warn = warn_text

    def _print_joint_angles(self):
        if self.q_measured is None:
            return
        if self.t - self.last_joint_print_time < 0.2:
            return
        self.last_joint_print_time = self.t

        joint_text = ', '.join(
            f'joint{i + 1}={angle:.4f} rad ({math.degrees(angle):.2f} deg)'
            for i, angle in enumerate(self.q_measured)
        )
        self.get_logger().info(f'当前关节角: {joint_text}')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sanitize_joint_velocity(self, dq: np.ndarray = None) -> np.ndarray:
        if dq is None:
            return np.zeros(JOINT_COUNT)
        try:
            dq_arr = np.asarray(dq, dtype=float).reshape(JOINT_COUNT)
        except ValueError:
            self.get_logger().warn(
                '关节速度维度非法，改为发布零速度',
                throttle_duration_sec=2.0,
            )
            return np.zeros(JOINT_COUNT)
        if not np.all(np.isfinite(dq_arr)):
            self.get_logger().warn(
                '关节速度包含 NaN/Inf，非法项改为 0',
                throttle_duration_sec=2.0,
            )
            dq_arr = np.where(np.isfinite(dq_arr), dq_arr, 0.0)
        return np.clip(
            dq_arr,
            -MAX_PUBLISHED_JOINT_SPEED_RAD_S,
            MAX_PUBLISHED_JOINT_SPEED_RAD_S,
        )

    def _publish_joint_positions(self, q: np.ndarray, dq: np.ndarray = None):
        msg = JointState()
        if self.phase == 'track':
            msg.header.stamp = self._trajectory_stamp(self.t)
            msg.header.frame_id = 'moving_base_track'
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'moving_base_reset'
        msg.name = [f'joint{i + 1}' for i in range(JOINT_COUNT)]
        msg.position = q.tolist()
        msg.velocity = self._sanitize_joint_velocity(dq).tolist()
        self.pub.publish(msg)

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

    def _run_reset_phase(self):
        self._publish_joint_positions(self.reset_q_target)
        self._print_joint_angles()

        if self.q_measured is None:
            self._warn_missing_joint_states()
            return

        reset_err = np.abs(self.q_measured - self.reset_q_target)
        if np.all(reset_err <= RESET_TOLERANCE_RAD):
            self.reset_hold_time += self.dt
            if self.reset_hold_time >= RESET_HOLD_SEC:
                self.phase = 'track'
                self.q_last = self.reset_q_target.copy()
                self.t = 0.0
                self.trajectory_start_time_sec = None
                self.last_tracking_command_time_sec = self._now_sec()
                self.get_logger().info('机械臂已到达轨迹起始点，开始轨迹跟踪')
        else:
            self.reset_hold_time = 0.0

    def _warn_missing_joint_states(self):
        now_sec = self._now_sec()
        if now_sec - self.last_feedback_warn_time < 2.0:
            return
        self.last_feedback_warn_time = now_sec
        self.get_logger().warn(
            '还没有收到合法 /joint_states 反馈，复位阶段会等待。'
            '请确认 /joint_states 中包含 joint1 到 joint7。',
            throttle_duration_sec=2.0,
        )

    def _run_tracking_phase(self):
        now_sec = self._now_sec()
        q_prev = self.q_last.copy()
        q, ok = self.ik.solve(
            self._ee_target_base(self.t),
            target_rotation=self._target_rotation_base(self.t),
            q_init=q_prev,
            rot_weight=ROT_WEIGHT,
        )
        if not ok:
            self.get_logger().warn('末端 IK 未收敛，跳过本帧', throttle_duration_sec=0.2)
            return
        if not np.all(np.isfinite(q)):
            self.get_logger().warn('IK 结果包含 NaN/Inf，跳过本帧', throttle_duration_sec=0.2)
            return
        elapsed = self.dt
        if self.last_tracking_command_time_sec is not None:
            elapsed = now_sec - self.last_tracking_command_time_sec
        if not math.isfinite(elapsed) or elapsed <= 1e-6:
            dq_cmd = np.zeros(JOINT_COUNT)
        else:
            dq_cmd = (q - q_prev) / elapsed
        dq_cmd = self._sanitize_joint_velocity(dq_cmd)
        self.q_last = q
        self.last_tracking_command_time_sec = now_sec
        self._warn_joint_limits(q)
        self._publish_joint_positions(q, dq_cmd)

        self._print_joint_angles()
        self._publish_world_markers()

    def tick(self):
        if self.phase == 'reset':
            self._publish_base_tf()
            self._run_reset_phase()
            return

        self._publish_base_tf()
        self._run_tracking_phase()
        self.t += self.dt


def main():
    rclpy.init()
    node = MoveBaseCircleVisualDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
