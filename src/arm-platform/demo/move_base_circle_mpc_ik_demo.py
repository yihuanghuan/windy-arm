#!/usr/bin/env python3
"""基座晃动场景下的 ACADOS MPC 末端轨迹跟踪 demo。"""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, TransformStamped
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

import moving_base_trajectory_config as traj
from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
from pinocchio_ik import PinocchioIK

JOINT_COUNT = 7
PUBLISH_RATE_HZ = 50.0
JOINT_LIMIT_WARN_MARGIN_DEG = 5.0
MAX_MARKER_POINTS = 2000
RESET_TOLERANCE_RAD = math.radians(2.0)
RESET_HOLD_SEC = 1.0
RECOMMENDED_CONTROLLER_MAX_VELOCITY_RAD_S = 1.0

WORLD_FRAME = traj.WORLD_FRAME
BASE_FRAME = traj.BASE_FRAME


class MoveBaseCircleMpcIkDemo(Node):
    def __init__(self):
        super().__init__('move_base_circle_mpc_ik_demo')
        # 默认值取自 tune_unified_mpc_params.py 统一闭环调参结果
        # (固定臂+浮动基座臂四场景统一): h=2, step=0.02, ow=0.375, tow=5.0。
        self.declare_parameter('solver_backend', 'acados')
        self.declare_parameter('horizon_steps', 2)
        self.declare_parameter('max_joint_step', 0.02)
        self.declare_parameter('terminal_tolerance', 0.01)
        self.declare_parameter('orientation_weight', 0.375)
        self.declare_parameter('terminal_orientation_weight', 5.0)
        self.declare_parameter('acados_rti_iterations', 3)
        self.declare_parameter('position_weight', 50.0)
        self.declare_parameter('terminal_position_weight', 500.0)
        self.declare_parameter('joint_delta_weight', 0.08)
        self.declare_parameter('joint_delta_change_weight', 0.25)
        self.declare_parameter('current_posture_weight', 0.01)
        self.declare_parameter('center_posture_weight', 0.002)
        self.declare_parameter('command_fk_step_warn_m', 0.006)

        self.solver_backend = self.get_parameter('solver_backend').value
        self.horizon_steps = int(self.get_parameter('horizon_steps').value)
        self.max_joint_step = float(self.get_parameter('max_joint_step').value)
        self.terminal_tolerance = float(self.get_parameter('terminal_tolerance').value)
        self.orientation_weight = float(self.get_parameter('orientation_weight').value)
        self.terminal_orientation_weight = float(
            self.get_parameter('terminal_orientation_weight').value)
        self.acados_rti_iterations = int(self.get_parameter('acados_rti_iterations').value)
        self.position_weight = float(self.get_parameter('position_weight').value)
        self.terminal_position_weight = float(
            self.get_parameter('terminal_position_weight').value)
        self.joint_delta_weight = float(self.get_parameter('joint_delta_weight').value)
        self.joint_delta_change_weight = float(
            self.get_parameter('joint_delta_change_weight').value)
        self.current_posture_weight = float(
            self.get_parameter('current_posture_weight').value)
        self.center_posture_weight = float(
            self.get_parameter('center_posture_weight').value)
        self.command_fk_step_warn_m = float(
            self.get_parameter('command_fk_step_warn_m').value)

        if self.solver_backend != 'acados':
            raise ValueError("solver_backend must be 'acados'")
        if self.horizon_steps < 1:
            raise ValueError('horizon_steps must be >= 1')
        if self.max_joint_step <= 0.0:
            raise ValueError('max_joint_step must be > 0')

        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.actual_marker_pub = self.create_publisher(Marker, '/ee_traj_marker', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/ee_target_traj_marker', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)

        self.dt = 1.0 / PUBLISH_RATE_HZ
        config = MpcIkConfig(
            horizon_steps=self.horizon_steps,
            control_dt=self.dt,
            max_joint_step=self.max_joint_step,
            success_tolerance=self.terminal_tolerance,
            solver_backend=self.solver_backend,
            position_weight=self.position_weight,
            terminal_position_weight=self.terminal_position_weight,
            orientation_weight=self.orientation_weight,
            terminal_orientation_weight=self.terminal_orientation_weight,
            joint_delta_weight=self.joint_delta_weight,
            joint_delta_change_weight=self.joint_delta_change_weight,
            current_posture_weight=self.current_posture_weight,
            center_posture_weight=self.center_posture_weight,
            acados_rti_iterations=self.acados_rti_iterations,
            acados_build_dir='/tmp/windylab_acados_moving_base_mpc_ik',
        )
        self.mpc_ik = MpcPinocchioIK(config=config)
        self.seed_ik = PinocchioIK()

        self.t = 0.0
        self.q_last = None
        self.q_measured = None
        self.warm_start = None
        self.last_limit_warn = None
        self.last_joint_print_time = -1.0
        self.last_feedback_warn_time = 0.0
        self.phase = 'reset'
        self.reset_hold_time = 0.0
        self.trajectory_start_time_sec = None
        self.last_command_fk_world = None

        self.ee_traj_marker = self._create_marker(
            topic_ns='ee_traj_mpc', marker_id=10, color=(1.0, 0.2, 0.2), scale=0.005)
        self.ee_target_traj_marker = self._create_marker(
            topic_ns='ee_target_traj_mpc', marker_id=11, color=(0.1, 0.8, 0.1), scale=0.004)

        p0 = self._ee_target_base(0.0)
        r0 = self._target_rotation_base(0.0)
        q0, ok = self.seed_ik.solve(p0, target_rotation=r0, rot_weight=0.2)
        if not ok:
            raise RuntimeError(
                f'末端起始点 {p0.tolist()} 在当前姿态约束下 IK 不可达，请调整轨迹参数')
        q_pose, pose_ok, pose_info = self.mpc_ik.solve(
            p0,
            q_init=q0,
            target_rotation=r0,
        )
        if bool(pose_info.get('command_usable', pose_ok)):
            q0 = q_pose
            self.warm_start = pose_info['trajectory']
        else:
            self.get_logger().warn(
                'MPC 起始姿态目标没有完全收敛，先使用普通 IK 起点: '
                f"terminal_error={pose_info['terminal_error']:.4f}, "
                f"orientation_error={pose_info.get('terminal_orientation_error')}")

        self.q_last = q0
        self.reset_q_target = q0.copy()

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'基座晃动 ACADOS MPC demo 启动: solver={self.solver_backend}, '
            f'horizon={self.horizon_steps}, max_joint_step={self.max_joint_step:.3f}, '
            f'orientation_weight={self.orientation_weight:.3f}, '
            f'terminal_orientation_weight={self.terminal_orientation_weight:.3f}, '
            f'closed_loop=True, '
            f'command_fk_step_warn={self.command_fk_step_warn_m * 1000.0:.1f}mm')
        recommended_max_step = RECOMMENDED_CONTROLLER_MAX_VELOCITY_RAD_S * self.dt
        if self.max_joint_step > recommended_max_step + 1e-9:
            self.get_logger().warn(
                'MPC max_joint_step 应与控制器 max_velocity 匹配: '
                'max_joint_step <= max_velocity * dt。'
                f'当前 dt={self.dt:.3f}s, max_joint_step={self.max_joint_step:.3f}rad; '
                '若 student_arm max_velocity:=1.0，建议 max_joint_step:=0.02。')

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

    def _target_horizon_base(self):
        times = self.t + self.dt * np.arange(self.horizon_steps)
        positions = np.array([self._ee_target_base(float(t)) for t in times])
        rotations = np.array([self._target_rotation_base(float(t)) for t in times])
        return positions, rotations

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
        q_for_marker = self.q_measured if self.q_measured is not None else self.q_last
        if q_for_marker is None:
            return
        base_pos_world = self._base_pose(self.t)
        base_rot_world = self._base_rotation(self.t)
        ee_pos_base, _ = self.mpc_ik.forward(q_for_marker)
        ee_pos_world = base_pos_world + base_rot_world @ ee_pos_base
        ee_target_pos_world = self._ee_target_world(self.t)
        self._append_point_and_publish(self.ee_traj_marker, self.actual_marker_pub, ee_pos_world)
        self._append_point_and_publish(self.ee_target_traj_marker, self.target_marker_pub, ee_target_pos_world)

    def _command_fk_world(self, q: np.ndarray, t: float) -> np.ndarray:
        base_pos_world = self._base_pose(t)
        base_rot_world = self._base_rotation(t)
        ee_pos_base, _ = self.mpc_ik.forward(q)
        return base_pos_world + base_rot_world @ ee_pos_base

    def _command_fk_step(self, q: np.ndarray) -> float:
        command_fk_world = self._command_fk_world(q, self.t)
        if self.last_command_fk_world is None:
            return 0.0
        return float(np.linalg.norm(command_fk_world - self.last_command_fk_world))

    def _update_command_fk_monitor(self, q: np.ndarray):
        if self.phase != 'track':
            return
        command_fk_world = self._command_fk_world(q, self.t)
        if self.last_command_fk_world is not None:
            step = float(np.linalg.norm(command_fk_world - self.last_command_fk_world))
            if step > self.command_fk_step_warn_m:
                phase_deg = (360.0 * self.t / traj.EE_PERIOD_SEC) % 360.0
                self.get_logger().warn(
                    'Command FK 单步变化偏大: '
                    f'step={step * 1000.0:.2f} mm, phase={phase_deg:.1f} deg',
                    throttle_duration_sec=1.0,
                )
        self.last_command_fk_world = command_fk_world

    def _warn_joint_limits(self, q: np.ndarray):
        margin = np.deg2rad(JOINT_LIMIT_WARN_MARGIN_DEG)
        lower_gap = q - self.mpc_ik.q_lower
        upper_gap = self.mpc_ik.q_upper - q
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
        if dq is None:
            msg.velocity = [0.0] * JOINT_COUNT
        else:
            dq_arr = np.asarray(dq, dtype=float).reshape(JOINT_COUNT)
            if not np.all(np.isfinite(dq_arr)):
                self.get_logger().warn(
                    'MPC 发布速度包含 NaN/Inf，非法项改为 0',
                    throttle_duration_sec=2.0,
                )
                dq_arr = np.where(np.isfinite(dq_arr), dq_arr, 0.0)
            velocity_limit = self.max_joint_step / self.dt
            msg.velocity = np.clip(dq_arr, -velocity_limit, velocity_limit).tolist()
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

    def _warn_missing_joint_states(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_feedback_warn_time < 2.0:
            return
        self.last_feedback_warn_time = now_sec
        self.get_logger().warn(
            '还没有收到合法 /joint_states 反馈，复位阶段会等待。'
            '请确认 /joint_states 中包含 joint1 到 joint7。',
            throttle_duration_sec=2.0,
        )

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
                self.warm_start = None
                self.last_command_fk_world = None
                self.get_logger().info('机械臂已到达轨迹起始点，开始 MPC 轨迹跟踪')
        else:
            self.reset_hold_time = 0.0

    def _run_tracking_phase(self):
        # 纯闭环:reset 阶段已确保 q_measured 到位,这里始终用实测反馈作为求解起点。
        q_current = self.q_measured.copy()
        targets, rotations = self._target_horizon_base()
        q_next, ok, info = self.mpc_ik.step(
            targets,
            q_current,
            warm_start=self.warm_start,
            target_rotation=rotations,
        )

        command_usable = bool(info.get('command_usable', ok))
        tracking_ok = bool(info.get('tracking_ok', ok))
        dq_cmd = np.zeros(JOINT_COUNT)
        if command_usable:
            command_fk_step = self._command_fk_step(q_next)
            if (
                self.last_command_fk_world is not None
                and command_fk_step > self.command_fk_step_warn_m
            ):
                phase_deg = (360.0 * self.t / traj.EE_PERIOD_SEC) % 360.0
                self.get_logger().warn(
                    'MPC 命令会造成 command FK 突变，保持上一帧: '
                    f'step={command_fk_step * 1000.0:.2f} mm, phase={phase_deg:.1f} deg',
                    throttle_duration_sec=1.0,
                )
                dq_cmd = np.zeros(JOINT_COUNT)
                self.warm_start = None
            else:
                self.q_last = q_next
                dq_cmd = (q_next - q_current) / self.dt
                self.warm_start = info['trajectory']
            if not tracking_ok:
                ori_text = ''
                if info.get('terminal_orientation_error') is not None:
                    ori_text = f", orientation_error={info['terminal_orientation_error']:.4f}"
                self.get_logger().warn(
                    'MPC IK 跟踪误差超过阈值，继续发布可行命令: '
                    f"terminal_error={info['terminal_error']:.4f}, status={info['status']}, "
                    f"message={info['message']}{ori_text}",
                    throttle_duration_sec=2.0,
                )
        else:
            ori_text = ''
            if info.get('terminal_orientation_error') is not None:
                ori_text = f", orientation_error={info['terminal_orientation_error']:.4f}"
            self.get_logger().warn(
                'MPC IK 命令不可用，保持上一帧: '
                f"terminal_error={info['terminal_error']:.4f}, status={info['status']}, "
                f"message={info['message']}, plan_feasible={info.get('plan_feasible')}{ori_text}",
                throttle_duration_sec=2.0,
            )
            self.warm_start = None

        self._warn_joint_limits(self.q_last)
        self._update_command_fk_monitor(self.q_last)
        self._publish_joint_positions(self.q_last, dq_cmd)
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
    node = MoveBaseCircleMpcIkDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
