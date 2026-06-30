#!/usr/bin/env python3
"""MPC-style Pinocchio IK trajectory demo.

The node publishes JointState commands to /student/joint_command. It supports a
vertical circle, a horizontal circle, and a line trajectory, all tracked with a
short-horizon MPC IK solver. Orientation soft constraints are optional and off
by default.
"""

import math

from builtin_interfaces.msg import Time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
from pinocchio_ik import PinocchioIK

JOINT_COUNT = 7
PUBLISH_RATE_HZ = 50.0
# 闭环复位阶段:先把真实臂驱动到轨迹起点,确认到位后再开始 MPC 跟踪。
RESET_TOLERANCE_RAD = math.radians(2.0)
RESET_HOLD_SEC = 1.0

CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
CIRCLE_RADIUS = 0.08
CIRCLE_PERIOD_SEC = 8.0

HORIZONTAL_CIRCLE_CENTER = np.array([0.35, 0.0, 0.15])
HORIZONTAL_CIRCLE_RADIUS = 0.08
HORIZONTAL_CIRCLE_PERIOD_SEC = 8.0

POINT_A = np.array([0.35, -0.15, 0.15])
POINT_B = np.array([0.35, 0.15, 0.25])
LINE_PERIOD_SEC = 6.0


class MoveArmMpcIkDemo(Node):
    def __init__(self, default_trajectory_mode: str = "circle", node_name: str = "move_arm_mpc_ik_demo"):
        super().__init__(node_name)
        # 默认值取自 tune_unified_mpc_params.py 的统一闭环调参结果
        # (固定臂+浮动基座臂四场景统一): h=2, step=0.02, ow=0.375, tow=5.0。
        # 注意 max_joint_step <= max_velocity*dt = 1.0*0.02 = 0.02 (角速度控制默认开启)。
        self.declare_parameter("trajectory_mode", default_trajectory_mode)
        self.declare_parameter("terminal_tolerance", 0.01)
        self.declare_parameter("solver_backend", "acados")
        self.declare_parameter("horizon_steps", 2)
        self.declare_parameter("max_joint_step", 0.02)
        self.declare_parameter("lock_orientation", False)
        self.declare_parameter("orientation_target", "initial")
        self.declare_parameter("orientation_weight", 0.375)
        self.declare_parameter("terminal_orientation_weight", 5.0)

        self.trajectory_mode = self.get_parameter("trajectory_mode").value
        self.terminal_tolerance = float(self.get_parameter("terminal_tolerance").value)
        self.solver_backend = self.get_parameter("solver_backend").value
        self.horizon_steps = int(self.get_parameter("horizon_steps").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.lock_orientation = bool(self.get_parameter("lock_orientation").value)
        self.orientation_target = self.get_parameter("orientation_target").value
        self.orientation_weight = float(self.get_parameter("orientation_weight").value)
        self.terminal_orientation_weight = float(self.get_parameter("terminal_orientation_weight").value)
        if self.trajectory_mode not in ("circle", "horizontal_circle", "line"):
            raise ValueError("trajectory_mode must be 'circle', 'horizontal_circle' or 'line'")
        if self.solver_backend not in ("casadi", "acados", "scipy"):
            raise ValueError("solver_backend must be 'casadi', 'acados' or 'scipy'")
        if self.orientation_target not in ("initial", "vertical_down"):
            raise ValueError("orientation_target must be 'initial' or 'vertical_down'")
        if self.horizon_steps < 1:
            raise ValueError("horizon_steps must be >= 1")
        if self.max_joint_step <= 0.0:
            raise ValueError("max_joint_step must be > 0")

        self.pub = self.create_publisher(JointState, "/student/joint_command", 10)
        # 闭环:始终用 /joint_states 实测关节角作为求解起点。
        self.create_subscription(JointState, "/joint_states", self._joint_states_callback, 10)
        self.dt = 1.0 / PUBLISH_RATE_HZ
        self.t = 0.0
        self.trajectory_start_time_sec = None

        config = MpcIkConfig(
            horizon_steps=self.horizon_steps,
            control_dt=self.dt,
            max_joint_step=self.max_joint_step,
            success_tolerance=self.terminal_tolerance,
            solver_backend=self.solver_backend,
            orientation_weight=self.orientation_weight if self.lock_orientation else 0.0,
            terminal_orientation_weight=self.terminal_orientation_weight if self.lock_orientation else 0.0,
        )
        self.mpc_ik = MpcPinocchioIK(config=config)
        self.seed_ik = PinocchioIK()
        self.warm_start = None

        p0 = self._target(0.0)
        q0, ok = self.seed_ik.solve(p0)
        if not ok:
            raise RuntimeError(f"起始点 {p0.tolist()} IK 不可达, 请调整轨迹参数")
        self.q_last = q0
        self.locked_rotation = self._orientation_target_rotation(q0)
        if self.lock_orientation and self.orientation_target != "initial":
            q_pose, pose_ok, pose_info = self.mpc_ik.solve(
                p0,
                q_init=q0,
                target_rotation=self.locked_rotation,
            )
            if bool(pose_info.get("command_usable", pose_ok)):
                self.q_last = q_pose
                self.warm_start = pose_info["trajectory"]
            else:
                self.get_logger().warn(
                    "起始姿态目标没有完全收敛, 先使用位置 IK 姿态启动: "
                    f"terminal_error={pose_info['terminal_error']:.4f}, "
                    f"orientation_error={pose_info.get('terminal_orientation_error')}"
                )

        # 闭环状态:reset 阶段把臂驱动到轨迹起点,到位后切 track 用实测反馈跟踪。
        self.q_measured = None
        self.phase = "reset"
        self.reset_q_target = q0.copy()
        self.reset_hold_time = 0.0
        self.last_feedback_warn_time = 0.0

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"MPC IK {self.trajectory_mode} trajectory, solver={config.solver_backend}, "
            f"horizon={config.horizon_steps}, "
            f"dt={config.control_dt:.3f}s, max step={config.max_joint_step:.3f} rad, "
            f"lock_orientation={self.lock_orientation}, "
            f"orientation_target={self.orientation_target}, "
            f"closed_loop=True"
        )

    def _orientation_target_rotation(self, q0: np.ndarray) -> np.ndarray:
        if self.orientation_target == "vertical_down":
            # 末端 x 轴对齐世界 -Z,y 轴对齐世界 +Y,z 轴对齐世界 +X。
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

    def _target(self, t: float) -> np.ndarray:
        if self.trajectory_mode == "circle":
            w = 2.0 * math.pi / CIRCLE_PERIOD_SEC
            return CIRCLE_CENTER + CIRCLE_RADIUS * np.array(
                [0.0, math.cos(w * t), math.sin(w * t)]
            )
        if self.trajectory_mode == "horizontal_circle":
            w = 2.0 * math.pi / HORIZONTAL_CIRCLE_PERIOD_SEC
            return HORIZONTAL_CIRCLE_CENTER + HORIZONTAL_CIRCLE_RADIUS * np.array(
                [math.cos(w * t), math.sin(w * t), 0.0]
            )

        s = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / LINE_PERIOD_SEC))
        return POINT_A + s * (POINT_B - POINT_A)

    def _target_horizon(self) -> np.ndarray:
        # 这里发布的是位置命令,没有单独的低层动力学延迟模型。预测时域第一步
        # 对齐当前目标,避免可视化/记录时把一拍预瞄表现成末端相位误差。
        times = self.t + self.dt * np.arange(self.mpc_ik.config.horizon_steps)
        return np.array([self._target(float(t)) for t in times])

    def _joint_states_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        joint_names = [f"joint{i + 1}" for i in range(JOINT_COUNT)]
        if not all(name in name_to_pos for name in joint_names):
            return
        self.q_measured = np.array([name_to_pos[name] for name in joint_names], dtype=float)

    def _publish_command(self, q: np.ndarray, dq: np.ndarray, use_trajectory_stamp: bool):
        msg = JointState()
        # Stamp the command with the trajectory time it represents, not merely
        # the wall-clock callback time. The visualizer can then compare expected
        # and actual at the same trajectory timestamp instead of inheriting ROS
        # callback jitter as apparent tracking error.
        if use_trajectory_stamp:
            msg.header.stamp = self._trajectory_stamp(self.t)
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint{i + 1}" for i in range(JOINT_COUNT)]
        msg.position = q.tolist()
        msg.velocity = dq.tolist()
        self.pub.publish(msg)

    def _warn_missing_feedback(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_feedback_warn_time < 2.0:
            return
        self.last_feedback_warn_time = now_sec
        self.get_logger().warn(
            "尚未收到包含 joint1~joint7 的 /joint_states 反馈, 复位阶段等待中。",
            throttle_duration_sec=2.0,
        )

    def tick(self):
        if self.phase == "reset":
            self._run_reset_phase()
            return
        self._run_tracking_phase()

    def _run_reset_phase(self):
        # 持续发布轨迹起点, 把真实臂驱动到位; 到位并保持后切换到跟踪。
        self._publish_command(self.reset_q_target, np.zeros(JOINT_COUNT), use_trajectory_stamp=False)
        if self.q_measured is None:
            self._warn_missing_feedback()
            return
        if np.all(np.abs(self.q_measured - self.reset_q_target) <= RESET_TOLERANCE_RAD):
            self.reset_hold_time += self.dt
            if self.reset_hold_time >= RESET_HOLD_SEC:
                self.phase = "track"
                self.q_last = self.reset_q_target.copy()
                self.t = 0.0
                self.trajectory_start_time_sec = None
                self.get_logger().info("机械臂已到达轨迹起始点, 开始闭环 MPC 跟踪")
        else:
            self.reset_hold_time = 0.0

    def _run_tracking_phase(self):
        if self.q_measured is None:
            self._warn_missing_feedback()
            return
        targets = self._target_horizon()
        target_rotation = self.locked_rotation if self.lock_orientation else None
        q_current = self.q_measured.copy()
        q_next, ok, info = self.mpc_ik.step(
            targets,
            q_current,
            warm_start=self.warm_start,
            target_rotation=target_rotation,
        )
        self.warm_start = info["trajectory"]

        command_usable = bool(info.get("command_usable", ok))
        tracking_ok = bool(info.get("tracking_ok", ok))
        dq_cmd = np.zeros(JOINT_COUNT)
        if command_usable:
            self.q_last = q_next
            dq_cmd = (q_next - q_current) / self.dt
            if not tracking_ok:
                ori_text = ""
                if info.get("terminal_orientation_error") is not None:
                    ori_text = f", orientation_error={info['terminal_orientation_error']:.4f}"
                self.get_logger().warn(
                    "MPC IK 跟踪误差超过阈值, 继续发布可行命令: "
                    f"terminal_error={info['terminal_error']:.4f}, status={info['status']}, "
                    f"message={info['message']}{ori_text}",
                    throttle_duration_sec=2.0,
                )
        else:
            ori_text = ""
            if info.get("terminal_orientation_error") is not None:
                ori_text = f", orientation_error={info['terminal_orientation_error']:.4f}"
            self.get_logger().warn(
                "MPC IK 命令不可用, 保持上一帧: "
                f"terminal_error={info['terminal_error']:.4f}, status={info['status']}, "
                f"message={info['message']}, plan_feasible={info.get('plan_feasible')}{ori_text}",
                throttle_duration_sec=2.0,
            )

        self._publish_command(self.q_last, dq_cmd, use_trajectory_stamp=True)
        self.t += self.dt

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
    node = MoveArmMpcIkDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
