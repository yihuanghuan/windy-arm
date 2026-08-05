from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import yaml

if __package__ in (None, ""):


    _here = Path(__file__).resolve().parent
    _root = _here
    while (_root / "__init__.py").exists():
        _root = _root.parent
    sys.path = [p for p in sys.path if Path(p or ".").resolve() != _here]
    sys.path.insert(0, str(_root))

from task_space_smc.core.actuator_adapter import ActuatorAdapter, ActuatorLimits
from task_space_smc.core.base_state import RosFeedbackBaseState, SimulationInjectedBase, SinusoidSpec
from task_space_smc.core.model import DEFAULT_ARM_JOINT_NAMES, FloatingBaseArmModel
from task_space_smc.core.config_loader import controller_from_config, default_config_path
from task_space_smc.core.pose_error import PoseState, enforce_quaternion_sign_continuity
from task_space_smc.core.redundancy_resolution import resolve_acceleration

try:
    import rclpy
    from dummy_interface.msg import MotorControl
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except Exception:
    rclpy = None
    Node = object
    JointState = object
    MotorControl = object


class TaskSpaceSmcTorqueNode(Node):
    def __init__(self):
        super().__init__("task_space_smc_torque_controller")
        self.declare_parameter("config_path", default_config_path())
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("base_state_mode", "simulation_injection")
        self.declare_parameter("command_topic", "/student/joint_effort_command")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("current_unit", "torque_nm")
        self.declare_parameter("fixed_world_position", [0.0, 0.0, 0.0])
        self.declare_parameter("fixed_world_quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])

        config_path = self.get_parameter("config_path").value
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)
        urdf_path = self.get_parameter("urdf_path").value or None
        self.model = FloatingBaseArmModel(urdf_path=urdf_path)
        self.controller = controller_from_config(self.config)
        self.q = np.zeros(7)
        self.qd = np.zeros(7)
        self.last_joint_stamp = None
        self.initialized_reference = False
        self.q_ref_prev = None
        self.reference_position = np.asarray(self.get_parameter("fixed_world_position").value, dtype=float).reshape(3)
        self.reference_quat = np.asarray(self.get_parameter("fixed_world_quaternion_wxyz").value, dtype=float).reshape(4)

        limits = ActuatorLimits(
            torque=np.asarray(self.config["limits"]["torque"], dtype=float).reshape(7),
            torque_rate=np.asarray(self.config["limits"]["torque_rate"], dtype=float).reshape(7),
            current=np.asarray(self.config["limits"]["current"], dtype=float).reshape(7),
        )
        self.adapter = ActuatorAdapter(
            joint_names=list(DEFAULT_ARM_JOINT_NAMES),
            limits=limits,
            current_unit=self.get_parameter("current_unit").value,
            command_timeout_sec=float(self.config["safety"]["command_timeout_sec"]),
            joint_state_timeout_sec=float(self.config["safety"]["joint_state_timeout_sec"]),
            base_state_timeout_sec=float(self.config["safety"]["base_state_timeout_sec"]),
            hardware_enabled=bool(self.get_parameter("hardware_enabled").value),
        )

        self.base_mode = self.get_parameter("base_state_mode").value
        self.ros_base = RosFeedbackBaseState(float(self.config["safety"]["base_state_timeout_sec"]))
        self.sim_base = SimulationInjectedBase(
            SinusoidSpec.from_lists(
                self.config["simulation_injection"]["translation_amplitude_m"],
                self.config["simulation_injection"]["translation_frequency_hz"],
                self.config["simulation_injection"]["translation_phase_rad"],
            ),
            SinusoidSpec.from_lists(
                np.deg2rad(self.config["simulation_injection"]["rpy_amplitude_deg"]),
                self.config["simulation_injection"]["rpy_frequency_hz"],
                self.config["simulation_injection"]["rpy_phase_rad"],
            ),
        )

        self.pub = self.create_publisher(MotorControl, self.get_parameter("command_topic").value, 10)
        self.sub_joint = self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        period = 1.0 / float(self.config["control_rate_hz"])
        self.timer = self.create_timer(period, self._control_loop)
        self.get_logger().warn(
            "Publishing MotorControl to /student/joint_effort_command; existing hardware nodes do not subscribe to this topic yet."
        )

    def _joint_state_cb(self, msg):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(DEFAULT_ARM_JOINT_NAMES):
            if name not in name_to_index:
                return
            i = name_to_index[name]
            self.q[j] = msg.position[i]
            self.qd[j] = msg.velocity[i] if i < len(msg.velocity) else 0.0
        self.last_joint_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _base_state(self, now_sec: float):
        if self.base_mode == "simulation_injection":
            return self.sim_base.sample(now_sec)
        return self.ros_base.latest(now_sec)

    def _control_loop(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        try:
            bs = self._base_state(now_sec)
            q_full = self.model.make_q_full(bs.position_w, bs.quaternion_wxyz, self.q)
            v_full = self.model.make_v_full(bs.velocity_lwa, self.qd)
            kin = self.model.compute_kinematics(q_full, v_full)
            if not self.initialized_reference:
                self.reference_position = kin.ee_position.copy()
                self.reference_quat = kin.ee_quaternion_wxyz.copy()
                self.initialized_reference = True
            self.reference_quat = enforce_quaternion_sign_continuity(self.reference_quat, self.q_ref_prev)
            self.q_ref_prev = self.reference_quat.copy()
            desired = PoseState(
                position=self.reference_position,
                quaternion_wxyz=self.reference_quat,
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
                linear_acceleration=np.zeros(3),
                angular_acceleration=np.zeros(3),
            )
            actual = PoseState(
                position=kin.ee_position,
                quaternion_wxyz=kin.ee_quaternion_wxyz,
                linear_velocity=kin.ee_velocity_lwa[:3],
                angular_velocity=kin.ee_velocity_lwa[3:],
            )
            smc = self.controller.compute(desired, actual)
            b6 = self.model.compute_task_bias_acceleration(q_full, v_full, bs.acceleration_lwa)
            rr = resolve_acceleration(
                kin.J_arm_lwa,
                self.model.compute_arm_mass_block(q_full),
                smc.nu - b6,
                self.q,
                self.qd,
                self.config["nullspace"]["q_nominal"],
                self.config["nullspace"]["kp"],
                self.config["nullspace"]["kd"],
                float(self.config["task"]["characteristic_length"]),
                float(self.config["singularity"]["sigma_warn"]),
                float(self.config["singularity"]["sigma_critical"]),
                float(self.config["singularity"]["damping_min"]),
                float(self.config["singularity"]["damping_max"]),
            )
            tau = self.model.compute_rnea_arm_torque(q_full, v_full, bs.acceleration_lwa, rr.qdd_cmd)
            motor = self.adapter.torque_to_motor_command(tau, now_sec, self.last_joint_stamp, bs.stamp)
        except Exception as exc:
            self.get_logger().warn(f"SMC command rejected: {exc}")
            motor = self.adapter.zero(list(DEFAULT_ARM_JOINT_NAMES), now_sec, str(exc))
        msg = MotorControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = self.q.tolist()
        msg.velocity = [0.0] * 7
        msg.current = motor.command.tolist()
        msg.p = [0.0] * 7
        msg.d = [0.0] * 7
        msg.enable = 1 if motor.enabled else 0
        self.pub.publish(msg)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("rclpy is not available")
    rclpy.init(args=args)
    node = TaskSpaceSmcTorqueNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
