"""Isolated, repeatable 6DOF simulation launch.

The physical launch intentionally uses a physical reset pose and handover
settings.  This launch instead starts TorqueSimArm at the analytical track
start in the selected profile of ``smc_params_whole6.yaml`` and disables
physical-only handover blending, so a default-duration run is exactly one
trajectory.
"""

import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Keep this launch geometrically identical to task_space_smc_whole6.launch.py.
# The trajectory module reads these values while the runner process imports it.
os.environ.setdefault("MB_STATION_LAYOUT", "horizontal")
os.environ.setdefault("MB_BASE_RADIUS", "0.0")
os.environ.setdefault("MB_BASE_Z_AMP", "0.0")
os.environ.setdefault("MB_TOP_STATION_HEIGHT", "0.215")
os.environ.setdefault("MB_HWTEST_SPEED", "0.02")


def _first_existing_path(*paths):
    for path in paths:
        if os.path.exists(path):
            return path
    return paths[0]


def _setup(context):
    pkg_share = FindPackageShare("manipulator").find("manipulator")
    pkg_prefix = get_package_prefix("manipulator")
    workspace_root = os.path.abspath(os.path.join(pkg_prefix, "..", ".."))
    controller_path = _first_existing_path(
        os.path.join(workspace_root, "src", "arm-platform", "task_space_smc",
                     "config", "smc_params_whole6.yaml"),
        os.path.join(pkg_prefix, "lib", "manipulator", "task_space_smc",
                     "config", "smc_params_whole6.yaml"),
    )
    with open(controller_path, "r", encoding="utf-8") as f:
        controller_config = yaml.safe_load(f) or {}
    profiles = controller_config.get("simulation_profiles", {}) or {}
    shape = LaunchConfiguration("trajectory_shape").perform(context)
    if shape not in profiles:
        raise RuntimeError("unknown 6DOF simulation shape: %s" % shape)
    profile = profiles[shape]
    initial_q = np.asarray(
        profile["initial_joint_positions_rad"], dtype=float).reshape(6).tolist()
    urdf_path = os.path.join(pkg_share, "%s.urdf" % controller_config["urdf"])
    if not os.path.exists(urdf_path):
        raise RuntimeError("simulation URDF does not exist: %s" % urdf_path)
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    control_rate = ParameterValue(
        LaunchConfiguration("control_rate_hz"), value_type=float)
    student = Node(
        package="manipulator", executable="student_arm_node", name="student_arm_node",
        output="screen", parameters=[{
            "arm_type": "torque_sim", "arm_version": "gamma6",
            "motor_config_path": os.path.join(pkg_share, "motor_config.yaml"),
            "arm_config_path": os.path.join(pkg_share, "arm_config.yaml"),
            "urdf_path": urdf_path, "initial_joint_positions": initial_q,
            "enable_direct_effort_command": True, "step_on_effort_command": True,
            "publish_joint_feedback": True, "effort_command_timeout_sec": 0.05,
            "control_rate_hz": control_rate,
            "joint_lower_limits": [-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0],
            "joint_upper_limits": [4.0, 2.0, 1.5708, 4.0, 2.0, 4.0],
        }])
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description}])
    runner = Node(
        package="manipulator", executable="whole_task_smc_ros_runner.py",
        name="whole_task_smc_ros_runner", output="screen", parameters=[{
            "config_path": controller_path, "urdf_path": urdf_path,
            "mode": "simulation", "trajectory_shape": shape,
            "duration_sec": ParameterValue(
                LaunchConfiguration("duration_sec"), value_type=float),
            "start_delay_sec": ParameterValue(
                LaunchConfiguration("start_delay_sec"), value_type=float),
            "hardware_enabled": False,
            "data_dir": LaunchConfiguration("data_dir"),
            "initial_joint_positions": initial_q,
            "control_rate_hz_override": control_rate,
            "simulation_handover_enabled": False,
            "simulation_profile_name": shape,
        }])
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", os.path.join(pkg_share, "task_space_smc_whole.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")))
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(
                reason="whole-task SMC simulation runner exited"))])),
        student, rsp, runner, rviz,
    ]


def generate_launch_description():
    pkg_prefix = get_package_prefix("manipulator")
    workspace_root = os.path.abspath(os.path.join(pkg_prefix, "..", ".."))
    return LaunchDescription([
        DeclareLaunchArgument("trajectory_shape", default_value="hwtest",
                              choices=["hwtest", "whole"]),
        DeclareLaunchArgument("duration_sec", default_value="0.0"),
        # The simulated plant is already at the analytical trajectory start.
        # A physical-style approach segment is intentionally not required.
        DeclareLaunchArgument("start_delay_sec", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("control_rate_hz", default_value="200.0"),
        DeclareLaunchArgument(
            "data_dir", default_value=os.path.join(
                workspace_root, "src", "arm-platform", "task_space_smc", "data")),
        OpaqueFunction(function=_setup),
    ])
