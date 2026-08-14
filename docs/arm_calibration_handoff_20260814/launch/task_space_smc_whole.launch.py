import os

import numpy as np
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _first_existing_path(*paths):
    for path in paths:
        if os.path.exists(path):
            return path
    return paths[0]


def generate_launch_description():
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    pkg_prefix = get_package_prefix('manipulator')
    workspace_root = os.path.abspath(os.path.join(pkg_prefix, '..', '..'))
    urdf_path = os.path.join(pkg_share, 'arm.urdf')
    smc_config_path = _first_existing_path(
        os.path.join(
            workspace_root, 'src', 'arm-platform', 'task_space_smc',
            'config', 'smc_params_whole.yaml'),
        os.path.join(
            pkg_share, 'task_space_smc', 'config', 'smc_params_whole.yaml'),
        os.path.join(
            pkg_prefix, 'lib', 'manipulator', 'task_space_smc',
            'config', 'smc_params_whole.yaml'),
    )
    rviz_config_path = os.path.join(pkg_share, 'task_space_smc_whole.rviz')
    motor_config_path = os.path.join(pkg_share, 'motor_config.yaml')
    arm_config_path = os.path.join(pkg_share, 'arm_config.yaml')
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()
    with open(smc_config_path, 'r', encoding='utf-8') as config_file:
        smc_config = yaml.safe_load(config_file)
    q_nominal = np.asarray(
        smc_config['nullspace']['q_nominal'], dtype=float)
    approach_offset = np.asarray(
        smc_config.get('approach', {}).get(
            'initial_joint_offset_rad', [0.0] * 7),
        dtype=float,
    )
    initial_joint_positions = (q_nominal + approach_offset).tolist()

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='simulation',
        choices=['simulation', 'internal_simulation', 'hardware'],
        description=(
            'simulation uses student_arm_node/TorqueSimArm; '
            'internal_simulation keeps the legacy Python plant; '
            'hardware connects to an externally started hardware node.'
        ),
    )
    duration_arg = DeclareLaunchArgument(
        'duration_sec',
        default_value='0.0',
        description='0 means one whole-task period.',
    )
    start_delay_arg = DeclareLaunchArgument(
        'start_delay_sec',
        default_value='3.0',
        description='Delay before advancing the trajectory and enabling tracking.',
    )
    hardware_arg = DeclareLaunchArgument(
        'hardware_enabled',
        default_value='false',
        description='Enable nonzero MotorControl output from the SMC adapter.',
    )
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value=os.path.abspath(os.path.join(
            workspace_root, 'src', 'arm-platform',
            'task_space_smc', 'data')),
        description='Directory for timestamped trajectory CSV, summary, and plots.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Start RViz with the task-space SMC visualization.',
    )
    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='500.0',
        description='Shared SMC and student_arm_node control-loop rate.',
    )

    mode = ParameterValue(LaunchConfiguration('mode'), value_type=str)
    duration = ParameterValue(LaunchConfiguration('duration_sec'), value_type=float)
    start_delay = ParameterValue(
        LaunchConfiguration('start_delay_sec'), value_type=float)
    hardware_enabled = ParameterValue(LaunchConfiguration('hardware_enabled'), value_type=bool)
    control_rate = ParameterValue(
        LaunchConfiguration('control_rate_hz'), value_type=float)
    shared_sim_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'simulation'",
    ]))

    student_arm_node = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        condition=shared_sim_condition,
        parameters=[{
            'arm_type': 'torque_sim',
            'arm_version': 'gamma',
            'motor_config_path': motor_config_path,
            'arm_config_path': arm_config_path,
            'urdf_path': urdf_path,
            'initial_joint_positions': initial_joint_positions,
            'enable_direct_effort_command': True,
            'step_on_effort_command': True,
            'publish_joint_feedback': True,
            'effort_command_timeout_sec': 0.05,
            'control_rate_hz': control_rate,
            'joint_lower_limits': [
                -4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0, -4.0],
            'joint_upper_limits': [
                4.0, 2.0, 1.5708, 4.0, 2.0, 4.0, 4.0],
        }],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    smc_runner_node = Node(
        package='manipulator',
        executable='whole_task_smc_ros_runner.py',
        name='whole_task_smc_ros_runner',
        output='screen',
        parameters=[{
            'config_path': smc_config_path,
            'urdf_path': urdf_path,
            'mode': mode,
            'trajectory_shape': 'whole',
            'duration_sec': duration,
            'start_delay_sec': start_delay,
            'hardware_enabled': hardware_enabled,
            'data_dir': LaunchConfiguration('data_dir'),
            'initial_joint_positions': initial_joint_positions,
            'control_rate_hz_override': control_rate,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        mode_arg,
        duration_arg,
        start_delay_arg,
        hardware_arg,
        data_dir_arg,
        rviz_arg,
        control_rate_arg,
        RegisterEventHandler(
            OnProcessExit(
                target_action=smc_runner_node,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='whole-task SMC runner exited'))],
            )
        ),
        student_arm_node,
        robot_state_publisher_node,
        smc_runner_node,
        rviz_node,
    ])
