import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    ros_home = os.path.abspath(os.path.expanduser(
        os.environ.get('ROS_HOME', '~/.ros')))
    run_data_path = os.path.join(
        ros_home, 'moving_base_uncertainty')

    # config/ and launch/ are installed flat under share/manipulator by the
    # package's existing CMake install rules.
    parameters_path = os.path.join(
        pkg_share, 'moving_base_uncertainty.yaml')
    urdf_path = os.path.join(pkg_share, 'arm.urdf')
    rviz_config_path = os.path.join(
        pkg_share, 'moving_base_uncertainty.rviz')
    motor_config_path = os.path.join(pkg_share, 'motor_config.yaml')
    arm_config_path = os.path.join(pkg_share, 'arm_config.yaml')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    trajectory_shape_arg = DeclareLaunchArgument(
        'trajectory_shape',
        default_value='whole',
        choices=['circle', 'triangle', 'square', 'sine', 'windylab', 'whole'],
        description='World-frame end-effector drawing task',
    )
    disturbance_profile_arg = DeclareLaunchArgument(
        'base_disturbance_profile',
        default_value='gust',
        choices=['harmonic', 'gust', 'fault', 'wideband'],
        description='Physical base-disturbance profile',
    )
    belief_mode_arg = DeclareLaunchArgument(
        'base_belief_mode',
        default_value='model',
        choices=['nominal', 'truth', 'estimated', 'model'],
        description='Base-motion belief available to the NMPC',
    )

    trajectory_shape = ParameterValue(
        LaunchConfiguration('trajectory_shape'), value_type=str)
    disturbance_profile = ParameterValue(
        LaunchConfiguration('base_disturbance_profile'), value_type=str)
    belief_mode = ParameterValue(
        LaunchConfiguration('base_belief_mode'), value_type=str)

    student_arm_node = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        parameters=[
            parameters_path,
            {
                # These installed resource paths are deployment details, not
                # experiment choices, and intentionally override YAML fallbacks.
                'motor_config_path': motor_config_path,
                'arm_config_path': arm_config_path,
            },
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    visualizer_node = Node(
        package='manipulator',
        executable='trajectory_visualizer_demo.py',
        name='trajectory_visualizer_demo',
        output='screen',
        parameters=[
            parameters_path,
            {
                # The scenario selection is applied after the shared YAML so
                # controller and visualizer cannot silently use different truth.
                'trajectory_shape': trajectory_shape,
                'base_disturbance_profile': disturbance_profile,
                # Output placement is host infrastructure rather than an
                # algorithm choice. Keep it portable across users/installs.
                'data_dir': run_data_path,
            },
        ],
    )

    controller_node = Node(
        package='manipulator',
        executable='move_base_circle_mpc_ik_demo.py',
        name='move_base_circle_mpc_ik_demo',
        output='screen',
        parameters=[
            parameters_path,
            {
                'trajectory_shape': trajectory_shape,
                'base_disturbance_profile': disturbance_profile,
                'base_belief_mode': belief_mode,
                'loop_data_dir': run_data_path,
            },
        ],
    )

    return LaunchDescription([
        trajectory_shape_arg,
        disturbance_profile_arg,
        belief_mode_arg,
        student_arm_node,
        robot_state_publisher_node,
        rviz_node,
        visualizer_node,
        controller_node,
    ])
