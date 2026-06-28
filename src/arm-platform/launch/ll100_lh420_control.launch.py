from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    urdf_path = os.path.join(pkg_share, 'll100_lh420.urdf')
    rviz_config_path = os.path.join(pkg_share, 'student_arm.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz for LL100-LH420 control visualization'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='0.5',
        description='Max LL100 simulated arm joint velocity (rad/s)'
    )

    student_arm_node = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        parameters=[
            {'arm_type': 'll100_sim'},
            {'arm_version': 'll100_lh420'},
            {'max_velocity': LaunchConfiguration('max_velocity')},
            {'command_timeout_sec': 1.0},
            {'joint_lower_limits': [-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0]},
            {'joint_upper_limits': [4.0, 2.0, 1.5708, 4.0, 2.0, 4.0]},
            {'p_gain': [30.0, 30.0, 30.0, 5.0, 5.0, 5.0]},
            {'d_gain': [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ll100_lh420_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    rotor_joint_state_publisher_node = Node(
        package='manipulator',
        executable='ll100_lh420_rotor_joint_state_publisher.py',
        name='ll100_lh420_rotor_joint_state_publisher',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='ll100_lh420_rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        max_velocity_arg,
        student_arm_node,
        robot_state_publisher_node,
        rotor_joint_state_publisher_node,
        rviz_node,
    ])
