from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='False',
        description='Use joint_state_publisher_gui to inspect movable joints if installed'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz for visualization'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ll100_lh420_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='ll100_lh420_joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    zero_joint_state_publisher_node = Node(
        package='manipulator',
        executable='ll100_lh420_zero_joint_state_publisher.py',
        name='ll100_lh420_zero_joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
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
        use_gui_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        zero_joint_state_publisher_node,
        rviz_node,
    ])
