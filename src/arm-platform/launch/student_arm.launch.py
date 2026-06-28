from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    # config/ 目录内容被安装到 share/manipulator/ 根目录
    urdf_path = os.path.join(pkg_share, 'arm.urdf')
    rviz_config_path = os.path.join(pkg_share, 'student_arm.rviz')
    # 真机 (a_l1) 初始化必须使用 InitFromConfig, 需要电机/机械臂配置文件
    motor_config_path = os.path.join(pkg_share, 'motor_config.yaml')
    arm_config_path = os.path.join(pkg_share, 'arm_config.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    ###
    # 可改参数
    ###
    # 机械臂类型: sim(虚拟臂, 默认) 或 a_l1(真实硬件)
    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='sim',
        description='Arm type: sim (virtual) or a_l1 (real hardware)'
    )

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyTHS3',
        description='Serial port for real hardware (ignored in sim mode)'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='0.5',
        description='Max joint velocity (rad/s) safety limit for students'
    )

    enable_velocity_feedforward_arg = DeclareLaunchArgument(
        'enable_velocity_feedforward',
        default_value='false',
        description='Enable velocity feedforward plus position feedback tracking'
    )

    velocity_feedforward_gain_arg = DeclareLaunchArgument(
        'velocity_feedforward_gain',
        default_value='0.7',
        description='Joint velocity feedforward gain'
    )

    position_feedback_tau_arg = DeclareLaunchArgument(
        'position_feedback_tau',
        default_value='0.15',
        description='Position feedback time constant (s) for velocity feedforward mode'
    )

    feedback_velocity_limit_arg = DeclareLaunchArgument(
        'feedback_velocity_limit',
        default_value='0.4',
        description='Position feedback velocity clamp (rad/s)'
    )

    enable_acceleration_limit_arg = DeclareLaunchArgument(
        'enable_acceleration_limit',
        default_value='true',
        description='Enable velocity command acceleration limiting'
    )

    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='5.0',
        description='Velocity command acceleration limit (rad/s^2)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz for visualization'
    )

    ###
    # 机械臂控制节点
    ###
    student_arm_node = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        parameters=[
            {'arm_type': LaunchConfiguration('arm_type')},
            {'arm_version': 'gamma'},
            {'port_name': LaunchConfiguration('port_name')},
            {'motor_config_path': motor_config_path},
            {'arm_config_path': arm_config_path},
            {'max_velocity': LaunchConfiguration('max_velocity')},
            {'enable_velocity_feedforward': LaunchConfiguration('enable_velocity_feedforward')},
            {'velocity_feedforward_gain': LaunchConfiguration('velocity_feedforward_gain')},
            {'position_feedback_tau': LaunchConfiguration('position_feedback_tau')},
            {'feedback_velocity_limit': LaunchConfiguration('feedback_velocity_limit')},
            {'enable_acceleration_limit': LaunchConfiguration('enable_acceleration_limit')},
            {'max_acceleration': LaunchConfiguration('max_acceleration')},
            {'command_timeout_sec': 1.0},
        ]
    )

    ###
    # ROS 官方的 TF 发布节点
    ###
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    ###
    # RViz 可视化节点
    ###
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        arm_type_arg,
        port_name_arg,
        max_velocity_arg,
        enable_velocity_feedforward_arg,
        velocity_feedforward_gain_arg,
        position_feedback_tau_arg,
        feedback_velocity_limit_arg,
        enable_acceleration_limit_arg,
        max_acceleration_arg,
        use_rviz_arg,
        student_arm_node,
        robot_state_publisher_node,
        rviz_node,
    ])
