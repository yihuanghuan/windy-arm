"""6 自由度实机构型的学生接口 launch。

与 student_arm.launch.py 的区别（原 7 关节 launch 完全不动）:

  * arm_version = gamma6  -> arm_config.yaml 只创建 6 个电机,
    驱动层不再读取固件的第 7 通道。该通道是幽灵数据:
    position -6.5231 rad / velocity 27.9731 rad/s / temperature 1,
    位置超出 ±4 rad 限位, 速度超 RS05 上限 5.6 倍。
  * urdf_path = arm6.urdf -> 末端 frame 为 link6。相对原 link7
    末端位置前移 91.00 mm, 轨迹与工作空间需按新末端重新标定。
  * joint_lower/upper_limits 为 6 元素 -> student_arm_node 的
    joint_count_ = 6。若沿用 7 元素默认值, 6 元素的力矩指令会被
    ValidateEffortCommand 直接拒收。
  * control_rate_hz 默认 100 -> 位置模式的上限。实测板子上报速率为
    200.6 Hz (0x20/0x21/0x22/0x24 各 602 帧 / 3.00 s), 所以原来的
    500 Hz 在真机上有一半周期拿不到新反馈。但位置路径还有一道更紧的
    限制: ArmPlatform::ExecuteControlCycle 对 dt < 0.01 直接抛异常,
    即位置模式不得超过 100 Hz。力矩路径 ExecuteDirectMotorCommand
    没有这个检查, 启用 enable_direct_effort_command 后可提到 200。

真机串口: 本机通过 CP2102 USB-UART 桥接为 /dev/ttyUSB0;
装机到 NX 时按 SOP 为 /dev/ttyTHS3。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# 6 关节限位, 与 arm_config.yaml 的 a_l1_gamma6 保持一致
JOINT_LOWER_LIMITS = [-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0]
JOINT_UPPER_LIMITS = [4.0, 2.0, 1.5708, 4.0, 2.0, 4.0]


def generate_launch_description():
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    urdf_path = os.path.join(pkg_share, 'arm6.urdf')
    rviz_config_path = os.path.join(pkg_share, 'student_arm.rviz')
    motor_config_path = os.path.join(pkg_share, 'motor_config.yaml')
    arm_config_path = os.path.join(pkg_share, 'arm_config.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='sim',
        description='sim (虚拟 6 关节臂) 或 a_l1 (真机)',
    )

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='真机串口; 仿真模式忽略。装机到 NX 时为 /dev/ttyTHS3',
    )

    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='100.0',
        description=(
            '控制周期频率 (Hz)。位置模式硬上限 100 '
            '(ExecuteControlCycle 拒绝 dt < 0.01); '
            '力矩模式可到 200, 即板子实测上报速率 200.6 Hz'
        ),
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='0.5',
        description='关节最大速度 (rad/s)。标定阶段保持保守值',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='是否启动 RViz',
    )

    publish_joint_feedback_arg = DeclareLaunchArgument(
        'publish_joint_feedback',
        default_value='True',
        description='发布 /student/joint_feedback (标定需要读 0x22 力矩)',
    )

    enable_direct_effort_command_arg = DeclareLaunchArgument(
        'enable_direct_effort_command',
        default_value='False',
        description=(
            '启用 0x12 力矩前馈通路 (阶段 2 起需要)。默认关闭: '
            '标定前 0x12 命令方向的标度尚未经台架验证。'
        ),
    )

    step_on_effort_command_arg = DeclareLaunchArgument(
        'step_on_effort_command',
        default_value='False',
        description='收到力矩指令即步进一次控制周期 (而非定时器驱动)',
    )

    student_arm_node = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        parameters=[{
            'arm_type': LaunchConfiguration('arm_type'),
            'arm_version': 'gamma6',
            'port_name': LaunchConfiguration('port_name'),
            'motor_config_path': motor_config_path,
            'arm_config_path': arm_config_path,
            'urdf_path': urdf_path,
            'joint_lower_limits': JOINT_LOWER_LIMITS,
            'joint_upper_limits': JOINT_UPPER_LIMITS,
            'control_rate_hz': ParameterValue(
                LaunchConfiguration('control_rate_hz'), value_type=float),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'publish_joint_feedback': ParameterValue(
                LaunchConfiguration('publish_joint_feedback'), value_type=bool),
            # 力矩通路默认关闭 (见对应 DeclareLaunchArgument)。位置模式
            # 标定全程无需它; 阶段 2 起显式传 true 打开。
            'enable_direct_effort_command': ParameterValue(
                LaunchConfiguration('enable_direct_effort_command'),
                value_type=bool),
            'step_on_effort_command': ParameterValue(
                LaunchConfiguration('step_on_effort_command'),
                value_type=bool),
            'command_timeout_sec': 1.0,
            # 直驱力矩消息超时后的驱动兜底保位也叠加重力前馈。否则 joint2/3
            # 会在任何控制器退出或 ROS 短暂抖动后仅靠 PD 下垂。
            'gravity_hold_enabled': True,
            'gravity_hold_zero_offset_rad': [
                0.039576, -0.297745, -0.020218,
                -0.192988, -0.111667, 0.0,
            ],
            'gravity_hold_torque_limit': [9.0, 9.0, 9.0, 1.6, 1.6, 1.6],
        }],
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
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        arm_type_arg,
        port_name_arg,
        control_rate_arg,
        max_velocity_arg,
        use_rviz_arg,
        publish_joint_feedback_arg,
        enable_direct_effort_command_arg,
        step_on_effort_command_arg,
        student_arm_node,
        robot_state_publisher_node,
        rviz_node,
    ])
