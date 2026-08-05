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


# 6 关节实机构型用【水平】书写面（四个面平铺在基座平面上、笔轴朝下）。
#
# 这是 6 关节【专用】的布局，必须在 import 轨迹模块【之前】设好环境变量 ——
# demo/moving_base_trajectory_config.py 在模块级读取它。
# 默认值是 vertical（原竖直板），7 关节那版已验证基线依赖默认值，不要改默认。
#
# 为什么 6 关节要水平面：竖直板要求腕部产生【水平】笔轴，而 6 关节没有球腕
# （腕部只有 joint5 绕 z、joint6 绕 x）。实测站心附近 20 点：
#     竖直板   限位余量 0.126      sigma_min 0.0075
#     水平面   限位余量 0.47~0.52  sigma_min 0.081~0.143
# 7 关节有球腕不受此限，换成水平面反而从达标发散到 RMS 353.9mm。
os.environ.setdefault('MB_STATION_LAYOUT', 'horizontal')

# 【基座固定】。实物装在桌边、基座不动，whole 任务里那个解析圆周摆动
# （末端处 0.156 m/s，是画笔速度的 2.4 倍）是仿真里人为注入的鲁棒性
# 考验，阶段 A 不该背它。基座扰动抑制留给动捕接入后的阶段 B 单独做。
os.environ.setdefault('MB_BASE_RADIUS', '0.0')
os.environ.setdefault('MB_BASE_Z_AMP', '0.0')

# The 6-joint link6 endpoint has no 91 mm pen extension.  Its compensated
# top-station reference is therefore lowered so the physical endpoint remains
# in the same reachable workspace as the former link7 tool tip.
os.environ.setdefault('MB_TOP_STATION_HEIGHT', '0.215')

# hwtest 使用轨迹模块的标准切向速度 0.02 m/s（周期 12.57 s）。实机若需要
# 保守速度，可在启动前显式设 MB_HWTEST_SPEED=0.01。
os.environ.setdefault('MB_HWTEST_SPEED', '0.02')


def generate_launch_description():
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    pkg_prefix = get_package_prefix('manipulator')
    workspace_root = os.path.abspath(os.path.join(pkg_prefix, '..', '..'))
    # URDF 由 SMC 配置的 `urdf:` 键决定（见下方读取处）。
    #
    # ⚠️ 这里原来写死成 arm6_tool.urdf，而 runner 的 urdf_path 参数会【覆盖】
    # 配置里的 `urdf:` 键（whole_task_smc_ros_runner.py:120 是
    # `get_parameter("urdf_path").value or None`，launch 永远传非空）——
    # 结果是配置里那个键一直是死的，改它不生效。2026-08-03 发现：实物上并未
    # 安装笔架，而模型一直带着 arm6_tool 那段 91 mm 幻影工具（60 g，joint5
    # 重力前馈误差达其静摩擦的 67%）。改成由配置决定，让 `urdf:` 名副其实。
    # 调参用：允许外部指定配置文件。默认仍按下面的顺序查找。
    # 【调参必须跑真实 ROS 节点链】—— 离线代理会被优化器钻空子，
    # 实测同一份参数离线报 0.168mm、真实 ROS 是 116mm（差 700 倍），
    # 机制是重阻尼下 N 不再是真投影、零空间泄漏进任务空间放大 7 个数量级。
    smc_config_default = _first_existing_path(
        os.path.join(
            workspace_root, 'src', 'arm-platform', 'task_space_smc',
            'config', 'smc_params_hwtest.yaml'),
        os.path.join(
            pkg_share, 'task_space_smc', 'config', 'smc_params_hwtest.yaml'),
        os.path.join(
            pkg_prefix, 'lib', 'manipulator', 'task_space_smc',
            'config', 'smc_params_whole6.yaml'),
    )
    smc_config_path = os.environ.get('WINDYLAB_SMC_CONFIG') or smc_config_default
    rviz_config_path = os.path.join(pkg_share, 'task_space_smc_whole.rviz')
    motor_config_path = os.path.join(pkg_share, 'motor_config.yaml')
    arm_config_path = os.path.join(pkg_share, 'arm_config.yaml')
    with open(smc_config_path, 'r', encoding='utf-8') as config_file:
        smc_config = yaml.safe_load(config_file)
    # 同一个路径同时喂给仿真 plant、robot_state_publisher 和 SMC runner，
    # 三者必须一致，否则被控对象与控制器模型不同。
    urdf_path = os.path.join(pkg_share, f"{smc_config.get('urdf', 'arm6')}.urdf")
    if not os.path.exists(urdf_path):
        raise RuntimeError(f'配置指定的 URDF 不存在: {urdf_path}')
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()
    approach_offset = np.asarray(
        smc_config.get('approach', {}).get(
            'initial_joint_offset_rad', [0.0] * 6),
        dtype=float,
    )
    # 6 关节做 6D 任务时零空间秩为 0、已停用，nullspace.q_nominal 不再存在。
    # 起始位形由 initial_pose_rad 给出 —— 退化到零位是危险的，
    # 其雅可比 sigma_min = 0.00001（cond 326263），起步即奇异。
    init_pose = smc_config.get('initial_pose_rad')
    if init_pose is None or len(init_pose) != 6:
        raise RuntimeError(
            'smc_params_whole6.yaml 必须提供 initial_pose_rad（6 个角度）—— '
            '零空间已停用，没有其它来源，退化到零位会从奇异位形起步')
    initial_joint_positions = list(
        np.asarray(init_pose, dtype=float) + approach_offset)

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
    shape_arg = DeclareLaunchArgument(
        'trajectory_shape',
        default_value='hwtest',
        choices=['hwtest', 'whole'],
        description=(
            'hwtest = 首次上机的 +x 侧小圆（基座固定）；'
            'whole = 五工作站完整任务'),
    )
    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='200.0',
        description='Shared SMC and student_arm_node control-loop rate.',
    )
    base_state_mode_arg = DeclareLaunchArgument(
        'base_state_mode',
        default_value='analytic',
        choices=['analytic', 'vicon'],
        description=(
            'analytic keeps the fixed/analytic base; vicon consumes '
            '/floating_base/vicon in hardware mode and blocks torque output '
            'until Vicon feedback is fresh.'),
    )
    external_base_topic_arg = DeclareLaunchArgument(
        'external_base_topic',
        default_value='/floating_base/vicon',
        description='FloatingBaseState topic used when base_state_mode=vicon.',
    )

    mode = ParameterValue(LaunchConfiguration('mode'), value_type=str)
    trajectory_shape = ParameterValue(
        LaunchConfiguration('trajectory_shape'), value_type=str)
    duration = ParameterValue(LaunchConfiguration('duration_sec'), value_type=float)
    start_delay = ParameterValue(
        LaunchConfiguration('start_delay_sec'), value_type=float)
    hardware_enabled = ParameterValue(LaunchConfiguration('hardware_enabled'), value_type=bool)
    control_rate = ParameterValue(
        LaunchConfiguration('control_rate_hz'), value_type=float)
    base_state_mode = ParameterValue(
        LaunchConfiguration('base_state_mode'), value_type=str)
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
            'arm_version': 'gamma6',
            'motor_config_path': motor_config_path,
            'arm_config_path': arm_config_path,
            'urdf_path': urdf_path,
            'initial_joint_positions': initial_joint_positions,
            'enable_direct_effort_command': True,
            'step_on_effort_command': True,
            'publish_joint_feedback': True,
            'effort_command_timeout_sec': 0.05,
            'control_rate_hz': control_rate,
            'joint_lower_limits': [-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0],
            'joint_upper_limits': [4.0, 2.0, 1.5708, 4.0, 2.0, 4.0],
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
            'trajectory_shape': trajectory_shape,
            'duration_sec': duration,
            'start_delay_sec': start_delay,
            'hardware_enabled': hardware_enabled,
            'data_dir': LaunchConfiguration('data_dir'),
            'initial_joint_positions': initial_joint_positions,
            'control_rate_hz_override': control_rate,
            'base_state_mode': base_state_mode,
            'external_base_topic': LaunchConfiguration('external_base_topic'),
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
        shape_arg,
        duration_arg,
        start_delay_arg,
        hardware_arg,
        data_dir_arg,
        rviz_arg,
        control_rate_arg,
        base_state_mode_arg,
        external_base_topic_arg,
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
