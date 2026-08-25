"""实物 demo：基座读动捕，末端在世界系里定点保持。

场景是一句话：**基座随便动，末端不动。**

基座位姿由 Vicon 经 `vicon_floating_base_bridge` 送进来，末端参考量是一个与
时间无关的常量位姿 —— 于是末端在世界系里的每一点偏差都【只】来自基座运动没
补干净，没有轨迹跟踪误差混在里面。这是动捕接入后最直接的验收场景。

目标位姿缺省在接管的第一拍由【实测】末端位姿捕获，语义是"保持你现在所在的
位置"：不需要就位移动，第一拍的任务空间误差严格为零。

这条 launch 不起硬件驱动，也不起 Vicon 接收（缺省）。按 README 第 4 节的顺序：
  1) 起驱动（student_arm6.launch.py，起了就不要关）
  2) 把 joint5 挪出腕部奇异（wrist_prepose.py --target-deg 40）
  3) 起 Vicon 接收并确认数据（vicon_floating_base.launch.py）
  4) 起本 demo
"""

import os

import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# 轨迹模块在【模块级】读这些变量，必须在运行器 import 它之前设好。
# 与 task_space_smc_whole6.launch.py 保持几何一致。
os.environ.setdefault('MB_STATION_LAYOUT', 'horizontal')
# 基座由动捕给，解析摆动必须关掉，否则解析基座会叠在动捕基座上。
os.environ.setdefault('MB_BASE_RADIUS', '0.0')
os.environ.setdefault('MB_BASE_Z_AMP', '0.0')
os.environ.setdefault('MB_TOP_STATION_HEIGHT', '0.215')
# hold 没有周期，这只是 duration_sec:=0 时的缺省运行时长（秒）。
os.environ.setdefault('MB_HOLD_PERIOD', '60.0')


def _first_existing_path(*paths):
    for path in paths:
        if os.path.exists(path):
            return path
    return paths[0]


def _parse_hold_target(text: str):
    text = str(text).strip()
    if not text:
        return [0.0, 0.0, 0.0]
    values = [float(v) for v in text.replace(' ', '').split(',') if v]
    if len(values) != 3:
        raise RuntimeError(
            'hold_target_position_m 需要三个逗号分隔的世界系坐标 (m)，'
            f'收到 {text!r}')
    return values


def _setup(context):
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    pkg_prefix = get_package_prefix('manipulator')
    workspace_root = os.path.abspath(os.path.join(pkg_prefix, '..', '..'))

    # hold 是独立实物任务，必须固定使用专用实物参数。刻意不读取
    # WINDYLAB_SMC_CONFIG：遗留的仿真/调参环境变量不得改变上机控制器。
    smc_config_path = _first_existing_path(
        os.path.join(workspace_root, 'src', 'arm-platform', 'task_space_smc',
                     'config', 'smc_params_hold.yaml'),
        os.path.join(pkg_share, 'task_space_smc', 'config',
                     'smc_params_hold.yaml'),
    )
    with open(smc_config_path, 'r', encoding='utf-8') as f:
        smc_config = yaml.safe_load(f)
    urdf_path = os.path.join(pkg_share, "%s.urdf" % smc_config.get('urdf', 'arm6'))
    if not os.path.exists(urdf_path):
        raise RuntimeError(f'配置指定的 URDF 不存在: {urdf_path}')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    hold_target = _parse_hold_target(
        LaunchConfiguration('hold_target_position_m').perform(context))
    capture_measured = (
        LaunchConfiguration('hold_capture_measured').perform(context).lower()
        == 'true')
    if not capture_measured and not any(hold_target):
        raise RuntimeError(
            'hold_capture_measured:=false 时必须同时给出 '
            'hold_target_position_m:="x,y,z"')

    control_rate = ParameterValue(
        LaunchConfiguration('control_rate_hz'), value_type=float)

    vicon = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vicon_floating_base.launch.py')),
        launch_arguments={
            'server': LaunchConfiguration('vicon_server'),
            'port': LaunchConfiguration('vicon_port'),
            'pose_topic': LaunchConfiguration('vicon_pose_topic'),
            'output_topic': LaunchConfiguration('external_base_topic'),
            # 留空 = 用 task_space_smc/config/vicon_extrinsics.yaml 的标定值。
            'rigid_from_base_translation_m': LaunchConfiguration(
                'rigid_from_base_translation_m'),
            'rigid_from_base_quaternion_xyzw': LaunchConfiguration(
                'rigid_from_base_quaternion_xyzw'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_vicon')))

    # 驱动那条 launch 也起了一个 robot_state_publisher。这里再起一个，是为了
    # 让 TF 树用的 URDF 与【控制器模型】同源 —— 两者不一致时 RViz 看着对、
    # 控制律算的是另一副骨架。
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(LaunchConfiguration('start_robot_state_publisher')))

    runner = Node(
        package='manipulator', executable='whole_task_smc_ros_runner.py',
        name='whole_task_smc_ros_runner', output='screen', parameters=[{
            'config_path': smc_config_path,
            'urdf_path': urdf_path,
            'mode': 'hardware',
            'trajectory_shape': 'hold',
            'base_state_mode': 'vicon',
            'external_base_topic': LaunchConfiguration('external_base_topic'),
            'hold_capture_measured': capture_measured,
            'hold_target_position_m': hold_target,
            'duration_sec': ParameterValue(
                LaunchConfiguration('duration_sec'), value_type=float),
            # 保持目标就是当前位姿，没有要走的距离，就位段恒定取消。
            'start_delay_sec': 0.0,
            'hardware_enabled': ParameterValue(
                LaunchConfiguration('hardware_enabled'), value_type=bool),
            'data_dir': LaunchConfiguration('data_dir'),
            'control_rate_hz_override': control_rate,
        }])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_share, 'task_space_smc_whole.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return [
        RegisterEventHandler(OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(reason='hold demo runner exited'))])),
        vicon, rsp, runner, rviz,
    ]


def generate_launch_description():
    pkg_prefix = get_package_prefix('manipulator')
    workspace_root = os.path.abspath(os.path.join(pkg_prefix, '..', '..'))
    return LaunchDescription([
        # 缺省 false：和 task_space_smc_whole6.launch.py 一致 —— 不显式打开，
        # 适配器输出恒为零，机械臂不动。
        DeclareLaunchArgument(
            'hardware_enabled', default_value='false', choices=['true', 'false'],
            description='true 才允许适配器输出非零 MotorControl'),
        DeclareLaunchArgument(
            'duration_sec', default_value='60.0',
            description='保持时长，秒。0 = MB_HOLD_PERIOD（缺省 60）'),
        DeclareLaunchArgument(
            'hold_capture_measured', default_value='true',
            choices=['true', 'false'],
            description='true = 接管第一拍捕获实测末端位姿作为保持目标'),
        DeclareLaunchArgument(
            'hold_target_position_m', default_value='',
            description='hold_capture_measured:=false 时的世界系目标 "x,y,z"'),
        DeclareLaunchArgument(
            'external_base_topic', default_value='/floating_base/vicon',
            description='FloatingBaseState 话题，由 Vicon 桥发布'),
        DeclareLaunchArgument(
            'start_vicon', default_value='false', choices=['true', 'false'],
            description='是否顺带起 VRPN 接收和 Vicon 桥。'
                        '缺省 false：先单独验收动捕数据再接 SMC'),
        # 只有 start_vicon:=true 时才有意义；留空即用标定文件里的实测外参。
        DeclareLaunchArgument(
            'rigid_from_base_translation_m', default_value='',
            description='覆盖 arm_base->base_link 平移 "x,y,z" (m)'),
        DeclareLaunchArgument(
            'rigid_from_base_quaternion_xyzw', default_value='',
            description='覆盖 arm_base->base_link 四元数 "x,y,z,w"'),
        DeclareLaunchArgument('vicon_server', default_value='192.168.200.60'),
        DeclareLaunchArgument('vicon_port', default_value='3883'),
        DeclareLaunchArgument(
            'vicon_pose_topic', default_value='/vrpn/arm_base/pose',
            description='基座刚体的 VRPN 话题。刚体名里的空格在话题里是下划线'),
        DeclareLaunchArgument('control_rate_hz', default_value='200.0'),
        # 缺省 false：实物跑 SMC 时 RViz 和 200 Hz 控制环抢 CPU。
        DeclareLaunchArgument('rviz', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_robot_state_publisher',
                              default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'data_dir', default_value=os.path.join(
                workspace_root, 'src', 'arm-platform', 'task_space_smc', 'data')),
        OpaqueFunction(function=_setup),
    ])
