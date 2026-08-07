"""Start the VRPN receiver and expose one Vicon rigid body as base state.

The ``arm_base -> base_link`` extrinsic is read from
``task_space_smc/config/vicon_extrinsics.yaml`` (measured 2026-08-05) and pushed
into the bridge.  It is not cosmetic: its rotation is 85.3 deg, and with
``zero_on_first_pose`` the bridge defines the controller's ``world`` from the
first pose it sees.  Left at identity, that world would be the *marker
cluster's* frame instead of ``base_link``'s -- 85 deg off the robot.
"""

import os

import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _first_existing_path(*paths):
    for path in paths:
        if os.path.exists(path):
            return path
    return paths[0]


def extrinsics_path():
    pkg_share = FindPackageShare("manipulator").find("manipulator")
    workspace_root = os.path.abspath(
        os.path.join(get_package_prefix("manipulator"), "..", ".."))
    # task_space_smc/ is installed to lib/manipulator (a copy), not to share.
    return _first_existing_path(
        os.path.join(workspace_root, "src", "arm-platform", "task_space_smc",
                     "config", "vicon_extrinsics.yaml"),
        os.path.join(get_package_prefix("manipulator"), "lib", "manipulator",
                     "task_space_smc", "config", "vicon_extrinsics.yaml"),
        os.path.join(pkg_share, "task_space_smc", "config",
                     "vicon_extrinsics.yaml"),
    )


def load_base_extrinsic():
    """``arm_base -> base_link`` as (translation_m, quaternion_xyzw).

    A missing file or missing keys is a hard error, never a silent identity: an
    identity extrinsic is a *wrong* world frame that still runs, which is
    exactly the kind of fault that only shows up on the robot.
    """
    path = extrinsics_path()
    if not os.path.exists(path):
        raise RuntimeError(
            "找不到 Vicon 外参文件 %s。它由 "
            "task_space_smc/hw/vicon_base_extrinsic_calibrate.py 标定产生。" % path)
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    block = data.get("arm_base_from_base_link") or {}
    translation = block.get("translation_m")
    quaternion = block.get("quaternion_xyzw")
    if (not isinstance(translation, list) or len(translation) != 3
            or not isinstance(quaternion, list) or len(quaternion) != 4):
        raise RuntimeError(
            "%s 里的 arm_base_from_base_link 需要 translation_m(3) 与 "
            "quaternion_xyzw(4)" % path)
    return [float(v) for v in translation], [float(v) for v in quaternion]


def _setup(context):
    translation, quaternion = load_base_extrinsic()

    def _override(name, fallback):
        text = LaunchConfiguration(name).perform(context).strip()
        if not text:
            return fallback
        values = [float(v) for v in text.replace(" ", "").split(",") if v]
        if len(values) != len(fallback):
            raise RuntimeError("%s 需要 %d 个逗号分隔的数" % (name, len(fallback)))
        return values

    translation = _override("rigid_from_base_translation_m", translation)
    quaternion = _override("rigid_from_base_quaternion_xyzw", quaternion)

    vrpn_listener = Node(
        package="vrpn_listener", executable="vrpn_listener", name="vrpn_listener",
        output="screen", parameters=[{
            "server": LaunchConfiguration("server"),
            "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
            "frame_id": "vicon_world",
            "mainloop_frequency": 200.0,
            "refresh_trackers_frequency": 1.0,
            "tracker_mainloop_frequency": 200.0,
        }])
    bridge = Node(
        package="manipulator", executable="vicon_floating_base_bridge.py",
        name="vicon_floating_base_bridge", output="screen", parameters=[{
            "pose_topic": LaunchConfiguration("pose_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "zero_on_first_pose": ParameterValue(
                LaunchConfiguration("zero_on_first_pose"), value_type=bool),
            "rigid_from_base_translation_m": translation,
            "rigid_from_base_quaternion_xyzw": quaternion,
            # Vicon 原生 twist/accel 与 arm_ee 都由【同一个】桥接入：
            # zero_on_first_pose 用第一帧基座位姿定义了控制器世界系，
            # 另起节点转发 arm_ee 会落在相差 33 mm 的另一个世界系里，
            # 而末端残差本身才 1.77 mm。
            "twist_topic": LaunchConfiguration("twist_topic"),
            "accel_topic": LaunchConfiguration("accel_topic"),
            "ee_pose_topic": LaunchConfiguration("ee_pose_topic"),
            "ee_twist_topic": LaunchConfiguration("ee_twist_topic"),
            "ee_accel_topic": LaunchConfiguration("ee_accel_topic"),
            "ee_output_topic": LaunchConfiguration("ee_output_topic"),
            "prefer_vicon_twist": ParameterValue(
                LaunchConfiguration("prefer_vicon_twist"), value_type=bool),
            "prefer_vicon_linear_accel": ParameterValue(
                LaunchConfiguration("prefer_vicon_linear_accel"), value_type=bool),
            "prefer_vicon_angular_accel": ParameterValue(
                LaunchConfiguration("prefer_vicon_angular_accel"), value_type=bool),
        }])
    return [vrpn_listener, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("server", default_value="172.16.20.21"),
        DeclareLaunchArgument("port", default_value="3883"),
        DeclareLaunchArgument(
            "pose_topic", default_value="/vrpn/arm_base/pose",
            description="VRPN listener pose topic for the base rigid body"),
        DeclareLaunchArgument(
            "output_topic", default_value="/floating_base/vicon"),
        DeclareLaunchArgument("zero_on_first_pose", default_value="true"),
        DeclareLaunchArgument(
            "rigid_from_base_translation_m", default_value="",
            description="覆盖标定的 arm_base->base_link 平移 \"x,y,z\" (m)。"
                        "留空则用 task_space_smc/config/vicon_extrinsics.yaml"),
        DeclareLaunchArgument(
            "rigid_from_base_quaternion_xyzw", default_value="",
            description="覆盖标定的 arm_base->base_link 四元数 \"x,y,z,w\"。"
                        "留空则用 task_space_smc/config/vicon_extrinsics.yaml"),
        DeclareLaunchArgument(
            "twist_topic", default_value="/vrpn/arm_base/twist",
            description="Vicon 原生基座 twist。留空则只用桥自己的差分。"),
        DeclareLaunchArgument(
            "accel_topic", default_value="/vrpn/arm_base/accel",
            description="Vicon 原生基座 accel。留空则只用桥自己的差分。"),
        DeclareLaunchArgument(
            "ee_pose_topic", default_value="/vrpn/arm_ee/pose",
            description="末端刚体位姿；经同一世界系换算后转发到 ee_output_topic"),
        DeclareLaunchArgument(
            "ee_twist_topic", default_value="/vrpn/arm_ee/twist",
            description="Vicon 原生末端 twist。留空则只用桥对 arm_ee 位姿的"
                        "差分。末端通路【不加杠杆臂】：Vicon 报的就是标记簇"
                        "原点，而控制点就是这个原点，两点重合。"),
        DeclareLaunchArgument(
            "ee_accel_topic", default_value="/vrpn/arm_ee/accel",
            description="Vicon 原生末端 accel。留空则只用桥自己的二阶差分。"),
        DeclareLaunchArgument(
            "ee_output_topic", default_value="/vicon/arm_ee",
            description="输出类型是 dummy_interface/FloatingBaseState"
                        "（位姿/速度/加速度共用一个时间戳），不再是 PoseStamped"),
        DeclareLaunchArgument(
            "prefer_vicon_twist", default_value="true",
            description="优先用 Vicon 原生 twist；缺帧或时间戳缺失时自动回退本地差分"),
        DeclareLaunchArgument(
            "prefer_vicon_linear_accel", default_value="true"),
        DeclareLaunchArgument(
            "prefer_vicon_angular_accel", default_value="false",
            description="默认关：acc_quat 是二阶差分且 acc_quat_dt 未经实测，"
                        "而角加速度直接变力矩 —— 实机验证过再打开"),
        OpaqueFunction(function=_setup),
    ])
