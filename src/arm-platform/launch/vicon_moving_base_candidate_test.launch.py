"""Clean moving-base candidate validation entry.

This launch deliberately keeps the existing whole-task runner and Vicon bridge
unchanged.  It selects the candidate calibration and the formal Vicon
extrinsics explicitly, then starts the runner in hold mode.  The default is a
preflight-only launch (hardware_enabled=false).
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _source_path(repo_root: Path, relative: str) -> Path:
    source = repo_root / "src" / "arm-platform" / relative
    if source.exists():
        return source
    raise RuntimeError("要求的 source-tree 文件不存在，拒绝回退到默认文件: %s" % source)


def _repo_root() -> Path:
    here = Path(__file__).resolve()
    return here.parents[3]


def _load_and_print_calibration(path: Path) -> None:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    zero = data.get("zero_offset_rad")
    ratio = data.get("first_moment_ratio")
    f_pos = data.get("kinetic_friction_pos_nm")
    f_neg = data.get("kinetic_friction_neg_nm")
    for key, value in (("zero_offset_rad", zero),
                       ("first_moment_ratio", ratio),
                       ("kinetic_friction_pos_nm", f_pos),
                       ("kinetic_friction_neg_nm", f_neg)):
        if not isinstance(value, list) or len(value) != 6:
            raise RuntimeError("candidate 的 %s 不是 6 关节数组" % key)
    print("[candidate] ARM6_CALIBRATION_YAML=%s" % path, flush=True)
    for i, name in enumerate(names):
        def _number(value):
            return "None" if value is None else "%.9f" % float(value)
        print(
            "[candidate] %s zero=%s rad ratio=%s F+=%s Nm F-=%s Nm"
            % (name, _number(zero[i]), _number(ratio[i]),
               _number(f_pos[i]), _number(f_neg[i])),
            flush=True)


def _load_and_print_vicon(path: Path) -> None:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    base = data.get("arm_base_from_base_link") or {}
    ee = data.get("arm_ee_from_link6") or {}
    translation = np.asarray(base.get("translation_m"), dtype=float)
    quaternion = np.asarray(base.get("quaternion_xyzw"), dtype=float)
    ee_translation = np.asarray(ee.get("translation_m"), dtype=float)
    ee_quaternion = np.asarray(ee.get("quaternion_xyzw"), dtype=float)
    if translation.shape != (3,) or quaternion.shape != (4,):
        raise RuntimeError("Vicon YAML 缺少有效 arm_base_from_base_link")
    if ee_translation.shape != (3,) or ee_quaternion.shape != (4,):
        raise RuntimeError("Vicon YAML 缺少有效 arm_ee_from_link6")
    norm = float(np.linalg.norm(quaternion))
    ee_norm = float(np.linalg.norm(ee_quaternion))
    if not np.isfinite(norm) or abs(norm - 1.0) > 1.0e-6:
        raise RuntimeError("base quaternion_xyzw 未归一化: norm=%.12f" % norm)
    if not np.isfinite(ee_norm) or abs(ee_norm - 1.0) > 1.0e-6:
        raise RuntimeError("EE quaternion_xyzw 未归一化: norm=%.12f" % ee_norm)
    print("[vicon] VICON_EXTRINSICS_YAML=%s" % path, flush=True)
    print("[vicon] arm_base_to_base_link translation_m=%s" %
          np.array2string(translation, precision=10), flush=True)
    print("[vicon] arm_base_to_base_link quaternion_xyzw=%s norm=%.12f" %
          (np.array2string(quaternion, precision=10), norm), flush=True)
    print("[vicon] link6_to_arm_ee translation_m=%s" %
          np.array2string(ee_translation, precision=10), flush=True)
    print("[vicon] link6_to_arm_ee quaternion_xyzw=%s norm=%.12f" %
          (np.array2string(ee_quaternion, precision=10), ee_norm), flush=True)
    print("[vicon] arm_ee orientation is validation-only; control orientation is FK.",
          flush=True)


def _setup(context):
    root = _repo_root()
    candidate = _source_path(root, "config/arm6_calibration_candidate.yaml")
    vicon_yaml = _source_path(root, "task_space_smc/config/vicon_extrinsics.yaml")
    smc_yaml = _source_path(root, "task_space_smc/config/smc_params_vicon_candidate_test.yaml")
    vicon_launch = _source_path(root, "launch/vicon_floating_base.launch.py")
    urdf = _source_path(root, "config/arm6.urdf")
    _load_and_print_calibration(candidate)
    _load_and_print_vicon(vicon_yaml)
    print("[preflight] runner=%s" %
          (root / "src/arm-platform/task_space_smc/hw/whole_task_smc_ros_runner.py"),
          flush=True)
    print("[candidate] runtime loader: model.py uses this YAML via "
          "ARM6_CALIBRATION_YAML; zero_offset enters q_model and "
          "first_moment_ratio scales only the RNEA gravity term. "
          "F+/F- and armature remain from the same file.", flush=True)
    print("[preflight] hardware_enabled=%s (false means no effort validation output)" %
          LaunchConfiguration("hardware_enabled").perform(context), flush=True)
    print("[preflight] dynamic gates: /joint_states, /floating_base/vicon, /vicon/arm_ee, "
          "joint limits, FK validity and stale/watchdog checks are enforced by runner.",
          flush=True)

    vicon = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(vicon_launch)),
        launch_arguments={
            "server": LaunchConfiguration("vicon_server"),
            "port": LaunchConfiguration("vicon_port"),
            "pose_topic": "/vrpn/arm_base/pose",
            "ee_pose_topic": "/vrpn/arm_ee/pose",
            "output_topic": "/floating_base/vicon",
            "ee_output_topic": "/vicon/arm_ee",
            "zero_on_first_pose": "true",
        }.items(),
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_candidate_test",
        output="screen",
        parameters=[{"robot_description": urdf.read_text(encoding="utf-8")}],
    )
    runner = Node(
        package="manipulator",
        executable="whole_task_smc_ros_runner.py",
        name="whole_task_smc_ros_runner_candidate_test",
        output="screen",
        parameters=[{
            "config_path": str(smc_yaml),
            "urdf_path": str(urdf),
            "mode": "hardware",
            "trajectory_shape": "hold",
            "duration_sec": ParameterValue(LaunchConfiguration("duration_sec"),
                                             value_type=float),
            "start_delay_sec": 0.0,
            "hardware_enabled": ParameterValue(
                LaunchConfiguration("hardware_enabled"), value_type=bool),
            "data_dir": LaunchConfiguration("data_dir"),
            "control_rate_hz_override": ParameterValue(
                LaunchConfiguration("control_rate_hz"), value_type=float),
            "base_state_mode": "vicon",
            "external_base_topic": "/floating_base/vicon",
            "hold_capture_measured": True,
        }],
    )
    return [vicon, rsp, runner]


def generate_launch_description():
    root = _repo_root()
    data_dir = root / "src" / "arm-platform" / "task_space_smc" / "data" / "candidate_moving_base_test"
    return LaunchDescription([
        DeclareLaunchArgument("vicon_server", default_value="192.168.200.60"),
        DeclareLaunchArgument("vicon_port", default_value="3883"),
        DeclareLaunchArgument("hardware_enabled", default_value="false",
                              description="必须人工确认安全后才设 true。"),
        DeclareLaunchArgument("duration_sec", default_value="0.0"),
        DeclareLaunchArgument("control_rate_hz", default_value="200.0"),
        DeclareLaunchArgument("data_dir", default_value=str(data_dir)),
        SetEnvironmentVariable(
            "ARM6_CALIBRATION_YAML",
            str(_source_path(root, "config/arm6_calibration_candidate.yaml"))),
        OpaqueFunction(function=_setup),
    ])
