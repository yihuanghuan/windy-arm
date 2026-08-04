"""Pinocchio floating-base A-L1-GAMMA model wrapper."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from .quaternion_math import quat_normalize


DEFAULT_ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")
DEFAULT_EE_FRAME = "link7"

# 6 自由度实机构型: 末端无电机, joint7/link7 物理不存在。
ARM6_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
ARM6_EE_FRAME = "link6"
ARM6_URDF_NAME = "arm6.urdf"

# 6 关节实物 + 刚性笔架。由 arm.urdf 把 joint7 从 revolute 改为 fixed 得到，
# link7 成为 link6 的刚性延伸（偏移 91.00 mm）。自由度仍是 6。
#
# 【末端必须显式取 link7】。自动探测规则是"最后一个驱动关节的子连杆"，
# joint7 变 fixed 后那是 link6 —— 恰好是我们要避开的那个点：末端落在腕
# 转轴上时 q6 转动不移动末端，只贡献 5D 任务的 2 行姿态行，容易掉秩。
# 实测沿轨迹 100 点：6D 姿态可达 43% -> 100%，5D 的 σ最差 0.0082 -> 0.0194。
ARM6_TOOL_EE_FRAME = "link7"
ARM6_TOOL_URDF_NAME = "arm6_tool.urdf"


def default_urdf_path(urdf_name: str = "arm.urdf") -> str:
    here = Path(__file__).resolve()
    source_tree_urdf = here.parent.parent / "config" / urdf_name
    if source_tree_urdf.exists():
        return str(source_tree_urdf)
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("manipulator"), urdf_name)
    except Exception:
        return str(source_tree_urdf)


ARM6_CALIBRATION_NAME = "arm6_calibration.yaml"


def arm6_calibration_path() -> str:
    """实机标定文件路径（与 URDF 同目录）。"""
    return os.path.join(os.path.dirname(arm6_urdf_path()), ARM6_CALIBRATION_NAME)


def load_arm6_calibration(path: str | None = None) -> dict:
    """加载实机标定值。文件不存在时返回恒等标定（全零偏差、ratio=1）。

    标定值的语义：
        实际关节角 = 命令角 + zero_offset_rad
        前馈重力   = g_model(q)   ← 【比值已烘进 URDF，不要再乘】

    ⚠️ first_moment_ratio 是"实测 / CAD名义模型"的比值。config/arm6.urdf 与
    config/arm6_tool.urdf 的连杆惯性参数【已按它修正】，所以模型算出的重力
    就是实机重力。再乘一次是重复修正 —— 曾在 test_phase2_probe.py 里出现过，
    把 2.83 Nm 夸大成 4.09 Nm。config/arm.urdf (7关节) 未修正，仍是名义值。

    ⚠️ 标定与机械状态绑定。拆装关节、改变基座【倾角】、手工掰动之后必须重测。
    纯平移不影响 —— 重力是均匀场。
    """
    import yaml
    path = path or arm6_calibration_path()
    n = len(ARM6_JOINT_NAMES)
    if not os.path.exists(path):
        return dict(zero_offset_rad=[0.0] * n, first_moment_ratio=[1.0] * n,
                    stiction_nm=[0.0] * n, converged=[False] * n,
                    armature_kgm2=None, inertia_kgm2=None,
                    inertia_stderr_kgm2=None,
                    kinetic_friction_nm=None, source=None)
    with open(path) as fh:
        raw = yaml.safe_load(fh)
    def vec(key, default):
        v = raw.get(key)
        if v is None or len(v) != n:
            return [default] * n
        return [float(x) for x in v]
    def optvec(key):
        v = raw.get(key)
        if v is None or len(v) != n:
            return None
        return [None if x is None else float(x) for x in v]
    return dict(
        armature_kgm2=optvec("armature_kgm2"),
        inertia_kgm2=optvec("inertia_kgm2"),
        inertia_stderr_kgm2=optvec("inertia_stderr_kgm2"),
        kinetic_friction_nm=optvec("kinetic_friction_nm"),
        zero_offset_rad=vec("zero_offset_rad", 0.0),
        first_moment_ratio=vec("first_moment_ratio", 1.0),
        stiction_nm=vec("stiction_nm", 0.0),
        converged=list(raw.get("zero_offset_converged", [False] * n)),
        uncertainty_deg=raw.get("zero_offset_uncertainty_deg"),
        metadata=raw.get("metadata", {}),
        source=path,
    )


def arm6_tool_urdf_path() -> str:
    """6 关节 + 笔架 URDF 的绝对路径。"""
    return default_urdf_path(ARM6_TOOL_URDF_NAME)


def arm6_urdf_path() -> str:
    """Path to the 6-joint URDF used by the real hardware."""
    return default_urdf_path(ARM6_URDF_NAME)


@dataclass
class Kinematics:
    ee_position: np.ndarray
    ee_rotation: np.ndarray
    ee_quaternion_wxyz: np.ndarray
    ee_velocity_lwa: np.ndarray
    J_full_lwa: np.ndarray
    J_base_lwa: np.ndarray
    J_arm_lwa: np.ndarray


class FloatingBaseArmModel:
    """Free-flyer base plus the actuated A-L1-GAMMA joints.

    Supports both the original 7-joint model (arm.urdf, EE=link7) and the
    6-joint hardware model (arm6.urdf, EE=link6). The joint count is derived
    from the URDF, never hardcoded: n_arm = nv - 6.

    q_full is [base_xyz(3), base_quat_xyzw(4), arm_q(n_arm)] because that is
    the Pinocchio FreeFlyer convention. Public helpers accept base quaternions
    as [w, x, y, z] and convert at the boundary.

    Note the base offsets 7 (nq) and 6 (nv) below are the FreeFlyer tangent
    sizes, not joint counts -- they stay fixed regardless of n_arm.
    """

    def __init__(
        self,
        urdf_path: str | None = None,
        ee_frame_name: str | None = None,
        arm_joint_names: tuple[str, ...] | None = None,
        apply_armature: bool = True,
    ):
        self.urdf_path = urdf_path or default_urdf_path()
        self.model = pin.buildModelFromUrdf(self.urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()

        # FreeFlyer contributes nq=7 / nv=6; the remainder is the arm.
        self.n_arm = self.model.nv - 6
        # 转子惯量经减速比反射的等效值，加在质量矩阵对角上。URDF 只有连杆
        # 惯量，而实测显示转子项占 60~95% —— 不加它 M 会小一个数量级，
        # 动态前馈几乎不起作用。
        if apply_armature:
            cal = load_arm6_calibration()
            arm = cal.get("armature_kgm2")
            if arm is not None and len(arm) == self.model.nv - 6:
                self.model.armature[6:] = np.asarray(
                    [0.0 if v is None else float(v) for v in arm])
                self.data = self.model.createData()
        if self.n_arm < 1 or self.model.nq != 7 + self.n_arm:
            raise ValueError(
                f"inconsistent free-flyer model from {self.urdf_path}: "
                f"nq={self.model.nq}, nv={self.model.nv}"
            )

        # Joint 0 is the free-flyer; the actuated joints follow in model order.
        model_joint_names = tuple(self.model.names[i] for i in range(2, self.model.njoints))
        if arm_joint_names is None:
            arm_joint_names = model_joint_names
        self.arm_joint_names = tuple(arm_joint_names)
        if len(self.arm_joint_names) != self.n_arm:
            raise ValueError(
                f"arm_joint_names has {len(self.arm_joint_names)} entries but the model "
                f"has {self.n_arm} actuated joints ({list(model_joint_names)})"
            )

        if ee_frame_name is None:
            # Body frame carried by the last actuated joint (robust against
            # extra sensor/tool frames appended after it).
            last_joint = self.model.njoints - 1
            bodies = [
                self.model.frames[i].name
                for i in range(self.model.nframes)
                if self.model.frames[i].parentJoint == last_joint
                and self.model.frames[i].type == pin.FrameType.BODY
            ]
            if len(bodies) != 1:
                raise ValueError(
                    f"cannot infer the end-effector frame from {self.urdf_path}: "
                    f"last joint carries {bodies}; pass ee_frame_name explicitly"
                )
            ee_frame_name = bodies[0]
        self.ee_frame_name = ee_frame_name
        self.ee_frame_id = self.model.getFrameId(ee_frame_name)
        if self.ee_frame_id >= len(self.model.frames):
            raise ValueError(f"frame {ee_frame_name!r} was not found in {self.urdf_path}")

        self.arm_lower_position_limit = self.model.lowerPositionLimit[7:].copy()
        self.arm_upper_position_limit = self.model.upperPositionLimit[7:].copy()

    def make_q_full(self, base_position_w: np.ndarray, base_quat_wxyz: np.ndarray, q_arm: np.ndarray) -> np.ndarray:
        p = np.asarray(base_position_w, dtype=float).reshape(3)
        q_wxyz = quat_normalize(base_quat_wxyz)
        q_arm = np.asarray(q_arm, dtype=float).reshape(self.n_arm)
        q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
        return np.concatenate([p, q_xyzw, q_arm])

    def make_v_full(self, base_velocity_lwa: np.ndarray, qd_arm: np.ndarray) -> np.ndarray:
        return np.concatenate([
            np.asarray(base_velocity_lwa, dtype=float).reshape(6),
            np.asarray(qd_arm, dtype=float).reshape(self.n_arm),
        ])

    def make_a_full(self, base_acceleration_lwa: np.ndarray, qdd_arm: np.ndarray) -> np.ndarray:
        return np.concatenate([
            np.asarray(base_acceleration_lwa, dtype=float).reshape(6),
            np.asarray(qdd_arm, dtype=float).reshape(self.n_arm),
        ])

    def neutral_q(self) -> np.ndarray:
        return pin.neutral(self.model)

    def split_arm_q(self, q_full: np.ndarray) -> np.ndarray:
        return np.asarray(q_full, dtype=float).reshape(self.model.nq)[7:].copy()

    def compute_kinematics(self, q_full: np.ndarray, v_full: np.ndarray | None = None) -> Kinematics:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        if v_full is None:
            v_full = np.zeros(self.model.nv)
        else:
            v_full = np.asarray(v_full, dtype=float).reshape(self.model.nv)
        pin.forwardKinematics(self.model, self.data, q_full, v_full)
        pin.updateFramePlacements(self.model, self.data)
        J_full = pin.computeFrameJacobian(
            self.model,
            self.data,
            q_full,
            self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        )
        vel = pin.getFrameVelocity(self.model, self.data, self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED).vector
        oMf = self.data.oMf[self.ee_frame_id]
        q_xyzw = pin.Quaternion(oMf.rotation).coeffs()
        q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
        return Kinematics(
            ee_position=oMf.translation.copy(),
            ee_rotation=oMf.rotation.copy(),
            ee_quaternion_wxyz=quat_normalize(q_wxyz),
            ee_velocity_lwa=vel.copy(),
            J_full_lwa=J_full.copy(),
            J_base_lwa=J_full[:, :6].copy(),
            J_arm_lwa=J_full[:, 6:].copy(),
        )

    def compute_mass_matrix(self, q_full: np.ndarray) -> np.ndarray:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        M = pin.crba(self.model, self.data, q_full)
        return np.asarray((M + M.T) * 0.5)

    def compute_arm_mass_block(self, q_full: np.ndarray) -> np.ndarray:
        return self.compute_mass_matrix(q_full)[6:, 6:].copy()

    def compute_frame_jacobian(self, q_full: np.ndarray) -> np.ndarray:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        return pin.computeFrameJacobian(
            self.model,
            self.data,
            q_full,
            self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        )

    def compute_task_bias_acceleration(self, q_full: np.ndarray, v_full: np.ndarray, a_base_lwa: np.ndarray) -> np.ndarray:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        v_full = np.asarray(v_full, dtype=float).reshape(self.model.nv)
        a_bias = np.concatenate([np.asarray(a_base_lwa, dtype=float).reshape(6), np.zeros(self.n_arm)])
        pin.forwardKinematics(self.model, self.data, q_full, v_full, a_bias)
        pin.updateFramePlacements(self.model, self.data)
        return pin.getFrameClassicalAcceleration(
            self.model,
            self.data,
            self.ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        ).vector.copy()

    def compute_rnea_arm_torque(
        self,
        q_full: np.ndarray,
        v_full: np.ndarray,
        a_base_lwa: np.ndarray,
        qdd_cmd: np.ndarray,
    ) -> np.ndarray:
        q_full = np.asarray(q_full, dtype=float).reshape(self.model.nq)
        v_full = np.asarray(v_full, dtype=float).reshape(self.model.nv)
        a_full = self.make_a_full(a_base_lwa, qdd_cmd)
        tau_full = pin.rnea(self.model, self.data, q_full, v_full, a_full)
        return np.asarray(tau_full[6:]).reshape(self.n_arm).copy()

    def compute_arm_forward_dynamics(
        self,
        q_full: np.ndarray,
        v_full: np.ndarray,
        a_base_lwa: np.ndarray,
        tau_arm: np.ndarray,
    ) -> np.ndarray:
        M = self.compute_mass_matrix(q_full)
        n_full = pin.rnea(self.model, self.data, q_full, v_full, np.zeros(self.model.nv))
        rhs = (
            np.asarray(tau_arm, dtype=float).reshape(self.n_arm)
            - M[6:, :6] @ np.asarray(a_base_lwa, dtype=float).reshape(6)
            - n_full[6:]
        )
        return np.linalg.solve(M[6:, 6:], rhs)
