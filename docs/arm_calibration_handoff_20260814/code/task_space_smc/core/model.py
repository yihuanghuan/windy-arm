from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from .quaternion_math import quat_normalize


DEFAULT_ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")
DEFAULT_EE_FRAME = "link7"


ARM6_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
ARM6_EE_FRAME = "link6"
ARM6_URDF_NAME = "arm6.urdf"


ARM6_TOOL_EE_FRAME = "link7"
ARM6_TOOL_URDF_NAME = "arm6_tool.urdf"


def default_urdf_path(urdf_name: str = "arm.urdf") -> str:
    here = Path(__file__).resolve()
    source_tree_urdf = here.parents[2] / "config" / urdf_name
    if source_tree_urdf.exists():
        return str(source_tree_urdf)
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("manipulator"), urdf_name)
    except Exception:
        return str(source_tree_urdf)


ARM6_CALIBRATION_NAME = "arm6_calibration.yaml"


def arm6_calibration_path() -> str:
    return os.path.join(os.path.dirname(arm6_urdf_path()), ARM6_CALIBRATION_NAME)


def load_arm6_calibration(path: str | None = None) -> dict:
    import yaml
    path = path or os.environ.get("ARM6_CALIBRATION_YAML") or arm6_calibration_path()
    n = len(ARM6_JOINT_NAMES)
    if not os.path.exists(path):
        return dict(zero_offset_rad=[0.0] * n, first_moment_ratio=[1.0] * n,
                    stiction_nm=[0.0] * n, converged=[False] * n,
                    armature_kgm2=None, inertia_kgm2=None,
                    inertia_stderr_kgm2=None,
                    kinetic_friction_nm=None, kinetic_friction_pos_nm=None,
                    kinetic_friction_neg_nm=None, source=None)
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
        kinetic_friction_pos_nm=optvec("kinetic_friction_pos_nm"),
        kinetic_friction_neg_nm=optvec("kinetic_friction_neg_nm"),
        zero_offset_rad=vec("zero_offset_rad", 0.0),
        first_moment_ratio=vec("first_moment_ratio", 1.0),
        stiction_nm=vec("stiction_nm", 0.0),
        converged=list(raw.get("zero_offset_converged", [False] * n)),
        uncertainty_deg=raw.get("zero_offset_uncertainty_deg"),
        metadata=raw.get("metadata", {}),
        source=path,
    )


def arm6_tool_urdf_path() -> str:
    return default_urdf_path(ARM6_TOOL_URDF_NAME)


def arm6_urdf_path() -> str:
    return default_urdf_path(ARM6_URDF_NAME)


def model_from_config(config: dict) -> "FloatingBaseArmModel":
    urdf = config.get("urdf")
    ee_frame = config.get("ee_frame") or None
    if urdf in ("arm6_tool", ARM6_TOOL_URDF_NAME):
        return FloatingBaseArmModel(
            urdf_path=arm6_tool_urdf_path(),
            ee_frame_name=ee_frame or ARM6_TOOL_EE_FRAME,
            arm_joint_names=ARM6_JOINT_NAMES,
        )
    if urdf in ("arm6", ARM6_URDF_NAME):
        return FloatingBaseArmModel(
            urdf_path=arm6_urdf_path(),
            ee_frame_name=ee_frame or ARM6_EE_FRAME,
            arm_joint_names=ARM6_JOINT_NAMES,
        )
    return FloatingBaseArmModel(urdf_path=urdf, ee_frame_name=ee_frame)


@dataclass
class Kinematics:
    ee_position: np.ndarray
    ee_rotation: np.ndarray
    ee_quaternion_wxyz: np.ndarray
    ee_velocity_lwa: np.ndarray
    J_full_lwa: np.ndarray
    J_base_lwa: np.ndarray
    J_arm_lwa: np.ndarray


# 浮动基座 + n 自由度机械臂的运动学/动力学模型（pinocchio 后端）。
# 广义坐标 q = [p_w(3), q_xyzw(4), q_arm(n)]，nq = 7 + n
# 广义速度 v = [v_base(6), q̇_arm(n)]，nv = 6 + n。注意两处用的是不同的参考系：
#   v_base 按 pinocchio 自由飞行关节的惯例，表达在基座体坐标系（LOCAL）；
#   末端的雅可比与速度显式指定 LOCAL_WORLD_ALIGNED（原点在末端、轴与世界系对齐）。
# 动力学按基座 b / 手臂 a 分块：
#   [M_bb M_ba][a_base]   [h_b]   [f_ext]
#   [M_ab M_aa][ q̈  ] + [h_a] = [ τ  ]
class FloatingBaseArmModel:

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


        self.n_arm = self.model.nv - 6


        # 电机转子惯量经减速比折算到关节侧，加在质量阵对角：M_aa ← M_aa + diag(armature)
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

    # base_velocity 表达在【基座体系】，不是世界系
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

    # 末端位姿、速度与雅可比，全部在 LOCAL_WORLD_ALIGNED 系下：
    #   ẋ = J_full·v = J_base·v_base + J_arm·q̇     （J_full 为 6×(6+n)）
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

    # 全模型质量阵 M(q)（CRBA）。取 (M + Mᵀ)/2 是因为 CRBA 只填上三角，
    # 数值上的微小不对称会让后续 Cholesky/求解失败。
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

    # 任务空间偏置加速度。末端加速度 ẍ = J_arm·q̈ + (J_base·a_base + J̇·v)，
    # 令 q̈ = 0 调用一次正向运动学，返回的就是括号里那一项，供下游解 q̈ 时移到右端。
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

    # 逆动力学取手臂分量（RNEA），基座加速度当作已知外部激励：
    #   τ_arm = M_aa·q̈_cmd + M_ab·a_base + h_a(q, v)
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

    # 手臂正向动力学，上式反解 q̈：
    #   q̈ = M_aa⁻¹·(τ_arm − M_ab·a_base − h_a),  h = RNEA(q, v, 0) 取手臂分量
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
