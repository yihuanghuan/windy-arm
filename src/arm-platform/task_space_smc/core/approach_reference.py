"""就位段的【任务空间】参考轨迹：从当前末端位姿平滑走到轨迹起点。

为什么要有这个模块（2026-08-04）
================================================================
就位（approach）此前有两种做法，两种都不理想：

  1. **位置模式**（``hw/full_prepose.py``）：MIT 模式发位置指令 + kp/kd。
     它不经过 SMC，是另一套控制器。代价是位置模式顶不过静摩擦 —— 实测
     j3 在 kp=80 下仍差约 2°、j2 差约 10°（j2 静摩擦 1.775 Nm 是其余关节的
     2.4~18 倍，同时承载最重）。于是每次实验的**起步位形都不是设定的那个**，
     跨天对比时末端还差约 10 mm。
  2. **关节空间插值 + 关节 PD + RNEA**（runner 里原来的 ``_run_approach``）：
     力矩模式，能压住摩擦，但规划在关节空间 —— 末端走的是一条没人算过的
     曲线，可能扫过桌面或围栏；而且它用的是另一套增益（approach_kp/kd），
     与跟踪段的 SMC 不是同一个控制器，交接处两套控制器换手。

本模块给第三种：**在末端空间规划，用与跟踪段完全相同的 Lagrange-SMC 执行**。
好处有三条：

  * 末端轨迹是显式的直线（位置）+ 测地线（姿态），可以事先做碰撞检查；
  * 全程只有一个控制器、一套增益，交接处不换手，
    ``handover_blend`` 面对的残差因此只剩跟踪误差量级；
  * 力矩模式天然压得住静摩擦，不再有"到不了位"的 2°/10° 残差。

时间标度与插值
================================================================
时间用五次 smoothstep ``s(u) = 10u³ − 15u⁴ + 6u⁵``（u = t/T），两端一阶、
二阶导都为零 —— 起步和到位都没有加速度阶跃。

位置走直线::

    p(t)  = p₀ + s·Δp        ṗ = ṡ·Δp        p̈ = s̈·Δp

姿态走 SO(3) 测地线（等价于四元数 SLERP，但直接给出角速度/角加速度）::

    Δr    = log(R₀ᵀ R₁)                     # R₀ 体系下的旋转矢量
    R(t)  = R₀ · exp(s·Δr^)

角速度取**世界系**（与 ``PoseState.angular_velocity`` 及
``kin.ee_velocity_lwa[3:]`` 的 LOCAL_WORLD_ALIGNED 口径一致）。推导：

    Ṙ = R₀ exp(sΔr^)·(ṡΔr^) = R·(ṡΔr^)      # 体系角速度 = ṡ·Δr
    ω_w = R·(ṡΔr) = R₀ exp(sΔr^)·ṡΔr = ṡ·R₀Δr

最后一步用了 ``exp(sΔr^)·Δr = Δr``（旋转不改变自己的轴）。所以::

    ω_w(t) = ṡ · (R₀ Δr)        ω̇_w(t) = s̈ · (R₀ Δr)

即角速度方向在世界系里是**恒定**的，只有大小随 ṡ 变 —— 单轴定轴转动。

⚠️ 边界情况
================================================================
* ``T <= 0``：直接返回目标位姿、速度加速度全零（等价于"已经在那儿"）。
* ``t >= T``：钳到 s=1，ṡ=s̈=0。到位后继续采样是安全的，参考量停在目标上。
* Δr 的模长恒在 [0, π]：``log`` 取主值，走的是**短程**测地线。
  R₀ 与 R₁ 相差恰好 180° 时轴不唯一，此时任选一轴都对，数值上由
  ``rotation_to_quat`` 的符号约定确定。
* 本模块**不做碰撞检查**。调用方要自己验直线路径 —— 见 runner 里的
  ``_check_approach_path_clear``。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .pose_error import PoseState
from .quaternion_math import rotation_to_quat


def quintic(u: float) -> tuple[float, float, float]:
    """五次 smoothstep 及其一、二阶导（对 u，不是对 t）。

    返回 ``(s, ds/du, d²s/du²)``。调用方自己除以 T 换算到时间导数。
    """
    u = float(min(max(u, 0.0), 1.0))
    s = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
    ds = 30.0 * u ** 2 - 60.0 * u ** 3 + 30.0 * u ** 4
    dds = 60.0 * u - 180.0 * u ** 2 + 120.0 * u ** 3
    return s, ds, dds


def rotation_log(R: np.ndarray) -> np.ndarray:
    """SO(3) 主值对数：返回旋转矢量，模长在 [0, π]。

    ⚠️ 走四元数，不走 ``θ/(2 sin θ)·(R−Rᵀ)`` 那个教科书式子。后者在 θ→π 时
    除以 sin θ（θ=179.9999° 时只有 1.7e-6），把舍入误差放大 6 个量级 ——
    实测该式在 179.9999° 处往返误差 2e-5，而本实现是 1e-15 量级。
    四元数式全域良态：θ→π 时 ‖v‖→1（分母趋于 1 而不是 0）。

    ``w < 0`` 时取反 —— q 与 −q 是同一个旋转，取 w≥0 那一支就是取**短程**。
    """
    q = rotation_to_quat(np.asarray(R, dtype=float).reshape(3, 3))
    w = float(q[0])
    v = np.asarray(q[1:], dtype=float)
    if w < 0.0:
        w, v = -w, -v
    n = float(np.linalg.norm(v))
    if n < 1e-15:
        return np.zeros(3)
    # atan2(‖v‖, w) = θ/2，w≥0 时落在 [0, π/2]，故 θ ∈ [0, π]。
    return (2.0 * np.arctan2(n, w) / n) * v


def rotation_exp(phi: np.ndarray) -> np.ndarray:
    """罗德里格斯公式：旋转矢量 -> 旋转矩阵。"""
    phi = np.asarray(phi, dtype=float).reshape(3)
    theta = float(np.linalg.norm(phi))
    if theta < 1e-12:
        return np.eye(3)
    k = phi / theta
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return (np.eye(3) + np.sin(theta) * K
            + (1.0 - np.cos(theta)) * (K @ K))


@dataclass
class ApproachPlan:
    """就位段的规划量，供日志与安全检查使用。"""
    distance_m: float           # 末端直线位移
    rotation_deg: float         # 姿态测地角
    duration_sec: float
    peak_linear_speed: float    # 1.875·|Δp|/T
    peak_angular_speed: float   # 1.875·|Δr|/T
    peak_linear_accel: float    # 5.7735·|Δp|/T²


class TaskSpaceApproach:
    """从起始末端位姿到目标末端位姿的五次时间标度插值。

    ``sample(t)`` 返回可以直接喂给 ``TaskSpaceSMC.compute`` 的 ``PoseState``。
    """

    # 五次 smoothstep 的极值：max ṡ = 1.875/T，max s̈ = 10/√3 /T² ≈ 5.7735/T²
    PEAK_DS = 1.875
    PEAK_DDS = 10.0 / np.sqrt(3.0)

    def __init__(self, start_position, start_rotation,
                 goal_position, goal_rotation, duration_sec: float):
        self.p0 = np.asarray(start_position, dtype=float).reshape(3)
        self.R0 = np.asarray(start_rotation, dtype=float).reshape(3, 3)
        self.p1 = np.asarray(goal_position, dtype=float).reshape(3)
        self.R1 = np.asarray(goal_rotation, dtype=float).reshape(3, 3)
        self.duration_sec = float(duration_sec)
        self.dp = self.p1 - self.p0
        # R₀ 体系下的旋转矢量；世界系的转轴 R₀·Δr 全程不变（见模块头推导）。
        self.dr = rotation_log(self.R0.T @ self.R1)
        self.axis_w = self.R0 @ self.dr

    def plan(self) -> ApproachPlan:
        T = max(self.duration_sec, 1e-9)
        d = float(np.linalg.norm(self.dp))
        a = float(np.linalg.norm(self.dr))
        return ApproachPlan(
            distance_m=d,
            rotation_deg=float(np.degrees(a)),
            duration_sec=self.duration_sec,
            peak_linear_speed=self.PEAK_DS * d / T,
            peak_angular_speed=self.PEAK_DS * a / T,
            peak_linear_accel=self.PEAK_DDS * d / (T * T),
        )

    def finished(self, elapsed: float) -> bool:
        return float(elapsed) >= self.duration_sec

    def sample(self, elapsed: float) -> PoseState:
        T = self.duration_sec
        if T <= 0.0:
            # 没有时间可走 —— 参考量就是目标，速度加速度为零。
            return PoseState(
                position=self.p1.copy(),
                quaternion_wxyz=rotation_to_quat(self.R1),
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
                linear_acceleration=np.zeros(3),
                angular_acceleration=np.zeros(3),
            )
        s, ds_du, dds_du = quintic(float(elapsed) / T)
        ds = ds_du / T                  # ṡ
        dds = dds_du / (T * T)          # s̈
        R = self.R0 @ rotation_exp(s * self.dr)
        return PoseState(
            position=self.p0 + s * self.dp,
            quaternion_wxyz=rotation_to_quat(R),
            linear_velocity=ds * self.dp,
            angular_velocity=ds * self.axis_w,
            linear_acceleration=dds * self.dp,
            angular_acceleration=dds * self.axis_w,
        )

    def sample_positions(self, count: int = 51) -> np.ndarray:
        """路径上均匀采样的末端位置，(count, 3)。给碰撞检查用。

        姿态不影响末端点的位置，所以只需要位置这一条直线；但连杆本体会不会
        碰要靠调用方按关节解算，本函数只负责末端点。
        """
        n = max(2, int(count))
        s = np.array([quintic(u)[0] for u in np.linspace(0.0, 1.0, n)])
        return self.p0[None, :] + s[:, None] * self.dp[None, :]
