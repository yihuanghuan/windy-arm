"""approach 段的静摩擦前馈。

为什么需要它
------------
approach 段是纯 PD（`approach.kp` / `approach.kd`），没有积分项。纯比例控制
器在静摩擦下必然停在一个死区里：关节只有在驱动力矩超过静摩擦 ``f_s`` 时才会
挣脱，所以稳态残差下界就是

    e_deadband = f_s / K,      K = 力矩刚度 [Nm/rad]

⚠️ **K 不是 approach.kp。** 看 ``_run_approach``：kp 进的是

    qdd_cmd = qdd_des + kp·(q_des − q) + kd·(q̇_des − q̇)

也就是说 kp 的量纲是 1/s²，是**加速度增益**；它还要再过一遍 RNEA 的惯量矩阵
才变成力矩。真正的力矩刚度是

    K_i = M_ii(q) · kp_i

腕部三轴的连杆惯量只有 0.011 ~ 0.013 kgm²，于是 kp=50 只折合 0.6 Nm/rad ——
比按 Nm/rad 直读小 80 倍。2026-08-03 上机位形下的实际数值：

    joint   M_ii(kgm²)   kp(1/s²)   K(Nm/rad)   f_s(Nm)   死区(deg)
      1       0.2374        60        14.24      0.725       2.92
      2       0.1952        60        11.71      1.775       8.68
      3       0.1379        60         8.27      0.625       4.33
      4       0.0121        50         0.61      0.125      11.80
      5       0.0128        50         0.64      0.100       8.94
      6       0.0106        40         0.42      0.100      13.52     ← 最差

2026-08-03 那次上机的实测停止点与这张表吻合。approach 目标 ``track_start_q``
本身是好的（离线复算：位置误差 0.034 mm、笔轴 0.036°、限位余量 1.180 rad，
需移动 23.2°），但机械臂只走了一部分就停住，逐关节欠量

    [2.33, -3.50, 3.44, 13.36, -18.43, 3.15]°

每一项都落在同关节的死区量级内。同期线上 0x12 力矩全程只有 0.39 ~ 1.47 Nm，
正是"刚度太软 → 驱动力矩起不来 → 挣不脱静摩擦"的直接证据。握手时末端位置
误差 34.2 mm、笔轴姿态误差 20.7°；随后 SMC 以这个误差接管，λ=72、k=102 需求
加速度 ≈ 251 m/s²，六轴在一个控制周期内全部饱和，55 ms 后看门狗按住。

补偿律
------
    τ_ff = scale · f_s · tanh( (q̇_des + error_gain · e) / velocity_epsilon )

三个设计选择，每一个都是为了避免一种已知的失效：

1. **用参考速度 q̇_des，不用实测速度 q̇。** 实测速度过 tanh 会把测量噪声整流成
   力矩抖动；参考速度是解析已知的，光滑且无噪声。这是"前馈"而非"补偿"。

2. **叠加 error_gain · e。** approach 用五次多项式混合，两端 q̇_des ≡ 0 —— 而
   末端恰恰是需要挣脱静摩擦去消残差的时刻。把位置误差折算成等效速度请求，
   保证收尾阶段前馈不消失。

3. **tanh 的位置拐点 κ = velocity_epsilon / error_gain 必须与实际跟踪误差同
   量级。** κ 远小于误差尺度时 tanh 全程饱和，过零即整幅翻转，前馈退化成
   ±scale·f_s 的方波 —— bang-bang，机械臂会抖。2026-08-03 上机用
   error_gain=10（κ=0.115°）实测：指令力矩符号翻转 j5 605 次/30 s = 20.2 Hz，
   j2 指令标准差 1.77 Nm，前馈 43.6% 时间顶在满幅，操作者直接观察到剧烈抖动；
   而同期跟踪误差中位数 0.23°~7.88°，是 κ 的 2~69 倍。改 error_gain=1.0
   （κ=1.15°）后与误差同量级，死区只从 2.73° 涨到 2.87°。

4. **scale 必须严格小于 1。** 这是硬约束，不是调参余量。scale ≥ 1 时零误差处
   的前馈本身就够挣脱静摩擦，任何测量噪声都能让关节松动，而松动后误差反号、
   前馈随之反号，把关节推回去 —— 典型的极限环，振幅约等于死区。scale < 1 时
   零误差处净驱动力矩恒小于 f_s，关节不可能自发松动。

补偿后的死区（q̇_des = 0 稳态，且 e >> velocity_epsilon / error_gain 时 tanh 已饱和）：

    K·e + scale·f_s > f_s   ⟹   e > (1 - scale) · f_s / K

scale = 0.8 时死区缩到原来的 1/5：最差的 joint6 由 13.52° 降到 2.70°。

⚠️ 前馈是**力矩**，必须叠加在 RNEA 之后，不能混进 ``qdd_cmd`` —— 混进去会被
惯量矩阵再乘一次，正好犯下上面那个量纲错误。

适用范围
--------
⚠️ 这是对【真实摩擦】的补偿。在无摩擦的被控对象上它是纯粹的扰动：

* ``mode=internal_simulation``：runner 自带的积分器走 ``compute_arm_forward_dynamics``，
  只有 armature，没有摩擦。
* ``mode=simulation``：ROS 仿真的 ``torque_sim_arm.cc`` 同样没有摩擦模型。

所以代码默认 ``enabled=False``，只在 ``config/smc_params_hwtest.yaml``（实机配置）
里显式打开。已有的仿真基线不受影响。
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "FrictionFeedforwardConfig",
    "approach_torque_stiffness",
    "stiction_feedforward",
    "residual_deadband",
]

# scale 的硬上界。见模块 docstring 设计选择 3。
_MAX_SAFE_SCALE = 1.0


class FrictionFeedforwardConfig:
    """approach 静摩擦前馈的参数，带取值校验。"""

    __slots__ = ("enabled", "stiction_nm", "scale", "velocity_epsilon",
                 "error_gain")

    def __init__(
        self,
        stiction_nm,
        *,
        enabled: bool = False,
        scale: float = 0.8,
        velocity_epsilon: float = 0.02,
        error_gain: float = 1.0,
    ) -> None:
        stiction = np.asarray(stiction_nm, dtype=float).ravel()
        if stiction.size == 0:
            raise ValueError("stiction_nm 不能为空")
        if not np.all(np.isfinite(stiction)):
            raise ValueError(f"stiction_nm 含非有限值: {stiction}")
        if np.any(stiction < 0.0):
            raise ValueError(f"stiction_nm 不能为负: {stiction}")

        scale = float(scale)
        if not np.isfinite(scale) or scale < 0.0:
            raise ValueError(f"scale 必须是 [0, 1) 内的有限值，收到 {scale}")
        if scale >= _MAX_SAFE_SCALE:
            raise ValueError(
                f"scale={scale} 不允许。必须严格小于 1 —— scale >= 1 时零误差处的"
                "前馈本身就够挣脱静摩擦，会自激成振幅约等于死区的极限环。"
                "详见 friction_feedforward 模块 docstring 设计选择 3。")

        velocity_epsilon = float(velocity_epsilon)
        if not np.isfinite(velocity_epsilon) or velocity_epsilon <= 0.0:
            raise ValueError(
                f"velocity_epsilon 必须为正（tanh 的速度尺度），收到 {velocity_epsilon}")

        error_gain = float(error_gain)
        if not np.isfinite(error_gain) or error_gain < 0.0:
            raise ValueError(
                f"error_gain 必须非负（1/s，把位置误差折成等效速度），收到 {error_gain}")

        self.enabled = bool(enabled)
        self.stiction_nm = stiction
        self.scale = scale
        self.velocity_epsilon = velocity_epsilon
        self.error_gain = error_gain

    @property
    def n_joints(self) -> int:
        return int(self.stiction_nm.size)

    def __repr__(self) -> str:  # pragma: no cover - 诊断用
        return (f"FrictionFeedforwardConfig(enabled={self.enabled}, "
                f"scale={self.scale}, velocity_epsilon={self.velocity_epsilon}, "
                f"error_gain={self.error_gain}, "
                f"stiction_nm={np.round(self.stiction_nm, 4).tolist()})")


def stiction_feedforward(
    config: FrictionFeedforwardConfig,
    velocity_reference,
    position_error,
) -> np.ndarray:
    """算出叠加到 RNEA 力矩上的静摩擦前馈。

    参数
    ----
    velocity_reference : 参考速度 q̇_des [rad/s]。用参考量而非实测量是刻意的，
        见模块 docstring 设计选择 1。
    position_error : 跟踪误差 q_des - q [rad]。

    返回
    ----
    与输入同维的力矩数组 [Nm]。``config.enabled`` 为假时返回全零。
    """
    n = config.n_joints
    qd_ref = np.asarray(velocity_reference, dtype=float).reshape(n)
    err = np.asarray(position_error, dtype=float).reshape(n)

    if not config.enabled:
        return np.zeros(n)

    if not np.all(np.isfinite(qd_ref)) or not np.all(np.isfinite(err)):
        # 反馈损坏时宁可不补偿，也不要把 NaN 灌进力矩链路。
        return np.zeros(n)

    drive = qd_ref + config.error_gain * err
    return config.scale * config.stiction_nm * np.tanh(
        drive / config.velocity_epsilon)


def approach_torque_stiffness(inertia_diagonal, kp) -> np.ndarray:
    """把 approach 的加速度增益折成力矩刚度 K_i = M_ii(q)·kp_i [Nm/rad]。

    ``approach.kp`` 的量纲是 1/s²（它进的是 ``qdd_cmd``，之后才过 RNEA），
    直接当 Nm/rad 用会把腕部刚度高估 80 倍。见模块 docstring。

    ``inertia_diagonal`` 取 ``pin.crba(model, data, q)`` 的手臂对角块，因此
    K 随位形变化 —— 报数时要说明是在哪个位形上算的。
    """
    M = np.asarray(inertia_diagonal, dtype=float).ravel()
    gains = np.asarray(kp, dtype=float).ravel()
    if M.shape != gains.shape:
        raise ValueError(f"惯量与增益维度不一致: {M.shape} vs {gains.shape}")
    if np.any(M <= 0.0):
        raise ValueError(f"惯量对角元必须为正，收到 {M}")
    return M * gains


def residual_deadband(config: FrictionFeedforwardConfig, torque_stiffness) -> np.ndarray:
    """补偿后的稳态死区 [rad]：关节能挣脱静摩擦所需的最小位置误差。

    q̇ = 0、q̇_des = 0 的稳态下，净驱动力矩是

        drive(e) = K·e + scale·f_s·tanh(e / κ),      κ = velocity_epsilon / error_gain

    关节被卡住当且仅当 drive(e) < f_s。死区 e* 是 ``drive(e*) = f_s`` 的解。
    drive 关于 e 严格单调递增，用二分求解。

    ⚠️ ``torque_stiffness`` 是 **Nm/rad**，不是 ``approach.kp``。用
    :func:`approach_torque_stiffness` 从 M_ii(q) 和 kp 算出来。

    ⚠️ 不要用渐近式 ``(1 - scale)·f_s/K``。那是 e* >> κ 时的极限；死区落到 κ
    附近时 tanh 还在线性段，渐近式会夸大补偿效果。这里返回的是精确解。

    ``config.enabled`` 为假时退化成未补偿的 f_s/K。
    """
    n = config.n_joints
    gains = np.asarray(torque_stiffness, dtype=float).reshape(n)
    if np.any(gains <= 0.0):
        raise ValueError(f"torque_stiffness 必须为正，收到 {gains}")

    f_s = config.stiction_nm
    if not config.enabled or config.scale == 0.0:
        return f_s / gains

    knee = config.velocity_epsilon / config.error_gain if config.error_gain else np.inf
    # tanh >= 0，所以未补偿死区 f_s/kp 必是上界；0 必是下界。
    lo = np.zeros(n)
    hi = f_s / gains
    for _ in range(200):                       # 二分到远超 float 分辨率
        mid = 0.5 * (lo + hi)
        drive = gains * mid + config.scale * f_s * np.tanh(mid / knee)
        stuck = drive < f_s
        lo = np.where(stuck, mid, lo)
        hi = np.where(stuck, hi, mid)
    return 0.5 * (lo + hi)
