"""交接瞬间的参考量偏置衰减。

要解决的问题
------------
approach 结束、SMC 接管的那一帧，末端位姿和轨迹起点之间总有残差。SMC 的
增益是按"已经在轨迹上"整定的（hwtest: λ=72, k=102），需求加速度约

    a ≈ k·λ·e = 7344 · e   [m/s²]

而 2026-08-03 上机位形处，六轴额定力矩折到末端只有 **48.8 m/s²**。也就是说

    交接误差超过 6.65 mm，SMC 的需求就超出执行器能力。

实测：

    approach 静摩擦前馈关 -> 交接误差 110.3 mm -> 需求 810 m/s²  (16.6x 超限)
    approach 静摩擦前馈开 -> 交接误差  22.1 mm -> 需求 162 m/s²  ( 3.3x 超限)

前馈把误差压了 5 倍，仍然超限 3.3 倍。2026-08-03 那次上机就是这样死的：
t=0 接管，一个控制周期内六轴全部饱和，t=0.06 看门狗按住，之后 25 秒是尸体。

**把 approach 调得更准不是出路** —— 6.65 mm 的门限意味着 approach 必须做到
亚毫米级，而它的稳态精度被静摩擦死区物理地限制在几毫米。

做法
----
不去追求交接误差小，而是让它**恒等于零**：把参考轨迹整体平移/旋转到当前
实际位姿上，再让这个偏置平滑衰减到零。

    Δp = x_actual(t₀) − p_traj(t₀)          位置偏置，交接时刻锁定
    ω̂  = log(R_actual(t₀) · R_traj(t₀)ᵀ)    姿态偏置（世界系轴角）

    p_ref(t) = p_traj(t) + s(τ)·Δp
    R_ref(t) = Exp(s(τ)·ω̂) · R_traj(t)          τ = t − t₀

A、B 都是五次多项式基：A(0)=1、A'(0)=0、B(0)=0、B'(0)=1，两者在 τ=T 处
连同一、二阶导全部归零。所以

* τ=0 时参考量**就是**当前实际位姿【和当前实际速度】 —— SMC 的滑模变量
  s = ė + Λe 两项同时为零，不会索要任何修正加速度；
* **速度项不可省**。approach 的五次插值末端速度为零，但机械臂滞后于插值。
  2026-08-03 上机实测：位置偏置已把 e 压到恒等于 0，末端却仍以 0.180 m/s
  运动，(K+Λ)·|ė| = 31.4 m/s² 独占了全部需求，joint2 被推到 17.9 Nm
  （额定 9），看门狗在交接后 70 ms 触发；
* τ=T 之后偏置完全消失，参考量就是原轨迹，不残留任何畸变；
* 全程 C² 连续，速度/加速度前馈都有解析表达，不会引入台阶。

代价：偏置衰减自身贡献的加速度峰值是 5.7735·|Δp|/T² + 3.9403·|Δv|/T。
2026-08-03 的实测值 Δp=28 mm、Δv=0.180 m/s、T=3 s 时是 **0.254 m/s²**，
相对 48.8 m/s² 的能力上限是千分之五。

⚠️ 这**不是**精度手段。偏置在 [0, T] 内让参考量偏离真实轨迹，所以这段时间的
"跟踪误差"没有物理意义，评分必须从 τ>T 之后开始算。它换来的是**能跑完**。

角速度的推导
------------
R_ref = R_off · R_traj，R_off = Exp(s(τ)·ω̂)，轴固定所以 Ṙ_off = [s'·ω̂]× R_off。

    Ṙ_ref = [s'ω̂]× R_off R_traj + R_off [ω_traj]× R_traj
          = ([s'ω̂]× + [R_off ω_traj]×) R_ref

    ⟹ ω_ref = s'·ω̂ + R_off · ω_traj

注意 ω_traj 要被 R_off 旋转，不是简单相加。再微分一次：

    ω̇_ref = s''·ω̂ + s'·(ω̂ × (R_off ω_traj)) + R_off · ω̇_traj

τ ≥ T 时 R_off = I、s' = s'' = 0，三式都退化成原轨迹。
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "HandoverBlend",
    "smoothstep_decay",
    "trajectory_clock",
    "velocity_decay",
]


def trajectory_clock(t: float, blend_sec: float, freeze: bool) -> float:
    """墙钟时间 t -> 喂给 ``reference_sample`` 的【轨迹时钟】。

    ``freeze=False`` 时恒等映射（逐位等价于改动前的 ``reference_sample(t)``）。

    ``freeze=True`` 时::

        t <  blend_sec : 0.0            拉入期间轨迹【停在起点】
        t >= blend_sec : t - blend_sec  轨迹时钟从拉入结束才开始走

    为什么需要它
    ------------
    偏置衰减把参考量在 [0, blend_sec] 内从"当前实际位姿"平滑并到"轨迹上"，
    但轨迹时钟原本照常走。hwtest 圆周期 25.13 s，blend_sec=45 s ——
    拉入期间参考点自己绕了 **1.79 圈**，参考路径长 445 mm、包络 243 mm。
    机械臂忠实跟随，于是 RViz 里是一团乱麻，而那 45 s 的运动没有任何任务意义。

    冻结之后参考路径降到 157 mm 的一条收敛直线（= |Δp| 本身）。

    代价是 t=blend_sec 处 ``dclock/dt`` 从 0 跳到 1，即参考速度出现阶跃。
    上界就是轨迹起点处的切向速度：hwtest 圆周 0.010 m/s，
    需求加速度 (K+Λ)·Δv = 52.2 × 0.010 = 0.52 m/s²，
    相对该位形 48.8 m/s² 的能力上限是 1%，可忽略。
    （hwtest 起点还有 20% 周期的 smoothstep 起步整形，实际阶跃比这个上界更小。）

    注意 ``clock(t)`` 本身是【连续】的（t→blend_sec⁻ 与 t→blend_sec⁺ 都给 0），
    所以参考【位置/姿态】没有跳变，只有速度以上的导数有台阶。
    """
    t = float(t)
    if not freeze:
        return t
    T = float(blend_sec)
    if not np.isfinite(T) or T <= 0.0:
        # blend_sec<=0 时偏置衰减本身就是关闭的，冻结无从谈起，退回恒等映射。
        return t
    if t < T:
        return 0.0
    return t - T


def smoothstep_decay(tau: float, duration: float) -> tuple[float, float, float]:
    """位置偏置的五次衰减基 A(u) 及其对 τ 的一、二阶导。

    A(0)=1, A(T)=0, A'(0)=A'(T)=A''(0)=A''(T)=0。
    duration <= 0 表示不混合，直接返回 (0, 0, 0)。
    """
    T = float(duration)
    if T <= 0.0:
        return 0.0, 0.0, 0.0
    u = float(tau) / T
    if u <= 0.0:
        return 1.0, 0.0, 0.0
    if u >= 1.0:
        return 0.0, 0.0, 0.0
    rise = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    rise_d = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / T
    rise_dd = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / (T * T)
    return 1.0 - rise, -rise_d, -rise_dd


def velocity_decay(tau: float, duration: float) -> tuple[float, float, float]:
    """速度偏置的五次衰减基 B(u)·T 及其对 τ 的一、二阶导。

    满足 B(0)=0, B'(0)=1, B''(0)=0, B(T)=B'(T)=B''(T)=0，其中

        B(u) = u - 6u³ + 8u⁴ - 3u⁵

    于是 ``o(τ) = Δp·A(τ) + Δv·B(τ)`` 在 τ=0 处同时满足 o=Δp、o'=Δv，
    即位置**和速度**误差都恰好为零；τ=T 处三阶都归零，无残留。

    为什么必须有它：approach 的五次插值末端速度为零，但机械臂是**滞后**于
    插值的 —— 2026-08-03 上机实测，交接瞬间位置误差已被 A(τ) 压到 0，末端
    却仍以 **0.180 m/s** 运动，而参考速度是 0。于是

        Λ·ė + K·s = (72 + 102) × 0.180 = 31.4 m/s²

    这一项独占了当时全部 32.9 m/s² 的需求，把 joint2 推到 17.9 Nm（额定 9），
    看门狗在交接后 70 ms 触发。只补位置不补速度是不够的。
    """
    T = float(duration)
    if T <= 0.0:
        return 0.0, 0.0, 0.0
    u = float(tau) / T
    if u <= 0.0:
        return 0.0, 1.0, 0.0
    if u >= 1.0:
        return 0.0, 0.0, 0.0
    val = (u - 6.0 * u**3 + 8.0 * u**4 - 3.0 * u**5) * T
    d = 1.0 - 18.0 * u**2 + 32.0 * u**3 - 15.0 * u**4
    dd = (-36.0 * u + 96.0 * u**2 - 60.0 * u**3) / T
    return val, d, dd


def _hat(w: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def _log_so3(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 轴角向量。数值稳健，覆盖 θ→0 与 θ→π。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cos = 0.5 * (np.trace(R) - 1.0)
    cos = float(np.clip(cos, -1.0, 1.0))
    theta = float(np.arccos(cos))
    if theta < 1e-8:
        # 一阶近似即可，反对称部分本身就是 θ·axis
        return np.array([R[2, 1] - R[1, 2],
                         R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]]) * 0.5
    if np.pi - theta < 1e-6:
        # θ≈π 时反对称部分退化，改从 R+I 的列取轴
        A = 0.5 * (R + np.eye(3))
        axis = np.sqrt(np.clip(np.diag(A), 0.0, None))
        k = int(np.argmax(axis))
        if axis[k] < 1e-12:
            return np.zeros(3)
        axis = A[:, k] / axis[k]
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        return theta * axis
    return (theta / (2.0 * np.sin(theta))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def _exp_so3(w: np.ndarray) -> np.ndarray:
    """轴角向量 -> 旋转矩阵（Rodrigues）。"""
    w = np.asarray(w, dtype=float).reshape(3)
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)
    K = _hat(w / theta)
    return (np.eye(3) + np.sin(theta) * K
            + (1.0 - np.cos(theta)) * (K @ K))


def _minimal_rotvec(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """把单位向量 source 转到 target 的【最小】旋转，返回轴角向量。

    绕 source 自身的自转分量为零 —— 这正是 5D 笔轴任务需要的口径。
    """
    s = np.asarray(source, dtype=float).reshape(3)
    t = np.asarray(target, dtype=float).reshape(3)
    ns, nt = np.linalg.norm(s), np.linalg.norm(t)
    if ns < 1e-12 or nt < 1e-12:
        return np.zeros(3)
    s = s / ns
    t = t / nt
    cross = np.cross(s, t)
    dot = float(np.clip(np.dot(s, t), -1.0, 1.0))
    sin_theta = float(np.linalg.norm(cross))
    if sin_theta < 1e-12:
        if dot > 0.0:
            return np.zeros(3)
        # 反向：绕任一垂直轴转 π
        axis = np.cross(s, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(s, np.array([0.0, 1.0, 0.0]))
        return np.pi * axis / np.linalg.norm(axis)
    return np.arctan2(sin_theta, dot) * cross / sin_theta


class HandoverBlend:
    """锁定交接时刻的位姿偏置，并按 smoothstep 把它衰减掉。

    用法::

        blend = HandoverBlend(duration_sec=3.0)
        blend.capture(t0, actual_position, actual_rotation,
                      ref_position, ref_rotation)      # 交接那一帧调一次
        p, R, v, w, a, wd = blend.apply(t, ref_...)    # 之后每帧调

    ``duration_sec <= 0`` 时整个类是恒等变换，capture / apply 都不改任何量，
    已有仿真基线因此逐字节不变。
    """

    def __init__(self, duration_sec: float = 0.0) -> None:
        duration_sec = float(duration_sec)
        if not np.isfinite(duration_sec) or duration_sec < 0.0:
            raise ValueError(
                f"duration_sec 必须是非负有限值，收到 {duration_sec}")
        self.duration_sec = duration_sec
        self.t0: float | None = None
        self.delta_position = np.zeros(3)
        self.delta_rotvec = np.zeros(3)
        self.delta_velocity = np.zeros(3)

    @property
    def enabled(self) -> bool:
        return self.duration_sec > 0.0

    @property
    def captured(self) -> bool:
        return self.t0 is not None

    def capture(self, t0, actual_position, actual_rotation,
                reference_position, reference_rotation,
                pen_axis_local=None,
                actual_linear_velocity=None,
                reference_linear_velocity=None) -> None:
        """在交接那一帧锁定偏置。重复调用会被忽略（偏置必须是常量）。

        ``pen_axis_local`` 给定时按【5D 笔轴任务】口径取姿态偏置：只补偿笔轴
        指向之差，绕笔轴的自转不补偿。

        ⚠️ 这一条是必须的。5D 任务本来就放弃了绕笔轴的自转，实际位姿与参考
        位姿之间那一份自转是【自由量】，不是误差。若按 6D 取 R_act·R_refᵀ，
        会把这份自由自转当成偏置塞进参考量，再让它在 blend_sec 内衰减 ——
        等于凭空造出一个几十度每秒的角速度参考。实测：approach 已收敛到
        0.04 mm 时，6D 口径报出的姿态偏置是 **120.8°**，而笔轴其实早就对齐了。
        """
        if self.captured:
            return
        self.t0 = float(t0)
        if not self.enabled:
            return
        p_act = np.asarray(actual_position, dtype=float).reshape(3)
        p_ref = np.asarray(reference_position, dtype=float).reshape(3)
        R_act = np.asarray(actual_rotation, dtype=float).reshape(3, 3)
        R_ref = np.asarray(reference_rotation, dtype=float).reshape(3, 3)
        self.delta_position = p_act - p_ref
        if (actual_linear_velocity is not None
                and reference_linear_velocity is not None):
            self.delta_velocity = (
                np.asarray(actual_linear_velocity, dtype=float).reshape(3)
                - np.asarray(reference_linear_velocity, dtype=float).reshape(3))
        if pen_axis_local is None:
            self.delta_rotvec = _log_so3(R_act @ R_ref.T)
        else:
            a = np.asarray(pen_axis_local, dtype=float).reshape(3)
            self.delta_rotvec = _minimal_rotvec(R_ref @ a, R_act @ a)

    def scale(self, t) -> float:
        """当前的衰减系数 s(τ)，只用于日志。"""
        if not self.enabled or not self.captured:
            return 0.0
        return smoothstep_decay(float(t) - self.t0, self.duration_sec)[0]

    def apply(self, t, position, rotation, linear_velocity, angular_velocity,
              linear_acceleration, angular_acceleration):
        """把偏置叠加到参考量上，返回混合后的 6 个量。

        未启用、未 capture、或 τ 已超出 duration 时原样返回。
        """
        p = np.asarray(position, dtype=float).reshape(3)
        R = np.asarray(rotation, dtype=float).reshape(3, 3)
        v = np.asarray(linear_velocity, dtype=float).reshape(3)
        w = np.asarray(angular_velocity, dtype=float).reshape(3)
        a = np.asarray(linear_acceleration, dtype=float).reshape(3)
        wd = np.asarray(angular_acceleration, dtype=float).reshape(3)

        if not self.enabled or not self.captured:
            return p, R, v, w, a, wd
        tau = float(t) - self.t0
        s, sd, sdd = smoothstep_decay(tau, self.duration_sec)
        bv, bvd, bvdd = velocity_decay(tau, self.duration_sec)
        if (s == 0.0 and sd == 0.0 and sdd == 0.0
                and bv == 0.0 and bvd == 0.0 and bvdd == 0.0):
            return p, R, v, w, a, wd

        # o(τ) = Δp·A(τ) + Δv·B(τ)：τ=0 处位置【和速度】误差同时为零
        p_out = p + s * self.delta_position + bv * self.delta_velocity
        v_out = v + sd * self.delta_position + bvd * self.delta_velocity
        a_out = a + sdd * self.delta_position + bvdd * self.delta_velocity

        R_off = _exp_so3(s * self.delta_rotvec)
        R_out = R_off @ R
        # ω_traj 要被 R_off 旋转后才能相加，推导见模块 docstring
        w_rot = R_off @ w
        w_out = sd * self.delta_rotvec + w_rot
        wd_out = (sdd * self.delta_rotvec
                  + sd * np.cross(self.delta_rotvec, w_rot)
                  + R_off @ wd)
        return p_out, R_out, v_out, w_out, a_out, wd_out

    def peak_extra_acceleration(self) -> float:
        """偏置衰减自身贡献的加速度峰值 [m/s²]，用于启动时的安全播报。

        五次 smoothstep 的 |s''| 峰值是 10√3/3 / T² ≈ 5.7735 / T²。
        """
        if not self.enabled:
            return 0.0
        # |A''|max = 10√3/3 / T² ≈ 5.7735/T²
        # |B''|max·T：B''(u)·T = -36u + 96u² - 60u³，驻点 45u²-48u+9=0
        #   ⟹ u = 0.2428 / 0.8239，取到 |−3.9403| / |1.9403| ⟹ 3.9403/T
        return (10.0 * np.sqrt(3.0) / 3.0
                * float(np.linalg.norm(self.delta_position))
                / (self.duration_sec ** 2)
                + 3.9403 * float(np.linalg.norm(self.delta_velocity))
                / self.duration_sec)
