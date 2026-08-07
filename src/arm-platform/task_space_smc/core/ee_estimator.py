"""末端反馈：控制器认为工具实际在哪。

把编码器正运动学（FK）与 Vicon 的 arm_ee 刚体位置融合，输出控制点的位姿与速度。
融合方式由两条实测事实决定：

**arm_ee 的姿态不可用。** 它的坐标轴与 link6 相差 114.39°（一次标定）/ 113.54°
（另一次）。常量本身重复到 0.85°，但扣掉常量后的残差一次是 9.97°、另一次 43.55°
—— 单个固定旋转解释不了这个通道。根因记在 config/vicon_extrinsics.yaml：joint6 的
零位偏置是 arm6_calibration.yaml 里唯一未标定项，且与标记簇的 roll 在数学上不可分离。
所以【所有模式】的姿态一律取自 FK，可选的旋转外参只影响记录下来的残差，不进控制。

**Vicon 位置有延迟。** 本机实测传输延迟均值 171 ms、p95 432 ms、最大 964 ms，而控制
回路跑 200 Hz、等效位置增益 k·λ = 144 /m。直接进环等于对五分之一秒前的位置做反应。
故默认模式让 FK 承担高频，Vicon 只用来修一个缓变偏置：

    p = p_fk + bias,   bias 以 bias_bandwidth_hz 的一阶低通跟踪 (p_vicon − p_fk)

稳态下位置仍精确收敛到 Vicon。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

FK_ONLY = "fk_only"
VICON_DIRECT = "vicon_direct"
VICON_BIAS_CORRECTED = "vicon_bias_corrected"
MODES = (FK_ONLY, VICON_DIRECT, VICON_BIAS_CORRECTED)

# 速度来源，记录的是【每拍的实际结果】而非配置回显
FK = "fk"
VICON = "vicon"
FK_STALE = "fk_stale"           # 配了 vicon，但样本超期、无数据，或位置新息已被拒
FK_FALLBACK = "fk_rejected"     # 配了 vicon，样本也新鲜，但与 FK 粗量级不符
VELOCITY_SOURCES = (FK, VICON)


def skew(v) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


@dataclass
class EeExtrinsic:
    """``link6 -> arm_ee`` 标记簇外参。

    rotation 默认为 None 即关闭（见模块头部：姿态通道不可用）。给了旋转也只改变
    记录的姿态残差怎么算，不进控制。
    """

    translation_m: np.ndarray
    rotation: np.ndarray | None = None

    def __post_init__(self):
        self.translation_m = np.asarray(self.translation_m, dtype=float).reshape(3)
        if not np.all(np.isfinite(self.translation_m)):
            raise ValueError("arm_ee_from_link6.translation_m 必须是有限值")
        if self.rotation is not None:
            self.rotation = np.asarray(self.rotation, dtype=float).reshape(3, 3)


@dataclass
class EeEstimate:
    position: np.ndarray
    rotation: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    source: str
    bias: np.ndarray
    vicon_age: float
    fk_position: np.ndarray
    vicon_position: np.ndarray | None
    position_residual: np.ndarray
    orientation_residual_rad: float
    degraded: bool
    degrade_reason: str
    hold_required: bool = False
    # 两条速度通道各自实际用了谁。用字符串而非布尔，因为回退有两种成因
    # （没数据 / 有数据但对照没过），验收时要分得开。
    linear_velocity_source: str = FK
    angular_velocity_source: str = FK
    # FK 与 Vicon 两路速度逐拍都留下来，供事后对照 Vicon 速度的尺度与坐标系。
    fk_linear_velocity: np.ndarray | None = None
    fk_angular_velocity: np.ndarray | None = None
    vicon_linear_velocity: np.ndarray | None = None
    vicon_angular_velocity: np.ndarray | None = None
    vicon_angular_accel: np.ndarray | None = None


def control_point_jacobian(J6, rotation, translation_local) -> np.ndarray:
    """把 LOCAL_WORLD_ALIGNED 雅可比从 link6 平移到标记簇原点：

        J_p' = J_p − [r]×·J_w ,   J_w' = J_w ,   r = R·t_local

    点平移中依赖 qdd 的部分恰好是 −[r]×·J_w·qdd，故这是精确变换而非小偏移近似。
    控制器与启动奇异门必须用同一个雅可比：17.9 mm 的偏移在静态检查里看不出来，
    一旦转起来就不是了。
    """
    J6 = np.asarray(J6, dtype=float)
    r = np.asarray(rotation, dtype=float).reshape(3, 3) @ \
        np.asarray(translation_local, dtype=float).reshape(3)
    out = J6.copy()
    out[:3, :] = J6[:3, :] - skew(r) @ J6[3:, :]
    return out


def control_point_bias(bias6, omega, rotation, translation_local) -> np.ndarray:
    """偏移点的漂移加速度：

        b_p' = b_p + b_w × r + ω × (ω × r)

    向心项在角速度恒定时依然存在，漏掉它产生的误差是任何静态测试都看不到的。
    """
    bias6 = np.asarray(bias6, dtype=float).reshape(6)
    omega = np.asarray(omega, dtype=float).reshape(3)
    r = np.asarray(rotation, dtype=float).reshape(3, 3) @ \
        np.asarray(translation_local, dtype=float).reshape(3)
    out = bias6.copy()
    out[:3] = bias6[:3] + np.cross(bias6[3:], r) + np.cross(omega, np.cross(omega, r))
    return out


# 两个姿态之间的测地角 θ = arccos((tr(Raᵀ·Rb) − 1)/2)
def _geodesic_angle(Ra, Rb) -> float:
    R = np.asarray(Ra, dtype=float).T @ np.asarray(Rb, dtype=float)
    return float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


@dataclass
class EeFeedbackConfig:
    mode: str = VICON_BIAS_CORRECTED  # fk_only=只用 FK / vicon_direct=直接用 Vicon 位置 / vicon_bias_corrected=FK 走高频、Vicon 只修慢变偏置
    apply_control_point_offset: bool = True  # 把控制点从 link6 搬到标记簇原点。关掉则 17.9 mm 力臂误差直接进力矩
    # 线速度与角速度分开选源
    velocity_source: str = FK
    angular_velocity_source: str = FK
    # 对照容差，满足 |v_vicon − v_fk| <= tol · max(|v_fk|, floor) 才采信 Vicon。
    # 取相对量是因为 FK 速度本身随运动幅度跨两个数量级，floor 保证静止时不会
    # 因除以零把任何值都判成不符。这道闸只抓粗错（尺度、坐标系、刚体），不做精度判据。
    velocity_cross_check_tol: float = 0.5      # 线速度相对容差 tol。大→几乎全放行、闸门失效；小→几乎全拒、退化成纯 FK
    velocity_cross_check_floor_m_s: float = 0.05   # 线速度绝对底噪 (m/s)，避免静止时除以 ~0。大→低速段放行更宽；小→静止噪声就触发拒绝
    angular_cross_check_tol: float = 0.5       # 角速度相对容差，同上
    angular_cross_check_floor_rad_s: float = 0.2   # 角速度绝对底噪 (rad/s)
    bias_bandwidth_hz: float = 0.2   # 偏置跟踪带宽 f_c (Hz)，α = 1−exp(−2π·f_c·dt)。大→偏置跟得快但把 Vicon 延迟引进环；小→稳态收敛慢
    bias_init: str = "first_sample"  # first_sample=首帧一次性置好。改成从零渐进则 hold 捕获在 FK 点，偏置长到真值的过程会把臂推走几毫米
    max_bias_m: float = 0.02         # 偏置幅值上限 (m)。实测残差 1.77 mm RMS，20 mm 已是错配量级。大→错配也被当偏置吸收；小→真实安装偏差修不完
    max_bias_rate_m_s: float = 0.01  # 偏置变化率上限 (m/s)。大→跳变直接传给控制器；小→跟不上真实漂移
    innovation_reject_m: float = 0.05    # 新息拒绝阈 (m)，超过即判为错刚体/错外参。大→标记簇错配也被吸收；小→正常噪声就误拒
    innovation_reject_hold_sec: float = 0.5  # 新息持续超阈多久判降级 (s)。大→偶发跳点不误判；小→反应快但易误报
    max_age_sec: float = 0.15        # Vicon 样本可用年龄上限 (s)。大→拿更旧的位置修偏置；小→频繁退回 FK
    hold_after_sec: float = 1.0      # 超过此年龄进入保位 (s)。大→更久拿旧数据硬撑；小→链路一抖就保位
    require_vicon_at_capture: bool = True  # 捕获保持目标时必须已收到有效 arm_ee 样本，否则拒绝启动（避免静默退化成 fk_only 把目标钉错）


class EeFeedbackEstimator:
    """把编码器正运动学与 Vicon 的 arm_ee 位置融合成末端反馈。"""

    def __init__(self, config: EeFeedbackConfig, extrinsic: EeExtrinsic | None):
        if config.mode not in MODES:
            raise ValueError("未知的 ee_feedback.mode: %r（可选 %s）"
                             % (config.mode, ", ".join(MODES)))
        for name in ("velocity_source", "angular_velocity_source"):
            value = getattr(config, name)
            if value not in VELOCITY_SOURCES:
                # 来源名打错必须报错：静默走 FK 分支属于「配了却没生效」，
                # 只有等数据回来才会发现。
                raise ValueError("未知的 ee_feedback.%s: %r（可选 %s）"
                                 % (name, value, ", ".join(VELOCITY_SOURCES)))
        if config.mode != FK_ONLY and extrinsic is None:
            # 外参缺失是硬错误，不静默退化成单位阵（与 launch 里 load_base_extrinsic
            # 同一条原则）：一个「还能跑」的错世界系只会在机器人身上暴露。
            raise ValueError(
                "ee_feedback.mode=%s 需要 vicon_extrinsics.yaml 的 "
                "arm_ee_from_link6.translation_m，但没拿到" % config.mode)
        self.config = config
        self.extrinsic = extrinsic
        self.bias = np.zeros(3)
        self.bias_initialised = False
        self.has_seen_vicon = False
        self._reject_since: float | None = None
        # 回退计数：否则「配了 vicon 但整段都在走 FK」从数据上看不出来。
        self.linear_velocity_rejects = 0
        self.angular_velocity_rejects = 0
        self.linear_velocity_stale = 0
        self.angular_velocity_stale = 0

    # -------------------------------------------------- helpers

    def _offset(self) -> np.ndarray:
        if self.extrinsic is None or not self.config.apply_control_point_offset:
            return np.zeros(3)
        return self.extrinsic.translation_m

    def control_point_position(self, fk_position, fk_rotation) -> np.ndarray:
        fk_position = np.asarray(fk_position, dtype=float).reshape(3)
        R = np.asarray(fk_rotation, dtype=float).reshape(3, 3)
        return fk_position + R @ self._offset()

    def control_point_velocity(self, fk_linear, fk_angular, fk_rotation) -> np.ndarray:
        fk_linear = np.asarray(fk_linear, dtype=float).reshape(3)
        omega = np.asarray(fk_angular, dtype=float).reshape(3)
        R = np.asarray(fk_rotation, dtype=float).reshape(3, 3)
        return fk_linear + np.cross(omega, R @ self._offset())

    def jacobian(self, J6, fk_rotation) -> np.ndarray:
        if self.extrinsic is None or not self.config.apply_control_point_offset:
            return np.asarray(J6, dtype=float)
        return control_point_jacobian(J6, fk_rotation, self.extrinsic.translation_m)

    def bias_acceleration(self, bias6, omega, fk_rotation) -> np.ndarray:
        if self.extrinsic is None or not self.config.apply_control_point_offset:
            return np.asarray(bias6, dtype=float).reshape(6)
        return control_point_bias(bias6, omega, fk_rotation,
                                  self.extrinsic.translation_m)

    # -------------------------------------------------- velocity selection

    def _select_velocity(self, name, fk_value, vicon_value, fresh, tol, floor):
        """在 FK 与 Vicon 之间二选一，返回 ``(value, source)``。

        Vicon 只有【新鲜】且【与 FK 粗量级相符】时才被采用：

            |v_vicon − v_fk| <= tol · max(|v_fk|, floor)

        两个条件的失败分开计数，它们指向不同的问题（链路 vs 单位/坐标系）。
        """
        if getattr(self.config, name) != VICON:
            return fk_value, FK
        if vicon_value is None or not fresh:
            if name == "velocity_source":
                self.linear_velocity_stale += 1
            else:
                self.angular_velocity_stale += 1
            return fk_value, FK_STALE
        candidate = np.asarray(vicon_value, dtype=float).reshape(3)
        if not np.all(np.isfinite(candidate)):
            return fk_value, FK_STALE
        gap = float(np.linalg.norm(candidate - fk_value))
        scale = max(float(np.linalg.norm(fk_value)), float(floor))
        if gap > float(tol) * scale:
            if name == "velocity_source":
                self.linear_velocity_rejects += 1
            else:
                self.angular_velocity_rejects += 1
            return fk_value, FK_FALLBACK
        return candidate, VICON

    # -------------------------------------------------- the update

    def update(
        self,
        now: float,
        fk_position,
        fk_rotation,
        fk_linear_velocity,
        fk_angular_velocity,
        dt: float,
        vicon_position=None,
        vicon_stamp: float | None = None,
        vicon_rotation=None,
        vicon_linear_velocity=None,
        vicon_angular_velocity=None,
        vicon_angular_accel=None,
    ) -> EeEstimate:
        cfg = self.config
        R_fk = np.asarray(fk_rotation, dtype=float).reshape(3, 3)
        p_fk = self.control_point_position(fk_position, R_fk)
        v_fk = self.control_point_velocity(fk_linear_velocity, fk_angular_velocity, R_fk)
        omega_fk = np.asarray(fk_angular_velocity, dtype=float).reshape(3)

        p_vicon = None
        age = float("inf")
        if vicon_position is not None and vicon_stamp is not None:
            candidate = np.asarray(vicon_position, dtype=float).reshape(3)
            if np.all(np.isfinite(candidate)) and math.isfinite(vicon_stamp):
                p_vicon = candidate
                age = float(now - vicon_stamp)
                self.has_seen_vicon = True

        fresh = p_vicon is not None and 0.0 <= age <= cfg.max_age_sec
        residual = (p_vicon - p_fk) if p_vicon is not None else np.zeros(3)

        orientation_residual = float("nan")
        if vicon_rotation is not None:
            R_vicon = np.asarray(vicon_rotation, dtype=float).reshape(3, 3)
            reference = R_fk if self.extrinsic is None or self.extrinsic.rotation is None \
                else R_fk @ self.extrinsic.rotation
            orientation_residual = _geodesic_angle(reference, R_vicon)

        degraded, reason, hold = False, "", False

        # 新息超过 innovation_reject_m 即判为错刚体或错外参，直接拒收不吸收进偏置
        if fresh and float(np.linalg.norm(residual - self.bias)) > cfg.innovation_reject_m:
            if self._reject_since is None:
                self._reject_since = now
            elif now - self._reject_since >= cfg.innovation_reject_hold_sec:
                degraded, hold = True, True
                reason = "arm_ee 与 FK 残差持续超过 %.0f mm" % (cfg.innovation_reject_m * 1e3)
            fresh = False
        else:
            self._reject_since = None

        if cfg.mode != FK_ONLY:
            if p_vicon is None and not self.has_seen_vicon:
                degraded = True
                reason = reason or "尚未收到任何 arm_ee 样本"
            elif age > cfg.hold_after_sec:
                degraded, hold = True, True
                reason = reason or "arm_ee 数据年龄 %.3f s 超过 %.2f s" % (age, cfg.hold_after_sec)
            elif not fresh and not degraded:
                degraded = True
                reason = "arm_ee 数据年龄 %.3f s 超过 %.2f s，偏置冻结" % (age, cfg.max_age_sec)

        # ---- 偏置更新。只有偏置被滤波，FK 以全带宽进入：
        #        α = 1 − exp(−2π·f_c·dt),   bias ← bias + clip(α·(residual − bias), ±rate·dt)
        if cfg.mode == VICON_BIAS_CORRECTED and fresh:
            if not self.bias_initialised and cfg.bias_init == "first_sample":
                # 首帧一次性置好。若从零渐进，hold 会捕获在 FK 点上，偏置长到真值的
                # 过程会把臂推走一段等于初始残差的距离。
                self.bias = residual.copy()
                self.bias_initialised = True
            else:
                tau = max(cfg.bias_bandwidth_hz, 0.0)
                alpha = 0.0 if tau <= 0.0 else 1.0 - math.exp(-2.0 * math.pi * tau * max(dt, 0.0))
                target = self.bias + alpha * (residual - self.bias)
                delta = np.clip(target - self.bias,
                                -cfg.max_bias_rate_m_s * max(dt, 0.0),
                                cfg.max_bias_rate_m_s * max(dt, 0.0))
                self.bias = self.bias + delta
                self.bias_initialised = True
            norm = float(np.linalg.norm(self.bias))
            if norm > cfg.max_bias_m > 0.0:
                self.bias = self.bias * (cfg.max_bias_m / norm)

        # ---- assemble
        if cfg.mode == FK_ONLY:
            position, source = p_fk, FK_ONLY
        elif cfg.mode == VICON_DIRECT:
            if fresh:
                position, source = p_vicon, VICON_DIRECT
            else:
                position, source = p_fk, "fk_fallback"
        else:
            position, source = p_fk + self.bias, VICON_BIAS_CORRECTED

        # ---- 速度是独立的一路选择：位置可以慢（偏置带宽 0.2 Hz 就够），速度不行，
        # 它直接乘 SMC 的阻尼项进力矩，所以只有「新鲜且对照通过」才用 Vicon，没有
        # 中间形态。此处的 fresh 已含新息拒绝：位置被判为错刚体时，它报的速度同样不可信。
        v_vicon = (np.asarray(vicon_linear_velocity, dtype=float).reshape(3)
                   if vicon_linear_velocity is not None else None)
        w_vicon = (np.asarray(vicon_angular_velocity, dtype=float).reshape(3)
                   if vicon_angular_velocity is not None else None)
        a_vicon = (np.asarray(vicon_angular_accel, dtype=float).reshape(3)
                   if vicon_angular_accel is not None else None)

        linear, linear_source = self._select_velocity(
            "velocity_source", v_fk, v_vicon, fresh,
            cfg.velocity_cross_check_tol, cfg.velocity_cross_check_floor_m_s)
        angular, angular_source = self._select_velocity(
            "angular_velocity_source", omega_fk, w_vicon, fresh,
            cfg.angular_cross_check_tol, cfg.angular_cross_check_floor_rad_s)
        if linear_source == VICON:
            source = source + "+vicon_velocity"
        if angular_source == VICON:
            source = source + "+vicon_omega"

        return EeEstimate(
            position=position,
            rotation=R_fk,                    # 姿态一律来自 FK，见模块头部
            linear_velocity=linear,
            angular_velocity=angular,
            source=source,
            bias=self.bias.copy(),
            vicon_age=age,
            fk_position=p_fk,
            vicon_position=p_vicon,
            position_residual=residual,
            orientation_residual_rad=orientation_residual,
            degraded=degraded,
            degrade_reason=reason,
            hold_required=hold,
            linear_velocity_source=linear_source,
            angular_velocity_source=angular_source,
            fk_linear_velocity=v_fk,
            fk_angular_velocity=omega_fk,
            vicon_linear_velocity=v_vicon,
            vicon_angular_velocity=w_vicon,
            # 角加速度只记录不使用：控制回路的输入是基座加速度
            # （compute_task_bias_acceleration），末端角加速度不进环。
            vicon_angular_accel=a_vicon,
        )

    def ready_for_capture(self) -> bool:
        """能否安全地在这一拍捕获保持目标。

        没收到过任何有效 arm_ee 样本却按 vicon_* 模式启动，等于静默退化成 fk_only
        并把目标钉在错的地方，故宁可拒绝启动。
        """
        if self.config.mode == FK_ONLY:
            return True
        if not self.config.require_vicon_at_capture:
            return True
        if self.config.mode == VICON_BIAS_CORRECTED:
            return self.bias_initialised
        return self.has_seen_vicon
