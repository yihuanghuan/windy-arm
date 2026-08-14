from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from task_space_smc.core.leso import JointLESO
from task_space_smc.core.smith_predictor import (
    DEFAULT_MAX_POSITION_CORRECTION,
    DEFAULT_MAX_VELOCITY_CORRECTION,
    JointStatePredictor,
    SmithPredictorConfig,
)

__all__ = [
    "JointStateEstimator",
    "EstimatorPlan",
    "resolve_estimator",
    "VELOCITY_SOURCES",
    "POSITION_SOURCES",
    "PREDICT_SOURCES",
    "LESO_INPUTS",
    "MODEL_B_MIN_OMEGA_RAD_S",
    "MODEL_B_MIN_BANDWIDTH_HZ",
    "ESTIMATOR_KEYS",
]

VELOCITY_SOURCES = ("motor", "finite_difference", "leso")
POSITION_SOURCES = ("measured", "leso")
PREDICT_SOURCES = ("commanded_acceleration", "acceleration_estimate")
LESO_INPUTS = ("none", "commanded_acceleration")


MODEL_B_MIN_OMEGA_RAD_S = 120.0
MODEL_B_MIN_BANDWIDTH_HZ = MODEL_B_MIN_OMEGA_RAD_S / (2.0 * np.pi)

ESTIMATOR_KEYS = (
    "source",
    "position_source",
    "predict_position_sec",
    "predict_velocity_sec",
    "predict_source",
    "beta_position",
    "beta_velocity",
    "predict_clamp_rad",
    "predict_clamp_rad_s",
    "predict_fallback_ticks",
    "hold_on_invalid",
    "leso",
)
_LESO_KEYS = ("bandwidth_hz", "input", "b0", "input_delay_sec")


def _per_joint_choice(value, n, name, allowed):
    if isinstance(value, str):
        items = [value] * int(n)
    else:
        items = list(value)
        if len(items) == 1:
            items = items * int(n)
    if len(items) != int(n):
        raise ValueError(f"{name} 需要 {n} 个值或 1 个标量，收到 {len(items)}")
    out = []
    for i, item in enumerate(items):
        s = str(item).strip().lower()
        if s not in allowed:
            raise ValueError(
                f"{name}[{i}] = {item!r} 不是合法取值，可选 {list(allowed)}")
        out.append(s)
    return tuple(out)


def _as_vector(value, n):
    arr = np.asarray(value, dtype=float)
    return arr if arr.shape == (n,) else arr.reshape(n)


def _per_joint_float(value, n, name, minimum=None):
    arr = np.asarray(value, dtype=float).ravel()
    if arr.size == 1:
        arr = np.full(int(n), float(arr[0]))
    if arr.size != int(n):
        raise ValueError(f"{name} 需要 {n} 个值或 1 个标量，收到 {arr.size}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} 含非有限值: {arr}")
    if minimum is not None and np.any(arr < minimum):
        raise ValueError(f"{name} 不能小于 {minimum}: {arr}")
    return arr


# 关节状态估计器：把电机反馈的 (q, q̇) 加工成控制器实际使用的状态。三段串联：
#   1. LESO 更新，得到位置估计 z1、速度估计 z2、总扰动估计 z3（见 leso.py）
#   2. 逐关节挑速度来源 —— motor 直通电机自报值；finite_difference 取位置后向差分
#      q̇ = (q_k − q_{k−1})/dt；leso 取观测器的 z2
#   3. Smith 预测，用已发出的指令加速度把状态推到当前时刻，补反馈链路延时
#      （见 smith_predictor.py）
# 位置默认取量测值，position_source = leso 时改用 z1。
# 反馈含 NaN/Inf 时按 hold_on_invalid 保持上一拍输出，不把非有限值传给控制律。
class JointStateEstimator:

    def __init__(
        self,
        n_joints: int,
        control_period: float,
        *,
        velocity_source=None,
        position_source: str = "measured",
        leso: JointLESO | None = None,
        predictor: JointStatePredictor | None = None,
        predict_source: str = "commanded_acceleration",
        leso_input: str = "none",
        leso_input_delay_sec: float = 0.0,
        hold_on_invalid: bool = True,
    ) -> None:
        n = int(n_joints)
        if n <= 0:
            raise ValueError(f"n_joints 必须为正，收到 {n_joints}")
        control_period = float(control_period)
        if not np.isfinite(control_period) or control_period <= 0.0:
            raise ValueError(f"control_period 必须为正，收到 {control_period}")
        self.n_joints = n
        self.control_period = control_period

        self.velocity_source = tuple(
            velocity_source if velocity_source is not None else ("motor",) * n)
        if len(self.velocity_source) != n:
            raise ValueError(
                f"velocity_source 需要 {n} 项，收到 {len(self.velocity_source)}")
        for s in self.velocity_source:
            if s not in VELOCITY_SOURCES:
                raise ValueError(f"未知的速度来源 {s!r}，可选 {list(VELOCITY_SOURCES)}")
        self.position_source = str(position_source)
        if self.position_source not in POSITION_SOURCES:
            raise ValueError(
                f"未知的位置来源 {self.position_source!r}，可选 {list(POSITION_SOURCES)}")
        self.predict_source = str(predict_source)
        if self.predict_source not in PREDICT_SOURCES:
            raise ValueError(
                f"未知的 predict_source {self.predict_source!r}，可选 {list(PREDICT_SOURCES)}")
        self.leso_input = str(leso_input)
        if self.leso_input not in LESO_INPUTS:
            raise ValueError(
                f"未知的 leso.input {self.leso_input!r}，可选 {list(LESO_INPUTS)}")

        self.leso = leso if leso is not None else JointLESO(
            np.zeros(n), n, nominal_dt=control_period)

        self._fd_mask = np.array(
            [s == "finite_difference" for s in self.velocity_source])


        self._leso_mask = (np.array([s == "leso" for s in self.velocity_source])
                           & (self.leso.omega_o > 0.0))
        self.predictor = predictor if predictor is not None else JointStatePredictor(
            SmithPredictorConfig(n))
        self.hold_on_invalid = bool(hold_on_invalid)

        self.leso_input_delay_sec = float(leso_input_delay_sec)
        if (not np.isfinite(self.leso_input_delay_sec)
                or self.leso_input_delay_sec < 0.0):
            raise ValueError(
                f"leso.input_delay_sec 必须是非负有限值，收到 {leso_input_delay_sec}")


        depth = int(self.leso_input_delay_sec / control_period + 0.5) + 8
        self._cmd_buf: deque = deque(maxlen=max(depth, 8))


        self._prev_q = None
        self._last_q_out = None
        self._last_qd_out = None


        self.hold_count = 0
        self.predict_clamp_hits = 0
        self.updates = 0


    @property
    def velocity_active(self) -> bool:
        return bool(self._fd_mask.any() or self._leso_mask.any())

    @property
    def position_from_leso(self) -> bool:
        return self.position_source == "leso" and bool(self._leso_mask.any())

    @property
    def predict_active(self) -> bool:
        return bool(self.predictor.config.active)

    @property
    def enabled(self) -> bool:
        return bool(self.velocity_active or self.position_from_leso
                    or self.predict_active)

    @property
    def leso_input_active(self) -> bool:
        return (self.leso_input == "commanded_acceleration"
                and bool(self._leso_mask.any()))

    @property
    def wants_commanded_acceleration(self) -> bool:
        return bool(
            self.leso_input_active
            or (self.predict_active
                and self.predict_source == "commanded_acceleration"))

    @property
    def acceleration_estimate(self) -> np.ndarray:
        out = np.full(self.n_joints, np.nan)
        if self._leso_mask.any():
            out[self._leso_mask] = self.leso.disturbance_estimate[self._leso_mask]
        return out


    def reset(self) -> None:
        self.leso.reset()
        self.predictor.reset()
        self._cmd_buf.clear()
        self._prev_q = None
        self._last_q_out = None
        self._last_qd_out = None


    def push_command(self, qdd_cmd, dt: float) -> None:
        if not self.wants_commanded_acceleration:
            return
        d = float(dt)
        if not np.isfinite(d) or d <= 0.0:

            return
        a = np.asarray(qdd_cmd, dtype=float).reshape(self.n_joints)
        if not np.all(np.isfinite(a)):
            return
        if self.leso_input_active:
            self._cmd_buf.append((a.copy(), d))
        if self.predict_active and self.predict_source == "commanded_acceleration":
            self.predictor.push_command(a, d)

    # LESO 的模型输入 u 要与「这条指令真正体现成关节加速度」的时刻对齐，否则 b0·u
    # 加在了错误的时刻上。从指令缓冲末尾往回累计时长，取首次凑满 input_delay_sec 的那一拍。
    def _aligned_command(self):
        if not self._cmd_buf:
            return None
        lag = self.leso_input_delay_sec
        if lag <= 0.0:
            return self._cmd_buf[-1][0]
        elapsed = 0.0
        for a, d in reversed(self._cmd_buf):
            elapsed += d
            if elapsed >= lag:
                return a
        return self._cmd_buf[0][0]


    def _sanitized_dt(self, dt):
        if dt is None:
            return self.control_period
        d = float(dt)
        if not np.isfinite(d) or d <= 0.0:
            return self.control_period
        return d

    def __call__(self, position, velocity, dt: float | None = None):
        if not self.enabled:
            return position, velocity

        n = self.n_joints
        q = _as_vector(position, n)
        qd = _as_vector(velocity, n)

        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd))):

            self.hold_count += 1
            if self.hold_on_invalid and self._last_q_out is not None:
                return self._last_q_out.copy(), self._last_qd_out.copy()
            return position, velocity

        step = self._sanitized_dt(dt)


        step_fd = float(np.clip(step, self.leso.dt_lo, self.leso.dt_hi))


        u = self._aligned_command() if self.leso_input_active else None
        qd_est = self.leso(q, qd, step, u)


        if self._fd_mask.any():
            qd_est = np.array(qd_est, dtype=float)
            if self._prev_q is not None:
                qd_est[self._fd_mask] = (
                    (q[self._fd_mask] - self._prev_q[self._fd_mask]) / step_fd)


        q_est = q
        if self.position_from_leso:
            q_est = q.copy()
            q_est[self._leso_mask] = self.leso.position_estimate[self._leso_mask]


        if self.predict_active and self.predict_source == "acceleration_estimate":
            acc = np.zeros(n)
            if self._leso_mask.any():
                acc[self._leso_mask] = (
                    self.leso.disturbance_estimate[self._leso_mask])
                if self.leso_input_active and u is not None:
                    u_arr = np.asarray(u, dtype=float).reshape(n)
                    acc[self._leso_mask] += (
                        self.leso.b0[self._leso_mask] * u_arr[self._leso_mask])
            self.predictor.push_command(acc, step)


        q_out, qd_out = self.predictor.predict(q_est, qd_est)
        if self.predictor.last_diagnostics.get("clamped", 0.0):
            self.predict_clamp_hits += 1

        self._prev_q = q.copy()
        self._last_q_out = np.asarray(q_out, dtype=float).copy()
        self._last_qd_out = np.asarray(qd_out, dtype=float).copy()
        self.updates += 1
        return q_out, qd_out


    def describe(self) -> str:
        if not self.enabled:
            return "关节状态估计器: 全部关闭（逐位直通，= 路线 A 电机自报速度基线）"
        parts = ["速度源 %s" % (list(self.velocity_source),)]
        if self._leso_mask.any():
            w = self.leso.omega_o
            parts.append(
                "LESO ω_o=%s rad/s (%s Hz)，24.8 Hz 相位 %s°（对照：电机自报那多出来的 15 ms 是 133.9°，裸差分半拍 22.3°），input=%s"
                % (np.round(w, 1).tolist(),
                   np.round(w / (2 * np.pi), 2).tolist(),
                   np.round(self.leso.phase_lag_deg(24.8), 1).tolist(),
                   self.leso_input))
        if self.position_from_leso:
            parts.append("位置源 leso(z1)")
        if self.predict_active:
            parts.append(self.predictor.config.describe()
                         + "，驱动量 " + self.predict_source)
        return "关节状态估计器: " + "；".join(parts)


@dataclass
class EstimatorPlan:

    estimator: JointStateEstimator
    messages: tuple = ()
    warnings: tuple = ()

    @property
    def enabled(self) -> bool:
        return self.estimator.enabled


# 从 estimator 配置段构造估计器，同时把配置本身的隐患整理成 messages/warnings 返回，
# 交给调用方在启动时打印。关键换算与约束：
#   ω_o = 2π·bandwidth_hz，只有选了 leso 的关节非零，其余置零即关闭观测器；
#   b0 仅在 input = commanded_acceleration 时生效，否则强制为 0（LESO 退化成纯微分器）；
#   predict_*_sec ≤ 0 时把对应的 β 清零，杜绝「配了时域没配 β」的半开状态。
def resolve_estimator(estimator_cfg, n_joints, control_period,
                      mode: str = "simulation",
                      feedback_delay_enabled: bool | None = None) -> EstimatorPlan:
    cfg = dict(estimator_cfg or {})
    n = int(n_joints)
    control_period = float(control_period)
    messages = []
    warnings = []

    unknown = [k for k in cfg if k not in ESTIMATOR_KEYS]
    if unknown:


        warnings.append(
            "estimator 段有无法识别的键 %s（拼错？可用键：%s）—— 它们【不生效】"
            % (sorted(unknown), list(ESTIMATOR_KEYS)))

    source = _per_joint_choice(
        cfg.get("source", "motor"), n, "estimator.source", VELOCITY_SOURCES)
    position_source = str(cfg.get("position_source", "measured")).strip().lower()
    if position_source not in POSITION_SOURCES:
        raise ValueError(
            f"estimator.position_source = {position_source!r} 不合法，"
            f"可选 {list(POSITION_SOURCES)}")

    leso_cfg = dict(cfg.get("leso") or {})
    unknown_leso = [k for k in leso_cfg if k not in _LESO_KEYS]
    if unknown_leso:
        warnings.append(
            "estimator.leso 段有无法识别的键 %s（可用键：%s）—— 它们【不生效】"
            % (sorted(unknown_leso), list(_LESO_KEYS)))
    bandwidth_hz = _per_joint_float(
        leso_cfg.get("bandwidth_hz", 0.0), n, "estimator.leso.bandwidth_hz",
        minimum=0.0)
    leso_input = str(leso_cfg.get("input", "none")).strip().lower()
    if leso_input not in LESO_INPUTS:
        raise ValueError(
            f"estimator.leso.input = {leso_input!r} 不合法，可选 {list(LESO_INPUTS)}")
    b0_cfg = _per_joint_float(
        leso_cfg.get("b0", 1.0), n, "estimator.leso.b0")
    input_delay_sec = float(leso_cfg.get("input_delay_sec", 0.0) or 0.0)

    uses_leso = any(s == "leso" for s in source)
    omega_o = np.where(
        [s == "leso" for s in source], 2.0 * np.pi * bandwidth_hz, 0.0)


    b0 = b0_cfg if leso_input == "commanded_acceleration" else np.zeros(n)
    leso = JointLESO(omega_o, n, nominal_dt=control_period, b0=b0)


    predict_position_sec = float(cfg.get("predict_position_sec", 0.0) or 0.0)
    predict_velocity_sec = float(cfg.get("predict_velocity_sec", 0.0) or 0.0)
    predict_source = str(
        cfg.get("predict_source", "commanded_acceleration")).strip().lower()
    if predict_source not in PREDICT_SOURCES:
        raise ValueError(
            f"estimator.predict_source = {predict_source!r} 不合法，"
            f"可选 {list(PREDICT_SOURCES)}")


    beta_velocity = float(cfg.get("beta_velocity", 1.0))
    beta_position = float(cfg.get("beta_position", 1.0))
    if predict_velocity_sec <= 0.0:
        beta_velocity = 0.0
    if predict_position_sec <= 0.0:
        beta_position = 0.0
    predictor_cfg = SmithPredictorConfig(
        n,
        enabled=(beta_velocity > 0.0 or beta_position > 0.0),
        velocity_horizon_sec=predict_velocity_sec,
        position_horizon_sec=predict_position_sec,
        beta_velocity=beta_velocity,
        beta_position=beta_position,


        max_velocity_correction=cfg.get(
            "predict_clamp_rad_s",
            DEFAULT_MAX_VELOCITY_CORRECTION
            if n == len(DEFAULT_MAX_VELOCITY_CORRECTION)
            else DEFAULT_MAX_VELOCITY_CORRECTION[0]),
        max_position_correction=cfg.get(
            "predict_clamp_rad", DEFAULT_MAX_POSITION_CORRECTION),
        fallback_after_clamped_ticks=int(cfg.get("predict_fallback_ticks", 10)),
    )

    estimator = JointStateEstimator(
        n, control_period,
        velocity_source=source,
        position_source=position_source,
        leso=leso,
        predictor=JointStatePredictor(predictor_cfg),
        predict_source=predict_source,
        leso_input=leso_input,
        leso_input_delay_sec=input_delay_sec,
        hold_on_invalid=bool(cfg.get("hold_on_invalid", True)),
    )


    if uses_leso and np.any(bandwidth_hz[[s == "leso" for s in source]] <= 0.0):
        warnings.append(
            "estimator.source 里有关节选了 leso，但它的 bandwidth_hz 是 0 "
            "-> 该关节实际【直通电机自报速度】，不是 LESO。")

    if not estimator.enabled:
        if cfg:
            messages.append(
                "关节状态估计器: 配置存在但全部为缺省 -> 逐位直通"
                "（= 路线 A 电机自报速度基线，与不写 estimator 段【逐位相同】）")
        return EstimatorPlan(estimator, tuple(messages), tuple(warnings))

    messages.append(estimator.describe())

    if uses_leso:
        active_hz = bandwidth_hz[[s == "leso" for s in source]]
        low = active_hz[(active_hz > 0.0)
                        & (active_hz < MODEL_B_MIN_BANDWIDTH_HZ)]
        if low.size:
            warnings.append(
                "estimator.leso.bandwidth_hz 有 %s Hz 低于 %.1f Hz "
                "(ω_o=%.0f rad/s)：模型 B(T_v=T_p=10.5ms, g=2.2) 下 ω_o=100 给出 "
                "σ=+1.56【发散】，ω_o=80 即便在模型 A 下也只有 ×5.82 对基线 ×5.72"
                "（等于没收益）。⚠️ 方向是反的 —— 直觉上『低带宽更平滑更安全』"
                "在这里不成立。建议 ω_o=200 rad/s = 31.8 Hz。"
                % (np.round(low, 2).tolist(), MODEL_B_MIN_BANDWIDTH_HZ,
                   MODEL_B_MIN_OMEGA_RAD_S))
        if leso_input == "none":
            warnings.append(
                "estimator.leso.input = none（无模型输入）：此时三阶 LESO 退化成"
                "纯微分器，观测器增益的全部相位都花在残差通路上。集成侧的分析"
                "认为这种配置下【没有任何 ω_o 能同时在相位和噪声上赢过位置裸差分】"
                "（要在噪声上赢 0.0289 需 ω_o ≲ 17 Hz，那里要付 98°~122° 相位，"
                "而裸差分只付 22.3°）—— 即路线 C 被路线 B 支配。"
                "这是【可证伪的预测】，值得作为消融对照跑一次，但不要指望它赢。")
        else:
            warnings.append(
                "estimator.leso.input = commanded_acceleration（b0=%s）："
                "模型输入准时速度估计对真速度全频段零相位，不准时纯亏。"
                "⚠️ 第 16 次上机实测跟踪段净力矩 |Δτ| 中位数 "
                "[0.453,0.369,0.480,0.081,0.124,0.051] Nm 对静摩擦 "
                "[0.725,1.775,0.625,0.125,0.100,0.100] Nm —— 六轴里四轴的指令"
                "力矩低于静摩擦，超过静摩擦的时间占比 j1/j2/j4/j6 全是 0%%，"
                "此时真实 q̈≈0 而 b0·u≈1.87 rad/s²(j2)。这条通路在本机是否成立"
                "是【未知数】，必须与 input:none 做消融对照。"
                % np.round(b0_cfg, 3).tolist())
        if input_delay_sec <= 0.0:
            messages.append(
                "estimator.leso.input_delay_sec = 0：模型输入未做延迟对齐。"
                "它的物理含义是『指令发出到体现为关节加速度』，理论上 = 位置路 T_p，"
                "而 T_p=10 ms 的 r 只有 0.43/0.45【未坐实】—— 首次上机建议当扫描"
                "变量（8/10/15 ms），不要先固定。")

    fd_idx = [i + 1 for i, s in enumerate(source) if s == "finite_difference"]
    if fd_idx:
        messages.append(
            "位置裸后向差分关节 j%s：省掉电机速度估计器那 15 ms（24.8 Hz 处 "
            "133.9° -> 22.3°），代价是量化噪声。实测参考速度 0.0133 rad/s 时"
            "裸差分 std 0.0289 rad/s；⚠️ loop_delay 里那个 0.0762 是"
            "『每拍跨一个量化步』的上界，本轨迹太慢达不到。"
            % (fd_idx,))
        if any(i <= 3 for i in fd_idx):
            warnings.append(
                "j1~j3 选了 finite_difference。表面上大臂自报速度噪声实测 0.0000、"
                "换成裸差分是『无中生有』地注入 0.0287~0.0370 rad/s；"
                "⚠️ 但 2026-08-04 从 3 份实机 CSV 反解出自报速度是【12 bit 量化】的"
                "（大臂 LSB 0.009768 = 40/4095，腕部 0.024418 = 100/4095），"
                "『大臂 σ=0.0000』是静止时停在同一个码上的假象，同期还带一个整码"
                "偏置（j1/j2/j3 位置 22~38 s 一个 LSB 没动，自报速度恒为 "
                "−0.00488 rad/s）。所以大臂不是干净，是分辨率不够且有偏。"
                "两种读法都要在实物上验，别当成已经解决。")

    if estimator.predict_active:
        warnings.append(
            "⚠️ Smith 预测已开启，而 position/orientation 增益是冻结的。"
            "稳态解析解：无预测 |e| = d/c0 + ẋ_ref·T_p；Smith |e| = (d/c0)·"
            "[1 + β_v·c1·T_v + ...]。代入 d_eff=0.411 m/s²、模型A、c1=30："
            "无预测 2.103 mm vs Smith 3.453 mm —— **差 64%，更糟**。"
            "惩罚项来源明确：预测器用指令加速度积分，指令里没有扰动 d，于是把"
            "速度低估了 d·T_v，控制器多给 c1·d·T_v 的加速度，恰好推着扰动走。"
            "追平现状需 c1>=42.2（只补速度路 40.5）。"
            "=> **绝不能先只开预测、不动增益地跑一次『对照组』** —— 那次一定更差，"
            "而且会被读成『Smith 没用』。预测与提增益必须同一次改动一起上。")
        if feedback_delay_enabled is False and mode != "hardware":
            messages.append(
                "⚠️ 仿真模式且 simulation 段没建延迟（plant 是 "
                "step_on_effort_command 锁步、零延迟），此时 Smith 预测补的是一段"
                "【不存在】的延迟 —— 在这上面它是净扰动，不是补偿。冒烟测试可以，"
                "任何性能判断都无效。")

    return EstimatorPlan(estimator, tuple(messages), tuple(warnings))


def estimator_from_config(estimator_cfg, n_joints, control_period,
                          mode: str = "simulation") -> JointStateEstimator:
    return resolve_estimator(estimator_cfg, n_joints, control_period, mode).estimator


JointStateEstimator.from_config = staticmethod(estimator_from_config)
