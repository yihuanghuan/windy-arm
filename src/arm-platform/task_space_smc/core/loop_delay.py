"""仿真侧的回路延迟建模。

为什么需要
----------
2026-08-03，第 15 次上机后**直连实测**（无抓包中继）得到回路滞后：

    关节   最佳滞后   相关系数        滞后 0 拍时的相关系数
     j4     5 拍 = 25.0 ms  +0.857        +0.126
     j6     5 拍 = 25.0 ms  +0.906        -0.095

方法是把【指令力矩】与【实测速度的数值微分】做互相关，取相关最高的滞后。
两个关节独立给出同一个数，相关 0.86/0.91，不是噪声。

25 ms 在 20 Hz 处贡献 180° 相位（`360°·f·T_d`），而实测极限环频率是
**24.8 Hz**（223°）—— 15 次上机全部在 0.07~0.52 s 内腕部失控，与此吻合。

**而仿真从来复现不出来**：`launch/task_space_smc_whole6.launch.py` 给仿真
plant 的参数是 `step_on_effort_command: True` —— 收到力矩指令才步进一次，
**完全锁步、零延迟**。仿真在结构上就不可能有这个失效模式。今天为此改了 9 处
控制律（静摩擦前馈、交接偏置、腕部刚度、增益降档、边界层、速度滤波、零空间、
6D 架构、冻结时钟），没有一处命中 —— 因为病根在控制律之外。

⚠️ 一次失败的测量，记录在此以免重做
------------------------------------
试图用 `serial_tap.py` 中继把 25 ms 拆成"线上往返 / 驱动侧"两段，**失败**：
中继自己就是瓶颈。

    上行帧率   直连 200.7 Hz  ->  经中继 153 Hz
    校验错     直连 0         ->  经中继 66
    丢字节     直连 0         ->  经中继 1554
    测得滞后   直连 25 ms     ->  经中继 60 ms

所以 `serial_tap.py` **不能用于时延测量**（抓包内容仍可信，帧率与时序不可信）。


==========================================================================
分段模型（2026-08-04）：位置路 10 ms、速度路 25 ms
==========================================================================
上面那 25 ms 是【总量】，且是拿**速度**的数值微分测出来的。同一份数据改用
**位置的二阶导**重测，两条路给出的滞后不一样：

    关节   力矩 -> 位置二阶导        力矩 -> 速度一阶导
     j4     2 拍 = 10.0 ms  r=+0.434   5 拍 = 25.0 ms  r=+0.857
     j6     2 拍 = 10.0 ms  r=+0.450   5 拍 = 25.0 ms  r=+0.906

=> 速度信号比位置信号【多滞后 3 拍 = 15 ms】。推测是电机内部速度估计器的
   滤波滞后（RS05 自报速度，不是我们做的差分）。

⚠️ 位置那 10 ms 的相关系数只有 0.43/0.45，**没有坐实**。整个分段方案的收益
   依赖它，所以它必须当【不确定参数】扫（8 / 10 / 15 ms），看结论对它敏不敏感。
   速度路那 25 ms（r=0.86/0.91）才是硬数。

于是模型分三件：

  1. ``transport_delay_sec``        传输/执行延迟，作用于【全部】反馈通路
  2. ``velocity_extra_delay_sec``   速度估计器的额外滞后，【只】作用于速度那一路
  3. ``position_quantum_rad``       反馈位置的量化步长

叠加后：位置路 = transport，速度路 = transport + velocity_extra。
按实测取 0.010 + 0.015 = 0.025 s，与 j4/j6 的互相关一致。

为什么必须含位置量化
--------------------
实测位置量化步长 0.000381 rad（16 bit / ±12.5 rad）。200 Hz 下裸后向差分的
量化噪声 = 0.000381/0.005 = **0.0762 rad/s**；作为对照，电机自报速度的噪声底
实测 j4/j5/j6 = 0.0440/0.0426/0.0407 rad/s，j1/j2/j3 = 0.0000（峰值 0.0049），
30 倍不对称。

也就是说"扔掉自报速度、自己差分位置"这个方案在实物上要付 0.0762 rad/s 的噪声，
比自报速度还差 1.7 倍。而仿真里位置是双精度浮点、完美无噪 —— 不建量化的话，
裸差分在仿真里会**假赢**，把一个实物上更差的方案送上机。


==========================================================================
⚠️ 建模位置：从【指令侧】改到【反馈侧】
==========================================================================
旧实现（``CommandDelayBuffer``）把整块 5 拍压在指令侧：控制器看到真实当前
状态，plant 应用 N 拍之前的 τ。对**总**回路的稳定性而言，延迟放在环路的哪
一段是等价的，指令侧最简单。

但分段模型做不到 —— 位置路 10 ms、速度路 25 ms 是【两条不同的反馈通路】，
指令侧只有一个 τ，无从区分。所以分段模型一律加在反馈侧。

对总回路的等价性：控制器 -> plant -> 测量 -> 控制器 这一圈里，位置路总滞后
仍是 transport，速度路仍是 transport+velocity_extra，与"transport 那一段究竟
物理上是执行器延迟还是测量传输延迟"无关。两种摆法给出同一个开环相位，
而后者能表达两路之差 —— 这正是本次要建的东西。
（那 10 ms 里执行侧与测量侧各占多少，serial_tap 那次失败后至今未拆开。）

这个改动对"控制器看到什么 / CSV 记什么"的影响：

  * 控制器看到的是【延迟且量化后】的 q/qd。这是**目的**，不是副作用。
  * CSV 的 ``joint_N_rad`` / ``joint_N_velocity_rad_s`` 记【控制器看到的那一
    份】。理由是与实物对齐：实物上我们本来就只有测量值，CSV 里从来记的就是
    /joint_states 回来的东西。若改记真值，仿真 CSV 与实机 CSV 就不再是同一
    个量，两边没法直接对比 —— 而"能跟实机对比"是这套仿真唯一的用处。
  * ``pos_error_m`` / ``rot_error_rad`` / ``desired_*`` / ``actual_*`` 用
    **真值**。这些是评判"机械臂到底跑得怎么样"的指标，必须是客观量；用测量值
    的话，延迟本身会把误差指标算错，等于让被测对象自己给自己打分。
    （实物上没有真值，那边这些指标本来就带测量误差 —— 这是仿真比实物多出来
    的信息，不是不一致。）
  * 看门狗用【测量值】。它在实物上就只有测量值，仿真里给它真值等于让仿真的
    看门狗比实机的灵敏，触发时刻对不上。速度路 25 ms 的滞后意味着看门狗比
    真值晚约 25 ms 才看到超速 —— 这个滞后本身就是要复现的现象之一。

⚠️ 全部只在【非 hardware】模式生效。实物本来就有这些延迟与量化，再叠一层
   就是双重计入 —— 由调用方按 ``mode != "hardware"`` 门控，本模块不自己判断。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

__all__ = [
    "CommandDelayBuffer",
    "FeedbackDelayLine",
    "FeedbackDelayModel",
    "DelayPlan",
    "quantize",
    "resolve_simulation_delay",
    "SEGMENTED_KEYS",
    "LEGACY_KEY",
]

LEGACY_KEY = "loop_delay_sec"
SEGMENTED_KEYS = (
    "transport_delay_sec",
    "velocity_extra_delay_sec",
    "position_quantum_rad",
)


def _steps_for(delay_sec: float, control_period: float) -> int:
    """把秒换算成整数拍。

    用【四舍五入】而非 Python 内建 ``round`` —— 后者对 .5 走银行家舍入
    （``round(2.5) == 2``），对延迟建模来说是个意外行为：25 ms @ 100 Hz
    会得到 2 拍而不是 3 拍，把延迟悄悄少算一拍。
    """
    delay_sec = float(delay_sec)
    control_period = float(control_period)
    if not np.isfinite(delay_sec) or delay_sec < 0.0:
        raise ValueError(f"delay_sec 必须是非负有限值，收到 {delay_sec}")
    if not np.isfinite(control_period) or control_period <= 0.0:
        raise ValueError(f"control_period 必须为正，收到 {control_period}")
    return int(delay_sec / control_period + 0.5)


def quantize(value, quantum: float):
    """把浮点量化到 ``quantum`` 的整数倍（编码器模型）。

    ``quantum <= 0`` 时是逐位直通，连对象都原样返回。

    舍入用 ``floor(x/q + 0.5)``（round-half-up）而不是 ``np.round``
    （round-half-to-even）—— 与本模块的 ``_steps_for`` 保持同一个约定，
    免得同一份代码里两种舍入规则并存。半步命中在浮点上是零测集，
    两者的差别只在可复现性与可解释性。
    """
    quantum = float(quantum)
    if not np.isfinite(quantum):
        raise ValueError(f"quantum 必须是有限值，收到 {quantum}")
    if quantum <= 0.0:
        return value
    arr = np.asarray(value, dtype=float)
    return np.floor(arr / quantum + 0.5) * quantum


class CommandDelayBuffer:
    """把【指令】延迟整数拍后再交给下游（旧的单块模型，保留做兼容）。

    ``delay_steps == 0`` 时是逐位直通（连对象都原样返回），已有仿真基线不受影响。

    ⚠️ 新代码请用 ``FeedbackDelayModel``：分段模型必须建在反馈侧，指令侧
    区分不了位置路与速度路。本类只在配置仍写老的 ``loop_delay_sec`` 时启用。
    """

    def __init__(self, delay_steps: int) -> None:
        delay_steps = int(delay_steps)
        if delay_steps < 0:
            raise ValueError(f"delay_steps 不能为负，收到 {delay_steps}")
        self.delay_steps = delay_steps
        self._buf: deque = deque()

    @staticmethod
    def steps_for(delay_sec: float, control_period: float) -> int:
        """把秒换算成整数拍。见模块内 ``_steps_for``。"""
        return _steps_for(delay_sec, control_period)

    @property
    def enabled(self) -> bool:
        return self.delay_steps > 0

    def reset(self) -> None:
        self._buf.clear()

    def __call__(self, command):
        """压入本拍指令，返回应当【现在生效】的那一拍。

        缓冲未满时返回零指令 —— 对应"延迟窗口内 plant 还没收到任何东西"。
        用零而不是当前指令，是为了不把延迟在起步阶段偷偷抹掉。
        """
        cmd = np.asarray(command, dtype=float)
        if self.delay_steps == 0:
            return command                      # 逐位直通，连拷贝都不做
        self._buf.append(cmd.copy())
        if len(self._buf) <= self.delay_steps:
            return np.zeros_like(cmd)
        return self._buf.popleft()

    def phase_lag_deg(self, frequency_hz: float, control_period: float) -> float:
        """该延迟在给定频率处贡献的相位滞后 [deg]，用于交易透明化。"""
        return 360.0 * float(frequency_hz) * self.delay_steps * float(control_period)


class FeedbackDelayLine:
    """把【反馈】延迟整数拍后再交给控制器。

    与 ``CommandDelayBuffer`` 的唯一区别是**预热行为**，而这个区别是必须的：

      * 指令侧预热必须发 **零** —— plant 在延迟窗口内确实还没收到任何力矩，
        发当前指令等于把延迟在起步阶段抹掉。
      * 反馈侧预热必须发 **最旧的那一份** —— 发零意味着"控制器认为机械臂在
        零位"。零位的雅可比 sigma_min = 0.00001（cond 326263），控制器会当场
        算出一个奇异位形下的巨大加速度需求，起步即炸。那是建模事故，不是被
        建模的现象。物理上也是保持最旧那份才对：真实系统在控制开始前早就在
        收反馈了，管道本来就是满的，且机械臂静止 —— 最旧的那份就是初始状态。

    ``delay_steps == 0`` 时逐位直通（连对象都原样返回）。
    """

    def __init__(self, delay_steps: int) -> None:
        delay_steps = int(delay_steps)
        if delay_steps < 0:
            raise ValueError(f"delay_steps 不能为负，收到 {delay_steps}")
        self.delay_steps = delay_steps
        self._buf: deque = deque()

    @property
    def enabled(self) -> bool:
        return self.delay_steps > 0

    def reset(self) -> None:
        self._buf.clear()

    def __call__(self, sample):
        if self.delay_steps == 0:
            return sample                       # 逐位直通
        arr = np.asarray(sample, dtype=float)
        self._buf.append(arr.copy())
        if len(self._buf) <= self.delay_steps:
            # 管道未满 -> 保持最旧的一份（= 起步状态），绝不发零。
            return self._buf[0].copy()
        return self._buf.popleft()


@dataclass
class FeedbackDelayModel:
    """分段反馈通路模型：传输延迟 + 速度额外滞后 + 位置量化。

    参数全部是**秒 / 弧度**，内部换成整数拍。三项独立，任一非零即启用；
    **三项全零时逐位直通**（``__call__`` 原样返回传进来的那几个对象），
    所以没写这些键的配置行为逐字节不变。

    ⚠️ 代码侧缺省【全零】，不是实测值 0.010/0.015/0.000381。
       理由与 ``freeze_trajectory_during_blend`` 相同：smc_params_whole6.yaml
       与 smc_params_hwtest_simbest.yaml 是已验证基线，若代码缺省取实测值，
       它们不改一个字行为就变了。要建模就在配置里显式写。
    """

    transport_delay_sec: float = 0.0
    velocity_extra_delay_sec: float = 0.0
    position_quantum_rad: float = 0.0
    control_period: float = 0.005

    transport_steps: int = field(init=False)
    velocity_extra_steps: int = field(init=False)

    def __post_init__(self) -> None:
        self.transport_delay_sec = float(self.transport_delay_sec)
        self.velocity_extra_delay_sec = float(self.velocity_extra_delay_sec)
        self.position_quantum_rad = float(self.position_quantum_rad)
        self.control_period = float(self.control_period)
        if not np.isfinite(self.position_quantum_rad) or self.position_quantum_rad < 0.0:
            raise ValueError(
                f"position_quantum_rad 必须是非负有限值，收到 {self.position_quantum_rad}")
        self.transport_steps = _steps_for(
            self.transport_delay_sec, self.control_period)
        self.velocity_extra_steps = _steps_for(
            self.velocity_extra_delay_sec, self.control_period)
        # 位置与力矩走同一条传输通路（同一帧串口报文里回来的），速度多一段
        # 电机内部估计器的滤波滞后。
        self._position_line = FeedbackDelayLine(self.transport_steps)
        self._velocity_line = FeedbackDelayLine(
            self.transport_steps + self.velocity_extra_steps)
        self._effort_line = FeedbackDelayLine(self.transport_steps)

    # -- 查询 ---------------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return (self.transport_steps > 0
                or self.velocity_extra_steps > 0
                or self.position_quantum_rad > 0.0)

    @property
    def position_delay_steps(self) -> int:
        return self.transport_steps

    @property
    def velocity_delay_steps(self) -> int:
        return self.transport_steps + self.velocity_extra_steps

    @property
    def position_delay_sec(self) -> float:
        """量化到整数拍之后的实际位置路滞后 [s]。"""
        return self.position_delay_steps * self.control_period

    @property
    def velocity_delay_sec(self) -> float:
        """量化到整数拍之后的实际速度路总滞后 [s]（应等于实测 0.025）。"""
        return self.velocity_delay_steps * self.control_period

    def phase_lag_deg(self, frequency_hz: float, steps: int) -> float:
        """给定拍数的延迟在给定频率处贡献的相位滞后 [deg]。"""
        return 360.0 * float(frequency_hz) * int(steps) * self.control_period

    def velocity_phase_lag_deg(self, frequency_hz: float) -> float:
        return self.phase_lag_deg(frequency_hz, self.velocity_delay_steps)

    def position_phase_lag_deg(self, frequency_hz: float) -> float:
        return self.phase_lag_deg(frequency_hz, self.position_delay_steps)

    def quantization_velocity_noise(self) -> float:
        """把位置量化裸差分成速度时的噪声幅度 [rad/s]。

        = quantum / control_period。实测 0.000381/0.005 = 0.0762 rad/s，
        比电机自报速度的噪声底（j4/j5/j6 0.041~0.044）还大 1.7 倍 ——
        这就是"自己差分位置"在实物上打不过自报速度的原因。
        """
        return self.position_quantum_rad / self.control_period

    # -- 使用 ---------------------------------------------------------------

    def reset(self) -> None:
        self._position_line.reset()
        self._velocity_line.reset()
        self._effort_line.reset()

    def __call__(self, position, velocity, effort=None):
        """真值 -> 控制器看到的那一份。每个控制周期【恰好调用一次】。

        必须按控制周期调用而不是按反馈回调调用：延迟的单位是"拍"，一拍就是
        一个控制周期。回调频率若与控制频率不一致（plant 自走时钟的情形），
        按回调推进会让实际滞后的秒数偏离配置值。

        返回 ``(position, velocity, effort)``。未启用时原样返回入参对象。
        """
        if not self.enabled:
            return position, velocity, effort
        q = quantize(position, self.position_quantum_rad)
        return (
            self._position_line(q),
            self._velocity_line(velocity),
            self._effort_line(effort) if effort is not None else effort,
        )

    def describe(self) -> str:
        dt_ms = 1000.0 * self.control_period
        return (
            "位置路 %.1f ms = %d 拍（24.8 Hz 处 %.0f°），"
            "速度路 %.1f ms = %d 拍（24.8 Hz 处 %.0f°，含估计器额外 %.1f ms），"
            "位置量化 %.6f rad（裸差分噪声 %.4f rad/s @ %.1f ms 拍）"
            % (1000.0 * self.position_delay_sec, self.position_delay_steps,
               self.position_phase_lag_deg(24.8),
               1000.0 * self.velocity_delay_sec, self.velocity_delay_steps,
               self.velocity_phase_lag_deg(24.8),
               1000.0 * self.velocity_extra_steps * self.control_period,
               self.position_quantum_rad,
               self.quantization_velocity_noise(), dt_ms))


@dataclass
class DelayPlan:
    """一份配置解析出来的完整延迟方案。

    ``command_delay`` 是旧的单块指令侧模型（只在配置仍写 ``loop_delay_sec``
    且没写任何分段键时非零），``feedback`` 是分段反馈侧模型。两者不会同时
    生效 —— 见 ``resolve_simulation_delay`` 的优先级规则。
    """

    command_delay: CommandDelayBuffer
    feedback: FeedbackDelayModel
    messages: tuple = ()
    warnings: tuple = ()

    @property
    def enabled(self) -> bool:
        return self.command_delay.enabled or self.feedback.enabled


def resolve_simulation_delay(simulation_cfg, control_period, mode="simulation"):
    """从配置的 ``simulation`` 段解析延迟方案。

    优先级规则（任务要求）：

      * 只写 ``loop_delay_sec``      -> 旧的指令侧单块模型，行为与改动前一致。
      * 写了任一分段键                -> 分段反馈侧模型。
      * 两者【同时】写                -> 以分段模型为准，并产生一条警告。
        （不静默忽略：同时写着两套延迟的配置几乎一定是改到一半忘了删，
          静默取其一会让人以为总滞后是两者之和。）
      * 都没写                        -> 全关，逐位直通。

    ``mode == "hardware"`` 时**一律全关** —— 实物自带这些延迟与量化，
    叠加即双重计入；但若配置里写了非零值，会产生一条提示说明它不生效。
    """
    cfg = dict(simulation_cfg or {})
    control_period = float(control_period)
    messages = []
    warnings = []

    legacy_sec = float(cfg.get(LEGACY_KEY, 0.0) or 0.0)
    has_segmented = any(k in cfg for k in SEGMENTED_KEYS)
    has_legacy = LEGACY_KEY in cfg

    transport = float(cfg.get("transport_delay_sec", 0.0) or 0.0)
    velocity_extra = float(cfg.get("velocity_extra_delay_sec", 0.0) or 0.0)
    quantum = float(cfg.get("position_quantum_rad", 0.0) or 0.0)

    if has_segmented and has_legacy:
        warnings.append(
            "simulation 段同时写了分段模型（%s）和旧的 %s=%.4f s —— "
            "【以分段模型为准】，旧键被忽略（不是相加）。请删掉旧键。"
            % ("/".join(k for k in SEGMENTED_KEYS if k in cfg),
               LEGACY_KEY, legacy_sec))

    if mode == "hardware":
        if has_segmented or legacy_sec > 0.0:
            messages.append(
                "hardware 模式：simulation 段的延迟/量化配置【不生效】"
                "（实物自带，叠加即双重计入）")
        return DelayPlan(
            command_delay=CommandDelayBuffer(0),
            feedback=FeedbackDelayModel(control_period=control_period),
            messages=tuple(messages),
            warnings=tuple(warnings),
        )

    if has_segmented:
        feedback = FeedbackDelayModel(
            transport_delay_sec=transport,
            velocity_extra_delay_sec=velocity_extra,
            position_quantum_rad=quantum,
            control_period=control_period,
        )
        command = CommandDelayBuffer(0)
        if feedback.enabled:
            messages.append("仿真反馈通路模型: " + feedback.describe())
        else:
            messages.append(
                "仿真反馈通路模型: 分段键全为零 -> 逐位直通"
                "（对照组：用于证明失稳确实来自延迟）")
    else:
        feedback = FeedbackDelayModel(control_period=control_period)
        command = CommandDelayBuffer(_steps_for(legacy_sec, control_period))
        if command.enabled:
            messages.append(
                "仿真回路延迟(旧的指令侧单块模型): %.1f ms = %d 拍 @ %.0f Hz"
                "（在 24.8 Hz 处相位滞后 %.0f°）"
                % (legacy_sec * 1000.0, command.delay_steps,
                   1.0 / control_period,
                   command.phase_lag_deg(24.8, control_period)))
            warnings.append(
                "仍在用指令侧单块延迟。它无法区分位置路(10 ms)与速度路(25 ms)，"
                "建议改用 transport_delay_sec / velocity_extra_delay_sec / "
                "position_quantum_rad 三个键。")

    return DelayPlan(
        command_delay=command,
        feedback=feedback,
        messages=tuple(messages),
        warnings=tuple(warnings),
    )
