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
    delay_sec = float(delay_sec)
    control_period = float(control_period)
    if not np.isfinite(delay_sec) or delay_sec < 0.0:
        raise ValueError(f"delay_sec 必须是非负有限值，收到 {delay_sec}")
    if not np.isfinite(control_period) or control_period <= 0.0:
        raise ValueError(f"control_period 必须为正，收到 {control_period}")
    return int(delay_sec / control_period + 0.5)


def quantize(value, quantum: float):
    quantum = float(quantum)
    if not np.isfinite(quantum):
        raise ValueError(f"quantum 必须是有限值，收到 {quantum}")
    if quantum <= 0.0:
        return value
    arr = np.asarray(value, dtype=float)
    return np.floor(arr / quantum + 0.5) * quantum


class CommandDelayBuffer:

    def __init__(self, delay_steps: int) -> None:
        delay_steps = int(delay_steps)
        if delay_steps < 0:
            raise ValueError(f"delay_steps 不能为负，收到 {delay_steps}")
        self.delay_steps = delay_steps
        self._buf: deque = deque()

    @staticmethod
    def steps_for(delay_sec: float, control_period: float) -> int:
        return _steps_for(delay_sec, control_period)

    @property
    def enabled(self) -> bool:
        return self.delay_steps > 0

    def reset(self) -> None:
        self._buf.clear()

    def __call__(self, command):
        cmd = np.asarray(command, dtype=float)
        if self.delay_steps == 0:
            return command
        self._buf.append(cmd.copy())
        if len(self._buf) <= self.delay_steps:
            return np.zeros_like(cmd)
        return self._buf.popleft()

    def phase_lag_deg(self, frequency_hz: float, control_period: float) -> float:
        return 360.0 * float(frequency_hz) * self.delay_steps * float(control_period)


class FeedbackDelayLine:

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
            return sample
        arr = np.asarray(sample, dtype=float)
        self._buf.append(arr.copy())
        if len(self._buf) <= self.delay_steps:

            return self._buf[0].copy()
        return self._buf.popleft()


@dataclass
class FeedbackDelayModel:

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


        self._position_line = FeedbackDelayLine(self.transport_steps)
        self._velocity_line = FeedbackDelayLine(
            self.transport_steps + self.velocity_extra_steps)
        self._effort_line = FeedbackDelayLine(self.transport_steps)


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
        return self.position_delay_steps * self.control_period

    @property
    def velocity_delay_sec(self) -> float:
        return self.velocity_delay_steps * self.control_period

    def phase_lag_deg(self, frequency_hz: float, steps: int) -> float:
        return 360.0 * float(frequency_hz) * int(steps) * self.control_period

    def velocity_phase_lag_deg(self, frequency_hz: float) -> float:
        return self.phase_lag_deg(frequency_hz, self.velocity_delay_steps)

    def position_phase_lag_deg(self, frequency_hz: float) -> float:
        return self.phase_lag_deg(frequency_hz, self.position_delay_steps)

    def quantization_velocity_noise(self) -> float:
        return self.position_quantum_rad / self.control_period


    def reset(self) -> None:
        self._position_line.reset()
        self._velocity_line.reset()
        self._effort_line.reset()

    def __call__(self, position, velocity, effort=None):
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

    command_delay: CommandDelayBuffer
    feedback: FeedbackDelayModel
    messages: tuple = ()
    warnings: tuple = ()

    @property
    def enabled(self) -> bool:
        return self.command_delay.enabled or self.feedback.enabled


def resolve_simulation_delay(simulation_cfg, control_period, mode="simulation"):
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
