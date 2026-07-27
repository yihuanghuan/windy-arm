#!/usr/bin/env python3
"""图形路径的在线时间缩放。

基座时间继续使用真实时钟；只调整图形路径时间 s。这样基座补偿占用关节预算时，
图形会减速或暂停，而不是把基座运动也一起“放慢”。
"""

from dataclasses import dataclass
from collections import deque
import math
import statistics

import numpy as np


@dataclass
class TimeScalingConfig:
    min_scale: float = 0.0
    velocity_reserve: float = 0.15
    tracking_slow_error_m: float = 0.010
    tracking_stop_error_m: float = 0.030
    scale_accel_per_sec: float = 1.0
    scale_decel_per_sec: float = 5.0


@dataclass(frozen=True)
class TrustLadderConfig:
    """信任驱动路径降速阶梯的物理阈值。

    δ 阈值全部相对在线学习的健康噪声基线，避免把 2mm 测量噪声对应的
    4–5mm 保形半径误判为故障。``b_degraded``/``b_recover`` 形成信任迟滞，
    ``delta_degraded_ratio``/``delta_recover_ratio`` 形成 δ 迟滞。
    """

    b_degraded: float = 0.3
    b_recover: float = 0.7
    delta_degraded_ratio: float = 1.35
    delta_recover_ratio: float = 1.2
    min_scale: float = 0.3
    baseline_warmup_sec: float = 4.0
    baseline_tau_sec: float = 20.0
    trust_commission_sec: float = 1.0
    recover_debounce_sec: float = 2.0

    def __post_init__(self):
        if not (0.0 <= self.b_degraded < self.b_recover <= 1.0):
            raise ValueError("需要 0 <= b_degraded < b_recover <= 1")
        if not (1.0 <= self.delta_recover_ratio
                < self.delta_degraded_ratio):
            raise ValueError(
                "需要 1 <= delta_recover_ratio < delta_degraded_ratio")
        if not (0.0 < self.min_scale <= 1.0):
            raise ValueError("min_scale 必须在 (0,1]")
        if self.baseline_warmup_sec <= 0.0 or self.baseline_tau_sec <= 0.0:
            raise ValueError("baseline 时间常数必须为正")
        if self.trust_commission_sec < 0.0:
            raise ValueError("trust_commission_sec 必须非负")
        if self.recover_debounce_sec < 0.0:
            raise ValueError("recover_debounce_sec 必须非负")


class TrustLadderStateMachine:
    """``warming -> healthy -> degraded -> recovering`` 四态降速阶梯。

    warming 在已知无故障的启动窗收集 δ，满 warmup 后以中位数建立健康基线；
    未建立基线前不降速，防止保形校准 fallback 值误触发。β 通道须连续健康
    ``trust_commission_sec`` 后才参与故障门；这避免谱模型尚未完成首次辨识时
    把“冷启动低 b”错当故障，启动窗内的真实 fault 仍由相对 δ 门捕获。
    healthy 中仅用明确健康且未超过恢复门的样本慢速更新基线。任一已启用的
    故障门触发立即进入 degraded；恢复门连续满足 debounce 后回到 1.0。
    """

    STATES = ('warming', 'healthy', 'degraded', 'recovering')

    def __init__(self, config: TrustLadderConfig | None = None):
        self.config = config or TrustLadderConfig()
        c = self.config
        self.reset()

    def reset(self):
        self.state = 'warming'
        self.cap = 1.0
        self.delta_baseline = float('nan')
        self._warm_samples = deque(maxlen=512)
        self._warm_since = None
        self._recover_since = None
        self._commission_since = None
        self.trust_commissioned = False
        self._last_t = None

    @staticmethod
    def _valid(t: float, b: float, delta: float) -> bool:
        return (math.isfinite(t) and math.isfinite(b)
                and math.isfinite(delta) and delta > 0.0)

    def _advance_warmup(self, t: float, delta: float):
        if self._warm_since is None:
            self._warm_since = t
        self._warm_samples.append(delta)
        if (t - self._warm_since >= self.config.baseline_warmup_sec
                and len(self._warm_samples) >= 3):
            self.delta_baseline = float(statistics.median(self._warm_samples))
            self._warm_samples.clear()
            self.state = 'healthy'

    def _update_commission(self, t: float, b: float):
        if self.trust_commissioned:
            return
        if b < self.config.b_recover:
            self._commission_since = None
            return
        if self._commission_since is None:
            self._commission_since = t
        if t - self._commission_since >= self.config.trust_commission_sec:
            self.trust_commissioned = True

    def _update_baseline(self, dt: float, b: float, delta: float):
        c = self.config
        if (b < c.b_recover
                or delta > self.delta_baseline * c.delta_recover_ratio):
            return
        alpha = 1.0 - math.exp(-max(0.0, dt) / c.baseline_tau_sec)
        self.delta_baseline += alpha * (delta - self.delta_baseline)

    def update(self, t: float, trust: float, delta: float) -> float:
        """推进状态机并返回目标 scale cap；输入非法时保持上一安全状态。"""
        t, b, d = float(t), float(trust), float(delta)
        if not self._valid(t, b, d):
            return self.cap
        if self._last_t is not None and t < self._last_t:
            raise ValueError("TrustLadderStateMachine 时间必须单调")
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t

        if self.state == 'warming':
            self._advance_warmup(t, d)
            return self.cap

        c = self.config
        self._update_commission(t, b)
        bad_b = self.trust_commissioned and b < c.b_degraded
        bad_delta = d > self.delta_baseline * c.delta_degraded_ratio
        if bad_b or bad_delta:
            b_cap = (max(0.0, b) / c.b_degraded) if bad_b else 1.0
            d_cap = ((self.delta_baseline * c.delta_degraded_ratio / d)
                     if bad_delta else 1.0)
            raw = max(c.min_scale, min(b_cap, d_cap))
            self.cap = min(self.cap, raw) if self.state != 'healthy' else raw
            self.state = 'degraded'
            self._recover_since = None
            return self.cap

        recover_ok = (
            (not self.trust_commissioned or b >= c.b_recover)
            and d <= self.delta_baseline * c.delta_recover_ratio)
        if self.state in ('degraded', 'recovering'):
            if not recover_ok:
                self.state = 'degraded'
                self._recover_since = None
                return self.cap
            if self._recover_since is None:
                self._recover_since = t
                self.state = 'recovering'
            if t - self._recover_since >= c.recover_debounce_sec:
                self.state = 'healthy'
                self.cap = 1.0
                self._recover_since = None
            return self.cap

        self._update_baseline(dt, b, d)
        self.cap = 1.0
        return self.cap


class OnlinePathTimeScaler:
    def __init__(self, config: TimeScalingConfig | None = None):
        self.config = config or TimeScalingConfig()
        if not 0.0 <= self.config.min_scale <= 1.0:
            raise ValueError("min_scale 必须在 [0,1]")
        if not 0.0 <= self.config.velocity_reserve < 1.0:
            raise ValueError("velocity_reserve 必须在 [0,1)")
        if not 0.0 <= self.config.tracking_slow_error_m < self.config.tracking_stop_error_m:
            raise ValueError("tracking error 阈值非法")
        self.reset()

    def reset(self, path_time: float = 0.0):
        self.path_time = float(path_time)
        self.scale = 1.0

    def target_scale(self, joint_velocity_utilization: float,
                     tracking_error_m: float,
                     external_scale_cap: float = 1.0) -> float:
        """external_scale_cap: 外部通道(如信任降速阶梯)给出的目标 scale 上限,
        默认 1.0 时与历史行为逐位一致; 只收紧目标, 不绕过 min_scale 下限与
        update() 的 accel/decel 速率限制。"""
        cap = float(external_scale_cap)
        if not np.isfinite(cap):
            raise ValueError("external_scale_cap 必须有限")
        utilization = max(0.0, float(joint_velocity_utilization))
        available = ((1.0 - utilization - self.config.velocity_reserve)
                     / (1.0 - self.config.velocity_reserve))
        velocity_scale = float(np.clip(available, 0.0, 1.0))
        error = max(0.0, float(tracking_error_m))
        if error <= self.config.tracking_slow_error_m:
            error_scale = 1.0
        elif error >= self.config.tracking_stop_error_m:
            error_scale = 0.0
        else:
            span = (self.config.tracking_stop_error_m
                    - self.config.tracking_slow_error_m)
            error_scale = 1.0 - (error - self.config.tracking_slow_error_m) / span
        return float(np.clip(
            min(velocity_scale, error_scale, cap), self.config.min_scale, 1.0))

    def update(self, dt: float, joint_velocity_utilization: float,
               tracking_error_m: float,
               external_scale_cap: float = 1.0) -> float:
        if dt < 0.0 or not np.isfinite(dt):
            raise ValueError("dt 必须有限且非负")
        target = self.target_scale(joint_velocity_utilization, tracking_error_m,
                                   external_scale_cap)
        rate = (self.config.scale_decel_per_sec
                if target < self.scale else self.config.scale_accel_per_sec)
        max_change = rate * dt
        self.scale += float(np.clip(target - self.scale, -max_change, max_change))
        self.scale = float(np.clip(self.scale, self.config.min_scale, 1.0))
        self.path_time += self.scale * dt
        return self.path_time

    def preview(self, offsets_sec) -> np.ndarray:
        offsets = np.asarray(offsets_sec, dtype=float)
        if np.any(offsets < 0.0) or not np.all(np.isfinite(offsets)):
            raise ValueError("preview offsets 必须有限且非负")
        return self.path_time + self.scale * offsets
