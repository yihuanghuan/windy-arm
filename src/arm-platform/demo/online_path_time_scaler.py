#!/usr/bin/env python3
"""图形路径的在线时间缩放。

基座时间继续使用真实时钟；只调整图形路径时间 s。这样基座补偿占用关节预算时，
图形会减速或暂停，而不是把基座运动也一起“放慢”。
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class TimeScalingConfig:
    min_scale: float = 0.0
    velocity_reserve: float = 0.15
    tracking_slow_error_m: float = 0.010
    tracking_stop_error_m: float = 0.030
    scale_accel_per_sec: float = 1.0
    scale_decel_per_sec: float = 5.0


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
                     tracking_error_m: float) -> float:
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
            min(velocity_scale, error_scale), self.config.min_scale, 1.0))

    def update(self, dt: float, joint_velocity_utilization: float,
               tracking_error_m: float) -> float:
        if dt < 0.0 or not np.isfinite(dt):
            raise ValueError("dt 必须有限且非负")
        target = self.target_scale(joint_velocity_utilization, tracking_error_m)
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
