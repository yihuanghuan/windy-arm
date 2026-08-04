"""关节速度反馈的一阶低通滤波。

为什么需要
----------
2026-08-03 实机静止（看门狗保位、位置变化 <0.05°）时实测速度反馈：

    关节        位置变化°   速度标准差 rad/s   速度峰值 rad/s
    j1/j2/j3     0.000        0.0000            0.0049
    j4/j5/j6     0.022~0.044  0.041~0.044       0.13~0.16

腕部三个 RS05 小电机有 ~0.04 rad/s 的速度估计噪声底，大臂的 RS 电机没有 ——
**30 倍不对称**。而动力学一致伪逆把任务加速度放大到 j4 (11.0x) 和 j6 (11.1x)，
是大臂的 2.8 倍（惯量最小的两个：0.0117 / 0.0106 kgm²）。噪声经这条通路进入
控制律，正是 25.8 Hz 极限环的候选激励源。

⚠️ 这是一笔【有代价的交易】，不是免费降噪
------------------------------------------
同批数据测得该噪声是**白的**（能量正比于带宽）：

    频段      0-5Hz   5-15Hz  15-35Hz  35-60Hz  60-100Hz
    j6 占比    5.4%    10.9%   20.3%    23.6%    39.7%
    能量中位频率 48.5 Hz（Nyquist 100 Hz 的一半）

白噪声要滤得动就得把截止频率压低，而失稳频率恰好是 25.8 Hz：

    截止频率   去掉的噪声功率   25 Hz 处引入的相位滞后
      30 Hz        ~70%              39.8°
      50 Hz        ~50%              26.6°
      80 Hz        ~20%              17.4°

也就是说，**滤波是在最付不起相位裕度的频率上花相位**。若失稳的根源是回路
负裕度（实测发散呈指数型，e 折时间约 30 ms，且发散速度随增益成比例变化 ——
增益 1.0 倍时 0.07 s、0.3 倍时 0.41~0.47 s），那么降噪救不了，滤波反而
可能提前失稳。本模块提供工具与开关，结论以实测为准。

默认关闭；只在配置里显式打开，且可逐关节设置截止频率（大臂噪声本就为零，
给它加滞后是纯亏损，应设为 0 = 不滤）。
"""

from __future__ import annotations

import numpy as np

__all__ = ["JointVelocityFilter"]


class JointVelocityFilter:
    """逐关节一阶低通：y[k] = y[k-1] + a·(x[k] - y[k-1])，a = dt/(tau + dt)。

    ``cutoff_hz`` 逐关节给定；<= 0 表示该关节不滤波（直通）。
    """

    def __init__(self, cutoff_hz, n_joints: int) -> None:
        cutoff = np.asarray(cutoff_hz, dtype=float).ravel()
        if cutoff.size == 1:
            cutoff = np.full(int(n_joints), float(cutoff[0]))
        if cutoff.size != int(n_joints):
            raise ValueError(
                f"cutoff_hz 需要 {n_joints} 个值或 1 个标量，收到 {cutoff.size}")
        if not np.all(np.isfinite(cutoff)):
            raise ValueError(f"cutoff_hz 含非有限值: {cutoff}")
        if np.any(cutoff < 0.0):
            raise ValueError(f"cutoff_hz 不能为负: {cutoff}")
        self.cutoff_hz = cutoff
        self.n_joints = int(n_joints)
        self._state = np.zeros(self.n_joints)
        self._primed = False

    @property
    def enabled(self) -> bool:
        return bool(np.any(self.cutoff_hz > 0.0))

    def alpha(self, dt: float) -> np.ndarray:
        """本采样周期下的混合系数。cutoff<=0 的关节返回 1.0（直通）。"""
        dt = float(dt)
        if dt <= 0.0:
            raise ValueError(f"dt 必须为正，收到 {dt}")
        a = np.ones(self.n_joints)
        active = self.cutoff_hz > 0.0
        tau = 1.0 / (2.0 * np.pi * self.cutoff_hz[active])
        a[active] = dt / (tau + dt)
        return a

    def phase_lag_deg(self, frequency_hz) -> np.ndarray:
        """在给定频率处引入的相位滞后 [deg]。用于交易透明化，不进控制律。"""
        f = float(frequency_hz)
        out = np.zeros(self.n_joints)
        active = self.cutoff_hz > 0.0
        out[active] = np.degrees(np.arctan2(f, self.cutoff_hz[active]))
        return out

    def reset(self, value=None) -> None:
        self._state = (np.zeros(self.n_joints) if value is None
                       else np.asarray(value, dtype=float).reshape(self.n_joints).copy())
        self._primed = value is not None

    def __call__(self, velocity, dt: float) -> np.ndarray:
        v = np.asarray(velocity, dtype=float).reshape(self.n_joints)
        if not self.enabled:
            return v.copy()
        if not np.all(np.isfinite(v)):
            # 反馈损坏时保持上一拍，不把 NaN 灌进控制律
            return self._state.copy()
        if not self._primed:
            # 首拍直接采纳，避免从 0 起步造成的假冲击
            self._state = v.copy()
            self._primed = True
            return self._state.copy()
        a = self.alpha(dt)
        self._state = self._state + a * (v - self._state)
        # cutoff<=0 的关节必须【逐位】直通。靠 a=1 的算术走一遍会引入 1 ULP
        # 误差（state + 1.0*(v-state) != v），对"不滤波"这个承诺不干净。
        passthrough = self.cutoff_hz <= 0.0
        self._state[passthrough] = v[passthrough]
        return self._state.copy()
