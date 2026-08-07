"""Vicon 数据流的时间戳对齐与刚体量搬运。全部不依赖 ROS，可脱离计算图测试。

* :class:`SourceClockAligner` —— 源时钟与本机时钟分离。求导必须用源时钟：拿真实
  8 ms 的位移去除以 0.09 ms 的到达间隔，速度会被放大约 90 倍，曾被误当成网络噪声
  （见 docs/QA_record.md）。
* :func:`shift_twist_accel` —— 把速度/加速度沿力臂从标记簇原点搬到 base_link。
* :class:`FrameAssembler` —— 按精确源时间戳把 pose/twist/accel 装配成整帧。
* :func:`pair_by_source_time` / :class:`DerivativeConsistencyMonitor` —— 跨通道
  配对与粗差检测。
"""

from __future__ import annotations

import math
from collections import OrderedDict, deque
from dataclasses import dataclass, field

import numpy as np


# --------------------------------------------------------------- clock

@dataclass
class AlignedStamp:
    """一个样本的两个时钟。

    sample_time 是源时钟，求导必须对它作差；stamp_local 是同一物理时刻在本机时钟
    下的表达，陈旧性判据拿它和 now 比。
    """

    sample_time: float
    stamp_local: float
    latency: float
    used_source: bool


class SourceClockAligner:
    """对 ``arrival − source`` 取滑动双窗最小值，估计两个时钟的常量偏移：

        offset = min(当前窗最小值, 上一窗最小值)
        stamp_local = source + offset

    取最小值而不是平均：窗内最小的那个样本是延迟最低的一个，它才是【常量偏移】的
    最好估计，平均值估的是延迟。两个交替窗口保证 O(1)，同时让估计跟着时钟漂移走，
    而不是锁死在某个早到的幸运包上。

    本机实测 Vicon 源时钟比本地快 11.992 h（12 h 时区差减去约 28 s 真实偏斜），
    量级大且会漂，只能在线估计，不能硬编码。
    """

    # window_sec：取最小值的滑动窗长 (s)。大→偏移估计更稳但跟不上时钟漂移；
    # 小→跟漂移快，但窗内样本少、容易被一个偶然低延迟包带偏。
    def __init__(self, window_sec: float = 5.0, enabled: bool = True) -> None:
        self.window_sec = max(0.5, float(window_sec))
        self.enabled = bool(enabled)
        self._window_start: float | None = None
        self._current = math.inf
        self._previous = math.inf
        self.offset_known = False

    def source_time(self, stamp_sec: int, stamp_nanosec: int) -> float | None:
        """从 header 里取源时间；没人填过则返回 None。

        stamp 为零或负正是 vrpn_listener 的 twist/accel 情形（实测 5156/5156 条
        stamp == 0），也是唯一一种「到达时间确实是最好选择」的情况。
        """
        if not self.enabled:
            return None
        stamp = float(stamp_sec) + 1e-9 * float(stamp_nanosec)
        return stamp if stamp > 0.0 and math.isfinite(stamp) else None

    def offset(self, arrival: float, source: float) -> float:
        difference = arrival - source
        if (self._window_start is None
                or arrival - self._window_start > self.window_sec):
            self._previous = self._current
            self._current = math.inf
            self._window_start = arrival
        self._current = min(self._current, difference)
        self.offset_known = True
        return min(self._current, self._previous)

    def align(self, arrival: float, stamp_sec: int, stamp_nanosec: int) -> AlignedStamp:
        source = self.source_time(stamp_sec, stamp_nanosec)
        if source is None:
            return AlignedStamp(arrival, arrival, 0.0, False)
        offset = self.offset(arrival, source)
        stamp_local = source + offset
        return AlignedStamp(source, stamp_local, max(0.0, arrival - stamp_local), True)


# --------------------------------------------------------------- lever arm

def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def shift_twist_accel(
    v_R: np.ndarray,
    omega_R: np.ndarray,
    a_R: np.ndarray,
    alpha_R: np.ndarray,
    r: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """把刚体的速度/加速度从点 R 搬到点 B = R + r：

        v_B = v_R + ω × r
        a_B = a_R + α × r + ω × (ω × r)

    r 必须已经表达在与速度同一个坐标系里（LOCAL_WORLD_ALIGNED 的量就用世界系轴）。
    向心项 ω × (ω × r) 在角速度恒定时依然存在，漏掉它的误差静态测试看不见。

    基座的 r 是 arm_base_from_base_link.translation_m（33.1 mm），不是
    arm_ee_from_link6（17.9 mm，那是末端标记的力臂）。两者混用会有 15.2 mm 误差，
    直接进 compute_task_bias_acceleration，最后落到关节力矩上。
    """
    v_R = np.asarray(v_R, dtype=float).reshape(3)
    omega_R = np.asarray(omega_R, dtype=float).reshape(3)
    a_R = np.asarray(a_R, dtype=float).reshape(3)
    alpha_R = np.asarray(alpha_R, dtype=float).reshape(3)
    r = np.asarray(r, dtype=float).reshape(3)

    v_B = v_R + np.cross(omega_R, r)
    a_B = a_R + np.cross(alpha_R, r) + np.cross(omega_R, np.cross(omega_R, r))
    return v_B, a_B


# --------------------------------------------------------------- pairing

@dataclass
class ChannelSample:
    source_time: float
    value: np.ndarray
    stamp_local: float


def pair_by_source_time(
    target_time: float,
    buffer,
    max_skew_sec: float,
) -> ChannelSample | None:
    """按源时间取 buffer 里最近的样本，错位超过 max_skew_sec 则返回 None。

    宁可拒绝也不外推：辅助通道配错帧属于「看着正常的错数据」。

    .. warning::
       不要在「到达即配对」的回调里用它。实测同帧 twist/accel 恒晚于 pose 到达
       （3000/3000，中位 +0.100 ms），pose 回调执行时缓冲区里最新的是上一帧，
       源错位正好一个帧周期 8.028 ms，必然超阈值。按帧装配用 :class:`FrameAssembler`。
    """
    best: ChannelSample | None = None
    best_skew = float("inf")
    for sample in buffer:
        skew_abs = abs(sample.source_time - target_time)
        if skew_abs < best_skew:
            best, best_skew = sample, skew_abs
    if best is None or best_skew > max_skew_sec:
        return None
    return best


# --------------------------------------------------------------- assembly

@dataclass
class AssembledFrame:
    """一个 Vicon 帧已到达的全部通道。

    key 是精确的源时间戳整数对 (sec, nanosec)，不用浮点：实测同一帧的
    pose/twist/accel 三条话题 header.stamp 逐位相同（3000/3000），整数相等就是最强
    的同帧判据，改用浮点反而要引入容差，而容差可能把相邻帧误配在一起。
    """

    key: tuple[int, int]
    samples: dict
    complete: bool          # True=收齐所有必需通道；False=等超时后放行
    wait_sec: float         # 第一条到达 -> 放行，实际等了多久
    missing: tuple[str, ...]


class FrameAssembler:
    """按精确源时间戳把多条通道装配成整帧，收齐即放行。

    等待代价只有「同帧各通道的到达跨度」，实测中位 0.158 ms、p95 0.424 ms、
    max 1.232 ms；相比之下整体延迟一帧要给控制回路加 8 ms（200 Hz 下 1.6 个控制拍），
    而放宽错位阈值到一个帧周期以上等于关掉错位检测本身。

    三条必须成立的边界行为：

    * **没配置的通道不计入齐全条件。** 只订阅 pose 时「齐全」等价于「pose 到了」，
      额外延迟严格为 0。
    * **从没来过的通道也不计入。** stamp=0 的通道会被上游整条丢弃、永不 submit，
      不能让它把每一帧都拖到超时。
    * **来过又断了的通道** 先由 timeout_sec 兜底，连续缺席 absent_frames_to_drop 帧
      后移出必需集合，稳态回到零等待；一旦恢复，submit 时自动重新计入。
    """

    def __init__(
        self,
        anchor: str,
        channels,
        timeout_sec: float = 0.003,       # 等齐一帧的上限 (s)。缺省 3 ms 大于实测最大到达跨度 1.232 ms、小于半个帧周期 4 ms。大→缺通道时白等更久；小→正常抖动就被判超时放行
        absent_frames_to_drop: int = 8,   # 连续缺席多少帧就把该通道移出必需集合。大→通道断了还要白等更久；小→偶发丢帧就把通道踢出、恢复后又加回来，反复横跳
        max_pending: int = 32,            # 待装配帧的队列深度。大→乱序容忍度高、内存占用大；小→积压时直接丢最老的帧
    ) -> None:
        self.anchor = anchor
        self.channels = tuple(dict.fromkeys(channels))
        if anchor not in self.channels:
            raise ValueError("anchor %r 必须在 channels 里" % anchor)
        self.timeout_sec = max(0.0, float(timeout_sec))
        self.absent_frames_to_drop = max(1, int(absent_frames_to_drop))
        self.max_pending = max(2, int(max_pending))
        self._pending: "OrderedDict[tuple, dict]" = OrderedDict()
        self._released: deque = deque(maxlen=256)
        self._released_set: set = set()
        self._alive: set = set()
        self._absent: dict = {c: 0 for c in self.channels}
        self.timeouts = 0
        self.dropped_anchorless = 0

    # ------------------------------------------------------------------
    def required(self) -> set:
        """本拍的"齐全"条件：锚通道 + 当前还活着的可选通道。"""
        return {self.anchor} | {c for c in self.channels
                                if c != self.anchor and c in self._alive}

    def submit(self, channel: str, key, payload, arrival: float) -> list:
        """收下一条通道样本，返回本次可以放行的帧（可能为空）。"""
        if channel not in self.channels:
            raise ValueError("未知通道 %r" % channel)
        key = tuple(key)
        self._alive.add(channel)
        self._absent[channel] = 0
        if key in self._released_set:
            # 迟到样本：那一帧已经放行过了。绝不能拿它重新开一个 pending，
            # 否则会攒出一堆只有辅助通道、没有锚通道的僵尸帧。
            return []
        slot = self._pending.get(key)
        if slot is None:
            slot = {"samples": {}, "first_arrival": arrival}
            self._pending[key] = slot
            # 只按 key 排序，不依赖通道的到达顺序
            self._pending = OrderedDict(sorted(self._pending.items()))
        slot["samples"][channel] = payload
        out = self._release(arrival)
        self._evict()
        return out

    def flush_expired(self, arrival: float) -> list:
        """只推进超时，不收新样本。给节点的定时器调用。

        没有它，"某条通道再也不来"这种情况要等下一帧 pose 到达（8 ms）才会
        被冲出去；有了它，等待上界就是 ``timeout_sec``。
        """
        out = self._release(arrival)
        self._evict()
        return out

    # ------------------------------------------------------------------
    def _release(self, arrival: float) -> list:
        released = []
        required = self.required()
        while self._pending:
            key, slot = next(iter(self._pending.items()))
            present = set(slot["samples"])
            complete = required <= present
            waited = arrival - slot["first_arrival"]
            if not complete and waited < self.timeout_sec:
                # 最老的一帧还没齐也还没超时 —— 停在这里，保证输出严格按源
                # 时间递增。超时 3 ms < 帧周期 8 ms，所以它一定先于下一帧放行。
                break
            self._pending.pop(key)
            self._note_released(key, present, complete)
            if self.anchor not in present:
                # 没有锚通道的帧无法发布（没有位姿就没有可发的状态）。
                self.dropped_anchorless += 1
                continue
            if not complete:
                self.timeouts += 1
            released.append(AssembledFrame(
                key=key,
                samples=dict(slot["samples"]),
                complete=complete,
                wait_sec=max(0.0, waited),
                missing=tuple(sorted(required - present)),
            ))
            required = self.required()
        return released

    def _note_released(self, key, present, complete) -> None:
        self._released.append(key)
        self._released_set.add(key)
        while len(self._released_set) > self._released.maxlen:
            self._released_set.discard(self._released.popleft())
        for channel in self.channels:
            if channel == self.anchor:
                continue
            if channel in present:
                self._absent[channel] = 0
            elif channel in self._alive:
                self._absent[channel] += 1
                if self._absent[channel] >= self.absent_frames_to_drop:
                    # 这条通道断了。继续把它算进必需集合，只会让每一帧都白等
                    # 一个 timeout；踢出去，等它回来时 submit 会自动重新加入。
                    self._alive.discard(channel)

    def _evict(self) -> None:
        while len(self._pending) > self.max_pending:
            key, _ = self._pending.popitem(last=False)
            self._released.append(key)
            self._released_set.add(key)


# --------------------------------------------------------------- consistency

@dataclass
class DerivativeConsistencyMonitor:
    """把上游自报的速度与本地差分出来的速度作差，判断是否粗差。

    判据取滑动窗中位数而非逐样本：偶然挤在一起的一对帧会给出一个巨大但无意义的
    瞬时差值。容差故意放宽 —— 本地差分带着桥那 40 ms 低通滞后，运动时任何严格的
    静态容差都是错的。这道检查只抓粗错（单位、坐标系、刚体），不做精度判据。
    """

    linear_tol: float = 0.05    # 线速度中位数差的容差 (m/s)。大→粗错也放行；小→本地差分的 40 ms 滞后就会误判
    angular_tol: float = 0.5    # 角速度中位数差的容差 (rad/s)，同上
    window_sec: float = 0.5     # 取中位数的滑动窗长 (s)。大→更抗偶发跳点但发现故障慢；小→单次挤帧就可能触发
    fail_sec: float = 2.0       # 持续不一致多久判失败 (s)。大→反应慢；小→易误报
    recover_sec: float = 2.0    # 持续一致多久才复位 (s)。大→恢复慢；小→在阈值边缘反复横跳
    _lin: deque = field(default_factory=lambda: deque(maxlen=256))
    _ang: deque = field(default_factory=lambda: deque(maxlen=256))
    _bad_since: float | None = None
    _good_since: float | None = None
    failed: bool = False
    recoveries: int = 0

    def update(self, now: float, v_reported, v_local, w_reported, w_local) -> tuple[float, float]:
        d_lin = float(np.linalg.norm(np.asarray(v_reported, dtype=float).reshape(3)
                                     - np.asarray(v_local, dtype=float).reshape(3)))
        d_ang = float(np.linalg.norm(np.asarray(w_reported, dtype=float).reshape(3)
                                     - np.asarray(w_local, dtype=float).reshape(3)))
        self._lin.append((now, d_lin))
        self._ang.append((now, d_ang))
        while self._lin and now - self._lin[0][0] > self.window_sec:
            self._lin.popleft()
        while self._ang and now - self._ang[0][0] > self.window_sec:
            self._ang.popleft()

        med_lin = float(np.median([d for _, d in self._lin])) if self._lin else 0.0
        med_ang = float(np.median([d for _, d in self._ang])) if self._ang else 0.0

        bad = med_lin > self.linear_tol or med_ang > self.angular_tol
        if bad:
            if self._bad_since is None:
                self._bad_since = now
            elif now - self._bad_since >= self.fail_sec:
                self.failed = True
                self._good_since = None
        else:
            self._bad_since = None
            # failed 必须能复位，否则链路抖一下桥就永久停发基座状态。
            # 要求连续 recover_sec 内都一致才复位，避免在阈值边缘反复横跳。
            if self.failed:
                if self._good_since is None:
                    self._good_since = now
                elif now - self._good_since >= self.recover_sec:
                    self.failed = False
                    self._good_since = None
                    self.recoveries += 1
        return med_lin, med_ang
