#!/usr/bin/env python3
"""不确定性感知基座误差估计器，纯 numpy 实现，无 ROS 依赖。

组成:
  参数通道 SpectralHarmonicRLS -- 基座实现误差 Δp 的谐波回归均值模型 θ̂:
                             谐波频率不再用固定对数网格(对 seed 抽取的随机
                             对数均匀频率线存在结构性失配), 而是数据驱动辨识:
                             维护 ≥20s 的 (t_meas, y) 环形缓冲, 慢速率(每
                             ident_period_sec=2s)对每轴做加窗周期图 + 匹配追踪
                             (CLEAN: 找谱峰 -> DTFT 局部细化 -> 联合最小二乘
                             重拟合 -> 减去再找下一条), 取前 n_peaks 条谱线
                             重建谐波库, 并用指数遗忘加权岭回归温启动 RLS;
                             新库须在末段 holdout(纯外推)上打败现役库才换入.
  快速残差 TwoRegimeResidualEstimator -- 统一三维状态的两工况 IMM:
                             瞬态衰减模型 + 持续偏差随机游走模型；输出融合均值、
                             含工况分歧项的协方差和工况概率。
  参数适用度 BetaConfidenceFilter -- 残差精度系数 β 的贝叶斯滤波，只评价
                             参数通道适用程度，不直接解释为整体预测可靠性。
  误差校准 ConformalCalibrator -- 滑窗保形分位数 δ + ACI 在线调 α, Mondrian 分组
                             (pen_down × 前瞻步数): h 步先验分数与一步分数
                             不可交换(非平稳失配下随 h 系统性增大), 故按
                             对数层级 L∈{1,2,4,...,H} 独立校准窗喂 θ̂ 冻结
                             的 L 步快照分数, 请求的 h 向上取最近层级(保守);
                             联合半径取 max_h δ_{L(h)}(α_L/N)(Bonferroni 只
                             修多重性, 边缘覆盖由各组自身校准 + ACI 闭环
                             保证). 快照打分与谱辨识共享分摊预算.

组合体 BaseTrustEstimator.step(t, e_meas, pen_down) 每拍喂入“基座实现误差测量”
(带噪声/延迟的 Δp 测量, 与 demo 的 estimated 通道同源);
predict_horizon_stats(times) 返回参数均值、快速残差均值、混合协方差、工况概率和
保形半径；predict_horizon(times) 保留兼容的 (Δ̂, δ) 接口。

时间约定(测量延迟的相位处理): 测量 y_k = Δ(t_k - τ_d) + ν, τ_d=meas_delay_sec
为名义已知量. 谐波基是全局时间函数, 所有回归特征一律构建在测量时刻
t_meas = t_k - τ_d 上(拟合 θ 后在任意未来绝对时刻求值, 相位自动正确);
环形缓冲、谱辨识、温启动同样全部使用 t_meas. 绝不读取任何真值通道.

50Hz step 单拍成本: 3 次单轴 RLS 更新, O((2K+1)²), K=n_peaks≤8.
谱辨识(实时性关键): 全量重辨识在 30s 缓冲 ×3 轴 ×K=12 时单块可达
150~250ms, **严禁**在 50Hz tick 内整块执行. 实现为分摊式状态机:
_identify_steps 生成器把流水线切成 ≤0.6ms 的原子单元(单次 FFT/单级
DTFT 细化/单次联合 LS), step() 每拍只推进 ident_units_per_tick 个单元
且受 ident_budget_sec 墙钟兜底(默认 5 单元/1.8ms, 最坏单拍增量 <3ms);
抬笔拍(sched_pen_down=False)放宽到 ident_units_pen_up/ident_budget_
pen_up_sec 加速收敛. 计算逐单元与整块执行完全同一数学, 仅完成时刻
推迟数十拍; 换库温启动在流水线末尾用**当前**缓冲重拟合, 与在线流无缝.
"""

import bisect
import math
import time
from collections import deque

import numpy as np


class HarmonicBank:
    """任意频率列表的谐波特征库.

    Φ(t) = [1, sin(2πf_1 t)…sin(2πf_K t), cos(2πf_1 t)…cos(2πf_K t)],
    维度 dim = 2K+1. 频率列表由谱辨识给出(每轴一个库), K=0 时退化为纯直流.
    """

    def __init__(self, freqs_hz):
        f = np.sort(np.asarray(freqs_hz, dtype=float).ravel())
        if f.size and (f <= 0.0).any():
            raise ValueError(f"frequencies must be positive, got {f}")
        self.freqs_hz = f
        self._omega = 2.0 * math.pi * f
        self.n_freq = int(f.size)
        self.dim = 2 * self.n_freq + 1

    @classmethod
    def log_grid(cls, n_freq: int, fmin_hz: float, fmax_hz: float):
        """冷启动用固定对数网格(仅在首次谱辨识前生效)."""
        n_freq = int(n_freq)
        if n_freq < 1:
            raise ValueError(f"n_freq must be >= 1, got {n_freq}")
        if fmin_hz <= 0.0 or fmax_hz <= fmin_hz:
            raise ValueError(
                f"need 0 < fmin_hz < fmax_hz, got ({fmin_hz}, {fmax_hz})")
        return cls(np.exp(np.linspace(
            math.log(fmin_hz), math.log(fmax_hz), n_freq)))

    def features(self, t) -> np.ndarray:
        """t 标量或 (n,) 数组 -> 特征矩阵 (n, dim)."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        if self.n_freq == 0:
            return np.ones((t.size, 1))
        arg = t[:, None] * self._omega[None, :]
        return np.concatenate(
            [np.ones((t.size, 1)), np.sin(arg), np.cos(arg)], axis=1)

    def feature(self, t: float) -> np.ndarray:
        """单时刻特征向量 (dim,)."""
        return self.features(t)[0]


class AxisRLS:
    """单轴高斯共轭递推最小二乘(带遗忘 λ).

    模型: e(t) = Φ(t)ᵀ μ + ε,  ε ~ N(0, σ0²). 每轴一个实例(各轴频率库独立).
    支持温启动: warm_start(mu, Sigma) 直接覆写后验(谱辨识重拟合后调用).
    """

    def __init__(self, dim: int, lam: float = 0.996, sigma0: float = 2.0e-3,
                 prior_std: float = 1.0e-2, jitter: float = 1.0e-12):
        if not (0.9 < lam <= 1.0):
            raise ValueError(f"lambda must be in (0.9, 1], got {lam}")
        if sigma0 <= 0.0 or prior_std <= 0.0:
            raise ValueError("sigma0/prior_std must be positive")
        self.lam = float(lam)
        self.sigma0 = float(sigma0)
        self.prior_std = float(prior_std)
        self.jitter = float(jitter)
        self._init_state(int(dim))
        self.n_updates = 0

    def _init_state(self, dim: int):
        self.dim = dim
        self.mu = np.zeros(dim)
        self.Sigma = (self.prior_std ** 2) * np.eye(dim)
        self._eye = np.eye(dim)

    def warm_start(self, mu: np.ndarray, Sigma: np.ndarray):
        mu = np.asarray(mu, dtype=float).ravel()
        self._init_state(mu.size)
        self.mu = mu
        self.Sigma = self._guard_psd(np.asarray(Sigma, dtype=float))

    def _guard_psd(self, S: np.ndarray) -> np.ndarray:
        """对称化 + jitter, 对角出现非正时追加修复量, 保 Σ 对称正定."""
        S = 0.5 * (S + S.T)
        dmin = float(np.min(np.diag(S)))
        bump = self.jitter if dmin > 0.0 else (self.jitter - dmin)
        return S + bump * self._eye

    def update(self, phi: np.ndarray, y: float) -> float:
        """喂入测量时刻特征 φ(t_meas) 与标量测量, 返回先验新息."""
        S = self.Sigma / self.lam                      # 遗忘: 先验协方差膨胀
        s_phi = S @ phi
        denom = self.sigma0 ** 2 + float(phi @ s_phi)  # 标量新息方差
        gain = s_phi / denom
        innov = float(y) - float(self.mu @ phi)
        self.mu = self.mu + gain * innov
        self.Sigma = self._guard_psd(S - np.outer(gain, s_phi))
        self.n_updates += 1
        return innov


def _spectral_lines_steps(t: np.ndarray, y: np.ndarray, out: dict,
                          n_peaks: int = 8, fmin_hz: float = 0.08,
                          fmax_hz: float = 5.0, peak_snr: float = 3.0,
                          n_refine: int = 3, n_relax: int = 2):
    """匹配追踪谱线辨识(RELAX/CLEAN)的**分步生成器**: 每两个 yield 之间只
    执行一个 O(n)~O(n log n) 原子单元(单次 FFT 找峰/单级 15 点 DTFT 细化/
    单次联合 LS 扣除), 供 50Hz tick 分摊推进; 结果写 out['freqs'](升序频率
    数组, 可能少于 n_peaks 条). 计算与整块执行完全同一数学.

    算法: 矩形窗周期图找峰 -> DTFT 局部细化 -> 联合 LS 重拟合减去 -> 迭代;
    收尾 n_relax 轮逐线再细化(固定其余线扣除后重估本线频率).
    对随机(离网)频率线, 单遍 FFT 峰值受泄漏/近邻线干扰; 逐线扣除(而非加窗)
    保留矩形窗 ±1/T 主瓣分辨率, 短窗(4-8s)下近邻线分辨明显优于 Hann;
    强线旁瓣在扣除后消失, 弱线随后可见. DTFT 三级缩放细化到 ~1e-3 Hz 级.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    out['freqs'] = np.empty(0)
    n = t.size
    if n < 32:
        return
    tr = t - t[0]                        # 相对时间(仅数值调理, 相位无关紧要)
    span = float(tr[-1])
    if span <= 0.0:
        return
    dt = span / (n - 1)
    fny = 0.5 / dt
    f_hi = min(float(fmax_hz), 0.9 * fny)
    f_lo = max(float(fmin_hz), 1.0 / span)
    if f_hi <= f_lo:
        return
    nfft = 1 << max(10, int(math.ceil(math.log2(4 * n))))
    fgrid = np.fft.rfftfreq(nfft, d=dt)
    band = (fgrid >= f_lo) & (fgrid <= f_hi)
    if not band.any():
        return
    df_coarse = 1.0 / (n * dt)           # 原始(未补零)频率分辨率

    def dtft_mag(x, f):
        return np.abs(np.exp(-2j * math.pi * np.outer(f, tr)) @ x)

    def refine_steps(x, fc, half, holder):
        """三级缩放细化, 每级一个原子单元(15 点 DTFT), 结果写 holder[0]."""
        for _ in range(int(n_refine)):
            yield
            cand = np.linspace(max(fc - half, f_lo), min(fc + half, f_hi), 15)
            fc = float(cand[int(np.argmax(dtft_mag(x, cand)))])
            half /= 7.0
        holder[0] = fc

    freqs = []
    resid = y - float(np.mean(y))
    for _ in range(int(n_peaks)):
        yield                            # 单元: 补零 FFT + 找峰 + 噪声地板
        spec = np.abs(np.fft.rfft(resid, nfft))
        spec = np.where(band, spec, 0.0)
        for f0 in freqs:                 # 已选线邻域禁选(防泄漏残留重触发)
            spec[np.abs(fgrid - f0) < 0.8 * df_coarse] = 0.0
        k = int(np.argmax(spec))
        floor = float(np.median(spec[band & (spec > 0.0)])) \
            if (band & (spec > 0.0)).any() else 0.0
        if spec[k] <= 0.0 or (floor > 0.0 and spec[k] < peak_snr * floor):
            break
        # DTFT 缩放细化: 以补零 FFT 峰为中心, ±1 个原始 bin, 三级 15 点网格
        holder = [float(fgrid[k])]
        yield from refine_steps(resid, float(fgrid[k]), df_coarse, holder)
        freqs.append(holder[0])
        # 联合 LS 重拟合当前全部谱线(含直流), 从原始信号扣除(两个单元)
        yield                            # 单元: 特征矩阵构建
        Phi = HarmonicBank(freqs).features(tr)
        yield                            # 单元: 最小二乘求解
        coef, *_ = np.linalg.lstsq(Phi, y, rcond=None)
        resid = y - Phi @ coef
    # RELAX 轮: 固定其余线, 对每条线在"扣除其余线后的残差"上再细化频率
    for _ in range(int(n_relax) if len(freqs) > 1 else 0):
        for i in range(len(freqs)):
            others = freqs[:i] + freqs[i + 1:]
            yield                        # 单元: 其余线特征矩阵
            Phi = HarmonicBank(others).features(tr)
            yield                        # 单元: 其余线 LS 扣除
            coef, *_ = np.linalg.lstsq(Phi, y, rcond=None)
            resid_i = y - Phi @ coef
            holder = [freqs[i]]
            yield from refine_steps(resid_i, freqs[i], 0.5 * df_coarse,
                                    holder)
            freqs[i] = holder[0]
    out['freqs'] = np.sort(np.asarray(freqs, dtype=float))


def find_spectral_lines(t: np.ndarray, y: np.ndarray, n_peaks: int = 8,
                        fmin_hz: float = 0.08, fmax_hz: float = 5.0,
                        peak_snr: float = 3.0, n_refine: int = 3,
                        n_relax: int = 2):
    """整块(阻塞)执行 _spectral_lines_steps, 语义与历史版本逐位一致.

    仅供离线脚本/自测使用; 50Hz 在线路径一律走分摊生成器.
    """
    out = {}
    for _ in _spectral_lines_steps(t, y, out, n_peaks=n_peaks,
                                   fmin_hz=fmin_hz, fmax_hz=fmax_hz,
                                   peak_snr=peak_snr, n_refine=n_refine,
                                   n_relax=n_relax):
        pass
    return out['freqs']


def trust_feedforward_gain(trust: float, low: float = 0.3,
                           high: float = 0.7) -> float:
    """把信任概率映射为模型前馈增益 ``g(b)``。

    ``b <= low`` 时模型完全退出、回到 nominal；``b >= high`` 时使用完整
    模型前馈；中间用三次 smoothstep 连续过渡，避免阈值附近把参考位置的一阶
    导数打出尖峰。非有限信任按最保守的 0 处理。
    """
    low, high = float(low), float(high)
    if not (0.0 <= low < high <= 1.0):
        raise ValueError(
            f"trust gate requires 0 <= low < high <= 1, got {low}, {high}")
    b = float(trust)
    if not math.isfinite(b):
        return 0.0
    x = min(1.0, max(0.0, (b - low) / (high - low)))
    return float(x * x * (3.0 - 2.0 * x))


class TrustFeedforwardGate:
    """带非对称时间平滑的 ``g(b)``。

    β 后验会在谱库切换或偶发大残差时产生短暂低脉冲。直接逐拍门控虽在数值
    上连续，却会把该脉冲变成参考位置突变。这里先对 b 做一阶非对称滤波：
    降信任较快、恢复较慢，再经 smoothstep 映射。持续故障仍会在 2s 内退出
    模型，健康段数百毫秒的孤立低脉冲不会吃掉谐波前馈收益。
    """

    def __init__(self, low: float = 0.2, high: float = 0.5,
                 fall_tau_sec: float = 0.3, rise_tau_sec: float = 1.0,
                 initial_trust: float = 0.5):
        trust_feedforward_gain(0.5, low, high)  # 集中校验阈值
        if fall_tau_sec <= 0.0 or rise_tau_sec <= 0.0:
            raise ValueError("trust gate time constants must be positive")
        self.low = float(low)
        self.high = float(high)
        self.fall_tau_sec = float(fall_tau_sec)
        self.rise_tau_sec = float(rise_tau_sec)
        self.initial_trust = float(np.clip(initial_trust, 0.0, 1.0))
        self.reset()

    def reset(self):
        self.filtered_trust = self.initial_trust
        self.gain = trust_feedforward_gain(
            self.filtered_trust, self.low, self.high)

    def update(self, trust: float, dt: float) -> float:
        b, dt = float(trust), float(dt)
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError("trust gate dt must be finite and non-negative")
        if not math.isfinite(b):
            b = 0.0
        b = min(1.0, max(0.0, b))
        tau = (self.fall_tau_sec
               if b < self.filtered_trust else self.rise_tau_sec)
        alpha = 1.0 - math.exp(-dt / tau)
        self.filtered_trust += alpha * (b - self.filtered_trust)
        self.gain = trust_feedforward_gain(
            self.filtered_trust, self.low, self.high)
        return self.gain


class BetaConfidenceFilter:
    """残差精度系数 β 的网格贝叶斯滤波, 输出连续健康分数 b.

    似然 N(e; Φθ̂, σ0²/β), β 取对数网格 10^linspace(log10_min, log10_max, n_grid);
    遗忘 γ_f 通过对 log 权重回火(向均匀先验收缩)实现.

    旧实现用 ``E[β]/β_max`` 归一化；网格上限 β_max=10 导致健康残差
    (β≈1) 被错误报告为 b≈0.1。新语义定义为
    先算后验精度均值 E[β]，再在 ``[β_degraded, β_healthy]`` 上用
    smoothstep 标定为 [0,1]。默认 0.6/0.9 分别对应残差标准差为名义 σ0 的
    1.29/1.05 倍；健康模型残差落在 b=0.7–1.0，明显失配在 2s 内落到
    b<0.3。该语义不依赖任意的网格上限，且在网格点之间连续。
    """

    def __init__(self, sigma0: float = 2.0e-3, n_grid: int = 15,
                 log10_min: float = -1.0, log10_max: float = 1.0,
                 gamma_f: float = 0.98, degraded_beta: float = 0.6,
                 healthy_beta: float = 0.9):
        if not (0.0 < gamma_f <= 1.0):
            raise ValueError(f"gamma_f must be in (0, 1], got {gamma_f}")
        if sigma0 <= 0.0:
            raise ValueError("sigma0 must be positive")
        if (not math.isfinite(degraded_beta)
                or not math.isfinite(healthy_beta)
                or not 0.0 < degraded_beta < healthy_beta):
            raise ValueError(
                "requires 0 < degraded_beta < healthy_beta")
        self.betas = 10.0 ** np.linspace(
            float(log10_min), float(log10_max), int(n_grid))
        if not (self.betas[0] <= degraded_beta < healthy_beta
                <= self.betas[-1]):
            raise ValueError(
                f"beta calibration [{degraded_beta}, {healthy_beta}] "
                f"is outside beta grid "
                f"[{self.betas[0]}, {self.betas[-1]}]")
        self.sigma0 = float(sigma0)
        self.gamma_f = float(gamma_f)
        self.degraded_beta = float(degraded_beta)
        self.healthy_beta = float(healthy_beta)
        self.log_w = np.full(len(self.betas), -math.log(len(self.betas)))

    def update(self, resid) -> float:
        """喂入当拍先验残差 e - Φθ̂ (任意维), 返回更新后的信任 b."""
        r = np.asarray(resid, dtype=float).ravel()
        r2 = float(r @ r)
        n = r.size
        # log N(r; 0, σ0²/β) 中与 β 有关的项: (n/2)ln β - β r²/(2σ0²)
        loglik = 0.5 * n * np.log(self.betas) \
            - self.betas * (r2 / (2.0 * self.sigma0 ** 2))
        lw = self.gamma_f * self.log_w + loglik
        lw -= float(np.max(lw))
        lw -= math.log(float(np.sum(np.exp(lw))))
        self.log_w = lw
        return self.trust

    @property
    def trust(self) -> float:
        """E[β] 经物理精度锚点 smoothstep 标定后的 b ∈ [0,1]."""
        x = ((self.beta_mean - self.degraded_beta)
             / (self.healthy_beta - self.degraded_beta))
        x = min(1.0, max(0.0, x))
        return float(x * x * (3.0 - 2.0 * x))

    @property
    def beta_mean(self) -> float:
        """后验 E[β]，仅供诊断；不再拿网格上限做信任归一化。"""
        return float(np.exp(self.log_w) @ self.betas)


class TwoRegimeResidualEstimator:
    """统一三维状态的两工况快速残差 IMM。

    工况 0 为向零衰减的瞬态残差，工况 1 为随机游走的持续偏差。两者共享
    ``x_r∈R³`` 的状态维数，因而可以执行标准 IMM 混合。观测是参数通道未解释
    的三轴残差 ``r = e - Φθ``。模式概率只表示哪一种残差动力学更符合数据，
    不解释为预测可靠性；可靠性由混合协方差和外层保形误差共同表示。
    """

    N_AXES = 3

    def __init__(
            self, dt: float = 0.02, stable_tau_sec: float = 0.12,
            stable_process_std: float = 8.0e-4,
            persistent_process_std: float = 2.5e-3,
            measurement_std: float = 2.0e-3,
            stable_stay_probability: float = 0.985,
            persistent_stay_probability: float = 0.97,
            initial_persistent_probability: float = 0.02,
            initial_std: float = 5.0e-3,
            covariance_floor: float = 1.0e-10):
        values = (
            dt, stable_tau_sec, stable_process_std,
            persistent_process_std, measurement_std, initial_std,
            covariance_floor,
        )
        if not np.all(np.isfinite(values)) or min(values) <= 0.0:
            raise ValueError('two-regime positive parameters must be finite')
        probabilities = (
            stable_stay_probability, persistent_stay_probability,
            initial_persistent_probability,
        )
        if (not np.all(np.isfinite(probabilities))
                or not 0.0 < stable_stay_probability < 1.0
                or not 0.0 < persistent_stay_probability < 1.0
                or not 0.0 <= initial_persistent_probability <= 1.0):
            raise ValueError('two-regime probabilities are outside [0,1]')
        self.dt = float(dt)
        self.a = np.array([
            math.exp(-self.dt / float(stable_tau_sec)),
            1.0,
        ])
        # 连续噪声强度按 sqrt(dt) 离散，避免改变控制频率后随机游走尺度失真。
        self.q = self.dt * np.square(np.array([
            stable_process_std, persistent_process_std], dtype=float))
        self.R = (float(measurement_std) ** 2) * np.eye(self.N_AXES)
        self.transition = np.array([
            [stable_stay_probability, 1.0 - stable_stay_probability],
            [1.0 - persistent_stay_probability,
             persistent_stay_probability],
        ], dtype=float)
        self.covariance_floor = float(covariance_floor)
        p1 = float(initial_persistent_probability)
        self.mode_probabilities = np.array([1.0 - p1, p1])
        self.states = np.zeros((2, self.N_AXES))
        self.covariances = np.repeat(
            ((float(initial_std) ** 2) * np.eye(self.N_AXES))[None, :, :],
            2, axis=0)
        self.prior_mean = np.zeros(self.N_AXES)
        self.prior_covariance = (
            float(initial_std) ** 2) * np.eye(self.N_AXES)
        self.mean = np.zeros(self.N_AXES)
        self.covariance = self.prior_covariance.copy()

    def _guard_covariance(self, covariance: np.ndarray) -> np.ndarray:
        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(eigenvalues, self.covariance_floor)
        return (eigenvectors * eigenvalues[None, :]) @ eigenvectors.T

    @staticmethod
    def _mixture_moments(weights, means, covariances):
        weights = np.asarray(weights, dtype=float)
        means = np.asarray(means, dtype=float)
        mean = weights @ means
        covariance = np.zeros((means.shape[1], means.shape[1]))
        for weight, component_mean, component_covariance in zip(
                weights, means, covariances):
            difference = component_mean - mean
            covariance += weight * (
                component_covariance + np.outer(difference, difference))
        return mean, covariance

    def step(self, residual) -> dict:
        """用当拍三轴谐波残差更新 IMM，并返回更新前后混合统计量。"""
        observation = np.asarray(residual, dtype=float).reshape(self.N_AXES)
        if not np.all(np.isfinite(observation)):
            raise ValueError('two-regime residual must be finite')
        identity = np.eye(self.N_AXES)
        destination_probabilities = (
            self.mode_probabilities @ self.transition)
        destination_probabilities = np.maximum(
            destination_probabilities, np.finfo(float).tiny)
        mixed_states = np.zeros_like(self.states)
        mixed_covariances = np.zeros_like(self.covariances)
        predicted_states = np.zeros_like(self.states)
        predicted_covariances = np.zeros_like(self.covariances)
        for destination in range(2):
            weights = (
                self.mode_probabilities
                * self.transition[:, destination]
                / destination_probabilities[destination])
            mixed_state, mixed_covariance = self._mixture_moments(
                weights, self.states, self.covariances)
            mixed_states[destination] = mixed_state
            mixed_covariances[destination] = self._guard_covariance(
                mixed_covariance)
            predicted_states[destination] = (
                self.a[destination] * mixed_state)
            predicted_covariances[destination] = self._guard_covariance(
                (self.a[destination] ** 2) * mixed_covariance
                + self.q[destination] * identity)

        prior_mean, prior_covariance = self._mixture_moments(
            destination_probabilities,
            predicted_states, predicted_covariances)
        self.prior_mean = prior_mean
        self.prior_covariance = self._guard_covariance(prior_covariance)

        updated_states = np.zeros_like(self.states)
        updated_covariances = np.zeros_like(self.covariances)
        log_weights = np.empty(2)
        for mode in range(2):
            innovation = observation - predicted_states[mode]
            innovation_covariance = self._guard_covariance(
                predicted_covariances[mode] + self.R)
            sign, logdet = np.linalg.slogdet(innovation_covariance)
            if sign <= 0.0:
                raise RuntimeError('two-regime innovation covariance not PD')
            solved = np.linalg.solve(innovation_covariance, innovation)
            log_likelihood = -0.5 * (
                self.N_AXES * math.log(2.0 * math.pi)
                + logdet + float(innovation @ solved))
            log_weights[mode] = (
                math.log(destination_probabilities[mode]) + log_likelihood)
            gain = np.linalg.solve(
                innovation_covariance, predicted_covariances[mode].T).T
            updated_states[mode] = (
                predicted_states[mode] + gain @ innovation)
            # Joseph 形式在长期在线运行中比 (I-K)P 更稳定。
            left = identity - gain
            updated_covariances[mode] = self._guard_covariance(
                left @ predicted_covariances[mode] @ left.T
                + gain @ self.R @ gain.T)

        log_weights -= float(np.max(log_weights))
        mode_probabilities = np.exp(log_weights)
        mode_probabilities /= float(np.sum(mode_probabilities))
        self.mode_probabilities = mode_probabilities
        self.states = updated_states
        self.covariances = updated_covariances
        self.mean, covariance = self._mixture_moments(
            mode_probabilities, updated_states, updated_covariances)
        self.covariance = self._guard_covariance(covariance)
        return {
            'prior_mean': self.prior_mean.copy(),
            'prior_covariance': self.prior_covariance.copy(),
            'posterior_mean': self.mean.copy(),
            'posterior_covariance': self.covariance.copy(),
            'mode_probabilities': self.mode_probabilities.copy(),
            'persistent_probability': float(self.mode_probabilities[1]),
        }

    def predict_horizon(self, steps) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """无测量外推到给定非负拍数，返回均值、协方差和工况概率。"""
        steps = np.asarray(steps, dtype=int).ravel()
        if np.any(steps < 0):
            raise ValueError('two-regime horizon steps must be non-negative')
        means = np.zeros((steps.size, self.N_AXES))
        covariances = np.zeros((steps.size, self.N_AXES, self.N_AXES))
        probabilities = np.zeros((steps.size, 2))
        identity = np.eye(self.N_AXES)
        # 预测网格通常是连续拍数。按最大拍数只推进一次，避免对每个目标拍
        # 从 0 重算而形成 O(H²) 开销。
        requested = {}
        for output_index, target_step in enumerate(steps.tolist()):
            requested.setdefault(target_step, []).append(output_index)
        mode_probabilities = self.mode_probabilities.copy()
        states = self.states.copy()
        mode_covariances = self.covariances.copy()
        for current_step in range(max(requested, default=0) + 1):
            if current_step in requested:
                mean, covariance = self._mixture_moments(
                    mode_probabilities, states, mode_covariances)
                covariance = self._guard_covariance(covariance)
                for output_index in requested[current_step]:
                    means[output_index] = mean
                    covariances[output_index] = covariance
                    probabilities[output_index] = mode_probabilities
            if current_step == max(requested, default=0):
                break
            destination_probabilities = mode_probabilities @ self.transition
            destination_probabilities = np.maximum(
                destination_probabilities, np.finfo(float).tiny)
            next_states = np.zeros_like(states)
            next_covariances = np.zeros_like(mode_covariances)
            for destination in range(2):
                weights = (
                    mode_probabilities
                    * self.transition[:, destination]
                    / destination_probabilities[destination])
                mixed_state, mixed_covariance = self._mixture_moments(
                    weights, states, mode_covariances)
                next_states[destination] = (
                    self.a[destination] * mixed_state)
                next_covariances[destination] = self._guard_covariance(
                    (self.a[destination] ** 2) * mixed_covariance
                    + self.q[destination] * identity)
            mode_probabilities = (
                destination_probabilities
                / float(np.sum(destination_probabilities)))
            states = next_states
            mode_covariances = next_covariances
        return means, covariances, probabilities


class UncertaintyAwareFeedforwardDecision:
    """由预测收益与风险预算共同决定补偿、权重和任务速度。

    参数适用度不直接缩放任何预测均值，而是放大参数失配对应的风险项。补偿增益是
    二次风险代理
    ``(1-g)^2 ||d||² + λ_r g² R + λ_s(g-g_prev)^2``
    的闭式极小值；其中 ``R`` 同时包含混合协方差与保形半径。这样持续偏差被
    快速模型稳定解释后，即使参数适用度降低，仍可以维持有收益的补偿。
    """

    def __init__(
            self, applicability_low: float = 0.2,
            applicability_high: float = 0.5,
            risk_weight: float = 0.25,
            applicability_risk_gain: float = 1.0,
            smoothing_weight: float = 2.5e-5,
            fall_tau_sec: float = 0.15,
            rise_tau_sec: float = 0.45,
            risk_budget_m: float = 0.012,
            min_speed_scale: float = 0.3,
            min_position_weight_scale: float = 0.35):
        trust_feedforward_gain(
            0.5, applicability_low, applicability_high)
        positive = (
            risk_weight, applicability_risk_gain, smoothing_weight, fall_tau_sec,
            rise_tau_sec, risk_budget_m,
        )
        if not np.all(np.isfinite(positive)) or min(positive) <= 0.0:
            raise ValueError('risk decision positive parameters must be finite')
        if (not 0.0 < min_speed_scale <= 1.0
                or not 0.0 < min_position_weight_scale <= 1.0):
            raise ValueError('risk decision minimum scales must be in (0,1]')
        self.applicability_low = float(applicability_low)
        self.applicability_high = float(applicability_high)
        self.risk_weight = float(risk_weight)
        self.applicability_risk_gain = float(applicability_risk_gain)
        self.smoothing_weight = float(smoothing_weight)
        self.fall_tau_sec = float(fall_tau_sec)
        self.rise_tau_sec = float(rise_tau_sec)
        self.risk_budget_m = float(risk_budget_m)
        self.min_speed_scale = float(min_speed_scale)
        self.min_position_weight_scale = float(min_position_weight_scale)
        self.gain = 0.0

    def reset(self):
        self.gain = 0.0

    def update(
            self, harmonic_prediction, fast_prediction, covariance,
            conformal_delta: float, parameter_applicability: float,
            dt: float) -> dict:
        harmonic = np.asarray(harmonic_prediction, dtype=float)
        fast = np.asarray(fast_prediction, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        if (harmonic.ndim != 2 or harmonic.shape[1] != 3
                or fast.shape != harmonic.shape
                or covariance.shape != (harmonic.shape[0], 3, 3)
                or not np.all(np.isfinite(harmonic))
                or not np.all(np.isfinite(fast))
                or not np.all(np.isfinite(covariance))):
            raise ValueError('risk decision horizon arrays have invalid shape')
        delta = float(conformal_delta)
        dt = float(dt)
        if not math.isfinite(delta) or delta < 0.0:
            raise ValueError('conformal_delta must be finite and non-negative')
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError('risk decision dt must be finite and non-negative')
        applicability_gain = trust_feedforward_gain(
            parameter_applicability,
            self.applicability_low, self.applicability_high)
        # 均值始终使用完整融合结果。参数通道不适用时，由风险项而不是硬门控
        # 处理；持续偏差的快速补偿不会因同一个 β 被连带删除。
        candidate = harmonic + fast
        signal_power = float(np.mean(np.sum(np.square(candidate), axis=1)))
        covariance_power = float(np.mean(
            np.trace(covariance, axis1=1, axis2=2)))
        applicability_multiplier = (
            1.0 + self.applicability_risk_gain
            * (1.0 - applicability_gain))
        risk_power = (
            applicability_multiplier * covariance_power
            + 3.0 * delta * delta)
        numerator = signal_power + self.smoothing_weight * self.gain
        denominator = (
            signal_power + self.risk_weight * risk_power
            + self.smoothing_weight + np.finfo(float).eps)
        raw_gain = float(np.clip(numerator / denominator, 0.0, 1.0))
        tau = (
            self.fall_tau_sec if raw_gain < self.gain else self.rise_tau_sec)
        alpha = 1.0 - math.exp(-dt / tau) if dt > 0.0 else 0.0
        self.gain += alpha * (raw_gain - self.gain)
        uncertainty_radius = math.sqrt(max(risk_power, 0.0))
        if uncertainty_radius <= self.risk_budget_m:
            speed_scale = 1.0
        else:
            speed_scale = math.sqrt(
                self.risk_budget_m / uncertainty_radius)
        speed_scale = float(np.clip(
            speed_scale, self.min_speed_scale, 1.0))
        position_weight_scale = float(np.clip(
            speed_scale * speed_scale,
            self.min_position_weight_scale, 1.0))
        return {
            'candidate_prediction': candidate,
            'compensated_prediction': self.gain * candidate,
            'parameter_applicability_gain': applicability_gain,
            'parameter_risk_multiplier': applicability_multiplier,
            'feedforward_gain': float(self.gain),
            'feedforward_gain_raw': raw_gain,
            'signal_power': signal_power,
            'risk_power': risk_power,
            'uncertainty_radius': uncertainty_radius,
            'speed_scale': speed_scale,
            'position_weight_scale': position_weight_scale,
        }


class ConformalCalibrator:
    """滑窗保形分位数 + ACI 在线调 α, Mondrian 分组 (pen_down, 前瞻步数 h).

    分数 s_h = ‖e(t₀+hΔt) - Δ̂_{t₀}(t₀+hΔt)‖∞ (θ̂ 冻结在预测拍 t₀ 的 h 步
    先验分数, 三轴联合盒); h=1 组即历史上的一步先验残差通道.
    多步分数与一步分数**不可交换**: RLS 跟踪非平稳失配(快变失配主导的目标
    工况)时 h 步先验分数随 h 系统性增大, 单一 1 步窗 + Bonferroni(只修
    多重性, 前提是每个 h 的边缘覆盖成立)对多步外推不是有效联合界(实测
    N=25 时联合覆盖可跌到 ~0.56). 故每个 (pen, h) 组独立滑窗 + 独立
    ACI(闭环目标: 组内边缘覆盖 1-α); 联合半径见 quantile_joint.
    ACI: α_g ← clip(α_g + γ_aci (α* - err)), err = 1{未覆盖}.
    窗口为预分配环形 numpy 缓冲, 分位数用 np.partition(O(m)), 50Hz 下
    每拍 O(H) 次组更新仍为亚毫秒级.
    """

    def __init__(self, n_cal: int = 250, alpha: float = 0.1,
                 gamma_aci: float = 0.01, alpha_min: float = 1.0e-3,
                 alpha_max: float = 0.5, delta_fallback: float = 0.05):
        if not (0.0 < alpha < 1.0):
            raise ValueError(f"alpha must be in (0, 1), got {alpha}")
        if int(n_cal) < 1:
            raise ValueError(f"n_cal must be >= 1, got {n_cal}")
        self.alpha_target = float(alpha)
        self.gamma_aci = float(gamma_aci)
        self.alpha_min = float(alpha_min)
        self.alpha_max = float(alpha_max)
        self.delta_fallback = float(delta_fallback)
        self.n_cal = int(n_cal)
        self._win = {}     # (pen: bool, h: int) -> [buf (n_cal,), m, head]
        self._alpha = {}   # (pen: bool, h: int) -> float

    @staticmethod
    def _key(pen_down: bool, h: int = 1):
        return (bool(pen_down), max(1, int(h)))

    def _window(self, key):
        win = self._win.get(key)
        if win is None:
            win = [np.empty(self.n_cal), 0, 0]
            self._win[key] = win
            self._alpha[key] = self.alpha_target
        return win

    def _quantile_key(self, key, n_joint: int) -> float:
        win = self._win.get(key)
        if win is None or win[1] == 0:
            return self.delta_fallback
        buf, m, _ = win
        alpha_eff = max(self._alpha.get(key, self.alpha_target),
                        self.alpha_min) / max(1, int(n_joint))
        k = math.ceil((m + 1) * (1.0 - alpha_eff))
        scores = buf[:m]
        if k > m:  # 有限样本修正越界: 取窗内最大值兜底(不返回无穷)
            return float(scores.max())
        return float(np.partition(scores, k - 1)[k - 1])

    def quantile(self, pen_down: bool, n_joint: int = 1,
                 h: int = 1) -> float:
        """组 (pen_down, h) 的保形半径 δ, 边缘界 1-α_h/n_joint."""
        return self._quantile_key(self._key(pen_down, h), n_joint)

    def quantile_joint(self, pen_down: bool, hs, n_joint: int = None) -> float:
        """前瞻步数集合 hs 的联合半径: max_h δ_{L(h)}(α/N), N=n_joint.

        每个 h 向上映射到同 pen 侧最近的已校准组 L(h) ≥ h(多步分数随 h
        随机增大, 上取保守; 调用方只需喂对数层级组, 此处即隐式完成层级
        映射), 每组边缘 1-α_L/N (Bonferroni 修多重性, 边缘覆盖由该组
        自身校准窗 + ACI 闭环保证), 取 max 保持标量半径接口且联合界
        只增不减. 无更大已校准组时回退最大已校准组; 完全无数据时回退
        delta_fallback.
        """
        pen = bool(pen_down)
        hs = np.asarray(hs, dtype=int).ravel()
        if n_joint is None:
            n_joint = int(hs.size)
        populated = sorted(
            h for (p, h), win in self._win.items()
            if p == pen and win[1] > 0)
        if not populated:
            return self.delta_fallback
        delta = 0.0
        seen = set()
        for h in np.unique(np.maximum(hs, 1)):
            i = bisect.bisect_left(populated, int(h))
            h_eff = populated[i] if i < len(populated) else populated[-1]
            if h_eff in seen:
                continue
            seen.add(h_eff)
            delta = max(delta, self._quantile_key((pen, h_eff), n_joint))
        return delta

    def update(self, score: float, pen_down: bool, h: int = 1):
        """喂入组 (pen_down, h) 新分数: 先按当前 δ 判定覆盖并做 ACI,
        再入窗. 返回 (covered, δ)."""
        key = self._key(pen_down, h)
        win = self._window(key)
        delta = self._quantile_key(key, 1)
        covered = bool(float(score) <= delta)
        err = 0.0 if covered else 1.0
        a = self._alpha[key] + self.gamma_aci * (self.alpha_target - err)
        if a < self.alpha_min:
            a = self.alpha_min
        elif a > self.alpha_max:
            a = self.alpha_max
        self._alpha[key] = a
        buf, m, head = win
        buf[head] = float(score)
        win[1] = min(m + 1, self.n_cal)
        win[2] = (head + 1) % self.n_cal
        return covered, delta

    def alpha(self, pen_down: bool, h: int = 1) -> float:
        return self._alpha.get(self._key(pen_down, h), self.alpha_target)


class BaseTrustEstimator:
    """组合体: 谱辨识谐波库 + 两工况快速残差 + 参数适用度 + 保形校准。

    step(t, e_meas, pen_down): 每拍喂"基座实现误差测量"(带噪声/延迟的 Δp 测量,
    与 demo 的 estimated 通道同源). 名义测量延迟 meas_delay_sec 为已知量,
    一切回归特征取 Φ(t - meas_delay_sec)(测量时间戳), 不使用任何真值信息.
    β 作用于谐波参数通道先验残差；保形分数作用于最终融合预测先验误差。

    谱辨识(慢速率, 每 ident_period_sec **触发**, 分摊执行): 对环形缓冲
    (容量 buffer_sec ≥ 20s, span ≥ min_ident_sec 才启动)每轴匹配追踪找
    ≤ n_peaks 条谱线, 指数遗忘加权岭回归温启动; 候选库在末段 holdout_sec
    纯外推对比中打败现役库才换入(防故障暂态/噪声峰导致的倒退).
    实时性: 触发时对缓冲拍快照并创建 _identify_steps 生成器任务, 之后每拍
    step() 只推进 ident_units_per_tick 个原子单元(墙钟兜底 ident_budget_sec,
    默认 5 单元/1.8ms; sched_pen_down=False 的抬笔拍放宽到 ident_units_
    pen_up/ident_budget_pen_up_sec), 全流水线 ~2s 内完成, 任何单拍增量
    <3ms. 任务在途时不重复触发; deepcopy/pickle 丢弃在途任务(生成器不可
    拷贝, 冻结快照本就不再喂数).

    predict_horizon(times, pen_down) -> (Δ̂ (n,3), δ), δ 为联合界 1-α 的
    保形半径: 各时刻按距最近测量的前瞻步数 h 向上取最近对数校准层级
    L∈{1,2,4,...,H}, 每层级用自己的校准窗(由每拍推入的 θ̂ 冻结前瞻快照
    在到期时打分喂入, 见 _push/_score_horizon_snapshot*)取边缘 1-α_L/n
    分位数, δ = max_h δ_{L(h)}.
    Δ̂ 为谐波参数均值与快速残差工况混合均值之和；混合协方差显式包含
    工况内协方差和工况均值分歧项。
    """

    N_AXES = 3

    def __init__(self, n_freq: int = 6, fmin_hz: float = 0.15,
                 fmax_hz: float = 2.5, lam: float = 0.996,
                 sigma0: float = 2.0e-3, prior_std: float = 1.0e-2,
                 meas_delay_sec: float = 0.0, dt_nominal: float = 0.02,
                 n_peaks: int = 8, ident_period_sec: float = 2.0,
                 buffer_sec: float = 30.0, min_ident_sec: float = 2.5,
                 holdout_sec: float = 1.0, f_ident_min_hz: float = 0.08,
                 f_ident_max_hz: float = 5.0, peak_snr: float = 3.0,
                 resid_lpf_hz: float = 0.0,
                 beta_sigma0: float = None, beta_gamma_f: float = 0.98,
                 beta_n_grid: int = 15, beta_degraded: float = 0.6,
                 beta_healthy: float = 0.9,
                 n_cal: int = 250, alpha: float = 0.1,
                 gamma_aci: float = 0.01,
                 ident_units_per_tick: int = 5,
                 ident_budget_sec: float = 1.8e-3,
                 ident_units_pen_up: int = 8,
                 ident_budget_pen_up_sec: float = 2.6e-3,
                 ident_decim: int = 1,
                 cal_horizon_steps: int = 40,
                 regime_enabled: bool = False,
                 regime_stable_tau_sec: float = 0.12,
                 regime_stable_process_std: float = 8.0e-4,
                 regime_persistent_process_std: float = 2.5e-3,
                 regime_measurement_std: float = None,
                 regime_stable_stay_probability: float = 0.985,
                 regime_persistent_stay_probability: float = 0.97,
                 regime_initial_persistent_probability: float = 0.02):
        if buffer_sec < 20.0:
            raise ValueError(
                f"buffer_sec must be >= 20 s for spectral ident, "
                f"got {buffer_sec}")
        self.meas_delay_sec = float(meas_delay_sec)
        self.dt_nominal = float(dt_nominal)
        self.n_peaks = int(n_peaks)
        self.ident_period_sec = float(ident_period_sec)
        self.min_ident_sec = float(min_ident_sec)
        self.holdout_sec = float(holdout_sec)
        self.f_ident_min_hz = float(f_ident_min_hz)
        self.f_ident_max_hz = float(f_ident_max_hz)
        self.peak_snr = float(peak_snr)
        self._resid_alpha = (
            1.0 - math.exp(-2.0 * math.pi * float(resid_lpf_hz)
                           * self.dt_nominal)
            if resid_lpf_hz > 0.0 else 0.0)
        self._resid_lpf = np.zeros(self.N_AXES)
        self._lam = float(lam)
        self._sigma0 = float(sigma0)
        self._prior_std = float(prior_std)
        bank0 = HarmonicBank.log_grid(n_freq, fmin_hz, fmax_hz)
        self.banks = [bank0 for _ in range(self.N_AXES)]
        self.rls = [AxisRLS(bank0.dim, lam=lam, sigma0=sigma0,
                            prior_std=prior_std)
                    for _ in range(self.N_AXES)]
        self.beta_filter = BetaConfidenceFilter(
            sigma0=(sigma0 if beta_sigma0 is None else beta_sigma0),
            n_grid=beta_n_grid, gamma_f=beta_gamma_f,
            degraded_beta=beta_degraded, healthy_beta=beta_healthy)
        self.regime_enabled = bool(regime_enabled)
        self.regime = (
            TwoRegimeResidualEstimator(
                dt=self.dt_nominal,
                stable_tau_sec=regime_stable_tau_sec,
                stable_process_std=regime_stable_process_std,
                persistent_process_std=regime_persistent_process_std,
                measurement_std=(
                    sigma0 if regime_measurement_std is None
                    else regime_measurement_std),
                stable_stay_probability=regime_stable_stay_probability,
                persistent_stay_probability=(
                    regime_persistent_stay_probability),
                initial_persistent_probability=(
                    regime_initial_persistent_probability),
            )
            if self.regime_enabled else None)
        self.conformal = ConformalCalibrator(
            n_cal=n_cal, alpha=alpha, gamma_aci=gamma_aci)
        # 保形校准按前瞻步数 h Mondrian 分组(h=1..cal_horizon_steps):
        # 每拍推入 θ̂ 冻结的前瞻快照 Δ̂(t_meas + hΔt), h 拍后到期时用当拍
        # 测量打分喂入组 h —— 校准分数与 predict_horizon 的使用语义(θ̂
        # 冻结外推)逐组一致, 这是联合界成立的前提(见 ConformalCalibrator).
        if int(cal_horizon_steps) < 1:
            raise ValueError(
                f'cal_horizon_steps must be >= 1, got {cal_horizon_steps}')
        self.cal_horizon_steps = int(cal_horizon_steps)
        # 校准层级取对数(倍增)网格 {1,2,4,...,H} 而非全部 h=1..H: 请求的
        # h 经 quantile_joint 向上取最近已校准层级(多步分数随 h 随机增大,
        # 上取保守), 把每拍快照打分/组更新量从 O(H) 压到 O(log H) ——
        # 50Hz 落笔拍上快照通道 <30µs, 与分摊辨识共享预算后单拍最坏增量
        # 与修复前持平。层级内的保守代价由相邻层级 ≤2 倍间距控制。
        levels = [1]
        while levels[-1] < self.cal_horizon_steps:
            levels.append(min(2 * levels[-1], self.cal_horizon_steps))
        self._cal_levels = levels
        # 快照环形存储(预分配, 50Hz tick 内零动态结构): 槽 i 存某拍的
        # (t0, pen, Δ̂(t0+LΔt), L 取 ≥2 的校准层级); 打分用向量化 gather。
        # 容量 = H: 层级 L=H 的快照要在 H 拍后到期打分。
        snap_levels = [lv for lv in levels if lv >= 2]
        snap_cap = max(1, self.cal_horizon_steps)
        self._snap_cap = snap_cap
        self._snap_t0 = np.full(snap_cap, np.nan)
        self._snap_pen = np.zeros(snap_cap, dtype=bool)
        self._snap_pred = np.zeros(
            (snap_cap, max(1, len(snap_levels)), self.N_AXES))
        self._snap_head = 0
        self._snap_offsets = self.dt_nominal * np.asarray(
            snap_levels, dtype=float)
        # 快照年龄(拍) -> 层级列号; 非层级年龄 = -1(该拍不打分)
        self._age_to_level_col = np.full(self.cal_horizon_steps + 1, -1)
        for col, lv in enumerate(snap_levels):
            self._age_to_level_col[lv] = col
        if int(ident_units_per_tick) < 1 or int(ident_units_pen_up) < 1 \
                or ident_budget_sec <= 0.0 or ident_budget_pen_up_sec <= 0.0:
            raise ValueError('ident amortization params must be positive')
        # 分摊预算标定(50Hz/K=12/30s 缓冲, 全流水线 ~350 单元 ×≤0.6ms):
        # 关键约束是**流水线周期 < ident_period**(2s), 否则换库时点系统性
        # 滞后, 收敛段 10 步预测 RMS 实测 +32%(5 seed 均值); 每拍 5 单元
        # (周期 ~1.4s)实测与整块执行打平(-4%), 墙钟 1.8ms 兜底后最坏单拍
        # 增量 <3ms. 预算持续吃紧时周期变长, 只软性退化收敛速度, 不违实时.
        self.ident_units_per_tick = int(ident_units_per_tick)
        self.ident_budget_sec = float(ident_budget_sec)
        self.ident_units_pen_up = int(ident_units_pen_up)
        self.ident_budget_pen_up_sec = float(ident_budget_pen_up_sec)
        # 谱线搜索降采样(仅找峰/细化用, 加权重拟合/holdout/温启动仍全速率),
        # 可把流水线总工作量 ~q 倍缩减; 默认 1(关闭): A/B 实测 q=3 会损失
        # DTFT 细化的相干增益, 稳态 10 步预测 RMS 劣化 ~35%, 而分摊节奏
        # (每拍 5 单元)下全速率流水线 ~1.4s 完成, 已短于 2s 触发周期,
        # 无需再压缩。保留参数供更高 tick 率/更长缓冲场景调优。
        if int(ident_decim) < 1:
            raise ValueError(f'ident_decim must be >= 1, got {ident_decim}')
        self.ident_decim = int(ident_decim)
        # 环形缓冲用预分配 numpy 数组(快照 = 两段 memcpy, 微秒级; deque 快照
        # 需逐元素迭代, 在 50Hz tick 内不可接受)。
        maxlen = max(64, int(round(buffer_sec / self.dt_nominal)))
        self._buf_cap = maxlen
        self._buf_t = np.zeros(maxlen)
        self._buf_y = np.zeros((maxlen, self.N_AXES))
        self._buf_len = 0
        self._buf_head = 0               # 下一写入位置
        self._ident_task = None          # 在途分摊辨识任务(生成器)
        self.last_ident_chunk_ms = 0.0
        self._t_last_ident = None
        self._t_meas_last = None
        self.n_ident = 0
        self.n_ident_accepted = 0

    def __getstate__(self):
        """deepcopy/pickle 支持: 在途辨识任务是生成器, 不可拷贝, 直接丢弃
        (拷贝件要么是冻结快照不再喂数, 要么下个触发周期自然重启辨识)。"""
        state = self.__dict__.copy()
        state['_ident_task'] = None
        return state

    # ---------- 环形缓冲 ----------

    def _buffer_append(self, t_meas: float, e: np.ndarray):
        self._buf_t[self._buf_head] = t_meas
        self._buf_y[self._buf_head] = e
        self._buf_head = (self._buf_head + 1) % self._buf_cap
        self._buf_len = min(self._buf_len + 1, self._buf_cap)

    def _buffer_oldest_t(self) -> float:
        if self._buf_len < self._buf_cap:
            return float(self._buf_t[0])
        return float(self._buf_t[self._buf_head])

    def _buffer_arrays(self):
        """按时间升序拷贝出 (t (n,), y (n,3)); 两段 memcpy, 微秒级."""
        if self._buf_len < self._buf_cap:
            return (self._buf_t[:self._buf_len].copy(),
                    self._buf_y[:self._buf_len].copy())
        h = self._buf_head
        return (np.concatenate((self._buf_t[h:], self._buf_t[:h])),
                np.concatenate((self._buf_y[h:], self._buf_y[:h])))

    # ---------- 谱辨识 + 温启动 ----------

    def _wls_fit_steps(self, bank: HarmonicBank, t: np.ndarray,
                       y: np.ndarray, out: dict):
        """指数遗忘加权岭回归的分步生成器: 权 λ^age(拍), 先验 N(0,
        prior_std² I). 结果 out['theta']/out['Sigma'] 与 AxisRLS 同一
        后验语义, 可直接温启动. 三个原子单元: 特征/正规方程/求解+逆.
        """
        yield                            # 单元: 特征矩阵 + 遗忘权
        Phi = bank.features(t)
        age = (float(t[-1]) - t) / self.dt_nominal
        w = np.power(self._lam, np.minimum(age, 6000.0))
        yield                            # 单元: 正规方程构建
        A = Phi.T @ (w[:, None] * Phi) \
            + (self._sigma0 ** 2 / self._prior_std ** 2) * np.eye(bank.dim)
        b = Phi.T @ (w * y)
        yield                            # 单元: 求解 + 协方差
        out['theta'] = np.linalg.solve(A, b)
        out['Sigma'] = self._sigma0 ** 2 * np.linalg.inv(A)

    def _wls_fit(self, bank: HarmonicBank, t: np.ndarray, y: np.ndarray):
        """整块执行 _wls_fit_steps(离线脚本/自测用), 返回 (θ, Σ)."""
        out = {}
        for _ in self._wls_fit_steps(bank, t, y, out):
            pass
        return out['theta'], out['Sigma']

    def _identify_steps(self, t_arr: np.ndarray, y_arr: np.ndarray):
        """分摊式重辨识流水线(生成器), 每轴: 找谱线 -> 加权重拟合 ->
        holdout 纯外推对比 -> 择优换库. 与整块执行同一数学; 差别仅在:
        (1) 完成时刻晚于触发快照数十拍(候选/holdout 都取自触发时快照);
        (2) 接受换库的轴在流水线末尾用**当前**环形缓冲重拟合温启动,
            覆盖分摊期间新到的测量, 与在线 RLS 流无缝衔接."""
        n_hold = int(round(self.holdout_sec / self.dt_nominal))
        n_hold = max(16, min(n_hold, t_arr.size // 4))
        t_fit, y_fit = t_arr[:-n_hold], y_arr[:-n_hold]
        t_hold, y_hold = t_arr[-n_hold:], y_arr[-n_hold:]
        # 谱线搜索用降采样视图(见 __init__ ident_decim 注释); 拟合全速率。
        q = self.ident_decim
        t_dec, y_dec = (t_fit, y_fit) if q == 1 else (t_fit[::q], y_fit[::q])
        accepted = []
        for a in range(self.N_AXES):
            lines = {}
            yield from _spectral_lines_steps(
                t_dec, y_dec[:, a], lines, n_peaks=self.n_peaks,
                fmin_hz=self.f_ident_min_hz, fmax_hz=self.f_ident_max_hz,
                peak_snr=self.peak_snr)
            freqs = lines['freqs']
            if freqs.size == 0:
                continue
            cand_bank = HarmonicBank(freqs)
            fit = {}
            yield from self._wls_fit_steps(
                cand_bank, t_fit, y_fit[:, a], fit)
            yield                        # 单元: holdout 纯外推对比
            e_cand = cand_bank.features(t_hold) @ fit['theta'] - y_hold[:, a]
            e_cur = self.banks[a].features(t_hold) @ self.rls[a].mu \
                - y_hold[:, a]
            if float(e_cand @ e_cand) < float(e_cur @ e_cur):
                accepted.append((a, cand_bank))
        for a, cand_bank in accepted:
            yield                        # 单元: 当前缓冲快照
            t_now, y_now = self._buffer_arrays()
            fit = {}
            yield from self._wls_fit_steps(cand_bank, t_now, y_now[:, a], fit)
            # 换库温启动(轻量, 与上一单元同拍完成)
            self.banks[a] = cand_bank
            self.rls[a].warm_start(fit['theta'], fit['Sigma'])
            self.n_ident_accepted += 1

    def _identify(self, t_arr: np.ndarray, y_arr: np.ndarray):
        """整块(阻塞)执行辨识流水线. 仅供离线脚本/自测; 在线路径一律分摊."""
        self.n_ident += 1
        for _ in self._identify_steps(t_arr, y_arr):
            pass

    def _advance_ident(self, sched_pen_down: bool = True,
                       spent_sec: float = 0.0) -> float:
        """推进在途辨识任务: 落笔拍 ≤ident_units_per_tick 个原子单元且
        墙钟 ≥ident_budget_sec 即停; 抬笔拍用放宽的 pen_up 预算.
        spent_sec 为本拍其它分摊工作(per-h 快照校准)已消耗的预算 ——
        共享同一分摊预算使单拍最坏增量不随新增校准工作变大, 代价只是
        流水线完成时刻软性推迟(每拍仍至少推进 1 个单元, 不会饿死).
        返回本拍实际耗时 ms."""
        task = self._ident_task
        if task is None:
            return 0.0
        if sched_pen_down:
            max_units, budget = self.ident_units_per_tick, \
                self.ident_budget_sec
        else:
            max_units, budget = self.ident_units_pen_up, \
                self.ident_budget_pen_up_sec
        budget -= float(spent_sec)
        t0 = time.perf_counter()
        for _ in range(max_units):
            try:
                next(task)
            except StopIteration:
                self._ident_task = None
                break
            if time.perf_counter() - t0 >= budget:
                break
        chunk_ms = 1e3 * (time.perf_counter() - t0)
        self.last_ident_chunk_ms = chunk_ms
        return chunk_ms

    def _maybe_identify(self, t_meas: float,
                        sched_pen_down: bool = True,
                        spent_sec: float = 0.0) -> float:
        """任务在途则推进一个分摊块; 否则按周期触发(快照 + 建任务, 本拍
        不再推进 —— 快照拷贝已计为本拍工作量). 返回本拍辨识耗时 ms."""
        if self._ident_task is not None:
            return self._advance_ident(sched_pen_down, spent_sec=spent_sec)
        if self._buf_len < 64:
            return 0.0
        span = t_meas - self._buffer_oldest_t()
        if span < self.min_ident_sec:
            return 0.0
        if self._t_last_ident is not None \
                and t_meas - self._t_last_ident < self.ident_period_sec:
            return 0.0
        t0 = time.perf_counter()
        self._t_last_ident = t_meas
        t_arr, y_arr = self._buffer_arrays()
        self.n_ident += 1
        self._ident_task = self._identify_steps(t_arr, y_arr)
        chunk_ms = 1e3 * (time.perf_counter() - t0)
        self.last_ident_chunk_ms = chunk_ms
        return chunk_ms

    # ---------- 在线接口 ----------

    def _predict_mean(self, times: np.ndarray) -> np.ndarray:
        """参数通道的谐波均值，不含快速残差。"""
        out = np.empty((times.size, self.N_AXES))
        for a in range(self.N_AXES):
            out[:, a] = self.banks[a].features(times) @ self.rls[a].mu
        return out

    def _predict_harmonic_covariance(self, times: np.ndarray) -> np.ndarray:
        covariance = np.zeros((times.size, self.N_AXES, self.N_AXES))
        for axis in range(self.N_AXES):
            features = self.banks[axis].features(times)
            variance = np.einsum(
                'ni,ij,nj->n',
                features, self.rls[axis].Sigma, features)
            covariance[:, axis, axis] = np.maximum(
                variance + self._sigma0 ** 2, 0.0)
        return covariance

    def _predict_total_components(self, times: np.ndarray):
        harmonic = self._predict_mean(times)
        harmonic_covariance = self._predict_harmonic_covariance(times)
        if self.regime is None:
            fast = np.repeat(
                self._resid_lpf.reshape(1, self.N_AXES),
                times.size, axis=0)
            fast_covariance = np.zeros_like(harmonic_covariance)
            probabilities = np.repeat(
                np.array([[1.0, 0.0]]), times.size, axis=0)
        else:
            if self._t_meas_last is None:
                steps = np.zeros(times.size, dtype=int)
            else:
                steps = np.maximum(
                    np.rint(
                        (times - self._t_meas_last)
                        / self.dt_nominal).astype(int),
                    0)
            fast, fast_covariance, probabilities = (
                self.regime.predict_horizon(steps))
        return (
            harmonic, fast,
            harmonic_covariance + fast_covariance,
            probabilities)

    def _score_horizon_snapshots(self, t_meas: float, e: np.ndarray):
        """到期快照打分: 对层级组 L∈{2,4,...,H} 喂入 θ̂ 冻结的 L 步先验分数.

        名义节拍下每拍每层级恰好到期一个分数(快照年龄=L 拍时打分);
        时间抖动超过 1/4 拍的错位样本直接丢弃(错组污染覆盖统计比丢样本
        更伤). 分数 gather 向量化, 逐组 ACI/入窗为 O(log H) 次 O(n_cal)
        标量更新, 实测每拍 <30µs.
        """
        if self.cal_horizon_steps < 2:
            return
        h_f = (t_meas - self._snap_t0) / self.dt_nominal
        mask = (h_f >= 1.75) & (h_f <= self.cal_horizon_steps + 0.25)
        idx = np.nonzero(mask)[0]
        if idx.size == 0:
            return
        h = np.rint(h_f[idx]).astype(int)
        good = np.abs(h_f[idx] - h) <= 0.25
        idx, h = idx[good], h[good]
        col = self._age_to_level_col[h]
        on_level = col >= 0
        idx, h, col = idx[on_level], h[on_level], col[on_level]
        if idx.size == 0:
            return
        preds = self._snap_pred[idx, col]                # (k, 3)
        scores = np.max(np.abs(preds - e[None, :]), axis=1)
        update = self.conformal.update
        for s, p, hh in zip(scores.tolist(),
                            self._snap_pen[idx].tolist(), h.tolist()):
            update(s, p, h=hh)

    def _push_horizon_snapshot(self, t_meas: float, pen_down: bool):
        """推入本拍 θ̂ 冻结的前瞻快照(层级 L>=2; L=1 组由一步残差通道喂),
        含与 predict_horizon 相同的残差慢通道保持项."""
        if self.cal_horizon_steps < 2:
            return
        head = self._snap_head
        self._snap_t0[head] = t_meas
        self._snap_pen[head] = bool(pen_down)
        snapshot_times = t_meas + self._snap_offsets
        harmonic, fast, _, _ = self._predict_total_components(snapshot_times)
        self._snap_pred[head] = harmonic + fast
        self._snap_head = (head + 1) % self._snap_cap

    def step(self, t: float, e_meas, pen_down: bool = True,
             sched_pen_down: bool = None) -> dict:
        """每拍更新. e_meas 为三轴 Δp 测量(m). 返回诊断字典.

        pen_down 决定保形 Mondrian 分组(语义不变); sched_pen_down 仅用于
        辨识分摊调度(抬笔拍放宽预算), 缺省继承 pen_down —— 两者分离是为了
        既有调用方(恒 pen_down=True)可以只接入调度而不改分组统计.
        常规路径(RLS/β/保形/慢通道)单拍 O((2K+1)²); 分摊辨识块的实际耗时
        经 'ident_ms' 返回, 供上层打印/落盘.
        """
        t_meas = float(t) - self.meas_delay_sec  # 测量时间戳(名义已知延迟)
        e = np.asarray(e_meas, dtype=float).reshape(self.N_AXES)
        harmonic_prior = self._predict_mean(np.array([t_meas]))[0]
        residual_harmonic = e - harmonic_prior
        # β 只评价参数通道的适用程度，不再解释为整体预测可靠性。
        trust = self.beta_filter.update(residual_harmonic)
        if self.regime is None:
            regime_diag = None
            residual_final_prior = (
                residual_harmonic - self._resid_lpf)
        else:
            regime_diag = self.regime.step(residual_harmonic)
            residual_final_prior = (
                residual_harmonic - regime_diag['prior_mean'])
        # 保形分数作用于最终融合预测的先验误差，覆盖快速模型的估计误差。
        score = float(np.max(np.abs(residual_final_prior)))
        covered, delta_1 = self.conformal.update(score, pen_down, h=1)
        # h>=2 组的多步快照打分/推入与分摊辨识共享同一预算(见 _advance_
        # ident): 单拍最坏增量不因新增校准工作变大, 只软性推迟流水线完成.
        cal_t0 = time.perf_counter()
        self._score_horizon_snapshots(t_meas, e)
        cal_spent = time.perf_counter() - cal_t0
        if self.regime is None:
            self._resid_lpf += self._resid_alpha * (
                residual_harmonic - self._resid_lpf)
        for a in range(self.N_AXES):
            self.rls[a].update(self.banks[a].feature(t_meas), e[a])
        self._buffer_append(t_meas, e)
        self._t_meas_last = t_meas
        cal_t0 = time.perf_counter()
        self._push_horizon_snapshot(t_meas, pen_down)
        cal_spent += time.perf_counter() - cal_t0
        ident_ms = self._maybe_identify(
            t_meas,
            pen_down if sched_pen_down is None else bool(sched_pen_down),
            spent_sec=cal_spent)
        return {
            'trust': trust,
            'parameter_applicability': trust,
            'beta_mean': self.beta_filter.beta_mean,
            'residual': residual_final_prior,
            'residual_harmonic': residual_harmonic,
            'score': score,
            'covered': covered,
            'delta_1step': delta_1,
            'alpha': self.conformal.alpha(pen_down),
            'ident_ms': ident_ms,
            'ident_active': self._ident_task is not None,
            'regime_enabled': self.regime is not None,
            'persistent_probability': (
                float(regime_diag['persistent_probability'])
                if regime_diag is not None else 0.0),
            'fast_residual_mean': (
                regime_diag['posterior_mean'].copy()
                if regime_diag is not None else self._resid_lpf.copy()),
            'fast_residual_covariance': (
                regime_diag['posterior_covariance'].copy()
                if regime_diag is not None
                else np.zeros((self.N_AXES, self.N_AXES))),
        }

    def predict_horizon(self, times, pen_down: bool = True):
        """times (n,) 绝对时间 -> (Δ̂ (n,3), δ 标量, 联合界 1-α).

        Δ̂ = 谐波外推 + 残差慢通道保持项(未分辨低频成分的常值保持).
        δ = max_h δ_h(α_h/n): 各时刻按距最近测量的前瞻步数 h 归组取该组
        校准窗的 1-α_h/n 分位数(Bonferroni 多重性), 再取 max —— 多步
        分数随 h 系统性增大, 单一 1 步窗外推到多步不是有效联合界.
        过去/当前时刻(插值域)归入 h=1; 超出 cal_horizon_steps 的前瞻
        钳到最大组(该组分数最保守).
        """
        stats = self.predict_horizon_stats(times, pen_down)
        return stats['mean'], stats['delta']

    def predict_horizon_stats(self, times, pen_down: bool = True):
        """返回控制决策所需的分通道均值、混合协方差、工况概率和保形半径。"""
        times = np.atleast_1d(np.asarray(times, dtype=float))
        harmonic, fast, covariance, probabilities = (
            self._predict_total_components(times))
        if self._t_meas_last is None:
            delta = self.conformal.quantile(pen_down, n_joint=times.size)
        else:
            hs = np.clip(
                np.rint((times - self._t_meas_last)
                        / self.dt_nominal).astype(int),
                1, self.cal_horizon_steps)
            delta = self.conformal.quantile_joint(
                pen_down, hs, n_joint=times.size)
        return {
            'mean': harmonic + fast,
            'harmonic_mean': harmonic,
            'fast_mean': fast,
            'covariance': covariance,
            'mode_probabilities': probabilities,
            'delta': float(delta),
        }
