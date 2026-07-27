#!/usr/bin/env python3
"""BaseTrustEstimator 谱辨识分摊(实时性)回归测试, python3 直跑.

背景: 周期性重辨识(周期图+谱线库重建+RLS 重拟合)整块执行在 30s 缓冲
×3 轴 ×K=12 时单块 150~250ms, 曾直接跑在 50Hz 控制 tick 内, 造成 ROS
闭环 timer P99 30~35ms(门限 30ms)与最坏 tick ~200ms. 修复为分摊状态机:
_identify_steps 生成器切成 ≤0.6ms 原子单元, 每拍 ≤5 单元且墙钟 1.8ms
兜底(落笔拍; 抬笔拍 8 单元/2.6ms), 最坏单拍增量 <3ms.

本测试断言(与修复要求逐条对应):
  T1 实时性: 90s@50Hz 模拟喂入(demo model 档同配置), 每拍
     step()+predict_horizon() 墙钟 P99 < 3ms(GC 禁用, 复刻 demo 控制期
     实时设计; BLAS 单线程+绑核, 隔离桌面机调度噪声, 测的是算法预算);
  T2 精度不劣化: 前瞻式(prequential) 10 步预测 RMS(t>=25s, 参考=真值)
     **3 个噪声种子均值**相对整块(阻塞)执行参考 ±10% 内 —— 阻塞参考即
     修复前逐位语义(同一数学, 触发拍内一次跑完). 用均值是因为换库 holdout
     对决在 σ=2mm 噪声地板附近是近似掷硬币, 单种子路径方差可达 ±30%,
     均值才反映系统性差异(标定实验: 5 seed 均值 5 单元/拍 -4%, 3 单元/拍
     +32% —— 后者流水线周期 2.3s 超过触发周期 2s, 换库系统性滞后).
"""

import os

os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import gc          # noqa: E402
import json        # noqa: E402
import math        # noqa: E402
import sys         # noqa: E402
import time        # noqa: E402

import numpy as np  # noqa: E402

from base_trust_estimator import BaseTrustEstimator  # noqa: E402
from moving_base_trajectory_config import base_disturbance_offset  # noqa: E402

DT = 0.02                 # 50 Hz
T_END = 90.0              # 60s 只够收敛段, 90s 使稳态主导(收敛路径噪声稀释)
AMP = 0.01                # 10 mm
SEED = 0
NOISE_STD = 2.0e-3        # 2 mm
DELAY_SEC = 0.04          # 40 ms
EVAL_START = 25.0         # 训练/评估分离(与 test_base_trust_estimator 一致)
P99_BUDGET_MS = 3.0
RMS_REL_TOL = 0.10

# demo model 档在用配置(moving_base_mpc_params_* 快照口径)
EST_KW = dict(n_freq=6, fmin_hz=0.15, fmax_hz=2.5, lam=0.995,
              sigma0=2.0e-3, prior_std=1.0e-2, meas_delay_sec=DELAY_SEC,
              dt_nominal=DT, n_peaks=12, ident_period_sec=2.0,
              buffer_sec=30.0, min_ident_sec=2.5, holdout_sec=1.0,
              n_cal=100, alpha=0.1, gamma_aci=0.005)
# demo 预测网格等效: dt*[-2..25] 共 28 点
PRED_GRID = DT * np.arange(-2, 26)


def truth_at(t: float) -> np.ndarray:
    return base_disturbance_offset(t, AMP, seed=SEED)


NOISE_SEEDS = (12345, 777, 2026)


def run(amortized: bool, noise_seed: int):
    """60s@50Hz 喂入; 返回 (每拍耗时数组 ms, 10 步预测 RMS mm, 估计器)."""
    kw = dict(EST_KW)
    if not amortized:
        # 阻塞参考 = 修复前逐位语义: 触发拍内整块跑完 + 全速率(不降采样)
        # 谱线搜索(同一数学流水线, 与修复前 _identify 数值一致)
        kw.update(ident_units_per_tick=10 ** 9, ident_budget_sec=1e9,
                  ident_units_pen_up=10 ** 9, ident_budget_pen_up_sec=1e9,
                  ident_decim=1)
    est = BaseTrustEstimator(**kw)
    rng = np.random.default_rng(noise_seed)
    n = int(round(T_END / DT))
    tick_ms = np.empty(n)
    rms10_sq, m10 = 0.0, 0
    # 模拟 whole 的落笔节律: 12s 落笔 + 3.6s 抬笔过渡(调度提示用)
    for k in range(n):
        t = k * DT
        pen = (t % 15.6) < 12.0
        meas = truth_at(t - DELAY_SEC) + rng.normal(0.0, NOISE_STD, 3)
        t0 = time.perf_counter()
        est.step(t, meas, sched_pen_down=pen)
        est.predict_horizon(t + PRED_GRID)
        tick_ms[k] = 1e3 * (time.perf_counter() - t0)
        if t >= EVAL_START and (k + 10) < n:
            t10 = t + 10 * DT
            pred, _ = est.predict_horizon(np.array([t10]))
            e10 = pred[0] - truth_at(t10)
            rms10_sq += float(e10 @ e10)
            m10 += 1
    rms10_mm = 1e3 * math.sqrt(rms10_sq / (3 * m10))
    return tick_ms, rms10_mm, est


def main():
    # 绑核 + 禁 GC: 复刻 demo 控制期实时设计, 隔离桌面机调度噪声
    try:
        cpus = sorted(os.sched_getaffinity(0))
        os.sched_setaffinity(0, {cpus[len(cpus) // 2]})
    except (AttributeError, OSError):
        pass
    gc.disable()
    gc.freeze()
    tick_all = []
    rms_a, rms_b = [], []
    n_ident_a = n_ident_b = 0
    worst_block_ms = 0.0
    for sd in NOISE_SEEDS:
        tick_a, r_a, est_a = run(amortized=True, noise_seed=sd)
        gc.collect()
        tick_b, r_b, est_b = run(amortized=False, noise_seed=sd)
        gc.collect()
        tick_all.append(tick_a)
        rms_a.append(r_a)
        rms_b.append(r_b)
        n_ident_a += est_a.n_ident
        n_ident_b += est_b.n_ident
        worst_block_ms = max(worst_block_ms, float(tick_b.max()))
        print(f'[T2] seed {sd}: 分摊 {r_a:.4f} mm vs 阻塞 {r_b:.4f} mm '
              f'({100.0 * (r_a - r_b) / r_b:+.1f}%)')
    gc.enable()
    tick_a = np.concatenate(tick_all)
    mean_a = float(np.mean(rms_a))
    mean_b = float(np.mean(rms_b))

    p50, p95, p99 = np.percentile(tick_a, [50, 95, 99])
    print(f'[T1] 分摊模式每拍 step+predict (ms, {len(NOISE_SEEDS)} 次 '
          f'{T_END:.0f}s): '
          f'P50={p50:.3f} P95={p95:.3f} P99={p99:.3f} max={tick_a.max():.3f}')
    print(f'[T1] 阻塞参考最坏拍 = {worst_block_ms:.1f} ms '
          f'(修复前即此值直接进 50Hz tick)')
    print(f'[T2] 10 步预测 RMS 均值: 分摊 {mean_a:.4f} mm vs '
          f'阻塞参考 {mean_b:.4f} mm '
          f'(相对差 {100.0 * (mean_a - mean_b) / mean_b:+.2f}%)')

    fails = []
    if not (p99 < P99_BUDGET_MS):
        fails.append(f'per-tick P99 {p99:.3f} ms >= {P99_BUDGET_MS} ms')
    if not (abs(mean_a - mean_b) <= RMS_REL_TOL * mean_b):
        fails.append(f'10-step RMS mean drift {mean_a:.4f} vs {mean_b:.4f} '
                     f'mm exceeds ±{100 * RMS_REL_TOL:.0f}%')
    # 卫生: 分摊模式没有饿死(辨识轮数与阻塞同量级)
    if n_ident_a < max(3, n_ident_b // 3):
        fails.append(f'amortized ident starved: {n_ident_a} rounds vs '
                     f'blocking {n_ident_b}')

    result = {
        'test_pass': not fails,
        'offline_p99_ms': round(float(p99), 4),
        'offline_max_ms': round(float(tick_a.max()), 4),
        'rms10_amortized_mm': round(mean_a, 4),
        'rms10_blocking_mm': round(mean_b, 4),
        'rms10_rel_diff': round((mean_a - mean_b) / mean_b, 4),
        'blocking_worst_tick_ms': round(worst_block_ms, 2),
        'notes': '; '.join(fails) if fails else 'ok',
    }
    print(json.dumps(result))
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
