#!/usr/bin/env python3
"""base_trust_estimator 自测: 精度/防泄漏外推/覆盖率/故障响应, python3 直跑.

场景: moving_base_trajectory_config.base_disturbance_offset(amp=0.01, seed=0)
生成 60s@50Hz 真值 Δp, 加 σ=2mm 高斯噪声与 40ms 延迟作为测量.

防泄漏打分协议:
  1. 一切打分只对真未来 Δ(t_future) —— 直接调 base_disturbance_offset,
     绝不用测量流当参考;
  2. 训练/评估时段分离: t < TRAIN_END(25s) 只训练不打分,
     打分在 [25s, 58s) 前瞻式(prequential)进行 —— 每拍先 step() 喂入
     t-40ms 时刻的测量, 再对 t+200ms 求值预测(信息差 240ms, 与闭环一致);
  3. 冻结外推终验: t=40s 处深拷贝估计器并停止喂数, 对 (40s, 55s] 纯外推
     打分 —— 任何依赖测量流插值/记忆的伪精度在此必然崩溃.
"""

import copy
import json
import math
import sys

import numpy as np

from base_trust_estimator import BaseTrustEstimator
from moving_base_trajectory_config import base_disturbance_offset

DT = 0.02                # 50 Hz
T_END = 60.0
AMP = 0.01               # 10 mm
SEED = 0
NOISE_STD = 2.0e-3       # 2 mm
DELAY_SEC = 0.04         # 40 ms = 2 拍
TRAIN_END = 25.0         # 纯训练时段终点(评估时段与之分离)
FREEZE_T = 40.0          # 冻结外推起点
FREEZE_END = 55.0
FAULT_T = 30.0
FAULT_BIAS = np.array([0.01, 0.0, 0.0])  # 10 mm 直流偏置(x 轴)

# 估计器配置(全部为名义已知量; 谱辨识/遗忘用类默认值)
EST_KW = dict(meas_delay_sec=DELAY_SEC, dt_nominal=DT,
              sigma0=2.0e-3, beta_sigma0=3.0e-3, beta_gamma_f=0.98,
              n_cal=250, alpha=0.1, gamma_aci=0.02)


def truth_at(t: float) -> np.ndarray:
    return base_disturbance_offset(t, AMP, seed=SEED)


def run(fault: bool):
    n = int(round(T_END / DT))
    rng = np.random.default_rng(12345)
    est = BaseTrustEstimator(**EST_KW)

    rms1_sq, rms10_sq, m1 = 0.0, 0.0, 0
    zoh10_sq, hold10_sq = 0.0, 0.0
    cov_hits, cov_tot = 0, 0
    log_t, log_b, log_delta, log_resid = [], [], [], []
    est_frozen = None

    for k in range(n):
        t = k * DT
        tm = t - DELAY_SEC                       # 测量对应的真值时刻
        true_m = truth_at(tm).copy()
        if fault and tm >= FAULT_T:
            true_m += FAULT_BIAS
        meas = true_m + rng.normal(0.0, NOISE_STD, 3)

        out = est.step(t, meas, pen_down=True)
        log_t.append(t)
        log_b.append(out['trust'])
        log_delta.append(out['delta_1step'])
        log_resid.append(out['residual'])

        if not fault and est_frozen is None and t >= FREEZE_T:
            est_frozen = copy.deepcopy(est)      # 冻结快照(其后不再喂数)

        if t >= TRAIN_END:
            cov_tot += 1
            cov_hits += int(out['covered'])
            if not fault and (k + 10) < n:
                t1, t10 = t + DT, t + 10 * DT
                pred, _ = est.predict_horizon(np.array([t1, t10]))
                e1 = pred[0] - truth_at(t1)      # 参考=真未来, 非测量流
                e10 = pred[1] - truth_at(t10)
                rms1_sq += float(e1 @ e1)
                rms10_sq += float(e10 @ e10)
                z10 = meas - truth_at(t10)       # 常值保持(最新测量)基线
                h10 = truth_at(tm) - truth_at(t10)  # 无噪常值保持基线
                zoh10_sq += float(z10 @ z10)
                hold10_sq += float(h10 @ h10)
                m1 += 1

    res = {
        'coverage': cov_hits / max(cov_tot, 1),
        't': np.array(log_t), 'b': np.array(log_b),
        'delta': np.array(log_delta), 'resid': np.array(log_resid),
    }
    if not fault:
        res['rms1_mm'] = 1e3 * math.sqrt(rms1_sq / (3 * m1))
        res['rms10_mm'] = 1e3 * math.sqrt(rms10_sq / (3 * m1))
        res['zoh10_mm'] = 1e3 * math.sqrt(zoh10_sq / (3 * m1))
        res['hold10_mm'] = 1e3 * math.sqrt(hold10_sq / (3 * m1))
        # 冻结外推: (40s, 55s] 纯外推, 估计器不再接收任何测量
        tf = FREEZE_T + DT * np.arange(1, int((FREEZE_END - FREEZE_T) / DT) + 1)
        pred_f, _ = est_frozen.predict_horizon(tf)
        err_f = pred_f - np.stack([truth_at(ti) for ti in tf])
        res['frozen_rms_mm'] = 1e3 * math.sqrt(float(np.mean(err_f ** 2)))
    return res


def main():
    notes = []
    fails = []

    # ---------- Run A: 无故障, 精度 + 防泄漏外推 + 覆盖率 ----------
    a = run(fault=False)
    print(f"[A] 1-step  pred RMS = {a['rms1_mm']:.3f} mm")
    print(f"[A] 10-step pred RMS = {a['rms10_mm']:.3f} mm   "
          f"(常值保持: 最新测量 {a['zoh10_mm']:.3f} mm, "
          f"无噪保持 {a['hold10_mm']:.3f} mm)")
    print(f"[A] frozen extrapolation RMS (40->55s, 纯外推 15s) = "
          f"{a['frozen_rms_mm']:.3f} mm")
    print(f"[A] conformal coverage (t>25s) = {a['coverage']*100:.1f} % "
          f"(target 90 %)")
    if not a['rms1_mm'] < 1.2:
        fails.append(f"1-step RMS {a['rms1_mm']:.3f} mm >= 1.2 mm")
    if not a['rms10_mm'] < 1.2:                      # 硬门
        fails.append(f"10-step RMS {a['rms10_mm']:.3f} mm >= 1.2 mm")
    if not a['rms10_mm'] < 0.5 * a['zoh10_mm']:
        fails.append(f"10-step RMS {a['rms10_mm']:.3f} mm not < 50% of "
                     f"const-hold {a['zoh10_mm']:.3f} mm")
    if not a['frozen_rms_mm'] < 2.5:
        fails.append(f"frozen extrapolation RMS {a['frozen_rms_mm']:.3f} mm "
                     f">= 2.5 mm (泄漏嫌疑: 模型未学到全局时间结构)")
    if not abs(a['coverage'] - 0.90) < 0.05:
        fails.append(f"coverage {a['coverage']:.3f} outside 0.90 +- 0.05")
    notes.append(f"const-hold 10-step = {a['zoh10_mm']:.2f} mm (meas) / "
                 f"{a['hold10_mm']:.2f} mm (clean); "
                 f"frozen 15s extrap = {a['frozen_rms_mm']:.2f} mm")

    # ---------- Run B: t=30s 注入 10mm 直流偏置 ----------
    b = run(fault=True)
    t = b['t']
    pre = (t >= FAULT_T - 1.0) & (t < FAULT_T)
    b_pre = float(np.mean(b['b'][pre]))
    d_pre = float(np.mean(b['delta'][pre]))

    in2s = (t > FAULT_T) & (t <= FAULT_T + 2.0)
    b_min_2s = float(np.min(b['b'][in2s]))
    t_half = None
    idx = np.where((t > FAULT_T) & (b['b'] < 0.5 * b_pre))[0]
    if idx.size:
        t_half = float(t[idx[0]] - FAULT_T)
    print(f"[B] trust b: pre-fault {b_pre:.3f}, min within 2s "
          f"{b_min_2s:.3f}, halved after {t_half} s")
    if not (b_min_2s < 0.5 * b_pre):
        fails.append(f"trust b did not halve within 2s "
                     f"({b_min_2s:.3f} vs pre {b_pre:.3f})")

    in5s = (t > FAULT_T) & (t <= FAULT_T + 5.0)
    d_max_5s = float(np.max(b['delta'][in5s]))
    print(f"[B] conformal delta: pre-fault {1e3*d_pre:.2f} mm, "
          f"max within 5s {1e3*d_max_5s:.2f} mm")
    if not (d_max_5s > 1.5 * d_pre):
        fails.append(f"delta did not expand within 5s "
                     f"({1e3*d_max_5s:.2f} vs pre {1e3*d_pre:.2f} mm)")

    late = (t >= FAULT_T + 19.0) & (t < FAULT_T + 20.0)
    resid_late = b['resid'][late]
    resid_rms_late = 1e3 * math.sqrt(float(np.mean(resid_late ** 2)))
    bias_left = 1e3 * abs(float(np.mean(resid_late[:, 0])))
    d_late = float(np.mean(b['delta'][late]))
    fault_win = (t > FAULT_T) & (t <= FAULT_T + 10.0)
    d_max_fault = float(np.max(b['delta'][fault_win]))
    print(f"[B] absorption @ fault+20s: resid RMS {resid_rms_late:.2f} mm, "
          f"x-bias left {bias_left:.2f} mm, delta {1e3*d_late:.2f} mm "
          f"(fault max {1e3*d_max_fault:.2f} mm, pre {1e3*d_pre:.2f} mm)")
    if not (resid_rms_late < 3.0 and bias_left < 2.0):
        fails.append(f"theta did not absorb bias within 20s "
                     f"(resid RMS {resid_rms_late:.2f} mm, "
                     f"x-bias {bias_left:.2f} mm)")
    if not (d_late < 0.6 * d_max_fault and d_late < 1.5 * d_pre):
        fails.append(f"delta did not contract within 20s "
                     f"({1e3*d_late:.2f} mm vs max {1e3*d_max_fault:.2f}, "
                     f"pre {1e3*d_pre:.2f})")
    notes.append(f"fault: b {b_pre:.2f}->halved in {t_half}s, "
                 f"delta {1e3*d_pre:.1f}->{1e3*d_max_5s:.1f}->"
                 f"{1e3*d_late:.1f} mm, x-bias left {bias_left:.2f} mm")

    test_pass = not fails
    if fails:
        notes.extend("FAIL: " + f for f in fails)
    result = {
        'test_pass': test_pass,
        'one_step_rms_mm': round(a['rms1_mm'], 4),
        'ten_step_rms_mm': round(a['rms10_mm'], 4),
        'const_hold_10step_mm': round(a['zoh10_mm'], 4),
        'frozen_extrap_rms_mm': round(a['frozen_rms_mm'], 4),
        'coverage': round(a['coverage'], 4),
        'notes': "; ".join(notes),
    }
    print(json.dumps(result))
    return 0 if test_pass else 1


if __name__ == '__main__':
    sys.exit(main())
