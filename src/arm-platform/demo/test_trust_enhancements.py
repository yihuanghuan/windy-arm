#!/usr/bin/env python3
"""β 信任新语义、前馈门控与相对 δ 阶梯状态机自测。"""

import math
import sys

import numpy as np

from base_trust_estimator import (
    BetaConfidenceFilter,
    TrustFeedforwardGate,
    trust_feedforward_gain,
)
from online_path_time_scaler import (
    TrustLadderConfig,
    TrustLadderStateMachine,
)


DT = 0.02


def test_beta_health_fault_recovery():
    rng = np.random.default_rng(20260726)
    filt = BetaConfidenceFilter(sigma0=2.0e-3, gamma_f=0.98)

    healthy = []
    for _ in range(int(12.0 / DT)):
        healthy.append(filt.update(rng.normal(0.0, 2.0e-3, 3)))
    healthy_mean = float(np.mean(healthy[-int(2.0 / DT):]))
    assert 0.7 <= healthy_mean <= 1.0, healthy_mean

    fault = []
    for _ in range(int(2.0 / DT)):
        resid = rng.normal(0.0, 2.0e-3, 3) + np.array([0.010, 0.0, 0.0])
        fault.append(filt.update(resid))
    below = np.flatnonzero(np.asarray(fault) < 0.3)
    assert below.size, min(fault)
    fault_drop_sec = (int(below[0]) + 1) * DT
    assert fault_drop_sec <= 2.0, fault_drop_sec

    recovery = []
    for _ in range(int(10.0 / DT)):
        recovery.append(filt.update(rng.normal(0.0, 2.0e-3, 3)))
    recovered = np.flatnonzero(np.asarray(recovery) >= 0.7)
    assert recovered.size, recovery[-1]
    recovery_sec = (int(recovered[0]) + 1) * DT
    assert recovery_sec <= 10.0, recovery_sec
    print(
        f'beta: healthy_mean={healthy_mean:.3f}, '
        f'fault<0.3@{fault_drop_sec:.2f}s, recover>=0.7@{recovery_sec:.2f}s')


def test_feedforward_gate_semantics():
    assert trust_feedforward_gain(float('nan')) == 0.0
    assert trust_feedforward_gain(0.0) == 0.0
    assert trust_feedforward_gain(0.3) == 0.0
    assert trust_feedforward_gain(0.7) == 1.0
    assert trust_feedforward_gain(1.0) == 1.0
    xs = np.linspace(0.0, 1.0, 101)
    ys = np.array([trust_feedforward_gain(x) for x in xs])
    assert np.all(np.diff(ys) >= -1e-15)
    assert abs(trust_feedforward_gain(0.5) - 0.5) < 1e-12

    gate = TrustFeedforwardGate()
    # 持续故障必须在 2s 内基本退出模型。
    fault_gain = []
    for _ in range(int(2.0 / DT)):
        fault_gain.append(gate.update(0.0, DT))
    assert fault_gain[-1] < 0.05, fault_gain[-1]
    # 恢复较慢，避免健康段低脉冲后立即来回切换；但 10s 内必须回满。
    recovered_at = None
    for k in range(int(10.0 / DT)):
        g = gate.update(1.0, DT)
        if recovered_at is None and g >= 0.99:
            recovered_at = (k + 1) * DT
    assert recovered_at is not None and recovered_at <= 10.0


def test_ladder_state_machine_semantics():
    cfg = TrustLadderConfig(
        b_degraded=0.3,
        b_recover=0.7,
        delta_degraded_ratio=1.5,
        delta_recover_ratio=1.3,
        min_scale=0.3,
        baseline_warmup_sec=2.0,
        baseline_tau_sec=20.0,
        recover_debounce_sec=2.0,
    )
    ladder = TrustLadderStateMachine(cfg)
    t = 0.0

    # 健康噪声地板约 5mm：warming 期间不得因为绝对值 >4mm 误降速。
    for k in range(int(3.0 / DT)):
        delta = 0.005 + 0.00025 * math.sin(2.0 * math.pi * k * DT)
        assert ladder.update(t, 0.9, delta) == 1.0
        t += DT
    assert ladder.state == 'healthy', ladder.state
    assert 0.0047 <= ladder.delta_baseline <= 0.0053, ladder.delta_baseline

    # 健康抖动不触发，且 cap 保持全速。
    for k in range(50):
        ladder.update(t, 0.75, 0.0058 + 0.0001 * (k % 2))
        t += DT
    assert ladder.state == 'healthy'
    assert ladder.cap == 1.0

    # b 与相对 δ 同时越门：当拍降速，不等待恢复防抖。
    cap = ladder.update(t, 0.1, 0.012)
    fault_t = t
    t += DT
    assert ladder.state == 'degraded'
    assert cfg.min_scale <= cap < 0.5, cap

    # 中间区不算恢复；健康门短暂满足后被打断，防抖应重新计时。
    for _ in range(25):
        ladder.update(t, 0.5, 0.006)
        t += DT
    assert ladder.state == 'degraded'
    for _ in range(50):
        ladder.update(t, 0.85, 0.0055)
        t += DT
    assert ladder.state == 'recovering'
    ladder.update(t, 0.6, 0.0055)
    t += DT
    assert ladder.state == 'degraded'

    recover_start = t
    while t - recover_start <= 2.2:
        ladder.update(t, 0.9, 0.0054)
        t += DT
    assert ladder.state == 'healthy', ladder.state
    assert ladder.cap == 1.0
    assert t - fault_t < 10.0
    print(
        f'ladder: delta0={1e3*ladder.delta_baseline:.2f}mm, '
        f'fault_cap={cap:.3f}, recovered_after={t-fault_t:.2f}s')

    # 冷启动模型的低 b 不得造成整程误降速；未 commission 时 δ 仍可捕获故障。
    cold = TrustLadderStateMachine(TrustLadderConfig(
        baseline_warmup_sec=1.0,
        delta_degraded_ratio=1.35,
        delta_recover_ratio=1.2,
    ))
    t = 0.0
    for _ in range(int(2.0 / DT)):
        cold.update(t, 0.0, 0.005)
        t += DT
    assert cold.state == 'healthy' and cold.cap == 1.0
    assert not cold.trust_commissioned
    cold.update(t, 0.0, 0.008)
    assert cold.state == 'degraded' and cold.cap < 1.0


def main():
    tests = [
        test_beta_health_fault_recovery,
        test_feedforward_gate_semantics,
        test_ladder_state_machine_semantics,
    ]
    failed = []
    for fn in tests:
        try:
            fn()
            print(f'PASS {fn.__name__}')
        except Exception as exc:  # noqa: BLE001
            failed.append(f'{fn.__name__}: {exc!r}')
            print(f'FAIL {fn.__name__}: {exc!r}')
    print(f'{len(tests) - len(failed)}/{len(tests)} passed')
    if failed:
        print('; '.join(failed))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
