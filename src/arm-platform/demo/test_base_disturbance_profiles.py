#!/usr/bin/env python3
"""base_disturbance_offset profile 扩展自测(pytest 风格, python3 可直跑).

验证三条硬性要求:
  1. 有界性: 所有 profile 下每轴 |Δp_i(t)| <= 2*amplitude_m
     (harmonic/wideband 实际界 amplitude, gust/fault 实际界 1.5*amplitude)
  2. 确定性: 同参同 t 同值(含清缓存后/子进程重算), 任意 t(含远未来)可求值
  3. 向后兼容: 默认参数(profile='harmonic')与历史实现输出比特一致
"""

import math
import os
import subprocess
import sys
from functools import lru_cache

import numpy as np

_DEMO_DIR = os.path.dirname(os.path.abspath(__file__))
if _DEMO_DIR not in sys.path:
    sys.path.insert(0, _DEMO_DIR)

import moving_base_trajectory_config as mbt  # noqa: E402

PROFILES = ('harmonic', 'gust', 'fault', 'wideband')
SEEDS = (0, 1, 7, 42)
AMPLITUDES = (0.005, 0.02, 0.1)


# --- 历史实现的独立参考副本(改动前的 base_disturbance_offset 逐行拷贝) ---
@lru_cache(maxsize=32)
def _ref_table(seed, n_harmonics, fmin_hz, fmax_hz):
    rng = np.random.default_rng(int(seed))
    freqs = np.exp(rng.uniform(
        math.log(fmin_hz), math.log(fmax_hz), (n_harmonics, 3)))
    phases = rng.uniform(0.0, 2.0 * math.pi, (n_harmonics, 3))
    weights = rng.uniform(0.5, 1.0, (n_harmonics, 3))
    weights = weights / weights.sum(axis=0, keepdims=True)
    return freqs, phases, weights


def _ref_offset_legacy(t, amplitude_m, seed=0, n_harmonics=6,
                       fmin_hz=0.15, fmax_hz=2.5):
    if amplitude_m <= 0.0:
        return np.zeros(3)
    freqs, phases, weights = _ref_table(
        int(seed), int(n_harmonics), float(fmin_hz), float(fmax_hz))
    s = np.sin(2.0 * math.pi * freqs * float(t) + phases)
    return float(amplitude_m) * (weights * s).sum(axis=0)


def _time_grid():
    # 密集网格覆盖脉冲区/故障窗边界 + 负时刻 + 远未来
    grid = list(np.arange(-20.0, 40.0, 0.013))
    grid += [5.999, 6.0, 6.001, 13.999, 14.0, 14.001]      # fault 边界
    for k in range(-2, 5):                                  # gust 脉冲区
        base = k * 8.0
        grid += list(base + np.linspace(0.0, 0.8, 33))
        grid += [base - 1e-9, base + 0.8 + 1e-9, base + 0.4]
    grid += [1e3, 1e4 + 0.37, 1e6, -1e5 + 0.11]             # 远未来/远过去
    return grid


# ---------- ① 有界性 ----------
def test_bound_all_profiles():
    ts = _time_grid()
    for profile in PROFILES:
        tight = 1.0 if profile in ('harmonic', 'wideband') else 1.5
        for seed in SEEDS:
            for amp in AMPLITUDES:
                eps = 1e-9 * amp
                for t in ts:
                    off = mbt.base_disturbance_offset(
                        t, amp, seed=seed, profile=profile)
                    assert off.shape == (3,)
                    assert np.all(np.isfinite(off)), (profile, seed, t)
                    m = np.max(np.abs(off))
                    assert m <= 2.0 * amp + eps, \
                        f'{profile} seed={seed} t={t}: |off|={m} > 2*amp'
                    assert m <= tight * amp + eps, \
                        f'{profile} seed={seed} t={t}: |off|={m} > {tight}*amp'


def test_bound_zero_amplitude():
    for profile in PROFILES:
        off = mbt.base_disturbance_offset(3.3, 0.0, seed=1, profile=profile)
        assert np.array_equal(off, np.zeros(3))


# ---------- ② 确定性 + 任意 t 可求值 ----------
def test_deterministic_across_cache_clear():
    ts = [-7.7, 0.0, 0.41, 6.5, 8.4, 14.0, 123.456, 1e6]
    for profile in PROFILES:
        first = [mbt.base_disturbance_offset(
            t, 0.02, seed=3, profile=profile) for t in ts]
        mbt._disturbance_table.cache_clear()
        mbt._disturbance_directions.cache_clear()
        second = [mbt.base_disturbance_offset(
            t, 0.02, seed=3, profile=profile) for t in ts]
        for t, a, b in zip(ts, first, second):
            assert a.tobytes() == b.tobytes(), \
                f'{profile} t={t}: 清缓存后结果不一致'


def test_deterministic_across_process():
    snippet = (
        "import sys; sys.path.insert(0, {d!r})\n"
        "import moving_base_trajectory_config as mbt\n"
        "for p in {profiles!r}:\n"
        "    for t in (0.41, 8.4, 7.5, 1e6):\n"
        "        print(mbt.base_disturbance_offset("
        "t, 0.02, seed=3, profile=p).tobytes().hex())\n"
    ).format(d=_DEMO_DIR, profiles=PROFILES)
    out = subprocess.run([sys.executable, '-c', snippet],
                         capture_output=True, text=True, check=True)
    lines = out.stdout.strip().splitlines()
    expected = [mbt.base_disturbance_offset(
        t, 0.02, seed=3, profile=p).tobytes().hex()
        for p in PROFILES for t in (0.41, 8.4, 7.5, 1e6)]
    assert lines == expected, '子进程重算与本进程不一致(非确定性)'


# ---------- ③ 默认参数与历史实现比特一致 ----------
def test_harmonic_bitwise_backward_compatible():
    for seed in SEEDS:
        for amp in (0.0, 0.005, 0.02, 0.1):
            for t in (-3.0, 0.0, 0.123, 1.0, 9.87, 100.0, 1e5):
                legacy = _ref_offset_legacy(t, amp, seed=seed)
                new_default = mbt.base_disturbance_offset(t, amp, seed=seed)
                new_explicit = mbt.base_disturbance_offset(
                    t, amp, seed=seed, profile='harmonic')
                assert legacy.tobytes() == new_default.tobytes(), \
                    f'默认调用与历史输出不比特一致 seed={seed} t={t}'
                assert legacy.tobytes() == new_explicit.tobytes()
    # 非默认谐波参数也应一致
    for t in (0.0, 2.5, 77.7):
        legacy = _ref_offset_legacy(t, 0.03, seed=5, n_harmonics=10,
                                    fmin_hz=0.3, fmax_hz=4.0)
        new = mbt.base_disturbance_offset(t, 0.03, seed=5, n_harmonics=10,
                                          fmin_hz=0.3, fmax_hz=4.0)
        assert legacy.tobytes() == new.tobytes()


# ---------- 结构性检查(各 profile 语义) ----------
def test_gust_structure():
    amp, seed = 0.02, 2
    dirs = mbt._disturbance_directions(seed)
    assert dirs.shape == (mbt._DISTURBANCE_N_DIRECTIONS, 3)
    assert np.allclose(np.linalg.norm(dirs, axis=1), 1.0, atol=1e-12)
    for k in (0, 1, 2, 9):
        # 脉冲峰值(中心 tau=width/2): 脉冲分量 == amp * 方向(轮换 k mod N)
        tc = k * 8.0 + 0.4
        half = mbt._harmonic_offset(tc, 0.5 * amp, seed, 6, 0.15, 2.5)
        pulse = mbt.base_disturbance_offset(
            tc, amp, seed=seed, profile='gust') - half
        want = amp * dirs[k % mbt._DISTURBANCE_N_DIRECTIONS]
        assert np.allclose(pulse, want, atol=1e-12 * amp + 1e-15), \
            f'gust 峰值/方向不符 k={k}'
        # 脉冲区外 == 半幅谐波
        to = k * 8.0 + 4.0
        off = mbt.base_disturbance_offset(to, amp, seed=seed, profile='gust')
        half_o = mbt._harmonic_offset(to, 0.5 * amp, seed, 6, 0.15, 2.5)
        assert np.array_equal(off, half_o), 'gust 脉冲区外应为纯半幅谐波'
    # 方向轮换: 相邻脉冲方向不同
    assert not np.allclose(dirs[0], dirs[1])


def test_gust_component_sample_boundaries():
    amp, seed = 0.02, 2
    direction = mbt._disturbance_directions(seed)[0]
    cases = (
        (0.0, True, 0.0),
        (0.4, True, 1.0),
        (0.8, False, 0.0),
    )
    for t, active, strength in cases:
        sample = mbt.base_disturbance_sample(
            t, amp, seed=seed, profile='gust')
        assert sample.event_active is active
        assert sample.event_label == 'gust'
        assert abs(sample.event_strength - strength) <= 1e-15
        assert np.array_equal(
            sample.total_offset,
            sample.background_offset + sample.event_offset)
        assert np.allclose(
            sample.event_offset, amp * strength * direction,
            atol=1e-15)


def test_fault_structure():
    amp, seed = 0.02, 4
    dirs = mbt._disturbance_directions(seed)
    for t in (6.0, 8.0, 10.5, 14.0):   # 窗内(含边界): 恒定偏置
        half = mbt._harmonic_offset(t, 0.5 * amp, seed, 6, 0.15, 2.5)
        bias = mbt.base_disturbance_offset(
            t, amp, seed=seed, profile='fault') - half
        assert np.allclose(bias, amp * dirs[0], atol=1e-15), \
            f'fault 窗内偏置不恒定/方向不符 t={t}'
        assert abs(np.linalg.norm(bias) - amp) < 1e-12 * amp + 1e-15
    for t in (-1.0, 5.999, 14.001, 100.0):  # 窗外: 纯半幅谐波
        off = mbt.base_disturbance_offset(t, amp, seed=seed, profile='fault')
        half = mbt._harmonic_offset(t, 0.5 * amp, seed, 6, 0.15, 2.5)
        assert np.array_equal(off, half), f'fault 窗外应为纯半幅谐波 t={t}'


def test_fault_component_sample_boundaries():
    amp, seed = 0.02, 4
    direction = mbt._disturbance_directions(seed)[0]
    for t, active in (
            (5.999, False), (6.0, True),
            (14.0, True), (14.001, False)):
        sample = mbt.base_disturbance_sample(
            t, amp, seed=seed, profile='fault')
        assert sample.event_active is active
        assert sample.event_strength == (1.0 if active else 0.0)
        assert sample.event_label == 'bias'
        assert np.array_equal(
            sample.total_offset,
            sample.background_offset + sample.event_offset)
        want = amp * direction if active else np.zeros(3)
        assert np.array_equal(sample.event_offset, want)


def test_wideband_structure():
    amp, seed = 0.02, 6
    # 等价于谐波数翻倍 + 频带 [0.15, 8.0]
    for t in (0.0, 1.234, 50.0):
        off = mbt.base_disturbance_offset(
            t, amp, seed=seed, profile='wideband')
        want = mbt._harmonic_offset(t, amp, seed, 12, 0.15, 8.0)
        assert np.array_equal(off, want)
    tbl = mbt._disturbance_table(seed, 12, 0.15, 8.0)
    freqs = tbl[0]
    assert freqs.shape == (12, 3)
    assert freqs.min() >= 0.15 and freqs.max() <= 8.0


def test_unknown_profile_raises():
    try:
        mbt.base_disturbance_offset(0.0, 0.02, profile='nope')
    except ValueError:
        return
    raise AssertionError('未知 profile 应抛 ValueError')


def _main():
    tests = [(name, fn) for name, fn in sorted(globals().items())
             if name.startswith('test_') and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f'PASS {name}')
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f'FAIL {name}: {exc!r}')
    print(f'\n{len(tests) - failed}/{len(tests)} passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(_main())
