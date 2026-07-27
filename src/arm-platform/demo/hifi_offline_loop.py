#!/usr/bin/env python3
"""High-fidelity offline closed loop for the moving-base second-order NMPC.

Event-timed replica of the real 3-terminal chain:
  - plant: 100 Hz substeps on its own grid (boundaries offset -8ms vs MPC ticks)
  - /joint_states: published each plant boundary; MPC uses latest -> ~8ms age
  - command: published ~9ms after solve start, applied from next plant boundary
  - MPC: production SecondOrderMpcIK defaults (N=10, RTI3, lead=1,
         velocity_source='commanded', posture reset_nominal)

Usage: run from src/arm-platform/demo with acados env sourced.
"""
import argparse
import math
import sys

import numpy as np

sys.path.insert(0, '/home/windylab/code/windylab_ws/src/arm-platform/demo')

import moving_base_trajectory_config as mbtraj
import tune_unified_mpc_params as T
from base_trust_estimator import BaseTrustEstimator, TrustFeedforwardGate
from pinocchio_ik import PinocchioIK
from second_order_mpc_ik import SecondOrderMpcConfig, SecondOrderMpcIK

DT = 0.02
SUB = 0.01
PLANT_PHASE = -0.008     # plant boundary grid offset vs tick grid
PUB_DELAY = 0.009        # solve start -> command active (measured P50 8.75ms)
BASE_F = 2.0


class Plant:
    """smooth_position_controller.cc velocity-feedforward branch, 100 Hz."""

    def __init__(self, q0, q_lower, q_upper, gain=1.0, vmax=1.0, amax=5.0,
                 fb_tau=0.15, fb_lim=0.4):
        self.q = q0.astype(float).copy()
        self.v_last = np.zeros_like(self.q)
        self.q_lower, self.q_upper = q_lower, q_upper
        self.gain, self.vmax, self.amax = gain, vmax, amax
        self.fb_tau, self.fb_lim = fb_tau, fb_lim
        self.accel_sat = 0
        self.vel_sat = 0
        self.substeps = 0

    def step(self, q_cmd, dq_ff):
        v_ff = np.clip(dq_ff, -self.vmax, self.vmax)
        v_fb = np.clip((q_cmd - self.q) / self.fb_tau, -self.fb_lim, self.fb_lim)
        raw = self.gain * v_ff + v_fb
        v_raw = np.clip(raw, -self.vmax, self.vmax)
        self.vel_sat += int(np.any(np.abs(raw) >= self.vmax - 1e-12))
        step = self.amax * SUB
        v_cmd = np.clip(v_raw, self.v_last - step, self.v_last + step)
        self.accel_sat += int(np.any(np.abs(v_raw - self.v_last) > step + 1e-12))
        q_new = np.clip(self.q + v_cmd * SUB, self.q_lower, self.q_upper)
        self.v_last = (q_new - self.q) / SUB
        self.q = q_new
        self.substeps += 1


def decompose(t, e_mm, v_ref_mm):
    """DC + 2Hz + 4Hz + along-track lag decomposition; returns dict of RMS."""
    w = 2 * np.pi * BASE_F
    A = np.stack([np.ones_like(t), np.sin(w * t), np.cos(w * t),
                  np.sin(2 * w * t), np.cos(2 * w * t)], axis=1)
    parts = {k: np.zeros_like(e_mm) for k in ('dc', 'f1', 'f2', 'lag')}
    resid = np.zeros_like(e_mm)
    for i in range(3):
        Ai = np.column_stack([A, v_ref_mm[:, i]])
        coef, *_ = np.linalg.lstsq(Ai, e_mm[:, i], rcond=None)
        parts['dc'][:, i] = coef[0]
        parts['f1'][:, i] = Ai[:, 1] * coef[1] + Ai[:, 2] * coef[2]
        parts['f2'][:, i] = Ai[:, 3] * coef[3] + Ai[:, 4] * coef[4]
        parts['lag'][:, i] = Ai[:, 5] * coef[5]
        resid[:, i] = e_mm[:, i] - Ai @ coef

    def rms(x):
        return float(np.sqrt(np.mean(np.sum(x ** 2, axis=1))))
    out = {k: rms(v) for k, v in parts.items()}
    out['resid'] = rms(resid)
    out['total'] = rms(e_mm)
    return out


def smith_predict(stamp, q_js, v_js, cmd_hist, t_target, gain, vmax, amax,
                  fb_tau, fb_lim, q_lower, q_upper):
    """Forward-simulate the known plant law from a js sample to t_target."""
    q, v_last = q_js.copy(), v_js.copy()
    t = stamp
    while t < t_target - 1e-9:
        active = cmd_hist[0]
        for c in cmd_hist:
            if c[0] <= t + 1e-9:
                active = c
        q_c, v_c = active[1], active[2]
        v_fb = np.clip((q_c - q) / fb_tau, -fb_lim, fb_lim)
        v_raw = np.clip(gain * v_c + v_fb, -vmax, vmax)
        v = np.clip(v_raw, v_last - amax * SUB, v_last + amax * SUB)
        q = np.clip(q + v * SUB, q_lower, q_upper)
        v_last = v
        t += SUB
    return q, v_last


def run(label, gain=1.0, plant_vmax=1.0, plant_amax=5.0,
        mpc_vmax=1.0, mpc_amax=5.0, lead=1,
        latency_comp=False, act_delay_ms=0.0, state_pred=False,
        dq_mode='model', dq_ff_comp=1.0,
        warm_sec=4.0, steady_sec=8.0, rti=3, horizon=10,
        ori_w=0.09375, ori_wt=1.25,
        smith=False, invert=False, tau_model=0.15,
        pub_jitter_ms=0.0, tau_true=None, phase_err_ms=0.0, seed=0,
        dist_amp=0.0, dist_seed=0, belief='truth',
        est_noise_m=0.0, est_delay_ms=0.0, est_lpf_hz=0.0, est_seed=1,
        dist_profile='harmonic', trust_kwargs=None,
        trust_gate_enabled=True, trust_gate_low=0.2, trust_gate_high=0.5,
        trust_gate_fall_tau_sec=0.3, trust_gate_rise_tau_sec=1.0):
    """belief: MPC 对基座扰动的信念通道。
    'nominal'   只知名义轨迹(第1级下界: 误差≈扰动本身);
    'truth'     精确知道扰动(上界/装置自检: 应与无扰动精度同量级);
    'estimated' 每拍用 真值(t-延迟)+噪声 经一阶低通得到 Δ̂, 时域内保持常值
                (第2级: 模拟无人机状态估计品质);
    'model'     每拍把 estimated 同款测量(真值(t-延迟)+噪声)喂给
                BaseTrustEstimator.step(), 预测域各 stage 用
                g(b)·predict_horizon(times) 的 Δ̂(t_j) 逐时刻前馈；低信任
                自动回到 nominal，高信任保留完整模型补偿，
                同时记录保形半径 δ 序列(第3级: 谱辨识谐波模型+保形校准)。
                注意: 估计器谱辨识需要 ≥20s 测量史才收敛(环形缓冲设计),
                warm_sec=4 只能覆盖冷启动暂态, 稳态评估应配 warm_sec>=20。
    真值(误差评估)恒为 名义+扰动; dist_amp=0 时各信念完全等价。

    dist_profile: 透传 base_disturbance_offset 的 profile
                  (真值/estimated测量/model测量三处同源, 天然一致);
    trust_kwargs: dict, 直传 BaseTrustEstimator 构造器
                  (lam/n_peaks/ident_period_sec/min_ident_sec/buffer_sec/
                  gamma_aci/n_cal 等; 可覆盖默认的
                  meas_delay_sec=est_delay_ms*1e-3 与 dt_nominal=DT)。
    后续调参与实验矩阵只准用这两个透传参数, 不再改源码。
    """
    sc = T.make_moving_base_scenario('hifi')
    ctx = T.make_context(1.0, 2.0)
    q0, ok = PinocchioIK().solve(sc.position_fn(0.0),
                                 target_rotation=sc.rotation_fn(0.0),
                                 rot_weight=0.2)
    assert ok
    q0 = np.clip(q0, ctx.q_lower, ctx.q_upper)
    max_joint_step = mpc_vmax * DT
    solver = SecondOrderMpcIK(config=SecondOrderMpcConfig(
        horizon_steps=horizon, control_dt=DT, max_joint_step=max_joint_step,
        success_tolerance=0.01,
        position_weight=5000.0, terminal_position_weight=500.0,
        transition_position_weight=5000.0,
        terminal_transition_position_weight=500.0,
        contour_tracking_enabled=False,
        orientation_weight=ori_w, terminal_orientation_weight=ori_wt,
        current_posture_weight=0.01, center_posture_weight=0.002,
        joint_velocity_weight=0.02, joint_acceleration_weight=0.0,
        max_joint_velocity=mpc_vmax, max_joint_acceleration=mpc_amax,
        min_translation_jacobian_sigma=0.05,
        acados_rti_iterations=rti,
        acados_build_dir=(f'/tmp/windylab_acados_hifi_v{mpc_vmax}'
                          f'_a{mpc_amax}_h{horizon}')))

    plant = Plant(q0, ctx.q_lower, ctx.q_upper, gain=gain,
                  vmax=plant_vmax, amax=plant_amax,
                  fb_tau=(tau_true if tau_true is not None else 0.15))
    rng = np.random.default_rng(seed)

    n_ticks = int(round((warm_sec + steady_sec) / DT))
    # plant boundaries within [PLANT_PHASE, end]
    warm_start = None
    last_cmd_dq = None
    last_ddq0 = None
    posture_ref = q0.copy()
    # command table: (activation_time, q_cmd, dq_ff); plant holds latest active
    cmd = [(-1e9, q0.copy(), np.zeros_like(q0))]
    # js samples: (stamp, q, v)
    js = [(PLANT_PHASE, q0.copy(), np.zeros_like(q0))]
    errs_t, errs, vrefs = [], [], []
    unusable = 0

    def truth_offset(tq):
        return mbtraj.base_disturbance_offset(tq, dist_amp, dist_seed,
                                              profile=dist_profile)

    def world_err(tq, q):
        p, _ = T.fk(ctx, q)
        pw = sc.base_pose_fn(tq) + truth_offset(tq) + sc.base_rot_fn(tq) @ p
        tw = sc.base_pose_fn(tq) + sc.base_rot_fn(tq) @ sc.position_fn(tq)
        return pw - tw, tw

    est_rng = np.random.default_rng(est_seed)
    belief_offset_state = np.zeros(3)
    est_alpha = (1.0 - math.exp(-2.0 * math.pi * est_lpf_hz * DT)
                 if est_lpf_hz > 0.0 else 1.0)
    trust_est = None
    trust_gate = None
    delta_log = []                     # (t_k, 保形半径 δ) 序列
    trust_log = []                     # (t_k, b, g(b)) 序列
    if belief == 'model':
        tk_kwargs = dict(meas_delay_sec=est_delay_ms * 1e-3, dt_nominal=DT)
        if trust_kwargs:
            tk_kwargs.update(trust_kwargs)
        trust_est = BaseTrustEstimator(**tk_kwargs)
        trust_gate = TrustFeedforwardGate(
            low=trust_gate_low, high=trust_gate_high,
            fall_tau_sec=trust_gate_fall_tau_sec,
            rise_tau_sec=trust_gate_rise_tau_sec)

    next_boundary = PLANT_PHASE + SUB
    for k in range(n_ticks):
        t_k = k * DT
        # ---- MPC tick ----
        stamp, q_fb, v_fb = js[-1]
        for s in reversed(js):
            if s[0] <= t_k - 0.0005:
                stamp, q_fb, v_fb = s
                break
        feedback_age = t_k - stamp
        dq0 = (last_cmd_dq if last_cmd_dq is not None else v_fb).copy()
        dq0 = np.clip(dq0, -mpc_vmax, mpc_vmax)
        q0_solve, dq0_solve = q_fb.copy(), dq0
        app_delay = (PUB_DELAY + act_delay_ms * 1e-3) if latency_comp else 0.0
        if smith:
            t_a_est = t_k + PUB_DELAY + act_delay_ms * 1e-3 + phase_err_ms * 1e-3
            n = max(0, int(round((t_a_est - stamp) / SUB)))
            q_pred, v_pred = smith_predict(
                stamp, q_fb, v_fb, cmd, stamp + n * SUB, gain,
                plant_vmax, plant_amax, tau_model, 0.4,
                ctx.q_lower, ctx.q_upper)
            q0_solve = q_pred
            dq0_solve = np.clip(
                last_cmd_dq if last_cmd_dq is not None else v_pred,
                -mpc_vmax, mpc_vmax)
        elif state_pred:
            age = feedback_age + app_delay
            ddq = last_ddq0 if last_ddq0 is not None else np.zeros_like(q_fb)
            q0_solve = np.clip(q_fb + age * dq0 + 0.5 * age * age * ddq,
                               ctx.q_lower, ctx.q_upper)
            dq0_solve = np.clip(dq0 + age * ddq, -mpc_vmax, mpc_vmax)
        times = t_k + app_delay + DT * (lead + np.arange(horizon))
        rots = np.array([sc.rotation_fn(float(ti)) for ti in times])
        # 信念通道: 基座系目标 = R_nomᵀ(p_ee_w - p_b_nom - Δ_belief)
        #         = position_fn - R_nomᵀ Δ_belief (扰动仅位置, 旋转名义)。
        model_pred = None
        if dist_amp > 0.0 and belief == 'estimated':
            meas = (truth_offset(t_k - est_delay_ms * 1e-3)
                    + est_rng.normal(0.0, est_noise_m, 3))
            belief_offset_state = (
                belief_offset_state + est_alpha * (meas - belief_offset_state))
        elif dist_amp > 0.0 and belief == 'model':
            meas = (truth_offset(t_k - est_delay_ms * 1e-3)
                    + est_rng.normal(0.0, est_noise_m, 3))
            step_diag = trust_est.step(t_k, meas)
            model_pred, delta_cal = trust_est.predict_horizon(times)
            gate = (trust_gate.update(step_diag['trust'], DT)
                    if trust_gate_enabled else 1.0)
            model_pred = gate * model_pred
            delta_log.append((t_k, float(delta_cal)))
            trust_log.append((
                t_k, float(step_diag['trust']), float(gate),
                float(trust_gate.filtered_trust)))
        targets = []
        for j, ti in enumerate(times):
            base_target = sc.position_fn(float(ti))
            if dist_amp > 0.0 and belief != 'nominal':
                if belief == 'truth':
                    off = truth_offset(float(ti))
                elif belief == 'model':
                    off = model_pred[j]
                else:
                    off = belief_offset_state
                base_target = base_target - sc.base_rot_fn(float(ti)).T @ off
            targets.append(base_target)
        targets = np.array(targets)
        q_next, dq_next, ok, info = solver.step(
            targets, q0_solve, dq0_solve, warm_start=warm_start,
            target_rotation=rots, posture_reference=posture_ref)
        warm_start = info.get('trajectory')
        if info.get('command_usable', ok):
            if invert:
                dqp = q_next - q0_solve
                v1 = dqp / (SUB * (2.0 - SUB / tau_model))
                dq_pub = np.clip((v1 - dqp / tau_model) / gain,
                                 -plant_vmax, plant_vmax)
            elif dq_mode == 'meanvel':
                dq_pub = (q_next - q0_solve) / DT
                dq_pub = np.clip(dq_pub / dq_ff_comp, -max_joint_step / DT,
                                 max_joint_step / DT)
            else:
                dq_pub = dq_next.copy()
                dq_pub = np.clip(dq_pub / dq_ff_comp, -max_joint_step / DT,
                                 max_joint_step / DT)
            t_pub = t_k + PUB_DELAY + (
                rng.uniform(-pub_jitter_ms, pub_jitter_ms) * 1e-3
                if pub_jitter_ms > 0 else 0.0)
            cmd.append((t_pub, q_next.copy(), dq_pub))
            last_cmd_dq = dq_next.copy()
            last_ddq0 = np.asarray(info['ddq0'], dtype=float)
        else:
            unusable += 1
            cmd.append((t_k + PUB_DELAY, q_fb.copy(), np.zeros_like(q_fb)))
            last_cmd_dq = None
            last_ddq0 = None
            warm_start = None
            solver.reset_warm_start()
        # ---- advance plant to next tick ----
        while next_boundary <= t_k + DT + 1e-9:
            active = cmd[0]
            for c in cmd:
                if c[0] <= next_boundary - SUB + 1e-9:
                    active = c
            plant.step(active[1], active[2])
            js.append((next_boundary, plant.q.copy(), plant.v_last.copy()))
            if len(js) > 8:
                js.pop(0)
            if len(cmd) > 8:
                cmd = cmd[-8:]
            if next_boundary >= warm_sec:
                e, tw = world_err(next_boundary, plant.q)
                h = 1e-3
                _, twp = world_err(next_boundary + h, plant.q)
                _, twm = world_err(next_boundary - h, plant.q)
                errs_t.append(next_boundary)
                errs.append(e * 1000.0)
                vrefs.append((twp - twm) / (2 * h) * 1000.0)
            next_boundary += SUB

    t = np.array(errs_t)
    e = np.array(errs)
    v = np.array(vrefs)
    d = decompose(t, e, v)
    nrm = np.linalg.norm(e, axis=1)
    d['p95'] = float(np.percentile(nrm, 95))
    d['max'] = float(np.max(nrm))
    if delta_log:
        dl_t = np.array([tt for tt, _ in delta_log])
        dl = np.array([dv for _, dv in delta_log]) * 1e3
        d['delta_series_mm'] = dl
        d['delta_t'] = dl_t
        steady = dl[dl_t >= warm_sec]
        d['delta_p95'] = float(np.percentile(steady, 95))
        d['delta_mean'] = float(np.mean(steady))
    if trust_log:
        tl = np.asarray(trust_log, dtype=float)
        steady = tl[tl[:, 0] >= warm_sec]
        d['trust_mean'] = float(np.mean(steady[:, 1]))
        d['trust_min'] = float(np.min(steady[:, 1]))
        d['gate_mean'] = float(np.mean(steady[:, 2]))
        d['gate_min'] = float(np.min(steady[:, 2]))
        d['trust_filtered_mean'] = float(np.mean(steady[:, 3]))
        d['trust_filtered_min'] = float(np.min(steady[:, 3]))
    print(f'[{label}] total={d["total"]:.3f} p95={np.percentile(nrm, 95):.3f} '
          f'max={np.max(nrm):.3f} | DC={d["dc"]:.3f} 2Hz={d["f1"]:.3f} '
          f'4Hz={d["f2"]:.3f} lag={d["lag"]:.3f} resid={d["resid"]:.3f} (mm) | '
          f'unusable={unusable} plant_acc_sat={plant.accel_sat}/{plant.substeps} '
          f'vel_sat={plant.vel_sat}'
          + (f' | δ_p95={d["delta_p95"]:.3f}mm δ_mean={d["delta_mean"]:.3f}mm'
             if 'delta_p95' in d else '')
          + (f' | b_mean/min={d["trust_mean"]:.3f}/{d["trust_min"]:.3f}'
             f' g_mean/min={d["gate_mean"]:.3f}/{d["gate_min"]:.3f}'
             if 'trust_mean' in d else ''), flush=True)
    return d


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--set', default='baseline')
    args = ap.parse_args()
    if args.set == 'baseline':
        run('baseline-replica gain1.0 N10 lead1')
    elif args.set == 'phase':
        run('B0 baseline', )
        run('B1 +latency_comp(act5ms)', latency_comp=True, act_delay_ms=5.0)
        run('B2 +state_pred', latency_comp=True, act_delay_ms=5.0,
            state_pred=True)
        run('B3 +meanvel', latency_comp=True, act_delay_ms=5.0,
            state_pred=True, dq_mode='meanvel')
    elif args.set == 'dsweep':
        run('D1 lat(act0) model', latency_comp=True, act_delay_ms=0.0)
        run('D3 lat(act10) model', latency_comp=True, act_delay_ms=10.0)
        run('D4 lat(act5) meanvel', latency_comp=True, act_delay_ms=5.0,
            dq_mode='meanvel')
        run('D5 lat(act5) model ffc0.94', latency_comp=True, act_delay_ms=5.0,
            dq_ff_comp=0.94)
        run('D6 plant_amax20 only', plant_amax=20.0)
        run('D7 lat(act5) + plant_amax20', latency_comp=True, act_delay_ms=5.0,
            plant_amax=20.0)
        run('D8 lat(act5) + plant v3/a20 + mpc v2/a15',
            latency_comp=True, act_delay_ms=5.0,
            plant_vmax=3.0, plant_amax=20.0, mpc_vmax=2.0, mpc_amax=15.0)
        run('D9 D8 + meanvel', latency_comp=True, act_delay_ms=5.0,
            plant_vmax=3.0, plant_amax=20.0, mpc_vmax=2.0, mpc_amax=15.0,
            dq_mode='meanvel')
    elif args.set == 'esweep':
        common = dict(latency_comp=True, act_delay_ms=5.0, smith=True,
                      invert=True)
        run('E1 smith+invert, plant v3/a20, mpc v2/a15',
            plant_vmax=3.0, plant_amax=20.0, mpc_vmax=2.0, mpc_amax=15.0,
            **common)
        run('E2 smith+invert, stock limits v1/a5', **common)
        run('E3 = E1 + jitter3ms/phase3ms/tau+20%',
            plant_vmax=3.0, plant_amax=20.0, mpc_vmax=2.0, mpc_amax=15.0,
            pub_jitter_ms=3.0, phase_err_ms=3.0, tau_true=0.18, **common)
        run('E4 smith only (no invert), raised limits',
            plant_vmax=3.0, plant_amax=20.0, mpc_vmax=2.0, mpc_amax=15.0,
            latency_comp=True, act_delay_ms=5.0, smith=True)
    elif args.set == 'uncertainty':
        # 生产等效配置(放开限幅+Smith+逆变换) × 真值/信念/估计器网格。
        prod = dict(plant_vmax=3.0, plant_amax=20.0, mpc_vmax=2.0,
                    mpc_amax=15.0, latency_comp=True, act_delay_ms=5.0,
                    smith=True, invert=True, steady_sec=10.0)
        AMP = 0.010
        run('U0 amp=0 (rig sanity)', **prod)
        run('U1 amp=10mm belief=truth', dist_amp=AMP, belief='truth', **prod)
        run('U2 amp=10mm belief=nominal', dist_amp=AMP, belief='nominal',
            **prod)
        for noise_mm, delay_ms in [(0, 0), (2, 0), (5, 0), (0, 40), (0, 100),
                                   (2, 40), (5, 40), (2, 100), (5, 100)]:
            run(f'U3 est σ={noise_mm}mm delay={delay_ms}ms lpf=10Hz',
                dist_amp=AMP, belief='estimated',
                est_noise_m=noise_mm * 1e-3, est_delay_ms=float(delay_ms),
                est_lpf_hz=10.0, **prod)
    elif args.set == 'model':
        # 同 uncertainty 集的 U2/U3(σ=2mm, delay=40ms) 配置, 加跑 belief='model',
        # 四线对比: nominal / const-est / model / truth。
        import json
        prod = dict(plant_vmax=3.0, plant_amax=20.0, mpc_vmax=2.0,
                    mpc_amax=15.0, latency_comp=True, act_delay_ms=5.0,
                    smith=True, invert=True, steady_sec=10.0)
        AMP, NOISE_M, DELAY_MS = 0.010, 2e-3, 40.0
        r_nom = run('M0 amp=10mm belief=nominal', dist_amp=AMP,
                    belief='nominal', **prod)
        r_tru = run('M1 amp=10mm belief=truth', dist_amp=AMP,
                    belief='truth', **prod)
        r_est = run('M2 const-est σ=2mm delay=40ms lpf=10Hz', dist_amp=AMP,
                    belief='estimated', est_noise_m=NOISE_M,
                    est_delay_ms=DELAY_MS, est_lpf_hz=10.0, **prod)
        r_mod = run('M3 model σ=2mm delay=40ms', dist_amp=AMP,
                    belief='model', est_noise_m=NOISE_M,
                    est_delay_ms=DELAY_MS, **prod)
        print('\n===== 四线对比 (steady RMS, mm) =====', flush=True)
        for name, r in (('nominal  ', r_nom), ('const-est', r_est),
                        ('model    ', r_mod), ('truth    ', r_tru)):
            print(f'{name} total={r["total"]:.3f} p95={r["p95"]:.3f} '
                  f'max={r["max"]:.3f}', flush=True)
        print('RESULT_JSON ' + json.dumps({
            'model_rms_mm': round(r_mod['total'], 4),
            'const_rms_mm': round(r_est['total'], 4),
            'nominal_rms_mm': round(r_nom['total'], 4),
            'truth_rms_mm': round(r_tru['total'], 4),
            'delta_p95_mm': round(r_mod['delta_p95'], 4),
        }), flush=True)
    elif args.set == 'limits':
        run('C1 raised limits only', plant_vmax=3.0, plant_amax=20.0,
            mpc_vmax=3.0, mpc_amax=20.0)
        run('C2 phase+limits', plant_vmax=3.0, plant_amax=20.0,
            mpc_vmax=3.0, mpc_amax=20.0,
            latency_comp=True, act_delay_ms=5.0, state_pred=True)
        run('C3 phase+limits+meanvel', plant_vmax=3.0, plant_amax=20.0,
            mpc_vmax=3.0, mpc_amax=20.0,
            latency_comp=True, act_delay_ms=5.0, state_pred=True,
            dq_mode='meanvel')
