#!/usr/bin/env python3
"""二阶 NMPC × FeedforwardPlant(生产执行器律 0.7/τ=0.15/±0.4/±5)离线闭环。

目的: 复现真实低速 A/B 的二阶滞后(默认参数 P95≈50mm / 调后≈23mm;一阶≈1mm)。
复现成功 => "双积分器模型 vs 0.7 前馈执行器"失配假设坐实,后续模型修正全部离线迭代。

  python3 second_order_offline_loop.py [--config default|tuned] [--ticks-scale 1.0]
低速工况与真实 A/B 一致: EE 周期 8s(time_scale=2), 基座周期 4s(base_scale=4)。
"""

import argparse

import numpy as np

import tune_unified_mpc_params as T
from pinocchio_ik import PinocchioIK
from second_order_mpc_ik import SecondOrderMpcConfig, SecondOrderMpcIK

CONFIGS = {
    # 与 demo second_order_* 默认一致
    'default': dict(horizon_steps=10, position_weight=50.0, terminal_position_weight=500.0,
                    joint_velocity_weight=0.02, joint_acceleration_weight=0.01),
    # 真实 A/B 调后候选
    'tuned': dict(horizon_steps=20, position_weight=50000.0, terminal_position_weight=500.0,
                  joint_velocity_weight=0.02, joint_acceleration_weight=0.0),
}


def run(cfg_name: str, warmup_periods=1.0, steady_periods=2.0, dq_mode='model',
        base_period=4.0, ee_period=8.0, horizon=None, latency_ticks=0, lead=1,
        dq0_source='measured'):
    """dq_mode: model=发布 dq_next(现状) | invgain=dq_next/0.7(逆增益前馈补偿)
    | posdiff=(q_next-q_meas)/dt(一阶式过驱动差分)。"""
    sc = T.make_moving_base_scenario('ol', base_period_sec=base_period, ee_period_sec=ee_period)
    ctx = T.make_context(warmup_periods, steady_periods)
    q0, ok = PinocchioIK().solve(sc.position_fn(0.0), target_rotation=sc.rotation_fn(0.0),
                                 rot_weight=0.2)
    assert ok
    q0 = np.clip(q0, ctx.q_lower, ctx.q_upper)

    kw = dict(CONFIGS[cfg_name])
    if horizon is not None:
        kw['horizon_steps'] = horizon
    solver = SecondOrderMpcIK(config=SecondOrderMpcConfig(
        control_dt=T.CONTROL_DT, current_posture_weight=0.01, center_posture_weight=0.002,
        max_joint_velocity=1.0, max_joint_acceleration=5.0,
        min_translation_jacobian_sigma=0.05,
        acados_build_dir=f"/tmp/windylab_acados_so_offline_{cfg_name}_h{kw['horizon_steps']}", **kw))
    N = kw['horizon_steps']

    plant = T.FeedforwardPlant(q0, ctx.q_lower, ctx.q_upper)   # gain=0.7 生产
    eval_period = sc.eval_period()
    warm = int(round(warmup_periods * eval_period / T.CONTROL_DT))
    steady = int(round(steady_periods * eval_period / T.CONTROL_DT))
    warm_start = None
    cmd_queue = []
    last_cmd_dq = None
    errs, usable_fail, status_nz, pred_gap = [], 0, 0, []
    for k in range(warm + steady):
        t = k * T.CONTROL_DT
        q_meas = plant.q.copy()
        dq_meas = (last_cmd_dq.copy() if (dq0_source == 'commanded' and last_cmd_dq is not None)
                   else plant.v_last.copy())
        times = T.target_times(t, N, lead=lead)
        targets = np.array([sc.position_fn(float(ti)) for ti in times])
        rots = np.array([sc.rotation_fn(float(ti)) for ti in times])
        h = min(0.005, 0.5 * T.CONTROL_DT)
        target_velocities = np.array([
            (sc.position_fn(float(ti) + h) - sc.position_fn(float(ti) - h))
            / (2.0 * h)
            for ti in times
        ])
        target_accelerations = np.array([
            (
                sc.position_fn(float(ti) + h)
                - 2.0 * sc.position_fn(float(ti))
                + sc.position_fn(float(ti) - h)
            ) / (h * h)
            for ti in times
        ])
        q_next, dq_next, ok, info = solver.step(
            targets, q_meas, dq_meas, warm_start=warm_start,
            target_rotation=rots,
            target_velocity=target_velocities,
            target_acceleration=target_accelerations)
        warm_start = info.get('trajectory')
        if not info.get('command_usable', ok):
            usable_fail += 1
            q_next, dq_next = q_meas, np.zeros_like(q_meas)
        if info.get('status', 0) != 0:
            status_nz += 1
        if dq_mode == 'invgain':
            dq_pub = np.clip(dq_next / T.PLANT_FF_GAIN, -1.0, 1.0)
        elif dq_mode == 'posdiff':
            dq_pub = (q_next - q_meas) / T.CONTROL_DT
        else:
            dq_pub = dq_next
        # 时延建模: latency_ticks>0 时,本拍执行的是 latency_ticks 拍前算出的命令。
        if latency_ticks > 0:
            cmd_queue.append((q_next.copy(), dq_pub.copy()))
            q_apply, dq_apply = cmd_queue.pop(0) if len(cmd_queue) > latency_ticks else (q_meas, np.zeros_like(q_meas))
        else:
            q_apply, dq_apply = q_next, dq_pub
        plant.advance(q_apply, dq_apply)
        last_cmd_dq = dq_next.copy()
        # 真值误差: FK(plant.q) 在 world 对 world 目标(t+dt)
        p_act, _ = T.fk(ctx, plant.q)
        t1 = t + T.CONTROL_DT
        p_act_w = sc.base_pose_fn(t1) + sc.base_rot_fn(t1) @ p_act
        p_tgt_w = sc.base_pose_fn(t1) + sc.base_rot_fn(t1) @ sc.position_fn(t1)
        if k >= warm:
            errs.append(np.linalg.norm(p_act_w - p_tgt_w))
            pred_gap.append(np.max(np.abs(plant.v_last - dq_next)))   # 预测 dq vs 实测
    e = np.array(errs) * 1000.0
    print(f'[{cfg_name}/{dq_mode}/lat{latency_ticks}/lead{lead}/{dq0_source}] steady actual: mean={np.mean(e):.2f} rms={np.sqrt(np.mean(e**2)):.2f} '
          f'p95={np.percentile(e, 95):.2f} max={np.max(e):.2f} mm | '
          f'usable_fail={usable_fail} status_nz={status_nz} | '
          f'|dq_pred - dq_meas| p95={np.percentile(pred_gap, 95):.4f} rad/s')
    return e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', choices=('default', 'tuned', 'both'), default='both')
    args = ap.parse_args()
    ap2 = args
    names = ('default', 'tuned') if ap2.config == 'both' else (ap2.config,)
    print('真实低速 A/B 参照: 一阶 P95=1.01mm | 二阶默认 P95=50.29mm | 二阶调后 P95=22.56mm')
    for n in names:
        for mode in ('model', 'invgain', 'posdiff'):
            run(n, dq_mode=mode)


if __name__ == '__main__':
    main()
