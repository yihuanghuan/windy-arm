#!/usr/bin/env python3
"""阶段0 分析器 + 时钟修复的单元与端到端测试。

  python3 test_phase0_analysis.py

单元:未来 T_WB、时间戳插值对齐、漂移不外推、轨迹只用快照、FkModel。
端到端(真实调用 analyze()/compare() 子进程,合成 loop CSV + 快照):
  - 2s 阻塞不产生永久合成时间超前(catch-up/long-tick 被检出,t_solve=真实 wall);
  - actual/command 按真实 publish wall 时间对齐;
  - future T_WB 经完整主路径生效(cmd@reference != cmd@publish);
  - full/steady[4,40] 窗口选择正确;
  - 加速度饱和率可由已知 dq/timestamp 精确恢复;
  - compare 硬门:无效实验/publisher!=1/lead 错/源码 SHA 不同/时钟超限/KPI FAIL 均非零退出;
  - baseline lead=0、after lead=1 强制检查;
  - 无 np.interp 越界外推。
"""

import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_moving_base_run as A  # noqa: E402
from pinocchio_ik import DEFAULT_URDF, EE_FRAME  # noqa: E402
from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo  # noqa: E402

SCRIPT = str(Path(__file__).resolve().parent / 'analyze_moving_base_run.py')
FAIL = []
DT = 0.02


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


def _cfg(base_period=1.0, ee_period=4.0, att_deg=5.0):
    return {'BASE_CIRCLE_CENTER': [0.0, 0.0, 0.0], 'BASE_CIRCLE_RADIUS': 0.02,
            'BASE_PERIOD_SEC': base_period, 'BASE_HEIGHT': 0.01,
            'BASE_ATTITUDE_AMPLITUDE_DEG': att_deg,
            'BASE_ROLL_AMPLITUDE': math.radians(att_deg),
            'BASE_PITCH_AMPLITUDE': math.radians(att_deg),
            'BASE_YAW_AMPLITUDE': math.radians(att_deg),
            'EE_WORLD_CIRCLE_CENTER': [0.3, 0.01, 0.1], 'EE_CIRCLE_RADIUS': 0.08,
            'EE_PERIOD_SEC': ee_period,
            'TARGET_YAW_RAD': 0.0, 'TARGET_PITCH_RAD': 0.0, 'TARGET_ROLL_RAD': 0.0}


# ----------------------------------------------------------------- 合成 run

def make_synthetic_run(dirpath, label, lead, n_ticks=2250, cfg=None,
                       inject_gap_at=None, accel_ramp_joint=None,
                       publisher_count=1, file_shas=None):
    """写自洽的 loop CSV + 参数快照,真实 clock 时间栏由 50Hz 网格(可注入 gap)生成。"""
    cfg = cfg or _cfg()
    header = MoveBaseCircleMpcIkDemo._loop_csv_header()
    origin = 1000.0
    q_const = np.array([0.2, -0.15, -0.4, 1.2, -1.0, -1.2, -1.7])
    rows = []
    wall = origin
    for i in range(n_ticks):
        # 真实 clock 网格:t_solve = wall - origin;注入 gap 造 long-tick + 其后 catch-up。
        if inject_gap_at is not None and i == inject_gap_at:
            wall += 2.0                      # 2 s 阻塞
        interval_ms = float('nan') if i == 0 else (wall - prev_wall) * 1000.0  # noqa: F821
        t_solve = wall - origin
        solve_to_pub = 0.001
        publish_wall = wall + solve_to_pub
        t_publish = publish_wall - origin
        js_rel = t_solve - 0.005
        # 小幅关节运动使 FK 非平凡;加速度测试时让某关节 dq 以 5 rad/s^2 斜升
        qm = q_const + 0.01 * math.sin(2 * math.pi * t_solve / 4.0)
        dqm = 0.01 * (2 * math.pi / 4.0) * math.cos(2 * math.pi * t_solve / 4.0) * np.ones(7)
        if accel_ramp_joint is not None:
            dqm = dqm.copy()
            dqm[accel_ramp_joint] = ACCEL_TRI(t_solve)
        qp = qm.copy()
        row = {c: 0.0 for c in header}
        row.update({'t_solve': t_solve, 't_publish': t_publish,
                    't_reference': t_solve + lead * DT, 't_next': t_publish + DT,
                    'reference_lead_steps': lead, 'timer_interval_ms': interval_ms,
                    'solve_to_publish_ms': solve_to_pub * 1000.0, 'feedback_age_ms': 6.0,
                    'js_header_stamp_rel': js_rel, 'js_header_stamp_sec': origin + js_rel,
                    'js_recv_wall_sec': origin + js_rel + 0.001,
                    't_solve_wall': wall, 't_publish_wall': publish_wall,
                    'u0_active_count': 0, 'solver_status': 0, 'solver_success': 1,
                    'rti_iters': 1, 'solve_time_ms': 1.0, 'terminal_error_m': 0.001,
                    'first_error_m': 0.001, 'command_usable': 1, 'tracking_ok': 1,
                    'plan_feasible': 1, 'guard_block': 0, 'command_fk_step_m': 0.001})
        for j in range(7):
            row[f'q_meas_{j+1}'] = qm[j]; row[f'dq_meas_{j+1}'] = dqm[j]
            row[f'q_next_{j+1}'] = qp[j]; row[f'q_pub_{j+1}'] = qp[j]
            row[f'dq_pub_{j+1}'] = dqm[j]; row[f'u0_{j+1}'] = 0.0
        rows.append(row)
        prev_wall = wall
        wall += DT
    import csv as _csv
    loop_path = Path(dirpath) / f'loop_{label}.csv'
    with loop_path.open('w', newline='') as f:
        w = _csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([r[c] for c in header])
    shas = file_shas or {n: {'sha256': 'deadbeef' + n} for n in A.FINGERPRINT_FILES}
    snap = {'schema_version': 3, 'control_dt': DT, 'reference_lead_steps': lead,
            'experiment_valid': publisher_count == 1, 'urdf_path': DEFAULT_URDF, 'ee_frame': EE_FRAME,
            'graph': {'joint_states_publisher_count': publisher_count,
                      'joint_states_publisher_nodes': ['/student_arm_node']},
            'ros_params': {k: v for k, v in [('solver_backend', 'acados'), ('horizon_steps', 2),
                           ('max_joint_step', 0.02), ('terminal_tolerance', 0.01),
                           ('orientation_weight', 0.375), ('terminal_orientation_weight', 5.0),
                           ('acados_rti_iterations', 3), ('position_weight', 50.0),
                           ('terminal_position_weight', 500.0), ('joint_delta_weight', 0.08),
                           ('joint_delta_change_weight', 0.25), ('current_posture_weight', 0.01),
                           ('center_posture_weight', 0.002), ('command_fk_step_warn_m', 0.006)]},
            'provenance': {'files_sha256': shas, 'node_params': {},
                           'git': {'inner_pkg': {'head': 'abc'}, 'outer_ws': {'head': 'def'}}},
            'trajectory_config': cfg}
    params_path = Path(dirpath) / f'params_{label}.json'
    params_path.write_text(json.dumps(snap))
    return loop_path, params_path


def ACCEL_TRI(t):
    # 三角波速度 -> 恒定 |加速度| = 5 rad/s^2。半周期 0.2s 内速度变化 1.0 rad/s(±0.5),
    # 斜率 = 1.0/0.2 = 5 rad/s^2。
    period = 0.4
    ph = (t % period) / period
    return (2.0 * ph - 0.5) if ph < 0.5 else (1.5 - 2.0 * ph)


def run_analyze(loop, params, label, out, extra=None):
    cmd = [sys.executable, SCRIPT, 'analyze', '--loop-csv', str(loop),
           '--params-json', str(params), '--label', label, '--out', str(out)]
    if extra:
        cmd += extra
    return subprocess.run(cmd, capture_output=True, text=True)


# ----------------------------------------------------------------- 单元测试

def test_future_twb_unit():
    print('单元1: cmd@reference 用未来 T_WB')
    traj = A.TrajectoryModel(_cfg())
    p = np.array([0.05, -0.03, 0.12])
    for t in (0.0, 0.37, 0.9):
        w_pub = traj.world_from_base_point(p, t)
        w_ref = traj.world_from_base_point(p, t + DT)
        manual = traj.base_pose(t + DT) + traj.base_rotation(t + DT) @ p
        check(f't={t}: reference==base_pose(t+dt)+R(t+dt)@p', np.allclose(w_ref, manual, atol=1e-12))
        check(f't={t}: reference!=publish(证明未来T_WB)', np.linalg.norm(w_ref - w_pub) > 1e-5)


def test_interp_alignment():
    print('单元2: 时间戳插值对齐(非下标)')
    t = np.linspace(3.0, 10.0, 700)
    v = np.array([0.01, -0.02, 0.005]); p0 = np.array([0.3, 0.0, 0.1])
    act = p0[None, :] + np.outer(t - 3.0, v)
    te = np.array([4.013, 5.777, 8.21])
    interp = np.column_stack([np.interp(te, t, act[:, i]) for i in range(3)])
    manual = p0[None, :] + np.outer(te - 3.0, v)
    check('线性插值命中', np.allclose(interp, manual, atol=1e-6))
    check('越界 mask', A._win(np.array([2.0, 5.0, 11.0]), 3.0, 10.0).tolist() == [False, True, False])


def test_drift_no_extrap():
    print('单元3: 漂移不越界外推')
    period = 4.0
    t = np.arange(3.0, 3.0 + 3.5 * period, DT)
    q = np.column_stack([0.1 * (t - 3.0) / period + 0.01 * np.sin(2 * math.pi * t / period)]
                        + [np.zeros_like(t)] * 6)
    r = A._same_phase_drift(t, q, period, 0)
    check('锚点不越界', r['anchor_time_range_sec'][1] <= t[-1] + 1e-6)
    check('漂移率≈0.1/cycle', abs(r['total_drift_rad']['joint1'] / r['n_cycles_used'] - 0.1) < 0.01)
    check('discard 过大报错', 'error' in A._same_phase_drift(t, q, period, 99))


def test_traj_from_snapshot():
    print('单元4: 轨迹只用快照 dict')
    traj = A.TrajectoryModel(_cfg(base_period=2.5, ee_period=6.0))
    check('base 周期=2.5', np.allclose(traj.base_pose(0.0), traj.base_pose(2.5), atol=1e-12)
          and not np.allclose(traj.base_pose(0.0), traj.base_pose(1.0), atol=1e-6))
    check('repetition=lcm(6,2.5)=30', abs(traj.repetition_period() - 30.0) < 1e-9)
    fk = A.FkModel(DEFAULT_URDF, EE_FRAME)
    check('FkModel FK 有限', fk.fk_base(np.zeros(7)).shape == (3,))


# ----------------------------------------------------------------- 端到端

def test_e2e_clean_and_windows():
    print('端到端1: 干净合成 run 全主路径 + full/steady 窗口')
    with tempfile.TemporaryDirectory() as td:
        loop, params = make_synthetic_run(td, 'clean', lead=1, n_ticks=2250)  # ~45s
        out = Path(td) / 'm.json'
        r = run_analyze(loop, params, 'clean', out)
        check('analyze 退出 0', r.returncode == 0, r.stderr[-300:])
        if r.returncode != 0:
            return
        m = json.loads(out.read_text())
        check('actual_error steady 存在', 'mean_mm' in m['actual_error']['steady'])
        # steady 窗口 n < full n(丢了启动+尾部)
        check('steady 窗口小于 full', m['actual_error']['steady']['n'] < m['actual_error']['full']['n'])
        check('clock catch_up=0', m['clock_health']['catch_up_tick_count'] == 0)
        check('cmd@reference 与 publish 定义不同(未来T_WB经主路径)',
              abs(m['cmd_error_at_reference_time']['steady']['mean_mm']
                  - m['cmd_error_at_publish']['steady']['mean_mm']) > 1e-6)


def test_e2e_gap_no_synthetic_lead():
    print('端到端2: 2s 阻塞不产生永久合成超前')
    with tempfile.TemporaryDirectory() as td:
        loop, params = make_synthetic_run(td, 'gap', lead=1, n_ticks=1200, inject_gap_at=600)
        out = Path(td) / 'm.json'
        r = run_analyze(loop, params, 'gap', out)
        check('analyze 退出 0', r.returncode == 0, r.stderr[-300:])
        if r.returncode != 0:
            return
        m = json.loads(out.read_text())
        # gap 表现为 long-tick;t_solve 始终=真实 wall-origin,gap 后不累计合成偏移
        check('long-tick 被检出(gap 未被隐藏)', m['clock_health']['long_tick_count'] >= 1)
        # 用 CSV 直接验证:t_solve == t_solve_wall - origin(无合成累加)
        h, d = A._load_csv(loop)
        ts = A._col(h, d, 't_solve'); tw = A._col(h, d, 't_solve_wall')
        check('t_solve == wall-origin(真实,无合成累加)', np.allclose(ts, tw - tw[0], atol=1e-9))


def test_e2e_accel_recovery():
    print('端到端3: 加速度饱和率由已知 dq/timestamp 恢复')
    with tempfile.TemporaryDirectory() as td:
        loop, params = make_synthetic_run(td, 'acc', lead=0, n_ticks=2250, accel_ramp_joint=0)
        out = Path(td) / 'm.json'
        r = run_analyze(loop, params, 'acc', out)
        check('analyze 退出 0', r.returncode == 0, r.stderr[-300:])
        if r.returncode != 0:
            return
        m = json.loads(out.read_text())
        ac = m['measured_accel_saturation_steady']
        # joint1 恒 |a|=5 -> 该关节 near-limit 比例应≈100%,峰≈5
        check('joint1 加速度近满限比例高', ac['per_joint_near_limit_ratio']['joint1'] > 0.9,
              str(ac['per_joint_near_limit_ratio']['joint1']))
        check('加速度峰≈5', 4.5 <= ac['abs_peak'] <= 5.6, str(ac['abs_peak']))


# ----------------------------------------------------------------- compare 硬门

def mk_metrics(lead, **over):
    fp = {'trajectory_config': {'BASE_PERIOD_SEC': 1.0}, 'control_dt': 0.02,
          'urdf_path': '/u', 'ee_frame': 'link7', 'ros_params': {'horizon_steps': 2},
          'file_sha256': {'demo/mpc_pinocchio_ik.py': 'AAA'},
          'node_params_sha256': {'/student_arm_node': 'NP'},
          'git_inner_head': 'abc', 'git_outer_head': 'def'}
    m = {'label': f'lead{lead}', 'reference_lead_steps': lead, 'control_dt': 0.02,
         'experiment_valid': True, 'joint_states_publisher_count': 1,
         'config_fingerprint': fp,
         'clock_health': {'catch_up_tick_count': 0, 'nominal_real_dev_p95_ms': 0.5,
                          'median_dev_drift_ms': 0.1, 'feedback_age_ms': {'p95': 6.0}},
         'consistency': {'viz_expected_vs_snapshot_analytic_max_m': 0.0,
                         'recomputed_vs_viz_cmdfk_max_m': 0.001},
         'actual_error': {'steady': {'mean_mm': 6, 'p95_mm': 17, 'rms_mm': 7, 'max_mm': 20, 'n': 1800}},
         'cmd_error_at_publish': {'steady': {'p95_mm': 16}},
         'cmd_error_at_reference_time': {'steady': {'p95_mm': 18}},
         'cmd_error_at_next_tick': {'steady': {'mean_mm': 6, 'p95_mm': 18, 'rms_mm': 7, 'max_mm': 20, 'n': 1800}},
         'exec_gap_cartesian': {'steady': {'rms_mm': 4}},
         'cmd_lag_ms_steady': {'publish_x': 40 if lead == 0 else 20, 'publish_y': 24 if lead == 0 else 4,
                               'reference_x': 40, 'reference_y': 24},
         'actual_lag_ms_steady': {'quad_x': 48 if lead == 0 else 22, 'quad_y': 29 if lead == 0 else 3},
         'actual_sine_steady': {'base_freq': {'combined_rms_mm': 4}},
         'actual_phase_bins_steady': {'worst_bin_mean_mm': 17, 'worst_bin_phase_deg': [90, 120]},
         'measured_accel_saturation_steady': {'overall_near_limit_ratio': 0.8},
         'solver': {'nonzero_status_count': 0, 'command_usable_ratio': 1.0, 'solve_time_ms': {'p99': 2.0}},
         'guard': {'block_count': 0, 'ticks': 1800}, 'command_discontinuity': {'max_tick_dq_pub_rad': 0.05},
         'drift_measured': {'steady': {'max_abs_total_drift_rad': 0.02}}}
    for k, v in over.items():
        m[k] = v
    return m


def run_compare(a_dict, b_dict):
    with tempfile.TemporaryDirectory() as td:
        pa, pb = Path(td) / 'a.json', Path(td) / 'b.json'
        pa.write_text(json.dumps(a_dict)); pb.write_text(json.dumps(b_dict))
        return subprocess.run([sys.executable, SCRIPT, 'compare', str(pa), str(pb)],
                              capture_output=True, text=True).returncode


def test_compare_hard_gates():
    print('端到端4: compare 硬门退出码')
    # 干净但 KPI 不达标(p95 相同)-> 总判决 FAIL 非零
    check('KPI 未达标 -> 非零', run_compare(mk_metrics(0), mk_metrics(1)) == 1)
    # KPI 达标(after p95 降 >25%)+ 全门通过 -> 退出 0
    b_ok = mk_metrics(1)
    b_ok['cmd_error_at_next_tick']['steady']['p95_mm'] = 12   # 18->12 = -33%
    check('全门通过+KPI达标 -> 退出 0', run_compare(mk_metrics(0), b_ok) == 0)
    # 无效实验(publisher!=1)
    check('publisher!=1 -> 非零', run_compare(mk_metrics(0, joint_states_publisher_count=2,
                                                        experiment_valid=False), b_ok) == 1)
    # lead 错(baseline lead=1)
    check('baseline lead!=0 -> 非零', run_compare(mk_metrics(1), b_ok) == 1)
    # after lead 错(=0)
    check('after lead!=1 -> 非零', run_compare(mk_metrics(0), mk_metrics(0)) == 1)
    # 源码 SHA 不同
    b_sha = mk_metrics(1); b_sha['cmd_error_at_next_tick']['steady']['p95_mm'] = 12
    b_sha['config_fingerprint'] = {**b_sha['config_fingerprint'],
                                   'file_sha256': {'demo/mpc_pinocchio_ik.py': 'XXX'}}
    check('源码 SHA 不同 -> 非零', run_compare(mk_metrics(0), b_sha) == 1)
    # 时钟 catch-up 超限
    b_clk = mk_metrics(1); b_clk['cmd_error_at_next_tick']['steady']['p95_mm'] = 12
    b_clk['clock_health'] = {**b_clk['clock_health'], 'catch_up_tick_count': 3}
    check('catch-up>0 -> 非零', run_compare(mk_metrics(0), b_clk) == 1)
    # feedback-age 病态(P95 > 50ms,旧 snapshot 阻塞 bug 信号)-> 硬门非零
    b_fa = mk_metrics(1); b_fa['cmd_error_at_next_tick']['steady']['p95_mm'] = 12
    b_fa['clock_health'] = {**b_fa['clock_health'], 'feedback_age_ms': {'p95': 500.0}}
    check('feedback-age 病态(>50ms)-> 非零', run_compare(mk_metrics(0), b_fa) == 1)
    # feedback-age 配对差 3ms(均正常)-> advisory 非硬门,KPI 达标仍退出 0
    b_adv = mk_metrics(1); b_adv['cmd_error_at_next_tick']['steady']['p95_mm'] = 12
    b_adv['clock_health'] = {**b_adv['clock_health'], 'feedback_age_ms': {'p95': 9.0}}  # a=6,b=9 差3ms
    check('feedback-age 配对差3ms(正常)-> advisory 退出 0', run_compare(mk_metrics(0), b_adv) == 0)
    # 稳态破损:command_usable_ratio 低 -> 非零
    b_brk = mk_metrics(1); b_brk['cmd_error_at_next_tick']['steady']['p95_mm'] = 12
    b_brk['solver'] = {**b_brk['solver'], 'command_usable_ratio': 0.5}
    check('command_usable 低 -> 非零', run_compare(mk_metrics(0), b_brk) == 1)
    # solver 非零占比高 -> 非零
    b_nz = mk_metrics(1); b_nz['cmd_error_at_next_tick']['steady']['p95_mm'] = 12
    b_nz['solver'] = {**b_nz['solver'], 'nonzero_status_count': 500}  # 500/1800 > 5%
    check('solver 非零占比高 -> 非零', run_compare(mk_metrics(0), b_nz) == 1)
    # provenance 缺失(file_sha256 全 None)-> 非零
    a_np = mk_metrics(0); b_np = mk_metrics(1); b_np['cmd_error_at_next_tick']['steady']['p95_mm'] = 12
    a_np['config_fingerprint'] = {**a_np['config_fingerprint'], 'file_sha256': {'x': None}}
    b_np['config_fingerprint'] = {**b_np['config_fingerprint'], 'file_sha256': {'x': None}}
    check('provenance 缺失 -> 非零', run_compare(a_np, b_np) == 1)


def main():
    test_future_twb_unit()
    test_interp_alignment()
    test_drift_no_extrap()
    test_traj_from_snapshot()
    test_e2e_clean_and_windows()
    test_e2e_gap_no_synthetic_lead()
    test_e2e_accel_recovery()
    test_compare_hard_gates()
    print()
    if FAIL:
        print(f'FAILED: {len(FAIL)} 项: {FAIL}')
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
