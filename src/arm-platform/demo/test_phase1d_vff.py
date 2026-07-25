#!/usr/bin/env python3
"""阶段1D velocity_feedforward_gain A/B 的 compare 门单元测试(fabricated metrics)。

  python3 test_phase1d_vff.py

覆盖(Codex 六节要求):
- 仅 gain 不同(0.7->1.0)且其余全同 -> 识别为 Phase 1D 且通过(exit 0);
- position_feedback_tau / max_velocity / feedback_velocity_limit / max_acceleration 任一不同 -> HARD FAIL;
- 文件或构建 SHA 不同 -> HARD FAIL;
- gain 未真正改变(两组相同)-> FAIL(无唯一实验变量)。
"""

import json
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT = str(Path(__file__).resolve().parent / 'analyze_moving_base_run.py')
FAIL = []


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


EXEC6 = {'max_velocity': 1.0, 'enable_velocity_feedforward': True,
         'position_feedback_tau': 0.15, 'feedback_velocity_limit': 0.4,
         'enable_acceleration_limit': True, 'max_acceleration': 5.0}


def mk_1d(gain, exec6_over=None, file_sha='AAA', **over):
    exec6 = dict(EXEC6, **(exec6_over or {}))
    fp = {'trajectory_config': {'BASE_PERIOD_SEC': 1.0}, 'control_dt': 0.02,
          'urdf_path': '/u', 'ee_frame': 'link7', 'ros_params': {'horizon_steps': 2},
          'file_sha256': {'demo/mpc_pinocchio_ik.py': file_sha},
          'node_params_sha256': {'/trajectory_visualizer_demo': 'VZ'},
          'student_arm_key_params_except_gain': exec6,
          'git_inner_head': 'abc', 'git_outer_head': 'def'}
    m = {'label': f'gain{gain}', 'reference_lead_steps': 1, 'control_dt': 0.02,
         'dq_prev_mode': 'measured_delta', 'rti_iteration_mode': 'fixed_iterations',
         'velocity_feedforward_gain': gain,
         'executor_params': dict(exec6, velocity_feedforward_gain=gain),
         'experiment_valid': True, 'joint_states_publisher_count': 1,
         'config_fingerprint': fp,
         'clock_health': {'catch_up_tick_count': 0, 'nominal_real_dev_p95_ms': 0.5,
                          'median_dev_drift_ms': 0.1, 'feedback_age_ms': {'p95': 6.0}},
         'consistency': {'viz_expected_vs_snapshot_analytic_max_m': 0.0,
                         'recomputed_vs_viz_cmdfk_max_m': 0.001},
         'dq_prev': {'fallback_ratio': 0.0004},
         'rti': {'configured_iters': 3, 'nit_eq_configured_ratio_success_steady': 1.0},
         'actual_error': {'steady': {'mean_mm': 7, 'p95_mm': 19, 'rms_mm': 8, 'max_mm': 21, 'n': 1800}},
         'cmd_error_at_next_tick': {'steady': {'mean_mm': 6, 'p95_mm': 19, 'rms_mm': 7, 'max_mm': 20, 'n': 1800}},
         'actual_sine_steady': {'base_freq': {'combined_rms_mm': 4}},
         'actual_phase_bins_steady': {'worst_bin_mean_mm': 17, 'worst_bin_phase_deg': [90, 120]},
         'actual_lag_ms_steady': {'quad_x': 43, 'quad_y': 25},
         'exec_gap_cartesian': {'steady': {'rms_mm': 2.2}}, 'exec_gap_joint_rad': {'steady_rms': 0.008},
         'phase1d_executor_match': {
             'exec_gap_joint_rad': {'rms': 0.008, 'p95_norm': 0.02, 'max_abs': 0.03},
             'exec_gap_cartesian_mm': {'rms': 2.2, 'p95': 5.0, 'max': 7.0},
             'vel_gap_dqpub_vs_dqmeas_rad_s': {'rms': 0.05, 'p95_norm': 0.12},
             'dq_pub_near_max_velocity_ratio': 0.0, 'dq_pub_abs_max_rad_s': 0.78},
         'measured_vel_saturation_steady': {'abs_peak': 0.78, 'near_limit_ratio': 0.0},
         'measured_accel_saturation_steady': {'abs_peak': 5.05, 'overall_near_limit_ratio': 0.85},
         'solver': {'nonzero_status_count': 0, 'command_usable_ratio': 1.0, 'solve_time_ms': {'p99': 2.1}},
         'guard': {'block_count': 0, 'ticks': 1800},
         'command_discontinuity': {'max_tick_dq_pub_rad': 0.05},
         'drift_measured': {'steady': {'max_abs_total_drift_rad': 0.05}}}
    for k, v in over.items():
        m[k] = v
    return m


def run_compare(a, b):
    with tempfile.TemporaryDirectory() as td:
        pa, pb = Path(td) / 'a.json', Path(td) / 'b.json'
        pa.write_text(json.dumps(a)); pb.write_text(json.dumps(b))
        r = subprocess.run([sys.executable, SCRIPT, 'compare', str(pa), str(pb)],
                           capture_output=True, text=True)
        return r.returncode, r.stdout


def test_phase1d_gates():
    print('阶段1D compare 门:')
    # 仅 gain 0.7->1.0,其余全同 -> 识别为 1D 且通过
    rc, out = run_compare(mk_1d(0.7), mk_1d(1.0))
    check('仅 gain 不同 -> 识别为 Phase 1D', 'compare[Phase 1D]' in out)
    check('仅 gain 不同且安全 -> 退出 0', rc == 0, f'rc={rc}')
    # 执行器其余六项任一不同 -> HARD FAIL
    for p, val in [('position_feedback_tau', 0.20), ('max_velocity', 0.5),
                   ('feedback_velocity_limit', 0.3), ('max_acceleration', 8.0)]:
        rc2, out2 = run_compare(mk_1d(0.7), mk_1d(1.0, exec6_over={p: val}))
        check(f'{p} 不同 -> HARD FAIL 非零退出', rc2 == 1, f'rc={rc2}')
    # 文件 SHA 不同 -> HARD FAIL
    rc3, _ = run_compare(mk_1d(0.7, file_sha='AAA'), mk_1d(1.0, file_sha='ZZZ'))
    check('文件 SHA 不同 -> 非零退出', rc3 == 1, f'rc={rc3}')
    # gain 未真正改变(两组都 0.7)-> 无唯一变量 -> FAIL
    rc4, out4 = run_compare(mk_1d(0.7), mk_1d(0.7))
    check('gain 未改变 -> 非零退出(无唯一实验变量)', rc4 == 1 and '实验变量不唯一' in out4, f'rc={rc4}')
    # 基线 gain 必须=0.7、after 必须=1.0
    rc5, _ = run_compare(mk_1d(1.0), mk_1d(0.7))  # 方向反了
    check('baseline gain!=0.7 -> 非零退出', rc5 == 1, f'rc={rc5}')
    # 安全门:after actual P95 恶化 >2% -> FAIL
    bad = mk_1d(1.0); bad['actual_error']['steady']['p95_mm'] = 20  # 19->20 = +5.3%
    rc6, _ = run_compare(mk_1d(0.7), bad)
    check('actual P95 恶化>2% -> 非零退出', rc6 == 1, f'rc={rc6}')


def main():
    test_phase1d_gates()
    print()
    if FAIL:
        print(f'FAILED: {len(FAIL)} 项: {FAIL}')
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
