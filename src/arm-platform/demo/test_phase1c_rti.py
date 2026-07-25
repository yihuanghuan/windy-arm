#!/usr/bin/env python3
"""阶段1C RTI 早退判据解耦的单元与 ACADOS 测试。

  python3 test_phase1c_rti.py [--skip-solver]

覆盖(Codex 六节要求):
1. mock solver status=0 且首轮 terminal_error 已低于 success_tolerance:
   legacy 必须 nit=1,fixed 必须 nit=3;
2. 第二轮返回非零 status:fixed 必须在第二轮停止(nit=2);
3. acados_rti_iterations=1 时两模式一致(nit=1);
4. 两模式不改变 OCP 配置签名(_acados_solver_signature 相等,且不含 rti 字段);
5. 真实 ACADOS smoke:fixed 跑满 nit=configured,status==0、command_usable。
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import moving_base_trajectory_config as traj  # noqa: E402

DT = 0.02
FAIL = []
BUILD_DIR = '/tmp/windylab_acados_phase1c_test'


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


class ScriptSolver:
    """脚本化 status 的 mock solver:solve() 按序列返回 status;get() 回读 set 的 x/u。"""

    def __init__(self, status_seq):
        self.status_seq = list(status_seq)
        self.calls = 0
        self.x, self.u = {}, {}

    def set(self, stage, key, val):
        v = np.asarray(val, dtype=float).copy()
        if key == 'x':
            self.x[stage] = v
        elif key == 'u':
            self.u[stage] = v

    def get(self, stage, key):
        return (self.x if key == 'x' else self.u)[stage]

    def solve(self):
        s = self.status_seq[min(self.calls, len(self.status_seq) - 1)]
        self.calls += 1
        return int(s)


def _run_rti(mode, status_seq, rti_iters=3, success_tol=1e9):
    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
    ik = MpcPinocchioIK(config=MpcIkConfig(
        horizon_steps=2, control_dt=DT, max_joint_step=0.02, success_tolerance=success_tol,
        solver_backend='acados', acados_rti_iterations=rti_iters,
        rti_iteration_mode=mode, acados_build_dir=BUILD_DIR))
    ik._acados_solver = ScriptSolver(status_seq)         # 注入 mock,绕过真实 acados
    q0 = np.array([0.10, -0.15, -0.40, 1.20, -1.00, -1.20, -1.70])
    targets = np.repeat(np.array([[0.30, -0.05, 0.12]]), 2, axis=0)
    rots = np.repeat(traj.TARGET_ROTATION.reshape(1, 3, 3), 2, axis=0)
    initial_plan = np.array([q0 + 0.010, q0 + 0.018])
    return ik._optimize_acados(q0, targets, rots, initial_plan, previous_joint_delta=np.zeros(7))


def test_mock_rti_modes():
    print('单元1: mock RTI 早退语义(legacy vs fixed)')
    # success_tolerance 巨大 -> 任何 terminal_error 都 <= 阈值;status 恒 0
    rl = _run_rti('legacy_early_exit', [0, 0, 0], rti_iters=3, success_tol=1e9)
    rf = _run_rti('fixed_iterations', [0, 0, 0], rti_iters=3, success_tol=1e9)
    check('legacy: 首轮误差已达标 -> nit=1 且 early_exit', rl.nit == 1 and rl.rti_early_exit is True,
          f'nit={rl.nit}')
    check('fixed: status0 跑满 -> nit=3 且非 early_exit', rf.nit == 3 and rf.rti_early_exit is False,
          f'nit={rf.nit}')
    check('fixed 记录每轮 status', rf.rti_status_per_iter == [0, 0, 0])

    # 第二轮非零 status:fixed 必须在第二轮停止
    rf2 = _run_rti('fixed_iterations', [0, 5, 0], rti_iters=3, success_tol=1e9)
    check('fixed: 第二轮 status!=0 -> nit=2 停止', rf2.nit == 2 and rf2.rti_status_per_iter == [0, 5],
          f'nit={rf2.nit}, status={rf2.rti_status_per_iter}')

    # acados_rti_iterations=1:两模式一致
    r1l = _run_rti('legacy_early_exit', [0], rti_iters=1, success_tol=1e9)
    r1f = _run_rti('fixed_iterations', [0], rti_iters=1, success_tol=1e9)
    check('rti_iterations=1: 两模式 nit=1 一致', r1l.nit == 1 and r1f.nit == 1)

    # legacy 在误差未达标时也会跑满(success_tol 极小)
    rl_full = _run_rti('legacy_early_exit', [0, 0, 0], rti_iters=3, success_tol=1e-12)
    check('legacy: 误差未达标 -> 跑满 nit=3(解耦对照)', rl_full.nit == 3, f'nit={rl_full.nit}')


def test_signature_unchanged():
    print('单元2: 两模式不改变 OCP 配置签名(不触发 codegen)')
    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK, _acados_solver_signature
    cfg_kw = dict(horizon_steps=2, control_dt=DT, max_joint_step=0.02, solver_backend='acados',
                  acados_build_dir=BUILD_DIR)
    ik_l = MpcPinocchioIK(config=MpcIkConfig(rti_iteration_mode='legacy_early_exit', **cfg_kw))
    ik_f = MpcPinocchioIK(config=MpcIkConfig(rti_iteration_mode='fixed_iterations', **cfg_kw))
    nq = ik_l.nq
    sig_l = _acados_solver_signature(ik_l, 2 * nq, nq, 1, 0, 0, 0, 'm')
    sig_f = _acados_solver_signature(ik_f, 2 * nq, nq, 1, 0, 0, 0, 'm')
    check('legacy/fixed 签名逐字节相等', json.dumps(sig_l, sort_keys=True) == json.dumps(sig_f, sort_keys=True))
    check('签名不含 rti_iteration_mode / acados_rti_iterations',
          'rti_iteration_mode' not in json.dumps(sig_l) and 'rti_iterations' not in json.dumps(sig_l))


def test_real_acados_fixed():
    print('单元5: 真实 ACADOS fixed 模式跑满 + status==0 + command_usable')
    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
    from pinocchio_ik import PinocchioIK
    ikf = MpcPinocchioIK(config=MpcIkConfig(
        horizon_steps=2, control_dt=DT, max_joint_step=0.02, success_tolerance=0.01,
        solver_backend='acados', acados_rti_iterations=3, orientation_weight=0.375,
        terminal_orientation_weight=5.0, position_weight=50.0, terminal_position_weight=500.0,
        rti_iteration_mode='fixed_iterations', acados_build_dir=BUILD_DIR))
    rot = traj.TARGET_ROTATION
    p = np.array([0.30, -0.05, 0.12])
    q0, ok = PinocchioIK().solve(p, target_rotation=rot, rot_weight=0.2)
    check('起点 IK 可达', bool(ok))
    targets = np.repeat(p.reshape(1, 3), 2, axis=0)
    rots = np.repeat(rot.reshape(1, 3, 3), 2, axis=0)
    warm = None
    nits = []
    for i in range(20):
        qn, ok2, info = ikf.step(targets, q0, warm_start=warm, target_rotation=rots,
                                 previous_joint_delta=np.zeros(7))
        warm = info['trajectory']
        if i >= 3:
            check(f'tick{i}: status==0 & command_usable', int(info['status']) == 0
                  and bool(info['command_usable']))
            nits.append(int(info['nit']))
        q0 = qn
    # fixed 模式:status==0 时应跑满 configured=3
    check('fixed 真实运行 nit 全=3(status0 跑满)', all(n == 3 for n in nits),
          f'nit set={sorted(set(nits))}')
    # 对照:legacy 真实运行大概率出现 nit<3(早退存在)
    ikl = MpcPinocchioIK(config=MpcIkConfig(
        horizon_steps=2, control_dt=DT, max_joint_step=0.02, success_tolerance=0.01,
        solver_backend='acados', acados_rti_iterations=3, orientation_weight=0.375,
        terminal_orientation_weight=5.0, position_weight=50.0, terminal_position_weight=500.0,
        rti_iteration_mode='legacy_early_exit', acados_build_dir=BUILD_DIR))
    q0b, _ = PinocchioIK().solve(p, target_rotation=rot, rot_weight=0.2)
    warm = None
    legacy_nits = []
    for i in range(20):
        qn, ok2, info = ikl.step(targets, q0b, warm_start=warm, target_rotation=rots,
                                 previous_joint_delta=np.zeros(7))
        warm = info['trajectory']
        if i >= 3:
            legacy_nits.append(int(info['nit']))
        q0b = qn
    check('legacy 真实运行存在早退(nit<3 出现)', any(n < 3 for n in legacy_nits),
          f'legacy nit set={sorted(set(legacy_nits))}')


def test_config_validation():
    print('单元3: MpcPinocchioIK 层配置校验(非法值抛 ValueError,不静默修正)')
    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK

    def expect_valueerror(kw, name):
        try:
            MpcPinocchioIK(config=MpcIkConfig(solver_backend='acados',
                                              acados_build_dir=BUILD_DIR, **kw))
            check(name, False, '未抛 ValueError')
        except ValueError:
            check(name, True)
        except Exception as exc:
            check(name, False, f'抛了 {type(exc).__name__} 而非 ValueError')
    expect_valueerror({'rti_iteration_mode': 'fixed'}, "非法 rti_iteration_mode('fixed' 拼写)-> ValueError")
    expect_valueerror({'rti_iteration_mode': 'FIXED_ITERATIONS'}, '大小写不匹配 -> ValueError')
    expect_valueerror({'acados_rti_iterations': 0}, 'acados_rti_iterations=0 -> ValueError')
    expect_valueerror({'acados_rti_iterations': -3}, 'acados_rti_iterations=-3 -> ValueError')
    # 合法值不应抛
    try:
        MpcPinocchioIK(config=MpcIkConfig(solver_backend='acados', acados_build_dir=BUILD_DIR,
                                          rti_iteration_mode='fixed_iterations', acados_rti_iterations=1))
        check('合法 fixed_iterations + rti=1 不抛', True)
    except Exception as exc:
        check('合法 fixed_iterations + rti=1 不抛', False, str(exc))


def test_demo_production_defaults():
    print('单元4: moving-base demo 生产默认 = lead1 / measured_delta / fixed_iterations')
    import ast
    src = (Path(__file__).resolve().parent / 'move_base_circle_mpc_ik_demo.py').read_text()
    tree = ast.parse(src)
    defaults = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter' and len(node.args) >= 2
                and isinstance(node.args[0], ast.Constant)):
            val = node.args[1]
            if isinstance(val, ast.Constant):
                defaults[node.args[0].value] = val.value
    check("reference_lead_steps 默认=1", defaults.get('reference_lead_steps') == 1,
          str(defaults.get('reference_lead_steps')))
    check("dq_prev_mode 默认='measured_delta'", defaults.get('dq_prev_mode') == 'measured_delta',
          str(defaults.get('dq_prev_mode')))
    check("rti_iteration_mode 默认='fixed_iterations'",
          defaults.get('rti_iteration_mode') == 'fixed_iterations', str(defaults.get('rti_iteration_mode')))
    # MpcIkConfig 层默认应仍为 legacy(兼容其他 demo),由 moving-base demo 显式传 fixed。
    from mpc_pinocchio_ik import MpcIkConfig
    check("MpcIkConfig.rti_iteration_mode 默认仍=legacy(兼容其他 demo)",
          MpcIkConfig().rti_iteration_mode == 'legacy_early_exit')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skip-solver', action='store_true')
    args = ap.parse_args()
    test_mock_rti_modes()
    test_signature_unchanged()
    test_config_validation()
    test_demo_production_defaults()
    if args.skip_solver:
        print('单元3: 跳过(--skip-solver)')
    else:
        test_real_acados_fixed()
    print()
    if FAIL:
        print(f'FAILED: {len(FAIL)} 项: {FAIL}')
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
