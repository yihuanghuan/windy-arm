#!/usr/bin/env python3
"""阶段1B dq_prev 修正的单元与 ACADOS 测试。

  python3 test_phase1b_dq_prev.py [--skip-solver]

覆盖(Codex 六节要求):
1. previous_joint_delta=None 与显式 dq_prev=0 数值行为一致;
2. measured_delta 能根据 q/timestamp 正确恢复已知执行增量;
3. 非法时间戳/NaN/首帧正确回退 0(带原因码);
4. clip 不超过 max_joint_step;
5. ACADOS stage0 的 x0 后半段(dq_prev)等于传入值(经 lbx/ubx 固定);
6. 求解轨迹满足动力学一致性 q_{k+1}≈q_k+u_k, dq_prev_{k+1}≈u_k(warm-start 索引修正验证);
7. 真实 ACADOS:status==0、command_usable、x0 dq_prev==输入、配置指纹不变(不重新 codegen);
8. zero/measured 两模式目标时序都保持 lead=1。
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import moving_base_trajectory_config as traj  # noqa: E402

DT = 0.02
FAIL = []
BUILD_DIR = '/tmp/windylab_acados_phase1b_test'


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


# ----------------------------------------------------------------- 单元:_compute_dq_prev

def _stub(mode, dq_measured=None, prev_q=None, prev_js=float('nan')):
    from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo, JOINT_COUNT  # noqa
    s = object.__new__(MoveBaseCircleMpcIkDemo)
    s.dq_prev_mode = mode
    s.dt = DT
    s.max_joint_step = 0.02
    s.dq_measured = dq_measured
    s._prev_solve_q_measured = prev_q
    s._prev_solve_js_stamp = prev_js
    return s


def test_compute_dq_prev():
    print('单元1: _compute_dq_prev(measured_delta 恢复/回退/clip)')
    from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo as D
    q_prev = np.array([0.10, 0.20, -0.30, 0.40, -0.50, 0.60, -0.70])
    # 已知执行增量 0.005/关节,Δt_feedback=0.02=dt -> Δq_exec = 0.005
    q_now = q_prev + 0.005
    s = _stub('measured_delta', dq_measured=np.full(7, 0.25), prev_q=q_prev.copy(), prev_js=100.0)
    r = D._compute_dq_prev(s, q_now, 100.0 + 0.02)
    check('measured 恢复执行增量 0.005', np.allclose(r['input'], 0.005, atol=1e-9)
          and r['source'] == 1 and r['fallback'] == 0, str(r['input'][:2]))
    check('velocity_estimate = dq_meas*dt', np.allclose(r['velocity_estimate'], 0.25 * DT))
    # Δt_feedback != dt 时归一化: Δt=0.01, 位移 0.005 -> Δq_exec = 0.005*dt/0.01 = 0.01
    r2 = D._compute_dq_prev(s, q_now, 100.0 + 0.01)
    check('Δt≠dt 归一化到一个控制周期', np.allclose(r2['input'], 0.01, atol=1e-9), str(r2['input'][:2]))
    # clip 到 max_joint_step:大位移
    s3 = _stub('measured_delta', prev_q=q_prev.copy(), prev_js=100.0)
    r3 = D._compute_dq_prev(s3, q_prev + 0.5, 100.0 + 0.02)
    check('clip 不超过 max_joint_step', np.all(np.abs(r3['input']) <= 0.02 + 1e-12)
          and np.max(np.abs(r3['raw'])) > 0.02, f"max|input|={np.max(np.abs(r3['input'])):.4f}")
    # zero 模式恒零
    rz = D._compute_dq_prev(_stub('zero', dq_measured=np.full(7, 0.3)), q_now, 100.02)
    check('zero 模式 dq_prev=0 source=0', np.all(rz['input'] == 0) and rz['source'] == 0)


def test_compute_dq_prev_fallback():
    print('单元2: _compute_dq_prev 回退(首帧/非递增/过旧/NaN)')
    from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo as D
    q = np.zeros(7)
    # 首帧:无 prev
    r1 = D._compute_dq_prev(_stub('measured_delta', prev_q=None, prev_js=float('nan')), q, 100.0)
    check('首帧回退0 fallback=1', np.all(r1['input'] == 0) and r1['fallback'] == 1 and r1['source'] == 2)
    # 非递增时间戳(Δt<=0)
    r2 = D._compute_dq_prev(_stub('measured_delta', prev_q=q.copy(), prev_js=100.0), q, 99.99)
    check('非递增时间戳回退 fallback=2', np.all(r2['input'] == 0) and r2['fallback'] == 2)
    # 反馈过旧(Δt>5dt)
    r3 = D._compute_dq_prev(_stub('measured_delta', prev_q=q.copy(), prev_js=100.0), q, 100.0 + 0.5)
    check('反馈过旧回退 fallback=3', np.all(r3['input'] == 0) and r3['fallback'] == 3)
    # 当前 q 含 NaN
    r4 = D._compute_dq_prev(_stub('measured_delta', prev_q=q.copy(), prev_js=100.0),
                            np.array([np.nan] * 7), 100.02)
    check('q 含 NaN 回退 fallback=4', np.all(r4['input'] == 0) and r4['fallback'] == 4)


def _run_seq(mode, samples, prev_q=None, prev_js=float('nan'), dq_meas=None):
    """按状态机跑一串 (q, stamp) 样本:每拍算 dq_prev 并**按 update_anchor 条件更新锚点**
    (模拟真实调用方),返回每拍结果 + 每拍后的锚点快照。"""
    from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo as D
    s = _stub(mode, dq_measured=dq_meas, prev_q=prev_q, prev_js=prev_js)
    results, anchors = [], []
    for q, js in samples:
        r = D._compute_dq_prev(s, q, js)
        if r['update_anchor']:
            s._prev_solve_q_measured = r['anchor_q']
            s._prev_solve_js_stamp = float(r['anchor_stamp'])
        results.append(r)
        anchors.append((None if s._prev_solve_q_measured is None
                        else s._prev_solve_q_measured.copy(), s._prev_solve_js_stamp))
    return results, anchors


def test_history_recovery():
    print('单元2b: 历史锚点异常污染防护 + 恢复')
    q = lambda a: np.full(7, a)  # noqa: E731
    NaNq = np.full(7, np.nan)
    base = 100.0
    # valid -> NaN q -> valid:NaN 拍不写脏锚点,下一合法拍对旧锚点仍能恢复
    r, anch = _run_seq('measured_delta', [(q(0.1), base), (NaNq, base + 0.02), (q(0.11), base + 0.04)])
    check('NaN q: [fb1, fb4, ok]', [x['fallback'] for x in r] == [1, 4, 0]
          and r[2]['source'] == 1, str([x['fallback'] for x in r]))
    check('NaN q 拍不污染锚点(锚点保持首个合法样本)', np.allclose(anch[1][0], 0.1))
    # valid -> NaN stamp -> valid
    r2, _ = _run_seq('measured_delta', [(q(0.1), base), (q(0.2), np.nan), (q(0.11), base + 0.04)])
    check('NaN stamp: [fb1, fb4, ok]', [x['fallback'] for x in r2] == [1, 4, 0] and r2[2]['source'] == 1)
    # valid -> 非递增 stamp -> valid:不污染合法锚点
    r3, a3 = _run_seq('measured_delta', [(q(0.1), base), (q(0.2), base - 0.01), (q(0.11), base + 0.02)])
    check('非递增 stamp: [fb1, fb2, ok]', [x['fallback'] for x in r3] == [1, 2, 0] and r3[2]['source'] == 1)
    check('非递增拍不污染锚点', np.allclose(a3[1][0], 0.1) and a3[1][1] == base)
    # valid -> 过旧 stamp -> 下一正常拍恢复(重锚)
    r4, _ = _run_seq('measured_delta', [(q(0.1), base), (q(0.11), base + 0.02),
                                        (q(0.12), base + 0.02 + 0.5), (q(0.13), base + 0.02 + 0.52)])
    check('过旧 stamp: [fb1, ok, fb3, ok](重锚后恢复)',
          [x['fallback'] for x in r4] == [1, 0, 3, 0] and r4[3]['source'] == 1,
          str([x['fallback'] for x in r4]))
    # previous q 人为 NaN -> 回退1 并用当前合法样本重锚,下一拍恢复
    r5, _ = _run_seq('measured_delta', [(q(0.2), base + 0.02)], prev_q=NaNq.copy(), prev_js=base)
    check('prev q NaN -> fb1 重锚', r5[0]['fallback'] == 1 and r5[0]['update_anchor'] is True)
    # 所有 source==1 行 raw/input 全有限
    mixed = [(q(0.1), base), (NaNq, base + 0.02), (q(0.11), base + 0.04),
             (q(0.12), base - 5.0), (q(0.13), base + 0.06)]
    r6, _ = _run_seq('measured_delta', mixed)
    ok_rows = [x for x in r6 if x['source'] == 1]
    check('所有 measured_ok 行 raw/input 有限', len(ok_rows) >= 1 and all(
        np.all(np.isfinite(x['input'])) and np.all(np.isfinite(x['raw'])) for x in ok_rows))
    # 异常后**在随后收到连续合法样本的前提下**一拍即恢复(过旧序列 fb3 后紧跟合法样本 -> ok)。
    # 若异常持续则会持续回退,这里验证的是"提供连续合法样本后能恢复,不会永久 fallback"。
    fbs = [x['fallback'] for x in r4]
    check('异常后(续接连续合法样本)一拍即恢复,不永久 fallback',
          fbs[2] == 3 and fbs[3] == 0)


def test_lead_independent_of_mode():
    print('单元3: 目标时序 lead=1 与 dq_prev_mode 无关')
    from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo as D
    for mode in ('zero', 'measured_delta'):
        s = object.__new__(D)
        s.dt = DT; s.horizon_steps = 2; s.reference_lead_steps = 1
        pos, _ = D._target_horizon_base(s, 0.5)
        exp = traj.base_rotation(0.5 + DT).T @ (traj.ee_target_world(0.5 + DT) - traj.base_pose(0.5 + DT))
        check(f'{mode}: targets[0]=target(t+dt) (lead=1)', np.allclose(pos[0], exp, atol=1e-12))


# ----------------------------------------------------------------- capturing-solver 映射

class CapturingSolver:
    """记录所有 solver.set() 的 mock:直接断言 solve 前设置的 initial guess / x0,
    绕过 ACADOS(不 codegen)。get() 回读最后设置的 x/u 供 _optimize_acados 提取。"""

    def __init__(self):
        self.sets = []
        self.x, self.u, self.lbx, self.ubx = {}, {}, {}, {}

    def set(self, stage, key, val):
        v = np.asarray(val, dtype=float).copy()
        self.sets.append((stage, key, v))
        {'x': self.x, 'u': self.u, 'lbx': self.lbx, 'ubx': self.ubx}.get(key, {})[stage] = v \
            if key in ('x', 'u', 'lbx', 'ubx') else None

    def get(self, stage, key):
        return (self.x if key == 'x' else self.u)[stage]

    def solve(self):
        return 0


def _capture(ik, q0, initial_plan, prev_delta):
    from mpc_pinocchio_ik import _AcadosResult  # noqa: F401 (ensure module importable)
    cap = CapturingSolver()
    ik._acados_solver = cap                      # 注入 mock,阻止真实 acados 构建
    N = ik.config.horizon_steps
    targets = np.repeat(np.array([[0.30, -0.05, 0.12]]), N, axis=0)
    rots = np.repeat(traj.TARGET_ROTATION.reshape(1, 3, 3), N, axis=0)
    ik._optimize_acados(q0, targets, rots, initial_plan, previous_joint_delta=prev_delta)
    return cap


def test_warm_start_capture():
    print('单元5: capturing-solver 直接断言 warm-start / x0 映射(不经 ACADOS)')
    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
    # 惰性构造:不触发 acados codegen(_acados_solver 被 mock 覆盖)
    ik = MpcPinocchioIK(config=MpcIkConfig(horizon_steps=2, control_dt=DT, max_joint_step=0.02,
                                           solver_backend='acados', acados_build_dir=BUILD_DIR))
    nq, N = ik.nq, ik.config.horizon_steps
    q0 = np.array([0.10, -0.15, -0.40, 1.20, -1.00, -1.20, -1.70])
    # 步长均在 max_joint_step 内,便于精确断言(clip=恒等)
    initial_plan = np.array([q0 + 0.010, q0 + 0.010 + 0.008])   # step0=0.010, step1=0.008
    pd = np.array([0.004, -0.006, 0.008, -0.010, 0.012, -0.003, 0.005])
    cap = _capture(ik, q0, initial_plan, pd)

    u_guess0 = np.clip(initial_plan[0] - q0, -0.02, 0.02)          # =0.010
    u_guess1 = np.clip(initial_plan[1] - initial_plan[0], -0.02, 0.02)  # =0.008
    check('stage0 x = [q0, prev_delta]', np.allclose(cap.x[0][:nq], q0)
          and np.allclose(cap.x[0][nq:], pd))
    check('stage0 u = clip(initial_plan[0]-q0)', np.allclose(cap.u[0], u_guess0))
    check('stage1 x.q = initial_plan[0]', np.allclose(cap.x[1][:nq], initial_plan[0]))
    check('stage1 x.dq_prev = u_guess[0]', np.allclose(cap.x[1][nq:], u_guess0))
    check('stage1 u = u_guess[1]', np.allclose(cap.u[1], u_guess1))
    check('terminal stage N x.q = initial_plan[N-1]', np.allclose(cap.x[N][:nq], initial_plan[N - 1]))
    check('terminal stage N x.dq_prev = u_guess[N-1]', np.allclose(cap.x[N][nq:], u_guess1))
    check('stage0 lbx==ubx==x0=[q0,prev_delta]',
          np.allclose(cap.lbx[0], np.concatenate([q0, pd]))
          and np.allclose(cap.ubx[0], np.concatenate([q0, pd])))
    check('所有 u_guess 满足步长 clip', all(np.all(np.abs(cap.u[k]) <= 0.02 + 1e-12) for k in range(N)))

    # clip 生效:超限 step 被夹到 max_joint_step
    ip_big = np.array([q0 + 0.05, q0 + 0.05 + 0.09])
    cap2 = _capture(_make_ik_lazy(), q0, ip_big, np.zeros(nq))
    check('超限 step 的 u_guess 被 clip 到 0.02',
          np.allclose(cap2.u[0], 0.02) and np.allclose(cap2.u[1], 0.02))

    # zero vs measured:除 stage-0 dq_prev 槽外初始 guess 全一致
    capz = _capture(_make_ik_lazy(), q0, initial_plan, np.zeros(nq))
    capm = _capture(_make_ik_lazy(), q0, initial_plan, pd)
    same = (np.allclose(capz.x[1], capm.x[1]) and np.allclose(capz.x[2], capm.x[2])
            and np.allclose(capz.u[0], capm.u[0]) and np.allclose(capz.u[1], capm.u[1])
            and np.allclose(capz.x[0][:nq], capm.x[0][:nq]))
    check('zero/measured 仅 stage-0 dq_prev 槽不同,其余 guess 一致',
          same and not np.allclose(capz.x[0][nq:], capm.x[0][nq:]))


def _make_ik_lazy():
    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
    return MpcPinocchioIK(config=MpcIkConfig(horizon_steps=2, control_dt=DT, max_joint_step=0.02,
                                             solver_backend='acados', acados_build_dir=BUILD_DIR))


# ----------------------------------------------------------------- ACADOS 求解

def _make_ik():
    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
    return MpcPinocchioIK(config=MpcIkConfig(
        horizon_steps=2, control_dt=DT, max_joint_step=0.02, success_tolerance=0.01,
        solver_backend='acados', position_weight=50.0, terminal_position_weight=500.0,
        orientation_weight=0.375, terminal_orientation_weight=5.0, joint_delta_weight=0.08,
        joint_delta_change_weight=0.25, current_posture_weight=0.01, center_posture_weight=0.002,
        acados_rti_iterations=3, acados_build_dir=BUILD_DIR))


def test_acados_dq_prev():
    print('单元4: 真实 ACADOS x0 dq_prev / None==zero / 动力学一致性 / 无 re-codegen')
    from pinocchio_ik import PinocchioIK
    ik = _make_ik()
    nq = ik.nq
    rot = traj.TARGET_ROTATION
    p = np.array([0.30, -0.05, 0.12])
    q0, ok = PinocchioIK().solve(p, target_rotation=rot, rot_weight=0.2)
    check('起点 IK 可达', bool(ok))
    targets = np.repeat(p.reshape(1, 3), 2, axis=0)
    rots = np.repeat(rot.reshape(1, 3, 3), 2, axis=0)

    # None == explicit zeros(重置 warm-start 使两次求解独立、初值相同)
    ik._last_plan = None
    qn_none, ok1, i1 = ik.step(targets, q0, target_rotation=rots, previous_joint_delta=None)
    ik._last_plan = None
    qn_zero, ok2, i2 = ik.step(targets, q0, target_rotation=rots, previous_joint_delta=np.zeros(nq))
    check('None == zeros 数值一致', np.allclose(qn_none, qn_zero, atol=1e-9))
    check('status==0 & command_usable (None)', int(i1['status']) == 0 and bool(i1['command_usable']))

    # x0 dq_prev == 输入(经 lbx/ubx 固定):求解后 stage0 状态后半段应等于输入
    pd = np.array([0.004, -0.006, 0.008, -0.01, 0.012, -0.003, 0.005])
    qn_m, okm, im = ik.step(targets, q0, target_rotation=rots, previous_joint_delta=pd)
    x0_solved = np.array(ik._acados_solver.get(0, 'x'), dtype=float)
    check('stage0 x0 dq_prev == 输入(lbx/ubx 固定)', np.allclose(x0_solved[nq:], pd, atol=1e-9),
          f'got {np.round(x0_solved[nq:],4)}')
    check('stage0 x0 q == q0', np.allclose(x0_solved[:nq], q0, atol=1e-9))
    check('measured status==0 & command_usable', int(im['status']) == 0 and bool(im['command_usable']))

    # 动力学一致性:求解轨迹满足 q_{k+1}=q_k+u_k, dq_prev_{k+1}=u_k
    xs = np.array([ik._acados_solver.get(i, 'x') for i in range(ik.config.horizon_steps + 1)], dtype=float)
    us = np.array([ik._acados_solver.get(i, 'u') for i in range(ik.config.horizon_steps)], dtype=float)
    q_ok = all(np.allclose(xs[k + 1][:nq], xs[k][:nq] + us[k], atol=1e-6)
               for k in range(ik.config.horizon_steps))
    dqp_ok = all(np.allclose(xs[k + 1][nq:], us[k], atol=1e-6)
                 for k in range(ik.config.horizon_steps))
    check('轨迹满足 q_{k+1}=q_k+u_k', q_ok)
    check('轨迹满足 dq_prev_{k+1}=u_k', dqp_ok)
    # clip:传入超限 dq_prev 被 sanitize clip
    pd_big = np.full(nq, 0.5)
    ik.step(targets, q0, target_rotation=rots, previous_joint_delta=pd_big)
    x0_big = np.array(ik._acados_solver.get(0, 'x'), dtype=float)
    check('超限 dq_prev 被 clip 到 max_joint_step', np.all(np.abs(x0_big[nq:]) <= 0.02 + 1e-9))

    # 无 re-codegen:同配置同 build 目录二次构造应复用 .so(mtime 不变)
    so = list(Path(BUILD_DIR + '_sqp_rti_full_condensing_qpoases_it500_ws1').glob(
        'c_generated_code/libacados_ocp_solver_*.so'))
    if so:
        m0 = so[0].stat().st_mtime
        _make_ik()  # 二次构造
        m1 = so[0].stat().st_mtime
        check('二次构造复用 .so(不 re-codegen)', abs(m1 - m0) < 1e-6)
    else:
        check('.so 存在', False, 'build dir 未找到 .so')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skip-solver', action='store_true')
    args = ap.parse_args()
    test_compute_dq_prev()
    test_compute_dq_prev_fallback()
    test_history_recovery()
    test_lead_independent_of_mode()
    test_warm_start_capture()
    if args.skip_solver:
        print('单元4: 跳过(--skip-solver)')
    else:
        test_acados_dq_prev()
    print()
    if FAIL:
        print(f'FAILED: {len(FAIL)} 项: {FAIL}')
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
