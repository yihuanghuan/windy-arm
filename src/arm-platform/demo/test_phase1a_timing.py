#!/usr/bin/env python3
"""阶段1A 参考时序最小测试(独立脚本,不依赖 pytest/ROS 运行时)。

  python3 test_phase1a_timing.py [--skip-solver] [--command-fk-csv PATH]

测试 1: _target_horizon_base 的目标时刻映射 —— targets[j] 必须等于
        t+(j+1)*dt 时刻的解析 base 系目标(q_k <-> t+k*dt 的入口保证)。
测试 2: 1 维匀速参考过真实 ACADOS 求解器 —— 新时序下 q1 更接近
        target(t+dt) 而非 target(t),q_N 更接近 target(t+N*dt);
        并与旧时序(targets 从 t 开始)对照,沿轨道超前量差 ≈ v*dt。
        使用独立 build 目录,不触碰 demo 的求解器缓存。
测试 3: command-FK 记录的比较时刻 —— CSV 的 expected 列必须等于
        解析目标在 time_sec(=命令 header.stamp)处的取值,证明
        录制定义是 @publish,时间戳没有被重定义。
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import moving_base_trajectory_config as traj  # noqa: E402

DT = 0.02
FAILURES = []


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAILURES.append(name)


# ---------------------------------------------------------------- 测试 1

def test_horizon_target_times():
    print('测试 1: _target_horizon_base 目标时刻映射(lead=0/1)')
    from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo

    for lead in (0, 1):
        for horizon in (2, 8):
            for t0 in (0.0, 1.234):
                stub = object.__new__(MoveBaseCircleMpcIkDemo)
                stub.dt = DT
                stub.horizon_steps = horizon
                stub.reference_lead_steps = lead
                # t_solve 现在是显式入参(真实 clock 相对轨迹原点),替代旧合成 self.t。
                positions, rotations = MoveBaseCircleMpcIkDemo._target_horizon_base(stub, t0)

                ok_shape = positions.shape == (horizon, 3) and rotations.shape == (horizon, 3, 3)
                check(f'lead={lead} h={horizon} t={t0}: 形状', ok_shape)

                max_err = 0.0
                for j in range(horizon):
                    t_expect = t0 + DT * (lead + j)
                    base_pos = traj.base_pose(t_expect)
                    base_rot = traj.base_rotation(t_expect)
                    expect = base_rot.T @ (traj.ee_target_world(t_expect) - base_pos)
                    max_err = max(max_err, float(np.max(np.abs(positions[j] - expect))))
                check(f'lead={lead} h={horizon} t={t0}: targets[j]==解析目标(t+(lead+j)dt)',
                      max_err < 1e-12, f'max_err={max_err:.2e}')
                check(f'lead={lead} h={horizon} t={t0}: _last_target_time0==t+lead*dt',
                      abs(stub._last_target_time0 - (t0 + lead * DT)) < 1e-12)


# ---------------------------------------------------------------- 测试 2

def _rollout(ik, q0, p_of, rot, n_ticks, shifted):
    """滚动仿真(理想执行 q<-q1)。shifted=True 用新时序 targets=[p(t+dt),p(t+2dt)]。
    每个 tick 断言 ACADOS 求解 ok / status==0 / command_usable。"""
    horizon = ik.config.horizon_steps
    q = q0.copy()
    warm = None
    records = []
    for i in range(n_ticks):
        t = i * DT
        base = 1 if shifted else 0
        targets = np.array([p_of(t + (base + k) * DT) for k in range(horizon)])
        rots = np.repeat(rot.reshape(1, 3, 3), horizon, axis=0)
        q_next, ok, info = ik.step(targets, q, warm_start=warm, target_rotation=rots)
        # 每 tick 求解健康断言(warmup 后)
        if i >= 5:
            check(f'tick{i}: solver status==0', int(info.get('status', -1)) == 0)
            check(f'tick{i}: command_usable', bool(info.get('command_usable', False)))
        warm = info['trajectory']
        fk1, _ = ik.forward(q_next)
        fkN, _ = ik.forward(info['trajectory'][-1])
        records.append((t, fk1, fkN))
        q = q_next
    return records


def test_solver_1d_constant_velocity():
    print('测试 2: 1D 匀速参考 × 真实 ACADOS 求解器(新旧时序对照)')
    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK
    from pinocchio_ik import PinocchioIK

    # 与 demo 相同的求解器配置,仅 build 目录独立,避免动 demo 缓存。
    def make_ik():
        return MpcPinocchioIK(config=MpcIkConfig(
            horizon_steps=2,
            control_dt=DT,
            max_joint_step=0.02,
            success_tolerance=0.01,
            solver_backend='acados',
            position_weight=50.0,
            terminal_position_weight=500.0,
            orientation_weight=0.375,
            terminal_orientation_weight=5.0,
            joint_delta_weight=0.08,
            joint_delta_change_weight=0.25,
            current_posture_weight=0.01,
            center_posture_weight=0.002,
            acados_rti_iterations=3,
            acados_build_dir='/tmp/windylab_acados_phase1a_timing_test',
        ))

    v = 0.06  # m/s, 远离步长/速度饱和
    direction = np.array([0.0, 1.0, 0.0])
    origin = np.array([0.30, -0.09, 0.10])

    def p_of(tau):
        return origin + v * tau * direction

    rot = traj.TARGET_ROTATION
    q0, ok = PinocchioIK().solve(p_of(0.0), target_rotation=rot, rot_weight=0.2)
    check('起点 IK 可达', bool(ok))
    if not ok:
        return

    n_ticks, warmup = 150, 50
    horizon = 2
    rec_new = _rollout(make_ik(), q0, p_of, rot, n_ticks, shifted=True)
    rec_old = _rollout(make_ik(), q0, p_of, rot, n_ticks, shifted=False)

    def med(vals):
        return float(np.median(vals))

    # q1 <-> target(t+dt)
    e_next = [np.linalg.norm(fk1 - p_of(t + DT)) for t, fk1, _ in rec_new[warmup:]]
    e_cur = [np.linalg.norm(fk1 - p_of(t)) for t, fk1, _ in rec_new[warmup:]]
    check('新时序: q1 更接近 target(t+dt) 而非 target(t)',
          med(e_next) < med(e_cur),
          f'median |FK(q1)-p(t+dt)|={med(e_next) * 1000:.3f}mm vs '
          f'|FK(q1)-p(t)|={med(e_cur) * 1000:.3f}mm')

    # qN <-> target(t+N*dt)
    eN = [np.linalg.norm(fkN - p_of(t + horizon * DT)) for t, _, fkN in rec_new[warmup:]]
    eN_prev = [np.linalg.norm(fkN - p_of(t + (horizon - 1) * DT)) for t, _, fkN in rec_new[warmup:]]
    check('新时序: qN 更接近 target(t+N*dt) 而非 target(t+(N-1)*dt)',
          med(eN) < med(eN_prev),
          f'median |FK(qN)-p(t+Ndt)|={med(eN) * 1000:.3f}mm vs '
          f'上一时刻 {med(eN_prev) * 1000:.3f}mm')

    # 新旧时序沿轨道超前量差 ≈ v*dt
    def along(records):
        return [float(np.dot(fk1 - p_of(t), direction)) for t, fk1, _ in records[warmup:]]

    gain = float(np.mean(along(rec_new))) - float(np.mean(along(rec_old)))
    expect = v * DT
    check('新-旧时序沿轨道超前量差 ≈ v*dt',
          0.5 * expect < gain < 1.5 * expect,
          f'gain={gain * 1000:.3f}mm, v*dt={expect * 1000:.3f}mm')


# ---------------------------------------------------------------- 测试 3

def test_command_fk_expected_time(csv_path):
    print('测试 3: command-FK 记录的比较时刻(expected == 解析目标@stamp)')
    if csv_path is None:
        data_dir = Path(__file__).resolve().parent / 'data'
        cands = sorted(data_dir.glob('moving_base_circle_trajectory_*_command_fk.csv'))
        if not cands:
            check('存在 command_fk CSV', False, '未找到,可用 --command-fk-csv 指定')
            return
        csv_path = cands[-1]
    csv_path = Path(csv_path)
    import csv as csv_mod
    with csv_path.open(newline='') as f:
        reader = csv_mod.reader(f)
        header = next(reader)
        rows = np.array([[float(x) for x in row] for row in reader if row])
    t = rows[:, header.index('time_sec')]
    exp = np.column_stack([rows[:, header.index(f'expected_{a}')] for a in 'xyz'])
    ana = np.array([traj.ee_target_world(float(x)) for x in t])
    max_err = float(np.max(np.linalg.norm(exp - ana, axis=1)))
    check(f'{csv_path.name}: expected 列 == target(time_sec)',
          max_err < 1e-9, f'max_err={max_err:.2e} m')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--skip-solver', action='store_true',
                        help='跳过需要 ACADOS 的测试 2')
    parser.add_argument('--command-fk-csv', default=None)
    args = parser.parse_args()

    test_horizon_target_times()
    if args.skip_solver:
        print('测试 2: 已跳过(--skip-solver)')
    else:
        test_solver_1d_constant_velocity()
    test_command_fk_expected_time(args.command_fk_csv)

    print()
    if FAILURES:
        print(f'FAILED: {len(FAILURES)} 项未通过: {FAILURES}')
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
