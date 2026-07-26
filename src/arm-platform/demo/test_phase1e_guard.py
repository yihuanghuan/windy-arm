#!/usr/bin/env python3
"""阶段1E: 期望运动感知 command-FK 守卫 单元/故障注入测试。

驱动**生产函数对象本体**(object.__new__ stub,同阶段1B DqPrevAdapter 手法):
_apply_command_guard / _guard_expected_eval / _guard_project / _guard_accept /
_guard_enter_fault / _validate_guard_config 全部来自 move_base_circle_mpc_ik_demo。
测试组(对应任务书九):
  1 静止目标退化  2 合法快速运动  3 错误方向跳变  4 异常大跳  5 安全投影
  6 锚点污染  7 hold恢复  8 连续异常fault  9 时间对应  10 模式回归  + 参数校验
"""

import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parent))

from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo, JOINT_COUNT  # noqa: E402
from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK  # noqa: E402
from pinocchio_ik import PinocchioIK  # noqa: E402
import moving_base_trajectory_config as traj  # noqa: E402

FAIL = []


def check(name, ok, detail=''):
    tag = 'PASS' if ok else 'FAIL'
    print(f'  [{tag}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


class FakeLogger:
    def __init__(self):
        self.warns, self.errors, self.infos = [], [], []

    def warn(self, msg, **kw):
        self.warns.append(msg)

    def error(self, msg, **kw):
        self.errors.append(msg)

    def info(self, msg, **kw):
        self.infos.append(msg)


MPC_IK = MpcPinocchioIK(config=MpcIkConfig())     # 只用 FK/q 限位,不触发 ACADOS 构建
SEED_IK = PinocchioIK()


def mk_demo(mode='expected_residual', **over):
    d = object.__new__(MoveBaseCircleMpcIkDemo)
    d.guard_mode = mode
    d.dt = 0.02
    d.reference_lead_steps = 1
    d.max_joint_step = 0.02
    d.command_fk_step_warn_m = 0.006
    d.guard_residual_abs_m = 0.0068
    d.guard_residual_relative_gain = 0.5
    d.guard_residual_accel_gain = 1.0
    d.guard_projection_enabled = True
    d.guard_projection_min_lambda = 0.05
    d.guard_projection_max_iterations = 8
    d.guard_max_consecutive_blocks = 25
    d.guard_backup_damping = 0.03
    d.guard_backup_position_gain = 0.8
    d.second_order_max_velocity = 1.0
    d.second_order_max_acceleration = 5.0
    d._so_last_cmd_dq = np.zeros(JOINT_COUNT)
    d.mpc_ik = MPC_IK
    d.last_command_fk_world = None
    d.q_last = None
    d.warm_start = 'WARM_INIT'
    d._guard_q_last = None
    d._guard_t_last_reference = float('nan')
    d._guard_p_cmd_last = None
    d._guard_p_ref_last = None
    d._guard_R_err_last = None
    d._guard_uncertainty_m = 0.0
    d.guard_block_count = 0
    d.guard_consecutive_blocks = 0
    d.guard_projection_count = 0
    d.guard_fault = False
    d.solver_step_violation_count = 0
    d.phase = 'track'
    logger = FakeLogger()
    d.get_logger = lambda lg=logger: lg
    d._test_logger = logger
    for k, v in over.items():
        setattr(d, k, v)
    return d


def freeze_world(d, p_target=None):
    """静止世界: 基座 identity 固定,期望目标固定 —— Δp_ref=0, a_ref=0。"""
    p0 = np.array([0.35, 0.0, 0.15]) if p_target is None else np.asarray(p_target, float)
    d._base_pose = lambda t: np.zeros(3)
    d._base_rotation = lambda t: np.eye(3)
    d._ee_target_world = lambda t, p=p0: p.copy()
    return p0


def linear_world(d, v_vec):
    """匀速世界目标(基座 identity): p_ref(t) = p0 + v t。"""
    p0 = np.array([0.35, 0.0, 0.15])
    v = np.asarray(v_vec, dtype=float)
    d._base_pose = lambda t: np.zeros(3)
    d._base_rotation = lambda t: np.eye(3)
    d._ee_target_world = lambda t, p=p0, vv=v: p + vv * t
    return p0, v


def ik_for(p_world, q_init):
    q, ok = SEED_IK.solve(np.asarray(p_world, float), q_init=np.asarray(q_init, float))
    assert ok, f'IK 不可达 {p_world}'
    return q


def tick(d, q_next, q_current, t_solve, usable=True):
    info = {'trajectory': f'traj@{t_solve:.3f}', 'command_usable': usable,
            'terminal_error': 0.0, 'status': 0, 'message': 'ok', 'plan_feasible': True}
    return d._apply_command_guard(np.asarray(q_next, dtype=float),
                                  np.asarray(q_current, dtype=float), t_solve, info, usable)


def seed_q(d, p0):
    q0 = ik_for(p0, np.zeros(JOINT_COUNT))
    d.q_last = q0.copy()
    return q0


def bounded_jump(q_from, delta_world, d, min_jump_m):
    """构造**满足步长契约**(max|dq|<=max_joint_step)但 FK 位移仍超门限的候选。

    残差门/投影只对契约内候选生效(契约外的由六节硬拒绝兜底,单独测)。"""
    p_from = MPC_IK.forward(np.asarray(q_from, float))[0]
    q_raw = ik_for(p_from + np.asarray(delta_world, float), q_from)
    dq = q_raw - np.asarray(q_from, float)
    m = float(np.max(np.abs(dq)))
    if m > d.max_joint_step:
        dq = dq * (d.max_joint_step * 0.999 / m)
    q_c = np.asarray(q_from, float) + dq
    jump = float(np.linalg.norm(MPC_IK.forward(q_c)[0] - p_from))
    assert jump > min_jump_m, f'构造跳变不足: {jump * 1000:.1f}mm < {min_jump_m * 1000:.1f}mm'
    return q_c, jump


def bang_jump(q_from, axis=(0.0, 1.0, 0.0)):
    """契约内最大 EE 跳变: dq_i = max_joint_step*sign(J_i·u)(bang-bang)。

    该姿态下 y 向实测 ≈13.3mm —— 即**契约内**能达到的最大单拍跳变;
    ≥20mm 单拍跳变物理上必违反关节步长契约(由六节硬拒兜底)。"""
    q_from = np.asarray(q_from, float)
    J = pin.computeFrameJacobian(MPC_IK.model, MPC_IK.data, q_from, MPC_IK.frame_id,
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
    u = np.asarray(axis, float)
    dq = 0.02 * np.sign(J.T @ u)
    q_c = q_from + dq
    jump = float(np.linalg.norm(MPC_IK.forward(q_c)[0] - MPC_IK.forward(q_from)[0]))
    return q_c, jump


# ----------------------------------------------------------------- 1 静止目标
def test_stationary_degenerates():
    print('测试1: 静止目标 -> 退化为异常命令残差检查(Δp_ref=0, a_ref≈0)')
    d = mk_demo()
    p0 = freeze_world(d)
    q0 = seed_q(d, p0)
    g = tick(d, q0, q0, 0.0)                 # 首帧: 合法初始化锚点,不拦截
    check('首帧无锚点 -> 合法初始化不拦截', not g['guard_block'] and g['guard_lambda'] == 1.0)
    g = tick(d, q0, q0, 0.02)
    ev = g['guard_ev']
    check('静止: Δp_ref=0', abs(ev['expected_step']) < 1e-12, f"{ev['expected_step']:.2e}")
    check('静止: r_allow == r_abs(rel/acc 项=0)',
          abs(ev['r_allow'] - d.guard_residual_abs_m) < 1e-9,
          f"r_allow={ev['r_allow']*1000:.3f}mm")
    check('静止: 原地命令通过', not g['guard_block'] and g['guard_lambda'] == 1.0)


# ----------------------------------------------------------------- 2 合法快速运动
def test_legal_fast_motion():
    print('测试2: 合法快速运动(FK 步长 8mm>6mm): legacy 拦, expected_residual 必须通过')
    v = np.array([0.0, 0.4, 0.0])            # 0.4 m/s * 0.02 s = 8mm/tick
    results = {}
    for mode in ('legacy_fixed_step', 'expected_residual'):
        d = mk_demo(mode)
        p0, _ = linear_world(d, v)
        q = seed_q(d, p0)
        blocks, lambdas = 0, []
        for k in range(4):
            t = k * 0.02
            t_new = t + d.dt                  # lead=1
            q_next = ik_for(d._ee_target_world(t_new), q)
            g = tick(d, q_next, q, t)
            blocks += int(g['guard_block'])
            lambdas.append(g['guard_lambda'])
            if not g['guard_block']:
                q = q_next
            if mode == 'legacy_fixed_step':
                d._update_command_fk_monitor(d.q_last, t)   # 复刻生产 tick 尾部
        results[mode] = blocks
    check('legacy 拦截合法快速运动(>6mm)', results['legacy_fixed_step'] >= 1,
          f"blocks={results['legacy_fixed_step']}")
    check('expected_residual 放行合法快速运动(0 拦截)',
          results['expected_residual'] == 0, f"blocks={results['expected_residual']}")


# ----------------------------------------------------------------- 3 错误方向跳变
def test_wrong_direction():
    print('测试3: 错误方向跳变(|Δp_cmd|≈|Δp_ref| 但方向相反): 向量残差必须拦')
    v = np.array([0.0, 0.35, 0.0])           # 期望 7mm/tick
    d = mk_demo()
    p0, _ = linear_world(d, v)
    q = seed_q(d, p0)
    g0 = tick(d, q, q, 0.0)                  # 零移动建锚(必然过契约与残差门)
    check('建锚成功', g0['guard_lambda'] == 1.0 and d._guard_q_last is not None)
    # 候选: 从锚点 FK 沿 -y 移动 ~7mm(标量步长≈期望 7mm,方向相反 -> r_pos≈14mm)
    q_bad, cmd_jump = bounded_jump(q, np.array([0.0, -0.007, 0.0]), d, 0.005)
    g = tick(d, q_bad, q, 0.02)
    ev = g['guard_ev']
    scalar_diff = abs(ev['command_step'] - ev['expected_step'])
    check('标量步长几乎无差(只比大小会漏过)', scalar_diff < 0.004,
          f"|cmd|={ev['command_step']*1000:.2f}mm |ref|={ev['expected_step']*1000:.2f}mm")
    check('向量残差检出(r_pos≈|cmd|+|ref|>r_allow)', ev['r_pos'] > ev['r_allow'],
          f"r_pos={ev['r_pos']*1000:.2f}mm r_allow={ev['r_allow']*1000:.2f}mm")
    check('决策为拦截或投影', g['guard_block'] or g['guard_projected'])


# ----------------------------------------------------------------- 4 异常大跳
def test_injected_jump():
    print('测试4: 注入 >=20mm 非期望跳变: 必须 block/project(契约内)或硬拒(契约外)')
    # 4a: 契约内最大跳变(bang-bang, ~13mm)-> 残差门必须 block 或 project
    d = mk_demo()
    p0 = freeze_world(d)
    q = seed_q(d, p0)
    tick(d, q, q, 0.0)                        # 建锚
    q_bad, jump = bang_jump(q)
    check('契约内跳变确实超残差门', jump > d.guard_residual_abs_m,
          f'jump={jump*1000:.1f}mm')
    g = tick(d, q_bad, q, 0.02)
    check(f'契约内 {jump*1000:.1f}mm 跳变不被原样放行',
          not (g['guard_lambda'] == 1.0 and not g['guard_block']))
    check('block 或 project', g['guard_block'] or g['guard_projected'],
          f"block={g['guard_block']} proj={g['guard_projected']} λ={g['guard_lambda']}")
    check('异常命令未被原样发布', not np.allclose(d.q_last, q_bad))
    # 4b: >=20mm 单拍跳变在该姿态必违反关节契约(契约内最大 ~13mm)-> 六节硬拒,不发布
    d2 = mk_demo()
    p02 = freeze_world(d2)
    q02 = seed_q(d2, p02)
    tick(d2, q02, q02, 0.0)
    q_huge = ik_for(p02 + np.array([0.0, 0.025, 0.0]), q02)     # 25mm 目标
    check('25mm 跳变必然契约外', np.max(np.abs(q_huge - q02)) > d2.max_joint_step)
    g2 = tick(d2, q_huge, q02, 0.02)
    check('20mm+ 跳变 -> 步长契约硬拒(不可用,保持上一帧)',
          (not g2['command_usable']) and g2['solver_step_violation']
          and np.allclose(d2.q_last, q02))


# ----------------------------------------------------------------- 5 安全投影
def test_projection():
    print('测试5: 安全投影 λ 搜索(最大安全比例/边界/λ=1 不改命令)')
    d = mk_demo()
    p0 = freeze_world(d)
    q0 = seed_q(d, p0)
    tick(d, q0, q0, 0.0)                      # 锚点 = q0
    # 契约内 ~13mm 跳变: λ 由残差门决定(λ* ≈ r_allow/jump ≈ 0.5-0.6)
    q_bad, jump = bang_jump(q0)
    dq_dir = q_bad - q0
    g = tick(d, q_bad, q0, 0.02)
    check('候选超限 -> 投影发生', g['guard_projected'],
          f"jump={jump*1000:.1f}mm λ={g['guard_lambda']:.4f}")
    lam = g['guard_lambda']
    check('λ ∈ [min_lambda, 1)', d.guard_projection_min_lambda <= lam < 1.0)
    q_pub = d.q_last
    check('投影命令 = 锚点 + λ(q_next-锚点)',
          np.allclose(q_pub, q0 + lam * dq_dir, atol=1e-12))
    check('投影命令满足步长约束', np.max(np.abs(q_pub - q0)) <= d.max_joint_step + 1e-9,
          f"max|Δq|={np.max(np.abs(q_pub - q0)):.5f}")
    # 投影后残差不超限 + 最大性: λ+步进即违约(用独立 stub 重算,不动 d 的状态)
    def residual_at(lam_x):
        d2 = mk_demo()
        freeze_world(d2)
        d2._guard_q_last = q0.copy()
        d2._guard_t_last_reference = 0.02 + d2.dt
        p_cmd0, _ = d2._guard_fk_world(q0, 0.02 + d2.dt)
        d2._guard_p_cmd_last = p_cmd0
        d2._guard_p_ref_last = d2._ee_target_world(0.02 + d2.dt)
        return d2._guard_expected_eval(q0 + lam_x * dq_dir, 0.02 + d2.dt)
    ev_pub = residual_at(lam)
    check('投影后残差 <= r_allow', ev_pub['r_pos'] <= ev_pub['r_allow'] + 1e-12,
          f"{ev_pub['r_pos']*1000:.2f} <= {ev_pub['r_allow']*1000:.2f}mm")
    ev_hi = residual_at(min(1.0, lam + 0.02))
    check('λ+0.02 违反残差约束(λ 近似最大)', ev_hi['r_pos'] > ev_hi['r_allow'],
          f"{ev_hi['r_pos']*1000:.2f} > {ev_hi['r_allow']*1000:.2f}mm")
    # λ=1: 正常命令不被修改
    d3 = mk_demo()
    p03 = freeze_world(d3)
    q03 = seed_q(d3, p03)
    tick(d3, q03, q03, 0.0)
    small = np.zeros(JOINT_COUNT)
    small[1] = 0.004
    g3 = tick(d3, q03 + small, q03, 0.02)
    check('λ=1 时命令原样发布(不投影不修改)',
          g3['guard_lambda'] == 1.0 and not g3['guard_projected']
          and np.allclose(d3.q_last, q03 + small, atol=0.0))
    # 找不到安全 λ: min_lambda 也不可行 -> block
    d4 = mk_demo(guard_projection_min_lambda=0.9)
    p04 = freeze_world(d4)
    q04 = seed_q(d4, p04)
    tick(d4, q04, q04, 0.0)
    q_bad4, _ = bang_jump(q04)
    g4 = tick(d4, q_bad4, q04, 0.02)           # λ=0.9 处残差 ~0.9*13mm > 6.8mm 必不可行
    check('min_lambda 不可行 -> block(hold)', g4['guard_block'] and not g4['guard_projected'])


# ----------------------------------------------------------------- 6 锚点污染
def test_anchor_pollution():
    print('测试6: solver失败/NaN/被拒命令/shadow 均不得污染 accepted 锚点')
    d = mk_demo()
    p0 = freeze_world(d)
    q0 = seed_q(d, p0)
    tick(d, q0, q0, 0.0)
    anchor = d._guard_q_last.copy()
    t_anchor = d._guard_t_last_reference
    # solver 失败(command_usable=False)
    tick(d, q0 + 1.0, q0, 0.02, usable=False)
    check('solver 失败不更新锚点', np.allclose(d._guard_q_last, anchor)
          and d._guard_t_last_reference == t_anchor)
    # NaN 命令: 步长契约硬拒(residual 模式),锚点不动
    g = tick(d, np.full(JOINT_COUNT, np.nan), q0, 0.04)
    check('NaN 命令 -> 契约硬拒(不可用)', not g['command_usable']
          and g['solver_step_violation'])
    check('NaN 不污染锚点', np.allclose(d._guard_q_last, anchor))
    # 被拒绝命令(block)不更新锚点
    d.guard_projection_enabled = False
    q_bad, _ = bang_jump(q0)
    g = tick(d, q_bad, q0, 0.06)
    check('被拒绝命令 -> block 且锚点不动', g['guard_block']
          and np.allclose(d._guard_q_last, anchor))
    # shadow: legacy 拒绝帧不更新 shadow 锚点
    ds = mk_demo('expected_shadow')
    p0s = freeze_world(ds)
    q0s = seed_q(ds, p0s)
    ds._update_command_fk_monitor(q0s, 0.0)     # 建 legacy 监视点
    g = tick(ds, q0s, q0s, 0.0)                  # 接受 -> shadow 锚点建立
    anchor_s = ds._guard_q_last.copy()
    q_bad_s, _ = bang_jump(q0s)                 # ~13mm > 6mm, legacy 必拦
    g = tick(ds, q_bad_s, q0s, 0.02)
    check('shadow: legacy 拒绝帧不更新锚点', g['guard_block']
          and np.allclose(ds._guard_q_last, anchor_s))


# ----------------------------------------------------------------- 7 hold 恢复
def test_hold_recovery():
    print('测试7: 单拍异常 -> 下一拍正常必须恢复(不永久 hold)')
    d = mk_demo(guard_projection_enabled=False)
    p0 = freeze_world(d)
    q0 = seed_q(d, p0)
    tick(d, q0, q0, 0.0)
    q_bad, _ = bang_jump(q0)
    g1 = tick(d, q_bad, q0, 0.02)
    check('异常拍被拦截', g1['guard_block'] and d.guard_consecutive_blocks == 1)
    check('拦截拍保持上一命令', np.allclose(d.q_last, q0))
    g2 = tick(d, q0, q0, 0.04)                # 正常候选
    check('下一拍正常命令立即恢复', not g2['guard_block'] and g2['guard_lambda'] == 1.0)
    check('连续拦截计数清零', d.guard_consecutive_blocks == 0)
    check('恢复未经过异常重锚(锚点=正常命令)', np.allclose(d._guard_q_last, q0))


# ----------------------------------------------------------------- 8 连续异常 fault
def test_consecutive_fault():
    print('测试8: 连续异常达上限 -> 显式 guard_fault,不无限静默 hold')
    d = mk_demo(guard_projection_enabled=False, guard_max_consecutive_blocks=3)
    p0 = freeze_world(d)
    q0 = seed_q(d, p0)
    tick(d, q0, q0, 0.0)
    q_bad, _ = bang_jump(q0)
    for k in range(3):
        tick(d, q_bad, q0, 0.02 * (k + 1))
    check('第3次连续拦截进入 fault', d.guard_fault and d.phase == 'guard_fault')
    check('fault 输出 ERROR 日志', any('guard_fault' in m for m in d._test_logger.errors))
    check('锚点未被异常命令重锚', np.allclose(d._guard_q_last, q0))
    check('拦截计数=3', d.guard_block_count == 3 and d.guard_consecutive_blocks == 3)


# ------------------------------------------------------- state-safe-filter recovery
def test_state_safe_filter_active_recovery():
    print('测试8B: state_safe_filter 从当前状态主动恢复，不受 stale accepted anchor 锁死')
    d = mk_demo('state_safe_filter', guard_projection_enabled=False)
    p0, velocity = linear_world(d, np.array([0.0, 0.10, 0.0]))
    q0 = seed_q(d, p0)
    # Deliberately poison the legacy accepted-command anchor. The new mode
    # must use q_current/t_state and therefore ignore this stale geometry.
    d._guard_q_last = q0 + 0.04
    d._guard_t_last_reference = -1.0
    d._guard_p_cmd_last = np.full(3, 99.0)
    d._guard_p_ref_last = np.full(3, -99.0)

    q_wrong, _ = bounded_jump(
        q0, np.array([0.0, -0.010, 0.0]), d, 0.006)
    g = tick(d, q_wrong, q0, 0.0)
    hold_error = np.linalg.norm(
        d._guard_fk_world(q0, d.dt)[0]
        - d._ee_target_world(d.dt))
    backup_error = np.linalg.norm(
        d._guard_fk_world(d.q_last, d.dt)[0]
        - d._ee_target_world(d.dt))
    check('错误 MPC 方向触发主动 backup 而不是 block',
          g['guard_backup'] and not g['guard_block'])
    check('backup 命令满足当前状态步长约束',
          np.max(np.abs(d.q_last - q0)) <= d.max_joint_step + 1e-9)
    check('backup 不增加 world tracking error',
          backup_error <= hold_error + 1e-9,
          f'{hold_error*1000:.3f}->{backup_error*1000:.3f}mm')
    check('stale accepted anchor 被安全输出替换',
          np.max(np.abs(d._guard_q_last - (q0 + 0.04))) > 1e-3)

    # A gross solver-contract violation is clipped and filtered, not converted
    # directly into an unrecoverable hold/fault.
    d2 = mk_demo('state_safe_filter')
    p02 = freeze_world(d2)
    q02 = seed_q(d2, p02)
    d2._guard_q_last = q02.copy()
    g2 = tick(d2, q02 + 1.0, q02, 0.0)
    check('契约外有限命令由 state filter 裁剪/恢复',
          g2['solver_step_violation'] and g2['command_usable']
          and not g2['guard_block'])
    check('裁剪/恢复结果不超过单拍关节限制',
          np.max(np.abs(d2.q_last - q02)) <= d2.max_joint_step + 1e-9)

    # A failed solver must use the active backup rather than republish a stale
    # accepted joint command while the base/reference keeps moving.
    d3 = mk_demo('state_safe_filter')
    p03, _ = linear_world(d3, np.array([0.0, 0.10, 0.0]))
    q03 = seed_q(d3, p03)
    d3.q_last = q03 + 0.03
    g3 = tick(d3, np.full(JOINT_COUNT, np.nan), q03, 0.0, usable=False)
    check('求解器命令不可用时切换主动 backup',
          g3['command_usable'] and g3['guard_backup']
          and not g3['guard_block'])
    check('失败降级不再发布 stale q_last',
          np.max(np.abs(d3.q_last - (q03 + 0.03))) > 1e-3)


# ----------------------------------------------------------------- 9 时间对应
def test_time_correspondence():
    print('测试9: t_new=t_solve+lead*dt / 锚点用自己的 t_last / 两种残差算法逐拍相等')
    d = mk_demo()                              # 用真实生产轨迹(基座在动)
    q0, ok = SEED_IK.solve(d._ee_target_base(0.0),
                           target_rotation=d._target_rotation_base(0.0), rot_weight=0.2)
    assert ok
    d.q_last = q0.copy()
    g0 = tick(d, q0, q0, 0.0)
    t0_new = g0['t_new']
    check('t_new = t_solve + 1*dt', abs(t0_new - 0.02) < 1e-12)
    check('锚点参考时刻 = 接受时的 t_new', d._guard_t_last_reference == t0_new)
    g1 = tick(d, q0, q0, 0.02)
    ev = g1['guard_ev']
    t1_new = g1['t_new']
    # 直接重算(锚点用 t0_new,新命令用 t1_new;基座变换各用各的时刻)
    p_cmd_last, _ = d._guard_fk_world(q0, t0_new)
    p_cmd_new, _ = d._guard_fk_world(q0, t1_new)
    p_ref_last = d._ee_target_world(t0_new)
    p_ref_new = d._ee_target_world(t1_new)
    check('expected_step == ||p_ref(t1)-p_ref(t0)||',
          abs(ev['expected_step'] - np.linalg.norm(p_ref_new - p_ref_last)) < 1e-12)
    check('command_step == ||FK_w(q,t1)-FK_w(q,t0)||(基座各用各时刻)',
          abs(ev['command_step'] - np.linalg.norm(p_cmd_new - p_cmd_last)) < 1e-12)
    # 两种算法逐拍相等: ||Δp_cmd - Δp_ref|| == ||e_new - e_last||
    e_new = p_cmd_new - p_ref_new
    e_last = p_cmd_last - p_ref_last
    r_via_e = np.linalg.norm(e_new - e_last)
    r_via_dp = np.linalg.norm((p_cmd_new - p_cmd_last) - (p_ref_new - p_ref_last))
    check('r_pos 两种算法相等(且=实现值)',
          abs(r_via_e - r_via_dp) < 1e-15 and abs(ev['r_pos'] - r_via_e) < 1e-15)
    # 反例: 把两条命令放在同一(当前)基座位姿下的虚假比较 != 正确值(基座在动)
    fake = np.linalg.norm((p_cmd_new - d._guard_fk_world(q0, t1_new)[0])
                          - (p_ref_new - p_ref_last))
    check('虚假比较(同一基座时刻)与正确值不同', abs(fake - ev['r_pos']) > 1e-6,
          f"fake={fake*1000:.3f}mm vs {ev['r_pos']*1000:.3f}mm")


# ----------------------------------------------------------------- 10 模式回归
def test_mode_regression():
    print('测试10: legacy 逐拍==修改前语义 / shadow 不改行为 / legacy 不算 expected 残差')
    # 参考序列: 正常 tick + 一次 >6mm 跳 + 恢复。旧语义(修改前)在测试内独立复刻。
    v = np.array([0.0, 0.1, 0.0])              # 2mm/tick 正常
    seq = []
    d = mk_demo('legacy_fixed_step')
    p0, _ = linear_world(d, v)
    q = seed_q(d, p0)
    qs = [q]
    for k in range(5):
        t = k * 0.02
        if k == 3:
            q_next = ik_for(d._ee_target_world(t + 0.02) + np.array([0.0, 0.02, 0.0]), q)
        else:
            q_next = ik_for(d._ee_target_world(t + 0.02), q)
        qs.append(q_next)

    def run_mode(mode):
        dm = mk_demo(mode)
        linear_world(dm, v)
        dm.q_last = qs[0].copy()
        out = []
        qc = qs[0]
        for k in range(5):
            t = k * 0.02
            g = tick(dm, qs[k + 1], qc, t)
            out.append((bool(g['guard_block']), dm.q_last.copy(),
                        None if dm.warm_start is None else str(dm.warm_start),
                        tuple(np.round(g['dq_cmd'], 12))))
            dm._update_command_fk_monitor(dm.q_last, t)   # 复刻生产 tick 尾部(全模式)
            if not g['guard_block']:
                qc = qs[k + 1]
        return dm, out

    # 修改前语义独立复刻(6mm 固定阈值 + 每 tick 监视点更新)
    def run_original():
        last_fk = None
        q_last = qs[0].copy()
        warm = 'WARM_INIT'
        out = []
        qc = qs[0]
        dm = mk_demo('legacy_fixed_step')
        linear_world(dm, v)
        for k in range(5):
            t = k * 0.02
            q_next = qs[k + 1]
            fk_new, _ = dm._guard_fk_world(q_next, t)      # 旧守卫: 基座时刻=t_solve
            step = (np.linalg.norm(fk_new - last_fk) if last_fk is not None else 0.0)
            if last_fk is not None and step > 0.006:
                blocked = True
                warm = None
                dq = np.zeros(JOINT_COUNT)
            else:
                blocked = False
                q_last = q_next.copy()
                dq = (q_next - qc) / 0.02
                warm = f'traj@{t:.3f}'
            out.append((blocked, q_last.copy(), warm if warm is None else str(warm),
                        tuple(np.round(dq, 12))))
            last_fk, _ = dm._guard_fk_world(q_last, t)     # 监视点: 每 tick 用发布命令@t
            if not blocked:
                qc = q_next
        return out

    d_leg, out_leg = run_mode('legacy_fixed_step')
    d_sh, out_sh = run_mode('expected_shadow')
    out_orig = run_original()
    same_lo = all(a[0] == b[0] and np.allclose(a[1], b[1]) and a[2] == b[2]
                  and np.allclose(a[3], b[3]) for a, b in zip(out_leg, out_orig))
    check('legacy 逐拍 == 修改前语义(block/q_pub/warm/dq)', same_lo)
    same_ls = all(a[0] == b[0] and np.allclose(a[1], b[1]) and a[2] == b[2]
                  and np.allclose(a[3], b[3]) for a, b in zip(out_leg, out_sh))
    check('shadow 发布行为逐拍 == legacy', same_ls)
    check('legacy 模式不计算 expected 残差(锚点恒 None)', d_leg._guard_q_last is None)
    check('shadow 模式有 expected 诊断与锚点', d_sh._guard_q_last is not None)
    check('legacy 有拦截发生(序列有效)', any(a[0] for a in out_leg))


# ----------------------------------------------------------------- 参数校验
def test_param_validation():
    print('测试11: 守卫参数校验(拼写/非法值 -> ValueError)')
    V = MoveBaseCircleMpcIkDemo._validate_guard_config
    V('expected_residual', 0.0068, 0.5, 1.0, 0.05, 8, 25)   # 合法不抛
    V('state_safe_filter', 0.0068, 0.5, 1.0, 0.05, 8, 25)
    cases = [
        ('expected_residul', 0.0068, 0.5, 1.0, 0.05, 8, 25),   # 拼写错误
        ('expected_residual', float('nan'), 0.5, 1.0, 0.05, 8, 25),
        ('expected_residual', 0.0, 0.5, 1.0, 0.05, 8, 25),
        ('expected_residual', 0.0068, -0.1, 1.0, 0.05, 8, 25),
        ('expected_residual', 0.0068, 0.5, float('inf'), 0.05, 8, 25),
        ('expected_residual', 0.0068, 0.5, 1.0, 0.0, 8, 25),
        ('expected_residual', 0.0068, 0.5, 1.0, 1.5, 8, 25),
        ('expected_residual', 0.0068, 0.5, 1.0, 0.05, 0, 25),
        ('expected_residual', 0.0068, 0.5, 1.0, 0.05, 8, 0),
    ]
    all_raise = True
    for c in cases:
        try:
            V(*c)
            all_raise = False
            print(f'    未抛: {c}')
        except ValueError:
            pass
    check('全部非法参数组合 -> ValueError', all_raise)


def main():
    test_stationary_degenerates()
    test_legal_fast_motion()
    test_wrong_direction()
    test_injected_jump()
    test_projection()
    test_anchor_pollution()
    test_hold_recovery()
    test_consecutive_fault()
    test_state_safe_filter_active_recovery()
    test_time_correspondence()
    test_mode_regression()
    test_param_validation()
    print()
    if FAIL:
        print(f'FAILED: {len(FAIL)} 项: {FAIL}')
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
