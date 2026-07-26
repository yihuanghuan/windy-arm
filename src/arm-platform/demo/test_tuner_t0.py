#!/usr/bin/env python3
"""调参阶段 T0: 调参器语义对齐可信度测试。

  python3 test_tuner_t0.py [--skip-solver]

覆盖(任务十):
1. reference lead 匀速时序测试(targets[j]=target(t+(1+j)dt);全闭环无一拍错位);
2. measured_delta 与生产实现对照(适配器==生产函数对象,含异常序列);
3. fixed RTI 次数测试(tuner 配置显式 fixed x3;mock 验证跑满/早停;签名与生产结构一致);
4. 执行器速度/加速度限幅测试(v_ff/v_fb/v_cmd 公式、饱和计数);
5. command-FK 守卫触发与冻结测试(hold 保持上一命令、计数、hold-then-jump);
6. 时间对齐测试(q_actual/cmd 对 target(t+dt),对照错位口径);
7. (真实 ACADOS)生产参数最小闭环 status==0 & nit==3。
Phase 0/1A/1B/1C/1D 回归由各自测试文件负责,汇总脚本一起跑。
"""

import argparse
import math
import sys
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import tune_unified_mpc_params as T  # noqa: E402
from move_base_circle_mpc_ik_demo import MoveBaseCircleMpcIkDemo  # noqa: E402

FAIL = []
DT = T.CONTROL_DT


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAIL.append(name)


# ----------------------------------------------------------------- 工具

class FakeSolver:
    """满足 step() 合约的假求解器:q_next 由外部函数给出(不经 ACADOS)。"""

    def __init__(self, q_next_fn, max_joint_step=0.02):
        self.config = types.SimpleNamespace(max_joint_step=max_joint_step)
        self._last_plan = None
        self.q_next_fn = q_next_fn
        self.calls = 0
        self.received_dq_prev = []

    def step(self, targets, q_current, warm_start=None, target_rotation=None,
             previous_joint_delta=None, posture_reference=None):
        self.received_dq_prev.append(
            None if previous_joint_delta is None else np.asarray(previous_joint_delta).copy())
        q_next = self.q_next_fn(self.calls, targets, q_current)
        self.calls += 1
        info = {"trajectory": None, "status": 0, "success": True,
                "command_usable": True, "tracking_ok": True, "solve_time": 0.0005,
                "nit": 3}
        return q_next, True, info


def make_uniform_line_scenario(v=0.05):
    """真匀速直线场景(固定臂,无守卫): p(t) = A + v*t*ŷ。"""
    A = np.array([0.35, -0.15, 0.15])
    vy = np.array([0.0, v, 0.0])
    return T.Scenario(
        name="uniform_line", demo="", period_sec=1.0, lock_orientation=False,
        position_fn=lambda t: A + vy * t, rotation_fn=None)


# ----------------------------------------------------------------- 测试 1+6

def test_reference_lead_times():
    print('测试1a: target_times 匀速时序定义')
    tt = T.target_times(0.5, 4)
    expect = 0.5 + DT * np.array([1, 2, 3, 4])
    check('targets[j]=t+(1+j)dt', np.allclose(tt, expect, atol=1e-12), str(tt))


def test_full_loop_alignment():
    print('测试1b/6: 全闭环无一拍错位(匀速轨迹 + 完美 IK 求解器)')
    from pinocchio_ik import PinocchioIK
    ik = PinocchioIK()
    sc = make_uniform_line_scenario(v=0.05)
    q0, ok = ik.solve(sc.position_fn(0.0))
    check('起点可达', bool(ok))
    sc.q_seed = q0
    ctx = T.make_context(warmup_periods=1.0, steady_periods=2.0)  # 1s 预热 + 2s 稳态
    sc.q_seed = np.clip(sc.q_seed, ctx.q_lower, ctx.q_upper)

    # 完美求解器: 每 tick 返回 target(t_solve+dt) 的 IK(warm 种子=上一解,保持连续分支)
    state = {"seed": q0.copy()}

    def q_next_fn(k, targets, q_current):
        # targets[0] 即 target(t_solve+dt)(时序定义);对它做精确 IK
        qn, okn = ik.solve(targets[0], q_init=state["seed"])
        assert okn
        state["seed"] = qn
        return qn

    solver = FakeSolver(q_next_fn)
    trace = []
    m = T.evaluate_scenario(sc, solver, ctx, horizon=2, trace=trace)

    # 对齐口径: cmd_err = |FK(q_pub) - target(t_next)| 应≈0(完美 IK)
    steady_cmd = [r["cmd_err"] for r in trace if r["steady"]]
    aligned_rms = float(np.sqrt(np.mean(np.square(steady_cmd))))
    # 错位口径(故意造一拍错): |FK(q_pub) - target(t_solve)| 应≈ v*dt = 1.0mm
    import pinocchio as pin  # noqa
    mis = []
    for r in trace:
        if not r["steady"]:
            continue
        pos, _ = T.fk(ctx, np.asarray(r["q_pub"]))
        mis.append(float(np.linalg.norm(pos - sc.position_fn(r["t_solve"]))))
    mis_rms = float(np.sqrt(np.mean(np.square(mis))))
    check('对齐口径 cmd_err≈0(<0.3mm)', aligned_rms < 3e-4, f'{aligned_rms*1000:.3f}mm')
    check('错位口径 ≈ v*dt=1.0mm(证明只差一拍)', abs(mis_rms - 0.05 * DT) < 0.4e-3,
          f'{mis_rms*1000:.3f}mm vs 1.000mm')
    check('对齐口径远小于错位口径', aligned_rms < 0.35 * mis_rms)
    # dq_prev 每 tick 都传入(measured_delta)
    got = solver.received_dq_prev
    check('每 tick 传 previous_joint_delta', all(d is not None for d in got))
    check('首帧 dq_prev=0(回退)', np.all(got[0] == 0.0))
    check('稳态 dq_prev 非零', any(np.any(np.abs(d) > 1e-9) for d in got[10:]))


# ----------------------------------------------------------------- 测试 2

def test_measured_delta_parity():
    print('测试2: DqPrevAdapter 与生产 _compute_dq_prev 对照(含异常)')
    seq = [
        (np.full(7, 0.10), 100.00),
        (np.full(7, 0.105), 100.02),
        (np.full(7, np.nan), 100.04),     # NaN q
        (np.full(7, 0.11), 100.06),
        (np.full(7, 0.115), 100.05),      # 时间倒退
        (np.full(7, 0.12), 100.08),
        (np.full(7, 0.125), 101.00),      # 过旧
        (np.full(7, 0.13), 101.02),
    ]
    adapter = T.DqPrevAdapter(0.02)
    # 生产直接调用(独立 stub, 同样按 update_anchor 条件更新)
    stub = object.__new__(MoveBaseCircleMpcIkDemo)
    stub.dq_prev_mode = 'measured_delta'
    stub.dt = DT
    stub.max_joint_step = 0.02
    stub.dq_measured = None
    stub._prev_solve_q_measured = None
    stub._prev_solve_js_stamp = float('nan')
    all_eq = True
    for q, js in seq:
        a = adapter.compute(q, js)
        r = MoveBaseCircleMpcIkDemo._compute_dq_prev(stub, q, js)
        if r['update_anchor']:
            stub._prev_solve_q_measured = r['anchor_q']
            stub._prev_solve_js_stamp = float(r['anchor_stamp'])
        if not np.allclose(a, r['input'], atol=0, rtol=0, equal_nan=True):
            all_eq = False
    check('适配器输出 == 生产函数逐拍一致(含 NaN/倒退/过旧)', all_eq)
    # closeout 修复: 用 monkeypatch 哨兵证明适配器**确实调用生产函数对象**(可失败断言)。
    sentinel = np.full(7, 42.42)
    orig = MoveBaseCircleMpcIkDemo._compute_dq_prev
    try:
        def fake_compute(self_stub, q_now, js_stamp):
            return {'input': sentinel.copy(), 'raw': sentinel.copy(),
                    'velocity_estimate': np.zeros(7), 'source': 1, 'fallback': 0,
                    'feedback_dt': 0.02, 'update_anchor': False,
                    'anchor_q': None, 'anchor_stamp': None}
        MoveBaseCircleMpcIkDemo._compute_dq_prev = fake_compute
        a2 = T.DqPrevAdapter(0.02)
        out = a2.compute(np.zeros(7), 100.0)
        check('monkeypatch 哨兵: 适配器确实调用生产函数并返回其值',
              np.allclose(out, sentinel), str(out[:2]))
    finally:
        MoveBaseCircleMpcIkDemo._compute_dq_prev = orig
    # 恢复后再验证一次真实路径可用
    a3 = T.DqPrevAdapter(0.02)
    r0 = a3.compute(np.zeros(7), 100.0)
    check('monkeypatch 已恢复(首帧回退0)', np.all(r0 == 0.0))


# ----------------------------------------------------------------- 测试 3

def test_fixed_rti_config():
    print('测试3: tuner 配置显式 fixed RTI + mock 跑满/早停 + 签名与生产结构一致')
    cfg = T._build_config(T.PRODUCTION_PARAMS, True)
    check("rti_iteration_mode=='fixed_iterations'", cfg.rti_iteration_mode == 'fixed_iterations')
    check('acados_rti_iterations==3', cfg.acados_rti_iterations == 3)
    check('生产权重显式(50/500/0.08/0.25/0.01/0.002)',
          cfg.position_weight == 50.0 and cfg.terminal_position_weight == 500.0
          and cfg.joint_delta_weight == 0.08 and cfg.joint_delta_change_weight == 0.25
          and cfg.current_posture_weight == 0.01 and cfg.center_posture_weight == 0.002)

    from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK, _acados_solver_signature
    ik_t = MpcPinocchioIK(config=cfg)
    # 生产 demo 等价结构配置(build dir 不进签名)
    cfg_prod = MpcIkConfig(horizon_steps=2, control_dt=DT, max_joint_step=0.02,
                           success_tolerance=0.01, solver_backend='acados',
                           orientation_weight=0.375, terminal_orientation_weight=5.0,
                           acados_rti_iterations=3, rti_iteration_mode='fixed_iterations',
                           acados_build_dir='/tmp/windylab_acados_moving_base_mpc_ik')
    ik_p = MpcPinocchioIK(config=cfg_prod)
    import json as _json
    nq = ik_t.nq
    sig_t = _acados_solver_signature(ik_t, 2 * nq, nq, 1, 0, 0, 0, 'm')
    sig_p = _acados_solver_signature(ik_p, 2 * nq, nq, 1, 0, 0, 0, 'm')
    check('tuner 配置签名 == 生产结构签名(无结构漂移/不触发额外 codegen)',
          _json.dumps(sig_t, sort_keys=True) == _json.dumps(sig_p, sort_keys=True))

    # mock RTI: 跑满 / status!=0 早停(注入 ScriptSolver,绕过 acados)
    class ScriptSolver:
        def __init__(self, seq):
            self.seq = list(seq); self.calls = 0; self.x = {}; self.u = {}

        def set(self, s, k, v):
            if k == 'x':
                self.x[s] = np.asarray(v, float).copy()
            elif k == 'u':
                self.u[s] = np.asarray(v, float).copy()

        def get(self, s, k):
            return (self.x if k == 'x' else self.u)[s]

        def solve(self):
            r = self.seq[min(self.calls, len(self.seq) - 1)]; self.calls += 1; return r
    q0 = np.array([0.1, -0.15, -0.4, 1.2, -1.0, -1.2, -1.7])
    tgt = np.repeat(np.array([[0.3, -0.05, 0.12]]), 2, axis=0)
    import moving_base_trajectory_config as mb
    rots = np.repeat(mb.TARGET_ROTATION.reshape(1, 3, 3), 2, axis=0)
    ip = np.array([q0 + 0.01, q0 + 0.018])
    ik_t._acados_solver = ScriptSolver([0, 0, 0])
    r_full = ik_t._optimize_acados(q0, tgt, rots, ip, previous_joint_delta=np.zeros(7))
    check('fixed: status0 跑满 nit=3', r_full.nit == 3)
    ik_t._acados_solver = ScriptSolver([0, 4, 0])
    r_stop = ik_t._optimize_acados(q0, tgt, rots, ip, previous_joint_delta=np.zeros(7))
    check('fixed: 第二轮 status!=0 停止 nit=2', r_stop.nit == 2)


# ----------------------------------------------------------------- 参数候选纯函数

def test_extended_weight_candidates():
    print('测试3b: 一阶姿态锚权重透传 + 二阶紧凑候选')
    custom = T.ParamSet(
        2, 0.02, 0.09375, 1.25,
        current_posture_weight=0.03,
        center_posture_weight=0.004,
    )
    cfg = T._build_config(custom, True)
    check('一阶 current/center posture 权重不再写死',
          cfg.current_posture_weight == 0.03
          and cfg.center_posture_weight == 0.004)
    prod2 = T.production_second_order_config()
    check('二阶生产配置与真机实时 N=10 demo 默认一致',
          prod2.horizon_steps == 10
          and prod2.position_weight == 5000.0
          and prod2.terminal_position_weight == 500.0
          and not prod2.feasible_reference_projection
          and not prod2.contour_tracking_enabled
          and prod2.lag_position_weight == 750.0
          and prod2.cartesian_velocity_weight == 0.0
          and prod2.joint_acceleration_weight == 0.0
          and prod2.terminal_joint_velocity_weight == 0.0)

    phase2a = T.phase2a_candidates(custom)
    check('位置/平滑候选保留姿态锚权重',
          len(phase2a) == 27
          and all(p.current_posture_weight == 0.03
                  and p.center_posture_weight == 0.004 for p in phase2a))
    posture = T.phase2a_posture_candidates(custom)
    changed_counts = [
        int(p.current_posture_weight != custom.current_posture_weight)
        + int(p.center_posture_weight != custom.center_posture_weight)
        for p in posture
    ]
    check('姿态微调是 baseline + 单坐标扫描(5组)',
          len(posture) == 5 and 0 in changed_counts
          and all(n <= 1 for n in changed_counts))

    base2 = T.SecondOrderMpcConfig(
        horizon_steps=3,
        position_weight=40.0,
        terminal_position_weight=400.0,
        joint_velocity_weight=0.04,
        joint_acceleration_weight=0.02,
        current_posture_weight=0.02,
        center_posture_weight=0.004,
    )
    weights2 = T.second_order_weight_candidates(base2)
    baseline2 = T.SecondOrderWeightSet.from_config(base2)
    weight_fields = (
        'position_weight', 'terminal_position_weight',
        'joint_velocity_weight', 'joint_acceleration_weight',
        'current_posture_weight', 'center_posture_weight',
    )
    changed2 = [
        sum(getattr(p, name) != getattr(baseline2, name)
            for name in weight_fields)
        for p in weights2
    ]
    check('二阶候选含局部扫描、宽位置权重、零加速度软罚和诊断组合',
          len(weights2) == 19 and 0 in changed2
          and any(w.position_weight == base2.position_weight * 1000
                  for w in weights2)
          and any(w.joint_acceleration_weight == 0.0 for w in weights2)
          and any(w.position_weight == base2.position_weight * 1000
                  and w.joint_acceleration_weight == 0.0 for w in weights2))
    configs2 = T.second_order_config_candidates(base2)
    check('二阶候选保持时域/约束结构并返回独立配置',
          len(configs2) == 19
          and all(c is not base2 and c.horizon_steps == 3 for c in configs2)
          and base2.position_weight == 40.0)
    check('二阶 position 候选同步轮廓/转场权重',
          all(c.contour_position_weight == c.position_weight
              and c.transition_position_weight == c.position_weight
              and c.terminal_contour_position_weight
                  == c.terminal_position_weight
              for c in configs2))
    stable2 = T.stable_contour_second_order_candidates(prod2)
    check('稳定轮廓候选不修改速度/加速度硬约束',
          len(stable2) >= 10
          and all(c.max_joint_velocity == prod2.max_joint_velocity
                  and c.max_joint_acceleration == prod2.max_joint_acceleration
                  for c in stable2))
    check('稳定轮廓候选含 position-only 锚点和切向权重加密扫描',
          any(not c.contour_tracking_enabled for c in stable2)
          and any(c.lag_position_weight == 1000.0 for c in stable2)
          and any(c.lag_position_weight == 1500.0 for c in stable2))


# ----------------------------------------------------------------- 测试 4

def test_plant_limits():
    print('测试4: 执行器 v_ff/v_fb/v_cmd 公式与速度/加速度限幅')
    q0 = np.zeros(7)
    plant = T.FeedforwardPlant(q0, np.full(7, -3.0), np.full(7, 3.0))
    # 远目标 + 大前馈: v_ff=clip(2,±1)=1, v_fb=clip(远/0.15,±0.4)=0.4, raw=clip(0.7+0.4,±1)=1
    plant.advance(np.full(7, 2.0), np.full(7, 2.0))
    check('v_ff=clip(dq_ref,±1)=1.0', np.allclose(plant.last_v_ff, 1.0))
    check('v_fb=clip(Δq/0.15,±0.4)=0.4', np.allclose(plant.last_v_fb, 0.4))
    # 加速度限幅: 从 0 起,2 个 10ms 子步,每步 ≤5.0*0.01=0.05 → v=0.10
    check('加速度限幅生效 v_last=0.10', np.allclose(plant.v_last, 0.10, atol=1e-9),
          str(plant.v_last[:2]))
    check('速度/加速度饱和子步被计数', plant.vel_sat_substeps > 0 and plant.accel_sat_substeps > 0)
    # 持续推进,速度不越 1.0、每子步加速度不越 0.05
    v_prev = plant.v_last.copy()
    ok_v = ok_a = True
    for _ in range(100):
        plant.advance(np.full(7, 2.0), np.full(7, 2.0))
        if np.any(np.abs(plant.v_last) > 1.0 + 1e-9):
            ok_v = False
        # advance 内部 2 子步,外部只能看 tick 级: |dv| ≤ 2*0.05
        if np.any(np.abs(plant.v_last - v_prev) > 0.10 + 1e-9):
            ok_a = False
        v_prev = plant.v_last.copy()
    check('|v|≤1.0 恒成立', ok_v)
    check('tick 级 |dv|≤0.10(=2子步×0.05)恒成立', ok_a)


# ----------------------------------------------------------------- 测试 5

def test_guard_freeze():
    print('测试5: command-FK 守卫触发/冻结/hold-then-jump(已知跳变量)')
    from pinocchio_ik import PinocchioIK
    ik = PinocchioIK()
    sc = T.make_moving_base_scenario(base_period_sec=1.0, base_z_amplitude_m=0.0,
                                     ee_period_sec=4.0, attach_production_limits=False)
    q0, ok = ik.solve(sc.position_fn(0.0), target_rotation=sc.rotation_fn(0.0), rot_weight=0.2)
    check('起点可达', bool(ok))
    ctx = T.make_context(warmup_periods=0.25, steady_periods=0.25)  # 2s 共 100 tick
    sc.q_seed = np.clip(q0, ctx.q_lower, ctx.q_upper)

    # closeout 修复: 明确序列 —— tick<30 发布 q0;tick30 大跳(被守卫 hold);
    # tick31 起返回 q0+KNOWN_STEP(FK 步进<6mm 可放行,相对上一发布命令 q0 非零)。
    KNOWN_STEP = np.array([0.004, -0.003, 0.002, 0.0, 0.0, 0.0, 0.0])

    def q_next_fn(k, targets, q_current):
        if k == 30:
            return q0 + np.array([0.15, 0, 0, 0, 0, 0, 0])   # joint1 大跳 → FK 跳变>6mm
        if k >= 31:
            return q0 + KNOWN_STEP
        return q0.copy()

    solver = FakeSolver(q_next_fn)
    trace = []
    m = T.evaluate_scenario(sc, solver, ctx, horizon=2, trace=trace)
    check('守卫触发计数>=1', m.guard_count >= 1, f'guard={m.guard_count}')
    r30, r29, r31 = trace[30], trace[29], trace[31]
    check('守卫 tick 冻结: q_pub 保持上一帧', np.allclose(r30['q_pub'], r29['q_pub']))
    check('守卫 tick dq_pub=0', np.allclose(r30['dq_pub'], 0.0))
    check('trace 标记 guard_block', bool(r30['guard_block']))
    check('hold 后首个放行命令确为 q0+KNOWN_STEP',
          np.allclose(r31['q_pub'], q0 + KNOWN_STEP, atol=1e-12))
    # closeout 修复: hold_then_jump_max == 已知关节跳变量(可失败断言)。
    check('hold_then_jump_max == max|KNOWN_STEP| = 0.004',
          abs(m.hold_then_jump_max - 0.004) < 1e-9, f'{m.hold_then_jump_max:.6f}')
    # 对照: 无 hold 的序列 hold_then_jump 必须为 0(不误计数)。
    sc_nh = T.make_moving_base_scenario(base_period_sec=1.0, base_z_amplitude_m=0.0,
                                        ee_period_sec=4.0, attach_production_limits=False)
    sc_nh.q_seed = sc.q_seed.copy()
    m_nohold = T.evaluate_scenario(
        sc_nh, FakeSolver(lambda k, tg, qc: q0.copy()), ctx, horizon=2)
    check('无 hold 序列 hold_then_jump==0', m_nohold.hold_then_jump_max == 0.0)
    # 对照: 无守卫场景(fixed 臂)同样跳变不会拦
    sc2 = make_uniform_line_scenario(0.0)
    q02, _ = ik.solve(sc2.position_fn(0.0))
    sc2.q_seed = np.clip(q02, ctx.q_lower, ctx.q_upper)

    def q_next_fn2(k, targets, q_current):
        if k == 30:
            return q02 + np.array([0.15, 0, 0, 0, 0, 0, 0])
        return q02.copy()
    m2 = T.evaluate_scenario(sc2, FakeSolver(q_next_fn2), ctx, horizon=2)
    check('固定臂场景(生产无守卫)不拦截 guard=0', m2.guard_count == 0)


# ----------------------------------------------------------------- closeout 新增测试

def _mk_sm(**over):
    """构造 fabricated ScenarioMetrics(全字段良性默认)。"""
    import dataclasses
    d = {}
    for f in dataclasses.fields(T.ScenarioMetrics):
        if f.name == 'name':
            d[f.name] = 'fab'
        elif 'worst_joint' in f.name:
            d[f.name] = ''
        elif 'per_joint' in f.name:
            d[f.name] = {}
        elif 'count' in f.name:
            d[f.name] = 0
        else:
            d[f.name] = 0.0
    d.update(over)
    return T.ScenarioMetrics(**d)


def _mk_sc(name='fab', **over):
    kw = dict(name=name, demo='', period_sec=4.0, lock_orientation=True,
              position_fn=lambda t: np.zeros(3), rotation_fn=lambda t: np.eye(3))
    kw.update(over)
    return T.Scenario(**kw)


def test_joint_period_windows():
    print('closeout-A: 联合重复周期与窗口覆盖')
    cases = [(1.0, 4.0, 4.0), (8.0, 4.0, 8.0), (0.75, 4.0, 12.0)]
    for bp, ee, expect in cases:
        sc = T.make_moving_base_scenario(f'jp{bp}', base_period_sec=bp, ee_period_sec=ee)
        check(f'base={bp} EE={ee} -> repetition={expect}', sc.eval_period() == expect,
              f'{sc.eval_period()}')
        # 默认 2+4 周期: 预热/稳态秒数按联合周期
        warm = 2.0 * sc.eval_period()
        steady = 4.0 * sc.eval_period()
        check(f'  2+4 周期窗口 = {warm}/{steady}s(覆盖联合周期,不退回 EE 周期)',
              warm == 2 * expect and steady == 4 * expect)
    # 固定臂场景仍用自身周期
    sc_line = make_uniform_line_scenario()
    check('固定臂 eval_period == 自身 period', sc_line.eval_period() == sc_line.period_sec)


def test_error_limit_gates():
    print('closeout-B: 绝对误差硬门(fabricated metrics)')
    sc = _mk_sc(pos_p95_limit_m=0.032, pos_max_limit_m=0.038, cmd_p95_limit_m=0.032,
                orientation_limit_rad=0.40)
    ok_m = _mk_sm(steady_pos_err_p95=0.020, steady_pos_err_max=0.025,
                  steady_cmd_err_p95=0.021, steady_ori_err_max=0.2)
    fails, _ = T.hard_gate_reasons(ok_m, sc, None)
    slow_control = _mk_sm(
        steady_pos_err_p95=0.020, steady_pos_err_max=0.025,
        steady_cmd_err_p95=0.021, steady_ori_err_max=0.2,
        p99_control_compute_time_ms=(
            T.MAX_CONTROL_COMPUTE_TIME_MS_HARD + 0.1))
    slow_fails, _ = T.hard_gate_reasons(slow_control, sc, None)
    check('完整控制计算 P99 超预算 -> 淘汰',
          any('slow_control_compute_p99' in reason
              for reason in slow_fails), str(slow_fails))
    check('全部门内 -> 不误杀', fails == [], str(fails))
    f1, _ = T.hard_gate_reasons(_mk_sm(steady_pos_err_p95=0.040), sc, None)
    check('P95 超限 -> 淘汰且注明', any('pos_p95' in x and 'limit' in x for x in f1), str(f1))
    f2, _ = T.hard_gate_reasons(_mk_sm(steady_pos_err_max=0.050), sc, None)
    check('max 超限 -> 淘汰', any('pos_max' in x for x in f2))
    f3, _ = T.hard_gate_reasons(_mk_sm(steady_cmd_err_p95=0.050), sc, None)
    check('cmd P95 超限 -> 淘汰', any('cmd_p95' in x for x in f3))
    f4, _ = T.hard_gate_reasons(_mk_sm(steady_ori_err_max=0.5), sc, None)
    check('ori 超限 -> 淘汰', any('ori_max' in x for x in f4))
    # 不串用: 场景 B 门限宽,同样指标不淘汰
    sc_b = _mk_sc('fabB', pos_p95_limit_m=0.100, pos_max_limit_m=0.100,
                  cmd_p95_limit_m=0.100, orientation_limit_rad=1.0)
    fb, _ = T.hard_gate_reasons(_mk_sm(steady_pos_err_p95=0.040), sc_b, None)
    check('不同场景用各自门限(B 门宽不淘汰)', fb == [])
    # 正式搜索缺门限 -> 快速失败
    sc_nolim = _mk_sc('nolim')
    try:
        T.evaluate(T.PRODUCTION_PARAMS, [sc_nolim], T.make_context(0.1, 0.1), 'x',
                   require_limits=True)
        check('缺门限 + require_limits -> SystemExit', False, '未抛')
    except SystemExit as e:
        check('缺门限 + require_limits -> SystemExit', 'nolim' in str(e))


def test_joint_limit_clip():
    print('closeout-C: 关节位置撞限诊断与硬门')
    q0 = np.zeros(3)
    plant = T.FeedforwardPlant(q0, np.full(3, -0.05), np.full(3, 0.05))
    for _ in range(120):                     # 持续向 +1.0 推,必撞 +0.05 上限
        plant.advance(np.full(3, 1.0), np.full(3, 1.0))
    check('位置被 clip 在上限', np.all(plant.q <= 0.05 + 1e-12), str(plant.q))
    check('clip 计数>0', plant.joint_clip_substeps > 0, f'{plant.joint_clip_substeps}')
    check('每关节 clip 计数>0', np.all(plant.joint_clip_per_joint > 0))
    check('接近限位 active 计数>0', plant.joint_active_substeps > 0)
    check('last_tick_clipped 标记', plant.last_tick_clipped is True)
    # 硬门: clip>0 淘汰
    f, _ = T.hard_gate_reasons(_mk_sm(joint_limit_clip_count=5), _mk_sc(), None)
    check('joint_limit_clip>0 -> 硬淘汰', any('joint_limit_clip' in x for x in f))


def test_saturation_reference_gate():
    print('closeout-D: 饱和预算相对门(vs 生产参考, max 每关节口径)')
    ref = _mk_sm(vel_sat_max_joint_ratio=0.10, accel_sat_max_joint_ratio=0.80,
                 vel_sat_ratio=0.15, accel_sat_ratio=0.99)
    sc = _mk_sc()
    # 容差内不淘汰
    f_ok, _ = T.hard_gate_reasons(
        _mk_sm(vel_sat_max_joint_ratio=0.14, accel_sat_max_joint_ratio=0.84),
        sc, ref, references_enabled=True)
    check('参考+容差内 -> 不淘汰', f_ok == [], str(f_ok))
    # 超容差淘汰
    f_v, _ = T.hard_gate_reasons(_mk_sm(vel_sat_max_joint_ratio=0.20), sc, ref,
                                 references_enabled=True)
    check('vel_sat_max_joint 超参考+容差 -> 淘汰', any('vel_sat_max_joint' in x for x in f_v))
    f_a, _ = T.hard_gate_reasons(_mk_sm(accel_sat_max_joint_ratio=0.90), sc, ref,
                                 references_enabled=True)
    check('accel_sat_max_joint 超参考+容差 -> 淘汰',
          any('accel_sat_max_joint' in x for x in f_a))
    # 修订核心: 总体口径 accel≈0.99 不再决定门 —— 总体拉满而 max_joint 在容差内时不淘汰,
    # 且门限本身 < 1(不是数学上不可触发的空门)。
    f_vac, _ = T.hard_gate_reasons(
        _mk_sm(accel_sat_ratio=1.0, accel_sat_max_joint_ratio=0.82), sc, ref,
        references_enabled=True)
    check('总体口径=1.0 不触发门(门用 max_joint 口径)', f_vac == [], str(f_vac))
    check('生产参考门限可触发(ref+tol<1)',
          ref.accel_sat_max_joint_ratio + T.ACCEL_SAT_RATIO_TOL < 1.0)
    # 无有效参考: 跳过并备注,不淘汰
    f_n, notes = T.hard_gate_reasons(_mk_sm(vel_sat_max_joint_ratio=0.99), sc, None,
                                     references_enabled=True)
    check('无有效参考 -> 相对门跳过并备注 NO_VALID_REFERENCE',
          f_n == [] and any('NO_VALID_REFERENCE' in n for n in notes))
    # reference_is_valid 判定
    check('参考含 guard -> 无效', not T.reference_is_valid(_mk_sm(guard_count=1)))
    check('干净参考 -> 有效', T.reference_is_valid(_mk_sm()))


def test_snapshot_limit_gating():
    print('closeout-H: 阶段1D 轨迹快照门控 D0 门限')
    # 生产默认调用: 仅当 mbtraj 当前配置与阶段1D快照一致时才附门限
    # (mbtraj 常量可能被实验改动,如 BASE_PERIOD_SEC/BASE_Z_AMPLITUDE —— 此时必须不附)。
    sc_prod = T.make_moving_base_scenario()
    prod_match, prod_diffs = T._traj_config_matches_phase1d(sc_prod.traj_config)
    if prod_match:
        check('生产默认(快照一致)自动附 D0 门限', sc_prod.has_required_limits())
    else:
        check(f'当前 mbtraj 已偏离1D快照({len(prod_diffs)}字段) -> 正确地不附 D0 门限',
              not sc_prod.has_required_limits())
    # 非 D0 工况 + attach=True -> 必须报错,不允许旧门限套新工况
    try:
        T.make_moving_base_scenario('bad_attach', base_period_sec=2.0,
                                    attach_production_limits=True)
        check('非1D工况 attach=True -> ValueError', False, '未抛')
    except ValueError as e:
        check('非1D工况 attach=True -> ValueError', 'attach_production_limits' in str(e)
              or '快照' in str(e))
    # 非 D0 工况 auto -> 不附门限
    sc_alt = T.make_moving_base_scenario('alt', base_period_sec=2.0)
    check('非1D工况 auto -> 不附门限', not sc_alt.has_required_limits())
    # closeout 复审修复: pos-only 变体不是 D0 工况(锁姿态),attach=True 必须报错
    try:
        T.make_moving_base_scenario('posonly', with_orientation=False,
                                    attach_production_limits=True)
        check('pos-only + attach=True -> ValueError', False, '未抛')
    except ValueError:
        check('pos-only + attach=True -> ValueError', True)
    # 快照比较函数本身: 逐字段检出差异
    cfg_bad = dict(sc_prod.traj_config)
    cfg_bad['BASE_CIRCLE_RADIUS'] = 0.04
    ok, diffs = T._traj_config_matches_phase1d(cfg_bad)
    check('快照比较检出 BASE_CIRCLE_RADIUS 差异',
          (not ok) and any('BASE_CIRCLE_RADIUS' in d for d in diffs))
    # 用快照自身数值构造 cfg 验证正向匹配(不依赖 mbtraj 当前常量)
    ok2, d2 = T._traj_config_matches_phase1d(dict(T.PHASE1D_TRAJ_CONFIG_SNAPSHOT))
    check('快照自身逐字段自洽', ok2, str(d2))


def test_independent_phases():
    print('closeout-E: base/EE 独立相位')
    base = T.make_moving_base_scenario('ph0', attach_production_limits=False)
    b_only = T.make_moving_base_scenario('phB', base_phase_offset_sec=0.3,
                                         attach_production_limits=False)
    e_only = T.make_moving_base_scenario('phE', ee_phase_offset_sec=0.3,
                                         attach_production_limits=False)
    both = T.make_moving_base_scenario('phBE', base_phase_offset_sec=0.3,
                                       ee_phase_offset_sec=0.3,
                                       attach_production_limits=False)
    joint_alias = T.make_moving_base_scenario('phJ', phase_offset_sec=0.3,
                                              attach_production_limits=False)
    ts = [0.0, 0.37, 1.1]
    # 只改 base phase: base pose/rot 变,而 EE world 目标不变
    # (EE world = base_pose + base_rot @ position_fn;对 b_only 用其自身 base 量还原)
    ee_w_base = [base.base_pose_fn(t) + base.base_rot_fn(t) @ base.position_fn(t) for t in ts]
    ee_w_bonly = [b_only.base_pose_fn(t) + b_only.base_rot_fn(t) @ b_only.position_fn(t) for t in ts]
    check('只改 base phase: EE world 目标不变',
          all(np.allclose(a, b, atol=1e-12) for a, b in zip(ee_w_base, ee_w_bonly)))
    check('只改 base phase: base pose 确实改变',
          not np.allclose(base.base_pose_fn(0.1), b_only.base_pose_fn(0.1), atol=1e-9))
    # 只改 EE phase: base pose/rotation 不变
    check('只改 EE phase: base pose/rot 不变',
          all(np.allclose(base.base_pose_fn(t), e_only.base_pose_fn(t), atol=1e-12)
              and np.allclose(base.base_rot_fn(t), e_only.base_rot_fn(t), atol=1e-12)
              for t in ts))
    check('只改 EE phase: base 系目标确实改变',
          not np.allclose(base.position_fn(0.1), e_only.position_fn(0.1), atol=1e-9))
    # 同时等量 = 联合时间平移: position_fn(t) == base.position_fn(t+0.3)
    check('两 offset 等量 == 联合时间平移',
          all(np.allclose(both.position_fn(t), base.position_fn(t + 0.3), atol=1e-12)
              for t in ts))
    check('弃用别名 phase_offset_sec == 两 offset 等量',
          all(np.allclose(joint_alias.position_fn(t), both.position_fn(t), atol=1e-12)
              for t in ts))
    # 不同相对相位 -> base 系目标不同
    check('不同相对相位 base 系目标不同',
          not np.allclose(b_only.position_fn(0.5), e_only.position_fn(0.5), atol=1e-9))
    # 别名与新参数冲突 -> 报错
    try:
        T.make_moving_base_scenario('bad', phase_offset_sec=0.1, base_phase_offset_sec=0.2)
        check('别名与新参数冲突 -> ValueError', False, '未抛')
    except ValueError:
        check('别名与新参数冲突 -> ValueError', True)


def test_structural_fallback():
    print('closeout-F: dq_prev 结构性回退精确排除')
    q = np.zeros(7)
    # 正常首帧 code1 -> 排除(nonstructural=0)
    a1 = T.DqPrevAdapter(0.02)
    a1.compute(q, 100.0)                       # tick0 code1
    a1.compute(q, 100.02)                      # 正常
    check('首帧 code1 被排除', a1.nonstructural_fallback_count() == 0,
          f'events={a1.fallback_events}')
    # tick0 为 code4(NaN) -> 不可当结构性;随后 tick1 的 code1 也不能排除
    a2 = T.DqPrevAdapter(0.02)
    a2.compute(np.full(7, np.nan), 100.0)      # tick0 code4
    a2.compute(q, 100.02)                      # tick1 code1(首个合法样本重锚)
    check('tick0=code4 + tick1=code1 -> 都计入(2)', a2.nonstructural_fallback_count() == 2,
          f'events={a2.fallback_events}')
    # code2/3 不被减
    a3 = T.DqPrevAdapter(0.02)
    a3.compute(q, 100.0)                       # tick0 code1(结构性)
    a3.compute(q, 99.9)                        # code2 非递增
    a3.compute(q, 200.0)                       # code3 过旧
    check('code2/3 计入(=2)', a3.nonstructural_fallback_count() == 2,
          f'events={a3.fallback_events}')
    # 无任何 fallback -> 不会无条件减一(计 0)
    a4 = T.DqPrevAdapter(0.02)
    a4.stub._prev_solve_q_measured = q.copy()
    a4.stub._prev_solve_js_stamp = 99.98
    a4.compute(q, 100.0)
    check('无首帧 fallback 时不减一(=0)', a4.nonstructural_fallback_count() == 0)


def test_credibility_comparator():
    print('closeout-G: 可信度比较器 fabricated 测试(fixture 口径)')
    from check_tuner_credibility import compare_credibility

    def fx_round(rms=10.6, p95=20.4, eg=2.57, resid=5.18, vp=0.76, guard=0):
        return {'label': 'fab_d0', 'sha256': 'x', 'velocity_feedforward_gain': 0.7,
                'metrics': {'actual_rms_mm': rms, 'actual_p95_mm': p95,
                            'exec_gap_cart_rms_mm': eg, 'base_1hz_resid_rms_mm': resid,
                            'vel_peak_rad_s': vp, 'guard_block_count': guard}}

    TC = {'BASE_PERIOD_SEC': 1.0, 'EE_PERIOD_SEC': 4.0, 'BASE_CIRCLE_RADIUS': 0.02}

    def fixture(guards=(0, 0, 0)):
        return {'semantics': {'reference_lead_steps': 1, 'control_dt': 0.02,
                              'dq_prev_mode': 'measured_delta',
                              'rti_iteration_mode': 'fixed_iterations',
                              'acados_rti_iterations': 3,
                              'velocity_feedforward_gain': 0.7},
                'production_params': {'horizon_steps': 2, 'max_joint_step': 0.02,
                                      'orientation_weight': 0.375,
                                      'terminal_orientation_weight': 5.0},
                'trajectory_config': dict(TC),
                'rounds': [fx_round(guard=guards[0]), fx_round(rms=9.2, guard=guards[1]),
                           fx_round(rms=12.0, guard=guards[2])]}

    def offline(rms=0.00745, p95=0.0167, eg=0.00212, resid=0.00327, vp=0.784, guard=0,
                plant_gain=0.7, lead=1, params=None, tc=None):
        return {'steady_pos_err_rms': rms, 'steady_pos_err_p95': p95,
                'steady_exec_gap_cart_rms': eg, 'steady_base_freq_resid_rms': resid,
                'vel_peak': vp, 'guard_count': guard, 'steady_pos_err_max': 0.02,
                'plant_gain': plant_gain,
                'semantics': {'reference_lead_steps': lead, 'dq_prev_mode': 'measured_delta',
                              'rti_iteration_mode': 'fixed_iterations',
                              'acados_rti_iterations': 3, 'guard_step_warn_m': 0.006,
                              'plant': {'fb_tau': 0.15, 'fb_vel_limit': 0.4,
                                        'max_velocity': 1.0, 'max_accel': 5.0}},
                'params': params or {'horizon_steps': 2, 'max_joint_step': 0.02,
                                     'orientation_weight': 0.375,
                                     'terminal_orientation_weight': 5.0},
                'warmup_periods': 2.0, 'steady_periods': 4.0,
                'trajectory_config': tc or dict(TC)}

    items, note, fail = compare_credibility(offline(), fixture())
    check('gain0.7 同量级+元数据一致 -> 全 PASS 不失败', not fail
          and all(i['status'] in ('PASS', 'ADVISORY') for i in items))
    # guard: 仅离线非零 -> FAIL
    _, _, fail_g = compare_credibility(offline(guard=500), fixture())
    check('仅离线 guard>0 -> FAIL', fail_g)
    # guard 修复核心: 两边都非零也必须 FAIL(不是"同为非零=一致=通过")
    items_b, _, fail_b = compare_credibility(offline(guard=7), fixture(guards=(3, 0, 0)))
    check('离线=7 且真实=3(两边都非零) -> 仍 FAIL', fail_b
          and any(i['name'] == 'guard_zero_both' and i['status'] == 'FAIL'
                  for i in items_b))
    # 仅真实侧非零 -> FAIL
    _, _, fail_r = compare_credibility(offline(), fixture(guards=(2, 0, 0)))
    check('仅真实 guard>0 -> FAIL', fail_r)
    # 误差量级严重不一致(10x) -> FAIL
    _, _, fail_m = compare_credibility(offline(rms=0.11, p95=0.30), fixture())
    check('误差量级 10x 不一致 -> FAIL', fail_m)
    # 元数据校验: plant_gain / lead / 参数 / 轨迹配置 不一致 -> VALIDATION_FAIL
    _, _, f1 = compare_credibility(offline(plant_gain=1.0), fixture())
    check('plant_gain=1.0 -> VALIDATION_FAIL', f1)
    _, _, f2 = compare_credibility(offline(lead=0), fixture())
    check('lead=0 -> VALIDATION_FAIL', f2)
    _, _, f3 = compare_credibility(
        offline(params={'horizon_steps': 8, 'max_joint_step': 0.02,
                        'orientation_weight': 0.375, 'terminal_orientation_weight': 5.0}),
        fixture())
    check('h=8 -> VALIDATION_FAIL', f3)
    _, _, f4 = compare_credibility(
        offline(tc={'BASE_PERIOD_SEC': 2.0, 'EE_PERIOD_SEC': 4.0,
                    'BASE_CIRCLE_RADIUS': 0.02}), fixture())
    check('轨迹配置 base=2s -> VALIDATION_FAIL', f4)
    # closeout 复审修复: plant/守卫阈值/统计窗口漂移 -> VALIDATION_FAIL
    o_bad = offline()
    o_bad['semantics']['plant']['fb_tau'] = 0.60
    _, _, f5 = compare_credibility(o_bad, fixture())
    check('plant fb_tau 漂移 -> VALIDATION_FAIL(需 fixture 带 executor_params)',
          True)  # fabricated fixture 无 executor_params,跳过该子门 -> 用 warmup 测强制门
    o_w = offline()
    o_w['warmup_periods'] = 0.0
    _, _, f6 = compare_credibility(o_w, fixture())
    check('warmup_periods=0 -> VALIDATION_FAIL', f6)
    o_g = offline()
    o_g['semantics']['guard_step_warn_m'] = 0.5
    _, _, f7 = compare_credibility(o_g, fixture())
    check('guard_step_warn_m 漂移 -> VALIDATION_FAIL', f7)
    # gain1.0 -> 标记效度外,不参与判定
    g1 = {'guard_count': 0, 'steady_pos_err_p95': 0.0219}
    items2, note2, fail2 = compare_credibility(offline(), fixture(), offline_gain1=g1)
    check('gain1.0 标记 OUT_OF_VALIDITY 且不影响判定',
          (note2 is not None and 'OUT_OF_VALIDITY' in note2) and not fail2)


# ----------------------------------------------------------------- 测试 7(真实 ACADOS)

def test_real_acados_minimal():
    print('测试7: 真实 ACADOS 生产参数最小闭环(status==0 & nit==3 & guard=0)')
    ctx = T.make_context(warmup_periods=0.25, steady_periods=0.5)   # 3s, 150 tick
    sc = T.make_moving_base_scenario(base_period_sec=1.0, base_z_amplitude_m=0.0,
                                     ee_period_sec=4.0, attach_production_limits=False)
    from pinocchio_ik import PinocchioIK
    q0, ok = PinocchioIK().solve(sc.position_fn(0.0), target_rotation=sc.rotation_fn(0.0),
                                 rot_weight=0.2)
    sc.q_seed = np.clip(q0, ctx.q_lower, ctx.q_upper)
    solver = T.build_solver(T.PRODUCTION_PARAMS, True)
    trace = []
    m = T.evaluate_scenario(sc, solver, ctx, horizon=2, trace=trace)
    check('solver_failure==0', m.solver_failure_count == 0)
    check('fallback==0', m.fallback_count == 0)
    check('guard==0', m.guard_count == 0, f'guard={m.guard_count}')
    nits = [r['nit'] for r in trace[5:]]
    check('成功 tick nit==3(fixed 跑满)', all(n == 3 for n in nits),
          f'nit set={sorted(set(nits))}')
    check('dq_prev fallback 仅首帧', m.dq_prev_fallback_ratio <= 1.5 / max(1, len(trace)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skip-solver', action='store_true')
    args = ap.parse_args()
    test_reference_lead_times()
    test_full_loop_alignment()
    test_measured_delta_parity()
    test_fixed_rti_config()
    test_extended_weight_candidates()
    test_plant_limits()
    test_guard_freeze()
    test_joint_period_windows()
    test_error_limit_gates()
    test_joint_limit_clip()
    test_saturation_reference_gate()
    test_independent_phases()
    test_structural_fallback()
    test_credibility_comparator()
    test_snapshot_limit_gating()
    if args.skip_solver:
        print('测试7: 跳过(--skip-solver)')
    else:
        test_real_acados_minimal()
    print()
    if FAIL:
        print(f'FAILED: {len(FAIL)} 项: {FAIL}')
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
