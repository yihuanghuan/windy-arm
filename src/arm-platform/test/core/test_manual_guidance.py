from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest
import yaml

from task_space_smc.core.manual_guidance import Guidance

ROOT = Path(__file__).resolve().parents[2]
Q = np.array([-.02, 1.2, -1.2, .1, .1, .1])


def make(**overrides):
    cfg = yaml.safe_load((ROOT / 'config/arm6_familiarization.yaml').read_text())
    cfg.update(overrides)
    result = Guidance(ROOT / 'config/arm6.urdf', cfg)
    result.set_hold_command(dict(position=Q.tolist(), velocity=[0.]*6, current=[0.]*6,
                                 p=cfg['hold_p'], d=cfg['hold_d']))
    return result


def advance(policy):
    for t in np.arange(0, 6.02, .01):
        result = policy.command(Q, np.zeros(6), t, 0.)
    return result


def test_guide_has_no_return_spring_and_only_one_damping_term():
    g = make()
    out = advance(g)
    assert out['phase'] == 'GUIDE'
    assert out['p'] == [0.] * 6
    moved = Q + .01
    out = g.command(moved, np.full(6, .1), 6.02, 0.)
    np.testing.assert_array_equal(out['position'], moved)
    np.testing.assert_allclose(out['current'], g.gravity(moved), atol=1e-12)
    np.testing.assert_allclose(out['d'], g.kd)
    # The model output has no -Kd*dq term; firmware Kd applies it exactly once.
    assert all(len(out[k]) == 6 for k in ('position', 'velocity', 'current', 'p', 'd'))


def test_offset_once_and_gravity_matches_potential_gradient():
    offset = np.array([.01, -.02, .03, 0, 0, 0])
    g = make(model_zero_offset_rad=offset.tolist())
    out = advance(g)
    q_model = Q + offset
    gradient = []
    for i in range(6):
        delta = np.eye(6)[i] * 1e-6
        gradient.append((pin.computePotentialEnergy(g.model, g.data, q_model + delta) -
                         pin.computePotentialEnergy(g.model, g.data, q_model - delta)) / 2e-6)
    np.testing.assert_allclose(out['current'], gradient, atol=1e-8)


def test_ramp_preserves_position_support_until_gravity_ready():
    g = make()
    a = g.command(Q, np.zeros(6), 0., 0.)
    assert a['current'] == [0.] * 6
    assert a['p'] == g.hold_p.tolist()
    previous = np.zeros(6)
    for t in np.arange(.01, 3.01, .01):
        a = g.command(Q, np.zeros(6), t, 0.)
        assert np.all(np.abs(np.array(a['current']) - previous) <= g.slew * .01 + 1e-12)
        np.testing.assert_allclose(a['p'], g.hold_p)
        previous = np.array(a['current'])


@pytest.mark.parametrize('bad', ['nan', 'seven', 'stale', 'speed', 'limit', 'loop_gap'])
def test_faults_latch_and_never_rearm(bad):
    g = make()
    g.command(Q, np.zeros(6), 0., 0.)
    q, dq, now, age = Q.copy(), np.zeros(6), .01, 0.
    if bad == 'nan': q[0] = np.nan
    if bad == 'seven': q = np.zeros(7)
    if bad == 'stale': age = .1
    if bad == 'speed': dq[4] = .6
    if bad == 'limit': q[1] = 1.55
    if bad == 'loop_gap': now = .1
    with pytest.raises(ValueError): g.command(q, dq, now, age)
    with pytest.raises(ValueError): g.command(Q, np.zeros(6), .02, 0.)


def test_torque_budget_refuses_instead_of_silent_undercompensation():
    g = make(torque_limits=[.1] * 6)
    with pytest.raises(ValueError, match='torque budget'):
        g.command(Q, np.zeros(6), 0., 0.)


@pytest.mark.parametrize('cfg', [dict(damping=[0.] * 7),
                                dict(model_zero_offset_rad=[float('nan')] * 6),
                                dict(torque_limits=[27.] * 6)])
def test_invalid_configuration_fails_before_io(cfg):
    with pytest.raises(ValueError): make(**cfg)


def test_guide_warning_and_short_sustained_speed_do_not_fault():
    g = make(guide_sustained_abort_speed=.8); advance(g)
    v = np.zeros(6); v[4] = .6
    out = g.command(Q, v, 6.02, 0.)
    assert out['phase'] == 'GUIDE' and g.speed_guard.status['warning'][4]
    v[4] = .85
    for t in [6.03, 6.05, 6.07, 6.09, 6.10]:
        g.command(Q, v, t, 0.)
    assert g.fault is None
    with pytest.raises(ValueError, match='joint5.*sustained'):
        g.command(Q, v, 6.12, 0.)
    with pytest.raises(ValueError): g.command(Q, np.zeros(6), 6.13, 0.)


def test_instant_speed_fault_and_reset_per_joint():
    from task_space_smc.core.speed_safety import SpeedSafety
    g = make(guide_sustained_abort_speed=.8); guard = g.speed_guard
    v = np.zeros(6); v[0] = .85
    guard.update(v, 'GUIDE', 0.)
    guard.update(v, 'GUIDE', .07)
    v[0] = .79; v[1] = .85
    guard.update(v, 'GUIDE', .075)
    assert guard.status['sustained_duration'][0] == 0.
    v[0] = .85
    guard.update(v, 'GUIDE', .09)
    guard.update(v, 'GUIDE', .15)
    assert guard.fault is None
    guard.update(v, 'GUIDE', .156)
    assert guard.status['fault_axes'] == [False, True, False, False, False, False]
    instant = SpeedSafety(dict(guard.cfg, **guard.manual, guide_manual_sustained_duration=guard.manual_duration))
    v = np.zeros(6); v[5] = 1.25
    instant.update(v, 'GUIDE', 0.)
    assert instant.status['fault_types'][5] == 'instantaneous'
    instant.update(np.zeros(6), 'GUIDE', 1.)
    assert instant.fault  # no reset after fault


@pytest.mark.parametrize('phase', ['RAMP_GRAVITY', 'RELEASE_STIFFNESS'])
def test_transition_keeps_immediate_point_five_limit(phase):
    guard = make().speed_guard
    guard.update(np.full(6, .51), phase, 0.)
    assert guard.status['fault_types'] == ['transition'] * 6


def test_threshold_equality_and_phase_change_reset():
    guard = make(guide_sustained_abort_speed=.8).speed_guard
    guard.update(np.full(6, .8), 'GUIDE', 0.)
    guard.update(np.full(6, .8), 'GUIDE', 1.)
    assert guard.fault is None and guard.status['sustained_duration'] == [0.] * 6
    guard.update(np.full(6, .85), 'GUIDE', 1.01)
    guard.update(np.zeros(6), 'RAMP_GRAVITY', 1.02)
    guard.update(np.full(6, .85), 'GUIDE', 1.03)
    assert guard.status['sustained_duration'] == [0.] * 6


def test_current_one_rad_profile_preserves_instant_limit_and_sustained_abort():
    g = make(); advance(g)
    assert g.speed_guard.cfg['guide_sustained_abort_speed'] == 1.0
    assert g.speed_guard.cfg['guide_instant_abort_speed'] == 1.2
    v = np.zeros(6); v[1] = .985
    for t in np.arange(6.02, 6.23, .01):
        g.command(Q, v, float(t), 0.)
    assert g.fault is None
    v[1] = 1.05
    for t in [6.23, 6.25, 6.27, 6.29, 6.30]:
        g.command(Q, v, t, 0.)
    with pytest.raises(ValueError, match='joint2.*sustained'):
        g.command(Q, v, 6.32, 0.)


def test_bumpless_first_command_preserves_loaded_pd_and_h1_total():
    g = make()
    seed = dict(position=(Q+np.array([.01,-.05,.03,0,.01,0])).tolist(),
                velocity=[0.]*6,current=[.1,-.2,.05,0,0,0],p=g.hold_p.tolist(),d=g.hold_d.tolist())
    g.set_hold_command(seed)
    dq=np.array([.01,-.01,.01,0,0,0])
    first=g.command(Q,dq,0.,0.)
    for k in ('position','velocity','current','p','d'):
        np.testing.assert_array_equal(first[k],seed[k])
    total=np.array(seed['p'])*(np.array(seed['position'])-Q)-np.array(seed['d'])*dq+seed['current']
    for t in np.arange(.01,3.,.01):
        out=g.command(Q,dq,float(t),0.)
        np.testing.assert_allclose(out['estimated_total_torque'],total,atol=1e-12)
        np.testing.assert_allclose(out['delta_total_command_from_previous_cycle'],0,atol=1e-12)
    for t in np.arange(3.,6.02,.01):
        out=g.command(Q,dq,float(t),0.)
        assert max(abs(np.array(out['delta_total_command_from_previous_cycle'])))<.05
    np.testing.assert_allclose(out['estimated_total_torque'],g.gravity(Q)-g.kd*dq)


def test_missing_seed_and_early_manual_are_rejected():
    g=make();g.seed=None
    with pytest.raises(ValueError,match='authoritative'):g.command(Q,np.zeros(6),0.,0.)
    g=make()
    with pytest.raises(ValueError,match='healthy GUIDE'):g.enable_manual_push()


@pytest.mark.parametrize('warning,sustained',[(.8,1.5),(1.,1.8)])
def test_manual_push_speed_profile_and_timer_reset(warning,sustained):
    g=make(guide_manual_warning_speed=[warning]*6,
           guide_manual_sustained_abort_speed=[sustained]*6)
    advance(g);g.enable_manual_push()
    v=np.zeros(6);v[1]=warning+.1
    out=g.command(Q,v,6.02,0.)
    assert out['phase']=='MANUAL_PUSH' and g.speed_guard.status['warning'][1]
    v[1]=sustained+.1
    for t in [6.03,6.05,6.07,6.08]:g.command(Q,v,t,0.)
    assert g.fault is None
    v[1]=sustained-.1;g.command(Q,v,6.09,0.)
    assert g.speed_guard.status['sustained_duration'][1]==0.
    v[1]=sustained+.1
    for t in [6.10,6.12,6.14,6.16,6.18,6.20,6.21]:g.command(Q,v,t,0.)
    with pytest.raises(ValueError,match='sustained'):g.command(Q,v,6.23,0.)
    with pytest.raises(ValueError):g.command(Q,np.zeros(6),6.24,0.)
    g=make();advance(g);g.enable_manual_push();v[1]=2.1
    with pytest.raises(ValueError,match='instantaneous'):g.command(Q,v,6.02,0.)


def test_manual_profile_cannot_bypass_transition_or_joint_limit():
    g=make();g.manual_active=True
    v=np.zeros(6);v[1]=.51
    with pytest.raises(ValueError,match='transition'):g.command(Q,v,0.,0.)
    g=make();advance(g);g.enable_manual_push()
    q=Q.copy();q[1]=g.upper[1]-.05
    with pytest.raises(ValueError,match='joint limit'):g.command(q,np.zeros(6),6.02,0.)


@pytest.mark.parametrize('axis',range(6))
def test_current_manual_profile_allows_17_and_keeps_independent_hard_timers(axis):
    guard=make().speed_guard
    np.testing.assert_array_equal(guard.manual['guide_manual_warning_speed'],[1.]*3+[1.5]*3)
    np.testing.assert_array_equal(guard.manual['guide_manual_sustained_abort_speed'],[1.8]*6)
    np.testing.assert_array_equal(guard.manual['guide_manual_instant_abort_speed'],[2.]*3+[2.2]*3)
    v=np.zeros(6);v[axis]=1.7
    for t in np.arange(0.,1.,.01):guard.update(v,'MANUAL_PUSH',float(t))
    assert not guard.fault and guard.status['warning'][axis]
    assert guard.status['sustained_duration'][axis]==0.
    v[axis]=1.9;guard.update(v,'MANUAL_PUSH',1.)
    other=(axis+1)%6;v[other]=1.9;guard.update(v,'MANUAL_PUSH',1.05)
    assert not guard.fault
    v[axis]=1.7;guard.update(v,'MANUAL_PUSH',1.10)
    assert guard.status['sustained_duration'][axis]==0.
    v[axis]=1.9;guard.update(v,'MANUAL_PUSH',1.13)
    assert not guard.fault
    guard.update(v,'MANUAL_PUSH',1.171)
    assert guard.fault and guard.status['fault_axes'][other]
    assert sum(guard.status['fault_axes'])==1


@pytest.mark.parametrize('phase',['HOLD','HANDOVER_START','HANDOVER_GRAVITY_MATCH','RAMP','KP_WITHDRAWAL'])
def test_wider_manual_profile_does_not_change_transition_limit(phase):
    guard=make().speed_guard
    guard.update([.51,0,0,0,0,0],phase,0.)
    assert guard.fault and guard.status['fault_types'][0]=='transition'


@pytest.mark.parametrize('axis',range(6))
def test_wrist_instant_extension_is_per_axis_and_keeps_sustained_abort(axis):
    guard=make().speed_guard
    v=np.zeros(6);v[axis]=2.1
    guard.update(v,'MANUAL_PUSH',0.)
    if axis<3:
        assert guard.fault and guard.status['fault_types'][axis]=='instantaneous'
    else:
        guard.update(v,'MANUAL_PUSH',.05)
        assert not guard.fault and guard.status['warning'][axis]
        v[axis]=1.7;guard.update(v,'MANUAL_PUSH',.06)
        assert guard.status['sustained_duration'][axis]==0.
        v[axis]=2.1;guard.update(v,'MANUAL_PUSH',.07)
        guard.update(v,'MANUAL_PUSH',.191)
        assert guard.fault and guard.status['fault_types'][axis]=='sustained'
    guard=make().speed_guard
    v[axis]=2.3;guard.update(v,'MANUAL_PUSH',0.)
    assert guard.fault and guard.status['fault_types'][axis]=='instantaneous'


@pytest.mark.parametrize('phase',['GUIDE','HANDOVER_GRAVITY_MATCH','KP_WITHDRAWAL'])
def test_wrist_extension_does_not_apply_outside_manual(phase):
    guard=make().speed_guard
    guard.update([0,0,0,1.3,0,0],phase,0.)
    assert guard.fault


@pytest.mark.parametrize('axis',range(6))
def test_wrist_warning_change_does_not_change_abort_limits(axis):
    guard=make().speed_guard
    v=np.zeros(6);v[axis]=1.2
    guard.update(v,'MANUAL_PUSH',0.)
    assert not guard.fault
    assert guard.status['warning'][axis] == (axis<3)
    v[axis]=1.5;guard.update(v,'MANUAL_PUSH',.01)
    assert guard.status['warning'][axis] == (axis<3)
    v[axis]=1.51;guard.update(v,'MANUAL_PUSH',.02)
    assert guard.status['warning'][axis] and not guard.fault
    v[axis]=1.9;guard.update(v,'MANUAL_PUSH',.03)
    guard.update(v,'MANUAL_PUSH',.151)
    assert guard.fault and guard.status['fault_types'][axis]=='sustained'
