"""Synthetic and saved-data tests only; policy has no serial transport."""
import importlib.util
import ast
from pathlib import Path
import numpy as np
import pytest

PATH=Path(__file__).resolve().parents[2]/'tools/gravity_tuning_policy.py'
spec=importlib.util.spec_from_file_location('tuning_policy',PATH)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def monitor():
    return m.Monitor([0]*6,[0]*6,.9,[6]*3+[1]*3,[.2]*3+[.02]*3,
                     m.MAX_ENVELOPE,[-3]*6,[3]*6)


def update(mon,t,q=None,v=None,**kw):
    defaults=dict(feedback_age=.001,desk_margin=.06,self_margin=.01)
    defaults.update(kw)
    return mon.update(t,[0]*6 if q is None else q,[0]*6 if v is None else v,**defaults)


@pytest.mark.parametrize('lam,kp,kd',[(1.01,[6]*6,[.2]*6),(.3,[6]*6,[.2]*6),(.9,[31]*3+[1]*3,[.2]*3+[.02]*3),(.9,[6]*3+[1]*3,[1]*3+[10]*3)])
def test_outside_bounds_rejected(lam,kp,kd):
    with pytest.raises(ValueError):m.validate_parameters(lam,kp,kd)


@pytest.mark.parametrize('fault',[{'communication_level':'COMM_HARD_ABORT'},{'feedback_age':.026},{'feedback_age':.051},{'desk_margin':.049},{'self_margin':.002},{'operator_stop':True}])
def test_hard_faults(fault):
    assert update(monitor(),0,**fault)['status']=='HARD_ABORT'


def test_nonfinite_aborts():
    assert update(monitor(),0,q=[np.nan]*6)['status']=='HARD_ABORT'


def test_80_percent_bound():
    q=np.zeros(6);q[4]=.8*m.MAX_ENVELOPE[4]
    assert update(monitor(),0,q=q)['reason']=='80 percent displacement envelope'


def test_small_constant_slow_motion_allowed():
    mon=monitor();rows=[]
    for t in np.arange(0,.25,.005):
        q=np.zeros(6);q[4]=.08*t;v=np.zeros(6);v[4]=.08
        rows.append(update(mon,float(t),q,v))
    assert all(r['status']=='NORMAL' for r in rows)
    assert m.classify_completed(rows,.245)=='SOFT_FAIL'  # safe drift, not stable


def test_isolated_raw_velocity_spike_not_abort():
    mon=monitor()
    for i in range(20):
        v=np.zeros(6);v[4]=.9 if i==10 else 0
        assert update(mon,i*.005,v=v)['status']=='NORMAL'


def test_comm_warning_does_not_abort_fresh_motion_state():
    assert update(monitor(),0,communication_level='COMM_WARNING')['status']=='NORMAL'


def test_sustained_velocity_position_agreement_aborts():
    mon=monitor();row=None
    for i in range(10):
        q=np.zeros(6);q[4]=.9*i*.005;v=np.zeros(6);v[4]=.9
        row=update(mon,i*.005,q,v)
        if row['status']=='HARD_ABORT':break
    assert row['reason']=='sustained velocity with position evidence'


def test_acceleration_stops_before_envelope():
    mon=monitor()
    for i in range(40):
        t=i*.005;q=np.zeros(6);q[4]=2*t*t;v=np.zeros(6);v[4]=4*t
        row=update(mon,t,q,v)
        if row['status']=='HARD_ABORT':break
    assert row['reason'] in ('continued position-derived acceleration','persistent expanding direction mismatch; no retry')
    assert abs(q[4])<.8*m.MAX_ENVELOPE[4]


def test_insufficient_duration_never_normal():
    assert m.classify_completed([update(monitor(),0)],.063)=='SOFT_FAIL'


def test_static_screen_and_lambda_step():
    candidates=m.feasible_candidates([0,2.2,1,0,0,0],m.MAX_ENVELOPE,[30]*3+[5]*3,[.4,.7,.9,1],previous_lambda=.99)
    assert [r['feasible'] for r in candidates]==[False,False,True,True]


def test_policy_has_no_transport_or_io_imports():
    tree=ast.parse(PATH.read_text())
    imports=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Import):imports.extend(a.name for a in node.names)
        elif isinstance(node,ast.ImportFrom):imports.append(node.module)
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Name):
            assert node.func.id not in ('open','exec','eval','__import__')
    assert set(imports)=={'collections','numpy'}
