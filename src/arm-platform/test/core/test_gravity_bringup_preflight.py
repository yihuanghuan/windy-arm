"""Offline-only feasibility and restore reference tests; no device access."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

P=Path(__file__).resolve().parents[2]/'tools/gravity_bringup_preflight.py'
spec=importlib.util.spec_from_file_location('bringup_preview',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
Q=np.array([-.031853675842285156,1.494049072265625,-1.415846824645996,.0024938583374023438,.16706371307373047,.002109527587890625])
U=P.parents[1]/'config/arm6.urdf'

def test_raw_urdf_no_scaling_and_no_hardware_entry():
    r=m.preview(Q,U)
    assert r['g_model']==pytest.approx(m.gravity(Q,U))
    assert r['serial_open_count']==r['serial_write_count']==0
    assert r['hardware_executable_available'] is False
    assert [s['lambda_value'] for s in r['stages']]==[.1,.2,.35,.5,.75,1.]
    for s in r['stages']:
        assert s['candidate_tau_gc']==pytest.approx(s['lambda_value']*np.array(r['g_model']))
        assert s['theoretical_arrays']['0x12'][:6]==pytest.approx(m.SIGNS*np.array(s['candidate_tau_gc']))
        assert s['theoretical_arrays']['0x12'][6]==0
        assert s['q_start_live'] is None and s['serial_tx_frames']==0

@pytest.mark.parametrize('q',[[0]*5,[0]*7,[np.nan]*6,[np.inf]*6])
def test_bad_input_rejected(q):
    with pytest.raises(ValueError):m.preview(q,U)

def test_envelope_never_expands_and_reserves_limit_headroom():
    e,margin=m.envelope(Q)
    assert np.all(e<=m.MAX_ENVELOPE)
    assert e[2]==pytest.approx((margin[2]-.15)/2)
    q=Q.copy();q[2]=-1.55
    with pytest.raises(ValueError):m.envelope(q)

def test_low_lambda_cannot_restore_in_nominal_static_model():
    r=m.preview(Q,U)
    for s in r['stages'][:5]:
        assert not s['static_feasible_within_authorized_max'][1]
        assert not s['static_feasible_within_restore_tolerance'][1]
    assert r['stages'][-1]['static_feasible_within_restore_tolerance']==[True]*6
    assert r['minimum_lambda_for_nominal_static_restore_tolerance'][1]>.99

def test_quintic_endpoints_and_peak_bounds():
    q0=Q+np.array([.005,.005,-.001,.01,.01,.01]);T=m.restore_duration(q0,Q)
    for t,expect in [(0,q0),(T,Q)]:
        q,v,a=m.restore_reference(q0,Q,t,T)
        assert q==pytest.approx(expect);assert v==pytest.approx(np.zeros(6));assert a==pytest.approx(np.zeros(6))
    samples=[m.restore_reference(q0,Q,t,T) for t in np.linspace(0,T,1001)]
    assert max(np.max(abs(x[1])) for x in samples)<=m.RESTORE_VMAX+1e-12
    assert max(np.max(abs(x[2])) for x in samples)<=m.RESTORE_AMAX+1e-12
    assert all(np.all(x[0]>=np.minimum(q0,Q)-1e-12) and np.all(x[0]<=np.maximum(q0,Q)+1e-12) for x in samples)

@pytest.mark.parametrize('key',['hard_abort','communication_ok','workspace_verified','stationary','gravity_sign_verified','gravity_scale_verified','static_feasible'])
def test_restore_refuses_each_missing_prerequisite(key):
    args=dict(hard_abort=False,communication_ok=True,workspace_verified=True,stationary=True,gravity_sign_verified=True,gravity_scale_verified=True,static_feasible=True)
    args[key]=(key=='hard_abort')
    assert not m.restore_gate(**args)

def test_restore_out_of_envelope_rejected():
    with pytest.raises(ValueError):m.restore_reference(Q+np.array([.031,0,0,0,0,0]),Q,0,2)
