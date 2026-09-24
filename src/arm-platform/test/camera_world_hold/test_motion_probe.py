import numpy as np
import pytest

from camera_fk_motion_probe import Probe, blend, checked_offsets, motor_command, incremental_tracking_error
from camera_fk_motion_probe import preserve_support_reference


def test_position_profile_preserves_existing_support_and_wrist_reference():
    anchor=np.array([.1,1.2,-1.1,-.5,.5,.2])
    reference=anchor+np.array([.01,-.1,.03,.02,-.01,0.])
    old=np.array([30.,30.,30.,5.,5.,5.])
    target=np.array([35.,35.,35.,5.,5.,5.])
    for fraction in np.linspace(0,1,101):
        gains=old+(target-old)*fraction
        q=preserve_support_reference(anchor,reference,old,gains)
        np.testing.assert_allclose(gains*(q-anchor),old*(reference-anchor),atol=1e-14)
        np.testing.assert_array_equal(q[3:],reference[3:])
    for invalid in ([0.]*6,[np.nan]*6,[1.]*5):
        with pytest.raises(ValueError):
            preserve_support_reference(anchor,reference,old,invalid)


def test_tracking_guard_retains_static_support_offset_but_detects_new_error():
    q=np.zeros(6);reference=np.array([.081,.025,0.,0.,0.,0.])
    assert np.allclose(incremental_tracking_error(q,reference,q,reference),0.)
    target=reference.copy();target[1]+=.04
    assert np.isclose(incremental_tracking_error(q,target,q,reference)[1],.04)
    followed=q.copy();followed[1]=.04
    assert np.allclose(incremental_tracking_error(followed,target,q,reference),0.)
    followed[0]=.09
    assert np.max(abs(incremental_tracking_error(followed,target,q,reference)))>.08


def test_failure_after_prepare_before_first_publish_cancels_handover():
    # A serialization/validation exception must not leave the driver waiting for
    # a first command and then trip its handover watchdog.
    from types import SimpleNamespace
    events=[]
    fake=SimpleNamespace(prepared=True,active=False,
                         call=lambda name:events.append(name),
                         log=SimpleNamespace(close=lambda:events.append('close')),
                         destroy_node=lambda:events.append('destroy'))
    Probe.close(fake)
    assert events==['/student_arm_node/operator_hold','close','destroy']


def test_fault_rejecting_hold_still_flushes_evidence_and_destroys_node():
    from types import SimpleNamespace
    events=[]
    def reject(name):
        events.append(name)
        raise RuntimeError('fault latched')
    fake=SimpleNamespace(prepared=True,active=True,call=reject,
                         log=SimpleNamespace(close=lambda:events.append('close')),
                         destroy_node=lambda:events.append('destroy'))
    with pytest.raises(RuntimeError,match='fault latched'):
        Probe.close(fake)
    assert not fake.active
    assert events==['/student_arm_node/operator_hold','close','destroy']


def test_authoritative_json_integer_fields_serialize_exactly():
    seed={k:[0]*6 for k in ('position','velocity','current','p','d')}
    seed['p']=[30,30,30,5,5,5]
    m=motor_command(seed)
    for k,v in seed.items():
        assert list(getattr(m,k))==v
    assert m.enable==1
    seed['current'][0]=float('nan')
    with pytest.raises(ValueError):
        motor_command(seed)


def test_plan_rejects_nonfinite_and_excess_excursion():
    for x in (np.nan, np.inf, 5.01):
        with pytest.raises(ValueError):
            checked_offsets([{'offset_deg':[x,0,0,0,0,0]}],5.)
    with pytest.raises(ValueError):
        checked_offsets([{'offset_deg':[0]*6,'hold_seconds':.1}],5.)


def test_slow_ramp_has_bounded_velocity_no_overshoot_and_exact_endpoint():
    start=np.array([.1,.2,.3,.4,.5,.6])
    end=start+np.deg2rad([3,-5,0,2,0,0])
    times=np.arange(0,8,.001)
    q=np.array([blend(start,end,t,.025)[0] for t in times])
    assert np.max(abs(np.diff(q,axis=0)/.001))<=.025+1e-8
    assert np.all(q>=np.minimum(start,end)-1e-12)
    assert np.all(q<=np.maximum(start,end)+1e-12)
    assert np.array_equal(q[0],start)
    assert np.allclose(q[-1],end)
    assert np.array_equal(q[:,4],np.full(len(q),start[4]))
