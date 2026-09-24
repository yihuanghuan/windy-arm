import numpy as np
import pytest

from camera_world_hold_position_bridge import integrate_position, integration_interval, PositionBridge


def test_scheduling_jitter_never_produces_a_catchup_position_jump():
    for elapsed in (.005,.03,.04,.08,.1):
        dt=integration_interval(elapsed)
        target=integrate_position(np.zeros(6),np.full(6,.12),dt,.12)
        np.testing.assert_allclose(target,np.full(6,.12*min(elapsed,.03)))
    for elapsed in (0.,-.1,.10001,np.nan,np.inf):
        with pytest.raises(ValueError):integration_interval(elapsed)


def test_integrates_from_support_reference_instead_of_resetting_to_feedback():
    seed = np.array([0., -.08, .04, 0., 0., 0.])
    q = seed.copy()
    rate = np.array([.01, -.02, .03, 0., 0., 0.])
    for _ in range(100):
        q = integrate_position(q, rate, .01, .03)
    np.testing.assert_allclose(q, seed+rate, atol=1e-12)
    np.testing.assert_array_equal(integrate_position(q, np.zeros(6), .01, .03), q)


def test_uniform_rate_clamp_and_bad_inputs():
    step = integrate_position(np.zeros(6), [.1,.05,0,0,0,0], .01, .03)
    np.testing.assert_allclose(step, [.0003,.00015,0,0,0,0])
    for dt in (0., -.1, .031, np.nan):
        with pytest.raises(ValueError):
            integrate_position(np.zeros(6), np.zeros(6), dt, .03)
    with pytest.raises(ValueError):
        integrate_position(np.zeros(6), [np.nan]*6, .01, .03)


def test_trial_speed_remains_a_uniform_bound_when_software_ceiling_increases():
    for speed in (.03,.08,.12,.2):
        q=integrate_position(np.zeros(6), [.4,-.2,.1,0,0,0], .01, speed)
        np.testing.assert_allclose(q,speed*.01*np.array([1,-.5,.25,0,0,0]))
    with pytest.raises(ValueError):
        integrate_position(np.zeros(6), np.zeros(6), .01, .20001)


def test_rate_rejects_stale_message_and_accepts_named_finite_axes():
    from types import SimpleNamespace as S
    from sensor_msgs.msg import JointState
    fake = S(get_clock=lambda:S(now=lambda:S(nanoseconds=10_000_000_000)),
             kin=S(names=['joint'+str(i) for i in range(1,7)]), rate_error=None)
    msg = JointState();msg.header.stamp.sec=10
    msg.name=fake.kin.names[::-1];msg.velocity=[.01]*6
    PositionBridge.on_rate(fake,msg)
    assert fake.rate_error is None
    np.testing.assert_allclose(fake.rate,[.01]*6)
    msg.velocity=[.12]*6
    PositionBridge.on_rate(fake,msg)
    assert fake.rate_error is None
    msg.velocity=[.20001]*6
    PositionBridge.on_rate(fake,msg)
    assert 'excessive' in fake.rate_error
    msg.header.stamp.sec=9
    PositionBridge.on_rate(fake,msg)
    assert 'stale' in fake.rate_error


def test_exit_disables_visible_hold_before_driver_handover(monkeypatch):
    from types import SimpleNamespace as S
    import camera_world_hold_position_bridge as bridge
    events=[]
    client=S(wait_for_service=lambda **kw:True,
             call_async=lambda req:events.append(('enable',req.data)))
    fake=S(prepared=True,create_client=lambda *args:client,
           destroy_client=lambda c:events.append('destroy client'))
    monkeypatch.setattr(bridge.rclpy,'spin_until_future_complete',lambda *a,**kw:None)
    monkeypatch.setattr(bridge.Probe,'close',lambda self:events.append('driver hold'))
    # Use a genuine subclass instance so super().close follows the normal path.
    node=PositionBridge.__new__(PositionBridge)
    node.prepared=fake.prepared
    node.create_client=fake.create_client
    node.destroy_client=fake.destroy_client
    node.close()
    assert events==[('enable',False),'destroy client','driver hold']
    events.clear();node.prepared=False;node.close()
    assert events==['driver hold']  # Preview cannot disable another controller.


def test_disable_service_failure_still_returns_driver_hold(monkeypatch):
    import camera_world_hold_position_bridge as bridge
    events=[]
    node=PositionBridge.__new__(PositionBridge)
    node.prepared=True
    def fail(*args):
        raise RuntimeError('service unavailable')
    node.create_client=fail
    monkeypatch.setattr(bridge.Probe,'close',lambda self:events.append('driver hold'))
    with pytest.raises(RuntimeError,match='service unavailable'):
        node.close()
    assert events==['driver hold']
