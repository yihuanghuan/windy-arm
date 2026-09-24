"""Kinematic feedforward, sample-time alignment and failure fallback (no hardware)."""
import numpy as np
import pinocchio as pin
import pytest

from camera_world_hold_core import Link6Kinematics, CameraKinematics, default_urdf, clamp_norm
from camera_world_hold_velocity import JointMotionHistory, VelocitySamples


def setup():
    kin = Link6Kinematics(default_urdf(), parent_to_camera=pin.SE3(
        pin.rpy.rpyToMatrix(.2,-.3,.4), np.array([-.0042,.018,.037])))
    q = np.array([.2,.6,-.7,.5,.8,-.3])
    wb = pin.SE3(pin.rpy.rpyToMatrix(.4,-.2,.8), np.array([2.,-1.,.5]))
    return kin, q, wb


def cfg():
    return dict(velocity_feedforward_linear_gain=1., velocity_feedforward_angular_gain=1.,
                twist_receive_timeout=.2, feedforward_max_sample_lag=.08, pose_header_max_age=1.,
                feedforward_linear_deadband=0., feedforward_angular_deadband=0.,
                feedforward_max_linear_velocity=.03, feedforward_max_angular_velocity=.10)


def derivative(a, b, dt):
    return np.r_[(b.translation-a.translation)/dt, pin.log3(b.rotation@a.rotation.T)/dt]


def test_self_motion_cancels_and_q6_remains_in_measured_link6_twist():
    kin,q,wb=setup();wc=wb*kin.fk(q)
    for dq in [np.array([.1,-.03,.02,-.1,.06,.15]), np.array([0.,0.,0.,0.,0.,.2])]:
        eps=1e-6
        vc=derivative(wb*kin.fk(q-eps*dq),wb*kin.fk(q+eps*dq),2*eps)
        result=kin.external_velocity(q,dq,wc,vc)
        np.testing.assert_allclose(result['external_link6_twist'],0.,atol=2e-9)
        ve=derivative(wb*kin.link6_fk(q-eps*dq),wb*kin.link6_fk(q+eps*dq),2*eps)
        np.testing.assert_allclose(result['measured_link6_twist'],ve,atol=2e-9)
    np.testing.assert_allclose(vc,0.,atol=2e-9)  # Camera on stator cannot see q6.
    assert np.linalg.norm(result['measured_link6_twist'][3:])>.19


def test_combined_base_and_joint_motion_matches_finite_difference_at_link6():
    kin,q,wb=setup();dq=np.array([.1,-.03,.02,-.1,.06,.15])
    v,w=np.array([.02,-.01,.015]),np.array([.08,-.04,.06]);eps=1e-6
    def moving_base(t):
        return pin.SE3(pin.exp3(w*t)@wb.rotation,wb.translation+v*t)
    vc=derivative(moving_base(-eps)*kin.fk(q-eps*dq),
                  moving_base(eps)*kin.fk(q+eps*dq),2*eps)
    result=kin.external_velocity(q,dq,wb*kin.fk(q),vc)
    base_at_e=derivative(moving_base(-eps)*kin.link6_fk(q),
                        moving_base(eps)*kin.link6_fk(q),2*eps)
    full_e=derivative(moving_base(-eps)*kin.link6_fk(q-eps*dq),
                     moving_base(eps)*kin.link6_fk(q+eps*dq),2*eps)
    np.testing.assert_allclose(result['external_link6_twist'],base_at_e,atol=2e-9)
    np.testing.assert_allclose(result['measured_link6_twist'],full_e,atol=2e-9)
    # This check is sensitive to omitting or reversing the camera/EE lever arm.
    assert np.linalg.norm(base_at_e[:3]-(v+np.cross(w,(wb*kin.fk(q)).translation-wb.translation)))>1e-3


def test_zero_feedforward_preserves_p_and_combined_command_is_clamped():
    kin,q,wb=setup();we=wb*kin.link6_fk(q);target=we.copy();target.translation+=np.array([.02,-.01,0.])
    baseline=kin.step(q,we,target,.04)
    disabled=kin.step(q,we,target,.04,feedforward=np.zeros(6))
    for key in baseline:np.testing.assert_array_equal(baseline[key],disabled[key])
    out=kin.step(q,we,we,.04,feedforward=np.array([1.,2.,3.,4.,5.,6.]),
                 max_linear_velocity=.04,max_angular_velocity=.1,max_joint_velocity=.12)
    assert np.linalg.norm(out['twist'][:3])==pytest.approx(.04)
    assert np.linalg.norm(out['twist'][3:])==pytest.approx(.1)
    assert max(abs(out['dq']))<=.12000001
    with pytest.raises(ValueError):kin.step(q,we,we,.04,feedforward=[np.nan]*6)


def test_encoder_slope_uses_image_time_not_latest_feedback_or_noisy_motor_velocity():
    kin,q,wb=setup();history=JointMotionHistory();qd=np.array([.1,-.03,.02,-.1,.06,.15]);t0=1_790_000_000.
    for dt in np.arange(-.3,.31,.01):history.add(t0+dt,q+qd*((t0+dt)-t0))
    qm,dqm=history.at(t0)
    np.testing.assert_allclose(qm,q,atol=1e-12);np.testing.assert_allclose(dqm,qd,atol=1e-12)
    with pytest.raises(ValueError):history.at(t0+.8)
    history.add(t0-.5,q)  # clock rewind invalidates old slope history
    with pytest.raises(ValueError):history.at(t0-.5)
    history=JointMotionHistory(.2)
    for dt in [-.09,-.08,-.07,.07,.08,.09]:history.add(10+dt,q)
    with pytest.raises(ValueError,match='gap'):history.at(10.)


def populated():
    kin,q,wb=setup();samples=VelocitySamples();dq=np.array([.1,-.03,.02,-.1,.06,.15])
    for stamp in np.arange(9.7,10.31,.01):samples.joints.add(stamp,q+dq*(stamp-10.))
    wc=wb*kin.fk(q);jc=CameraKinematics.jacobian(kin,q)
    robot=np.r_[wc.rotation@(jc@dq)[:3],wc.rotation@(jc@dq)[3:]]
    samples.add_pose(10.,100.,wc);samples.add_twist(10.,100.,robot)
    return kin,samples,robot,wc


def test_synchronized_self_motion_is_zero_despite_newer_joint_samples():
    kin,samples,_,_=populated()
    ff,status,details=samples.evaluate(kin,100.02,10.32,10.,cfg())
    assert status['ff_valid'];np.testing.assert_allclose(ff,0.,atol=1e-12)
    assert status['ff_sensor_latency']==pytest.approx(.32)
    np.testing.assert_allclose(details['synchronized_joint_velocity'],[.1,-.03,.02,-.1,.06,.15],atol=1e-12)


def test_external_feedforward_direction_and_gain_deadband_limits():
    kin,samples,robot,wc=populated();samples.add_twist(10.,100.,robot+np.array([.02,0,0,0,0,0]))
    c=cfg();c['velocity_feedforward_linear_gain']=.5;c['feedforward_linear_deadband']=.003
    ff,status,_=samples.evaluate(kin,100.02,10.32,10.,c)
    assert status['ff_valid'];np.testing.assert_allclose(ff,[-.0085,0,0,0,0,0],atol=1e-12)
    samples.add_twist(10.,100.,robot+np.array([1.,0,0,0,0,1.]))
    ff,status,_=samples.evaluate(kin,100.02,10.32,10.,c)
    assert np.linalg.norm(ff[:3])<=.030000001 and np.linalg.norm(ff[3:])<=.100000001


@pytest.mark.parametrize('problem',['missing','stale','mismatch','old_pair','bad_frame','joint_gap','future'])
def test_unavailable_velocity_falls_back_to_p_only(problem):
    kin,samples,_,_=populated();now,ros_now,latest=100.02,10.32,10.
    if problem=='missing':samples.twists.clear()
    if problem=='stale':now=100.3
    if problem=='mismatch':samples.poses.clear();samples.add_pose(10.01,100.,pin.SE3.Identity());latest=10.01
    if problem=='old_pair':latest=10.12
    if problem=='bad_frame':samples.invalidate('wrong frame')
    if problem=='joint_gap':samples.joints.rows.clear()
    if problem=='future':ros_now=9.
    ff,status,details=samples.evaluate(kin,now,ros_now,latest,cfg())
    assert not status['ff_valid'] and not details
    np.testing.assert_array_equal(ff,np.zeros(6))


def test_disabled_feedforward_needs_no_velocity_inputs():
    kin,_,_=setup();c=cfg();c['velocity_feedforward_linear_gain']=c['velocity_feedforward_angular_gain']=0.
    ff,status,_=VelocitySamples().evaluate(kin,100.,10.,10.,c)
    assert not status['ff_enabled'] and status['ff_reason']=='disabled'
    np.testing.assert_array_equal(ff,np.zeros(6))


def test_kinematic_moving_base_feedforward_reduces_error_without_pose_observer():
    kin,q0,wb0=setup()
    def run(gain):
        q=q0.copy();dq=np.zeros(6);desired=wb0*kin.link6_fk(q);errors=[]
        dt=.02;v=np.array([.006,0.,0.])
        for t in np.arange(0.,5.,dt):
            wb=wb0.copy();wb.translation+=v*t;wc=wb*kin.fk(q);we=kin.reconstruct(q,wc)
            jc=CameraKinematics.jacobian(kin,q);own=jc@dq
            vc=np.r_[wc.rotation@own[:3]+v,wc.rotation@own[3:]]
            ext=kin.external_velocity(q,dq,wc,vc)['external_link6_twist']
            result=kin.step(q,we,desired,dt,kp_position=1.2,kp_rotation=1.,
                            max_joint_velocity=.5,damping_lambda=.005,feedforward=-gain*ext)
            errors.append(np.linalg.norm(we.translation-desired.translation));dq=result['dq'];q=result['q_cmd']
        return max(errors)
    p_only,with_ff=run(0.),run(.5)
    assert with_ff<p_only*.65
