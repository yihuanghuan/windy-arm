import numpy as np
import pinocchio as pin
import pytest

from camera_world_hold_core import Link6Kinematics, default_urdf, pose_deadzone


def test_stationary_noise_does_not_accumulate_a_joint_reference():
    k=Link6Kinematics(default_urdf(), parent_to_camera=pin.SE3.Identity())
    q=np.array([.1,1.,-1.,.1,.4,.1]);target=k.link6_fk(q)
    reference=q.copy()
    rng=np.random.default_rng(42)
    for _ in range(1000):
        noise_p=rng.uniform(-.0003,.0003,3)
        noise_r=rng.uniform(-.001,.001,3)
        measured=pin.SE3(pin.exp3(noise_r)@target.rotation,target.translation+noise_p)
        step=k.step(q,measured,target,.04,position_deadband=.001,rotation_deadband=.005)
        reference+=step['dq']*.04
        assert np.linalg.norm(step['position_error'])>0  # Do not hide error in logs.
        np.testing.assert_array_equal(step['twist'],np.zeros(6))
    np.testing.assert_array_equal(reference,q)


def test_larger_disturbance_keeps_correct_direction_and_raw_error():
    k=Link6Kinematics(default_urdf(),parent_to_camera=pin.SE3.Identity())
    q=np.array([.1,1.,-1.,.1,.4,.1]);target=k.link6_fk(q)
    measured=pin.SE3(pin.exp3(np.array([0.,0.,.02]))@target.rotation,
                    target.translation+np.array([.003,0.,0.]))
    s=k.step(q,measured,target,.04,position_deadband=.001,rotation_deadband=.005)
    np.testing.assert_allclose(s['position_error'],[-.003,0,0],atol=1e-12)
    np.testing.assert_allclose(s['twist'][:3],[-.003,0,0],atol=1e-12)
    np.testing.assert_allclose(s['twist'][3:],[0,0,-.0225],atol=1e-12)
    assert np.linalg.norm(s['dq'])>0


def test_deadzone_continuous_boundary_and_invalid_radius():
    np.testing.assert_array_equal(pose_deadzone(np.zeros(3),0),np.zeros(3))
    np.testing.assert_allclose(pose_deadzone(np.array([.0010001,0,0]),.001),[1e-7,0,0])
    for r in [-1.,float('nan'),float('inf')]:
        with pytest.raises(ValueError):pose_deadzone(np.ones(3),r)
