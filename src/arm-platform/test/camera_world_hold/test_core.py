"""Synthetic geometry is ONLY used for these numerical tests."""
from pathlib import Path
import sys

import numpy as np
import pinocchio as pin
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'demo'))
from camera_world_hold_core import CameraKinematics, default_urdf, ordered_joints, rotation, transform


@pytest.fixture
def kin():
    return CameraKinematics(default_urdf(), parent_to_camera=pin.SE3(
        pin.rpy.rpyToMatrix(.3, -.4, .7), np.array([-.0042, .018, .037])))


def test_camera_jacobian_finite_difference(kin):
    rng = np.random.default_rng(24)
    maximum = 0.
    for _ in range(8):
        q = rng.uniform(-.7, .7, 6)
        t = kin.fk(q)
        j = kin.jacobian(q)
        for axis in range(6):
            delta = np.eye(6)[axis] * 1e-7
            after, before = kin.fk(q+delta), kin.fk(q-delta)
            numerical = np.r_[t.rotation.T @ (after.translation-before.translation)/(2e-7),
                              pin.log3(before.rotation.T @ after.rotation)/(2e-7)]
            maximum = max(maximum, np.max(np.abs(j[:, axis]-numerical)))
    print('camera LOCAL Jacobian maximum finite difference error:', maximum)
    assert maximum < 2e-7


def test_offset_is_not_link_origin(kin):
    q = np.array([.1,.4,-.5,.3,.2,-.4])
    t = kin.fk(q)
    parent = kin.data.oMf[kin.parent_id].copy()
    j = kin.jacobian(q)
    jp = pin.computeFrameJacobian(kin.model, kin.data, q, kin.parent_id, pin.LOCAL_WORLD_ALIGNED)
    expected = jp[:3] - pin.skew(t.translation-parent.translation) @ jp[3:]
    np.testing.assert_allclose(t.rotation @ j[:3], expected, atol=1e-12)
    assert np.linalg.norm(expected-jp[:3]) > .01


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1, 1])
def test_world_disturbance_corrects_each_axis(kin, axis, sign):
    q = np.array([.2,.6,-.7,.5,.4,-.3])
    world_base = pin.SE3(pin.rpy.rpyToMatrix(.4,-.2,.8), np.array([2.,-1.,.5]))
    desired = world_base * kin.fk(q)
    disturbance = pin.SE3.Identity()
    if axis < 3:
        disturbance.translation[axis] = sign*.004
    else:
        disturbance.rotation = pin.exp3(np.eye(3)[axis-3]*sign*.012)
    moved_base = disturbance * world_base
    current = moved_base * kin.fk(q)
    initial = np.linalg.norm(np.r_[desired.translation-current.translation,
                                   pin.log3(desired.rotation @ current.rotation.T)])
    for _ in range(80):
        step = kin.step(q, current, desired, .04)
        q = step['q_cmd']
        current = moved_base * kin.fk(q)
    final = np.linalg.norm(np.r_[desired.translation-current.translation,
                                 pin.log3(desired.rotation @ current.rotation.T)])
    assert final < initial*.3, (axis, sign, initial, final)


def test_clamps_stationary_singular_and_limits(kin):
    q = np.zeros(6)
    t = kin.fk(q)
    zero = kin.step(q, t, t, .04)
    np.testing.assert_allclose(zero['q_cmd'], q, atol=1e-14)
    target = pin.SE3(pin.exp3(np.array([.8,.2,.1])) @ t.rotation, t.translation+10)
    step = kin.step(q, t, target, .04)
    assert np.linalg.norm(step['twist'][:3]) <= .05+1e-12
    assert np.linalg.norm(step['twist'][3:]) <= .2+1e-12
    assert np.max(np.abs(step['dq'])) <= .3+1e-12
    assert np.max(np.abs(step['q_cmd']-q)) <= .012+1e-12
    q = kin.model.upperPositionLimit.copy()
    step = kin.step(q, kin.fk(q), target, .04)
    assert np.all(step['q_cmd'] <= kin.model.upperPositionLimit)


def test_names_and_missing_geometry():
    names = [f'joint{i}' for i in range(1,7)]
    np.testing.assert_array_equal(ordered_joints(names[::-1], list(range(6))[::-1], names), range(6))
    with pytest.raises(ValueError):
        ordered_joints(names+['joint7'], list(range(7)), names)
    with pytest.raises(ValueError):
        rotation([0,0,0,0])
    with pytest.raises(ValueError):
        transform([0,np.nan,0], [0,0,0,1])
    with pytest.raises(ValueError):
        CameraKinematics(default_urdf()).jacobian(np.zeros(6))


def test_seven_dof_model():
    urdf = str(Path(default_urdf()).with_name('arm.urdf'))
    kin = CameraKinematics(urdf, 'link6', pin.SE3.Identity())
    assert kin.jacobian(np.zeros(7)).shape == (6,7)
    np.testing.assert_array_equal(kin.jacobian(np.zeros(7))[:,6], np.zeros(6))


def test_joint6_housing_camera_has_no_joint6_motion():
    mount=pin.SE3(pin.rpy.rpyToMatrix(.3,-.4,.7),np.array([-.0042,.018,.037]))
    kin=CameraKinematics(default_urdf(),'joint6_housing',mount)
    q=np.array([.2,.6,-.7,.5,.4,0.])
    moving=CameraKinematics(default_urdf(),'link6',mount)
    np.testing.assert_allclose(kin.fk(q).homogeneous,moving.fk(q).homogeneous,atol=1e-12)
    reference=kin.fk(q)
    q[5]=.8
    np.testing.assert_allclose(kin.fk(q).homogeneous,reference.homogeneous,atol=1e-12)
    j=kin.jacobian(q)
    np.testing.assert_array_equal(j[:,5],np.zeros(6))
    assert np.linalg.matrix_rank(j)==5
    for axis in range(6):
        delta=np.eye(6)[axis]*1e-7
        after,before=kin.fk(q+delta),kin.fk(q-delta)
        numerical=np.r_[reference.rotation.T@(after.translation-before.translation)/2e-7,
                        pin.log3(before.rotation.T@after.rotation)/2e-7]
        np.testing.assert_allclose(j[:,axis],numerical,atol=2e-7)
    desired=pin.SE3(pin.exp3(np.array([.01,-.02,.03]))@reference.rotation,
                    reference.translation+np.array([.001,.002,-.003]))
    result=kin.step(q,reference,desired,.04)
    assert result['dq'][5]==0.
    assert result['q_cmd'][5]==q[5]
