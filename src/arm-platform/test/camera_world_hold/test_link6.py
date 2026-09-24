"""Reconstruct and control the rotating output, not its housing-mounted camera."""
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'demo'))
from camera_world_hold_core import Link6Kinematics, default_urdf


def make_kin():
    return Link6Kinematics(default_urdf(),parent_to_camera=pin.SE3(
        pin.rpy.rpyToMatrix(.2,-.3,.4),np.array([-.0042,.018,.037])))


def test_fixed_camera_q6_changes_reconstructed_link6_only():
    kin=make_kin();q=np.array([.2,.6,-.7,.5,.4,-.3])
    wb=pin.SE3(pin.rpy.rpyToMatrix(.4,-.2,.8),np.array([2.,-1.,.5]))
    wc=wb*kin.fk(q);we=kin.reconstruct(q,wc)
    np.testing.assert_allclose(we.homogeneous,(wb*kin.link6_fk(q)).homogeneous,atol=1e-12)
    changed=q.copy();changed[5]+=.2
    np.testing.assert_allclose((wb*kin.fk(changed)).homogeneous,wc.homogeneous,atol=1e-12)
    after=kin.reconstruct(changed,wc)
    np.testing.assert_allclose(after.translation,we.translation,atol=1e-12)
    np.testing.assert_allclose(after.rotation,we.rotation@pin.exp3(np.array([.2,0.,0.])),atol=1e-12)
    np.testing.assert_allclose(kin.housing_link6(changed).rotation,
                               pin.exp3(np.array([changed[5],0.,0.])),atol=1e-12)
    correction=kin.step(changed,after,we,.04)
    assert correction['dq'][5]<-.01


def test_all_six_world_jacobian_columns_against_reconstruction_difference():
    kin=make_kin();rng=np.random.default_rng(27);maximum=0.
    for _ in range(8):
        q=rng.uniform(-.7,.7,6)
        wb=pin.SE3(pin.exp3(rng.normal(0,.5,3)),rng.normal(0,2,3))
        we=kin.reconstruct(q,wb*kin.fk(q));j=kin.world_jacobian(q,we)
        for k in range(6):
            d=np.eye(6)[k]*1e-7
            plus=kin.reconstruct(q+d,wb*kin.fk(q+d))
            minus=kin.reconstruct(q-d,wb*kin.fk(q-d))
            numeric=np.r_[(plus.translation-minus.translation)/2e-7,
                          pin.log3(plus.rotation@minus.rotation.T)/2e-7]
            maximum=max(maximum,float(np.abs(numeric-j[:,k]).max()))
        np.testing.assert_allclose(j[:3,5],np.zeros(3),atol=1e-12)
        np.testing.assert_allclose(j[3:,5],we.rotation[:,0],atol=1e-12)
        lwa=pin.computeFrameJacobian(kin.model,kin.data,q,kin.link6_id,pin.LOCAL_WORLD_ALIGNED)
        np.testing.assert_allclose(j,np.vstack((wb.rotation@lwa[:3],wb.rotation@lwa[3:])),atol=1e-12)
    print('link6 world Jacobian finite difference max error:',maximum)
    assert maximum<2e-7


def test_general_and_singular_configuration_diagnostics():
    kin=make_kin();q=np.array([.2,.6,-.7,.5,.4,-.3]);we=kin.link6_fk(q)
    result=kin.step(q,we,we,.04)
    assert result['jacobian_rank']==6
    assert np.isfinite(result['jacobian_condition'])
    print('general rank/singular values/condition:',result['jacobian_rank'],
          result['jacobian_singular_values'],result['jacobian_condition'])
    q=np.zeros(6);we=kin.link6_fk(q);result=kin.step(q,we,we,.04)
    print('zero configuration rank/singular values/condition:',result['jacobian_rank'],
          result['jacobian_singular_values'],result['jacobian_condition'])
    assert np.isfinite(result['q_cmd']).all()


def test_six_world_disturbance_correction_directions():
    kin=make_kin();q=np.array([.2,.6,-.7,.5,.4,-.3])
    wb=pin.SE3(pin.rpy.rpyToMatrix(.3,-.4,.5),np.array([.2,-.3,.1]))
    desired=wb*kin.link6_fk(q)
    for axis in range(6):
        for sign in [-1,1]:
            current=desired.copy()
            if axis<3:current.translation[axis]+=sign*.003
            else:current.rotation=pin.exp3(np.eye(3)[axis-3]*sign*.01)@current.rotation
            result=kin.step(q,current,desired,.04)
            realized=kin.world_jacobian(q,current)@result['dq']
            assert realized[axis]*sign<0
            assert np.dot(realized,result['twist'])>0
