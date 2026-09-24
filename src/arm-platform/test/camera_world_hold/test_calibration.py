"""Known synthetic hand-eye truth; never produces hardware mounting values."""
from pathlib import Path
import sys

import numpy as np
import pinocchio as pin
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'demo'))
from camera_mount_calibration_core import solve_rotation,angle_deg
from calibrate_camera_mount_rotation import apply_config, Calibration
from camera_world_hold_core import default_urdf


def dataset(noise=0., coaxial=False, move_base=False):
    rng=np.random.default_rng(53)
    x=pin.rpy.rpyToMatrix(.7,-.5,1.2)
    rwb=pin.rpy.rpyToMatrix(-.2,.3,-.4)
    pwb=np.array([2.,-1.,.5]);plc=np.array([-.0042,.018,.037])
    samples=[]
    for i in range(10):
        rl=pin.exp3(np.array([0.,0.,i*.2]) if coaxial else rng.normal(0,.6,3))
        pl=rng.uniform(-.2,.2,3)
        rc=rwb@rl@x@pin.exp3(rng.normal(0,noise,3))
        pc=pwb+rwb@(pl+rl@plc)
        if move_base:pc+=np.array([i*.04,0,0])
        samples.append(dict(robot_rotation=rl.tolist(),robot_position=pl.tolist(),
                            camera_rotation=rc.tolist(),camera_position=pc.tolist()))
    return samples,x,plc


def test_exact_rotation_and_optical_conversion():
    samples,x,p=dataset()
    tf=pin.SE3(pin.Quaternion(np.array([.5,-.5,.5,.5])).matrix(),np.zeros(3))
    result=solve_rotation(samples,tf,p)
    assert result['accepted']
    assert angle_deg(np.array(result['R_link6_camera'])@x.T)<1e-7
    assert result['max_rotation_residual_deg']<1e-7
    assert result['max_translation_residual_m']<1e-10
    left=pin.Quaternion(np.array(result['joint6_to_left_optical_xyzw'])).matrix()
    np.testing.assert_allclose(left@tf.rotation,x,atol=1e-10)
    assert result['usable_relative_pairs']>=12


def test_noise_and_coaxial_rejection():
    samples,x,p=dataset(noise=.002)
    result=solve_rotation(samples,pin.SE3.Identity(),p)
    assert result['accepted']
    assert angle_deg(np.array(result['R_link6_camera'])@x.T)<.2
    samples,_,p=dataset(coaxial=True)
    with pytest.raises(ValueError,match='coaxial'):
        solve_rotation(samples,pin.SE3.Identity(),p)


def test_moving_base_or_bad_translation_rejected():
    samples,_,p=dataset(move_base=True)
    result=solve_rotation(samples,pin.SE3.Identity(),p)
    assert not result['accepted']
    assert result['max_translation_residual_m']>.1
    samples,_,p=dataset()
    result=solve_rotation(samples,pin.SE3.Identity(),p+np.array([.1,.1,0]))
    assert not result['accepted']


def test_rotation_outlier_not_silently_removed():
    samples,_,p=dataset()
    samples[4]['camera_rotation']=(np.array(samples[4]['camera_rotation'])@pin.exp3(np.array([.3,0,0]))).tolist()
    result=solve_rotation(samples,pin.SE3.Identity(),p)
    assert not result['accepted']
    assert result['max_rotation_residual_deg']>5.


def test_update_is_gated_and_preserves_gains(tmp_path):
    source=Path(default_urdf()).with_name('camera_world_hold.yaml')
    config=tmp_path/'hold.yaml';config.write_text(source.read_text())
    samples,_,p=dataset()
    import yaml
    before=yaml.safe_load(config.read_text())['camera_world_hold']['ros__parameters']
    parent=before['parent_frame']
    result=solve_rotation(samples,pin.SE3.Identity(),p,parent_frame=parent)
    apply_config(config,result)
    after=yaml.safe_load(config.read_text())['camera_world_hold']['ros__parameters']
    for key in before:
        if key != 'joint6_to_left_optical_xyzw':
            assert after[key] == before[key]
    assert 'publish_command: false' in config.read_text()
    previous=config.read_text();result['accepted']=False
    with pytest.raises(ValueError):apply_config(config,result)
    assert config.read_text()==previous
    assert config.with_suffix('.yaml.before_calibration').exists()


def test_short_pause_during_guidance_is_not_a_settled_capture():
    from types import SimpleNamespace
    import time
    now=time.monotonic()
    model=pin.buildModelFromUrdf(default_urdf())
    node=SimpleNamespace(args=SimpleNamespace(settle_seconds=2.,average_seconds=.5),valid=True,
                         valid_received=now,model=model,data=model.createData(),
                         frame_id=model.getFrameId('link6'),
                         get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000)))
    node.poses=[(t,now,pin.SE3.Identity()) for t in np.linspace(99.5,100.,16)]
    node.joints=[(t,now,np.array([.03 if t<99. else 0.,0.,0.,0.,0.,0.]))
                 for t in np.linspace(97.5,100.,251)]
    with pytest.raises(ValueError,match='not settled'):
        Calibration.snapshot(node)
    node.joints=[(t,now,np.zeros(6)) for t in np.linspace(97.5,100.,251)]
    sample=Calibration.snapshot(node)
    assert sample['frames']>=10
    assert sample['joint_span_deg']==0.
