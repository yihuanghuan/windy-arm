"""Offline dynamics regression: no ROS nodes or hardware modules are constructed."""
import ast
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pinocchio as pin
import pytest
from task_space_smc.core.model import FloatingBaseArmModel, arm6_urdf_path, arm6_tool_urdf_path
from task_space_smc.core.gravity_semantics import explicit_offsets, urdf_feedforward
from task_space_smc.core.manual_guidance_preview import UrdfGravity, preview, SIGNS

Q=np.array([-.031854,1.356337,-1.334973,.001726,.007481,.001726])

@pytest.mark.parametrize('path',[arm6_urdf_path(),arm6_tool_urdf_path()])
def test_single_urdf_gravity_matches_potential_gradient_and_smc(path):
    fixed=UrdfGravity(path);g=fixed.gravity(Q)
    h=1e-6
    gradient=[]
    for j in range(6):
        delta=np.eye(6)[j]*h
        gradient.append((pin.computePotentialEnergy(fixed.model,fixed.data,Q+delta)-pin.computePotentialEnergy(fixed.model,fixed.data,Q-delta))/(2*h))
    np.testing.assert_allclose(g,gradient,atol=1e-8)
    smc=FloatingBaseArmModel(path,ee_frame_name='link6',apply_armature=False)
    np.testing.assert_array_equal(smc.model_joint_angles(Q),Q)
    q=smc.make_q_full(np.zeros(3),[1,0,0,0],Q)
    tau=smc.compute_rnea_arm_torque(q,np.zeros(12),np.zeros(6),np.zeros(6))
    np.testing.assert_allclose(tau,g,atol=1e-12)
    np.testing.assert_allclose(smc.compute_arm_forward_dynamics(q,np.zeros(12),np.zeros(6),g),0,atol=1e-11)

@pytest.mark.parametrize('file,cls',[('phase4_gravity_compensation.py','Compensator'),('phase5_inertia_id.py','InertiaId')])
def test_actual_phase_feedforward_ignores_historical_ratio(file,cls):
    # Compile just the real numerical method, avoiding hardware-capable module imports.
    path=Path(arm6_urdf_path()).parent.parent/'task_space_smc/hw'/file
    tree=ast.parse(path.read_text());c=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==cls)
    fn=next(n for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='feedforward')
    ns={'np':np,'urdf_feedforward':urdf_feedforward}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),ns)
    fixed=UrdfGravity(arm6_urdf_path())
    fake=SimpleNamespace(model=fixed,offset=np.zeros(6),ratio=np.array([1.265,1.349,1.228,.972,1.021,1.]))
    np.testing.assert_allclose(ns['feedforward'](fake,Q),fixed.gravity(Q),atol=1e-12)

def test_offsets_are_explicit_and_shared_by_kinematics_and_dynamics():
    offset=np.array([.01,-.02,.03,0,0,0])
    smc=FloatingBaseArmModel(arm6_urdf_path(),apply_armature=False,zero_offset_rad=offset)
    q=smc.make_q_full(np.zeros(3),[1,0,0,0],Q)
    np.testing.assert_allclose(q[7:],Q+offset)
    np.testing.assert_allclose(smc.gravity_arm_torque(q),UrdfGravity(arm6_urdf_path()).gravity(Q+offset),atol=1e-12)
    with pytest.raises(ValueError):explicit_offsets(6,[float('nan')]*6)

def test_preview_six_joint_mapping_float32_and_limits():
    out=preview(Q);arrays=out['theoretical_float32_protocol_arrays']
    np.testing.assert_allclose(arrays['0x10'][:6],SIGNS*Q,rtol=1e-7)
    np.testing.assert_allclose(arrays['0x12'][:6],SIGNS*np.array(out['gravity_nm']),rtol=1e-7)
    assert all(len(a)==7 and a[6]==0 for a in arrays.values())
    assert out['hardware_ready'] is False and out['MotorControl']['enable']==1
    assert out['gravity_fraction_rated'][1]<.25
    assert abs(out['gravity_nm'][5])<.001
    with pytest.raises(ValueError):preview([float('nan')]*6)
    bad=Q.copy();bad[2]=-1.55
    with pytest.raises(ValueError):preview(bad)
