from pathlib import Path
import numpy as np
import pytest
import yaml
from task_space_smc.core.manual_guidance import Guidance
from task_space_smc.core.supported_startup import propose_supported_hold

ROOT=Path(__file__).resolve().parents[2]

def setup():
    cfg=yaml.safe_load((ROOT/'config/arm6_familiarization.yaml').read_text())
    p=Guidance(ROOT/'config/arm6.urdf',cfg)
    samples=[dict(t=i*.01,q=[.05,1.26,-1.35,0.,0.,0.],dq=[-.005]*6,
                  effort=[-.5,-1.6,.75,.02,.02,.01]) for i in range(201)]
    return p,samples

def test_supported_proposal_matches_measured_support_without_gravity_or_gain_change():
    p,s=setup();r=propose_supported_hold(s,p);c=r['command']
    total=np.array(c['p'])*(np.array(c['position'])-s[-1]['q'])-np.array(c['d'])*s[-1]['dq']
    np.testing.assert_allclose(total,s[-1]['effort'],atol=1e-12)
    assert c['current']==[0.]*6 and not r['support_is_command_readback']
    assert c['p']==p.hold_p.tolist() and c['d']==p.hold_d.tolist()
    first=p.command(s[-1]['q'],s[-1]['dq'],3.,0.)
    np.testing.assert_allclose(first['estimated_total_torque'],total,atol=1e-12)
    assert first['delta_total_command_from_previous_cycle']==[0.]*6

@pytest.mark.parametrize('bad',['short','gap','backtime','nan','seven','moving','limit','effort','noisy','reference'])
def test_supported_proposal_refuses_bad_feedback(bad):
    p,s=setup()
    if bad=='short':s=s[:100]
    if bad=='gap':
        for r in s[100:]:r['t']+=.06
    if bad=='backtime':s[-1]['t']=s[-2]['t']
    if bad=='nan':s[-1]['effort'][0]=float('nan')
    if bad=='seven':
        for r in s:r['q'].append(0.)
    if bad=='moving':s[-1]['q'][0]+=.003
    if bad=='limit':s[-1]['q'][1]=1.48
    if bad=='effort':s[-1]['effort'][0]=5.
    if bad=='noisy':s[-1]['effort'][0]=2.
    if bad=='reference':
        for r in s:r['q'][1]=1.469;r['effort'][1]=1.
    with pytest.raises(ValueError):propose_supported_hold(s,p)
