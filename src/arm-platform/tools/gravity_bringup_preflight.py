"""OFFLINE ONLY: bounded gravity bring-up and return-to-start feasibility.

Consumes a saved PRE_SEND_SNAPSHOT, computes unscaled arm6 URDF gravity.
No ROS, serial, device opener, controller import, or execution mode.
Does not grant a hardware execution permit.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pinocchio as pin

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/arm-platform'))
from task_space_smc.core.approach_reference import quintic

KP=np.array([6.,6.,6.,1.,1.,1.])
KD=np.array([.3,.3,.3,.03,.03,.03])
SIGNS=np.array([1.,1.,1.,-1.,-1.,-1.])
LAMBDAS=(.1,.2,.35,.5,.75,1.)
MAX_ENVELOPE=np.array([.03,.03,.03,.05,.05,.05])
LOWER=np.array([-4.,-3.1415,-1.5708,-4.,-2.,-4.])
UPPER=np.array([4.,2.,1.5708,4.,2.,4.])
RATED=np.array([9.,9.,9.,1.6,1.6,1.6])
# Retained for the conservative offline proposal, not installed in an active controller.
JOINT_MARGIN=.15
RESTORE_TOL=4*.0003833770751953125  # four observed position quanta
RESTORE_VMAX=.03  # 1/6 of author's 0.18rad/s approach reference; PROPOSED only
RESTORE_AMAX=.06  # reaches proposed .03rad/s in >=0.5s; PROPOSED only
STAGE_SECONDS=.5


def vector(values):
    v=np.asarray(values,dtype=float)
    if v.shape!=(6,) or not np.isfinite(v).all():raise ValueError('six finite values required')
    return v


def gravity(q,urdf):
    q=vector(q)
    m=pin.buildModelFromUrdf(str(urdf))
    if m.nq!=6 or m.nv!=6 or list(m.names)[1:]!=[f'joint{i}' for i in range(1,7)]:
        raise ValueError('wrong URDF joints')
    g=np.asarray(pin.computeGeneralizedGravity(m,m.createData(),q)).copy()
    return vector(g)


def envelope(q):
    margin=np.minimum(vector(q)-LOWER,UPPER-vector(q))
    if np.min(margin)<=JOINT_MARGIN:raise ValueError('insufficient joint-limit margin')
    # Reserve half of the room beyond the retained .15rad margin for lag/uncertainty.
    return np.minimum(MAX_ENVELOPE,.5*(margin-JOINT_MARGIN)),margin


def restore_reference(q_now,q_start,elapsed,duration):
    a=vector(q_now);b=vector(q_start)
    if not np.isfinite(duration) or duration<=0:raise ValueError('invalid duration')
    if not np.isfinite(elapsed):raise ValueError('invalid elapsed time')
    if np.any(abs(a-b)>MAX_ENVELOPE):raise ValueError('return exceeds authorized envelope')
    s,ds,dds=quintic(float(elapsed)/duration)
    return a+s*(b-a),ds*(b-a)/duration,dds*(b-a)/(duration*duration)


def restore_duration(q_now,q_start):
    delta=float(np.max(abs(vector(q_start)-vector(q_now))))
    # Exact quintic peak factors: max s'=1.875, max |s''|=10/sqrt(3).
    return max(.5,1.875*delta/RESTORE_VMAX,np.sqrt((10/np.sqrt(3))*delta/RESTORE_AMAX))


def restore_gate(*,hard_abort,communication_ok,workspace_verified,stationary,
                 gravity_sign_verified,gravity_scale_verified,static_feasible):
    # No recovery from failures; kinematic reference alone never authorizes motion.
    return (not hard_abort and communication_ok and workspace_verified and stationary
            and gravity_sign_verified and gravity_scale_verified and static_feasible)


def preview(q,urdf):
    q=vector(q);g=gravity(q,urdf);env,margin=envelope(q)
    if np.any(abs(g)>RATED):raise ValueError('model gravity exceeds repository rated FF')
    stages=[]
    for lam in LAMBDAS:
        ff=lam*g
        residual=(1-lam)*g
        required_error=abs(residual)/KP
        # A necessary nominal static condition, not a hardware guarantee or friction model.
        feasible_authorized=required_error<=MAX_ENVELOPE
        feasible_settle=required_error<=RESTORE_TOL
        arrays={'0x10':(SIGNS*q).tolist()+[0.], '0x11':[0.]*7,
                '0x12':(SIGNS*ff).tolist()+[0.], '0x13':KP.tolist()+[0.], '0x14':KD.tolist()+[0.]}
        stages.append(dict(lambda_value=lam,execution_status='NOT EXECUTED',planned_observation_seconds=STAGE_SECONDS,
            q_start_live=None,q_end=None,delta_q=None,dq=None,sag_rate_j2_j3=None,
            g_from_saved_q=g.tolist(),candidate_tau_gc=ff.tolist(),
            hypothetical_zero_velocity_zero_error_proxy=ff.tolist(),
            residual_gravity_nominal=residual.tolist(),required_static_position_error_rad=required_error.tolist(),
            static_feasible_within_authorized_max=feasible_authorized.tolist(),
            static_feasible_within_restore_tolerance=feasible_settle.tolist(),
            executed_duration=0.,guard_utilization=None,serial_tx_frames=0,
            theoretical_arrays=arrays))
    minimum_lambda=np.maximum(0,1-KP*RESTORE_TOL/np.maximum(abs(g),np.finfo(float).tiny))
    return dict(verdict='INCONCLUSIVE',execution_status='NOT EXECUTED',offline_only=True,
        source='saved prior Gate A q, NOT a fresh live reading',q_saved=q.tolist(),g_model=g.tolist(),
        model_assumptions=['arm6 fixed base, default gravity [0,0,-9.81] in URDF base',
            'no additional tool/payload beyond arm6 URDF', 'no runtime first_moment_ratio, offset, alpha or friction compensation',
            '0x12 nominal joint-torque scaling assumed for feasibility only, NOT validated'],
        urdf=str(Path(urdf).resolve()),urdf_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
        kp=KP.tolist(),kd=KD.tolist(),velocity_target=[0.]*6,
        absolute_authorized_envelope=MAX_ENVELOPE.tolist(),proposed_reduced_envelope=env.tolist(),
        retained_limit_margin=JOINT_MARGIN,joint_limit_margin=margin.tolist(),
        restore_position_tolerance_proposed=RESTORE_TOL,
        minimum_lambda_for_nominal_static_restore_tolerance=minimum_lambda.tolist(),
        restore_vmax_proposed=RESTORE_VMAX,restore_amax_proposed=RESTORE_AMAX,
        unknowns=['actual 0x12 scale/sign','board total torque limit','persistent/exit board state',
                  'current workspace/desk and passive-catch geometry','unmodelled tool/payload/friction'],
        blockers=['site workspace/catch clearance not yet supplied',
            'low-lambda, fixed-low-Kp, unsupported slow restore lacks nominal static torque authority within authorized envelope'],
        stages=stages,serial_open_count=0,serial_write_count=0,hardware_executable_available=False)


def write_artifacts(result,out):
    out.mkdir(parents=True,exist_ok=True)
    (out/'gravity_bringup_summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    # No fabricated experimental rows: only a header because no hardware trial happened.
    with (out/'gravity_bringup_timeseries.csv').open('w',newline='') as f:
        csv.writer(f).writerow(['run','stage','lambda','t','joint','q_start','q','dq','g_hold','tau_gc','torque_proxy','feedback_age','tx_count','guard'])
    plots=out/'gravity_bringup_plots';plots.mkdir(exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for ax,j in zip(axes,(1,2)):
        ax.plot(LAMBDAS,[s['required_static_position_error_rad'][j] for s in result['stages']],'-o',label='nominal static error needed')
        ax.axhline(MAX_ENVELOPE[j],color='red',ls='--',label='absolute authorized displacement')
        ax.axhline(RESTORE_TOL,color='black',ls=':',label='proposed restore settle tolerance')
        ax.set_title(f'Joint {j+1}');ax.set_xlabel('lambda');ax.set_ylabel('rad');ax.grid(alpha=.3)
    axes[0].legend(fontsize=8);fig.suptitle('OFFLINE MODEL ONLY — not measured gravity bring-up')
    fig.tight_layout();fig.savefig(plots/'offline_restore_feasibility.png',dpi=150);plt.close(fig)
    (plots/'README.md').write_text('No live experiment was executed. Only an explicitly labeled offline feasibility plot is present. No measured q/dq/lambda traces exist.\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--urdf',type=Path,default=ROOT/'src/arm-platform/config/arm6.urdf')
    a=p.parse_args();saved=json.loads(a.snapshot.read_text())
    result=preview(saved['measured']['q'],a.urdf)
    result['snapshot_source']=str(a.snapshot.resolve())
    result['program_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    write_artifacts(result,a.output)
    print(json.dumps(result,indent=2,allow_nan=False))

if __name__=='__main__':main()
