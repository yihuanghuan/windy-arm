"""Dedicated bounded tuning runner. Offline plan/dry-run by default subcommands.

Fixed fresh q_start, frozen lambda*g(q_start), unchanged FEAA command order.
Hard abort closes without recovery writes. Close NEVER implies disabled motors.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import struct
import sys
import time
import numpy as np

import gravity_comm as comm
import gravity_tuning_policy as policy
import bounded_gravity_bringup as geometry

ROOT=Path(__file__).resolve().parents[3]
PERIOD=.01
RESTORE_TOL=np.array([.01]*3+[.015]*3)


class Abort(RuntimeError):pass


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def hashes():
    return {Path(p).name:sha(p) for p in (__file__,comm.__file__,policy.__file__,geometry.__file__,geometry.URDF)}


def packet(q,ff,kp,kd):
    arrays=[np.r_[policy.vector(q)*comm.SIGNS,0],np.zeros(7),
        np.r_[policy.vector(ff)*comm.SIGNS,0],np.r_[kp,0],np.r_[kd,0]]
    return b''.join(geometry.serial_frame(c,a) for c,a in zip(range(0x10,0x15),arrays))


def make_plan(q,age_limit,lam=.95,duration=.2,kp=None,kd=None):
    if duration not in (.2,.5,1.2,2.,3.):raise ValueError('duration tier')
    kp=policy.KP_MAX.copy() if kp is None else policy.vector(kp)
    kd=policy.KD_MAX.copy() if kd is None else policy.vector(kd)
    policy.validate_parameters(lam,kp,kd);q=policy.vector(q)
    gm=geometry.Geometry();g=gm.gravity(q)
    margins=gm.margins(q)
    if min(margins)<.05:raise Abort('initial joint margin')
    env=np.minimum(policy.MAX_ENVELOPE,.5*(margins-.05))
    for _ in range(30):
        cert=gm.certify(q,env)
        if cert['minimum_desk_clearance']>=.052 and cert['self_distance_lower']>=.004:break
        env*=.8
    else:raise Abort('no geometry box')
    # Expand one axis at a time, with geometry evidence, not a uniform relaxation.
    for j in (4,5,3,0,2,1):
        lo,hi=env[j],min(policy.MAX_ENVELOPE[j],.5*(margins[j]-.05))
        for _ in range(10):
            test=env.copy();test[j]=(lo+hi)/2;c=gm.certify(q,test)
            if c['minimum_desk_clearance']>=.052 and c['self_distance_lower']>=.004:lo=test[j]
            else:hi=test[j]
        env[j]=lo
    env*=.9  # Do not select a box right on the geometry screening boundary.
    cert=gm.certify(q,env)
    candidate=policy.feasible_candidates(g,env,kp,[lam])[0]
    if not candidate['feasible']:raise Abort('candidate static residual/proxy infeasible')
    return dict(hashes=hashes(),q_geometry_center=q.tolist(),envelope=env.tolist(),
        lower=gm.lower.tolist(),upper=gm.upper.tolist(),certificate=cert,
        mesh_hashes=gm.meshes,lambda_value=lam,kp=kp.tolist(),kd=kd.tolist(),
        g_preview=g.tolist(),static_error_preview=candidate['static_error'],duration_s=duration,
        critical_age_limit=age_limit,control_period_s=PERIOD,restore_tolerance=RESTORE_TOL.tolist(),
        restore_method='q_start is already the reference; retain it continuously while settling; no target excursion',
        maximum_settle_extension_s=.5,scene_assumptions='URDF/base/desk z=0 x<=0.02; user J3 approx10cm/no tool/clear workspace; conservative minimum',
        unknowns=['firmware version','0x12 physical scale/sign','board saturation','persistent state','disconnect behavior'])


def snapshot(plan,fb,t):
    q=fb.q.copy();v=fb.v.copy();g=geometry.GravityOnly().gravity(q)
    env=np.array(plan['envelope'])-abs(q-np.array(plan['q_geometry_center']))
    if np.any(env<4*policy.POSITION_QUANTUM):raise Abort('live pose outside geometry box')
    lam=plan['lambda_value'];kp=np.array(plan['kp']);kd=np.array(plan['kd'])
    if not policy.feasible_candidates(g,env,kp,[lam])[0]['feasible']:raise Abort('live static feasibility')
    mon=policy.Monitor(q,g,lam,kp,kd,env,plan['lower'],plan['upper'])
    pkt=packet(q,lam*g,kp,kd)
    wire_q=np.array(struct.unpack('<7f',pkt[4:32])[:6])*comm.SIGNS
    if not np.array_equal(wire_q,q):raise Abort('serialized first position target mismatch')
    age=max(t-fb.at[c] for c in comm.CRITICAL)
    row=mon.update(t,q,v,feedback_age=age,desk_margin=plan['certificate']['minimum_desk_clearance'],
        self_margin=plan['certificate']['self_distance_lower'],communication_level=fb.check(t)[0],critical_age_limit=plan['critical_age_limit'])
    if row['status']=='HARD_ABORT':raise Abort(row['reason'])
    snap=dict(kind='PRE_SEND_SNAPSHOT',t=t,measured_q=q.tolist(),measured_dq=v.tolist(),
        q_start=q.tolist(),q_command=q.tolist(),velocity_command=[0]*6,
        lambda_value=lam,kp=kp.tolist(),kd=kd.tolist(),g_hold=g.tolist(),current_command=(lam*g).tolist(),
        p=row['p'],d=row['d'],ff=row['ff'],proxy=row['proxy'],
        q_command_minus_measured=(wire_q-q).tolist(),dq_command_minus_measured=(-v).tolist(),
        critical_feedback_ages={hex(c):t-fb.at[c] for c in comm.CRITICAL},effective_envelope=env.tolist(),
        joint_limit_margin=np.minimum(q-plan['lower'],np.array(plan['upper'])-q).tolist(),
        packet_hex=pkt.hex(),theoretical_arrays={hex(c):list(struct.unpack('<7f',pkt[i*34+4:i*34+32])) for i,c in enumerate(range(0x10,0x15))},
        hashes=hashes(),unknowns=plan['unknowns'])
    return mon,pkt,snap


def qualified(fb,t):
    level,_=fb.check(t)
    if level=='COMM_HARD_ABORT':raise Abort(fb.hard_reason)
    if level=='COMM_SOFT_FAIL':return False
    if not all(fb.counts[hex(c)]>=180 for c in (0x20,0x21,0x22,0x24)):return False
    if not fb.positions or t-fb.positions[0][0]<1.:return False
    if any(x==0 for x in fb.latest[0x24][:6]):raise Abort('axis temperature/online invalid')
    if np.any(np.ptp([q for _,q in fb.positions],axis=0)>4*policy.POSITION_QUANTUM):return False
    return True


def settled(mon):
    if len(mon.rows)<20:return False
    rr=[r for r in mon.rows if mon.rows[-1]['t']-r['t']<=.15]
    if len(rr)<20 or rr[-1]['t']-rr[0]['t']<.12:return False
    q=np.array([r['q'] for r in rr]);span=rr[-1]['t']-rr[0]['t']
    return bool(np.all(abs(q[-1]-mon.q0)<=RESTORE_TOL) and
        np.all(np.ptp(q,axis=0)<=4*policy.POSITION_QUANTUM) and
        np.all(abs(q[-1]-q[0])/span<.015))


def live(plan,review,log_path):
    if plan['hashes']!=hashes() or review['hashes']!=hashes():raise Abort('hash binding')
    if review['plan_sha256']!=hashlib.sha256(json.dumps(plan,sort_keys=True).encode()).hexdigest():raise Abort('plan binding')
    for key in ('current_bounded_tuning_authorization','operator_present_isolation_available','workspace_clear_no_support_no_load','communication_recheck_passed'):
        if review.get(key) is not True:raise Abort('missing condition '+key)
    if not sys.stdin.isatty():raise Abort('operator terminal required')
    print('Fixed current q_start. Bounded gravity trial; close is NOT disable. Hard abort means NO RETRY.',flush=True)
    if input('Type BOUNDED TUNING READY: ')!='BOUNDED TUNING READY':raise Abort('not armed')
    import termios,fcntl,stat
    fd=None;fb=comm.Quality(plan['critical_age_limit']);first=None;last_tx=None;accepted=0;nb=0;write_attempts=0
    mon=None;pkt=None;last_gen=-1;last_send=None;state='QUALIFY';reason=None;states=[];q_end=None
    duration=plan['duration_s'];total_limit=min(3.,duration+plan['maximum_settle_extension_s'])
    log=Path(log_path).open('x');origin=time.monotonic();closed=None;restore='NOT_ATTEMPTED'
    def emit(row):log.write(json.dumps(row,allow_nan=False)+'\n');log.flush()
    def interrupt(signum,frame):raise Abort('operator signal '+str(signum))
    old={s:signal.signal(s,interrupt) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        fd=os.open('/dev/ttyUSB0',os.O_RDWR|os.O_NONBLOCK|os.O_NOCTTY|os.O_CLOEXEC)
        if not stat.S_ISCHR(os.fstat(fd).st_mode):raise Abort('not character device')
        fcntl.ioctl(fd,termios.TIOCEXCL)
        a=termios.tcgetattr(fd);a[0]=a[1]=a[3]=0;a[2]=termios.CS8|termios.CREAD|termios.CLOCAL
        a[4]=a[5]=termios.B921600;a[6][termios.VMIN]=a[6][termios.VTIME]=0
        termios.tcsetattr(fd,termios.TCSANOW,a);origin=time.monotonic()
        while True:
            if select.select([fd],[],[],.002)[0]:
                try:b=os.read(fd,4096)
                except BlockingIOError:b=None
                if b==b'':raise Abort('serial EOF')
                if b:
                    t=time.monotonic()-origin;emit(dict(kind='RX',t=t,hex=b.hex()));before=len(fb.error_events);fb.feed(b,t)
                    for e in fb.error_events[before:]:emit(dict(kind='COMM_ERROR',**e))
            t=time.monotonic()-origin;level,why=fb.check(t)
            if level=='COMM_HARD_ABORT':raise Abort(why)
            if mon is None:
                if t>2.5:
                    state='COMM_SOFT_FAIL_BEFORE_TX' if level=='COMM_SOFT_FAIL' else 'PREFLIGHT_NOT_QUALIFIED';reason='qualification timeout';break
                if not qualified(fb,t):continue
                mon,pkt,snap=snapshot(plan,fb,t)
                geometry.persist(str(log_path)+'.pre_send.json',snap)
                emit(dict(kind='PRE_SEND_SNAPSHOT',snapshot=snap));os.fsync(log.fileno())
                print('PRE_SEND_SNAPSHOT persisted; q='+str(snap['q_start'])+'; lambda='+str(plan['lambda_value'])+'; ff='+str(snap['current_command']),flush=True)
                last_gen=fb.generation
            if fb.generation!=last_gen:
                row=mon.update(t,fb.q,fb.v,feedback_age=max(t-fb.at[c] for c in comm.CRITICAL),
                    desk_margin=plan['certificate']['minimum_desk_clearance'],self_margin=plan['certificate']['self_distance_lower'],
                    communication_level=level,critical_age_limit=plan['critical_age_limit'])
                row.update(kind='STATE',lambda_value=plan['lambda_value'],kp=plan['kp'],kd=plan['kd'],
                    q_start=mon.q0.tolist(),g_hold=mon.g.tolist(),feedback_ages={hex(c):t-fb.at[c] for c in comm.CRITICAL},
                    communication_level=level,desk_lower_bound=plan['certificate']['minimum_desk_clearance'])
                emit(row);states.append(row);last_gen=fb.generation;q_end=fb.q.tolist()
                if row['status']=='HARD_ABORT':raise Abort(row['reason'])
                # A communication warning plus an expanding model-direction mismatch
                # warrants a hard stop, independently of either isolated symptom.
                if fb.error_events and t-fb.error_events[-1]['t']<=.1 and any(row['direction_mismatch']):
                    raise Abort('communication corruption coincides with abnormal direction dynamics')
            if first is not None and t-first>=duration:
                if settled(mon):
                    restore='SUCCESS_ALREADY_NEAR_Q_START_WITH_FIXED_REFERENCE'
                    state='SOFT_FAIL' if level=='COMM_SOFT_FAIL' or any(any(r.get('direction_mismatch',[])) for r in states) else 'NORMAL'
                    break
                if level=='COMM_SOFT_FAIL' or t-first>=total_limit:
                    state='SOFT_FAIL';reason='safe window ended but return/stability not established';restore='NOT_ESTABLISHED';break
                state='SETTLE_FIXED_Q_START'
            if last_send is not None and t-last_send<PERIOD:continue
            now=time.monotonic()-origin
            if now-t>PERIOD:raise Abort('prewrite host delay')
            if fb.check(now)[0]=='COMM_HARD_ABORT':raise Abort(fb.hard_reason)
            # Every-write checks, even without a new position sample.
            if np.any(abs(fb.q-mon.q0)>=.8*mon.env):raise Abort('prewrite envelope')
            if min(np.minimum(fb.q-plan['lower'],np.array(plan['upper'])-fb.q))<.05:raise Abort('prewrite joint margin')
            proxy=mon.kp*(mon.q0-fb.q)-mon.kd*fb.v+mon.ff
            if not np.isfinite(proxy).all() or np.any(abs(proxy)>policy.PROXY_LIMIT):raise Abort('prewrite nominal proxy')
            if first is not None and now-first>=total_limit:state='SOFT_FAIL';reason='absolute time deadline';break
            if struct.unpack('I',fcntl.ioctl(fd,termios.TIOCOUTQ,struct.pack('I',0)))[0]:raise Abort('TX queue pending')
            if time.monotonic()-origin-now>PERIOD:raise Abort('prewrite queue delay')
            write_attempts+=1
            n=os.write(fd,pkt)  # ONLY device write; no retry, exit or recovery frames.
            accepted+=n;nb+=1;last_tx=dict(t=now,accepted_bytes=n,hex=pkt[:n].hex());emit(dict(kind='TX_KERNEL_ACCEPTED',**last_tx))
            if first is None and n>0:first=now
            if n!=len(pkt):raise Abort('partial serial write')
            state='OBSERVE'
            last_send=now
    except BaseException as exc:
        state='HARD_ABORT' if write_attempts else 'HARD_ABORT_BEFORE_FIRST_WRITE';reason=str(exc);restore='PROHIBITED_AFTER_HARD_ABORT'
    finally:
        last_receive=fb.last_chunk
        # Capture quality before close; teardown duration is not a feedback age.
        quality_summary=fb.summary(last_receive if last_receive is not None else 0.)
        if fd is not None:os.close(fd);closed=time.monotonic()-origin
        for s,h in old.items():signal.signal(s,h)
        result=dict(kind='EXIT',state=state,reason=reason,first_tx_time=first,last_tx=last_tx,
            accepted_bytes=accepted,tx_batches=nb,write_attempts=write_attempts,last_receive_t=last_receive,serial_closed_t=closed,
            q_end=q_end,restore_result=restore,communication=quality_summary,hashes=hashes(),
            exit_meaning='No further TX; board state UNKNOWN. Close is NOT motor disable. Hard abort: physical isolation, no retry.')
        try:emit(result);os.fsync(log.fileno())
        finally:log.close()
        print(json.dumps(result,indent=2),flush=True)
        print('STOPPED AND CLOSED. Motor disable is NOT established. Use physical power isolation as required.',flush=True)
    return result


def dry_run(plan,capture,out):
    fb=comm.Quality(plan['critical_age_limit'])
    for t,b in comm.rx_rows(capture):
        fb.feed(b,t)
        if qualified(fb,t):
            _,_,snap=snapshot(plan,fb,t);geometry.persist(out,snap)
            return dict(status='READY TO WRITE — DRY RUN ONLY',snapshot=snap,serial_open_count=0,serial_write_count=0)
    raise Abort('offline capture did not qualify')


def main():
    ap=argparse.ArgumentParser();sub=ap.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('plan');p.add_argument('--state',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--lambda-value',type=float,default=.95);p.add_argument('--duration',type=float,default=.2)
    for name in ('dry-run','execute'):
        p=sub.add_parser(name);p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
        p.add_argument('--capture' if name=='dry-run' else '--review',type=Path,required=True)
    a=ap.parse_args()
    if a.mode=='plan':
        b=json.loads(a.baseline.read_text());s=json.loads(a.state.read_text())
        if s['status'] not in ('COMM_OK','COMM_WARNING') or s['duration_s']<5:raise Abort('preflight not passed')
        result=make_plan(s['q'],b['proposed_critical_age_limit'],a.lambda_value,a.duration);geometry.persist(a.output,result)
    else:
        plan=json.loads(a.plan.read_text())
        if plan['hashes']!=hashes():raise Abort('plan dependencies changed')
        result=dry_run(plan,a.capture,a.output) if a.mode=='dry-run' else live(plan,json.loads(a.review.read_text()),a.output)
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':main()
