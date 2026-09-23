"""Dedicated bounded gravity bring-up. Default commands are offline.

No ROS, Phase4 runner, SMC, offsets, calibration ratios, alpha or sign flips.
Live requires an explicit current authorization manifest tied to program/plan hashes.
Closing UART NEVER means motor disable. Hard abort never restores or retries.
"""
import argparse
from collections import deque
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import select
import signal
import struct
import sys
import time

import numpy as np
import pinocchio as pin
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/real_arm_readonly'))
from real_arm_readonly.protocol import Decoder, SIGNS

URDF=ROOT/'src/arm-platform/config/arm6.urdf'
KP=np.array([30.,30.,30.,5.,5.,5.])
KD=np.array([1.,1.,1.,.1,.1,.1])
RATED=np.array([9.,9.,9.,1.6,1.6,1.6])
PROXY_LIMIT=.5*RATED
NEEDED=(0x20,0x21,0x22,0x24)
PERIOD=.01
STALE=.05
POS_AGE=.01
P_QUANT=.0003833770751953125
RESTORE_TOL=4*P_QUANT
MIN_MARGIN=.05
DESK_Z=0.
DESK_EDGE=.02
DESK_MIN=.05
SPEED_WARNING=.293060302734375
SHORT=.25
SETTLE_MAX=.75
SETTLE_WINDOW=.15

class Abort(RuntimeError):pass
class Inconclusive(RuntimeError):pass

def finite(v):
    a=np.asarray(v,dtype=float)
    if a.shape!=(6,) or not np.isfinite(a).all():raise Abort('six finite values required')
    return a

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def serial_frame(cmd,values):
    b=b'\xfe\xaa'+bytes((cmd,28))+struct.pack('<7f',*values)
    return b+bytes((sum(b)%256,sum(sum(b[:i+1]) for i in range(len(b)))%256))

def packet(q,ff):
    q=finite(q);ff=finite(ff)
    if np.any(abs(ff)>PROXY_LIMIT):raise Abort('feedforward limit')
    arrays=[np.r_[q*np.array(SIGNS),0],np.zeros(7),np.r_[ff*np.array(SIGNS),0],np.r_[KP,0],np.r_[KD,0]]
    return b''.join(serial_frame(c,v) for c,v in zip(range(0x10,0x15),arrays))

def persist(path,item):
    txt=json.dumps(item,indent=2,allow_nan=False)+'\n'
    with Path(path).open('x') as f:
        if f.write(txt)!=len(txt):raise Abort('short audit write')
        f.flush();os.fsync(f.fileno())
    if Path(path).read_text()!=txt:raise Abort('audit readback mismatch')
    return hashlib.sha256(txt.encode()).hexdigest()

class Geometry:
    """Exact triangle/desk clipping, continuous joint-box bounds, nonadjacent meshes."""
    def __init__(self):
        self.model=pin.buildModelFromUrdf(str(URDF));self.data=self.model.createData()
        if self.model.nq!=6 or list(self.model.names)[1:]!=[f'joint{i}' for i in range(1,7)]:raise Abort('URDF joints')
        # Conservative intersection of URDF and hardware configuration limits.
        self.lower=np.maximum(self.model.lowerPositionLimit,[-4,-3.1415,-1.5708,-4,-2,-4])
        self.upper=np.minimum(self.model.upperPositionLimit,[4,2,1.5708,4,2,4])
        self.meshes=[];self.triangles=[];self.radius=np.zeros((7,6))
        for i in range(1,7):
            p=ROOT/f'src/dummy_description/meshes/arm/A-L1-GAMMA/Link{i}.STL';b=p.read_bytes()
            n=struct.unpack_from('<I',b,80)[0]
            if len(b)!=84+50*n:raise Abort('unexpected STL format')
            dt=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
            triangles=np.frombuffer(b,dt,count=n,offset=84)['v'].astype(float)
            self.triangles.append(triangles);self.meshes.append(dict(path=str(p),sha256=sha(p)))
            mesh_radius=float(np.max(np.linalg.norm(triangles.reshape(-1,3),axis=1)))
            for j in range(1,i+1):
                self.radius[i,j-1]=mesh_radius+sum(np.linalg.norm(self.model.jointPlacements[k].translation) for k in range(j+1,i+1))
        self.gm=pin.buildGeomFromUrdf(self.model,str(URDF),pin.GeometryType.COLLISION,package_dirs=[str(ROOT/'src')])
        for i in range(7):
            for j in range(i+2,7):self.gm.addCollisionPair(pin.CollisionPair(i,j))
        self.gd=self.gm.createData()
    def gravity(self,q):return np.asarray(pin.computeGeneralizedGravity(self.model,self.data,finite(q))).copy()
    def margins(self,q):return np.minimum(q-self.lower,self.upper-q)
    @staticmethod
    def clipped_min_z(triangles,xmax):
        flat=triangles.reshape(-1,3);inside=flat[flat[:,0]<=xmax]
        z=float(np.min(inside[:,2])) if len(inside) else math.inf
        for i,j in ((0,1),(1,2),(2,0)):
            a=triangles[:,i];b=triangles[:,j]
            cross=(a[:,0]-xmax)*(b[:,0]-xmax)<0
            if np.any(cross):
                zc=a[cross,2]+(xmax-a[cross,0])/(b[cross,0]-a[cross,0])*(b[cross,2]-a[cross,2])
                z=min(z,float(np.min(zc)))
        return z
    def certify(self,q,env):
        pin.framesForwardKinematics(self.model,self.data,q)
        bounds=self.radius@env;desk=[];minimum=math.inf
        for i,t in enumerate(self.triangles,1):
            tf=self.data.oMf[self.model.getFrameId(f'link{i}')]
            world=t@tf.rotation.T+tf.translation
            # A point over the desk anywhere in the joint box must start inside x<=edge+B.
            z=self.clipped_min_z(world,DESK_EDGE+bounds[i])-DESK_Z-bounds[i]
            desk.append(None if math.isinf(z) else z);minimum=min(minimum,z)
        # Site says J3 is lowest at ~10cm. Reserve 1cm for its approximate measurement.
        site_bound=.10-.01-bounds[3]
        minimum=min(minimum,site_bound)
        pin.updateGeometryPlacements(self.model,self.data,self.gm,self.gd,q);pin.computeDistances(self.gm,self.gd)
        distances=[];self_min=math.inf
        for cp,res in zip(self.gm.collisionPairs,self.gd.distanceResults):
            i,j=cp.first,cp.second
            # Upstream rotations common to both links preserve pairwise distance.
            relative_bound=float(sum(self.radius[j,k]*env[k] for k in range(i,j)))
            lower=float(res.min_distance)-relative_bound;self_min=min(self_min,lower)
            distances.append(dict(links=[i,j],initial=float(res.min_distance),lower_bound=lower))
        return dict(minimum_desk_clearance=minimum,per_link_desk_lower=desk,site_j3_lower=site_bound,
                    self_distance_lower=self_min,pair_distances=distances,point_motion_bounds=bounds.tolist())
    def plan(self,q):
        q=finite(q);m=self.margins(q)
        if min(m)<=MIN_MARGIN:raise Inconclusive('current pose within conservative limit margin')
        env=np.minimum([.02,.02,.02,.03,.03,.03],.5*(m-MIN_MARGIN))
        attempts=[]
        for _ in range(12):
            c=self.certify(q,env);attempts.append(dict(envelope=env.tolist(),certificate=c))
            if c['minimum_desk_clearance']>=DESK_MIN+.001 and c['self_distance_lower']>=.003:break
            env*=.8
        else:raise Inconclusive('no geometric joint-box certificate')
        if np.any(env<4*P_QUANT):raise Inconclusive('certified box too narrow relative to observed position quantization')
        g=self.gravity(q)
        if np.any(abs(g)>PROXY_LIMIT):raise Inconclusive('gravity exceeds conservative nominal proxy limit')
        lam=choose_lambda(g,env)
        return dict(program_sha256=sha(__file__),urdf_sha256=sha(URDF),meshes=self.meshes,
            q_geometry_center=q.tolist(),envelope=env.tolist(),lower=self.lower.tolist(),upper=self.upper.tolist(),
            kp=KP.tolist(),kd=KD.tolist(),minimum_joint_margin=MIN_MARGIN,
            g_preview=g.tolist(),lambda_preview=lam,short_seconds=SHORT,restore_settle_max=SETTLE_MAX,
            certificate=c,geometry_search=attempts,geometry_method='triangle plane clipping + continuous conservative joint box; nonadjacent exact mesh distances minus relative motion bounds',
            scene_assumptions='author horizontal desk z=0 x<=0.02 in URDF base; full link meshes; J3 ~10cm site clearance with 1cm estimate reserve; use smaller bound; scene clear/no tool per user',
            unknowns=['current board 0x12 scale/sign','board total saturation','exit persistent state'],
            active_motion_target='fresh q_start; never a changed position setpoint',
            restore_method='keep the already-commanded q_start continuous; wait for verified near-start stable feedback, no unnecessary reference excursion')

def choose_lambda(g,env,minimum=0.):
    tol=np.minimum(.25*finite(env),.5*RESTORE_TOL)
    need=np.max(np.maximum(0,1-KP*tol/np.maximum(abs(finite(g)),1e-12)))
    lam=min(1.,math.ceil(max(need,minimum)*200-1e-12)/200)
    if not 0<=lam<=1:raise Abort('lambda bounds')
    return lam

class Feedback:
    def __init__(self):
        self.decoder=Decoder();self.q=None;self.v=None;self.ages={};self.counts={};self.positions=deque();self.generation=0;self.last=None
    def ingest(self,b,t):
        if self.last is not None and (t<self.last or t-self.last>STALE):raise Abort('feedback receive gap/clock')
        self.last=t;self.decoder.feed(b)
        if self.decoder.bad_checksum or self.decoder.bad_length or self.decoder.nonfinite:raise Abort('checksum/malformed/nonfinite')
        for c,v in self.decoder.raw_frames:
            if not all(math.isfinite(x) for x in v):raise Abort('nonfinite raw slot')
            if c not in NEEDED:continue
            self.ages[c]=t;self.counts[c]=self.counts.get(c,0)+1
            if c==0x24 and any(x==0 for x in v[:6]):raise Abort('six-axis telemetry validity')
            if c==0x20:
                self.q=np.array(v[:6])*SIGNS;self.positions.append((t,self.q.copy()));self.generation+=1
                while self.positions and t-self.positions[0][0]>1.05:self.positions.popleft()
            elif c==0x21:self.v=np.array(v[:6])*SIGNS
    def fresh(self,t):
        if not all(c in self.ages for c in NEEDED):raise Abort('incomplete feedback')
        if any(not 0<=t-self.ages[c]<=STALE for c in NEEDED):raise Abort('feedback stale')
        return {hex(c):t-self.ages[c] for c in NEEDED}
    def qualified(self,t):
        if not all(c in self.ages for c in NEEDED):return False
        self.fresh(t)
        if min(self.counts.values())<180 or not self.positions or t-self.positions[0][0]<1.:return False
        qs=np.array([q for _,q in self.positions])
        if np.any(np.ptp(qs,axis=0)>4*P_QUANT):raise Abort('not stationary during qualification')
        return True

class Monitor:
    def __init__(self,plan,q,g,lam):
        self.plan=plan;self.q0=finite(q).copy();self.g=finite(g).copy();self.lam=float(lam)
        if not 0<=lam<=1:raise Abort('lambda range')
        self.ff=lam*self.g
        self.env=np.array(plan['envelope'])-abs(self.q0-np.array(plan['q_geometry_center']))
        if np.any(self.env<4*P_QUANT):raise Abort('live pose outside useful certified box')
        if np.any(abs((1-lam)*g)/KP>np.minimum(self.env*.25,RESTORE_TOL*.5)+1e-12):raise Abort('static feasibility')
        self.rows=deque();self.last_generation=-1;self.warning_count=0
    def check(self,fb,t):
        age=fb.fresh(t);q=finite(fb.q);v=finite(fb.v);e=self.q0-q
        if np.any(abs(e)>self.env):raise Abort('displacement envelope')
        margins=np.minimum(q-self.plan['lower'],np.array(self.plan['upper'])-q)
        if min(margins)<MIN_MARGIN:raise Abort('joint limit margin')
        p=KP*e;d=-KD*v;proxy=p+d+self.ff
        warning=bool(np.any(abs(v)>SPEED_WARNING))
        if warning:self.warning_count+=1
        # Raw velocity spikes are warnings; a raw proxy over-limit stops before TX.
        # This is a nominal torque protection, not a claim of board saturation.
        if np.any(abs(proxy)>PROXY_LIMIT):raise Abort('nominal total torque proxy limit')
        if fb.generation!=self.last_generation:
            self.last_generation=fb.generation;self.rows.append((t,q.copy(),proxy.copy()))
            while self.rows and t-self.rows[0][0]>1.:self.rows.popleft()
            recent=[r for r in self.rows if t-r[0]<=.05]
            if len(recent)>=5 and t-recent[0][0]>=.035:
                delta=q-recent[0][1];speed=delta/(t-recent[0][0])
                vmax=np.minimum([.15]*3+[.20]*3,self.env/.15)
                if np.any((abs(speed)>vmax)&(abs(delta)>2*P_QUANT)):raise Abort('sustained position-consistent velocity')
            # Compare consecutive 50ms block means; no artificial old torque=0.
            blocks=[]
            for lo,hi in ((.10,.15),(.05,.10),(0,.05)):
                rr=[r for r in self.rows if lo<=t-r[0]<hi]
                if len(rr)<5 or rr[-1][0]-rr[0][0]<.025:break
                blocks.append((np.mean([r[0] for r in rr]),np.mean([r[1] for r in rr],axis=0),np.mean([r[2] for r in rr],axis=0)))
            if len(blocks)==3:
                a,b,c=blocks;v1=(b[1]-a[1])/(b[0]-a[0]);v2=(c[1]-b[1])/(c[0]-b[0])
                if np.any((v1*v2>0)&(abs(v2)>abs(v1)+.05)&(abs(v2)>.05)):
                    raise Abort('sustained increasing position-derived speed')
                if np.any(abs(c[2]-b[2])/(c[0]-b[0])>PROXY_LIMIT/.05):raise Abort('50ms mean torque proxy slew')
                if np.any((v1*v2<0)&(abs(v2)>abs(v1)+.05)&(abs(c[1]-b[1])>4*P_QUANT)):
                    raise Abort('growing oscillation')
        return dict(t=t,q=q.tolist(),dq=v.tolist(),q_start=self.q0.tolist(),delta_q=(-e).tolist(),
            position_command=self.q0.tolist(),velocity_command=[0.]*6,current_command=self.ff.tolist(),
            kp=KP.tolist(),kd=KD.tolist(),lambda_value=self.lam,g_hold=self.g.tolist(),
            p_term=p.tolist(),d_term=d.tolist(),ff_term=self.ff.tolist(),torque_proxy=proxy.tolist(),
            feedback_age=age,joint_limit_margin=margins.tolist(),displacement_utilization=(abs(e)/self.env).tolist(),
            proxy_utilization=(abs(proxy)/PROXY_LIMIT).tolist(),desk_clearance_certified=self.plan['certificate']['minimum_desk_clearance'],velocity_spike_warning=warning)
    def stable(self,t):
        r=[r for r in self.rows if t-r[0]<=SETTLE_WINDOW]
        if len(r)<20 or r[-1][0]-r[0][0]<.12:return False
        q=np.array([x[1] for x in r])
        return bool(np.all(abs(q[-1]-self.q0)<=RESTORE_TOL) and np.all(np.ptp(q,axis=0)<=4*P_QUANT))


def snapshot(plan,fb,model,t,minimum_lambda):
    fb.fresh(t)
    if t-fb.ages[0x20]>POS_AGE:raise Abort('position stale before snapshot')
    q=finite(fb.q).copy();g=model.gravity(q)
    env=np.array(plan['envelope'])-abs(q-np.array(plan['q_geometry_center']))
    lam=choose_lambda(g,env,minimum_lambda);mon=Monitor(plan,q,g,lam);row=mon.check(fb,t)
    pkt=packet(q,mon.ff)
    wire_q=np.array(struct.unpack('<7f',pkt[4:32])[:6])*SIGNS
    if not np.array_equal(wire_q,q):raise Abort('first target mismatch')
    row.update(kind='PRE_SEND_SNAPSHOT',program_sha256=sha(__file__),urdf_sha256=sha(URDF),
        q_command_minus_measured=(wire_q-q).tolist(),packet_hex=pkt.hex(),
        theoretical_arrays={hex(c):list(struct.unpack('<7f',pkt[i*34+4:i*34+32])) for i,c in enumerate(range(0x10,0x15))},
        effective_envelope=mon.env.tolist(),short_seconds=SHORT,maximum_total_run_seconds=SHORT+SETTLE_MAX,
        unknowns=plan['unknowns'])
    return mon,pkt,row

class GravityOnly:
    def __init__(self):self.m=pin.buildModelFromUrdf(str(URDF));self.d=self.m.createData()
    def gravity(self,q):return np.asarray(pin.computeGeneralizedGravity(self.m,self.d,finite(q))).copy()


def dry_run(plan,capture,out,minimum_lambda=0.):
    fb=Feedback();model=GravityOnly()
    for line in Path(capture).read_text().splitlines():
        r=json.loads(line);t=r['t_sec'];fb.ingest(bytes.fromhex(r['bytes']),t)
        if fb.qualified(t):
            mon,pkt,snap=snapshot(plan,fb,model,t,minimum_lambda)
            persist(out,snap)
            return dict(status='READY TO WRITE — OFFLINE ONLY',snapshot=snap,serial_open_count=0,serial_write_count=0)
    raise Abort('capture did not qualify')


def live(plan,review,log_path,minimum_lambda=0.):
    if review.get('program_sha256')!=sha(__file__) or review.get('plan_sha256')!=hashlib.sha256(json.dumps(plan,sort_keys=True).encode()).hexdigest():raise Abort('review binding mismatch')
    for k in ('current_user_authorization','operator_present_power_isolation_available','no_support_no_extra_load','workspace_clear','accept_unknown_firmware_risk'):
        if review.get(k) is not True:raise Abort('missing live condition '+k)
    if not sys.stdin.isatty():raise Abort('operator terminal required')
    print('Six-axis bounded gravity feedforward, fixed measured q_start, at most 1s including settle.\nClose is NOT disable; physical isolation is the final stop boundary.',flush=True)
    if input('Type BOUNDED GRAVITY READY: ')!='BOUNDED GRAVITY READY':raise Abort('not armed')
    import fcntl,stat,termios
    fb=Feedback();model=GravityOnly();fd=None;mon=None;pkt=None;first=None;last_send=None;last_gen=-1;accepted=0;tx=0;state='QUALIFY';reason=None;last=None
    log=Path(log_path).open('x')
    def emit(x):log.write(json.dumps(x,allow_nan=False)+'\n');log.flush()
    def interrupted(signum,frame):raise Abort('operator signal '+str(signum))
    old={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}
    origin=time.monotonic()
    try:
        fd=os.open('/dev/ttyUSB0',os.O_RDWR|os.O_NONBLOCK|os.O_NOCTTY|os.O_CLOEXEC)
        if not stat.S_ISCHR(os.fstat(fd).st_mode):raise Abort('not character device')
        fcntl.ioctl(fd,termios.TIOCEXCL)
        a=termios.tcgetattr(fd);a[0]=a[1]=a[3]=0;a[2]=termios.CS8|termios.CREAD|termios.CLOCAL;a[4]=a[5]=termios.B921600;a[6][termios.VMIN]=a[6][termios.VTIME]=0
        termios.tcsetattr(fd,termios.TCSANOW,a);origin=time.monotonic()
        while True:
            if select.select([fd],[],[],.002)[0]:
                try:b=os.read(fd,4096)
                except BlockingIOError:b=None
                if b==b'':raise Abort('serial disconnect')
                if b:
                    t=time.monotonic()-origin;emit(dict(kind='RX',t=t,hex=b.hex()));fb.ingest(b,t)
            t=time.monotonic()-origin
            if mon is None:
                if t>2.5:raise Abort('feedback qualification timeout')
                if not fb.qualified(t):continue
                mon,pkt,snap=snapshot(plan,fb,model,t,minimum_lambda)
                audit_t0=time.monotonic()
                persist(str(log_path)+'.pre_send.json',snap)
                emit(dict(kind='PRE_SEND_SNAPSHOT',snapshot=snap));os.fsync(log.fileno())
                audit_t1=time.monotonic()
                # Full audit remains durable; avoid a multi-kilobyte PTY burst on the TX path.
                print('PRE_SEND_SNAPSHOT saved: '+str(log_path)+'.pre_send.json; lambda='+str(mon.lam)+
                      '; q='+str(mon.q0.tolist())+'; ff='+str(mon.ff.tolist()),flush=True)
                emit(dict(kind='AUDIT_LOG_TIMING',persist_seconds=audit_t1-audit_t0,
                          terminal_seconds=time.monotonic()-audit_t1))
            row=mon.check(fb,t);row['kind']='STATE';row['stage']=state;emit(row)
            if first is not None and t-first>=SHORT:
                state='RESTORE_SETTLE'  # q_start already commanded; keep continuous reference.
                if mon.stable(t):state='NORMAL_COMPLETION';break
                if t-first>=SHORT+SETTLE_MAX:state='NORMAL_INCONCLUSIVE';reason='restore/settle not established; no automatic next run';break
            if last_send is not None and t-last_send<PERIOD:continue
            if fb.generation==last_gen:continue
            # Check the CURRENT clock after compute/logging/snapshot persistence.
            now=time.monotonic()-origin
            if now-t>PERIOD:raise Abort('prewrite host delay')
            mon.check(fb,now)
            if now-fb.ages[0x20]>POS_AGE:raise Abort('prewrite position stale')
            if first is not None and now-first>=SHORT+SETTLE_MAX:state='NORMAL_INCONCLUSIVE';reason='deadline';break
            if struct.unpack('I',fcntl.ioctl(fd,termios.TIOCOUTQ,struct.pack('I',0)))[0]:raise Abort('TX queue pending')
            if time.monotonic()-origin-now>PERIOD:raise Abort('prewrite queue delay')
            n=os.write(fd,pkt)  # SOLE device-write call. No retry, flush, exit or recovery frames.
            accepted+=n;tx+=1;last=dict(t=now,accepted_bytes=n,hex=pkt[:n].hex());emit(dict(kind='TX_KERNEL_ACCEPTED',**last))
            if n!=len(pkt):raise Abort('partial serial write; no retry')
            if first is None:first=now;state='SHORT'
            last_send=now;last_gen=fb.generation
    except BaseException as exc:
        state='HARD_ABORT' if first is not None else 'ABORT_BEFORE_FIRST_WRITE';reason=str(exc)
    finally:
        closed=None
        if fd is not None:os.close(fd);closed=time.monotonic()-origin
        for s,h in old.items():signal.signal(s,h)
        result=dict(kind='EXIT',state=state,reason=reason,accepted_bytes=accepted,tx_batches=tx,
            first_tx_time=first,serial_closed_t=closed,last_tx=last,
            exit_meaning='TX stopped only; board state unknown; operator physical power isolation required',
            restore_reference_changed=False,program_sha256=sha(__file__))
        try:emit(result);os.fsync(log.fileno())
        finally:log.close()
        print(json.dumps(result,indent=2),flush=True)
        print('STOPPED AND CLOSED. Close != motor disable. Use physical power isolation as required.',flush=True)
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('plan');p.add_argument('--q-json',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('dry-run');p.add_argument('--plan',type=Path,required=True);p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--minimum-lambda',type=float,default=0.)
    p=sub.add_parser('execute');p.add_argument('--plan',type=Path,required=True);p.add_argument('--review',type=Path,required=True);p.add_argument('--log',type=Path,required=True);p.add_argument('--minimum-lambda',type=float,default=0.)
    a=ap.parse_args()
    if a.mode=='plan':
        r=Geometry().plan(json.loads(a.q_json.read_text())['q']);persist(a.output,r);print(json.dumps(r,indent=2));return 0
    plan=json.loads(a.plan.read_text())
    if plan['program_sha256']!=sha(__file__) or plan['urdf_sha256']!=sha(URDF):raise Abort('plan code/model hash mismatch')
    if a.mode=='dry-run':r=dry_run(plan,a.capture,a.output,a.minimum_lambda)
    else:r=live(plan,json.loads(a.review.read_text()),a.log,a.minimum_lambda)
    print(json.dumps(r,indent=2));return 0 if a.mode=='dry-run' or r['state']=='NORMAL_COMPLETION' else 1

if __name__=='__main__':raise SystemExit(main())
