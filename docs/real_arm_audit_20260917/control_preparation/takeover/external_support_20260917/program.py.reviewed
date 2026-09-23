"""Gate A only. Default is OFFLINE replay. Hardware mode requires reviewed interlocks.
No ROS, gravity, trajectory, reset, homing, or changing targets. DO NOT EXECUTE live
without separate explicit human authorization. Closing UART is NOT motor disable.
"""
import argparse
import hashlib
from collections import deque
import json
import math
import os
from pathlib import Path
import select
import signal
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/real_arm_readonly'))
from real_arm_readonly.protocol import Decoder, SIGNS

KP = (6., 6., 6., 1., 1., 1.)  # historical Phase4 L2, NOT a firmware certification
KD = (.3, .3, .3, .03, .03, .03)
P_QUANT = .0003833770751953125  # observed encoder step
V_QUANT = .02442169189453125   # observed RS05 velocity step
E_BOUND = 16 * P_QUANT
V_BOUND = 12 * V_QUANT
PROXY_BOUND = tuple(p*E_BOUND+d*V_BOUND for p,d in zip(KP,KD))
STALE = .05  # same as repository direct effort command timeout
WINDOW = .05
SLEW_BOUND = tuple(b/WINDOW for b in PROXY_BOUND)
LOWER = (-4., -3.1415, -1.5708, -4., -2., -4.)
UPPER = (4., 2., 1.5708, 4., 2., 4.)
MARGIN = .15  # historical FK calibration guard, exceeds DMMotor's .05
PERIOD = .01  # historical 100 Hz
DURATION = 2.
QUALIFY = 1.
NEEDED = (0x20, 0x21, 0x22, 0x24)


class Abort(RuntimeError):
    pass


def frame(cmd, values):
    payload = struct.pack('<7f', *values)
    body = b'\xfe\xaa'+bytes((cmd,28))+payload
    total = acc = 0
    for b in body:
        total=(total+b)&255; acc=(acc+total)&255
    return body+bytes((total,acc))


def command_bundle(q_hold):
    if len(q_hold)!=6 or not all(math.isfinite(x) for x in q_hold):
        raise Abort('invalid held target')
    # No unwrap here: target captured from the exact signed wire coordinates.
    # This preserves encoder winding and avoids changing target by 2*pi.
    position=[s*q for s,q in zip(SIGNS,q_hold)]+[0.]
    arrays=((0x10,position),(0x11,[0.]*7),(0x12,[0.]*7),
            (0x13,list(KP)+[0.]),(0x14,list(KD)+[0.]))
    return b''.join(frame(c,v) for c,v in arrays)


class GateA:
    """Pure state machine: returns candidate bytes, never performs I/O."""
    def __init__(self):
        self.decoder=Decoder()
        self.latest={}; self.age={}; self.q=None; self.v=None
        self.positions=deque(); self.proxies=deque(); self.previous_mean=None
        self.q_hold=None; self.packet=None; self.started=None; self.first=None
        self.last_now=None; self.last_tick=None; self.last_send=None
        self.counts={c:0 for c in NEEDED}; self.samples=0; self.last_sample_sent=-1
        self.state='QUALIFY'; self.reason=None; self.proxy=None
        self.qualified_frames=0; self.batches=0; self.startup_time=None

    def abort(self, why):
        self.state='ABORT';self.reason=why
        raise Abort(why)

    def ingest(self, chunk, now):
        if self.state in ('DONE','ABORT'):return
        if self.last_now is not None and now<self.last_now:self.abort('nonmonotonic clock')
        if all(c in self.age for c in NEEDED) and any(now-self.age[c]>STALE for c in NEEDED):
            self.abort('telemetry gap before next read')
        self.last_now=now
        self.decoder.feed(chunk)
        if self.decoder.bad_checksum or self.decoder.bad_length or self.decoder.nonfinite:
            self.abort('checksum/length/nonfinite frame')
        for cmd,raw in self.decoder.raw_frames:
            if not all(math.isfinite(x) for x in raw):self.abort('nonfinite raw slot')
            if cmd not in NEEDED:continue
            self.latest[cmd]=raw;self.age[cmd]=now;self.counts[cmd]+=1
            if cmd==0x24 and any(x==0 for x in raw[:6]):self.abort('zero temperature: six-axis validity unresolved')
            if cmd==0x21:
                self.v=[s*x for s,x in zip(SIGNS,raw[:6])]
                if max(map(abs,self.v))>V_BOUND:self.abort('raw velocity bound')
            if cmd==0x20:
                # Same sign convention; no model offset and no modulo target changes.
                self.q=[s*x for s,x in zip(SIGNS,raw[:6])]
                if any(min(q-lo,hi-q)<MARGIN for q,lo,hi in zip(self.q,LOWER,UPPER)):
                    self.abort('joint limit margin')
                if self.positions and max(abs(a-b) for a,b in zip(self.q,self.positions[-1][1]))>E_BOUND:
                    self.abort('position step bound')
                self.positions.append((now,self.q.copy()));self.samples+=1
                while self.positions and now-self.positions[0][0]>QUALIFY:self.positions.popleft()
                recent=[(t,q) for t,q in self.positions if now-t<=WINDOW]
                if recent and now-recent[0][0]>=.8*WINDOW:
                    if max(abs(a-b) for a,b in zip(self.q,recent[0][1]))>4*P_QUANT:
                        self.abort('continuous position displacement in 50ms window')
            if self.q_hold is not None and self.q is not None and self.v is not None:
                error=[h-q for h,q in zip(self.q_hold,self.q)]
                if max(map(abs,error))>E_BOUND:self.abort('held position error')
                self.proxy=[p*e-d*v for p,d,e,v in zip(KP,KD,error,self.v)]
                if any(abs(x)>b+1e-12 for x,b in zip(self.proxy,PROXY_BOUND)):
                    self.abort('instantaneous torque proxy bound')
                if cmd==0x20:
                    self.proxies.append((now,self.proxy.copy()))
                    if now-self.proxies[0][0]>=WINDOW:
                        mean=[sum(v[j] for _,v in self.proxies)/len(self.proxies) for j in range(6)]
                        if self.previous_mean:
                            t0,m0=self.previous_mean;dt=now-t0
                            if any(abs(a-b)/dt>bound for a,b,bound in zip(mean,m0,SLEW_BOUND)):
                                self.abort('50ms mean torque proxy slew bound')
                        self.previous_mean=(now,mean);self.proxies.clear()

    def tick(self, now):
        if self.state in ('ABORT','DONE'):return None
        if self.startup_time is None:self.startup_time=now
        if self.last_tick is not None and now<self.last_tick:self.abort('nonmonotonic tick')
        if self.started is not None and now-self.last_tick>STALE:self.abort('host scheduling stalled')
        self.last_tick=now
        if self.started is not None and now-self.started>=DURATION:
            self.state='DONE';return None
        if not all(c in self.age for c in NEEDED):
            if now-self.startup_time>2:self.abort('no full spontaneous feedback within 2s')
            return None
        if any(now-self.age[c]>STALE for c in NEEDED):self.abort('telemetry stale')
        if self.first is None:self.first=now
        if self.state=='QUALIFY':
            if self.positions:
                for j in range(6):
                    vals=[q[j] for _,q in self.positions]
                    if max(vals)-min(vals)>4*P_QUANT:self.abort('not stationary during qualification')
            if now-self.first<QUALIFY or min(self.counts.values())<180:return None
            # Every stream must be close to its proven 200Hz cadence, not a replay burst.
            span=self.positions[-1][0]-self.positions[0][0]
            if span<.9 or len(self.positions)<180:self.abort('insufficient continuous position window')
            self.q_hold=tuple(self.q);self.packet=command_bundle(self.q_hold)
            self.qualified_frames=self.samples
            self.state='PRE_SEND'
        if now-self.age[0x20]>PERIOD:return None
        if self.last_send is not None and now-self.last_send<PERIOD:return None
        if self.samples==self.last_sample_sent:return None
        self.last_send=now;self.last_sample_sent=self.samples;self.batches+=1
        return self.packet

    def mark_sent(self, now):
        """Called only after an accepted full batch (or explicitly simulated replay)."""
        if self.state == 'PRE_SEND':
            self.state='ACTIVE';self.started=now
            if self.proxy is not None:self.proxies.append((now,self.proxy.copy()))

    def summary(self):
        return dict(state=self.state,reason=self.reason,q_hold=self.q_hold,
                    kp=KP,kd=KD,tau_ff=[0.]*6,position_bound=E_BOUND,
                    velocity_bound=V_BOUND,proxy_bound=PROXY_BOUND,
                    mean_proxy_slew_bound=SLEW_BOUND,samples=self.samples,
                    qualified_frames=self.qualified_frames,candidate_batches=self.batches,
                    last_proxy=self.proxy,
                    exit_meaning='TX stops only; board state unknown until physical power isolation')


def replay(path):
    gate=GateA();packets=[]
    for line in Path(path).read_text().splitlines():
        r=json.loads(line);now=r['t_sec']
        try:
            gate.ingest(bytes.fromhex(r['bytes']),now)
            packet=gate.tick(now)
            if packet is not None:
                pre_send_snapshot(gate,packet,now,first=gate.started is None)
                packets.append(packet.hex());gate.mark_sent(now)
        except Abort:break
        if gate.state=='DONE':break
    result=gate.summary();result['offline_only']=True
    result['identical_candidate_packets']=len(set(packets))<=1
    result['first_packet_hex']=packets[0] if packets else None
    return result


# These facts are unresolved for the current board; an acknowledgement is not evidence.
UNKNOWNS = dict.fromkeys((
    'exact_board_firmware_version', 'board_internal_total_torque_limit',
    'actual_0x12_torque_scale', 'disconnect_behavior', 'persistent_board_state'), 'UNKNOWN')
REVIEW_FLAGS=(
 'explicit_current_session_gate_a_authorization', 'operator_present',
 'load_bearing_support_all_links', 'physical_power_cut_tested',
 'slot_mapping_signs_confirmed', 'partial_frame_update_risk_reviewed',
 'previous_control_state_and_gain_switch_reviewed', 'unknown_board_risks_acknowledged',
 'normal_and_abort_power_isolation_agreed', 'exclusive_port_confirmed')


def validate_review(review):
    if review.get('gate')!='A':raise Abort('review must identify Gate A')
    if review.get('firmware_version')!='UNKNOWN' or review.get('unknowns')!=UNKNOWNS:
        raise Abort('current board metadata must remain explicitly UNKNOWN')
    if not review.get('authorization_reference') or not review.get('power_off_procedure'):
        raise Abort('missing explicit authorization reference / physical stop procedure')
    expected=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    if review.get('reviewed_program_sha256')!=expected:
        raise Abort('review does not match this exact program revision')
    for k in REVIEW_FLAGS:
        if review.get(k) is not True:raise Abort('unmet review interlock: '+k)


def pre_send_snapshot(gate, packet, now, *, first=True):
    """Validate the actual encoded bytes against current feedback, without I/O."""
    if gate.state not in ('PRE_SEND','ACTIVE'):raise Abort('not qualified')
    if not math.isfinite(now):raise Abort('nonfinite timestamp')
    if len(packet)!=170:raise Abort('invalid command length')
    arrays=[]
    for i,cmd in enumerate(range(0x10,0x15)):
        f=packet[i*34:(i+1)*34]
        values=struct.unpack('<7f',f[4:32])
        if f!=frame(cmd,values):raise Abort('command format/checksum')
        if not all(math.isfinite(x) for x in values) or values[6]!=0:
            raise Abort('nonfinite command or unused slot')
        arrays.append(values)
    q=list(gate.q or []);v=list(gate.v or [])
    if len(q)!=6 or len(v)!=6 or not all(math.isfinite(x) for x in q+v):
        raise Abort('nonfinite/incomplete measured state')
    ages={hex(c):now-gate.age.get(c,float('-inf')) for c in NEEDED}
    if any(not 0<=age<=STALE for age in ages.values()) or ages['0x20']>PERIOD:
        raise Abort('pre-write stale feedback')
    cq=[s*x for s,x in zip(SIGNS,arrays[0][:6])]
    cv=[s*x for s,x in zip(SIGNS,arrays[1][:6])]
    ff=[s*x for s,x in zip(SIGNS,arrays[2][:6])]
    if tuple(cq)!=gate.q_hold:raise Abort('command does not equal latched q_hold')
    if any(cv+ff):raise Abort('nonzero velocity/feedforward command')
    for actual,expected in zip(arrays[3:5],(KP,KD)):
        if actual!=struct.unpack('<7f',struct.pack('<7f',*expected,0.)):
            raise Abort('gains changed')
    error=[c-m for c,m in zip(cq,q)];ve=[c-m for c,m in zip(cv,v)]
    if any(abs(e)>E_BOUND for e in error):raise Abort('position error guard')
    # Incoming and outgoing positions are the same float32 wire coordinates.
    # No numeric tolerance is needed or silently granted for the FIRST target.
    if first and any(e!=0 for e in error):raise Abort('first target differs from current measured q')
    if any(abs(x)>V_BOUND for x in v):raise Abort('velocity guard')
    margins=[min(x-lo,hi-x) for x,lo,hi in zip(q,LOWER,UPPER)]
    if min(margins)<MARGIN:raise Abort('joint limit margin')
    pt=[p*e for p,e in zip(KP,error)];dt=[d*e for d,e in zip(KD,ve)]
    proxy=[p+d+f for p,d,f in zip(pt,dt,ff)]
    ratios=[abs(x)/b for x,b in zip(proxy,PROXY_BOUND)]
    if max(ratios)>1+1e-12:raise Abort('pre-write torque proxy guard')
    gate.proxy=proxy
    return dict(kind='PRE_SEND_SNAPSHOT',timestamp=now,
        timestamp_basis='host monotonic seconds since capture/start; not board sampling time',
        measured=dict(q=q,dq=v,feedback_age=ages,timestamp=now,
                      feedback_receive_timestamps={hex(c):gate.age[c] for c in NEEDED}),
        command=dict(position=cq,velocity=cv,current=ff,kp=list(KP),kd=list(KD)),
        difference=dict(position_command_minus_measured=error,velocity_command_minus_measured=ve),
        limits=dict(joint_limit_margin=margins,minimum_margin=MARGIN,
                    position_error_guard=E_BOUND,velocity_guard=V_BOUND,
                    torque_proxy_guard=PROXY_BOUND,mean_proxy_slew_guard=SLEW_BOUND),
        torque_proxy=dict(p_term=pt,d_term=dt,ff_term=ff,total=proxy,
                         max_abs_tau_proxy=max(map(abs,proxy)),max_tau_proxy_ratio_to_guard=max(ratios)),
        first_position_equality_tolerance=0.,
        slew_initialization='no prior commanded proxy; old board torque UNKNOWN',
        unknowns=UNKNOWNS.copy(),
        theoretical_arrays={hex(c):list(v) for c,v in zip(range(0x10,0x15),arrays)},
        theoretical_packet_hex=packet.hex(),
        program_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


def persist_snapshot(snapshot, path):
    """Exclusive creation, flush/fsync and read-back before granting a TX permit."""
    text=json.dumps(snapshot,indent=2,allow_nan=False)+'\n'
    with Path(path).open('x') as out:
        if out.write(text)!=len(text):raise Abort('short snapshot write')
        out.flush();os.fsync(out.fileno())
    if Path(path).read_text()!=text:raise Abort('snapshot read-back mismatch')
    print(text,flush=True)
    return hashlib.sha256(text.encode()).hexdigest()


def first_write_barrier(gate, packet, now, persist, recheck, writer):
    """No writer call unless snapshot validation, persistence and final check succeed.

    writer is the sole transport capability; dry-run never receives it.
    """
    try:
        snapshot=pre_send_snapshot(gate,packet,now)
        receipt=persist(snapshot)
        if not receipt:raise Abort('snapshot persistence not proven')
        recheck()
        return writer(packet)
    except BaseException as exc:
        gate.state='ABORT';gate.reason='ABORT BEFORE FIRST WRITE: '+str(exc)
        raise


def dry_run(path, snapshot_path):
    """Saved RX bytes only. No live(), device opener, or transport writer capability."""
    gate=GateA()
    for line in Path(path).read_text().splitlines():
        r=json.loads(line);now=r['t_sec']
        gate.ingest(bytes.fromhex(r['bytes']),now)
        packet=gate.tick(now)
        if packet is None:continue
        snapshot=pre_send_snapshot(gate,packet,now)
        receipt=persist_snapshot(snapshot,snapshot_path)
        result=dict(status='READY TO WRITE',offline_only=True,
                    serial_open_count=0,serial_write_count=0,
                    snapshot_sha256=receipt,**gate.summary())
        print(json.dumps(result,indent=2),flush=True)
        return result
    raise Abort('capture ended before feedback qualification')


def live(review, port, log_path):
    """Only reachable with separate human review and typed acknowledgement."""
    validate_review(review)
    if not sys.stdin.isatty():raise Abort('live mode requires local human terminal')
    print('ACTIVE Gate A; all six joints; 2s PD; zero feedforward; support required.\n'
          'Normal/abort/crash: physical power isolation required. Close != disable.')
    if input('Type GATE A SUPPORTED POWER CUT READY: ')!='GATE A SUPPORTED POWER CUT READY':
        raise Abort('operator did not arm')
    import fcntl
    import stat
    import termios
    resolved=os.path.realpath(port)
    if resolved!='/dev/ttyUSB0':raise Abort('only the specifically reviewed port is supported')
    fd=None;gate=GateA();accepted=0;last_batch=None;exit_reason='not started'
    log=Path(log_path).open('x')  # fail before opening port if evidence cannot be created
    def emit(item):
        log.write(json.dumps(item,allow_nan=False)+'\n');log.flush()
    def interrupted(signum,frame):raise Abort('operator signal '+str(signum))
    old={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}
    started=time.monotonic()
    try:
        # O_RDWR opening can affect UART lines; it is part of active authorization.
        fd=os.open(resolved,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK|os.O_CLOEXEC)
        if not stat.S_ISCHR(os.fstat(fd).st_mode):raise Abort('not a character device')
        fcntl.ioctl(fd,termios.TIOCEXCL)
        attrs=termios.tcgetattr(fd);attrs[0]=attrs[1]=attrs[3]=0
        attrs[2]=termios.CS8|termios.CREAD|termios.CLOCAL
        attrs[4]=attrs[5]=termios.B921600
        attrs[6][termios.VMIN]=attrs[6][termios.VTIME]=0
        termios.tcsetattr(fd,termios.TCSANOW,attrs)
        started=time.monotonic()
        while gate.state not in ('DONE','ABORT'):
            readable=select.select([fd],[],[],.002)[0]
            if readable:
                try:b=os.read(fd,4096)
                except BlockingIOError:b=None
                if b==b'':raise Abort('EOF/disconnect')
                if b:
                    now=time.monotonic()-started
                    emit(dict(kind='RX',t=now,hex=b.hex()))
                    gate.ingest(b,now)
            now=time.monotonic()-started
            packet=gate.tick(now)
            if packet is not None:
                # Re-check wall time after logging/calculation before any write.
                if time.monotonic()-started-now>PERIOD:raise Abort('pre-write host delay')
                pending=struct.unpack('I',fcntl.ioctl(fd,termios.TIOCOUTQ,struct.pack('I',0)))[0]
                if pending:raise Abort('kernel TX queue not empty; do not accumulate commands')
                if time.monotonic()-started-now>PERIOD:raise Abort('pre-write queue-check delay')
                def recheck():
                    check_now=time.monotonic()-started
                    if check_now-now>PERIOD:raise Abort('pre-write snapshot/queue delay')
                    if gate.started is not None and check_now-gate.started>=DURATION:
                        raise Abort('deadline reached before write; no extra batch')
                    pre_send_snapshot(gate,packet,check_now,first=gate.started is None)
                def writer(data):
                    return os.write(fd,data)  # SOLE serial TX; no retries or exit frames
                if gate.started is None:
                    def persist(item):
                        receipt=persist_snapshot(item,str(log_path)+'.pre_send.json')
                        emit(dict(kind='PRE_SEND_SNAPSHOT',snapshot=item,sha256=receipt))
                        os.fsync(log.fileno())
                        return receipt
                    n=first_write_barrier(gate,packet,now,persist,recheck,writer)
                else:
                    recheck();n=writer(packet)
                accepted+=n;last_batch=dict(t=now,accepted_bytes=n,hex=packet[:n].hex())
                emit(dict(kind='TX_KERNEL_ACCEPTED',**last_batch))
                if n!=len(packet):raise Abort('partial TX; controller state unknown; DO NOT retry')
                gate.mark_sent(now)
        exit_reason='NORMAL EXIT: stop TX; physically isolate power now'
    except BaseException as exc:
        gate.state='ABORT';gate.reason=str(exc)
        exit_reason='ABORT EXIT: stop TX; physically isolate power now: '+str(exc)
    finally:
        # No last-moment hold, zero-gain, disable, reset, flush, or drain commands.
        if fd is not None:os.close(fd)
        for s,h in old.items():signal.signal(s,h)
        try:emit(dict(kind='EXIT',exit_reason=exit_reason,accepted_bytes=accepted,
                      last_kernel_accepted_batch=last_batch,**gate.summary()))
        finally:log.close()
        print(exit_reason+'\nUART closed does NOT prove motor disable.',flush=True)
    return gate.summary()


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    modes=ap.add_mutually_exclusive_group(required=True)
    modes.add_argument('--replay',type=Path)
    modes.add_argument('--dry-run',action='store_true')
    ap.add_argument('--capture',type=Path)
    ap.add_argument('--snapshot',type=Path)
    modes.add_argument('--execute-gate-a',action='store_true')
    ap.add_argument('--review-file',type=Path)
    ap.add_argument('--port',default='/dev/ttyUSB0')
    ap.add_argument('--log',type=Path)
    args=ap.parse_args(argv)
    if args.dry_run:
        if not args.capture or not args.snapshot:ap.error('dry-run needs --capture and --snapshot')
        dry_run(args.capture,args.snapshot);return 0
    if args.replay:
        print(json.dumps(replay(args.replay),indent=2));return
    if not args.review_file or not args.log:ap.error('live needs --review-file and --log')
    result=live(json.loads(args.review_file.read_text()),args.port,args.log)
    return 0 if result['state']=='DONE' else 1


if __name__=='__main__':raise SystemExit(main())
