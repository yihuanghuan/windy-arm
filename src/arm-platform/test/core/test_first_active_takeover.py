"""Pure Gate-A replay/synthetic tests. Never invoke live() or open any device."""
import importlib.util
import json
from pathlib import Path
import struct
import pytest

PATH=Path(__file__).resolve().parents[2]/'tools/first_active_takeover.py'
spec=importlib.util.spec_from_file_location('takeover_under_test',PATH)
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
Q=[-.03,1.35,-1.33,.002,.007,.002]

def rx(cmd,values):
    payload=struct.pack('<7H' if cmd==0x24 else '<7f',*values)
    b=b'\xfe\x55'+bytes((cmd,len(payload)))+payload
    total=acc=0
    for x in b:total=(total+x)&255;acc=(acc+total)&255
    return b+bytes((total,acc))

def telemetry(q=Q,v=None):
    v=v or [0.]*6
    rawq=[s*x for s,x in zip(a.SIGNS,q)]+[0.]
    rawv=[s*x for s,x in zip(a.SIGNS,v)]+[0.]
    return rx(0x20,rawq)+rx(0x21,rawv)+rx(0x22,[0.]*7)+rx(0x24,[26]*6+[0])

def qualify():
    gate=a.GateA();packet=None
    for i in range(210):
        t=i*.005
        gate.ingest(telemetry(),t);p=gate.tick(t)
        if p is not None:packet=p;break
    assert gate.state=='PRE_SEND' and packet is not None
    a.pre_send_snapshot(gate,packet,t)
    gate.mark_sent(t)  # simulated accepted batch, no transport
    return gate,t,packet

def test_no_command_before_full_continuous_feedback():
    gate=a.GateA()
    for i in range(199):
        gate.ingest(telemetry(),i*.005)
        assert gate.tick(i*.005) is None
    assert gate.q_hold is None

def test_no_feedback_times_out_without_candidate():
    gate=a.GateA();assert gate.tick(0) is None
    with pytest.raises(a.Abort):gate.tick(2.01)
    assert gate.batches==0

def test_first_target_is_measured_and_every_packet_remains_identical():
    gate,t,p=qualify();hold=gate.q_hold
    assert hold==pytest.approx(Q)
    packets=[p]
    for i in range(1,405):
        q=Q.copy();q[5]+=a.P_QUANT*(i%2)
        now=t+i*.005
        gate.ingest(telemetry(q),now);b=gate.tick(now)
        if b:packets.append(b)
    assert gate.state=='DONE' and len(set(packets))==1
    assert gate.q_hold==hold and gate.tick(t+100) is None

def test_frames_are_exact_seven_slots_zero_ff_and_no_enable_frame():
    _,_,packet=qualify();assert len(packet)==170
    frames=[packet[i:i+34] for i in range(0,170,34)]
    assert [f[2] for f in frames]==[0x10,0x11,0x12,0x13,0x14]
    for f in frames:
        assert f[:2]==b'\xfe\xaa' and f[3]==28
        assert f[-2]==sum(f[:-2])%256
        assert f[-1]==sum(sum(f[:i+1]) for i in range(32))%256
        assert struct.unpack('<7f',f[4:-2])[6]==0
    assert struct.unpack('<7f',frames[0][4:-2])[:6]==pytest.approx([s*q for s,q in zip(a.SIGNS,Q)])
    assert struct.unpack('<7f',frames[2][4:-2])==(0.,)*7

def test_stale_abort_never_generates_exit_command():
    gate,t,_=qualify();n=gate.batches
    with pytest.raises(a.Abort):gate.tick(t+.051)
    assert gate.tick(t+.06) is None and gate.batches==n

def test_host_pause_cannot_be_hidden_by_new_buffered_feedback():
    gate,t,_=qualify()
    with pytest.raises(a.Abort):gate.ingest(telemetry(),t+.051)
    assert gate.state=='ABORT'

@pytest.mark.parametrize('kind',['position','velocity','temperature','nan','checksum','length','limit'])
def test_faults_abort_without_further_command(kind):
    gate,t,_=qualify();q=Q.copy();v=[0.]*6
    if kind=='position':q[0]+=a.E_BOUND*2
    if kind=='velocity':v[0]=a.V_BOUND*1.01
    if kind=='nan':q[0]=float('nan')
    if kind=='limit':q[2]=-1.55
    data=telemetry(q,v)
    if kind=='temperature':data=rx(0x24,[26]*5+[0,0])
    if kind=='checksum':data=bytearray(data);data[10]^=1
    if kind=='length':data=b'\xfe\x55\x20\x04'+b'\0'*6
    with pytest.raises(a.Abort):gate.ingest(data,t+.005)
    assert gate.state=='ABORT' and gate.tick(t+.006) is None

def test_observed_velocity_spike_does_not_mean_unsafe_motion_by_itself():
    gate,t,_=qualify()
    for i in range(1,15):
        gate.ingest(telemetry(v=[0,0,0,0,.2319908 if i==2 else 0,0]),t+i*.005)
        gate.tick(t+i*.005)
    assert gate.state=='ACTIVE'

def test_sustained_proxy_mean_slew_is_checked():
    gate,t,_=qualify()
    gate.previous_mean=(t,[-a.PROXY_BOUND[0]]*3+[0.]*3)
    with pytest.raises(a.Abort,match='slew'):
        for i in range(1,20):
            now=t+i*.005
            gate.ingest(telemetry(v=[-a.V_BOUND*.99]*3+[0.]*3),now)
            gate.tick(now)

def test_review_example_is_closed_by_default():
    with pytest.raises(a.Abort):a.validate_review({})
    review=dict(gate='A',firmware_version='test',authorization_reference='test',power_off_procedure='test')
    for key in a.REVIEW_FLAGS:review[key]=False
    with pytest.raises(a.Abort):a.validate_review(review)

def test_capture_replay_never_opens_devices_or_sockets(monkeypatch):
    # Keep replay structurally incapable of calling transport.
    monkeypatch.setattr(a,'live',lambda *args:pytest.fail('transport called'))
    monkeypatch.setattr(a.os,'open',lambda *args,**kwargs:pytest.fail('fd open called'))
    root=PATH.parents[3]
    capture=root/'docs/real_arm_audit_20260917/firmware_recheck/capture.chunks.jsonl'
    if not capture.exists():pytest.skip('local capture not distributed')
    result=a.replay(capture)
    assert result['state']=='DONE' and result['identical_candidate_packets']


def prepared():
    gate,t,packet=qualify()
    gate.state='PRE_SEND';gate.started=None
    return gate,t,packet


def barrier(gate,t,packet,persist=None,recheck=None):
    writes=[]
    a.first_write_barrier(gate,packet,t,
                         persist or (lambda snapshot:'fake-durable-receipt'),
                         recheck or (lambda:None),
                         lambda data:writes.append(data))
    return writes


@pytest.mark.parametrize('fault', ['mismatch','stale','nan','velocity','proxy','limit'])
def test_first_write_guard_rejects_before_writer(fault,monkeypatch):
    gate,t,packet=prepared();writes=[]
    if fault=='mismatch':gate.q[0]+=a.P_QUANT  # inside old bound, still rejects first mismatch
    if fault=='stale':t+=a.STALE*2
    if fault=='nan':gate.v[0]=float('nan')
    if fault=='velocity':gate.v[0]=a.V_BOUND*2
    if fault=='proxy':
        gate.v[0]=a.V_BOUND*.5
        monkeypatch.setattr(a,'PROXY_BOUND',(0.001,)*6)
    if fault=='limit':
        gate.q[2]=-1.55;gate.q_hold=tuple(gate.q);packet=a.command_bundle(gate.q_hold)
    with pytest.raises(a.Abort):
        a.first_write_barrier(gate,packet,t,lambda _:pytest.fail('invalid state persisted'),
                             lambda:None,lambda data:writes.append(data))
    assert writes==[] and gate.state=='ABORT' and gate.started is None


def test_prewrite_proxy_uses_damping_and_reports_wire_arrays():
    gate,t,packet=prepared();gate.v=[.01,-.02,.03,-.04,.05,-.06]
    snap=a.pre_send_snapshot(gate,packet,t)
    assert snap['command']['position']==snap['measured']['q']
    assert snap['torque_proxy']['p_term']==[0]*6
    assert snap['torque_proxy']['ff_term']==[0]*6
    assert snap['torque_proxy']['total']==pytest.approx([-.003,.006,-.009,.0012,-.0015,.0018])
    assert snap['unknowns']==a.UNKNOWNS
    assert snap['theoretical_arrays']['0x10'][5]==-gate.q[5]


@pytest.mark.parametrize('stage', ['persist','recheck'])
def test_exception_before_first_write_has_zero_tx(stage):
    gate,t,packet=prepared();writes=[]
    def broken(*_):raise OSError('injected failure')
    with pytest.raises(OSError):
        a.first_write_barrier(gate,packet,t,broken if stage=='persist' else lambda _: 'receipt',
                             broken if stage=='recheck' else lambda:None,
                             lambda data:writes.append(data))
    assert writes==[] and gate.state=='ABORT' and gate.started is None


def test_snapshot_fsync_failure_blocks_tx(tmp_path,monkeypatch):
    gate,t,packet=prepared();writes=[]
    def fail(_):raise OSError('fsync failed')
    monkeypatch.setattr(a.os,'fsync',fail)
    with pytest.raises(OSError):
        a.first_write_barrier(gate,packet,t,
            lambda s:a.persist_snapshot(s,tmp_path/'snapshot.json'),lambda:None,
            lambda data:writes.append(data))
    assert writes==[] and gate.state=='ABORT'


def test_no_receipt_blocks_tx():
    gate,t,packet=prepared();writes=[]
    with pytest.raises(a.Abort,match='persistence'):
        a.first_write_barrier(gate,packet,t,lambda _:None,lambda:None,lambda d:writes.append(d))
    assert writes==[]


def test_snapshot_durably_saved_before_writer(tmp_path):
    gate,t,packet=prepared();path=tmp_path/'snapshot.json';order=[]
    def writer(data):
        snap=json.loads(path.read_text())
        assert snap['kind']=='PRE_SEND_SNAPSHOT'
        assert snap['theoretical_packet_hex']==data.hex()
        assert order==['rechecked']
        order.append('writer')
    a.first_write_barrier(gate,packet,t,lambda s:a.persist_snapshot(s,path),
                         lambda:order.append('rechecked'),writer)
    assert order==['rechecked','writer']


def test_unknown_firmware_is_risk_not_fake_known():
    review=dict(gate='A',firmware_version='UNKNOWN',unknowns=a.UNKNOWNS.copy(),
                authorization_reference='SYNTHETIC TEST ONLY',power_off_procedure='SYNTHETIC',
                reviewed_program_sha256=a.hashlib.sha256(PATH.read_bytes()).hexdigest())
    review.update({k:True for k in a.REVIEW_FLAGS})
    a.validate_review(review)  # Pure validation, never an execution permit for hardware.
    assert all(v=='UNKNOWN' for v in review['unknowns'].values())
    review['unknowns']['actual_0x12_torque_scale']=1.033
    with pytest.raises(a.Abort):a.validate_review(review)


def test_dry_run_never_opens_or_writes_serial(tmp_path,monkeypatch):
    capture=PATH.parents[3]/'docs/real_arm_audit_20260917/ros_rviz_readonly/capture.chunks.jsonl'
    monkeypatch.setattr(a,'live',lambda *x:pytest.fail('live called'))
    monkeypatch.setattr(a.os,'open',lambda *x,**kw:pytest.fail('device open called'))
    monkeypatch.setattr(a.os,'write',lambda *x,**kw:pytest.fail('serial write called'))
    monkeypatch.setattr(a.select,'select',lambda *x:pytest.fail('serial read polling called'))
    result=a.dry_run(capture,tmp_path/'snapshot.json')
    assert result['status']=='READY TO WRITE'
    assert result['state']=='PRE_SEND'
    assert result['serial_open_count']==result['serial_write_count']==0
    assert json.loads((tmp_path/'snapshot.json').read_text())['kind']=='PRE_SEND_SNAPSHOT'
