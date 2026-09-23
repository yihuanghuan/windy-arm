"""All transport tests use fake in-memory devices; never access physical UART."""
import builtins
import importlib.util
import json
from pathlib import Path
import struct
import numpy as np
import pytest

P=Path(__file__).resolve().parents[2]/'tools/bounded_gravity_bringup.py'
spec=importlib.util.spec_from_file_location('bounded_under_test',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
PLAN_PATH=P.parents[3]/'docs/real_arm_audit_20260917/gravity_bringup/adaptive_20260917/plan_v1.json'
PLAN=json.loads(PLAN_PATH.read_text());Q=np.array(PLAN['q_geometry_center']);G=np.array(PLAN['g_preview'])

def frame(c,v):
    payload=struct.pack('<7H' if c==0x24 else '<7f',*v)
    b=b'\xfe\x55'+bytes([c,len(payload)])+payload
    return b+bytes((sum(b)%256,sum(sum(b[:i+1]) for i in range(len(b)))%256))

def telemetry(q=Q,v=None):
    v=np.zeros(6) if v is None else np.array(v)
    return frame(0x20,list(np.array(q)*m.SIGNS)+[0])+frame(0x21,list(v*m.SIGNS)+[0])+frame(0x22,[0]*7)+frame(0x24,[28]*6+[0])

def fb_state(q=Q,v=None,t=1.):
    fb=m.Feedback();fb.ingest(telemetry(q,v),t);return fb

def test_nominal_gains_and_packets():
    assert m.KP.tolist()==[30,30,30,5,5,5]
    assert m.KD.tolist()==[1,1,1,.1,.1,.1]
    b=m.packet(Q,.99*G);assert len(b)==170
    for i,c in enumerate(range(0x10,0x15)):
        f=b[i*34:(i+1)*34];assert f[:4]==bytes((254,170,c,28))
        assert f[-2]==sum(f[:-2])%256
        assert f[-1]==sum(sum(f[:j+1]) for j in range(32))%256
        assert struct.unpack('<7f',f[4:32])[6]==0
    assert struct.unpack('<7f',b[72:100])[:6]==pytest.approx(.99*G*np.array(m.SIGNS),rel=1e-6)

def test_lambda_skips_statically_infeasible_values():
    lam=m.choose_lambda(G,PLAN['envelope']);assert lam==.99
    assert m.choose_lambda(G,PLAN['envelope'],1)==1
    with pytest.raises(m.Abort,match='feasibility'):m.Monitor(PLAN,Q,G,.5)
    with pytest.raises(m.Abort,match='lambda'):m.Monitor(PLAN,Q,G,-.1)
    with pytest.raises(m.Abort,match='lambda'):m.Monitor(PLAN,Q,G,1.01)

@pytest.mark.parametrize('kind',['displacement','stale','nan','checksum','malformed','temperature','proxy'])
def test_hard_faults_reject(kind):
    mon=m.Monitor(PLAN,Q,G,.99);fb=fb_state();q=Q.copy();v=np.zeros(6)
    with pytest.raises(m.Abort):
        if kind=='displacement':q[1]+=.02;fb.ingest(telemetry(q),1.005)
        elif kind=='stale':mon.check(fb,1.1)
        elif kind=='nan':q[1]=np.nan;fb.ingest(telemetry(q),1.005)
        elif kind=='checksum':b=bytearray(telemetry());b[10]^=1;fb.ingest(b,1.005)
        elif kind=='malformed':fb.ingest(b'\xfe\x55\x20\x04'+b'\0'*6,1.005)
        elif kind=='temperature':fb.ingest(frame(0x24,[28]*5+[0,0]),1.005)
        elif kind=='proxy':v[1]=4;fb.ingest(telemetry(v=v),1.005)
        mon.check(fb,1.005)

def test_single_wrist_velocity_spike_is_warning():
    fb=fb_state(v=[0,0,0,0,.4,0]);mon=m.Monitor(PLAN,Q,G,.99)
    row=mon.check(fb,1);assert row['velocity_spike_warning'];assert mon.warning_count==1

def test_sustained_physical_speed_aborts():
    mon=m.Monitor(PLAN,Q,G,.99);fb=m.Feedback()
    with pytest.raises(m.Abort,match='sustained'):
        for i in range(20):
            q=Q.copy();q[1]+=.12*i*.005;fb.ingest(telemetry(q),i*.005);mon.check(fb,i*.005)

def test_current_limit_intersection_is_conservative():
    gm=m.GravityOnly();assert gm.m.upperPositionLimit[1]==1.57
    assert PLAN['upper'][1]==1.57
    assert min(np.minimum(Q-PLAN['lower'],np.array(PLAN['upper'])-Q))>.05

def test_triangle_crossing_desk_edge_is_not_missed():
    t=np.array([[[0,0,.10],[.1,0,0],[.1,.1,0]]])
    assert m.Geometry.clipped_min_z(t,.02)==pytest.approx(.08)

def test_snapshot_exact_wire_latch_no_offset():
    fb=fb_state(v=[.01]*6);mon,b,s=m.snapshot(PLAN,fb,m.GravityOnly(),1.,0)
    assert s['q_command_minus_measured']==[0]*6
    assert np.array_equal(s['position_command'],fb.q)
    assert s['lambda_value']==.99
    assert s['current_command']==pytest.approx(.99*np.array(s['g_hold']))
    assert np.array_equal(s['p_term'],np.zeros(6))

def test_dry_run_has_no_transport(tmp_path,monkeypatch):
    capture=PLAN_PATH.parent/'preflight_rx.jsonl'
    monkeypatch.setattr(m.os,'open',lambda *a,**k:pytest.fail('device open'))
    monkeypatch.setattr(m.os,'write',lambda *a,**k:pytest.fail('device write'))
    monkeypatch.setattr(m,'live',lambda *a,**k:pytest.fail('live entry'))
    r=m.dry_run(PLAN,capture,tmp_path/'snapshot.json')
    assert r['serial_open_count']==r['serial_write_count']==0

@pytest.mark.parametrize('persist_failure',[False,True])
def test_fake_live_transport_first_write_barrier_and_exit(tmp_path,monkeypatch,persist_failure):
    import termios,fcntl,stat
    clock=[0.];writes=[];closes=[];persist_calls=[]
    class FakeStat:st_mode=stat.S_IFCHR
    monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(m.os,'open',lambda *a,**k:9999)
    monkeypatch.setattr(m.os,'fstat',lambda fd:FakeStat())
    monkeypatch.setattr(m.os,'read',lambda *a:telemetry())
    monkeypatch.setattr(m.os,'write',lambda fd,data:writes.append(data) or len(data))
    monkeypatch.setattr(m.os,'close',lambda fd:closes.append(fd))
    monkeypatch.setattr(m.os,'fsync',lambda fd:None)
    def selected(*a):clock[0]+=.005;return ([9999],[],[])
    monkeypatch.setattr(m.select,'select',selected)
    monkeypatch.setattr(fcntl,'ioctl',lambda *a:struct.pack('I',0))
    monkeypatch.setattr(termios,'tcgetattr',lambda fd:[0,0,0,0,0,0,[0]*32])
    monkeypatch.setattr(termios,'tcsetattr',lambda *a:None)
    monkeypatch.setattr(m.signal,'signal',lambda *a:None)
    monkeypatch.setattr(m.sys.stdin,'isatty',lambda:True)
    monkeypatch.setattr(builtins,'input',lambda *a:'BOUNDED GRAVITY READY')
    if persist_failure:
        def broken(*a):persist_calls.append(True);raise OSError('injected snapshot fsync failure')
        monkeypatch.setattr(m,'persist',broken)
    review=dict(program_sha256=m.sha(P),plan_sha256=m.hashlib.sha256(json.dumps(PLAN,sort_keys=True).encode()).hexdigest())
    review.update({k:True for k in ['current_user_authorization','operator_present_power_isolation_available','no_support_no_extra_load','workspace_clear','accept_unknown_firmware_risk']})
    r=m.live(PLAN,review,tmp_path/'log.jsonl')
    assert closes==[9999]
    if persist_failure:
        assert persist_calls and writes==[] and r['state']=='ABORT_BEFORE_FIRST_WRITE'
    else:
        assert r['state']=='NORMAL_COMPLETION' and len(writes)>10
        assert len(set(writes))==1
        assert r['restore_reference_changed'] is False
        records=[json.loads(x) for x in (tmp_path/'log.jsonl').read_text().splitlines()]
        assert next(i for i,x in enumerate(records) if x['kind']=='PRE_SEND_SNAPSHOT')<next(i for i,x in enumerate(records) if x['kind']=='TX_KERNEL_ACCEPTED')
