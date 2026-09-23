"""Fake transport integration only. These tests never open a physical device."""
import builtins
import hashlib
import json
from pathlib import Path
import struct
import sys
import numpy as np
import pytest

TOOLS=Path(__file__).resolve().parents[2]/'tools'
sys.path.insert(0,str(TOOLS))
import gravity_tuning_runner as m

P=m.ROOT/'docs/real_arm_audit_20260917/gravity_tuning/comm_recheck_20260918/plan01.json'


def frame(c,v):
    p=struct.pack('<7H' if c==0x24 else '<7f',*v);b=b'\xfe\x55'+bytes([c,len(p)])+p
    return b+bytes((sum(b)%256,sum((len(b)-i)*x for i,x in enumerate(b))%256))


def telemetry(q,v=None):
    v=np.zeros(6) if v is None else v
    return frame(0x20,list(np.array(q)*m.comm.SIGNS)+[0])+frame(0x21,list(v*m.comm.SIGNS)+[0])+frame(0x22,[0]*7)+frame(0x24,[28]*6+[0])


def plan():
    p=json.loads(P.read_text());p['hashes']=m.hashes();return p


def test_snapshot_packet_matches_feedback_and_frozen_gravity():
    p=plan();fb=m.comm.Quality(p['critical_age_limit']);fb.feed(telemetry(p['q_geometry_center']),1.)
    _,pkt,s=m.snapshot(p,fb,1.)
    assert s['q_command_minus_measured']==[0]*6
    assert s['current_command']==pytest.approx(p['lambda_value']*np.array(s['g_hold']))
    assert len(pkt)==170
    for i,c in enumerate(range(0x10,0x15)):
        f=pkt[i*34:(i+1)*34];assert f[:4]==bytes([254,170,c,28])
        assert f[-2]==sum(f[:-2])%256
        assert f[-1]==sum((32-k)*x for k,x in enumerate(f[:-2]))%256
        assert struct.unpack('<7f',f[4:-2])[6]==0


def test_dry_run_has_zero_serial_open_write(tmp_path,monkeypatch):
    monkeypatch.setattr(m.os,'open',lambda *a,**k:pytest.fail('device open'))
    monkeypatch.setattr(m.os,'write',lambda *a,**k:pytest.fail('device write'))
    r=m.dry_run(plan(),P.parent/'receive/preflight01.jsonl',tmp_path/'snapshot.json')
    assert r['serial_open_count']==r['serial_write_count']==0


@pytest.mark.parametrize('fault',['none','isolated_effort_checksum','snapshot_failure','critical_stale','critical_burst','partial_write','write_exception','accelerating_j5','nonfinite'])
def test_fake_live_barriers_and_close(tmp_path,monkeypatch,fault):
    import stat,fcntl,termios
    p=plan();clock=[0.];writes=[];closes=[]
    class FakeStat:st_mode=stat.S_IFCHR
    monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(m.os,'open',lambda *a,**k:9999)
    monkeypatch.setattr(m.os,'fstat',lambda *a:FakeStat())
    monkeypatch.setattr(m.os,'close',lambda fd:closes.append(fd))
    monkeypatch.setattr(m.os,'fsync',lambda *a:None)
    def select(*a):clock[0]+=.005;return ([9999],[],[])
    monkeypatch.setattr(m.select,'select',select)
    monkeypatch.setattr(fcntl,'ioctl',lambda *a:struct.pack('I',0))
    monkeypatch.setattr(termios,'tcgetattr',lambda *a:[0,0,0,0,0,0,[0]*32])
    monkeypatch.setattr(termios,'tcsetattr',lambda *a:None)
    monkeypatch.setattr(m.signal,'signal',lambda *a:None)
    monkeypatch.setattr(m.sys.stdin,'isatty',lambda:True)
    monkeypatch.setattr(builtins,'input',lambda *a:'BOUNDED TUNING READY')
    def read(*a):
        q=np.array(p['q_geometry_center']);v=np.zeros(6);t=clock[0]
        if fault=='accelerating_j5' and t>1.01:q[4]+=2*(t-1.01)**2;v[4]=4*(t-1.01)
        if fault=='nonfinite' and t>1.04:q[4]=np.nan
        if fault=='critical_stale' and t>1.04:return frame(0x22,[0]*7)+frame(0x24,[28]*6+[0])
        b=telemetry(q,v)
        if fault=='isolated_effort_checksum' and 1.04<t<1.046:
            f=bytearray(frame(0x22,[0]*7));f[-1]^=1;b+=bytes(f)
        if fault=='critical_burst' and t>1.04:
            f=bytearray(frame(0x20,[0]*7));f[-1]^=1;b+=bytes(f)*2
        return b
    monkeypatch.setattr(m.os,'read',read)
    def write(fd,b):
        writes.append(b)
        if fault=='write_exception':raise OSError('injected write exception')
        return 10 if fault=='partial_write' else len(b)
    monkeypatch.setattr(m.os,'write',write)
    if fault=='snapshot_failure':
        monkeypatch.setattr(m.geometry,'persist',lambda *a:(_ for _ in ()).throw(OSError('snapshot failed')))
    review=dict(hashes=m.hashes(),plan_sha256=hashlib.sha256(json.dumps(p,sort_keys=True).encode()).hexdigest(),
        current_bounded_tuning_authorization=True,operator_present_isolation_available=True,
        workspace_clear_no_support_no_load=True,communication_recheck_passed=True)
    path=tmp_path/'run.jsonl';r=m.live(p,review,path)
    assert closes==[9999]
    rows=[json.loads(x) for x in path.read_text().splitlines()]
    if fault in ('none','isolated_effort_checksum'):
        assert r['state']=='NORMAL' and r['restore_result'].startswith('SUCCESS')
        assert len(writes)>10 and len(set(writes))==1
    elif fault=='snapshot_failure':
        assert not writes and r['state']=='HARD_ABORT_BEFORE_FIRST_WRITE'
    else:
        assert r['state']=='HARD_ABORT' and r['restore_result']=='PROHIBITED_AFTER_HARD_ABORT'
    if writes:
        assert next(i for i,x in enumerate(rows) if x['kind']=='PRE_SEND_SNAPSHOT')<next(i for i,x in enumerate(rows) if x['kind'] in ('TX_KERNEL_ACCEPTED','EXIT'))
        assert len(set(writes))==1  # no altered target, disable, restore, reset or exit packet


def test_review_hash_mismatch_before_open(tmp_path,monkeypatch):
    monkeypatch.setattr(m.os,'open',lambda *a,**k:pytest.fail('device open'))
    with pytest.raises(m.Abort):m.live(plan(),dict(hashes={}),tmp_path/'x')
