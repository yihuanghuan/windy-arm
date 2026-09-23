"""Offline FE55 corruption/recovery and communication-tier tests."""
import importlib.util
from pathlib import Path
import struct
import pytest

P=Path(__file__).resolve().parents[2]/'tools/gravity_comm.py'
s=importlib.util.spec_from_file_location('comm_test',P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)


def frame(c,values=None):
    values=[1]*7 if values is None else values
    payload=struct.pack('<7H' if c==0x24 else '<7f',*values)
    b=b'\xfe\x55'+bytes([c,len(payload)])+payload
    return b+bytes((sum(b)%256,sum((len(b)-i)*x for i,x in enumerate(b))%256))


def good():return frame(0x20)+frame(0x21)+frame(0x22)+frame(0x24)


def bad(c):
    b=bytearray(frame(c));b[-1]^=1;return bytes(b)


def test_isolated_effort_error_warning_and_state_not_poisoned():
    q=m.Quality(.025);q.feed(good(),0);before=q.latest[0x22]
    level,_=q.feed(bad(0x22)+frame(0x20)+frame(0x21),.005)
    assert level=='COMM_WARNING' and q.latest[0x22]==before
    assert q.counts['0x22']==1 and q.errors['checksum']==1


def test_missing_checksum_resyncs_to_next_valid_frame():
    q=m.Quality(.025);q.feed(good(),0)
    b=frame(0x22);level,_=q.feed(b[:-2]+b[-1:]+frame(0x20)+frame(0x21),.005)
    assert level=='COMM_WARNING' and q.counts['0x20']==2


def test_isolated_critical_error_recovered_in_time():
    q=m.Quality(.025);q.feed(good(),0)
    assert q.feed(bad(0x20)+frame(0x21),.005)[0]=='COMM_WARNING'
    assert q.feed(good(),.010)[0]=='COMM_WARNING'
    assert not q.hard_reason


def test_stale_position_hard():
    q=m.Quality(.025);q.feed(good(),0);q.feed(frame(0x21),.020)
    assert q.check(.025)[0]=='COMM_HARD_ABORT'


def test_two_consecutive_critical_bad_hard():
    q=m.Quality(.025);q.feed(good(),0)
    assert q.feed(bad(0x20)+bad(0x21),.005)[0]=='COMM_HARD_ABORT'


def test_nonconsecutive_three_bad_soft():
    q=m.Quality(.025)
    for i in range(160):
        q.feed(good(),i*.005)
        if i in (2,60,140):q.feed(bad(0x22),i*.005)
    assert q.check(.795)[0]=='COMM_SOFT_FAIL'


def test_burst_error_rate_hard():
    q=m.Quality(.025);q.feed(good(),0)
    assert q.feed(bad(0x22)*5,.005)[0]=='COMM_HARD_ABORT'


def test_diagnostics_missing_do_not_stale_state():
    q=m.Quality(.025)
    for i in range(100):assert q.feed(frame(0x20)+frame(0x21),i*.005)[0]=='COMM_OK'


def test_nonfinite_critical_hard_and_no_update():
    q=m.Quality(.025);q.feed(good(),0);old=q.q.copy()
    assert q.feed(frame(0x20,[float('nan')]*7),.005)[0]=='COMM_HARD_ABORT'
    assert (q.q==old).all()


def test_saved_isolated_error_now_warning():
    p=P.parents[3]/'docs/real_arm_audit_20260917/gravity_tuning/session_20260918/preflight_rx.jsonl'
    q=m.Quality(.025)
    for t,b in m.rx_rows(p):q.feed(b,t)
    assert q.check(t)[0]=='COMM_WARNING' and q.errors['checksum']==1


def test_isolated_malformed_warning():
    q=m.Quality(.025);q.feed(good(),0)
    assert q.feed(b'\xfe\x55\x22\x02'+frame(0x20)+frame(0x21),.005)[0]=='COMM_WARNING'


def test_io_exception_preflight_closes_no_retry(tmp_path,monkeypatch):
    closed=[]
    class Fake:
        fd=9000
        def __init__(self,*a,**k):pass
        def read(self):raise OSError('injected disconnect')
        def close(self):closed.append(True)
    monkeypatch.setattr(m,'Receiver',Fake)
    monkeypatch.setattr(m.select,'select',lambda *a:([9000],[],[]))
    results=m.preflight(tmp_path,.025,5)
    assert len(results)==1 and closed==[True]
    assert results[0]['status']=='COMM_HARD_ABORT' and results[0]['serial_write_count']==0
