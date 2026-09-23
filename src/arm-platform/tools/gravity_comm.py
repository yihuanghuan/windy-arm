"""FE55 integrity tiers and receive-only preflight. Never sends serial data."""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import select
import struct
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/real_arm_readonly'))
from real_arm_readonly.receiver import Receiver

LENGTHS={0x20:28,0x21:28,0x22:28,0x23:28,0x24:14}
CRITICAL=(0x20,0x21)
SIGNS=np.array([1,1,1,-1,-1,-1])


class Decoder:
    def __init__(self):self.buffer=b'';self.discarded=0
    def feed(self,data,t):
        self.buffer+=data;events=[]
        while self.buffer:
            i=self.buffer.find(b'\xfe\x55')
            if i<0:
                keep=int(self.buffer.endswith(b'\xfe'));self.discarded+=len(self.buffer)-keep
                self.buffer=self.buffer[-1:] if keep else b'';break
            self.discarded+=i;self.buffer=self.buffer[i:]
            if len(self.buffer)<4:break
            cmd,n=self.buffer[2:4]
            if (cmd in LENGTHS and n!=LENGTHS[cmd]) or n>128:
                events.append(dict(t=t,cmd=cmd,error='malformed',hex=self.buffer[:4].hex()))
                self.buffer=self.buffer[1:];continue
            if len(self.buffer)<n+6:break
            f=self.buffer[:n+6];expected=bytes((sum(f[:-2])%256,sum((len(f)-2-k)*b for k,b in enumerate(f[:-2]))%256))
            if f[-2:]!=expected:
                events.append(dict(t=t,cmd=cmd,error='checksum',hex=f.hex(),expected=expected.hex()))
                self.buffer=self.buffer[1:];continue
            self.buffer=self.buffer[n+6:];v=None
            if cmd in LENGTHS:
                v=struct.unpack('<7H' if cmd==0x24 else '<7f',f[4:-2])
                if not all(math.isfinite(x) for x in v):
                    events.append(dict(t=t,cmd=cmd,error='nonfinite',hex=f.hex()));continue
            events.append(dict(t=t,cmd=cmd,error=None,values=v))
        return events


class Quality:
    def __init__(self,age_limit):
        if not .020<=age_limit<=.030:raise ValueError('baseline-derived age must be 20..30ms')
        self.age_limit=age_limit;self.decoder=Decoder();self.latest={};self.at={}
        self.times={};self.counts=Counter();self.errors=Counter();self.error_events=[]
        self.events=deque();self.critical_bad=0;self.max_critical_bad=0;self.bad_streak=0;self.max_bad_streak=0
        self.generation=0;self.positions=deque();self.hard_reason=None;self.soft_seen=False
        self.chunk_sizes=[];self.chunk_times=[];self.max_age=0.;self.last_chunk=None
    @property
    def q(self):return np.array(self.latest[0x20][:6])*SIGNS
    @property
    def v(self):return np.array(self.latest[0x21][:6])*SIGNS
    def feed(self,data,t):
        if self.last_chunk is not None and t<self.last_chunk:self.hard_reason='nonmonotonic host clock'
        self.last_chunk=t;self.chunk_sizes.append(len(data));self.chunk_times.append(t)
        for e in self.decoder.feed(data,t):
            self.events.append(e);cmd=e['cmd']
            if e['error']:
                self.errors[e['error']]+=1;self.error_events.append(e)
                self.bad_streak+=1;self.max_bad_streak=max(self.max_bad_streak,self.bad_streak)
                if cmd in CRITICAL:
                    self.critical_bad+=1;self.max_critical_bad=max(self.max_critical_bad,self.critical_bad)
                    if self.critical_bad>=2:self.hard_reason='consecutive critical corrupt frames'
                    if e['error']=='nonfinite':self.hard_reason='nonfinite critical state'
            else:
                self.bad_streak=0
                if cmd in CRITICAL:self.critical_bad=0
                self.counts[hex(cmd)]+=1;self.at[cmd]=t;self.times.setdefault(cmd,[]).append(t)
                if e['values'] is not None:self.latest[cmd]=e['values']
                if cmd==0x20:
                    self.generation+=1;self.positions.append((t,self.q.copy()))
                    while self.positions and t-self.positions[0][0]>1.2:self.positions.popleft()
            self._rates(t)
        return self.check(t)
    def _rates(self,t):
        while self.events and t-self.events[0]['t']>1:self.events.popleft()
        recent=[e for e in self.events if t-e['t']<=.25]
        bad=sum(e['error'] is not None for e in recent)
        # At normal ~824 total frames/s: 5 bad / .25s ~2.4%, far above zero baseline.
        if bad>=5 and bad/max(1,len(recent))>=.02:self.hard_reason='sustained error-rate degradation'
        if sum(e['error'] is not None for e in self.events)>=3:self.soft_seen=True
    def check(self,t):
        self._rates(t)
        if self.hard_reason:return 'COMM_HARD_ABORT',self.hard_reason
        if all(c in self.at for c in CRITICAL):
            ages=[t-self.at[c] for c in CRITICAL];self.max_age=max(self.max_age,max(ages))
            if min(ages)<0 or max(ages)>=self.age_limit:
                self.hard_reason='critical state stale';return 'COMM_HARD_ABORT',self.hard_reason
        elif t>=self.age_limit:
            self.hard_reason='critical state not established';return 'COMM_HARD_ABORT',self.hard_reason
        if self.soft_seen:return 'COMM_SOFT_FAIL','multiple nonconsecutive errors / 1s'
        if self.error_events:return 'COMM_WARNING','isolated dropped corrupt frame(s)'
        return 'COMM_OK','valid feedback'
    def summary(self,t):
        streams={}
        for c,tt in self.times.items():
            steady=np.array([x for x in tt if x>=.05]);gap=np.diff(steady)
            streams[hex(c)]=dict(count=len(tt),rate_hz=float((len(steady)-1)/(steady[-1]-steady[0])) if len(steady)>1 and steady[-1]>steady[0] else None,
                max_gap_s=float(max(gap)) if len(gap) else None,
                p99_gap_s=float(np.quantile(gap,.99)) if len(gap) else None,
                p999_gap_s=float(np.quantile(gap,.999)) if len(gap) else None,
                final_age_s=t-tt[-1])
        return dict(status=self.check(t)[0],reason=self.check(t)[1],duration_s=t,
            counts=dict(self.counts),errors=dict(self.errors),error_events=self.error_events,
            streams=streams,critical_age_limit=self.age_limit,max_critical_age_s=self.max_age,
            max_consecutive_bad=self.max_bad_streak,max_consecutive_critical_bad=self.max_critical_bad,
            error_fraction=len(self.error_events)/max(1,sum(self.counts.values())+len(self.error_events)),
            chunks=len(self.chunk_sizes),max_chunk_bytes=max(self.chunk_sizes,default=0),
            max_inter_chunk_gap_s=float(max(np.diff(self.chunk_times))) if len(self.chunk_times)>1 else None,
            discarded_bytes=self.decoder.discarded,incomplete_tail_hex=self.decoder.buffer.hex(),
            q=self.q.tolist() if 0x20 in self.latest else None,
            dq=self.v.tolist() if 0x21 in self.latest else None)


def rx_rows(path):
    for line in path.read_text().splitlines():
        r=json.loads(line)
        if 't_sec' in r and 'bytes' in r:yield r['t_sec'],bytes.fromhex(r['bytes'])
        elif r.get('kind')=='RX':yield r['t'],bytes.fromhex(r['hex'])


def baseline(root):
    files=[];pooled={c:[] for c in CRITICAL}
    for p in sorted(root.rglob('*.jsonl')):
        if 'comm_recheck' in str(p):continue
        q=Quality(.030);count=0
        for t,b in rx_rows(p):q.feed(b,t);count+=1
        if count==0:continue
        s=q.summary(t);normal=not q.error_events and all(len(q.times.get(c,[]))>=100 for c in CRITICAL)
        files.append(dict(path=str(p),included=normal,summary=s))
        if normal:
            for c in CRITICAL:
                ts=np.array(q.times[c]);ts=ts[ts>=.05];pooled[c].extend(np.diff(ts).tolist())
    stats={hex(c):dict(samples=len(v),max_gap_s=max(v),p99_gap_s=float(np.quantile(v,.99)),p999_gap_s=float(np.quantile(v,.999))) for c,v in pooled.items()}
    maxgap=max(x['max_gap_s'] for x in stats.values())
    typical=float(np.median([x for v in pooled.values() for x in v if x>0]))
    # Cover the observed worst gap plus two nominal feedback periods, and
    # independently at least twice p99.9 plus one period. No silent >30ms cap.
    limit=max(.020,math.ceil(max(maxgap+2*typical,max(x['p999_gap_s'] for x in stats.values())*2+typical)*1000)/1000)
    return dict(files=files,critical_gap_stats=stats,proposed_critical_age_limit=limit,
        typical_period_s=typical,acceptable_for_live=limit<=.030,
        derivation='ceil_ms(max(historical max gap + 2 median periods, 2*p99.9 + median period)), minimum20ms; refuse above30ms')


def preflight(out,age_limit,seconds=8.):
    if not 5<=seconds<=10:raise ValueError('5..10 seconds required')
    out.mkdir(parents=True,exist_ok=True);attempts=[]
    for attempt in range(1,4):
        quality=Quality(age_limit);receiver=None;start=time.monotonic();io_error=None
        try:
            with (out/f'preflight{attempt:02}.jsonl').open('x') as f:
                receiver=Receiver('/dev/ttyUSB0',uart_open_authorized=True);start=time.monotonic()
                while time.monotonic()-start<seconds:
                    if select.select([receiver.fd],[],[],.002)[0]:
                        b=receiver.read();t=time.monotonic()-start
                        if not b:raise OSError('serial EOF/read-ready with zero bytes')
                        f.write(json.dumps(dict(t_sec=t,bytes=b.hex()))+'\n');quality.feed(b,t)
                    t=time.monotonic()-start;level,reason=quality.check(t)
                    if level=='COMM_HARD_ABORT':break
                summary=quality.summary(t)
                # No freshness checks after close; they would measure teardown delay.
        except BaseException as exc:
            io_error=str(exc);summary=quality.summary(time.monotonic()-start)
            summary.update(status='COMM_HARD_ABORT',reason='serial/tool exception: '+io_error)
        finally:
            if receiver:receiver.close()
        if summary['status'] not in ('COMM_HARD_ABORT','COMM_SOFT_FAIL'):
            rates=[summary['streams'].get(hex(c),{}).get('rate_hz') for c in (0x20,0x21,0x22,0x24)]
            if any(x is None or not 180<=x<=220 for x in rates):summary.update(status='COMM_SOFT_FAIL',reason='preflight rates outside historical band')
            if 0x24 not in quality.latest or any(x==0 for x in quality.latest[0x24][:6]):summary.update(status='COMM_SOFT_FAIL',reason='physical-axis telemetry incomplete')
        summary.update(serial_open_mode='O_RDONLY',serial_write_count=0,attempt=attempt)
        (out/f'preflight{attempt:02}.summary.json').write_text(json.dumps(summary,indent=2)+'\n');attempts.append(summary)
        print(json.dumps({k:summary[k] for k in ('attempt','status','reason','duration_s','errors','max_critical_age_s','q')},indent=2),flush=True)
        if summary['status']!='COMM_SOFT_FAIL':break
        if attempt<3:time.sleep(.3)
    (out/'preflight_summary.json').write_text(json.dumps(attempts,indent=2)+'\n')
    return attempts


def main():
    ap=argparse.ArgumentParser();sub=ap.add_subparsers(dest='mode',required=True)
    b=sub.add_parser('baseline');b.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('preflight');p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.mode=='baseline':
        s=baseline(ROOT/'docs/real_arm_audit_20260917');a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(s,indent=2)+'\n');print(json.dumps(s['critical_gap_stats'],indent=2));print(s['proposed_critical_age_limit']);return
    b=json.loads(a.baseline.read_text())
    if not b['acceptable_for_live']:raise ValueError('historical latency cannot meet30ms limit')
    preflight(a.output,b['proposed_critical_age_limit'])


if __name__=='__main__':main()
