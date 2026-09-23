"""Offline seven-slot analysis; never imports serial/ROS or opens hardware."""
import argparse
import csv
import json
from pathlib import Path
import struct

import numpy as np


def decode_all(blob):
    rows=[]; i=0
    while i+6 <= len(blob):
        if blob[i:i+2] != b'\xfe\x55': i+=1; continue
        cmd,n=blob[i+2:i+4]; size=n+6
        if i+size>len(blob): break
        f=blob[i:i+size]; total=acc=0
        for b in f[:-2]: total=(total+b)&255; acc=(acc+total)&255
        if f[-2:]!=bytes((total,acc)): i+=1; continue
        if cmd in (32,33,34,35) and n==28:
            rows.append((i,cmd,struct.unpack('<7f',f[4:-2])))
        elif cmd==36 and n==14:
            rows.append((i,cmd,struct.unpack('<7H',f[4:-2])))
        i+=size
    return rows


def runs(mask):
    starts=np.flatnonzero(mask & ~np.r_[False,mask[:-1]])
    ends=np.flatnonzero(mask & ~np.r_[mask[1:],False])
    return list(zip(starts.tolist(),ends.tolist()))


def corr(a,b):
    if len(a)<3 or np.std(a)<1e-12 or np.std(b)<1e-12: return None
    return float(np.corrcoef(a,b)[0,1])


def analyze(capture, metadata, output):
    rows=decode_all(Path(capture).read_bytes()); output=Path(output); output.mkdir(parents=True,exist_ok=True)
    duration=json.loads(Path(metadata).read_text())['elapsed_sec']
    arrays={c:np.array([v for _,cmd,v in rows if cmd==c]) for c in (32,33,34,35,36)}
    n=len(arrays[32]); dt=duration/n
    result={'time_basis':'No per-frame timestamps in old capture/strace. Uniform sequence time is approximate; durations and lags are estimates, not measured device time.', 'duration':duration,'estimated_dt':dt,'raw_channels':[], 'signed_joint_velocity_stats':[], 'alignment':[], 'cross_channel_velocity_correlations':[]}
    for c,a in arrays.items():
        with (output/f'raw_0x{c:02x}.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['index','estimated_time_sec']+[f'channel{i}' for i in range(7)])
            for k,v in enumerate(a):w.writerow([k,k*duration/len(a),*v])
    for ch in range(7):
        item={'channel':ch}
        for c,key in ((32,'position'),(33,'velocity'),(34,'current'),(36,'temperature')):
            a=arrays[c][:,ch];item[key]={'min':float(a.min()),'max':float(a.max()),'all_zero':bool(np.all(a==0)),'unique':len(np.unique(a))}
        result['raw_channels'].append(item)
    signs=np.array([1,1,1,-1,-1,-1])
    q=arrays[32][:,:6]*signs;v=arrays[33][:,:6]*signs
    for j in range(6):
        x=v[:,j]; med=np.median(x);unique=np.unique(x); steps=np.diff(unique)
        s={'joint':j+1,'n':len(x),'min':float(x.min()),'max':float(x.max()),'mean':float(x.mean()),'median':float(med),'std_population':float(x.std()),'p95_p99_p999':np.percentile(x,[95,99,99.9]).tolist(),'abs_p95_p99_p999':np.percentile(abs(x),[95,99,99.9]).tolist(),'MAD':float(np.median(abs(x-med))),'unique_values':unique.tolist(),'unique_step_values':np.unique(steps).tolist(),'thresholds':{}}
        for th in (.05,.1,.15):
            rr=runs(abs(x)>th)
            s['thresholds'][str(th)]={'frames':int(np.sum(abs(x)>th)),'estimated_total_sec':float(np.sum(abs(x)>th)*dt),'runs':[{'start':a,'end':b,'frames':b-a+1,'estimated_sec':(b-a+1)*dt,'right_censored':b==len(x)-1} for a,b in rr], 'longest_frames':max([b-a+1 for a,b in rr],default=0)}
        s['autocorrelation_lags_1_to_40']=[corr(x[:-lag],x[lag:]) for lag in range(1,41)]
        if x.std()>1e-12:
            power=abs(np.fft.rfft(x-x.mean()))**2; freq=np.fft.rfftfreq(len(x),dt); inds=np.argsort(power[1:])[-5:]+1
            s['spectral_peaks_hz_power_fraction']=[[float(freq[k]),float(power[k]/power[1:].sum())] for k in inds[::-1]]
        result['signed_joint_velocity_stats'].append(s)
        dq=np.diff(q[:,j])/dt
        # Inter-frame derivative belongs to midpoint; adjacent 0x21 indices bracket it.
        lagged={}
        for lag in range(-10,11):
            idx=np.arange(len(dq)); ix=idx+lag; ok=(ix>=0)&(ix<len(v));lagged[str(lag)]=corr(dq[ok],v[ix[ok],j])
        result['alignment'].append({'joint':j+1,'position_ptp':float(np.ptp(q[:,j])),'dq_min':float(dq.min()),'dq_max':float(dq.max()),'dq_mean':float(dq.mean()),'dq_std':float(dq.std()),'lag_correlations':lagged,'position_displacement':float(q[-1,j]-q[0,j]),'integrated_protocol_velocity_approx':float(np.sum(x)*dt)})
        result['cross_channel_velocity_correlations'].append([corr(x,v[:,k]) for k in range(6)])
    result['frame_order_first_30']=[f'0x{cmd:02x}' for _,cmd,_ in rows[:30]]
    (output/'analysis.json').write_text(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('capture');p.add_argument('metadata');p.add_argument('output');a=p.parse_args();r=analyze(a.capture,a.metadata,a.output)
    print('Seven raw channels:');print(json.dumps(r['raw_channels'],indent=2))
    print('velocity stats summary:')
    for s in r['signed_joint_velocity_stats']:
        print({k:s[k] for k in ('joint','min','max','mean','median','std_population','p95_p99_p999','MAD','unique_step_values')})
        print({k:{kk:vv for kk,vv in val.items() if kk!='runs'} for k,val in s['thresholds'].items()})
    print('alignment:',r['alignment']);print('cross correlations:',r['cross_channel_velocity_correlations'])
