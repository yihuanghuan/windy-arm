"""Pure bounded tuning policy; no transport or live entry point in this module.

This module is not an authorization to resume after a communication hard abort.
Proxy direction is diagnostic, not a measurement of net generalized force.
"""
from collections import deque
import numpy as np

MAX_ENVELOPE = np.array([.04,.05,.05,.08,.08,.08])
KP_MIN = np.array([6,6,6,1,1,1])
KP_MAX = np.array([30,30,30,5,5,5])
# Phase4 L0..L4 normal compensation range. Excludes Phase2 escape/ramp Kd=10.
KD_MIN = np.array([.2,.2,.2,.02,.02,.02])
KD_MAX = np.array([1,1,1,.1,.1,.1])
VELOCITY_CEILING = np.array([.6,.6,.6,.8,.8,.8])
PROXY_LIMIT = np.array([4.5,4.5,4.5,.8,.8,.8])
POSITION_QUANTUM = .0003833770751953125


def vector(x):
    x=np.asarray(x,dtype=float)
    if x.shape!=(6,) or not np.isfinite(x).all():
        raise ValueError('six finite numbers required')
    return x


def validate_parameters(lam,kp,kd):
    kp,kd=vector(kp),vector(kd)
    if not np.isfinite(lam) or not .4<=lam<=1:
        raise ValueError('lambda outside authorized tuning range')
    if np.any(kp<KP_MIN) or np.any(kp>KP_MAX):raise ValueError('kp range')
    if np.any(kd<KD_MIN) or np.any(kd>KD_MAX):raise ValueError('kd normal historical range')
    return kp,kd


def feasible_candidates(g,envelope,kp,lambdas,previous_lambda=None):
    """Static residual is screening only; retain 50% of 80% abort displacement."""
    g,envelope,kp=vector(g),vector(envelope),vector(kp)
    if np.any(envelope<=0) or np.any(envelope>MAX_ENVELOPE):raise ValueError('envelope')
    if np.any(kp<KP_MIN) or np.any(kp>KP_MAX):raise ValueError('kp range')
    results=[]
    for lam in lambdas:
        if not .4<=lam<=1:raise ValueError('lambda')
        residual=abs((1-lam)*g)/kp
        feasible=bool(np.all(residual<=.4*envelope) and np.all(abs(lam*g)<=PROXY_LIMIT))
        step_ok=previous_lambda is None or abs(lam-previous_lambda)<=.15+1e-12
        results.append(dict(lambda_value=lam,static_error=residual.tolist(),
            feasible=feasible and step_ok,reason='eligible for further review' if feasible and step_ok else 'SKIP static/step/proxy constraint'))
    return results


class Monitor:
    """Proposed guard evaluated once per unique position sample.

    For live use a separate every-loop freshness/TX barrier is still mandatory.
    All duration thresholds use elapsed sample time, never loop iteration counts.
    """
    def __init__(self,q_start,g,lam,kp,kd,envelope,lower,upper):
        self.kp,self.kd=validate_parameters(lam,kp,kd)
        self.q0,self.g=vector(q_start),vector(g)
        self.env=vector(envelope)
        if np.any(self.env<=0) or np.any(self.env>MAX_ENVELOPE):raise ValueError('envelope')
        self.lower,self.upper=vector(lower),vector(upper)
        self.lam=lam;self.ff=lam*self.g
        self.rows=deque();self.mismatch_since=[None]*6;self.last=None

    def update(self,t,q,dq,*,feedback_age,desk_margin,self_margin,
               communication_level='COMM_OK',critical_age_limit=.026,operator_stop=False):
        try:q,dq=vector(q),vector(dq)
        except ValueError:return dict(status='HARD_ABORT',reason='nonfinite/shape')
        scalars=[t,feedback_age,desk_margin,self_margin]
        if not np.isfinite(scalars).all():return dict(status='HARD_ABORT',reason='nonfinite metadata')
        if self.last is not None and t<=self.last:return dict(status='HARD_ABORT',reason='nonmonotonic/duplicate sample')
        if communication_level=='COMM_HARD_ABORT':return dict(status='HARD_ABORT',reason='communication integrity')
        if operator_stop:return dict(status='HARD_ABORT',reason='operator stop')
        if not 0<=feedback_age<critical_age_limit:return dict(status='HARD_ABORT',reason='stale')
        if desk_margin<.05 or self_margin<.003:return dict(status='HARD_ABORT',reason='geometry')
        if np.min(np.minimum(q-self.lower,self.upper-q))<.05:return dict(status='HARD_ABORT',reason='joint limit')
        p=self.kp*(self.q0-q);d=-self.kd*dq;proxy=p+d+self.ff
        delta=q-self.q0
        out=dict(status='NORMAL',reason='within observed guards',t=t,q=q.tolist(),dq=dq.tolist(),
            delta=delta.tolist(),p=p.tolist(),d=d.tolist(),ff=self.ff.tolist(),proxy=proxy.tolist(),
            nominal_proxy_minus_gravity=(proxy-self.g).tolist(),
            displacement_ratio=(abs(delta)/self.env).tolist(),
            raw_velocity_warning=(abs(dq)>VELOCITY_CEILING).tolist(),
            direction_consistency_score=[None]*6,direction_mismatch=[False]*6,
            acceleration_estimate=[None]*6,mismatch_duration_s=[0.]*6)
        def abort(reason):out.update(status='HARD_ABORT',reason=reason);return out
        if np.any(abs(delta)>=.8*self.env):return abort('80 percent displacement envelope')
        if np.any(abs(proxy)>PROXY_LIMIT):return abort('nominal torque proxy')
        if self.last is not None and t-self.last>=critical_age_limit:return abort('position stream stale')
        self.last=t
        self.rows.append(dict(t=t,q=q.copy(),v=dq.copy(),proxy=proxy.copy()))
        while self.rows and t-self.rows[0]['t']>.4:self.rows.popleft()
        rows=list(self.rows)
        def interval(lo,hi):return [r for r in rows if lo-1e-9<=r['t']<=hi+1e-9]
        def rate(rr):
            if len(rr)<4 or rr[-1]['t']-rr[0]['t']<.014:return None
            tt=np.array([r['t'] for r in rr]);qq=np.array([r['q'] for r in rr]);tt-=tt.mean()
            return np.sum(tt[:,None]*(qq-qq.mean(axis=0)),axis=0)/np.sum(tt**2)
        window=interval(t-.04,t)
        # Raw sustained velocity must agree with encoder displacement over >=35ms.
        if len(window)>=7 and window[-1]['t']-window[0]['t']>=.035-1e-9:
            vv=np.array([r['v'] for r in window]);pv=rate(window)
            sustained=np.all(abs(vv)>VELOCITY_CEILING,axis=0)&np.all(vv*vv[-1]>0,axis=0)
            consistent=(pv*vv[-1]>0)&(abs(pv)>.5*VELOCITY_CEILING)&(abs(q-window[0]['q'])>2*POSITION_QUANTUM)
            if np.any(sustained&consistent):return abort('sustained velocity with position evidence')
        earlier=interval(t-.04,t-.02);recent=interval(t-.02,t)
        v0,v1=rate(earlier),rate(recent)
        if v0 is not None and v1 is not None:
            accel=(v1-v0)/.02;out['acceleration_estimate']=accel.tolist()
            # Two encoder quanta over 20ms suppress quantization-only acceleration signs.
            growing=(v1*v0>=0)&(abs(v1)>abs(v0)+2*POSITION_QUANTUM/.02)
            sign_floor=.005
            mismatch=(proxy*v1<0)&(proxy*accel<0)&growing&(abs(proxy)>sign_floor)&(abs(v1)>.03)
            # +1 = proxy and measured acceleration agree, -1 oppose. Not net-force proof.
            score=np.sign(proxy*accel).astype(float);score[abs(proxy)<=sign_floor]=np.nan
            out['direction_consistency_score']=[float(x) if np.isfinite(x) else None for x in score]
            out['direction_mismatch']=mismatch.tolist()
            for j in range(6):
                if mismatch[j]:
                    if self.mismatch_since[j] is None:self.mismatch_since[j]=t
                    duration=t-self.mismatch_since[j];out['mismatch_duration_s'][j]=duration
                    if duration>=.05 and abs(delta[j])>4*POSITION_QUANTUM:
                        return abort('persistent expanding direction mismatch; no retry')
                else:self.mismatch_since[j]=None
        # Three disjoint 30ms position windows: increasing speed or growing reversal.
        blocks=[interval(t-.09,t-.06),interval(t-.06,t-.03),interval(t-.03,t)]
        rates=[rate(b) for b in blocks]
        if all(v is not None for v in rates) and t-rows[0]['t']>=.085:
            a,b,c=rates;noise=2*POSITION_QUANTUM/.03
            increase=(a*b>0)&(b*c>0)&(abs(b)>abs(a)+noise)&(abs(c)>abs(b)+noise)&(abs(c)>.08)
            if np.any(increase):return abort('continued position-derived acceleration')
            amps=[np.ptp(np.array([r['q'] for r in block]),axis=0) for block in blocks]
            oscillation=(a*b<0)&(b*c<0)&(amps[1]>amps[0]+2*POSITION_QUANTUM)&(amps[2]>amps[1]+2*POSITION_QUANTUM)
            if np.any(oscillation):return abort('growing oscillation')
        # Retain nominal proxy slew protection without interpreting an isolated
        # velocity quantization step as the derivative of actual actuator torque.
        older,newer=interval(t-.1,t-.05),interval(t-.05,t)
        if len(older)>=8 and len(newer)>=8:
            dt=np.mean([r['t'] for r in newer])-np.mean([r['t'] for r in older])
            slew=abs(np.mean([r['proxy'] for r in newer],axis=0)-np.mean([r['proxy'] for r in older],axis=0))/dt
            out['mean_proxy_slew']=slew.tolist()
            if np.any(slew>PROXY_LIMIT/.05):return abort('50ms mean nominal proxy slew')
        return out


def classify_completed(rows,observation_s):
    """Not a live timeout/restore handler. No evidence is not a PASS."""
    if any(r['status']=='HARD_ABORT' for r in rows):return 'HARD_ABORT'
    if observation_s<.15 or not rows:return 'SOFT_FAIL'
    if any(any(r.get('direction_mismatch',[])) for r in rows):return 'SOFT_FAIL'
    q=np.array([r['q'] for r in rows[-10:]])
    if np.any(np.ptp(q,axis=0)>4*POSITION_QUANTUM):return 'SOFT_FAIL'
    return 'NORMAL'
