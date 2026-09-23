"""Offline stage 2: compare project parser against independently generated evidence.
No Receiver construction, ROS node, serial open, or publisher is used.
"""
import copy
import json
import math
from pathlib import Path
import struct
from types import SimpleNamespace
from unittest.mock import patch

from real_arm_readonly.protocol import Decoder, JointMapping, NAMES

ROOT=Path('docs/real_arm_audit_20260917')
OUT=ROOT/'j6_software'
TARGETS=(0x20,0x21,0x22,0x24)


def make_frame(cmd, values):
    payload=struct.pack('<7H' if cmd==0x24 else '<7f',*values)
    body=b'\xfe\x55'+bytes((cmd,len(payload)))+payload
    checksum=sum(body)%256
    additional=sum(sum(body[:i+1]) for i in range(len(body)))%256
    return body+bytes((checksum,additional))


def crosscheck(chunks, independent):
    parser=Decoder(); raw=[]; physical=[]
    for b in chunks:
        physical.extend((c,tuple(v)) for c,v in parser.feed(b) if c in TARGETS)
        raw.extend((c,tuple(v)) for c,v in parser.raw_frames if c in TARGETS)
    expected=[(r['cmd'],tuple(r['values'])) for r in independent]
    assert raw==expected
    assert physical==[(c,v[:6]) for c,v in expected]
    assert parser.bad_checksum==parser.bad_length==parser.nonfinite==0
    return len(raw)


def main():
    blob=(ROOT/'recapture/capture.rx.bin').read_bytes()
    independent=json.loads((OUT/'independent_frames.json').read_text())
    results={}
    for size in (1,7,34,4096,len(blob)):
        results[f'chunk_size_{size}']=crosscheck((blob[i:i+size] for i in range(0,len(blob),size)),independent)
    chunks=[bytes.fromhex(json.loads(l)['bytes']) for l in (ROOT/'recapture/capture.chunks.jsonl').read_text().splitlines()]
    assert b''.join(chunks)==blob
    results['original_read_chunks']=crosscheck(chunks,independent)
    sentinel=[]
    fv=(1.125,2.25,3.375,4.5,5.625,6.75,7.875)
    for cmd in TARGETS:
        values=(21,22,23,24,25,26,27) if cmd==0x24 else fv
        wire=make_frame(cmd,values);d=Decoder();physical=d.feed(wire)
        assert d.raw_frames==[(cmd,values)]
        assert len(set(d.raw_frames[0][1]))==7
        assert physical==[(cmd,values[:6])]
        assert physical[0][1][5]==values[5]
        item={'cmd':hex(cmd),'frame_hex':wire.hex(' '),'raw_seven':values,'physical_six':physical[0][1]}
        if cmd==0x20:
            mapped=JointMapping().position(physical[0][1]); expected=math.remainder(-6.75,2*math.pi)
            assert mapped[5]==expected and mapped[5]!=0
            # Change only slot6: joint6 must be unchanged.
            d2=Decoder();p2=d2.feed(make_frame(cmd,(*fv[:6],-123.5)))
            assert JointMapping().position(p2[0][1])[5]==expected
            item['joint6_position']=mapped[5]
        elif cmd==0x21:
            mapped=JointMapping.velocity(physical[0][1]);assert mapped[5]==-6.75
            item['joint6_velocity']=mapped[5]
        elif cmd==0x22:
            item['joint6_raw_current']=physical[0][1][5]
            # C++ feedback sign convention; Python ROS deliberately omits effort.
            item['cpp_expected_signed_current']=-physical[0][1][5]
        else:
            assert physical[0][1][5]==26
            item['joint6_temperature']=physical[0][1][5]
        sentinel.append(item)
    # Exercise actual FeedbackNode.poll without constructing Node or Receiver.
    from real_arm_readonly.feedback_node import FeedbackNode
    from builtin_interfaces.msg import Time
    sample=make_frame(0x21,fv)+make_frame(0x22,fv)+make_frame(0x24,(21,22,23,24,25,26,27))+make_frame(0x20,fv)
    parts=[sample,b''];messages=[]
    harness=SimpleNamespace(fault=None,receiver=SimpleNamespace(read=lambda:parts.pop(0)),
        decoder=Decoder(),mapping=JointMapping(),last_position=None,last_velocity=None,
        velocity=[],timeout=.25,count=0,
        pub=SimpleNamespace(publish=lambda m:messages.append(copy.deepcopy(m))),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time())))
    with patch('os.open',side_effect=AssertionError('No fd open allowed in offline sentinel')):
        FeedbackNode.poll(harness)
    assert len(messages)==1
    msg=messages[0];assert list(msg.name)==list(NAMES)
    assert msg.position[5]==math.remainder(-6.75,2*math.pi)
    assert msg.velocity[5]==-6.75 and len(msg.position)==len(msg.name)==6
    assert len(msg.effort)==0
    zero=JointMapping().position([0.0]*6);assert zero==[0.0]*6
    report={'comparison_frames_per_run':results,'all_equal':True,'sentinels':sentinel,
        'actual_feedback_poll_offline':{'names':list(msg.name),'position':list(msg.position),'velocity':list(msg.velocity),'effort':list(msg.effort),'ros_node_created':False,'serial_opened':False,'real_publisher_created':False},'zero_fresh_mapping':zero}
    (OUT/'crosscheck_sentinel.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
