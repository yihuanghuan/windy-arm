"""Independent OFFLINE FE55 audit. Python standard library only; no project imports."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import random
import struct

TARGETS = {0x20:28, 0x21:28, 0x22:28, 0x24:14}


def audit(path, out):
    blob=path.read_bytes()
    cursor=0; valid_index=0; bad_checksum=0; bad_length=0; skipped=0
    counts=Counter(); records=[]; hist={c:[Counter() for _ in range(7)] for c in TARGETS}
    values={c:[[] for _ in range(7)] for c in TARGETS}
    while cursor < len(blob):
        if blob[cursor:cursor+2] != b'\xfe\x55':
            cursor+=1;skipped+=1;continue
        if cursor+4>len(blob):break
        command=blob[cursor+2];length=blob[cursor+3]
        stop=cursor+length+6
        if stop>len(blob):break
        wire=blob[cursor:stop]
        sum_byte=0;add_byte=0
        for byte in wire[:-2]:
            sum_byte=(sum_byte+byte)%256
            add_byte=(add_byte+sum_byte)%256
        if wire[-2] != sum_byte or wire[-1] != add_byte:
            bad_checksum+=1;cursor+=1;continue
        valid_index+=1;counts[command]+=1
        if command in TARGETS:
            if length != TARGETS[command]:
                bad_length+=1
            else:
                payload=wire[4:-2]
                if command==0x24:
                    decoded=struct.unpack('<7H',payload)
                    width=2
                else:
                    decoded=struct.unpack('<7f',payload)
                    width=4
                slots=[payload[width*j:width*(j+1)].hex(' ') for j in range(7)]
                for j in range(7):hist[command][j][slots[j]]+=1;values[command][j].append(decoded[j])
                records.append(dict(frame_number=valid_index,byte_offset=cursor,cmd=command,length=length,
                    frame_hex=wire.hex(' '),payload_hex=payload.hex(' '),slot_bytes=slots,values=decoded,
                    checksum_hex=wire[-2:].hex(' ')))
        cursor=stop
    result=dict(input=str(path),sha256=hashlib.sha256(blob).hexdigest(),byte_count=len(blob),
        valid_frames=valid_index,counts={hex(c):n for c,n in counts.items()},bad_checksum=bad_checksum,
        malformed_target_length=bad_length,skipped_bytes=skipped,trailing_bytes=len(blob)-cursor,slots={})
    for c in TARGETS:
        result['slots'][hex(c)]=[dict(slot=j,n=len(values[c][j]),min=min(values[c][j]),max=max(values[c][j]),
            unique_raw_bytes=dict(hist[c][j]),all_bytes_zero=all(set(bytes.fromhex(h))<={0} for h in hist[c][j])) for j in range(7)]
    out.mkdir(parents=True,exist_ok=True)
    (out/'independent_summary.json').write_text(json.dumps(result,indent=2))
    (out/'independent_frames.json').write_text(json.dumps(records))
    # Deterministic stratified random sampling: 20 separated byte-offset bands.
    rng=random.Random(20260917);selected=[]
    for k in range(20):
        cmd=(0x20,0x21,0x22,0x24)[k%4]
        candidates=[r for r in records if r['cmd']==cmd and len(blob)*k/20<=r['byte_offset']<len(blob)*(k+1)/20]
        selected.append(rng.choice(candidates))
    text=[]
    for r in selected:
        text.append(f"frame #{r['frame_number']} offset={r['byte_offset']} cmd=0x{r['cmd']:02x} len={r['length']}\nframe: {r['frame_hex']}\npayload: {r['payload_hex']}\n"+'\n'.join(f"slot{j}: bytes={r['slot_bytes'][j]} value={r['values'][j]!r}" for j in range(7)))
    dumps='\n\n'.join(text)+'\n'
    (out/'20_frame_hex_dumps.txt').write_text(dumps)
    print(json.dumps(result,indent=2));print(dumps)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('output',type=Path);a=p.parse_args();audit(a.capture,a.output)
