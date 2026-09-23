"""Offline-only analysis regression tests; no control program import."""
import importlib.util
import json
from pathlib import Path
import struct
import unittest

ROOT = Path(__file__).resolve().parents[4]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


A = module('analysis', ROOT/'src/arm-platform/tools/analyze_gravity_bringup.py')
P = module('readonly_protocol', ROOT/'src/real_arm_readonly/real_arm_readonly/protocol.py')
BASE = ROOT/'docs/real_arm_audit_20260917/gravity_bringup/adaptive_20260917'


class AnalysisTests(unittest.TestCase):
    def frame(self):
        data = b'\xfe\x55\x20\x1c'+struct.pack('<7f',1,2,3,4,5,6,7)
        return data+A.checksum(data)

    def test_chunk_boundaries(self):
        frame = self.frame()
        rows = [dict(kind='RX',t=i/1000,hex=bytes([byte]).hex()) for i,byte in enumerate(frame)]
        parsed, errors, tail = A.decode_rx(rows)
        self.assertEqual(parsed[0]['values'], (1,2,3,4,5,6,7))
        self.assertEqual(errors['checksum'],0)
        self.assertEqual(tail,'')

    def test_incomplete_tail_not_telemetry(self):
        parsed, errors, tail = A.decode_rx([dict(kind='RX',t=0,hex=self.frame()[:-1].hex())])
        self.assertEqual(parsed,[])
        self.assertTrue(tail)
        self.assertEqual(errors['malformed'],0)

    def test_checksum_error(self):
        frame=bytearray(self.frame());frame[-1]^=1
        parsed, errors, _=A.decode_rx([dict(kind='RX',t=0,hex=frame.hex())])
        self.assertEqual(parsed,[])
        self.assertEqual(errors['checksum'],1)

    def test_saved_captures_match_audited_decoder(self):
        for run in ('run01','run02'):
            rows=[json.loads(line) for line in (BASE/(run+'.jsonl')).read_text().splitlines()]
            frames, errors, tail=A.decode_rx(rows)
            audited=P.Decoder();expected=[]
            for row in rows:
                if row['kind']=='RX':
                    audited.feed(bytes.fromhex(row['hex']))
                    expected.extend(audited.raw_frames)
            self.assertEqual([(f['cmd'],f['values']) for f in frames if f['values'] is not None], expected)
            self.assertEqual(bytes.fromhex(tail),bytes(audited.buffer))
            self.assertEqual(errors['checksum'],audited.bad_checksum)
            self.assertEqual(errors['malformed'],audited.bad_length)

    def test_run01_not_treated_as_active(self):
        plan=json.loads((BASE/'plan_v2_logging_only.json').read_text())
        result,history=A.analyze(BASE/'run01.jsonl',plan)
        self.assertEqual(result['tx']['bytes'],0)
        self.assertIsNone(result['joint_metrics'])
        self.assertEqual(history,[])

    def test_run02_includes_abort_sample(self):
        plan=json.loads((BASE/'plan_v2_logging_only.json').read_text())
        result,history=A.analyze(BASE/'run02.jsonl',plan)
        self.assertEqual(result['tx']['batches'],6)
        self.assertEqual(result['tx']['bytes'],1020)
        self.assertEqual(result['joint_metrics'][4]['max_abs_displacement'],.007671356201171875)
        self.assertGreater(result['guard_reconstruction']['utilization'][4],1)
        self.assertIsNone(result['transients']['100'])
        self.assertEqual(history[-1]['cmd'],0x20)

    def test_changed_command_rejected(self):
        snapshot=json.loads((BASE/'run02.jsonl.pre_send.json').read_text())
        packet=bytearray.fromhex(snapshot['packet_hex']);packet[4]^=1
        with self.assertRaises(AssertionError):
            A.check_tx([dict(kind='TX_KERNEL_ACCEPTED',t=0,accepted_bytes=170,hex=packet.hex())],snapshot)


if __name__=='__main__':
    unittest.main()
