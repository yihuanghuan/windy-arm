"""Receive-only ProtocolV1 decoder. No ROS or device access."""
from collections import Counter
import math
import struct

NAMES = tuple(f'joint{i}' for i in range(1, 7))
SIGNS = (1, 1, 1, -1, -1, -1)
LENGTHS = {0x20: 28, 0x21: 28, 0x22: 28, 0x23: 28, 0x24: 14}


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_checksum = self.bad_length = self.nonfinite = 0
        self.valid_frames = 0
        self.frame_counts = Counter()
        self.frame_lengths = Counter()
        self.discarded_bytes = 0

    def feed(self, chunk):
        self.raw_frames = []
        self.buffer.extend(chunk)
        frames = []
        while True:
            start = self.buffer.find(b'\xfe\x55')
            if start < 0:
                self.discarded_bytes += len(self.buffer) - (1 if self.buffer[-1:] == b'\xfe' else 0)
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\xfe' else b''
                break
            self.discarded_bytes += start
            del self.buffer[:start]
            if len(self.buffer) < 4:
                break
            cmd, length = self.buffer[2:4]
            if cmd in LENGTHS and length != LENGTHS[cmd]:
                self.bad_length += 1
                del self.buffer[0]
                continue
            size = length + 6
            if len(self.buffer) < size:
                break
            frame = self.buffer[:size]
            total = accumulated = 0
            for byte in frame[:-2]:
                total = (total + byte) & 255
                accumulated = (accumulated + total) & 255
            if frame[-2:] != bytes((total, accumulated)):
                self.bad_checksum += 1
                del self.buffer[0]
                continue
            del self.buffer[:size]
            self.valid_frames += 1
            self.frame_counts[cmd] += 1
            self.frame_lengths[(cmd, length)] += 1
            if cmd not in LENGTHS:
                continue
            values = struct.unpack('<7H' if cmd == 0x24 else '<7f', frame[4:-2])
            self.raw_frames.append((cmd, values))
            # Channel 7 is carried by the protocol but is not a physical joint.
            if not all(math.isfinite(v) for v in values[:6]):
                self.nonfinite += 1
                continue
            frames.append((cmd, values[:6]))
        return frames


class JointMapping:
    """Driver sign convention and continuous encoder angles; NO calibration."""
    def __init__(self):
        self.previous = None

    def position(self, values):
        signed = [s * v for s, v in zip(SIGNS, values)]
        if self.previous is None:
            result = [math.remainder(v, 2 * math.pi) for v in signed]
        else:
            result = [p + math.remainder(v - p, 2 * math.pi)
                      for p, v in zip(self.previous, signed)]
        self.previous = result
        return result

    @staticmethod
    def velocity(values):
        return [s * v for s, v in zip(SIGNS, values)]
