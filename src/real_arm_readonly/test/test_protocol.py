import math
import struct
from unittest.mock import patch

import pytest
from real_arm_readonly.protocol import Decoder, JointMapping
from real_arm_readonly.receiver import Receiver


def frame(cmd, values):
    payload = struct.pack('<7f', *values)
    body = b'\xfe\x55' + bytes((cmd, len(payload))) + payload
    total = accumulated = 0
    for b in body:
        total = (total+b) & 255
        accumulated = (accumulated+total) & 255
    return body + bytes((total, accumulated))


def test_chunk_boundaries_and_resynchronization():
    good = frame(0x20, [1, 2, 3, 4, 5, 6, -6.52])
    bad = bytearray(good)
    bad[9] ^= 1
    data = b'garbage\xfe' + bytes(bad) + b'\xfe\x55\x20\xff' + good
    for chunk_size in range(1, len(data)+1):
        decoder = Decoder()
        result = []
        for i in range(0, len(data), chunk_size):
            result.extend(decoder.feed(data[i:i+chunk_size]))
        assert result == [(0x20, (1, 2, 3, 4, 5, 6))]
        assert decoder.bad_checksum >= 1
        assert decoder.bad_length == 1
        assert decoder.frame_counts[0x20] == 1
        assert decoder.frame_lengths[(0x20, 28)] == 1
        assert len(decoder.buffer) < 2


def test_bad_data_and_nonphysical_channel():
    decoder = Decoder()
    assert decoder.feed(frame(0x20, [math.nan]*7)) == []
    assert decoder.nonfinite == 1
    assert decoder.feed(frame(0x20, [0]*6+[math.nan])) == [(0x20, (0,)*6)]
    assert decoder.feed(b'\xfe\xaa'+frame(0x20, [0]*7)[2:]) == []


def test_signs_wrap_and_no_offsets():
    mapping = JointMapping()
    q = mapping.position([0.1, 0.2, 0.3, -0.4, -0.5, -3.13])
    assert q == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 3.13])
    assert mapping.position([0, 0, 0, 0, 0, 3.13])[5] == pytest.approx(2*math.pi-3.13)
    assert mapping.velocity([1]*6) == [1, 1, 1, -1, -1, -1]


def test_no_device_open_without_physical_confirmation():
    with patch('os.open', side_effect=AssertionError('must not open device')):
        with pytest.raises(RuntimeError, match='Refusing'):
            Receiver('/dev/ttyUSB0')
        with pytest.raises(ValueError):
            Receiver('/tmp/not-a-device', rx_isolated=True)


def test_readonly_fd_rejects_writes_and_reads_binary_on_pty():
    # This is a local pseudo-terminal only, never /dev/ttyUSB*.
    import errno
    import fcntl
    import os
    import pty
    master, slave = pty.openpty()
    read_fd = os.open(os.ttyname(slave), os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
    receiver = None
    try:
        with patch('os.open', return_value=read_fd) as opened:
            receiver = Receiver('/dev/ttyUSB_FAKE_OFFLINE', rx_isolated=True)
            flags = opened.call_args.args[1]
            assert flags & os.O_ACCMODE == os.O_RDONLY
        assert fcntl.fcntl(receiver.fd, fcntl.F_GETFL) & os.O_ACCMODE == os.O_RDONLY
        with pytest.raises(OSError) as exc:
            os.write(receiver.fd, b'not allowed')
        assert exc.value.errno == errno.EBADF
        payload = frame(0x20, [0, 1, 2, 3, 4, 5, 6])
        os.write(master, payload)
        import select
        assert select.select([receiver.fd], [], [], 1)[0]
        assert receiver.read() == payload
        assert not select.select([master], [], [], 0.05)[0], 'Unexpected echo/output'
    finally:
        if receiver:
            receiver.close()
        else:
            os.close(read_fd)
        os.close(master)
        os.close(slave)


def test_explicit_uart_authorization_is_not_claimed_as_rx_isolation():
    with patch('os.open', side_effect=PermissionError('offline sentinel')) as opened:
        with pytest.raises(PermissionError, match='offline sentinel'):
            Receiver('/dev/ttyUSB0', uart_open_authorized=True)
        assert opened.call_count == 1


def test_all_seven_wire_slots_are_retained_for_audit():
    decoder = Decoder()
    decoded = decoder.feed(frame(0x20, [1, 2, 3, 4, 5, 0, -6.5]))
    assert decoded == [(0x20, (1, 2, 3, 4, 5, 0))]
    assert decoder.raw_frames == [(0x20, (1, 2, 3, 4, 5, 0, -6.5))]
