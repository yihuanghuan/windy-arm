"""Bounded, explicitly authorized receive-only hardware audit. Never sends UART bytes."""
import argparse
import json
import math
from pathlib import Path
import select
import time

from .protocol import Decoder, JointMapping, NAMES
from .receiver import Receiver
from .read_guard import ReadGuard


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--uart-open-authorized', action='store_true')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--observe-seconds', type=float, default=16)
    ap.add_argument('--stream-seconds', type=float, default=35)
    ap.add_argument('--report', required=True)
    args = ap.parse_args()
    if not args.uart_open_authorized:
        ap.error('Explicit UART open/raw/921600 authorization is required')
    if not (10 <= args.observe_seconds <= 60 and 1 <= args.stream_seconds <= 120):
        ap.error('Observation must be 10..60s; streaming 1..120s')
    report_path = Path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    decoder, mapping = Decoder(), JointMapping()
    guard = ReadGuard()
    raw_ranges = {}
    cmd_last_time = {}
    cmd_gaps = {}
    channel5_nonzero = False
    timed_capture = None
    started = time.monotonic()
    first_q = last_q = last_v = None
    q_min, q_max = [math.inf]*6, [-math.inf]*6
    max_delta = [0.0]*6
    last_position_time = last_velocity_time = None
    max_gap = 0.0
    stale_intervals = []
    latest = {}
    position_count = published = 0
    serial_opened = False
    observation = None
    node = prepared_node = receiver = None
    reason = 'completed'
    bytes_received = 0
    publish_start = None
    capture = None

    def snapshot(now):
        elapsed = now-started
        counts = {f'0x{k:02x}': v for k,v in sorted(decoder.frame_counts.items())}
        return dict(elapsed_sec=elapsed, serial_opened=serial_opened,
                    bytes_received=bytes_received, counts=counts,
                    rates_hz={k:v/elapsed for k,v in counts.items()},
                    payload_lengths={f'0x{k:02x}/{n}':v for (k,n),v in decoder.frame_lengths.items()},
                    checksum_errors=decoder.bad_checksum, malformed_frames=decoder.bad_length,
                    nonfinite_frames=decoder.nonfinite, discarded_bytes=decoder.discarded_bytes,
                    pending_bytes=len(decoder.buffer), position_frames=position_count,
                    raw_7_channel_ranges=raw_ranges, warnings_by_joint=guard.warnings,
                    cmd_gap_sec={k:dict(n=len(v),min=min(v) if v else None,max=max(v) if v else None,mean=sum(v)/len(v) if v else None) for k,v in cmd_gaps.items()},
                    cmd_age_sec={k:now-t for k,t in cmd_last_time.items()},
                    channel5_nonzero_observed=channel5_nonzero,
                    first_position=first_q, current_position=last_q, current_velocity=last_v,
                    position_min=q_min if position_count else None,
                    position_max=q_max if position_count else None,
                    max_position_step=max_delta, max_position_gap_sec=max_gap,
                    stale_intervals=stale_intervals, latest_feedback=latest,
                    published=published, publish_elapsed_sec=None if publish_start is None else now-publish_start,
                    units='position rad, velocity rad/s; 0x22 units unconfirmed',
                    coordinates='gamma6 signed encoder coordinates; no historical zero offsets',
                    wire_timestamps='host read time; not firmware acquisition time')
    try:
        # DDS initialization can block longer than the receive stale threshold.
        # Prepare the feedback-only publisher BEFORE opening the serial port.
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        rclpy.init(args=[])
        prepared_node = Node('real_arm_readonly_live_audit', start_parameter_services=False)
        publisher = prepared_node.create_publisher(JointState, '/real_arm/raw_joint_states', qos_profile_sensor_data)
        capture = report_path.with_suffix('.rx.bin').open('wb')
        timed_capture = report_path.with_suffix('.chunks.jsonl').open('w')
        receiver = Receiver(args.port, uart_open_authorized=True)
        serial_opened = True
        started = time.monotonic()
        print('UART opened O_RDONLY, 921600/raw. Observation only; no ROS position publication yet.', flush=True)
        deadline = started+args.observe_seconds+args.stream_seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            if last_position_time is None and now-started > 2:
                reason = 'STOP: no spontaneous position feedback within 2s; no requests sent'
                break
            if last_position_time is not None and now-last_position_time > 0.25:
                stale_intervals.append(dict(start_sec=last_position_time-started, observed_gap_sec=now-last_position_time))
                reason = 'STOP: position stale >0.25s; possible link/board interruption'
                break
            if not select.select([receiver.fd], [], [], 0.01)[0]:
                continue
            chunk = receiver.read()
            if not chunk:
                reason = 'STOP: tty readable but empty; possible disconnect'
                break
            now = time.monotonic()
            capture.write(chunk)  # local evidence file, never the serial fd
            bytes_received += len(chunk)
            timed_capture.write(json.dumps({'t_sec':now-started,'bytes':chunk.hex()})+'\n')
            parsed = decoder.feed(chunk)
            for raw_cmd,raw_values in decoder.raw_frames:
                if not all(math.isfinite(v) for v in raw_values):
                    raise RuntimeError('STOP: nonfinite raw protocol slot')
                key=f'0x{raw_cmd:02x}'
                if key in cmd_last_time:
                    cmd_gaps.setdefault(key,[]).append(now-cmd_last_time[key])
                cmd_last_time[key]=now
                if key not in raw_ranges:
                    raw_ranges[key]={'min':list(raw_values),'max':list(raw_values)}
                else:
                    raw_ranges[key]['min']=[min(a,b) for a,b in zip(raw_ranges[key]['min'],raw_values)]
                    raw_ranges[key]['max']=[max(a,b) for a,b in zip(raw_ranges[key]['max'],raw_values)]
                if raw_cmd in (0x20,0x21,0x22,0x24) and raw_values[5] != 0:
                    channel5_nonzero=True
            for cmd, values in parsed:
                latest[f'0x{cmd:02x}'] = list(values)
                if cmd == 0x21:
                    last_v = mapping.velocity(values)
                    last_velocity_time = now
                    why=guard.update(cmd,last_v,now)
                    if why: raise RuntimeError('STOP: '+why)
                elif cmd == 0x20:
                    q = mapping.position(values)
                    if first_q is None:
                        first_q = q.copy()
                    if last_q is not None:
                        for i in range(6):
                            max_delta[i] = max(max_delta[i], abs(q[i]-last_q[i]))
                    why=guard.update(cmd,q,now)
                    if why: raise RuntimeError('STOP: '+why)
                    if last_position_time is not None:
                        max_gap = max(max_gap, now-last_position_time)
                    last_position_time = now
                    last_q = q
                    position_count += 1
                    q_min = [min(a,b) for a,b in zip(q_min,q)]
                    q_max = [max(a,b) for a,b in zip(q_max,q)]
                    if node is not None:
                        from sensor_msgs.msg import JointState
                        msg = JointState()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.header.frame_id = 'base_link'
                        msg.name = list(NAMES)
                        msg.position = q
                        if last_velocity_time is not None and now-last_velocity_time <= 0.025:
                            msg.velocity = last_v
                        publisher.publish(msg)
                        published += 1
            if decoder.nonfinite:
                raise RuntimeError('STOP: nonfinite feedback')
            if decoder.bad_checksum or decoder.bad_length:
                raise RuntimeError('STOP: malformed/checksum feedback; inspect captured bytes offline')
            if node is None and now-started >= args.observe_seconds:
                observation = snapshot(now)
                report_path.with_suffix('.observation.json').write_text(json.dumps(observation,indent=2))
                print('OBSERVATION COMPLETE: '+json.dumps({'elapsed_sec':observation['elapsed_sec'],'counts':observation['counts'],'warnings':guard.warnings,'channel5_nonzero':channel5_nonzero}), flush=True)
                if not channel5_nonzero:
                    reason='completed raw observation; ROS withheld: channel5 all-zero telemetry validity unresolved'
                    break
                node = prepared_node
                publish_start = time.monotonic()
                print('LIVE FEEDBACK STARTED; bounded receive-only stream.', flush=True)
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0)
    except (Exception, KeyboardInterrupt) as exc:
        reason = str(exc) or 'STOP: interrupted'
        print(reason, flush=True)
    finally:
        if receiver is not None:
            receiver.close()
        ended = time.monotonic()
        if capture:
            capture.close()
        if timed_capture:
            timed_capture.close()
        result = snapshot(ended)
        result['result'] = reason
        result['observation'] = observation
        report_path.write_text(json.dumps(result,indent=2))
        if prepared_node is not None:
            prepared_node.destroy_node()
            rclpy.shutdown()
        print('UART CLOSED; result='+reason+'; report='+str(report_path), flush=True)
    return 0 if reason.startswith('completed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
