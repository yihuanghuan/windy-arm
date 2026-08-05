from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np
import yaml

HEAD = b"\xfe\x55"
CMD_POSITION = 0x20
N_CHANNEL = 7


def _forbid_writes(port):

    def _blocked(*_args, **_kwargs):
        raise RuntimeError(
            "pose_check_readonly 试图向串口发送数据。这个脚本是只读的，"
            "出现这个异常说明代码路径有误，请修复而不是放宽限制。")

    for name in ("write", "flush", "flushOutput", "reset_output_buffer",
                 "send_break", "sendBreak"):
        if hasattr(port, name):
            setattr(port, name, _blocked)
    return port


def load_sign_convention(motor_config: str, arm_config: str, arm_name: str,
                         n_arm: int) -> np.ndarray:
    motors = yaml.safe_load(open(motor_config, encoding="utf-8"))["motor_models"]
    arms = yaml.safe_load(open(arm_config, encoding="utf-8"))["arms"]
    if arm_name not in arms:
        raise KeyError(f"{arm_config} 里没有 arms.{arm_name}")
    joints = arms[arm_name]["joints"]
    if len(joints) < n_arm:
        raise ValueError(
            f"arms.{arm_name} 只有 {len(joints)} 个关节，少于 n_arm={n_arm}")
    signs = np.ones(n_arm)
    for i in range(n_arm):
        model = joints[i]["model"]
        if model not in motors:
            raise KeyError(f"{motor_config} 里没有 motor_models.{model}")

        if motors[model].get("coord_system") != "right_hand":
            signs[i] = -1.0
    return signs


def _checksum_ok(frame: bytes) -> bool:
    body = frame[:-2]
    s = 0
    a = 0
    for byte in body:
        s = (s + byte) & 0xFF
        a = (a + s) & 0xFF
    return frame[-2] == s and frame[-1] == a


def parse_position_frames(buf: bytes):
    out = []
    total = 0
    bad = 0
    i = 0
    while True:
        i = buf.find(HEAD, i)
        if i < 0 or i + 4 > len(buf):
            break
        cmd = buf[i + 2]
        length = buf[i + 3]
        end = i + 4 + length + 2
        if end > len(buf):
            break
        frame = buf[i:end]
        total += 1
        if not _checksum_ok(frame):
            bad += 1
            i += 2
            continue
        if cmd == CMD_POSITION and length == 4 * N_CHANNEL:
            out.append(np.array(
                struct.unpack("<%df" % N_CHANNEL, frame[4:4 + length])))
        i = end
    return out, dict(frames=total, checksum_errors=bad)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[1]
                    / "config" / "smc_params_hwtest.yaml"))
    _pkg = Path(__file__).resolve().parents[2]
    ap.add_argument("--motor-config",
                    default=str(_pkg / "config" / "motor_config.yaml"))
    ap.add_argument("--arm-config",
                    default=str(_pkg / "config" / "arm_config.yaml"))
    ap.add_argument("--arm-name", default="a_l1_gamma6",
                    help="arm_config.yaml 里的构型名，决定各关节旋向")
    ap.add_argument("--offline-selftest", action="store_true",
                    help="不开串口，用合成帧自检解析逻辑")
    args = ap.parse_args(argv)

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))
    want = np.asarray(cfg["initial_pose_rad"], dtype=float)
    n_arm = want.size

    if args.offline_selftest:
        return _selftest(n_arm)

    signs = load_sign_convention(args.motor_config, args.arm_config,
                                 args.arm_name, n_arm)

    try:
        import serial
    except ImportError:
        print("需要 pyserial：pip install pyserial", file=sys.stderr)
        return 2

    print(f"只读打开 {args.port} @ {args.baud}，采集 {args.seconds:.1f}s"
          f"（不会发送任何字节）")
    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        _forbid_writes(port)
        port.reset_input_buffer()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            chunk = port.read(4096)
            if chunk:
                buf.extend(chunk)
        elapsed = time.time() - t0

    frames, stats = parse_position_frames(bytes(buf))
    if not frames:
        print(f"没有解析到 0x20 位置帧（收到 {len(buf)} 字节，"
              f"帧 {stats['frames']}，校验错 {stats['checksum_errors']}）")
        print("检查：实物是否上电？端口是否正确？是否被其它进程占用？")
        return 1

    q = np.array(frames)
    rate = len(frames) / max(elapsed, 1e-9)

    raw_median = np.median(q[:, :n_arm], axis=0)
    actual = raw_median * signs
    jitter = np.percentile(q[:, :n_arm], 97.5, axis=0) - np.percentile(
        q[:, :n_arm], 2.5, axis=0)
    diff = actual - want

    print(f"\n采集 {len(frames)} 帧 / {elapsed:.2f}s = {rate:.1f} Hz"
          f"（固件标称 200.6）  校验错 {stats['checksum_errors']}")
    flipped = [f"j{i+1}" for i in range(n_arm) if signs[i] < 0]
    print(f"旋向（取自 {args.arm_name}）：取反的关节 "
          f"{flipped if flipped else '无'}"
          f" —— 下表【实物】列已转换到控制器约定，可直接写进 initial_pose_rad")
    print(f"\n{'关节':>5} {'实物(rad)':>12} {'配置(rad)':>12} "
          f"{'差(rad)':>10} {'差(deg)':>10} {'抖动(rad)':>11}")
    for j in range(n_arm):
        flag = "  <<<" if abs(diff[j]) > 0.05 else ""
        print(f"j{j+1:<4} {actual[j]:12.4f} {want[j]:12.4f} "
              f"{diff[j]:10.4f} {np.degrees(diff[j]):10.2f} "
              f"{jitter[j]:11.5f}{flag}")

    worst = int(np.argmax(np.abs(diff)))
    print(f"\n最大偏差 j{worst+1}: {diff[worst]:+.4f} rad "
          f"= {np.degrees(diff[worst]):+.2f}°")
    if N_CHANNEL > n_arm:
        ghost = np.median(q[:, n_arm:], axis=0)
        print(f"通道 {n_arm+1}（幽灵数据，实物无此关节）: "
              f"{np.round(ghost, 4).tolist()}")


    if np.max(np.abs(diff)) > 0.05:
        print("\n⚠️ 偏差超过 0.05 rad。直接启动会让第一条指令命令一次跳跃。")
        print("   处理方式（二选一）：")
        print("   a) 把上面的【实物】列写进 config 的 initial_pose_rad；")
        print("   b) 用 approach 段从实物位形平滑过渡到轨迹起点。")
        print(f"\n   可直接粘贴：initial_pose_rad: "
              f"[{', '.join('%.4f' % v for v in actual)}]")
        return 3
    print("\n✅ 偏差在摩擦死区量级内，可以直接起步。")
    return 0


def _selftest(n_arm):
    truth = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, -6.5231][:N_CHANNEL])
    payload = struct.pack("<%df" % N_CHANNEL, *truth)
    body = HEAD + bytes([CMD_POSITION, len(payload)]) + payload
    s = 0
    a = 0
    for byte in body:
        s = (s + byte) & 0xFF
        a = (a + s) & 0xFF
    good = body + bytes([s, a])
    corrupt = bytearray(good)
    corrupt[6] ^= 0xFF

    frames, stats = parse_position_frames(
        b"\x00\x11" + good * 3 + bytes(corrupt) + good)
    assert stats["frames"] == 5, stats
    assert stats["checksum_errors"] == 1, stats
    assert len(frames) == 4, len(frames)
    for f in frames:
        assert np.allclose(f, truth, atol=1e-6), f

    class _Fake:
        def write(self, *_):
            return 0

    blocked = _forbid_writes(_Fake())
    try:
        blocked.write(b"x")
    except RuntimeError:
        pass
    else:
        raise AssertionError("发送封锁失效")

    _selftest_signs(n_arm)

    print("离线自检通过：帧解析 4/5（1 帧校验错被正确拒绝）、"
          "幽灵通道保留、发送封锁生效、旋向约定正确")
    return 0


def _selftest_signs(n_arm):
    pkg = Path(__file__).resolve().parents[2]
    signs = load_sign_convention(
        str(pkg / "config" / "motor_config.yaml"),
        str(pkg / "config" / "arm_config.yaml"), "a_l1_gamma6", n_arm)

    expected = np.array([1.0, 1.0, 1.0, -1.0, -1.0, -1.0])[:n_arm]
    assert np.array_equal(signs, expected), (
        f"旋向与实测不符：得到 {signs.tolist()}，实测应为 {expected.tolist()}")

    raw = np.array([0.1111, 0.2222, 0.3333, -1.3467, 0.3654, -0.2626])[:n_arm]
    ros = np.array([0.1111, 0.2222, 0.3333, 1.3467, -0.3654, 0.2626])[:n_arm]
    assert np.allclose(raw * signs, ros, atol=1e-9), "取反后与实测输出对不上"


if __name__ == "__main__":
    raise SystemExit(main())
