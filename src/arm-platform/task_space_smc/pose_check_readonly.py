#!/usr/bin/env python3
"""上机前的【只读】位形核对：实物当前位形 vs 配置里的 initial_pose_rad。

为什么需要它：仿真从 config 的 initial_pose_rad 起步，如果实物上电后不在
那个位形，控制器的第一条指令就会命令一次大跳跃。之前实测过原配置与实物
相差最大 1.35 rad —— 那种跳跃在实机上是危险的。

【安全性由结构保证，不是靠自觉】：
  * 全程只调用 serial.Serial.read()，从不 write()。
  * 打开串口后立刻把 write / flush 等发送类方法替换成抛异常的桩，
    任何代码路径想发字节都会当场崩掉而不是悄悄发出去。
  * 不 import rclpy，不发布任何话题，不启动任何节点。
  * 机械臂上电后电机是锁死的，本脚本不改变这一点。

协议（见 task_space_smc/README.md 第 10 节，3 秒抓包 2480 帧零校验错误）：
    0xFE 0x55 | cmd | len | payload | sumCheck | addCheck      板 -> PC
    cmd 0x20 = position，7 x float32（第 7 通道是幽灵数据，见 10.4）
    上报速率 200.6 Hz

⚠️【坐标系取反 —— 2026-08-03 修复的一个安全关键缺陷】
串口里的 0x20 是【原始】位置，而 `initial_pose_rad` 用的是控制器约定。
两者对 `coord_system: left_hand` 的关节【差一个负号】：
    DMMotor::UpdateState()  ->  LeftHand 时 position_ = -position_
本机 joint4/5/6 是 RS05，motor_config.yaml 里标着 left_hand，所以这三个
关节的串口读数必须取反才能与配置比较。

本脚本原先直接拿原始值比对，j4/j5/j6 的结论是【反的】。之所以一直没暴露，
是因为此前记录的位形里这三个关节都≈0（0.0006 / -0.0025 / -0.0029），
符号看不出来。2026-08-03 实测 j4 到了 -1.3467 rad，照原样写进配置会
差 154°—— 那是一次剧烈甩动。

取反关系【从配置推导】而不是写死，这样换电机型号时不会失效。
端到端验证：给 student_arm_node 喂已知原始位置，核对 /joint_states 的输出。
"""

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
N_CHANNEL = 7                      # 固件固定按 7 通道上报，实物只有前 6 个


def _forbid_writes(port):
    """把所有发送类方法换成抛异常的桩 —— 只读是结构保证，不是约定。"""

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
    """从配置推导每个关节的取反系数（+1 右手 / -1 左手）。

    链路与 C++ 侧完全一致：
        arm_config.yaml  的 arms.<arm_name>.joints[i].model
        -> motor_config.yaml 的 motor_models.<model>.coord_system
        -> a_l1.cc:37     "right_hand" ? RightHand : LeftHand
        -> dm_motor.cc:40 LeftHand 时 position/velocity/torque 全部取反

    注意 arm_config.yaml 的 joints[i] 里也有个 coord_system 字段，
    但 LoadArmConfig 不解析它 —— 以 motor_config.yaml 的为准。
    """
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
        # 与 a_l1.cc 一致：只有精确等于 right_hand 才是右手，其余一律左手
        if motors[model].get("coord_system") != "right_hand":
            signs[i] = -1.0
    return signs


def _checksum_ok(frame: bytes) -> bool:
    """sumCheck = 累加和，addCheck = 累加和的累加和，都取低 8 位。"""
    body = frame[:-2]
    s = 0
    a = 0
    for byte in body:
        s = (s + byte) & 0xFF
        a = (a + s) & 0xFF
    return frame[-2] == s and frame[-1] == a


def parse_position_frames(buf: bytes):
    """从字节流里抽出所有校验通过的 0x20 位置帧。返回 (positions, stats)。"""
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
        default=str(Path(__file__).resolve().parent
                    / "config" / "smc_params_hwtest.yaml"))
    _pkg = Path(__file__).resolve().parent.parent
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
    # 【必须先取反再比较】—— 串口是原始值，initial_pose_rad 是控制器约定
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

    # 判据：0.05 rad ≈ 2.9°。实测各关节摩擦死区 0.020~0.059 rad，
    # 小于死区的偏差本来就无法通过位置指令消除，没必要报警。
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
    """用合成帧验证解析与校验逻辑 —— 无硬件时也能确认脚本是对的。"""
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
    corrupt[6] ^= 0xFF                      # 破坏 payload，校验应当拒绝

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
    """旋向必须与实测一致 —— 这是 2026-08-03 修掉的安全关键缺陷。

    基准来自端到端实测：给 student_arm_node 喂原始 0x20 位置
        [0.1111, 0.2222, 0.3333, -1.3467, 0.3654, -0.2626]
    /joint_states 发布出来的是
        [0.1111, 0.2222, 0.3333, +1.3467, -0.3654, +0.2626]
    即 j1~j3 不变、j4~j6 取反。
    """
    pkg = Path(__file__).resolve().parent.parent
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
