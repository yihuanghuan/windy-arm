#!/usr/bin/env python3
"""双向串口抓包：验证【下行】力矩帧是否真的按我们的意图发出去了。

【为什么需要它 —— 见 HANDOFF 10.3】
第 4 次力矩模式上机：approach 段 24 秒里 j3 恒 −9.00 Nm、j4 恒 −1.60 Nm、
标准差 0.00、符号翻转 0 次，六轴饱和占比 76~99.9%，而 q_j2 精确停在 33.124°
纹丝不动。恒定满幅直流力矩 + 零响应，已排除控制律振荡、丢帧、ROS 域冲突。
结论是【力矩没有到达电机】，但下行方向从未被独立验证过。

【为什么不能沿用 pose_check_readonly.py 的被动读法】
UART 不会把 TX 回环到 RX —— 打开同一个 /dev/ttyUSB0 去 read()，永远只能
看到板子发来的上行流，看不到我们自己发出去的字节。何况串口被控制节点
独占，第二个进程连打开都打不开。所以被动读【原理上】就抓不到下行。

【本脚本的做法：伪终端透明中继】
    控制节点  --port_name:=/tmp/.../ttyTAP-->  [本脚本]  --> /dev/ttyUSB0
                     (PTY 从设备)              原样转发         (真串口)
两个方向的字节全部打时间戳落盘，同时在线解码统计。节点看到的是一个普通
串口，代码一行都不用改。

【安全边界 —— 与 pose_check_readonly.py 不同，必须讲清楚】
本脚本【会】向真串口发送字节 —— 这正是它的职责（它是链路的一段）。
但它【只转发，不生成】：所有下行字节都直接来自节点侧 read() 的原始缓冲，
_forward() 是唯一的发送点。回归自检 test_relay_is_byte_exact 用随机字节流
验证「进什么出什么」，禁止任何形式的改写。

【判读方法（这是本工具存在的意义）】
    下行 0x12 缺失 / 帧率低于 0x10 / 校验错 / 被截断
        -> 问题在驱动层（DMMotor::UpdateCommand / ProtocolV1::Pop / SendCore）
    下行 0x12 正确，但上行 0x22 不跟随
        -> 帧到了板子、力矩没到电机 -> 电机固件对 kp=kd=0 的处理
    下行 0x12 正确且上行 0x22 跟随
        -> 力矩确实产生了，机械臂不动是机械/摩擦问题，不是通路问题

    专门盯住的两处代码嫌疑：
      * serial_bus.cc:30  serial_.write() 的【返回值被丢弃】，短写会静默
        截断帧，板子按校验丢掉 —— 这条从没验证过。本脚本统计短写次数。
      * protocol_v1.cc    Pop() 里五个 *_cmd_updated_ 标志各自独立，
        0x12 的帧率若低于 0x10 就直接定位到驱动层。

【协议（见 README 第 10 节）】
    0xFE 0xAA | cmd | len | payload | sumCheck | addCheck    PC  -> 板（下行）
    0xFE 0x55 | cmd | len | payload | sumCheck | addCheck    板 -> PC（上行）
    sumCheck = 累加和, addCheck = 累加和的累加和, 都取低 8 位。

【用法】
    # 无硬件自检（先跑这个，确认解析与中继逻辑本身是对的）
    python3 task_space_smc/hw/serial_tap.py --selftest

    # 抓包：先起中继，它会打印出给节点用的端口路径
    python3 task_space_smc/hw/serial_tap.py --port /dev/ttyUSB0 \\
        --seconds 30 --capture /tmp/tap.bin
    # 另一个终端（注意 ROS_DOMAIN_ID，见 HANDOFF 10.2）
    export ROS_DOMAIN_ID=77
    ros2 launch manipulator student_arm6.launch.py \\
        arm_type:=a_l1 port_name:=/tmp/claude-.../ttyTAP ...

    # 离线复解已有抓包
    python3 task_space_smc/hw/serial_tap.py --decode /tmp/tap.bin
"""

from __future__ import annotations

import argparse
import errno
import os
import select
import struct
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

HEAD = 0xFE
ADDR_TO_BOARD = 0xAA          # PC -> 板，下行
ADDR_TO_PC = 0x55             # 板 -> PC，上行
N_CHANNEL = 7                 # 固件固定按 7 通道收发，实物只有前 6 个

# 本协议最大的载荷就是 7 x float32 = 28 字节（0x10~0x14 与 0x20~0x25 全是）。
# dataLen 是单字节，语法上能到 255，但凡声明超过这个上界的都必然是载荷里
# 撞巧出现的假帧头。不设这个上界，一个假帧头会让解析器停下来干等 254 字节
# 才发现校验不过 —— 200 Hz 下足以吞掉将近一个控制周期的真帧。
MAX_PAYLOAD = 64

DOWN_CMDS = {
    0x10: ("position", "f"),
    0x11: ("velocity", "f"),
    0x12: ("torque", "f"),     # 单位 Nm，不是安培（README 10.3）
    0x13: ("kp", "f"),
    0x14: ("kd", "f"),
}
UP_CMDS = {
    0x01: ("heartbeat", None),
    0x20: ("position", "f"),
    0x21: ("velocity", "f"),
    0x22: ("current", "f"),    # 板子上报的力矩，与下行 0x12 同单位
    0x23: ("voltage", "f"),
    0x24: ("temperature", "H"),
    0x25: ("unknown", None),   # 2 Hz，payload [1,1,1,0,0,0,0]，含义未确认
}

# 抓包文件的记录格式：方向(1B) | 时间戳 ns(8B, LE) | 长度(4B, LE) | 原始字节
_REC_HDR = struct.Struct("<BQI")
_DIR_DOWN = 0
_DIR_UP = 1


def checksum(body: bytes) -> tuple[int, int]:
    """返回 (sumCheck, addCheck)。body 是不含两个校验字节的帧头+载荷。"""
    s = 0
    a = 0
    for byte in body:
        s = (s + byte) & 0xFF
        a = (a + s) & 0xFF
    return s, a


def build_frame(addr: int, cmd: int, payload: bytes) -> bytes:
    """按 MakeFrame 的规则造一帧 —— 仅供自检与离线构造使用。"""
    body = bytes([HEAD, addr, cmd, len(payload)]) + payload
    return body + bytes(checksum(body))


@dataclass
class Frame:
    t: float
    addr: int
    cmd: int
    payload: bytes

    @property
    def name(self) -> str:
        table = DOWN_CMDS if self.addr == ADDR_TO_BOARD else UP_CMDS
        return table.get(self.cmd, ("cmd_%02x" % self.cmd, None))[0]

    def values(self):
        """按 cmd 解出通道数组；无法解释的返回 None。"""
        table = DOWN_CMDS if self.addr == ADDR_TO_BOARD else UP_CMDS
        fmt = table.get(self.cmd, (None, None))[1]
        if fmt is None:
            return None
        width = struct.calcsize("<" + fmt)
        n = len(self.payload) // width
        if n == 0:
            return None
        return np.array(struct.unpack("<%d%s" % (n, fmt), self.payload[:n * width]),
                        dtype=float)


class FrameStream:
    """单方向的流式帧解析器。

    只认属于本方向的地址字节，这样即使两个方向的日志被混在一起也不会串味。
    校验失败时前进 1 字节重新同步 —— 前进 2 会跳过紧邻的真帧头。
    """

    def __init__(self, addr: int):
        self.addr = addr
        self._buf = bytearray()
        self._marker = bytes([HEAD, addr])
        self.frames = 0
        self.checksum_errors = 0
        self.resync_bytes = 0       # 被丢弃的、不属于任何合法帧的字节数
        self.bytes_seen = 0

    def feed(self, chunk: bytes, t: float) -> list[Frame]:
        """喂入原始字节，返回本次解出的完整帧。"""
        self._buf += chunk
        self.bytes_seen += len(chunk)
        out = []
        i = 0
        while True:
            j = self._buf.find(self._marker, i)
            if j < 0:
                # 保留末尾 1 字节，防止帧头正好被切在两次 read 之间
                self.resync_bytes += max(0, len(self._buf) - 1 - i)
                i = max(i, len(self._buf) - 1)
                break
            self.resync_bytes += j - i
            if j + 4 > len(self._buf):
                i = j
                break
            length = self._buf[j + 3]
            if length > MAX_PAYLOAD:        # 假帧头，别为它干等（见 MAX_PAYLOAD）
                self.checksum_errors += 1
                i = j + 1
                continue
            end = j + 4 + length + 2
            if end > len(self._buf):
                i = j
                break                       # 帧还没收全，等下一次 read
            frame = bytes(self._buf[j:end])
            s, a = checksum(frame[:-2])
            if frame[-2] != s or frame[-1] != a:
                self.checksum_errors += 1
                i = j + 1
                continue
            self.frames += 1
            out.append(Frame(t=t, addr=self.addr, cmd=frame[2],
                             payload=frame[4:4 + length]))
            i = end
        del self._buf[:i]
        return out

    @property
    def pending_bytes(self) -> int:
        """尚未凑成完整帧的残留字节 —— 持续增长意味着流被截断了。"""
        return len(self._buf)


@dataclass
class Stats:
    """按 cmd 分类的统计，外加中继层的健康指标。"""
    per_cmd: dict = field(default_factory=lambda: defaultdict(list))
    first_t: float | None = None
    last_t: float | None = None
    short_writes: int = 0        # os.write 未一次写完的次数
    write_bytes: int = 0
    stalled_writes: int = 0      # 对侧不排空、超时放弃的次数
    dropped_bytes: int = 0       # 因上一条而丢掉的字节

    def add(self, frame: Frame):
        if self.first_t is None:
            self.first_t = frame.t
        self.last_t = frame.t
        self.per_cmd[frame.cmd].append(frame)

    @property
    def span(self) -> float:
        if self.first_t is None or self.last_t is None:
            return 0.0
        return max(self.last_t - self.first_t, 0.0)


def _set_nonblocking(fd: int):
    import fcntl
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)


def _write_all(fd: int, data: bytes, stats: Stats,
               timeout: float = 0.5) -> None:
    """把 data 全部写出去，统计短写次数；对侧写不进去时【有界放弃】。

    serial_bus.cc:30 的 serial_.write() 丢弃了返回值 —— 那正是本函数要
    暴露的失败模式：短写会把一帧截断，板子按校验丢掉，表现就是"力矩没到"。

    【为什么必须有超时 —— 实测踩过】
    中继模式下本工具坐在控制链路正中间。若无条件重试，对侧一旦停止排空
    （缓冲写满），这里就会永远阻塞，整个中继无声冻结：节点收不到反馈、
    抓包停在某一秒、而屏幕上看不出任何异常。对一个上机时在链路里的工具，
    静默卡死是最坏的结局 —— 宁可丢字节并【大声报出来】，也不能装作还活着。
    丢掉的半帧会在板子那边校验失败，是可见且安全的失效方式。
    """
    view = memoryview(data)
    deadline = time.monotonic() + timeout
    while view:
        try:
            n = os.write(fd, view)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    stats.stalled_writes += 1
                    stats.dropped_bytes += len(view)
                    return
                select.select([], [fd], [], min(remaining, 0.05))
                continue
            raise
        if n < len(view):
            stats.short_writes += 1
        stats.write_bytes += n
        view = view[n:]


class Recorder:
    """把两个方向的原始字节按时间戳落盘，供事后复解。"""

    def __init__(self, path: str | None):
        self._fh = open(path, "wb") if path else None
        self.path = path

    def write(self, direction: int, t_ns: int, data: bytes):
        if self._fh is None:
            return
        self._fh.write(_REC_HDR.pack(direction, t_ns, len(data)))
        self._fh.write(data)

    def close(self):
        if self._fh is not None:
            self._fh.close()
            self._fh = None


def read_capture(path: str):
    """复解抓包文件，返回 (down_frames, up_frames, down_stats, up_stats)。"""
    down = FrameStream(ADDR_TO_BOARD)
    up = FrameStream(ADDR_TO_PC)
    dstats, ustats = Stats(), Stats()
    with open(path, "rb") as fh:
        blob = fh.read()
    off = 0
    while off + _REC_HDR.size <= len(blob):
        direction, t_ns, length = _REC_HDR.unpack_from(blob, off)
        off += _REC_HDR.size
        data = blob[off:off + length]
        off += length
        t = t_ns / 1e9
        if direction == _DIR_DOWN:
            for f in down.feed(data, t):
                dstats.add(f)
        else:
            for f in up.feed(data, t):
                ustats.add(f)
    return down, up, dstats, ustats


def relay(node_fd: int, board_fd: int, seconds: float,
          recorder: Recorder, dstats: Stats, ustats: Stats,
          progress: bool = True):
    """透明中继主循环。

    node_fd  控制节点那一侧（PTY 主设备）—— 从这里读到的就是【下行】字节
    board_fd 真串口 —— 从这里读到的就是【上行】字节

    唯一的两个发送点都在这里，且发送的都是对侧 read() 回来的原始缓冲，
    脚本自身不构造任何字节。
    """
    down = FrameStream(ADDR_TO_BOARD)
    up = FrameStream(ADDR_TO_PC)
    t0 = time.time()
    next_report = t0 + 1.0
    while time.time() - t0 < seconds:
        ready, _, _ = select.select([node_fd, board_fd], [], [], 0.02)
        now = time.time()
        for fd in ready:
            try:
                chunk = os.read(fd, 8192)
            except OSError as exc:
                # PTY 主设备在从设备没打开时返回 EIO；节点还没起来而已
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO):
                    continue
                raise
            if not chunk:
                continue
            if fd == node_fd:
                _write_all(board_fd, chunk, dstats)          # 原样转发，不改写
                recorder.write(_DIR_DOWN, int(now * 1e9), chunk)
                for f in down.feed(chunk, now):
                    dstats.add(f)
            else:
                _write_all(node_fd, chunk, ustats)           # 原样转发，不改写
                recorder.write(_DIR_UP, int(now * 1e9), chunk)
                for f in up.feed(chunk, now):
                    ustats.add(f)
        if progress and now >= next_report:
            next_report = now + 1.0
            n12 = len(dstats.per_cmd.get(0x12, ()))
            n22 = len(ustats.per_cmd.get(0x22, ()))
            print(f"\r  t={now - t0:5.1f}s  下行帧 {down.frames:6d}"
                  f"（0x12 力矩 {n12:6d}）  上行帧 {up.frames:6d}"
                  f"（0x22 电流 {n22:6d}）  校验错 "
                  f"{down.checksum_errors}/{up.checksum_errors}",
                  end="", flush=True)
    if progress:
        print()
    return down, up


def report(down: FrameStream, up: FrameStream, dstats: Stats, ustats: Stats,
           n_arm: int = 6, fake_board: bool = False,
           expect_torque: bool = True) -> int:
    """打印判读报告，返回进程退出码。0 = 下行通路已验证正确。"""
    print("\n" + "=" * 72)
    print("下行（PC -> 板，0xFE 0xAA）")
    print("=" * 72)
    if down.frames == 0:
        print("  没有抓到任何下行帧。")
        print("  说明控制节点根本没往这个端口写字节 —— 先确认：")
        print("    * 节点的 port_name 是否指向本脚本打印的 PTY 路径")
        print("    * enable_direct_effort_command 是否为 true")
        print("    * /student/joint_effort_command 是否真的有人发布")
        return 1
    _print_stream(down, dstats, DOWN_CMDS, n_arm)

    if fake_board:
        # 上行是本脚本自己造的，拿它做跟随性判据毫无意义 —— 必须挡掉，
        # 否则一份假板子的报告会被误读成对真板子的结论。
        print("\n" + "=" * 72)
        print("判读（假板子模式：上行由本脚本合成，不构成对真板子的结论）")
        print("=" * 72)
        return _verdict_downlink_only(down, dstats, n_arm, expect_torque)

    print("\n" + "=" * 72)
    print("上行（板 -> PC，0xFE 0x55）")
    print("=" * 72)
    if up.frames == 0:
        print("  没有抓到任何上行帧 —— 板子没上电，或串口路径不对。")
    else:
        _print_stream(up, ustats, UP_CMDS, n_arm)

    print("\n" + "=" * 72)
    print("判读")
    print("=" * 72)
    return _verdict(down, up, dstats, ustats, n_arm, expect_torque)


def _detect_position_hold(dstats: Stats, n_arm: int) -> str | None:
    """认出「节点其实在位置保持，根本没发力矩」这个静默降级。

    纯力矩（MIT）模式的定义就是 kp = kd = 0，力矩全靠 t_ff。所以只要
    kp 恒不为零、而 0x12 力矩恒为零，节点就没在做力矩控制 —— 不管它
    自己以为在做什么。这个判据不依赖任何配置值。

    【为什么专门查它 —— 真实踩过】
    student_arm_node.cc 的 StudentEffortCommandCallback 在 msg.enable == 0
    时【静默】把力矩指令换成带默认增益的位置保持，一行日志都不打；
    而 ValidateEffortCommand 在 enable == 0 时提前 return true，连长度
    检查都跳过。上游的 hardware_enabled 在 runner（默认 False）和
    launch（default_value='false'）两处都默认关闭。三者叠加的结果是：
    控制器日志里满幅力矩，线上却一个字节的力矩都没有，机械臂纹丝不动。
    """
    tq_frames = dstats.per_cmd.get(0x12, ())
    kp_frames = dstats.per_cmd.get(0x13, ())
    if not tq_frames or not kp_frames:
        return None
    tq = np.array([f.values()[:n_arm] for f in tq_frames])
    kp = np.array([f.values()[:n_arm] for f in kp_frames])
    if np.abs(tq).max() > 1e-6:
        return None                       # 确实发了力矩，不是这个毛病
    if np.abs(kp).max() < 1e-6:
        return None                       # kp 也是 0，那是真的在发零力矩
    return (
        f"力矩恒为 0 而 kp 恒不为 0（均值 {np.round(kp.mean(axis=0), 2).tolist()}）"
        " —— 节点在【位置保持】，根本没有在做力矩控制。\n"
        "        纯力矩模式的定义是 kp=kd=0，出现默认增益就说明指令被降级了。\n"
        "        按可能性排查：\n"
        "          1. MotorControl.enable 是不是 0？student_arm_node 会【静默】\n"
        "             把 enable=0 的力矩指令换成位置保持，不打任何日志。\n"
        "          2. SMC 的 hardware_enabled 是不是没开？runner 与 launch\n"
        "             两处默认都是 false，不显式传 true 就会发 enable=0。\n"
        "          3. 力矩指令超时了？超时兜底同样是带默认增益的位置保持。")


def _downlink_problems(down: FrameStream, dstats: Stats, n_arm: int = 6,
                       expect_torque: bool = True) -> list[str]:
    """只看下行本身能发现的缺陷 —— 与上行是否存在无关。

    expect_torque=False 时不把"位置保持"当缺陷：没发力矩指令的抓包里
    位置保持本来就是【正确】状态。诊断工具误报比漏报更危险 —— 喊过一次
    狼来了，下次真出问题时就没人信了。
    """
    problems = []
    n_pos = len(dstats.per_cmd.get(0x10, ()))
    n_tq = len(dstats.per_cmd.get(0x12, ()))

    if expect_torque:
        hold = _detect_position_hold(dstats, n_arm)
        if hold:
            problems.append(hold)

    if n_tq == 0:
        problems.append(
            "下行【没有 0x12 力矩帧】。指令没走到 ProtocolV1::SetCurrent，"
            "或 cur_cmd_updated_ 没被置位。查 DMMotor::UpdateCommand 的"
            "早退分支（is_received_ / id 越界）。")
    elif n_pos > 0 and n_tq < 0.9 * n_pos:
        problems.append(
            f"下行 0x12 帧数 {n_tq} 明显少于 0x10 的 {n_pos}"
            f"（{n_tq / n_pos:.1%}）。Pop() 里力矩标志位有周期没被置起来。")

    if down.checksum_errors or dstats.short_writes:
        problems.append(
            f"下行有 {down.checksum_errors} 个校验错、{dstats.short_writes} 次短写"
            " —— 帧被截断，板子会直接丢弃。查 SerialBus::SendCore 的写返回值。")

    if dstats.stalled_writes:
        problems.append(
            f"中继写阻塞放弃 {dstats.stalled_writes} 次、丢 "
            f"{dstats.dropped_bytes} 字节 —— **本次抓包不完整，任何结论都不成立**。"
            "对侧没有排空缓冲：检查真串口是否被别的进程占用、板子是否还活着。")
    return problems


def _verdict_downlink_only(down: FrameStream, dstats: Stats, n_arm: int,
                           expect_torque: bool = True) -> int:
    """假板子模式的结论：驱动层发出来的字节对不对。"""
    problems = _downlink_problems(down, dstats, n_arm, expect_torque)
    if problems:
        for p in problems:
            print(f"  ⚠️ {p}")
        print("\n  【结论】驱动层就发错了 —— 不必上机也能定位，"
              "问题在 C++ 侧，与电机固件无关。")
        return 4

    tq = np.array([f.values()[:n_arm] for f in dstats.per_cmd[0x12]])
    kp = _mean_of(dstats, 0x13, n_arm)
    kd = _mean_of(dstats, 0x14, n_arm)
    print(f"  ✅ 驱动层发出的下行字节【正确】：0x12 力矩帧 {len(tq)} 帧，"
          f"校验全通过，无短写、无截断。")
    print(f"     力矩均值 {np.round(tq.mean(axis=0), 3).tolist()}")
    print(f"     峰值绝对值 {np.round(np.abs(tq).max(axis=0), 3).tolist()}")
    if kp is not None:
        print(f"     kp {np.round(kp, 3).tolist()}")
    if kd is not None:
        print(f"     kd {np.round(kd, 3).tolist()}")
    print()
    print("  【结论】C++ 侧（DMMotor::UpdateCommand / ProtocolV1::Pop /")
    print("          SerialBus::SendCore）已排除。HANDOFF 10.3 的嫌疑收敛到")
    print("          电机固件对 kp=kd=0 的处理 —— 上机时直接做 msg.p=5.0 的")
    print("          旁证实验，不必再花一次上机名额试探驱动层。")
    return 0


def _print_stream(stream: FrameStream, stats: Stats, table: dict, n_arm: int):
    span = stats.span
    print(f"  总帧 {stream.frames}  校验错 {stream.checksum_errors}  "
          f"丢弃字节 {stream.resync_bytes}  残留 {stream.pending_bytes}  "
          f"时长 {span:.2f}s")
    if stats.short_writes:
        print(f"  ⚠️ 短写 {stats.short_writes} 次 —— 帧被截断，"
              f"对应 serial_bus.cc:30 丢弃 write() 返回值的隐患")
    if stats.stalled_writes:
        print(f"  ⚠️ 写阻塞放弃 {stats.stalled_writes} 次、丢 "
              f"{stats.dropped_bytes} 字节 —— 对侧停止排空，本次抓包不完整，"
              f"结论不可用")
    print(f"\n  {'cmd':>5} {'名称':<12} {'帧数':>7} {'Hz':>8}   逐关节数值")
    for cmd in sorted(stats.per_cmd):
        frames = stats.per_cmd[cmd]
        name = table.get(cmd, ("cmd_%02x" % cmd, None))[0]
        hz = len(frames) / span if span > 0 else float("nan")
        vals = [f.values() for f in frames]
        vals = [v for v in vals if v is not None]
        if vals:
            arr = np.array(vals)[:, :n_arm]
            mean = np.round(arr.mean(axis=0), 3).tolist()
            std = float(arr.std(axis=0).max())
            desc = f"均值 {mean}  最大逐轴标准差 {std:.4f}"
        else:
            desc = "（无数值载荷）"
        print(f"  0x{cmd:02X} {name:<12} {len(frames):>7} {hz:>8.1f}   {desc}")


def _verdict(down: FrameStream, up: FrameStream, dstats: Stats, ustats: Stats,
             n_arm: int, expect_torque: bool = True) -> int:
    """把统计翻译成 HANDOFF 10.3 需要的那个二选一结论。"""
    problems = _downlink_problems(down, dstats, n_arm, expect_torque)
    tq_frames = dstats.per_cmd.get(0x12, ())
    n_tq = len(tq_frames)

    if not problems and n_tq:
        tq = np.array([f.values()[:n_arm] for f in tq_frames])
        kp = _mean_of(dstats, 0x13, n_arm)
        kd = _mean_of(dstats, 0x14, n_arm)
        print(f"  ✅ 下行 0x12 力矩帧 {n_tq} 帧，校验全通过，无短写。")
        print(f"     力矩均值 {np.round(tq.mean(axis=0), 3).tolist()}")
        print(f"     峰值绝对值 {np.round(np.abs(tq).max(axis=0), 3).tolist()}")
        if kp is not None:
            print(f"     kp {np.round(kp, 3).tolist()}   kd "
                  f"{np.round(kd, 3).tolist() if kd is not None else '—'}")

        cur_frames = ustats.per_cmd.get(0x22, ())
        if not cur_frames:
            print("\n  ⚠️ 上行没有 0x22 电流帧，无法做跟随性对比。")
            return 2
        cur = np.array([f.values()[:n_arm] for f in cur_frames])
        print(f"\n     上行 0x22 电流均值 {np.round(cur.mean(axis=0), 3).tolist()}")

        # 跟随性判据：满幅下行 vs 上行幅值。用均值绝对值比即可 —— 10.3 的
        # 现象是恒定直流，不需要做互相关。
        d_amp = np.abs(tq.mean(axis=0))
        u_amp = np.abs(cur.mean(axis=0))
        ratio = np.where(d_amp > 1e-6, u_amp / np.maximum(d_amp, 1e-9), np.nan)
        print(f"     上行/下行 幅值比 {np.round(ratio, 3).tolist()}")

        driven = d_amp > 0.5                      # 只看确实在使劲的关节
        if not driven.any():
            print("\n  下行力矩本来就接近零，这段抓包不构成对 10.3 的检验。"
                  "请在 approach 段（力矩饱和期）重抓。")
            return 2
        follow = np.nanmedian(ratio[driven])
        print()
        if follow < 0.2:
            print("  ⚠️ 【结论】下行帧内容正确，但板子上报的电流不跟随"
                  f"（幅值比中位 {follow:.3f}）。")
            print("     帧到了板子、力矩没到电机 -> 问题在电机固件对 "
                  "kp=kd=0 的处理。")
            print("     下一步：做 HANDOFF 10.3 的旁证实验 —— 把 msg.p 从 0 "
                  "改成 5.0，若机械臂开始响应即可确认。")
            return 3
        print(f"  ✅ 【结论】下行帧正确，上行电流跟随（幅值比中位 {follow:.3f}）。")
        print("     力矩确实产生了。机械臂不动就不是通路问题，"
              "应转向机械/摩擦方向（j2 静摩擦 1.775 Nm）。")
        return 0

    for p in problems:
        print(f"  ⚠️ {p}")
    print("\n  【结论】问题在【驱动层】，不在电机固件。")
    return 4


def _mean_of(stats: Stats, cmd: int, n_arm: int):
    frames = stats.per_cmd.get(cmd, ())
    if not frames:
        return None
    return np.array([f.values()[:n_arm] for f in frames]).mean(axis=0)


def _open_pty(link: str | None):
    """开一个 PTY 并伪装成串口，返回 (主设备 fd, 从设备 fd, 给节点用的路径)。

    必须设 raw：默认行规程会把 0x0A 翻成 0x0D 0x0A，二进制帧当场被改写。
    """
    import pty
    import tty

    master_fd, slave_fd = pty.openpty()
    tty.setraw(master_fd)
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    if link:
        if os.path.islink(link) or os.path.exists(link):
            os.unlink(link)
        os.symlink(slave_name, link)
    _set_nonblocking(master_fd)
    return master_fd, slave_fd, (link or slave_name)


def run_fake_board(args) -> int:
    """冒充控制板：按实测速率回放上行帧，同时解码节点发来的下行帧。

    【为什么值得有这个模式】
    它把「驱动层发的字节对不对」和「电机固件怎么处理」彻底分开，而且
    【不需要实物、不需要通电】。真正的 student_arm_node 接上来，我们就能
    在本机确认 0x12 力矩帧是否按预期发出、数值对不对、kp/kd 是不是 0。

    如果这里一切正常，HANDOFF 10.3 的嫌疑就只剩电机固件对 kp=kd=0 的处理，
    上机时直接做 msg.p=5.0 的旁证实验即可，不必再耗一次上机名额去试探。

    上行速率照抄实测值（README 10.2）：
        0x20/0x21/0x22/0x24  200 Hz      0x23 voltage  2 Hz（固件未实现，恒 0）
        0x01 heartbeat        20 Hz      0x25          2 Hz
    心跳【必须】发 —— student_arm_node 在心跳陈旧时会封锁力矩前馈、改为
    位置保持，那样抓到的就不是我们想验的东西了。
    """
    master_fd, slave_fd, node_port = _open_pty(args.link)
    keep_open = os.open(os.ttyname(slave_fd), os.O_RDWR | os.O_NOCTTY)

    # 位形默认取 HANDOFF 第 1 节最后读到的实物值，第 7 通道用幽灵数据，
    # 顺带检验 arm_version:=gamma6 的屏蔽是否生效。
    # ⚠️ 默认值里 j4~j6 都≈0，验"启动瞬间是否命令回零"这类问题时【必须】
    # 用 --fake-pose 传一组非零值，否则 0 和真值分不开。传的是【原始】
    # 0x20 值（未按 coord_system 取反），与串口上看到的一致。
    pos = np.array([0.0250, 1.3209, -1.3235, 0.0006, -0.0025, -0.0029, -6.5231])
    if getattr(args, "fake_pose", None):
        given = [float(v) for v in args.fake_pose.split(",")]
        if len(given) != 6:
            print("--fake-pose 需要 6 个逗号分隔的原始位置", file=sys.stderr)
            return 2
        pos[:6] = given
    vel = np.zeros(N_CHANNEL)
    vel[6] = 27.9731
    cur = np.zeros(N_CHANNEL)
    temp = np.array([28, 29, 30, 27, 28, 31, 1], dtype=np.uint16)

    recorder = Recorder(args.capture)
    dstats, ustats = Stats(), Stats()
    down = FrameStream(ADDR_TO_BOARD)

    print(f"假板子模式：不碰真串口，节点可用端口  {node_port}")
    print()
    print("在另一个终端启动节点（专属域，见 HANDOFF 10.2）：")
    print("  export ROS_DOMAIN_ID=77")
    print(f"  ros2 run manipulator student_arm_node --ros-args \\")
    print(f"      -p arm_type:=a_l1 -p arm_version:=gamma6 \\")
    print(f"      -p port_name:={node_port} \\")
    print(f"      -p enable_direct_effort_command:=true \\")
    print(f"      -p motor_config_path:=<share>/motor_config.yaml \\")
    print(f"      -p arm_config_path:=<share>/arm_config.yaml")
    print()
    print(f"回放上行 {args.seconds:.0f}s，Ctrl-C 提前结束……")

    t0 = time.monotonic()
    period = 1.0 / 200.0
    tick = 0
    next_tick = t0
    next_report = time.time() + 1.0
    try:
        while time.monotonic() - t0 < args.seconds:
            now_m = time.monotonic()
            if now_m >= next_tick:
                tick += 1
                next_tick += period
                if args.echo_torque:
                    # 冒充一块"确实把力矩加到电机上"的板子，用来验判读逻辑
                    last = dstats.per_cmd.get(0x12, ())
                    if last:
                        cur[:] = last[-1].values()
                out = bytearray()
                out += build_frame(ADDR_TO_PC, 0x20,
                                   struct.pack("<%df" % N_CHANNEL, *pos))
                out += build_frame(ADDR_TO_PC, 0x21,
                                   struct.pack("<%df" % N_CHANNEL, *vel))
                out += build_frame(ADDR_TO_PC, 0x22,
                                   struct.pack("<%df" % N_CHANNEL, *cur))
                out += build_frame(ADDR_TO_PC, 0x24,
                                   struct.pack("<%dH" % N_CHANNEL, *temp))
                if tick % 10 == 0:                      # 20 Hz 心跳
                    out += build_frame(ADDR_TO_PC, 0x01, b"")
                if tick % 100 == 0:                     # 2 Hz
                    out += build_frame(ADDR_TO_PC, 0x23,
                                       struct.pack("<%df" % N_CHANNEL,
                                                   *np.zeros(N_CHANNEL)))
                    out += build_frame(ADDR_TO_PC, 0x25, bytes(
                        struct.pack("<%df" % N_CHANNEL,
                                    *[1, 1, 1, 0, 0, 0, 0])))
                _write_all(master_fd, bytes(out), ustats)
                recorder.write(_DIR_UP, int(time.time() * 1e9), bytes(out))

            ready, _, _ = select.select([master_fd], [], [], 0.001)
            if ready:
                try:
                    chunk = os.read(master_fd, 8192)
                except OSError as exc:
                    if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK,
                                         errno.EIO):
                        raise
                    chunk = b""
                if chunk:
                    wall = time.time()
                    recorder.write(_DIR_DOWN, int(wall * 1e9), chunk)
                    for f in down.feed(chunk, wall):
                        dstats.add(f)

            wall = time.time()
            if wall >= next_report:
                next_report = wall + 1.0
                n12 = len(dstats.per_cmd.get(0x12, ()))
                print(f"\r  t={time.monotonic() - t0:5.1f}s  "
                      f"上行已发 {tick} 周期  下行帧 {down.frames:6d}"
                      f"（0x12 力矩 {n12:6d}）  校验错 {down.checksum_errors}",
                      end="", flush=True)
        print()
    except KeyboardInterrupt:
        print("\n已中断，输出已抓到的部分。")
    finally:
        recorder.close()
        os.close(keep_open)
        os.close(master_fd)
        os.close(slave_fd)
        if args.link and os.path.islink(args.link):
            os.unlink(args.link)

    if args.capture:
        print(f"原始抓包已存 {args.capture}")
    return report(down, FrameStream(ADDR_TO_PC), dstats, ustats, args.n_arm,
                  fake_board=True,
                  expect_torque=not args.no_expect_torque)


def run_proxy(args) -> int:
    try:
        import serial
    except ImportError:
        print("需要 pyserial：pip install pyserial", file=sys.stderr)
        return 2

    master_fd, slave_fd, node_port = _open_pty(args.link)
    slave_name = os.ttyname(slave_fd)
    link = args.link

    port = serial.Serial(args.port, args.baud, timeout=0)
    board_fd = port.fileno()
    _set_nonblocking(master_fd)
    _set_nonblocking(board_fd)

    recorder = Recorder(args.capture)
    dstats, ustats = Stats(), Stats()

    print(f"真串口 {args.port} @ {args.baud} 已打开")
    print(f"给控制节点用的端口： {node_port}")
    print()
    print("在另一个终端启动节点（注意专属域，见 HANDOFF 10.2）：")
    print("  export ROS_DOMAIN_ID=77")
    print(f"  ros2 launch manipulator student_arm6.launch.py \\")
    print(f"      arm_type:=a_l1 port_name:={node_port} \\")
    print(f"      enable_direct_effort_command:=true use_rviz:=false")
    print()
    print(f"抓 {args.seconds:.0f}s，Ctrl-C 提前结束……")

    # 保持从设备常开：否则节点未连接时主设备 read 会一直返回 EIO
    keep_open = os.open(slave_name, os.O_RDWR | os.O_NOCTTY)
    try:
        down, up = relay(master_fd, board_fd, args.seconds,
                         recorder, dstats, ustats)
    except KeyboardInterrupt:
        print("\n已中断，输出已抓到的部分。")
        down, up = FrameStream(ADDR_TO_BOARD), FrameStream(ADDR_TO_PC)
        recorder.close()
        if args.capture:
            down, up, dstats, ustats = read_capture(args.capture)
    finally:
        recorder.close()
        os.close(keep_open)
        port.close()
        os.close(master_fd)
        os.close(slave_fd)
        if link and os.path.islink(link):
            os.unlink(link)

    if args.capture:
        print(f"原始抓包已存 {args.capture}"
              f"（{os.path.getsize(args.capture)} 字节）")
    return report(down, up, dstats, ustats, args.n_arm,
                  expect_torque=not args.no_expect_torque)


# --------------------------------------------------------------------------
# 自检：无硬件也能确认解析、重同步、中继保真都是对的
# --------------------------------------------------------------------------

def _selftest() -> int:
    _test_frame_roundtrip()
    _test_checksum_rejected()
    _test_resync_after_garbage()
    _test_split_across_reads()
    _test_truncated_frame_is_not_reported()
    _test_direction_isolation()
    _test_relay_is_byte_exact()
    _test_pty_accepts_baudrate()
    _test_write_stall_is_bounded()
    _test_detects_position_hold()
    _test_hold_not_flagged_when_torque_not_expected()
    _test_verdict_paths()
    print("自检全部通过。")
    return 0


def _torque_frame(vals, addr=ADDR_TO_BOARD, cmd=0x12):
    payload = struct.pack("<%df" % N_CHANNEL, *vals)
    return build_frame(addr, cmd, payload)


def _test_frame_roundtrip():
    truth = [1.5, -9.0, 0.25, -1.6, 0.0, 0.125, 0.0]
    stream = FrameStream(ADDR_TO_BOARD)
    frames = stream.feed(_torque_frame(truth), 0.0)
    assert len(frames) == 1, frames
    assert frames[0].cmd == 0x12 and frames[0].name == "torque"
    assert np.allclose(frames[0].values(), truth, atol=1e-6)
    assert stream.checksum_errors == 0 and stream.resync_bytes == 0
    print("  ✓ 帧构造/解析往返一致")


def _test_checksum_rejected():
    good = bytearray(_torque_frame([0.0] * N_CHANNEL))
    good[6] ^= 0xFF                                  # 破坏载荷
    stream = FrameStream(ADDR_TO_BOARD)
    assert stream.feed(bytes(good), 0.0) == []
    assert stream.checksum_errors == 1
    print("  ✓ 校验错被拒绝")


def _test_resync_after_garbage():
    good = _torque_frame([1.0] * N_CHANNEL)
    stream = FrameStream(ADDR_TO_BOARD)
    blob = b"\x00\xfe\xaa\x99" + good + b"\x11\x22" + good
    frames = stream.feed(blob, 0.0)
    assert len(frames) == 2, len(frames)
    assert stream.frames == 2
    print("  ✓ 垃圾字节后能重新同步")


def _test_split_across_reads():
    """一帧被切成任意两段送进来，都必须完整解出 —— 200 Hz 下这是常态。"""
    good = _torque_frame([-9.0] * N_CHANNEL)
    for cut in range(1, len(good)):
        stream = FrameStream(ADDR_TO_BOARD)
        got = stream.feed(good[:cut], 0.0) + stream.feed(good[cut:], 0.01)
        assert len(got) == 1, (cut, len(got))
        assert np.allclose(got[0].values(), [-9.0] * N_CHANNEL, atol=1e-6)
    print("  ✓ 跨 read 边界切分的帧能拼回（%d 个切点全过）" % (len(good) - 1))


def _test_truncated_frame_is_not_reported():
    """短写导致的截断必须表现为 pending 残留，而不是被当成合法帧。"""
    good = _torque_frame([2.0] * N_CHANNEL)
    stream = FrameStream(ADDR_TO_BOARD)
    assert stream.feed(good[:-3], 0.0) == []
    assert stream.frames == 0
    assert stream.pending_bytes == len(good) - 3
    print("  ✓ 被截断的帧不会被误报为合法帧")


def _test_direction_isolation():
    """上行流里混进下行帧不得被计入，反之亦然。"""
    down = _torque_frame([1.0] * N_CHANNEL, addr=ADDR_TO_BOARD, cmd=0x12)
    up = _torque_frame([2.0] * N_CHANNEL, addr=ADDR_TO_PC, cmd=0x22)
    ds, us = FrameStream(ADDR_TO_BOARD), FrameStream(ADDR_TO_PC)
    assert len(ds.feed(down + up, 0.0)) == 1
    assert len(us.feed(down + up, 0.0)) == 1
    print("  ✓ 两个方向按地址字节隔离")


def _test_relay_is_byte_exact():
    """中继必须【进什么出什么】—— 用随机字节流验证，含非法帧。"""
    import pty as _pty
    import random

    rng = random.Random(20260802)
    payload = bytes(rng.randrange(256) for _ in range(20000))

    a_master, a_slave = _pty.openpty()      # 冒充控制节点
    b_master, b_slave = _pty.openpty()      # 冒充控制板
    for fd in (a_master, a_slave, b_master, b_slave):
        import tty as _tty
        _tty.setraw(fd)
    _set_nonblocking(a_master)
    _set_nonblocking(b_master)

    stats = Stats()
    got = bytearray()
    sent = 0
    deadline = time.time() + 5.0
    _set_nonblocking(a_slave)
    _set_nonblocking(b_slave)
    while len(got) < len(payload) and time.time() < deadline:
        if sent < len(payload):
            n = os.write(a_slave, payload[sent:sent + 2048])
            sent += n
        ready, _, _ = select.select([a_master], [], [], 0.05)
        for fd in ready:
            try:
                chunk = os.read(fd, 8192)
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EIO):
                    continue
                raise
            _write_all(b_master, chunk, stats)
        while True:
            try:
                chunk = os.read(b_slave, 8192)
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EIO):
                    break
                raise
            if not chunk:
                break
            got += chunk

    for fd in (a_master, a_slave, b_master, b_slave):
        os.close(fd)
    assert bytes(got) == payload[:len(got)], "中继改写了字节流"
    assert len(got) == len(payload), f"只中继了 {len(got)}/{len(payload)} 字节"
    print(f"  ✓ 中继字节级保真（{len(payload)} 随机字节，含非法帧）")


def _test_pty_accepts_baudrate():
    """PTY 必须能被 pyserial 按 921600 打开 —— C++ serial 库走同样的 termios。"""
    import pty as _pty
    try:
        import serial
    except ImportError:
        print("  – 跳过 PTY 波特率检查（无 pyserial）")
        return
    master_fd, slave_fd = _pty.openpty()
    try:
        port = serial.Serial(os.ttyname(slave_fd), 921600, timeout=0)
        os.write(master_fd, b"\xfe\x55ping")
        time.sleep(0.05)
        assert port.read(16) == b"\xfe\x55ping"
        port.close()
    finally:
        os.close(master_fd)
        os.close(slave_fd)
    print("  ✓ PTY 接受 921600 并透传二进制（含 0x0A 之外的控制字节）")


def _test_write_stall_is_bounded():
    """对侧不排空时必须【有界放弃并计数】，不能无限阻塞。

    实测踩过：用 socat 串起来的台架里，下游只写不读，缓冲写满后中继
    在 1 秒后就永远卡在写调用上，抓包停摆而屏幕上毫无异常。
    """
    import pty as _pty
    import tty as _tty

    master_fd, slave_fd = _pty.openpty()
    _tty.setraw(master_fd)
    _tty.setraw(slave_fd)
    _set_nonblocking(master_fd)
    stats = Stats()
    t0 = time.monotonic()
    # 从设备一个字节都不读，主设备缓冲很快写满
    for _ in range(200):
        _write_all(master_fd, b"\x00" * 4096, stats, timeout=0.15)
        if stats.stalled_writes:
            break
    elapsed = time.monotonic() - t0
    os.close(master_fd)
    os.close(slave_fd)

    assert stats.stalled_writes > 0, "对侧不排空却没有触发放弃 —— 会无限阻塞"
    assert stats.dropped_bytes > 0, "放弃了却没统计丢掉的字节"
    assert elapsed < 30, f"放弃得太慢（{elapsed:.1f}s）"
    print(f"  ✓ 对侧不排空时有界放弃（{elapsed:.2f}s 内触发，"
          f"丢 {stats.dropped_bytes} 字节且已计数）")


def _test_detects_position_hold():
    """静默降级成位置保持必须被认出来 —— 数据取自真实 A/B 实测。

    实测（假板子 + 真 student_arm_node，control_rate 200 Hz）：
        enable=0  0x12 力矩 [0,0,0,0,0,0]   0x13 kp [30,30,30,5,5,5]
        enable=1  0x12 力矩确实出现，且 j4~j6 因 left_hand 坐标系取反
    """
    n = 6

    def stats_of(torque, kp):
        st = Stats()
        stream = FrameStream(ADDR_TO_BOARD)
        for k in range(50):
            blob = (_torque_frame(list(torque) + [0.0])
                    + build_frame(ADDR_TO_BOARD, 0x13, struct.pack(
                        "<%df" % N_CHANNEL, *(list(kp) + [0.0]))))
            for f in stream.feed(blob, k * 0.005):
                st.add(f)
        return st

    # enable=0 的实测特征：力矩恒 0，kp 是默认增益
    hold = _detect_position_hold(stats_of([0.0] * n, [30, 30, 30, 5, 5, 5]), n)
    assert hold is not None and "位置保持" in hold, hold
    assert "enable" in hold, "报告必须点名 MotorControl.enable"

    # 真正的纯力矩模式：kp 也是 0，不该报警
    assert _detect_position_hold(stats_of([0.0] * n, [0.0] * n), n) is None
    # 确实在发力矩：不该报警
    assert _detect_position_hold(
        stats_of([-9.0] * n, [30, 30, 30, 5, 5, 5]), n) is None
    print("  ✓ 认得出 enable=0 导致的静默降级（力矩恒 0 而 kp 非 0）")


def _test_hold_not_flagged_when_torque_not_expected():
    """没打算发力矩时，位置保持【不得】被报成缺陷。

    实测踩过：阶段 1 的零位移接管里本来就不发力矩指令，位置保持是正确
    状态，工具却报了「问题在驱动层」。诊断误报比漏报更危险 —— 喊过一次
    狼来了，下次真出问题就没人信了。
    """
    n = 6
    st = Stats()
    stream = FrameStream(ADDR_TO_BOARD)
    for k in range(50):
        blob = (_torque_frame([0.0] * (n + 1))
                + build_frame(ADDR_TO_BOARD, 0x13, struct.pack(
                    "<%df" % N_CHANNEL, *([30, 30, 30, 5, 5, 5, 1]))))
        for f in stream.feed(blob, k * 0.005):
            st.add(f)

    flagged = _downlink_problems(stream, st, n, expect_torque=True)
    assert any("位置保持" in p for p in flagged), "该报的时候没报"

    quiet = _downlink_problems(stream, st, n, expect_torque=False)
    assert not any("位置保持" in p for p in quiet), f"不该报却报了: {quiet}"
    print("  ✓ 未预期力矩时不把位置保持误报为缺陷")


def _test_verdict_paths():
    """三条判读分支各造一份数据，确认结论不会互相串。"""
    n = 6

    def make(tq, cur, n_frames=200):
        ds, us = Stats(), Stats()
        down, up = FrameStream(ADDR_TO_BOARD), FrameStream(ADDR_TO_PC)
        for k in range(n_frames):
            t = k * 0.005
            for f in down.feed(_torque_frame(list(tq) + [0.0]), t):
                ds.add(f)
            for f in down.feed(build_frame(
                    ADDR_TO_BOARD, 0x10,
                    struct.pack("<%df" % N_CHANNEL, *([0.0] * N_CHANNEL))), t):
                ds.add(f)
            for f in up.feed(_torque_frame(list(cur) + [0.0],
                                           addr=ADDR_TO_PC, cmd=0x22), t):
                us.add(f)
        return down, up, ds, us

    # 板子不跟随 -> 固件嫌疑
    assert _verdict(*make([-9.0] * n, [0.0] * n), n) == 3
    # 板子跟随 -> 机械嫌疑
    assert _verdict(*make([-9.0] * n, [-8.7] * n), n) == 0
    # 完全没有下行力矩帧 -> 驱动层
    ds, us = Stats(), Stats()
    down, up = FrameStream(ADDR_TO_BOARD), FrameStream(ADDR_TO_PC)
    for f in down.feed(build_frame(ADDR_TO_BOARD, 0x10, struct.pack(
            "<%df" % N_CHANNEL, *([0.0] * N_CHANNEL))), 0.0):
        ds.add(f)
    assert _verdict(down, up, ds, us, n) == 4
    print("  ✓ 三条判读分支互不串味")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0", help="真串口")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--capture", default=None, help="原始抓包落盘路径")
    ap.add_argument("--link", default=None,
                    help="给 PTY 做一个稳定的符号链接路径，方便写进 launch 参数")
    ap.add_argument("--n-arm", type=int, default=6, help="实物关节数")
    ap.add_argument("--decode", default=None, help="离线复解已有抓包文件")
    ap.add_argument("--selftest", action="store_true",
                    help="不碰硬件，自检解析与中继逻辑")
    ap.add_argument("--fake-board", action="store_true",
                    help="冒充控制板（不碰真串口），验证驱动层发的字节对不对")
    ap.add_argument("--echo-torque", action="store_true",
                    help="假板子模式下让 0x22 回显收到的 0x12，冒充一块正常的板子")
    ap.add_argument("--no-expect-torque", action="store_true",
                    help="本次抓包不打算发力矩指令（例如只做零位移接管）。"
                         "此时位置保持是正确状态，不再报为缺陷")
    ap.add_argument("--fake-pose", default=None,
                    help="假板子上报的【原始】0x20 位置，6 个逗号分隔值。"
                         "验证启动瞬间行为时必须用非零值")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()
    if args.decode:
        down, up, dstats, ustats = read_capture(args.decode)
        return report(down, up, dstats, ustats, args.n_arm,
                      expect_torque=not args.no_expect_torque)
    if args.fake_board:
        return run_fake_board(args)
    return run_proxy(args)


if __name__ == "__main__":
    raise SystemExit(main())
