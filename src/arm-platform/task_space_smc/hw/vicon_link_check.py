#!/usr/bin/env python3
"""上机前的动捕链路体检：这条链路今天能不能跑 200 Hz 的保持 demo？

判据是「控制拍里拿不到可用基座数据的比例」，而不是话题在不在。链路劣化在 demo
跑起来之前看不出来：话题、rate、ping 都正常，实测传输延迟 p95 却可能比基线差 5 倍
（79.9 ms → 431.8 ms），一半的控制拍因此停发力矩。

本脚本【只读】：不启动驱动、不发任何 /student/* 话题、不碰串口，只 ping、读 WiFi
状态、订阅动捕话题。

用法：
    python3 src/arm-platform/task_space_smc/hw/vicon_link_check.py
    python3 src/arm-platform/task_space_smc/hw/vicon_link_check.py --seconds 60
    python3 src/arm-platform/task_space_smc/hw/vicon_link_check.py --json out.json

比较两条链路（例如 2.4G vs 5G）就是切换 WiFi 后各跑一次，对比末尾那张表。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# 判据。数字来自 docs/QA_record.md 的实测基线与 smc_params_hold.yaml 的阈值。
BASELINE = {
    "p95_ms": 79.9,          # QA_record 当时测到的诚实传输延迟 p95
    "max_ms": 185.6,
    "loss_pct": 2.81,
    "stall_pct_at_150ms": 0.42,   # 0.15 s 阈值下会停发力矩的控制拍占比
}
# 通过线：比基线差一倍以内算可接受；再差就不要上机 —— 190 ms 前的基座位姿
# 拿去算补偿本身就是错的，停发力矩才是正确行为，但停一半的拍就没有 demo 了。
PASS = {"stall_pct": 1.0, "p95_ms": 160.0, "loss_pct": 6.0}
WARN = {"stall_pct": 5.0, "p95_ms": 300.0, "loss_pct": 12.0}

CONTROL_HZ = 200.0
DEFAULT_TIMEOUT_SEC = 0.15      # 与 smc_params_hold.yaml 的 base_state_timeout_sec 一致


def _run(cmd, timeout=20):
    try:
        out = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                             timeout=timeout)
        return out.stdout.strip()
    except Exception:
        return ""


# --------------------------------------------------------------- WiFi

def wifi_status():
    """当前无线链路：频段、信道、信号、速率、同频邻居数。"""
    info = {}
    link = _run("iw dev wlo1 link")
    if not link or "Not connected" in link:
        for dev in _run("ls /sys/class/net").split():
            if dev.startswith(("wl", "wlan")):
                link = _run("iw dev %s link" % dev)
                info["iface"] = dev
                break
    else:
        info["iface"] = "wlo1"
    m = re.search(r"freq:\s*(\d+)", link)
    if m:
        info["freq_mhz"] = int(m.group(1))
        info["band"] = "5GHz" if info["freq_mhz"] > 5000 else "2.4GHz"
    m = re.search(r"signal:\s*(-?\d+)", link)
    if m:
        info["signal_dbm"] = int(m.group(1))
    m = re.search(r"rx bitrate:\s*([\d.]+)", link)
    if m:
        info["rx_mbit"] = float(m.group(1))
    m = re.search(r"tx bitrate:\s*([\d.]+)", link)
    if m:
        info["tx_mbit"] = float(m.group(1))
    # iw 把非 ASCII 的 SSID 转义成 \xNN，中文 SSID 会打成乱码；nmcli 给的是 UTF-8。
    active = _run("nmcli -t -f ACTIVE,SSID,CHAN dev wifi list")
    for line in active.splitlines():
        if line.startswith("yes:"):
            parts = line.split(":")
            info["ssid"] = parts[1] if len(parts) > 1 else "?"
            if len(parts) > 2 and parts[2].isdigit():
                info["channel"] = int(parts[2])
            break
    if "ssid" not in info:
        m = re.search(r"SSID:\s*(.+)", link)
        if m:
            raw = m.group(1).strip()
            try:                       # 把 \xNN 还原成真正的字节再按 UTF-8 解
                info["ssid"] = (raw.encode("latin-1", "backslashreplace")
                                .decode("unicode_escape")
                                .encode("latin-1").decode("utf-8", "replace"))
            except (UnicodeDecodeError, UnicodeEncodeError):
                info["ssid"] = raw

    # 同频邻居：2.4G 上挤在同一信道的 BSSID 数，是空口争用的直接指标。
    scan = _run("nmcli -t -f SSID,FREQ,CHAN dev wifi list", timeout=25)
    same, total = 0, 0
    for line in scan.splitlines():
        parts = line.split(":")
        if len(parts) < 3:
            continue
        try:
            freq = int(parts[1].split()[0])
        except (ValueError, IndexError):
            continue
        total += 1
        if info.get("freq_mhz") and abs(freq - info["freq_mhz"]) < 1:
            same += 1
    info["cochannel_bssids"] = same
    info["bssids_seen"] = total
    info["power_save"] = "off" if "off" in _run(
        "iw dev %s get power_save" % info.get("iface", "wlo1")) else "ON(应关掉)"
    return info


# --------------------------------------------------------------- ping

def ping_stats(target, count=60, interval=0.1):
    out = _run("ping -c %d -i %s -q %s" % (count, interval, target),
               timeout=count * interval + 20)
    m = re.search(r"= ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+) ms", out)
    loss = re.search(r"([\d.]+)% packet loss", out)
    if not m:
        return None
    return {
        "min_ms": float(m.group(1)), "avg_ms": float(m.group(2)),
        "max_ms": float(m.group(3)), "mdev_ms": float(m.group(4)),
        "loss_pct": float(loss.group(1)) if loss else float("nan"),
    }


def hop_localisation(vicon_host):
    """逐跳 ping，定位延迟是在本机、无线口、还是动捕主机之后。"""
    gw = ""
    for line in _run("ip route").splitlines():
        if line.startswith("default"):
            gw = line.split()[2]
            break
    hops = {"loopback": ping_stats("127.0.0.1", 40)}
    if gw:
        hops["gateway(%s)" % gw] = ping_stats(gw, 60)
    hops["vicon(%s)" % vicon_host] = ping_stats(vicon_host, 60)
    return hops


# --------------------------------------------------------------- UDP drops

def _udp_counters():
    out = _run("nstat -az UdpInDatagrams UdpRcvbufErrors")
    got = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] in ("UdpInDatagrams", "UdpRcvbufErrors"):
            got[parts[0]] = int(parts[1])
    return got


# --------------------------------------------------------------- Vicon stream

def measure_stream(topic, seconds):
    """按 QA_record 的方法测：源时钟 dt、真实丢帧、诚实传输延迟。

    关键点仍然是那条：微分和年龄要用【采集时刻】，不是【到达时刻】。到达打戳
    会把一条飞了 180 ms 的样本报成 0 ms 新鲜，安全判据就形同虚设。
    """
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node

    samples = []

    class Probe(Node):
        def __init__(self):
            super().__init__("vicon_link_check")
            self.create_subscription(PoseStamped, topic, self._cb, 300)

        def _cb(self, msg):
            samples.append((
                time.time(),
                msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec))

    owned = not rclpy.ok()
    if owned:
        rclpy.init()
    node = Probe()
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.002)
    node.destroy_node()
    if owned:
        rclpy.shutdown()

    if len(samples) < 20:
        # "一条都没收到"和"链路很差"是两种【完全不同】的故障，判成同一个
        # FAIL 会把人推去修网络，而真正的原因往往只是这个 shell 的
        # ROS_DOMAIN_ID 里根本没有 listener 在跑。
        return {"nodata": True,
                "topic": topic,
                "count": len(samples),
                "domain": os.environ.get("ROS_DOMAIN_ID", "0（未设置）")}

    arr = np.array(samples)
    arrival, source = arr[:, 0], arr[:, 1]
    if np.all(source <= 0):
        return {"error": "%s 的 header.stamp 全是 0，无法做时间分析。"
                         "vrpn_listener 需要修复（twist/accel 曾有此问题）" % topic}

    d_src = np.diff(source)
    nominal = float(np.median(d_src))
    lost = int(round(np.clip(d_src / nominal - 1.0, 0, None).sum()))

    # 诚实年龄 = 到达 - (源时刻 + 最小时钟偏置)。最小偏置那条是延迟最低的样本，
    # 最接近纯钟差；取均值会把常态延迟算进偏置，于是所有样本都被报成偏新鲜。
    offset = arrival - source
    age = offset - offset.min()

    ticks = np.arange(arrival[0], arrival[-1], 1.0 / CONTROL_HZ)
    idx = np.searchsorted(arrival, ticks, side="right") - 1
    keep = idx >= 0
    age_at_tick = ticks[keep] - (source[idx[keep]] + offset.min())

    return {
        "topic": topic,
        "messages": len(samples),
        "rate_hz": len(samples) / seconds,
        "source_dt_ms": {"median": nominal * 1e3, "std": float(d_src.std()) * 1e3,
                         "max": float(d_src.max()) * 1e3},
        "clock_offset_sec": float(offset.min()),
        "real_frame_loss_pct": 100.0 * lost / (len(samples) + lost),
        "latency_ms": {
            "mean": float(age.mean()) * 1e3,
            "p50": float(np.percentile(age, 50)) * 1e3,
            "p95": float(np.percentile(age, 95)) * 1e3,
            "p99": float(np.percentile(age, 99)) * 1e3,
            "max": float(age.max()) * 1e3,
        },
        "stall_pct": {
            "%.2f" % thr: 100.0 * float((age_at_tick > thr).mean())
            for thr in (0.05, 0.08, 0.10, 0.15, 0.20, 0.30)
        },
    }


# --------------------------------------------------------------- verdict

def verdict(stream):
    if stream.get("nodata"):
        # 独立结论：不是链路差，是这个 domain 里没有数据源。
        return "NODATA", [
            "%s 上只收到 %d 条消息" % (stream["topic"], stream["count"]),
            "当前 ROS_DOMAIN_ID=%s —— 先确认这个 domain 里有 vrpn_listener 在跑"
            % stream["domain"],
            "这【不是】链路质量结论：网络指标一条都没测到，别去改网络",
        ]
    if "error" in stream:
        return "FAIL", [stream["error"]]
    reasons = []
    stall = stream["stall_pct"]["%.2f" % DEFAULT_TIMEOUT_SEC]
    p95 = stream["latency_ms"]["p95"]
    loss = stream["real_frame_loss_pct"]

    level = "PASS"
    for name, value, key in (("停发力矩拍占比", stall, "stall_pct"),
                             ("延迟 p95", p95, "p95_ms"),
                             ("真实丢帧", loss, "loss_pct")):
        if value > WARN[key]:
            level = "FAIL"
            reasons.append("%s %.2f 超过 FAIL 线 %.2f" % (name, value, WARN[key]))
        elif value > PASS[key]:
            if level != "FAIL":
                level = "WARN"
            reasons.append("%s %.2f 超过 PASS 线 %.2f" % (name, value, PASS[key]))
    if level == "PASS":
        reasons.append("三项判据全部在 PASS 线内")
    return level, reasons


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.200.60", help="Vicon 主机 IP")
    ap.add_argument("--topic", default="/vrpn/arm_base/pose")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--json", default="", help="把完整结果写到这个文件")
    ap.add_argument("--skip-ping", action="store_true")
    args = ap.parse_args()

    print("=" * 68)
    print("动捕链路自检（只读：不起驱动、不发 /student/*、不碰串口）")
    print("=" * 68)

    report = {"timestamp": time.strftime("%Y-%m-%d %H:%M:%S")}

    wifi = wifi_status()
    report["wifi"] = wifi
    if wifi.get("band"):
        print("\n[WiFi] %s  %s  %d MHz (ch%s)  signal %s dBm  "
              "rx %s / tx %s Mbit/s  power_save=%s"
              % (wifi.get("ssid", "?"), wifi["band"], wifi.get("freq_mhz", 0),
                 wifi.get("channel", "?"), wifi.get("signal_dbm"),
                 wifi.get("rx_mbit"), wifi.get("tx_mbit"), wifi.get("power_save")))
        print("       同频 BSSID %d 个 / 共扫到 %d 个 —— 同频数越大，空口等待越久"
              % (wifi.get("cochannel_bssids", -1), wifi.get("bssids_seen", -1)))
    else:
        print("\n[WiFi] 未连无线（有线？那很好）")

    if not args.skip_ping:
        print("\n[逐跳延迟] 延迟出现在哪一跳")
        hops = hop_localisation(args.host)
        report["hops"] = hops
        print("  %-24s %8s %8s %8s %8s" % ("target", "min", "avg", "max", "loss%"))
        for name, s in hops.items():
            if s:
                print("  %-24s %8.2f %8.2f %8.2f %8.1f"
                      % (name, s["min_ms"], s["avg_ms"], s["max_ms"], s["loss_pct"]))
        gw = [v for k, v in hops.items() if k.startswith("gateway")]
        if gw and gw[0] and gw[0]["avg_ms"] > 20.0:
            print("  ⚠ 网关（一跳无线）就已经 %.0f ms —— 延迟在【空口】，"
                  "不在动捕主机、也不在本机" % gw[0]["avg_ms"])

    print("\n[动捕流] 订阅 %s %.0f 秒 …" % (args.topic, args.seconds))
    before = _udp_counters()
    stream = measure_stream(args.topic, args.seconds)
    after = _udp_counters()
    report["stream"] = stream
    report["udp_rcvbuf_errors"] = (after.get("UdpRcvbufErrors", 0)
                                   - before.get("UdpRcvbufErrors", 0))

    if stream.get("nodata"):
        print("  ❔ %s 上只收到 %d 条消息（ROS_DOMAIN_ID=%s）"
              % (stream["topic"], stream["count"], stream["domain"]))
    elif "error" in stream:
        print("  ✗ " + stream["error"])
    else:
        lat = stream["latency_ms"]
        print("  速率 %.1f Hz，源时钟 dt 中位 %.2f ms（std %.2f）"
              % (stream["rate_hz"], stream["source_dt_ms"]["median"],
                 stream["source_dt_ms"]["std"]))
        print("  时钟偏置 %.1f s（%.3f h）—— 源时钟与本机的常量差，必须在线估计"
              % (stream["clock_offset_sec"], stream["clock_offset_sec"] / 3600))
        print("  真实丢帧 %.2f%%   （基线 %.2f%%）"
              % (stream["real_frame_loss_pct"], BASELINE["loss_pct"]))
        print("  诚实传输延迟 ms: mean %.1f  p50 %.1f  p95 %.1f  p99 %.1f  max %.1f"
              % (lat["mean"], lat["p50"], lat["p95"], lat["p99"], lat["max"]))
        print("                   （基线 p95 %.1f / max %.1f）"
              % (BASELINE["p95_ms"], BASELINE["max_ms"]))
        print("  内核 UDP 接收缓冲丢包：%d 条" % report["udp_rcvbuf_errors"])
        print("\n  200 Hz 回放：各 base_state_timeout 下会【停发力矩】的拍占比")
        for thr, pct in stream["stall_pct"].items():
            mark = "  <- 当前配置" if abs(float(thr) - DEFAULT_TIMEOUT_SEC) < 1e-9 else ""
            print("    %s s -> %6.2f%%%s" % (thr, pct, mark))

    level, reasons = verdict(stream)
    report["verdict"] = {"level": level, "reasons": reasons}
    icon = {"PASS": "✅", "WARN": "⚠️", "FAIL": "🔴", "NODATA": "❔"}[level]
    print("\n" + "=" * 68)
    print("%s  结论：%s" % (icon, level))
    for r in reasons:
        print("    - " + r)
    if level == "NODATA":
        print("\n  没有收到任何动捕消息，所以【没有】链路质量结论。要查的是：")
        print("    ① 这个 shell 的 ROS_DOMAIN_ID 里有没有 vrpn_listener 在跑")
        print("       ros2 node list | grep vrpn      # 空 = 没在跑")
        print("    ② listener 连的是不是同一台 Vicon 主机")
        print("    ③ Vicon Tracker 里这个刚体是不是在推流")
        print("  在排除以上三条之前，不要去动网络配置。")
    elif level != "PASS":
        print("\n  这条链路上跑出来的跟踪精度数据没有意义 —— 先修链路。")
        print("  按性价比：① 换 5 GHz  ② USB 千兆网卡接有线  ③ 自带 AP 组专网")
        print("  ④ Vicon Tracker 里只流 arm_base + arm_ee，关掉其余刚体")
    print("=" * 68)

    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=2, ensure_ascii=False),
                                   encoding="utf-8")
        print("完整结果已写入 %s" % args.json)
    # 退出码分开，便于上层脚本分流：0=PASS 1=WARN 2=FAIL 3=NODATA。
    # NODATA 单独一个码，就是为了不让"没有数据源"被当成"链路差"处理。
    return {"PASS": 0, "WARN": 1, "FAIL": 2, "NODATA": 3}[level]


if __name__ == "__main__":
    sys.exit(main())
