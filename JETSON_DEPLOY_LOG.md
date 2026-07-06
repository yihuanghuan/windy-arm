# Jetson 部署日志（LL100-LH420 NMPC）

按时间倒序记录 Jetson Orin NX 上 NMPC 求解器部署与性能优化的**已做变更、实测结论、遗留事项**。
求解器选型与 benchmark 方法学见 [LL100_LH420_NMPC_Solver_README.md](LL100_LH420_NMPC_Solver_README.md)；
部署步骤见总 [README.md](README.md) 第六节。本文档只记"这台机器上实际发生了什么"。

## 硬件与系统基线

- 板卡：NVIDIA Jetson Orin NX 16GB（Engineering Reference Developer Kit **Super**，P-Number p3767-0000）
- 系统：JetPack 6.2 / L4T 36.4.3 / Ubuntu 22.04，内核 5.15.148-tegra（PREEMPT，非 PREEMPT_RT）
- CPU：8× Cortex-A78AE，物理上限 1984 MHz
- 用户：`nvidia`；工作空间 `/home/nvidia/code/windylab_ws`；acados 在 `/home/nvidia/code/acados`
- acados/BLASFEO 编译确认：`BLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57`、`CMAKE_BUILD_TYPE=Release`（2026-07-06 从 CMakeCache 核实，**不是** GENERIC）
- 内网 IP：`172.16.20.135`（SSH 端口默认 22）

---

## 2026-07-06 系统调优 + MAXN_SUPER 复测

### 做了什么（均已生效并验证）

1. **功耗档 40W(4) → MAXN_SUPER(0)**：`sudo nvpmodel -m 0`，CPU 上限 1497.6 → 1984 MHz（+32%）。
2. **锁频**：`sudo jetson_clocks`（CPU 全核 Min=Max=1984000，EMC/GPU 拉满）+ `jetson_clocks --fan` 风扇全速。
3. **开机自动锁频**：新建 `/etc/systemd/system/jetson-clocks.service`（oneshot，multi-user.target 后执行 `jetson_clocks`），已 enable。
4. **RT 调度权限**：`/etc/security/limits.conf` 追加 `nvidia - rtprio 95` 与 `nvidia - memlock unlimited`（原文件备份为 `limits.conf.bak.20260706-093505`）。仅对**新登录会话**生效；非登录环境（如桌面 VS Code 里的 Codex）需在**同一条命令内**先 `sudo prlimit --rtprio=95:95 --pid $$` 再 `chrt`。

### 复测结果（数据：`src/arm-platform/demo/data/jetson_maxn_super_20260706/`）

测试口径与 40W 批次一致：50Hz 零反馈源、5s 暖机、≥1500 样本、`taskset -c 4-7`；
本轮新增 `chrt -f 80`（SCHED_FIFO 实时优先级）；桌面会话未关闭。

**end_to_end P95（ms），40W → MAXN_SUPER：**

| 场景 | qpOASES | GRAMPC | HPIPM |
|---|---|---|---|
| circle | 17.70 → **13.70** ✅ | 25.17 → 19.90 | 29.54 → (无效) |
| lower_obstacle_circle | 18.74 → **14.54** ✅ | 25.20 → 20.13 | 23.34 → (无效) |
| front_wing_arc | 25.56 → **18.28** ✅ | 25.37 → 20.06 | 41.79 → (无效) |

- **qpOASES 三场景全部进入 20ms 周期**，提速比 1.29~1.40×，与主频提升(×1.325)吻合——收益就是纯 CPU 频率，未改任何代码。
- **GRAMPC** 压在 20ms 线上（19.9~20.1ms），50Hz 下仍不作默认后端，≤40Hz 可用。
- **HPIPM 本轮数据无效，不代表求解器性能**：compute P95 飙到 115~226ms（含 18.5s 极端样本），比 40W 还慢 5~10 倍。判断为 **Linux RT 节流伪影**：内核默认限制 SCHED_FIFO 任务每秒占用 ≤95% CPU（`sched_rt_runtime_us=950000`），HPIPM 占空比本就 >100%（40W 时 compute 22~41ms > 20ms 周期），挂 `chrt -f 80` 后触顶被内核周期性挂起，延迟雪崩。40W 批次没用 RT 优先级故未触发。HPIPM 反正已非候选默认后端，不再为它重测。
- 温度全程 tj ≤57.2°C，无热降频（>85°C 样本数 0），MAXN_SUPER 散热裕度充足。

### 部署推荐（当前定稿）

```
MAXN_SUPER + jetson_clocks（已开机自启）
+ acados qpOASES（FULL_CONDENSING_QPOASES）
+ chrt -f 80 taskset -c 4-7 启动 NMPC 节点
```

### 风险与注意

- **RT 节流隐患**：front_wing_arc 的 qpOASES compute ~17.9ms/20ms 周期 = 90% 占空比，离 95% 节流线不远，
  尖峰密集时可能瞬时触发。备选缓解：`sudo sysctl -w kernel.sched_rt_runtime_us=-1` 关 RT 节流
  （机器人控制器常规做法；风险是 RT 进程死循环会占死 4-7 核，0-3 核仍可救，尚未实施）。
- 复测时桌面会话未关闭；RT 优先级下影响很小，但极限复测应经 SSH 关 GUI 后进行。
- `latency_lower_obstacle_circle_qpoases.csv` 有 7994 样本（其余 ~1700），系首轮有效运行停止较晚，P95 统计不受影响。

## 2026-07-05 部署诊断（40W 基线时期）

- 40W 基线数据：`src/arm-platform/demo/data/jetson_40w_20260703/`（纯求解器 + ROS 端到端两套，
  结论见求解器 README §4.4/§4.5）。
- 诊断发现：当时 CPU 被 40W 档压在 1497.6 MHz、governor=schedutil 未锁频（空闲核趴在 729.6 MHz）、
  `ulimit -r`=0 无 RT 权限、桌面 gnome-shell/VS Code 抢占 CPU 明显。→ 促成 07-06 的四项调优。
- BLASFEO target 排查：确认为 A57 专项 target，排除"按 GENERIC 编译导致 HPIPM 慢"的假设。

---

## 2026-07-06（下午）工作站远程验收

从 windylab 工作站经 SSH（密码认证）远程核实：

- SSH **登录**会话 `ulimit -r`=95 ✅（PAM limits 正常继承，经 SSH 跑测试不再需要 prlimit 补丁）
- CPU 锁频保持 1984000 ✅；`jetson-clocks.service` enabled ✅；tj 51.5°C
- `sched_rt_runtime_us`=**950000** → **坐实** §07-06 复测中 HPIPM 数据无效的 RT 节流判断

## 遗留事项 / 下一步

- [ ] 本机（windylab 工作站）→ Jetson 免密 SSH：密码认证已通，公钥（注释 `windylab-ll100-lh420`）
      待用户自行 `ssh-copy-id nvidia@172.16.20.135` 授权持久免密。
- [x] 求解器 README §4.6 已补 MAXN_SUPER 基线小节（2026-07-06）。
- [ ] 决定是否关闭 RT 节流（`sched_rt_runtime_us=-1`），或 front_wing_arc 降为普通优先级运行。
- [ ] 真机接入后复测端到端延迟（当前均为零反馈源回环，不含真实传感器/总线延迟）。
