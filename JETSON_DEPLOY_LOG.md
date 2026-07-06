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
  尖峰密集时可能瞬时触发。→ **已于 07-06 下午关闭 RT 节流并持久化**，实施与复位方法见下文《2026-07-06（下午）工作站远程验收 + 关闭 RT 节流》。
- 复测时桌面会话未关闭；RT 优先级下影响很小，但极限复测应经 SSH 关 GUI 后进行。
- `latency_lower_obstacle_circle_qpoases.csv` 有 7994 样本（其余 ~1700），系首轮有效运行停止较晚，P95 统计不受影响。

## 2026-07-05 部署诊断（40W 基线时期）

- 40W 基线数据：`src/arm-platform/demo/data/jetson_40w_20260703/`（纯求解器 + ROS 端到端两套，
  结论见求解器 README §4.4/§4.5）。
- 诊断发现：当时 CPU 被 40W 档压在 1497.6 MHz、governor=schedutil 未锁频（空闲核趴在 729.6 MHz）、
  `ulimit -r`=0 无 RT 权限、桌面 gnome-shell/VS Code 抢占 CPU 明显。→ 促成 07-06 的四项调优。
- BLASFEO target 排查：确认为 A57 专项 target，排除"按 GENERIC 编译导致 HPIPM 慢"的假设。

---

## 2026-07-06（下午）工作站远程验收 + 关闭 RT 节流

从 windylab 工作站经 SSH 远程核实（免密已配好，工作站可直接 `ssh nvidia@172.16.20.135`）：

- SSH **登录**会话 `ulimit -r`=95 ✅（PAM limits 正常继承，经 SSH 跑测试不再需要 prlimit 补丁）
- CPU 锁频保持 1984000 ✅；`jetson-clocks.service` enabled ✅；tj 51.5°C
- `sched_rt_runtime_us`=**950000** → **坐实** §07-06 复测中 HPIPM 数据无效的 RT 节流判断

### 关闭 RT 节流（经用户批准，已实施并持久化）

front_wing_arc 的 qpOASES 占空比 ~90%，离内核默认 95% 节流线过近，故拆除该限制：

- 立即生效：`sudo sysctl -w kernel.sched_rt_runtime_us=-1`（已执行，核实读回 -1）
- 开机持久化：新建 `/etc/sysctl.d/99-rt-no-throttle.conf`，内容一行 `kernel.sched_rt_runtime_us=-1`

**复位方法**（如后续需要恢复内核默认的 95% RT 上限）：

```bash
sudo rm /etc/sysctl.d/99-rt-no-throttle.conf   # 删掉持久化配置
sudo sysctl -w kernel.sched_rt_runtime_us=950000   # 立即恢复默认值(或直接重启)
```

**关闭后的风险与自救**：若 SCHED_FIFO 进程死循环，其绑定的 4-7 核会被占死；
0-3 核不受影响，SSH 进去 `kill` 该进程即可恢复。切勿把 RT 进程绑到全部 8 核。

## 2026-07-06（午后）关节流确认 + HPIPM 归因修正 + 节点热路径优化

全程从工作站免密 SSH 远程执行,测试脚本 `/tmp/run_latency.sh`(Jetson 上,零反馈源 + 45s 采样)。

### 确认测试与归因修正

- qpOASES front_wing_arc(部署默认参数):compute P95 16.90ms,**max 从 67ms 收敛到 18.9ms**
  ——RT 节流关闭对长尾确有实效。
- **HPIPM 异常归因修正**:此前"RT 节流伪影"判断**错误**。真因是部署调参引入的
  `acados_qp_solver_iter_max=500` 默认值:HPIPM 在本问题上每步跑满迭代上限
  (iter=500→173ms、iter=50→31.6ms,耗时恒定=铁证;31.6 = 40W 的 41.3 ÷ 1.325 主频比,完全吻合)。
  qpOASES 热启动早收敛不受影响。**用 HPIPM 须显式 `-p acados_qp_solver_iter_max:=50`**。
  注意:改 iter_max 是结构量,触发 acados 全量重编,Jetson 上约 5 分钟。

### 热路径优化(cProfile 剖析驱动)

- 剖析发现:①避障诊断在 `_plan_feasible`/`_make_info` 重复计算(~25% tick);②marker 每 tick
  重建 2500 点尾迹(~40% tick,在延迟打点外但挤占周期)。
- 修复:诊断去重(mpc_pinocchio_ik.py,语义等价);demo 节点新增 `marker_publish_hz`(默认 10Hz)。
- **效果(compute P95)**:circle 13.29→9.33ms,lower 14.06→8.69ms,front_wing_arc 17.86→12.34ms;
  front_wing_arc 有效控制频率 **34.7→49.7Hz**(优化前 tick 超 20ms,节点一直欠频饱和运行,
  此前基线的 e2e 是饱和态测得,feedback_age 假性 0.4ms;详见求解器 README §4.7)。
- 数据:`src/arm-platform/demo/data/jetson_maxn_super_opt_20260706/`。
- 注意:优化改动先经 scp 直推 Jetson 验证,Jetson 工作区这两个文件是脏的;下次 git pull 前先
  `git checkout -- src/arm-platform/demo/mpc_pinocchio_ik.py src/arm-platform/demo/move_ll100_lh420_nmpc_demo.py`。

## 历史性能基线数据（README 第4章演进留档）

以下是求解器 README 第 4 章曾经收录、后因文档改为纯"使用说明"口径而移出的历史基线数据、对比表与踩坑归因。作为演进记录留档于此(日志允许对比与时间线);README 只保留 MAXN_SUPER 定稿口径下的现状陈述。

### 40W 基线：纯求解器测试结果（2026-07-03）

测试条件:Jetson `nvpmodel` **40W** 模式(mode id `4`)、部署避障几何(中心线胶囊 `capsule_points` + 斥力,前桨/机身共约 50 硬约束)、`periods=1.0`、`repeats=3`、稳态 RMS 口径。"最大求解"= 三次重复中的最坏单步(含启动/重建尖峰)。原始数据在 `src/arm-platform/demo/data/jetson_40w_20260703/benchmark_nmpc_*.csv`。

**lower_obstacle_circle**
| 求解器配置 | 平均求解(ms) | P95求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 4.92 | 12.00 | 22.42 | 0.69 | ✅ |
| acados SQP_RTI + HPIPM | 15.98 | 35.65 | 84.99 | 0.69 | ✅ |
| GRAMPC | 19.56 | 20.30 | 21.65 | 0.59 | ✅ |

**circle**
| 求解器配置 | 平均求解(ms) | P95求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 3.98 | 4.27 | 13.60 | 2.41 | ✅ |
| acados SQP_RTI + HPIPM | 14.10 | 14.39 | 85.45 | 2.41 | ✅ |
| GRAMPC | 19.49 | 19.97 | 20.48 | 0.96 | ✅ |

**front_wing_arc**
| 求解器配置 | 平均求解(ms) | P95求解(ms) | 最大求解(ms) | 稳态RMS(mm) | safe |
|---|---:|---:|---:|---:|:---:|
| acados SQP_RTI + qpOASES | 5.30 | 12.76 | 20.48 | 2.37 | ✅ |
| acados SQP_RTI + HPIPM | 17.76 | 36.04 | 85.80 | 2.37 | ✅ |
| GRAMPC | 19.68 | 20.18 | 21.37 | 1.67 | ✅ |

结论(40W 纯求解器):三者全 safe;按优先级(安全>求解时间>精度)首选 acados qpOASES。qpOASES 在三场景 P95 均低于 13ms,实时裕度最好;HPIPM 在 lower/front 场景有明显启动/重建尖峰(P95 35ms+、最大约 85ms);GRAMPC 的 RMS 往往最好,但 P95 接近或略高于 20ms,适合作为精度/鲁棒性参考而非默认最快后端。

备注 `avoidance_sweep`:该场景每周期短暂穿近奇异。新中心线胶囊几何下 qpOASES/HPIPM 已能保持 σ_min≈2.6e-3 过门槛(safe),但 GRAMPC 在此场景擦障(viol≈14mm)+ 近奇异 → GRAMPC 的增广拉格朗日是软约束、苛刻场景可被突破,而 acados 是真硬约束。对"绝不碰撞"要求,acados 更可靠。

### 40W 基线：接入 ROS 端到端延迟（2026-07-03）

测试条件:Jetson **40W** 模式、RMW=Fast-DDS、`taskset -c 4-7` 固定 CPU、默认调度(`nice=0`)、独立 `ROS_DOMAIN_ID=81..89`、50Hz 反馈、跳过 5s 暖机、每格 1503-1520 个样本;三后端各在三场景跑一次。原始数据在 `src/arm-platform/demo/data/jetson_40w_20260703/latency_*.csv`,汇总见 `latency_summary_40w.csv`。

**lower_obstacle_circle**
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 17.80 | 18.24 | 18.26 | 18.74 |
| acados SQP_RTI + HPIPM | 22.50 | 22.85 | 22.98 | 23.34 |
| GRAMPC | 24.28 | 24.73 | 24.73 | 25.20 |

**circle**
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 16.74 | 17.21 | 17.20 | 17.70 |
| acados SQP_RTI + HPIPM | 28.44 | 29.03 | 28.93 | 29.54 |
| GRAMPC | 24.27 | 24.69 | 24.72 | 25.17 |

**front_wing_arc**
| 求解器配置 | compute 均值(ms) | compute P95(ms) | end_to_end 均值(ms) | end_to_end P95(ms) |
|---|---:|---:|---:|---:|
| acados SQP_RTI + qpOASES | 24.37 | 25.07 | 24.84 | 25.56 |
| acados SQP_RTI + HPIPM | 40.72 | 41.29 | 41.22 | 41.79 |
| GRAMPC | 24.38 | 24.89 | 24.84 | 25.38 |

结论(40W 接入 ROS):只有 lower/circle 的 qpOASES 保持在 20ms 周期内(e2e P95 18.74/17.70ms);front_wing_arc + qpOASES 端到端 P95 25.56ms;HPIPM 上机后一致最慢(front_wing_arc e2e P95 41.79ms);GRAMPC 延迟稳定但约 25ms。此批 e2e 于饱和态测得(节点欠频满负荷、feedback_age 假性偏小),不能与 MAXN_SUPER 定稿批次的 e2e 直接横比。

### 40W → MAXN_SUPER 对比（2026-07-06 复测）

功耗档切 MAXN_SUPER(CPU 1497.6→1984 MHz,+32%)、`jetson_clocks` 锁频、NMPC 节点以 SCHED_FIFO 80 运行(`chrt -f 80 taskset -c 4-7`);其余口径与 40W ROS 批次一致。原始数据在 `src/arm-platform/demo/data/jetson_maxn_super_20260706/`,汇总见 `latency_summary_maxn_super.csv`。

**end_to_end P95(ms),40W → MAXN_SUPER:**

| 场景 | qpOASES | GRAMPC | HPIPM |
|---|---|---|---|
| circle | 17.70 → **13.70** ✅ | 25.17 → 19.90 | 29.54 → (无效,见下) |
| lower_obstacle_circle | 18.74 → **14.54** ✅ | 25.20 → 20.13 | 23.34 → (无效,见下) |
| front_wing_arc | 25.56 → **18.28** ✅ | 25.37 → 20.06 | 41.79 → (无效,见下) |

- qpOASES 三场景全部进入 20ms 周期,提速比 1.29~1.40× 与主频提升(×1.325)吻合——纯频率收益,零代码改动。
- GRAMPC 压在 20ms 线上(19.9~20.1ms),50Hz 下仍不作默认后端,≤40Hz 可用。

### HPIPM 数据无效的归因修正（2026-07-06 下午定论）

最初判断为 **RT 节流伪影**,后经复测**推翻**:真因是部署调参把 QP 迭代上限默认设为 `acados_qp_solver_iter_max=500`,而 HPIPM 在本问题上**每步都跑满迭代上限**(iter=500→约 173ms、iter=50→31.6ms,耗时呈"恒定值"特征即为铁证;31.6ms 恰等于 40W 的 41.3ms ÷ 主频比 1.325)。qpOASES 靠热启动早收敛,不受该默认值影响。→ 若用 HPIPM,须显式传 `-p acados_qp_solver_iter_max:=50`。原始数据见 `data/jetson_maxn_super_opt_20260706/latency_front_wing_arc_hpipm_iter{50,500}.csv`。

**RT 节流**:虽非 HPIPM 异常主因,但已在 Jetson 上永久关闭(`sched_rt_runtime_us=-1`)。关闭后 qpOASES 的最坏单步从 67ms 收敛到 18.9ms——节流确实影响长尾。

### 节点热路径优化：cProfile 剖析 + 优化前→后（2026-07-06）

cProfile 剖析(front_wing_arc, 720 tick)发现两处大头:①`step()` 里避障诊断被 `_plan_feasible` 和 `_make_info` **各算一遍**(全时域 FK+数百次符号距离,约占 tick 的 1/4);②`_publish_markers` 每 tick 重建整套 MarkerArray(含最多 2500 点尾迹,约占 40%,虽在延迟打点之后、不计入 compute,但挤占 20ms 周期)。两项修复:诊断算一遍两处复用(`mpc_pinocchio_ik.py` 的 `step/solve`,语义等价);新增 `marker_publish_hz` 参数(默认 10Hz,≤0 关闭,≥50 恢复逐 tick)。

**优化前→后 qpOASES compute P95(ms):**

| 场景 | 优化前 | 优化后 | 有效控制频率 |
|---|---:|---:|---|
| circle | 13.29 | **9.33** | 49.7Hz |
| lower_obstacle_circle | 14.06 | **8.69** | 49.7Hz |
| front_wing_arc | 17.86 | **12.34** | **34.7 → 49.7Hz** |

front_wing_arc 优化前 tick 超 20ms、节点一直欠频饱和运行,此前基线的 e2e 是饱和态测得(feedback_age 假性 0.4ms);优化后三后端完整对照(即 README 现 §4.5 三张表)数据在 `data/jetson_maxn_super_opt_20260706/latency_summary_opt.csv`。

## 遗留事项 / 下一步

- [x] 本机（windylab 工作站）→ Jetson 免密 SSH：用户已 `ssh-copy-id` 授权（2026-07-06），
      工作站可直接 `ssh nvidia@172.16.20.135` 免密操作。
- [x] 求解器 README §4.6 已补 MAXN_SUPER 基线小节（2026-07-06）。
- [x] RT 节流已关闭并持久化（2026-07-06，复位方法见上文）。
- [ ] 真机接入后复测端到端延迟（当前均为零反馈源回环，不含真实传感器/总线延迟）。
