# Joint6 Host-side Software Root-cause Audit

日期：2026-09-17。分支 `lagrange-smc`，HEAD `b496a4f11152539da3a399a4fbe5aaddd0117df8`。
本轮只分析源码、Git 历史和已有采集文件；没有打开串口、创建真实 ROS 发布器或运行硬件程序。未修改 joint mapping、URDF、标定参数或生产代码。

## 1. 最终判定：B. Host software decoding/mapping excluded with high confidence

**主机侧 ROS/Python/C++ 解码和映射不能解释 joint6 全零。零值已经存在于控制板发送到 USB-UART 的原始 FE55 payload 中。**

该判定针对保存的这次 recapture 和审查的 gamma6 接收路径，不等于证明整个主机软件在任意配置、任意坏包下均无缺陷，也不代表已经确定控制板内部原因。原始采集文件是主机 `os.read` 得到、在解码前写入的字节，未使用外部逻辑分析仪。文件与原 read-chunk 日志逐字节一致。

| B 的必要证据 | 结果 |
|---|---|
| 独立解析原始文件，slot5 全零 | 四类反馈各 3214 帧，全部成立 |
| 直接检查 slot5 payload 字节 | float 四字节、temperature 两字节全部为 00 |
| 独立 parser 与现有 parser 一致 | 12856 个目标帧，全部 7 槽，6 种分块方式完全一致 |
| 非零 sentinel 穿过 slot5 到 joint6 | 4 类通过；实际 FeedbackNode.poll 离线假接收器/假发布器测试通过 |
| C++ joint6=id5 | YAML、构造、getter、状态更新链一致 |
| 无 joint6 mask 或 slot5→slot6 重排 | 已审查；截取前六保留 index5，只移除 index6 |
| slot6 与已知 ghost 一致 | position/velocity/temp 与历史记录的有效精度完全一致；历史未给 current 精确值 |

## 2. 独立原始字节解析

脚本：[audit_raw_independent.py](j6_software/audit_raw_independent.py)。只使用 Python 标准库；不导入任何项目模块。
输入：[recapture/capture.rx.bin](recapture/capture.rx.bin)，396532 字节。
SHA-256：`8d1649717b757d5e215615c72a13f8277724acf2fa3e640e5bed7bcf1b959411`。

从每个 `FE 55` 候选开始，读取 cmd、len、payload、sum、add；校验覆盖 header+cmd+len+payload：sum 是逐字节累加模 256，add 是各步 sum 的累计模 256。仅合法长度目标帧进入解包：0x20/21/22 严格 `struct.unpack("<7f", payload)`；0x24 严格 `struct.unpack("<7H", payload)`。

有效帧 13241：0x20/21/22/24 各 3214，0x01 为 321，0x23/25 各 32。checksum error=0，目标帧 malformed length=0，尾部残帧=0。文件开头跳过 1 个非帧字节，这是抓取起点对齐，不计作合法帧 checksum 失败。

### 2.1 直接 byte 证据（每行覆盖全部 3214 帧）

| cmd | payload 长度 | slot5 原始 bytes | slot5 数值 | slot6 原始 bytes | slot6 数值 |
|---|---:|---|---:|---|---:|
| 0x20 position | 28 | `00 00 00 00` | +0.0 | `75 bd d0 c0` | -6.52312707901001 |
| 0x21 velocity | 28 | `00 00 00 00` | +0.0 | `fc c8 df 41` | 27.97313690185547 |
| 0x22 current | 28 | `00 00 00 00` | +0.0 | `b4 25 5b 40` | 3.4241762161254883 |
| 0x24 temperature | 14 | `00 00` | 0 | `01 00` | 1 |

float 的检查窗口是 `payload[20:24]`、`payload[24:28]`；temperature 是 `[10:12]`、`[12:14]`。这些 bytes 不是浮点显示舍入后的零，也不是 Python 补齐的零。

### 2.2 全部 7 槽 min … max

position 单位 rad，velocity 单位 rad/s；current 保留协议原值，不在这里断言电流/力矩单位；temperature 保留 uint16 原值。

| raw slot | position | velocity | current | temperature |
|---:|---|---|---|---|
| 0 | 0.00629425048828 … 0.00629425048828 | -0.00488471984863 … -0.00488471984863 | -0.348718643188 … 0.088888168335 | 28 … 28 |
| 1 | 1.3254365921 … 1.3254365921 | -0.00488471984863 … -0.00488471984863 | -2.37265014648 … -2.07179450989 | 28 … 28 |
| 2 | -1.34947013855 … -1.34947013855 | -0.00488471984863 … -0.00488471984863 | 0.594871520996 … 0.977777481079 | 28 … 28 |
| 3 | -0.0017261505127 … -0.000959396362305 | -0.158729553223 … 0.134311676025 | -0.0307693481445 … 0.0161170959473 | 31 … 32 |
| 4 | -0.0028772354126 … -0.0024938583374 | -0.158729553223 … 0.109889984131 | -0.0102562904358 … 0.0307693481445 | 32 … 32 |
| 5 | 0 … 0 | 0 … 0 | 0 … 0 | 0 … 0 |
| 6 | -6.52312707901 … -6.52312707901 | 27.9731369019 … 27.9731369019 | 3.42417621613 … 3.42417621613 | 1 … 1 |

完整每槽 byte 直方图：[independent_summary.json](j6_software/independent_summary.json)。每个目标帧的 offset、frame hex、payload hex、7 槽 bytes 和解码值：[independent_frames.json](j6_software/independent_frames.json)。

附录给出 20 个随机分层抽样的完整帧（种子 20260917，流分为 20 个等长 byte 区段，每段一个；四类各 5 帧）。时间由原 read-chunk 日志对齐到帧最后一个字节所在 read，属于主机接收时刻，非固件采样时刻。

## 3. 现有 Python / ROS 接收链审查

原始证据链：`Receiver.read → os.read bytes → capture.write(chunk) → Decoder.feed → raw_frames(7) → values[:6] → JointMapping → JointState`。
`live_audit.py` 第 94–102 行在 decode 之前保存 chunk；本轮另验证 chunks.jsonl 拼接与 binary 完全相同。不存在先解析再重新编码保存“原始”文件的过程。

源码均在 `src/real_arm_readonly/real_arm_readonly/`：

| 文件/位置 | 审查结果 |
|---|---|
| protocol.py:6–8,20–63 | NAMES 正好 joint1..6；unpack 得到 7 项；raw_frames 保留 7 项；统一 `values[:6]` 包括 slot5。有限值检查失败会丢整帧，不补零。无 `[:5]` 后补零。 |
| protocol.py:66–84 | SIGNS 长度 6，与 values 六项 zip；后三轴取负。位置另作 remainder/连续解缠；无 clamp/mask。 |
| receiver.py:25–52 | read 原样返回 os.read；EAGAIN 返回空 bytes，不造数据；raw tty 设置禁用字符翻译、parity 处理、echo；不按槽改写。此轮未实例化 Receiver。 |
| feedback_node.py:40–70 | joint names 与六项 position 同序；velocity 六项同序。velocity 过龄时整数组省略，非第六项置零。effort 故意为空数组（单位未确认），没有发布伪造的六项零。temperature 未作为 JointState 发布。 |
| live_audit.py:94–150 | raw_ranges 使用全部7项；latest 使用同一 decoder 的前六项；position/velocity 同一映射。current/temp 留原协议符号，不换 channel。 |
| live_audit.py:116,165–170 | `raw_values[5] != 0` 只设置有效性观察标志；全窗口零会阻止进入 ROS 发布阶段，不改变任何数据。不能倒因为果，将观察到零解释为这个 gate 制造零。 |
| read_guard.py | 六项循环只检查、计数、返回停止读取理由；无反馈数组覆写。 |
| analyze_capture.py、测试及 setup/README | 已搜索 mapping/slicing/index/id/zip/joint6；未发现另一条 slot5 改零映射。离线报告对前六/全部七槽的用途有区分。 |

搜索记录：[python_mapping_search.txt](j6_software/python_mapping_search.txt)。结论是四种量的**索引一致**，不声称它们的符号处理与 ROS 字段相同：位置/速度应用旋向，current/temp 诊断保留 raw。

### 3.1 逐帧交叉验证

[crosscheck_and_sentinel.py](j6_software/crosscheck_and_sentinel.py) 在独立解析完成后才导入项目 Decoder。分别按 1、7、34、4096、396532 字节分块，以及原始 os.read 分块重放；每次全部 12856 目标帧的 cmd 和 7 值精确一致；feed 返回的六值也逐帧等于独立结果 `[0:6]`。所有运行 bad_checksum/bad_length/nonfinite=0。证明分包跨帧不会丢第六项。

### 3.2 Sentinel（纯内存，合法 FE55 + 独立 checksum 构造）

float payload `[1.125,2.25,3.375,4.5,5.625,6.75,7.875]`；temperature `[21,22,23,24,25,26,27]`。四类均得到七个不同 raw 值，six-channel 输出 slot5 不为零。

| cmd | raw slot5 | 映射/下游结果 |
|---|---:|---|
| 0x20 | 6.75 | joint6 position = remainder(-6.75,2π) = **-0.46681469282041377** |
| 0x21 | 6.75 | joint6 velocity = **-6.75** |
| 0x22 | 6.75 | 六轴 raw current 第六项 **6.75**；C++ 左手约定对应 -6.75（源码推导，未运行硬件 C++） |
| 0x24 | 26 | 六轴 temperature 第六项 **26** |

另外只改 slot6 为 -123.5，joint6 position 不变。调用真实 `FeedbackNode.poll` 方法，但使用纯内存假 receiver/pub/clock，不构造 Node；`os.open` 在该调用期间替换为抛异常函数。生成的 JointState names 正确，position[5] 非零、velocity[5]=-6.75；没有真实 ROS 发布。全部结果见 [crosscheck_sentinel.json](j6_software/crosscheck_sentinel.json)。

必须精确区分：旋向转换本身只是乘 -1；位置解缠是另一操作。数学上非零整圈角可能在首帧规范化成零，不能笼统声称“任何非零永不变零”。但本次 raw slot5 已是 +0.0，且 velocity/current/temp 也同时原始字节为零，不存在这个解释。全零序列从新建映射开始保持 ±0；若已有跨圈历史，连续位置可能保留圈数，这不适用于本次新建采集。

## 4. 与原 C++ 逐项比对

| physical joint | protocol id / raw slot | model | q/v/current 旋向 |
|---|---:|---|---:|
| joint1 | 0 | DM4340-48V | +1 |
| joint2 | 1 | DM4340-48V | +1 |
| joint3 | 2 | DM4340-48V | +1 |
| joint4 | 3 | RS05 | -1 |
| joint5 | 4 | RS05 | -1 |
| joint6 | 5 | RS05 | -1 |
| ghost（gamma6 不创建电机） | 6 | 无 physical joint | 不映射 |

证据（均为 `src/arm-platform/` 下源码）：

- `config/arm_config.yaml:48–79`：gamma6 明列 joint1..joint6/id0..id5。旧七轴 gamma 的 joint6 也为 id5；其 joint7=id6，不能搬到 gamma6 充当第六轴。
- `src/robotics/arm/a_l1.cc:26–44`：按 YAML 顺序以 joint.id 创建 DMMotor，按 joint.name AddMotor；旋向读 motor model。arm_config 内个别 coord_system 字段不是实际旋向来源。
- `config/motor_config.yaml:9,20`：DM 右手、RS05 左手。
- `src/robotics/protocol/protocol_v1.cc:149–178`：三种 float 顺序 memcpy 7 项到对应 index；温度同样按 i*2 memcpy 到 index i。
- `include/manipulator/robotics/protocol/protocol_v1.h:53–74`：四类 getter 直接 `[id]`。
- `src/robotics/motor/dm_motor.cc:34–60`：四类同 id；q/v/current 左手乘 -1、温度不变；q 连续解缠。没有 joint6 分支、slot5 mask、slot5→6 重排；command setter 的 clamp 不在反馈路径。
- `src/robotics/arm/abs_arm.cc:30–39,59–69`：初始化所有关节零，收到状态后按保存的 name 找 motor 并覆盖每一项，没有第六项例外。启动默认零可能影响无反馈时的 C++ 输出，但不能解释这份先于解析保存的完整合法 payload。

**健壮性问题单独记录：**原 C++ DecodeFrame 校验 checksum 后按 cmd 解包，没有先严格验证各 cmd payload length。若短包自带合法校验，读取未被新 payload 覆盖的 dataBuf 区域可能混入旧值/初始值。这应另行修复，但本次所有 0x20/21/22 均为 28 B、0x24 均为 14 B 且校验正确，没有触发条件；更不可能逆向改变 Python receiver 已保存的原始字节。此次未修改 C++，未运行其带硬件能力的程序。

## 5. 作者历史证据与局限

已执行用户指定的三条命令，完整输出保存：

- [git_log_gamma6.txt](j6_software/git_log_gamma6.txt)：`git log -S "a_l1_gamma6" --all -- src/arm-platform/config/arm_config.yaml`。
- [git_log_joint6.txt](j6_software/git_log_joint6.txt)：`git log -S "joint6" --all -- src/arm-platform`。
- [git_blame_arm_config.txt](j6_software/git_blame_arm_config.txt)：`git blame src/arm-platform/config/arm_config.yaml`。

`a_l1_gamma6` 及配置中的 ghost 注释由 **ef84a9db6730c935153df81900869b1f9a6d19bf** 引入，author date `2026-08-04T17:23:06-07:00`（北京时间 8 月 5 日）。原 gamma joint6=id5 可追溯到初始提交 `7563c11`。

1. `config/arm6_calibration.yaml:21` 确有 `temperature_c: [28,29,27,31,32,31]`；第六轴历史记录温度 **31°C**，并非约定为零。joint6 零偏为 0 是“无重力信号，未辨识”的标定处理，不是让 encoder=0 的代码；armature 借同型号 joint5 值也不是反馈映射。
2. `hw/fk_gravity_verify.py:42`：`N_ARM=6`。phase2/3/4/5 接收函数均检查六项数组后整体保存。phase3 跳过 joint6 的**重力辨识**，`--j6-hold-rad` 改的是扫动目标姿态，未将传感器第六项置零。phase4/5 内 `zeros(N_ARM)` 用于命令/缺测记录等上下文，不在本次 decoder 链。相关检索见 [historical_source_search.txt](j6_software/historical_source_search.txt)。这些脚本均未执行。
3. 找到 Git 保存的历史实测文档，当前路径已不存在，已从 blob 原样导出：[historical_PHASE1_FINDINGS.md](j6_software/historical_PHASE1_FINDINGS.md)，blob `b71bbe21e736803f8b720606599df1dff43f2978`。其第 339–354 行明确记载 joint6 **-1.4991 → 0.8077**，后续恢复测试第六轴实测 **+0.4983**，以及一次重启后 **0.0024**。这是作者明确标为实测的非零第六轴历史证据，不是仅有控制目标。原文最早在 `e5531d9` 收入仓库，`f77ec52` 移到 docs。
4. 同文第 53–62 行和 [historical_REFERENCE.md](j6_software/historical_REFERENCE.md)（blob `9bf86d7329f292b7b26e56af8dff055a1fe5c852`）第 446–454 行记录 slot6 position=-6.5231、velocity=27.9731、temperature=1，与本次原始结果按历史精度一致。历史未保留 slot6 current 精确值，故不声称对该字段做了历史逐 bit 比对。ghost 判断基于平台配置及这组历史反馈特征，并非仅因有7槽或温度1。
5. 搜索全部 Git 对象中的 bin/csv/json/jsonl/npz/npy/log：[historical_data_inventory.txt](j6_software/historical_data_inventory.txt)。可见历史 CSV 主要是求解器性能汇总，Vicon JSON 为外参/残差；未找到与上述实验对应、可重新按帧核验 joint6 的历史 UART binary。历史文档与 YAML 支持“作者曾记录非零第六轴”，不冒充历史原始抓包。它们也不能证明本机当前固件/供电状态与当时相同。

历史文档还记载 RS05 曾出现上电后不响应、后来恢复。该作者归因仅作为历史背景；本次没有重复该实验，不能据此认定当前就是同一原因，更没有执行重启或试动。

## 6. 下一层边界与安全记录

在排除本次 host 解码/映射之后，候选调查顺序才是：

`controller-board firmware/config → board ↔ joint6 internal communication → joint6 telemetry/power wiring → motor/encoder hardware`

这只是后续候选层级，当前证据不能区分，更不能断言“电机坏了”。没有建议直接拆硬件、切模式、复位、重启电源或发送请求帧；需要后续单独制定获准的检查步骤。

本轮执行记录：串口打开=0；串口写入=0；0x10–0x14 发送=0；新建 MotorControl publisher=0；新启动 controller/student/master/slave 节点=0。构造 sentinel 的 bytes 仅存在于内存，未发送；报告/JSON/文本的普通文件写入不属于 serial write。未开启 ROS 实机测试、RViz 修复或重力补偿。以上范围是**本轮审计行为**，不是未经进程/系统追踪而断言整台机器全局 syscall 或节点数量为零。

离线复现（仓库根目录）：

```bash
python3 docs/real_arm_audit_20260917/j6_software/audit_raw_independent.py docs/real_arm_audit_20260917/recapture/capture.rx.bin docs/real_arm_audit_20260917/j6_software
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/real_arm_readonly:$PYTHONPATH" python3 docs/real_arm_audit_20260917/j6_software/crosscheck_and_sentinel.py
```

第二条 Python 仅离线调用 poll，ROS 导入只提供消息类型，不 init rclpy、不建节点、不建真实 publisher、不实例化 Receiver。报告中 C++ 结论来自源码审查，并未伪称跑过原 C++ 真机程序。

## 附录：20 个时间分散的完整原始帧

```text
frame #646 offset=19328 cmd=0x20 len=28 host_read_t=0.770842437s
frame: fe 55 20 1c 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0 5f 7e
payload: 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0
slot0: bytes=00 40 ce 3b value=0.00629425048828125
slot1: bytes=e8 a7 a9 3f value=1.3254365921020508
slot2: bytes=70 bb ac bf value=-1.3494701385498047
slot3: bytes=00 00 b0 ba value=-0.0013427734375
slot4: bytes=00 70 23 bb value=-0.0024938583374023438
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=75 bd d0 c0 value=-6.52312707901001

frame #801 offset=23972 cmd=0x21 len=28 host_read_t=0.955612374s
frame: fe 55 21 1c 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 10 48 bc 00 10 48 bc 00 00 00 00 fc c8 df 41 dd 79
payload: 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 10 48 bc 00 10 48 bc 00 00 00 00 fc c8 df 41
slot0: bytes=00 10 a0 bb value=-0.0048847198486328125
slot1: bytes=00 10 a0 bb value=-0.0048847198486328125
slot2: bytes=00 10 a0 bb value=-0.0048847198486328125
slot3: bytes=00 10 48 bc value=-0.012210845947265625
slot4: bytes=00 10 48 bc value=-0.012210845947265625
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=fc c8 df 41 value=27.97313690185547

frame #1651 offset=49421 cmd=0x22 len=28 host_read_t=1.982773196s
frame: fe 55 22 1c 00 0d d2 bd 60 19 16 c0 00 cb 30 3f 00 08 90 bb 00 0a 28 3c 00 00 00 00 b4 25 5b 40 eb 04
payload: 00 0d d2 bd 60 19 16 c0 00 cb 30 3f 00 08 90 bb 00 0a 28 3c 00 00 00 00 b4 25 5b 40
slot0: bytes=00 0d d2 bd value=-0.10256385803222656
slot1: bytes=60 19 16 c0 value=-2.3452987670898438
slot2: bytes=00 cb 30 3f value=0.6905975341796875
slot3: bytes=00 08 90 bb value=-0.004395484924316406
slot4: bytes=00 0a 28 3c value=0.010256290435791016
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=b4 25 5b 40 value=3.4241762161254883

frame #2260 offset=67677 cmd=0x24 len=14 host_read_t=2.719741334s
frame: fe 55 24 0e 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00 1a 45
payload: 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00
slot0: bytes=1c 00 value=28
slot1: bytes=1c 00 value=28
slot2: bytes=1c 00 value=28
slot3: bytes=20 00 value=32
slot4: bytes=20 00 value=32
slot5: bytes=00 00 value=0
slot6: bytes=01 00 value=1

frame #3239 offset=96983 cmd=0x20 len=28 host_read_t=3.904763016s
frame: fe 55 20 1c 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0 5f 7e
payload: 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0
slot0: bytes=00 40 ce 3b value=0.00629425048828125
slot1: bytes=e8 a7 a9 3f value=1.3254365921020508
slot2: bytes=70 bb ac bf value=-1.3494701385498047
slot3: bytes=00 00 b0 ba value=-0.0013427734375
slot4: bytes=00 70 23 bb value=-0.0024938583374023438
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=75 bd d0 c0 value=-6.52312707901001

frame #3562 offset=106657 cmd=0x21 len=28 host_read_t=4.294553249s
frame: fe 55 21 1c 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 10 48 bc 00 10 48 3c 00 00 00 00 fc c8 df 41 5d f9
payload: 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 10 48 bc 00 10 48 3c 00 00 00 00 fc c8 df 41
slot0: bytes=00 10 a0 bb value=-0.0048847198486328125
slot1: bytes=00 10 a0 bb value=-0.0048847198486328125
slot2: bytes=00 10 a0 bb value=-0.0048847198486328125
slot3: bytes=00 10 48 bc value=-0.012210845947265625
slot4: bytes=00 10 48 3c value=0.012210845947265625
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=fc c8 df 41 value=27.97313690185547

frame #3979 offset=119151 cmd=0x22 len=28 host_read_t=4.801733136s
frame: fe 55 22 1c 00 0f ee bd d8 58 0d c0 40 cd 53 3f 00 08 90 3b 00 0a 28 3c 00 00 00 00 b4 25 5b 40 9c 51
payload: 00 0f ee bd d8 58 0d c0 40 cd 53 3f 00 08 90 3b 00 0a 28 3c 00 00 00 00 b4 25 5b 40
slot0: bytes=00 0f ee bd value=-0.11623954772949219
slot1: bytes=d8 58 0d c0 value=-2.208547592163086
slot2: bytes=40 cd 53 3f value=0.8273506164550781
slot3: bytes=00 08 90 3b value=0.004395484924316406
slot4: bytes=00 0a 28 3c value=0.010256290435791016
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=b4 25 5b 40 value=3.4241762161254883

frame #4788 offset=143373 cmd=0x24 len=14 host_read_t=5.778743907s
frame: fe 55 24 0e 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00 1a 45
payload: 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00
slot0: bytes=1c 00 value=28
slot1: bytes=1c 00 value=28
slot2: bytes=1c 00 value=28
slot3: bytes=20 00 value=32
slot4: bytes=20 00 value=32
slot5: bytes=00 00 value=0
slot6: bytes=01 00 value=1

frame #5691 offset=170401 cmd=0x20 len=28 host_read_t=6.869708094s
frame: fe 55 20 1c 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0 5f 7e
payload: 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0
slot0: bytes=00 40 ce 3b value=0.00629425048828125
slot1: bytes=e8 a7 a9 3f value=1.3254365921020508
slot2: bytes=70 bb ac bf value=-1.3494701385498047
slot3: bytes=00 00 b0 ba value=-0.0013427734375
slot4: bytes=00 70 23 bb value=-0.0024938583374023438
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=75 bd d0 c0 value=-6.52312707901001

frame #6540 offset=195843 cmd=0x21 len=28 host_read_t=7.896731056s
frame: fe 55 21 1c 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 10 48 3c 00 10 48 3c 00 00 00 00 fc c8 df 41 dd 79
payload: 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 10 48 3c 00 10 48 3c 00 00 00 00 fc c8 df 41
slot0: bytes=00 10 a0 bb value=-0.0048847198486328125
slot1: bytes=00 10 a0 bb value=-0.0048847198486328125
slot2: bytes=00 10 a0 bb value=-0.0048847198486328125
slot3: bytes=00 10 48 3c value=0.012210845947265625
slot4: bytes=00 10 48 3c value=0.012210845947265625
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=fc c8 df 41 value=27.97313690185547

frame #6970 offset=208710 cmd=0x22 len=28 host_read_t=8.415476999s
frame: fe 55 22 1c 00 10 e0 bb 10 d9 10 c0 e0 cb 3e 3f 00 0b b4 bc 00 10 c0 3a 00 00 00 00 b4 25 5b 40 16 99
payload: 00 10 e0 bb 10 d9 10 c0 e0 cb 3e 3f 00 0b b4 bc 00 10 c0 3a 00 00 00 00 b4 25 5b 40
slot0: bytes=00 10 e0 bb value=-0.0068378448486328125
slot1: bytes=10 d9 10 c0 value=-2.2632484436035156
slot2: bytes=e0 cb 3e 3f value=0.7452983856201172
slot3: bytes=00 0b b4 bc value=-0.021977901458740234
slot4: bytes=00 10 c0 3a value=0.0014653205871582031
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=b4 25 5b 40 value=3.4241762161254883

frame #7856 offset=235257 cmd=0x24 len=14 host_read_t=9.487640322s
frame: fe 55 24 0e 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00 1a 45
payload: 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00
slot0: bytes=1c 00 value=28
slot1: bytes=1c 00 value=28
slot2: bytes=1c 00 value=28
slot3: bytes=20 00 value=32
slot4: bytes=20 00 value=32
slot5: bytes=00 00 value=0
slot6: bytes=01 00 value=1

frame #8497 offset=254435 cmd=0x20 len=28 host_read_t=10.263835066s
frame: fe 55 20 1c 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0 5f 7e
payload: 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0
slot0: bytes=00 40 ce 3b value=0.00629425048828125
slot1: bytes=e8 a7 a9 3f value=1.3254365921020508
slot2: bytes=70 bb ac bf value=-1.3494701385498047
slot3: bytes=00 00 b0 ba value=-0.0013427734375
slot4: bytes=00 70 23 bb value=-0.0024938583374023438
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=75 bd d0 c0 value=-6.52312707901001

frame #8826 offset=264285 cmd=0x21 len=28 host_read_t=10.662651441s
frame: fe 55 21 1c 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 08 16 bd 00 08 16 3d 00 00 00 00 fc c8 df 41 eb 8f
payload: 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 08 16 bd 00 08 16 3d 00 00 00 00 fc c8 df 41
slot0: bytes=00 10 a0 bb value=-0.0048847198486328125
slot1: bytes=00 10 a0 bb value=-0.0048847198486328125
slot2: bytes=00 10 a0 bb value=-0.0048847198486328125
slot3: bytes=00 08 16 bd value=-0.03662872314453125
slot4: bytes=00 08 16 3d value=0.03662872314453125
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=fc c8 df 41 value=27.97313690185547

frame #9495 offset=284331 cmd=0x22 len=28 host_read_t=11.470634273s
frame: fe 55 22 1c 00 09 13 be 38 79 13 c0 e0 cb 3e 3f 00 10 f0 3b 00 08 84 3c 00 00 00 00 b4 25 5b 40 8e 1f
payload: 00 09 13 be 38 79 13 c0 e0 cb 3e 3f 00 10 f0 3b 00 08 84 3c 00 00 00 00 b4 25 5b 40
slot0: bytes=00 09 13 be value=-0.14358901977539062
slot1: bytes=38 79 13 c0 value=-2.3042736053466797
slot2: bytes=e0 cb 3e 3f value=0.7452983856201172
slot3: bytes=00 10 f0 3b value=0.0073261260986328125
slot4: bytes=00 08 84 3c value=0.016117095947265625
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=b4 25 5b 40 value=3.4241762161254883

frame #10439 offset=312600 cmd=0x24 len=14 host_read_t=12.611626316s
frame: fe 55 24 0e 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00 1a 45
payload: 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00
slot0: bytes=1c 00 value=28
slot1: bytes=1c 00 value=28
slot2: bytes=1c 00 value=28
slot3: bytes=20 00 value=32
slot4: bytes=20 00 value=32
slot5: bytes=00 00 value=0
slot6: bytes=01 00 value=1

frame #11010 offset=329690 cmd=0x20 len=28 host_read_t=13.303325493s
frame: fe 55 20 1c 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0 5f 7e
payload: 00 40 ce 3b e8 a7 a9 3f 70 bb ac bf 00 00 b0 ba 00 70 23 bb 00 00 00 00 75 bd d0 c0
slot0: bytes=00 40 ce 3b value=0.00629425048828125
slot1: bytes=e8 a7 a9 3f value=1.3254365921020508
slot2: bytes=70 bb ac bf value=-1.3494701385498047
slot3: bytes=00 00 b0 ba value=-0.0013427734375
slot4: bytes=00 70 23 bb value=-0.0024938583374023438
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=75 bd d0 c0 value=-6.52312707901001

frame #11277 offset=337696 cmd=0x21 len=28 host_read_t=13.627395783s
frame: fe 55 21 1c 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 08 16 bd 00 10 48 bc 00 00 00 00 fc c8 df 41 a4 52
payload: 00 10 a0 bb 00 10 a0 bb 00 10 a0 bb 00 08 16 bd 00 10 48 bc 00 00 00 00 fc c8 df 41
slot0: bytes=00 10 a0 bb value=-0.0048847198486328125
slot1: bytes=00 10 a0 bb value=-0.0048847198486328125
slot2: bytes=00 10 a0 bb value=-0.0048847198486328125
slot3: bytes=00 08 16 bd value=-0.03662872314453125
slot4: bytes=00 10 48 bc value=-0.012210845947265625
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=fc c8 df 41 value=27.97313690185547

frame #12309 offset=368609 cmd=0x22 len=28 host_read_t=14.873389538s
frame: fe 55 22 1c 00 0a 9a bd 10 d9 10 c0 40 cd 53 3f 00 10 c0 ba 00 10 c0 3a 00 00 00 00 b4 25 5b 40 52 06
payload: 00 0a 9a bd 10 d9 10 c0 40 cd 53 3f 00 10 c0 ba 00 10 c0 3a 00 00 00 00 b4 25 5b 40
slot0: bytes=00 0a 9a bd value=-0.07521438598632812
slot1: bytes=10 d9 10 c0 value=-2.2632484436035156
slot2: bytes=40 cd 53 3f value=0.8273506164550781
slot3: bytes=00 10 c0 ba value=-0.0014653205871582031
slot4: bytes=00 10 c0 3a value=0.0014653205871582031
slot5: bytes=00 00 00 00 value=0.0
slot6: bytes=b4 25 5b 40 value=3.4241762161254883

frame #12624 offset=378039 cmd=0x24 len=14 host_read_t=15.256583549s
frame: fe 55 24 0e 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00 1a 45
payload: 1c 00 1c 00 1c 00 20 00 20 00 00 00 01 00
slot0: bytes=1c 00 value=28
slot1: bytes=1c 00 value=28
slot2: bytes=1c 00 value=28
slot3: bytes=20 00 value=32
slot4: bytes=20 00 value=32
slot5: bytes=00 00 value=0
slot6: bytes=01 00 value=1
```
