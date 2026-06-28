# MPC 参数粗到细搜索说明

本文说明 `tune_mpc_params.py` 的用途、运行方法、参数含义和结果解读。这个脚本用于离线调节 MPC IK 参数,默认针对的场景:

```text
水平圆轨迹 + 锁定初始末端姿态
```

脚本不启动 ROS、不画图,而是直接调用 `MpcPinocchioIK.step()` 模拟一圈轨迹。它会把多组参数跑完后排序,输出一个 CSV 和终端 Top 列表。

运行本文命令前,请先按 `NMPC_README.md` 完成 ACADOS 安装,并确认:

```bash
cd ~/code/windylab_ws/src/arm-platform
source scripts/setup_acados_env.sh
scripts/check_nmpc_env.sh
```

## 1. 快速运行

默认使用 ACADOS 后端。运行前先配置 ACADOS 环境:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 tune_mpc_params.py
```

第一次运行某些参数组合时,ACADOS 会生成并编译求解器,因此会比较慢。想先确认脚本能跑通,可以限制组合数量:

```bash
python3 tune_mpc_params.py --skip-fine --max-combinations 3
```

## 2. 输出文件

脚本会生成:

```text
demo/data/mpc_tuning_<时间戳>.csv
```

终端会打印 Top 参数组合,例如:

```text
rank score h step ow tow rms_pos max_pos rms_ori max_ori bad fallback mean_ms max_ms
```

最后还会打印一条推荐运行命令,可直接复制运行水平圆 demo。

## 3. 参数含义

命令行参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--backend` | `acados` | 求解器后端:`acados` / `casadi` / `scipy` |
| `--orientation-target` | `initial` | 锁姿态目标:`initial` 锁定起始 IK 姿态 / `vertical_down` 末端 x 轴竖直朝下 |
| `--periods` | `1.0` | 仿真水平圆周期数 |
| `--top-k` | `10` | 终端打印排名前 K 的参数组合 |
| `--output-dir` | `/home/windylab/code/windylab_ws/src/arm-platform/demo/data` | 调参 CSV 保存目录 |
| `--skip-fine` | `False` | 是否跳过细搜索。Ture：跳过，False：不跳过 |
| `--fine-from-csv` | `None` | 从已有粗搜索 CSV 的 Top 参数继续细搜索,不重复粗搜索 |
| `--fine-top` | `5` | 用 Top 几名生成细搜索邻域 |
| `--no-quiet-build` | `False` | 是否静音 ACADOS 代码生成和编译输出;默认静音（False）,只保留每组参数摘要 |
| `--max-combinations` | `None` | 限制每个搜索阶段最多评估多少组参数,适合快速测试 |

内部搜索的 MPC 参数

| 参数 | 粗搜索默认候选值 | 说明 |
|---|---|---|
| `horizon_steps` | `[8, 12, 16]` | MPC 预测步数 |
| `max_joint_step` | `[0.08, 0.10]` | 每个控制周期最大关节步长,单位 rad |
| `orientation_weight` | `[0.5, 1.0, 2.0]` | 姿态误差权重 |
| `terminal_orientation_weight` | `[5.0, 10.0, 20.0]` | 终端姿态误差权重 |

细搜索会基于粗搜索 Top 结果生成邻域：

| 参数 | 细搜索变化方式 |
|---|---|
| `horizon_steps` | 当前值加 `[-2, 0, 2]`,最小为 1 |
| `max_joint_step` | 当前值加 `[-0.01, 0.0, 0.01]` |
| `orientation_weight` | 当前值乘 `[0.75, 1.0, 1.25]` |
| `terminal_orientation_weight` | 当前值乘 `[0.75, 1.0, 1.25]` |

内部搜索的参数可以在 src/arm-platform/demo/tune_mpc_params.py 内调整。

### `horizon_steps`

MPC 每次向未来看的步数。当前控制周期是:

```text
control_dt = 0.02 s
```

所以:

```text
horizon_steps = 8  -> 看未来 0.16 s
horizon_steps = 12 -> 看未来 0.24 s
horizon_steps = 16 -> 看未来 0.32 s
```

通常:

- 增大 `horizon_steps` 会让 MPC 看得更远
- 但优化变量更多,求解更慢
- 对当前“水平圆 + 锁姿态”数据,短时域反而更稳,已测推荐优先试 `4`

### `max_joint_step`

每个控制周期内,单个关节允许变化的最大角度,单位 rad。

```text
max_joint_step = 0.08 rad 约等于 4.58 deg
```

因为周期是 0.02 s,粗略对应关节速度:

```text
0.08 / 0.02 = 4 rad/s
```

通常:

- 增大 `max_joint_step` 会提升跟踪能力
- 但关节命令会更激进
- 对当前“水平圆 + 锁姿态”数据,`0.06` 比 `0.07/0.08/0.10` 综合评分更好

### `orientation_weight`

预测时域内每一步的末端姿态误差权重。

通常:

- 越大,越努力保持姿态
- 太大可能牺牲位置跟踪
- 太小会让姿态漂移更多

### `terminal_orientation_weight`

预测时域最后一步的姿态误差权重。

通常:

- 比 `orientation_weight` 更影响时域末端姿态
- 当前默认搜索范围是 `[5, 10, 20]`
- 如果位置误差偏大,可以降低它

## 4. 搜索流程

脚本默认做两阶段搜索。

### 粗搜索

默认组合:

```text
horizon_steps = [8, 12, 16]
max_joint_step = [0.08, 0.10]
orientation_weight = [0.5, 1.0, 2.0]
terminal_orientation_weight = [5.0, 10.0, 20.0]
```

共 54 组。

### 细搜索

脚本会取粗搜索 Top 5,然后在附近继续搜索:

```text
horizon_steps: 当前值 ± 2
max_joint_step: 当前值 ± 0.01
orientation_weight: 当前值 * [0.75, 1.0, 1.25]
terminal_orientation_weight: 当前值 * [0.75, 1.0, 1.25]
```

如果只想快速粗扫:

```bash
python3 tune_mpc_params.py --skip-fine
```

如果担心运行太久:

```bash
python3 tune_mpc_params.py --max-combinations 10
```

`--max-combinations` 会限制每个阶段最多评估多少组。

如果已经有一份粗搜索 CSV,可以不重复粗搜索,直接从 Top 参数继续细搜索:

```bash
python3 tune_mpc_params.py \
  --fine-from-csv demo/data/mpc_tuning_<时间戳>.csv \
  --fine-top 5 \
  --max-combinations 60
```

这个模式会把 CSV 中评分最好的 `--fine-top` 组作为中心,生成附近邻域继续搜索。

如果用旧 CSV 给新的姿态目标生成邻域,脚本只会借用旧参数,不会把旧评分混入新结果。

如果想切换为水平圆轨迹 + 锁定垂直末端姿态：

```bash
python3 tune_mpc_params.py \
  --orientation-target vertical_down \
  --fine-from-csv demo/data/mpc_tuning_<时间戳>.csv \
  --fine-top 5 \
  --max-combinations 60
```

## 5. 评分指标怎么看

CSV 和终端 Top 表里常用字段:

- `score`: 总评分,越低越好
- `rms_position_error`: 位置 RMS 误差,单位 m
- `max_position_error`: 最大位置误差,单位 m
- `rms_orientation_error`: 姿态 RMS 误差
- `max_orientation_error`: 最大姿态误差
- `tracking_bad_count`: 超过 `terminal_tolerance=0.01 m` 的次数
- `fallback_count`: 命令不可用并回退的次数
- `solver_failure_count`: 求解器状态失败次数
- `rms_control_delta`: 相邻关节命令变化 RMS
- `mean_solve_time_ms`: 平均求解时间,单位 ms
- `max_solve_time_ms`: 最大求解时间,单位 ms

当前评分是“位置和姿态均衡”。选择结果时建议按这个顺序看:

1. `fallback_count` 必须为 0
2. `max_position_error` 不要过大
3. `rms_position_error` 越小越好
4. `rms_orientation_error` / `max_orientation_error` 不要明显恶化
5. `mean_solve_time_ms` 和 `max_solve_time_ms` 不能超过实时需求太多
6. `rms_control_delta` 不要明显高于其他候选

## 6. 推荐使用流程

先跑一个小测试:

```bash
python3 tune_mpc_params.py --skip-fine --max-combinations 3
```

确认环境没问题后跑完整粗搜索:

```bash
python3 tune_mpc_params.py --skip-fine
```

如果粗搜索结果里 Top 组合明显集中在某个区域,再跑完整粗到细:

```bash
python3 tune_mpc_params.py
```

得到推荐命令后,用真实 demo 和可视化器验证:

```bash
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p horizon_steps:=4 \
  -p max_joint_step:=0.06 \
  -p lock_orientation:=true \
  -p orientation_weight:=0.9375 \
  -p terminal_orientation_weight:=25.0
```

末端竖直朝下版本:

```bash
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p horizon_steps:=8 \
  -p max_joint_step:=0.1100 \
  -p lock_orientation:=true \
  -p orientation_target:=vertical_down \
  -p orientation_weight:=0.2000 \
  -p terminal_orientation_weight:=5.0000
```

同时建议开控制信号记录:

```bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p actual_topic:=/student/joint_command \
  -p record_control_signal:=true
```

## 7. 调参经验

- 如果位置误差大,先看 `max_joint_step` 是否太小或姿态权重是否过强
- 如果轨迹速度不快但误差/抖动明显,可以尝试减小 `horizon_steps`
- 如果位置误差大但姿态误差很小,说明姿态锁太强,降低姿态权重
- 如果姿态漂移明显,提高 `terminal_orientation_weight`
- 如果控制信号抖动变大,降低 `max_joint_step` 或提高平滑代价项
- 如果 ACADOS 求解时间太长,降低 `horizon_steps`

## 8. 当前已测推荐

2026-06-24 的两轮 ACADOS 离线搜索结果,锁定起始姿态:

```text
horizon_steps = 4
max_joint_step = 0.06
orientation_weight = 0.9375
terminal_orientation_weight = 25.0
```

离线评估指标:

```text
score = 8.668
rms_position_error = 0.0013 m
max_position_error = 0.0025 m
rms_orientation_error = 0.0128
max_orientation_error = 0.0225
tracking_bad_count = 0
fallback_count = 0
mean_solve_time_ms = 0.24 ms
```

可直接运行:

```bash
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p horizon_steps:=4 \
  -p max_joint_step:=0.0600 \
  -p lock_orientation:=true \
  -p orientation_weight:=0.9375 \
  -p terminal_orientation_weight:=25.0000
```

末端竖直朝下的一圈 ACADOS 离线测试结果:

```text
horizon_steps = 8
max_joint_step = 0.11
orientation_weight = 0.2
terminal_orientation_weight = 5.0
```

离线评估指标:

```text
score = 87.572
rms_position_error = 0.0017 m
max_position_error = 0.0089 m
rms_orientation_error = 0.0749
max_orientation_error = 0.6804
tracking_bad_count = 0
fallback_count = 0
mean_solve_time_ms = 0.47 ms
```

可直接运行:

```bash
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p horizon_steps:=8 \
  -p max_joint_step:=0.1100 \
  -p lock_orientation:=true \
  -p orientation_target:=vertical_down \
  -p orientation_weight:=0.2000 \
  -p terminal_orientation_weight:=5.0000
```
