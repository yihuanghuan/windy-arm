# ACADOS NMPC IK 运行说明

当前 NMPC 做的是固定基 7 自由度机械臂的末端位置 MPC,可选末端姿态软约束。控制器最终发布的是 7 个关节角到 `/student/joint_command`。ACADOS 是外部依赖,仓库不会提交 ACADOS 源码、生成 C 代码或 build 产物。

## 1. 环境部署

> 环境部署(系统依赖、编译工作空间、安装 acados、环境自检)已统一到**总 [README.md](README.md) 的《六、部署步骤》**,并区分**一般电脑 / Jetson** 两类。请先按那里完成部署,再回到本文跑 demo。

部署完成后,**每个新终端**运行本文的 demo 前先激活环境:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
```

验证 ACADOS Python 接口:`python3 -c "import acados_template; print('acados ok')"`。

## 2. 最小自测 — `mpc_pinocchio_ik.py`

进入 demo 目录:

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ../scripts/setup_acados_env.sh
```

运行 NMPC 自测:

```bash
python3 mpc_pinocchio_ik.py  # 随机局部目标 MPC 回代验证
```

在你自己的代码中使用:

```python
import numpy as np
from mpc_pinocchio_ik import MpcPinocchioIK

ik = MpcPinocchioIK()
q_terminal, ok, info = ik.solve(np.array([0.4, 0.0, 0.2]))
q_next, ok, info = ik.step(np.array([0.4, 0.0, 0.2]), q_current=q_terminal)
```

如果希望同时软约束末端姿态,给配置设置非零姿态权重,再传入目标旋转矩阵:

```python
from mpc_pinocchio_ik import MpcIkConfig, MpcPinocchioIK

ik = MpcPinocchioIK(config=MpcIkConfig(
    orientation_weight=2.0,
    terminal_orientation_weight=20.0,
))
target_pos, target_rot = ik.forward(q_terminal)
q_next, ok, info = ik.step(target_pos, q_current=q_terminal, target_rotation=target_rot)
print(info["terminal_orientation_error"])
```

第一次运行 ACADOS 后端时会生成并编译 C 求解器,可能需要几秒到几十秒。后续在线求解通常是毫秒以内,这个首次编译时间不是每个控制周期都会发生。

ACADOS 求解器缓存规则:

- 第一次运行某个配置时,程序使用 `generate=True, build=True`,在 `MpcIkConfig.acados_build_dir` 下生成 `c_generated_code/`、`acados_ocp.json` 和 `acados_solver_metadata.json`。
- 后续同配置运行时,如果检测到 `c_generated_code/libacados_ocp_solver_*.so`、`acados_ocp.json` 和 `acados_solver_metadata.json` 都存在且配置指纹一致,程序使用 `generate=False, build=False`,直接加载已编译的求解器。
- 如果配置变了,例如 `horizon_steps`、`max_joint_step`、权重、姿态软约束开关、前桨避障开关、关键点数量、避障球数量、机身 box 数量、capsule 半径或 capsule 采样点数量变化,配置指纹会不一致,程序会自动重新生成并编译。
- 如果怀疑缓存损坏,可以删除对应的 `acados_build_dir` 目录,下次启动会重新生成。

## 3. 启动 demo

先启动学生机械臂仿真:

```bash
source /opt/ros/humble/setup.bash
source ~/code/windylab_ws/install/setup.bash
ros2 launch manipulator student_arm.launch.py
```

### 3.1 竖直圆 — `move_arm_mpc_ik_demo.py`

末端在 y-z 平面画半径 8 cm 的圆。

配合绘图程序查看 MPC 轨迹:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=circle 

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py
```

### 3.2 直线 — `move_arm_mpc_ik_demo.py`

末端在两点间沿直线平滑往返(余弦插值,端点速度为零)。

配合绘图程序查看 MPC 轨迹:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=line \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=line
```

### 3.3 水平圆 — `move_arm_mpc_ik_demo.py 或 move_arm_horizontal_circle_mpc_ik_demo`

末端在 x-y 平面画半径 8 cm 的圆。

①直接启动
```bash
cd ~/code/windylab_ws/src/arm-platform/demo
python3 move_arm_horizontal_circle_mpc_ik_demo.py
```
②也可以直接用通用 MPC demo 切换模式:
```bash
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=horizontal_circle
```

配合绘图程序查看 MPC 轨迹:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_mpc_ik_demo.py --ros-args -p trajectory_mode:=horizontal_circle
```

### 3.4 软锁姿态水平圆 — `move_arm_mpc_ik_demo.py 或 move_arm_horizontal_circle_mpc_ik_demo`

锁定起始末端姿态运行水平圆:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p lock_orientation:=true
```

水平圆并约束末端 x 轴竖直朝下:

```bash
source /opt/ros/humble/setup.bash && source ~/code/windylab_ws/install/setup.bash
ros2 run manipulator trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=horizontal_circle \
  -p ignore_initial_sec:=2.0

# 另开一个终端
cd ~/code/windylab_ws/src/arm-platform/demo
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
python3 move_arm_horizontal_circle_mpc_ik_demo.py --ros-args \
  -p lock_orientation:=true \
  -p orientation_target:=vertical_down
```

其中 `vertical_down` 定义为:末端坐标系 x 轴对齐世界坐标 `-Z`,y 轴对齐世界坐标 `+Y`,z 轴对齐世界坐标 `+X`。

### 3.5 基座运动下的 world 系末端跟踪

这组实验的末端目标定义在 `world` 坐标系下。控制器把 world 系目标变换到当前
`base_link` 系后求解 NMPC，可视化节点再用同一组基座真值和轨迹时钟把关节反馈变换回
world 系。标准入口是 `moving_base_uncertainty.launch.py`：它统一启动机械臂仿真、
RViz、控制器和数据记录/可视化节点，并把同一组场景选择传给所有节点，避免控制端和
可视化端使用不同的轨迹或扰动。

启动时只选择 `trajectory_shape`、`base_disturbance_profile` 和
`base_belief_mode`。NMPC 权重、执行器模型、估计器、信任度算法、安全门、记录器和
RViz 的生产参数都由
`src/arm-platform/config/moving_base_uncertainty.yaml` 统一加载，不应在启动命令中
逐项覆盖。

#### 3.5.1 可选择的绘图任务

启动统一 launch 时通过 `trajectory_shape` 选择任务。允许值为
`circle`、`triangle`、`square`、`sine`、`windylab` 和 `whole`;
下表中的周期均为配置文件中 `trajectory_time_scale: 1.0` 时的默认值，所有任务都会
循环执行。

| `trajectory_shape` | 默认周期 | world 系轨迹和抬笔行为 |
|---|---:|---|
| `circle` | 4.0 s | 兼容原有任务:中心 `[0.30, 0.01, 0.10]` m、半径 0.08 m,在水平 XY 平面连续画圆 |
| `triangle` | 6.0 s | 在同一水平平面连续画三角形,宽 0.14 m、高 0.12 m;每条边使用平滑起停 |
| `square` | 8.0 s | 在同一水平平面连续画边长 0.12 m 的正方形;每条边使用平滑起停 |
| `sine` | 8.0 s | 5.0 s 画一周期正弦,宽 0.16 m、振幅 0.05 m;随后抬笔 0.5 s、悬空返回 2.0 s、落笔 0.5 s |
| `windylab` | 约 32.590 s | 绘制水平镜像的小写 `windylab`:字宽 0.016 m、字间距 0.016 m(原 0.006 m 上增加 0.010 m)、字高 0.050 m,总宽约 0.24 m。共有 10 个独立笔画(`i` 的点和 `y` 的下行笔画独立),每次换笔和换字都真实抬起末端 |
| `whole` | 约 73.039 s | 完整五面任务:正面圆 → 左面三角形 → 后面镜像 `windylab` → 右面正弦 → 上面正方形;任务之间抬笔、移动并改变基座朝向 |

`whole` 的五个绘图面和顺序如下:

| 顺序 | 方位/绘图面 | 图形 | 默认绘图面中心 |
|---:|---|---|---|
| 1 | 正面,竖直面 `x=+0.30` m | 圆 | `[+0.30, 0.00, 0.12]` m |
| 2 | 左面,竖直面 `y=+0.30` m | 三角形 | `[0.00, +0.30, 0.12]` m |
| 3 | 后面,竖直面 `x=-0.30` m | `windylab` | `[-0.30, 0.00, 0.12]` m |
| 4 | 右面,竖直面 `y=-0.30` m | 正弦 | `[0.00, -0.30, 0.12]` m |
| 5 | 上面,水平面 `z=+0.30` m | 正方形 | `[+0.22, 0.00, 0.30]` m |

每两个面之间默认执行 0.6 s 抬笔、2.4 s 悬空转场、0.6 s 落笔。
基座在这些转场期间依次转向 0°、90°、180°、270° 和 360°,
同时仍叠加共享配置中的小幅浮动扰动。第五个任务结束后也会抬笔并返回第一个任务,
因此循环边界不会连笔。

独立 `windylab` 是周期任务,写完最后一个 `b` 后会抬笔返回下一轮 `w` 的起点;
这个返回全程 `pen_down=false`,不会点到绘图面。`whole` 中的 `windylab` 是一次性子任务,
最后一个 `b` 结束后直接由五面任务执行抬笔转场,不会先返回或触碰 `w` 起点。

`windylab`、`sine` 的返回段以及 `whole` 的转场段都会生成真实的悬空末端目标,
不是只在图上隐藏连线。轨迹样本使用 `pen_down` 和 `stroke_id` 标记笔画;
RViz Marker、实时/离线 Matplotlib 曲线仅连接同一 `stroke_id` 的落笔点。
保存的轨迹 CSV 也包含 `pen_down` 和 `stroke_id`,所以字母间和各绘图面之间不会出现
可视化连笔。

#### 3.5.2 不确定性实验

首次使用，或修改了 launch、YAML、轨迹配置和可视化代码后，先重新构建：

```bash
cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select manipulator --symlink-install
```

标准 `whole + gust + model` 不确定性实验只需要一个终端：

```bash
cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/arm-platform/scripts/setup_acados_env.sh
source src/arm-platform/demo/setup_fastdds_udp_only.sh

ros2 launch manipulator moving_base_uncertainty.launch.py \
  trajectory_shape:=whole \
  base_disturbance_profile:=gust \
  base_belief_mode:=model
```

测试持续偏置故障时，只把 profile 改为 `fault`：

```bash
ros2 launch manipulator moving_base_uncertainty.launch.py \
  trajectory_shape:=whole \
  base_disturbance_profile:=fault \
  base_belief_mode:=model
```

`setup_fastdds_udp_only.sh` 内部集中设置 DDS 所需环境；同一个终端只需 source 一次，
不需要在每条 `ros2`/`python3` 命令前重复写 `export`。脚本默认使用
`ROS_DOMAIN_ID=78`；只有同时运行多组互不通信的实验时，才在 source 命令末尾传入另一个
`0..232` domain id。该 profile 只用于同机进程。停止整组实验时在 launch 终端按
`Ctrl+C`。

#### 3.5.3 启动命令与场景参数说明

启动块中各条命令的作用如下：

| 命令 | 作用 | 可选输入 |
|---|---|---|
| `source /opt/ros/humble/setup.bash` | 加载 ROS 2 Humble | 无 |
| `source install/setup.bash` | 加载当前工作空间中已构建的 `manipulator` 包 | 无 |
| `source src/arm-platform/scripts/setup_acados_env.sh` | 加载 ACADOS 动态库和 Python 接口 | 无 |
| `source src/arm-platform/demo/setup_fastdds_udp_only.sh` | 本机仿真使用 UDP-only FastDDS，避免共享内存残留 | 通常无；多组实验并行时可传 domain id，默认 `78` |
| `ros2 launch manipulator moving_base_uncertainty.launch.py ...` | 启动整组不确定性实验 | 仅下表三项 |

统一 launch 只公开三个场景参数：

| 参数 | 示例值 | 允许值与作用 |
|---|---|---|
| `trajectory_shape` | `whole` | `circle` / `triangle` / `square` / `sine` / `windylab` / `whole`，选择末端绘图任务 |
| `base_disturbance_profile` | `gust` | `harmonic` / `gust` / `fault` / `wideband`，选择施加到基座平移真值的扰动 |
| `base_belief_mode` | `model` | `nominal` / `truth` / `estimated` / `model`，选择控制器可获得的基座扰动信息 |

这里使用的是 ROS launch 参数语法，所以三项后面不写 `-p`。四种扰动 profile 的语义为：

| profile | 扰动真值 |
|---|---|
| `harmonic` | 有界多谐波基座漂移 |
| `gust` | 半幅谐波背景上叠加平滑、短时的阵风脉冲 |
| `fault` | 半幅谐波背景上叠加一个限定时间窗口内的恒定偏置 |
| `wideband` | 频带更宽、谐波数更多的有界扰动 |

四种 belief mode 的语义为：

| belief mode | 控制器看到的信息 |
|---|---|
| `nominal` | 只知道名义基座轨迹，不知道附加扰动 |
| `truth` | 直接使用扰动真值，作为全知对照 |
| `estimated` | 使用带延迟、噪声和低通的模拟状态估计 |
| `model` | 使用同源测量驱动频谱辨识、保形校准和预测模型，为主要不确定性算法 |

`model` 模式默认运行完整 B4。算法内部保留五个对照层级，但它们属于研究配置，
不作为 launch 参数公开：

| `uncertainty_control_mode` | 感知与控制作用 |
|---|---|
| `b0` | 不使用扰动预测前馈 |
| `b1` | 仅使用谐波参数通道点预测，不做适用度或快速残差修正 |
| `b2` | 使用原有 β 平滑门控谐波前馈，作为兼容回退 |
| `b3` | 估计并记录参数适用度，使用完整的谐波与两工况快速残差融合均值，但保持固定调速和 NMPC 权重 |
| `b4` | 使用完整融合预测、风险补偿、任务调速、NMPC 位置权重适配和安全过滤半径 |

两工况快速残差模型使用统一三维状态。瞬态工况描述向零衰减的短时残差，持续工况
描述随机游走偏差。工况概率只用于动力学模型融合，不等同于预测可靠性。β 只表示
谐波参数通道的适用程度；融合协方差和多步保形半径共同描述最终预测风险。因此持续
偏差出现时，即使谐波参数适用度下降，快速残差补偿也不会被同一个 β 门整体关闭。

扰动幅值、随机种子、估计噪声和延迟都属于实验定义，不再作为启动命令参数。
CSV、参数快照和退出时图片默认保存到
`${ROS_HOME:-~/.ros}/moving_base_uncertainty`；这是 launch 自动选择的主机输出目录，
不属于算法调参项。

#### 3.5.4 集中配置文件

统一 launch 自动读取：

```text
src/arm-platform/config/moving_base_uncertainty.yaml
```

这是本实验唯一推荐的算法调参入口。当前生产配置的关键配对如下：

| 配置组 | 关键配置 |
|---|---|
| 扰动真值 | `base_disturbance_amplitude_m: 0.01`，`base_disturbance_seed: 0` |
| 模拟估计器 | `base_estimator_noise_std_m: 0.002`，`base_estimator_delay_ms: 40.0` |
| 执行器与内部 plant | 速度前馈增益 `1.0`，最大速度 `3 rad/s`，最大加速度 `20 rad/s²` |
| 二阶 OCP | 预测步数 `N=10`，最大速度 `2 rad/s`，最大加速度 `15 rad/s²` |
| 应用时延补偿 | 开启；名义求解到发布延迟 `9 ms`，发布后执行器延迟 `5 ms` |
| 不确定性感知 | `uncertainty_control_mode: b4`，两工况快速残差与多步保形校准开启 |
| 风险调控 | 风险预算 `0.012 m`，最小任务速度比例 `0.3`，最小位置权重比例 `0.35` |
| 在线自适应 | `online_time_scaling_enabled: true`；旧 `trust_ladder_enabled: false`，只保留作 B2 消融 |
| 安全与记录 | transport delay guard 开启；保形半径接入守卫并限幅到 `0.015 m`；loop/control/trust CSV 记录开启 |
| 可视化 | `frame_id: world`，`path_clock_mode: required`，RViz 路径 5 Hz/最多 2500 点，实时 Matplotlib 关闭 |

执行器实际限幅、控制器内部 plant 模型和 OCP 约束是三组不同语义。YAML 集中保存
这三组值；调整时必须保持 plant 与执行器一致，并保证 OCP 约束不超过执行器限幅。
不要只改其中一处，也不要通过命令行临时覆盖 plant 或 OCP 参数，否则参数快照与实验
条件会失去一致性。

B4 每拍在 `trust_shadow_<stamp>.csv` 中额外记录参数适用度、持续工况概率、快速残差
均值、融合风险半径、补偿增益、任务速度上限、NMPC 位置权重比例和守卫不确定性半径。
这些字段用于 B0--B4 配对对照及扰动后恢复时间分析。解析平台只有基座平移不确定性，
所以基座预测误差到 world 系末端位置的传播雅可比为单位阵；进入含姿态与动力学耦合的
Gazebo 平台后必须替换为完整的构型相关传播。

不启动 ROS 时，可先运行纯 numpy 的 B0--B4 代理验收：

```bash
python3 src/arm-platform/demo/eval_uncertainty_b4.py \
  --seeds 0,1,2 \
  --duration 30 \
  --out /tmp/uncertainty_b4_eval.json
```

该脚本检查持续工况是否激活、B3/B4 是否优于对应旧基线，以及估计与决策快路径的
P99 是否低于 `5 ms`。它只评价扰动补偿代理误差，不得写成机械臂闭环实验结果；
最终验收仍需运行 3.5.2 的 ROS 实验并比较末端 RMSE、峰值误差和恢复时间。

修改 YAML 后停止当前 launch，重新构建并启动：

```bash
cd ~/code/windylab_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select manipulator --symlink-install
```

轨迹几何、笔画定义、五个绘图面和基座名义运动仍集中在
`src/arm-platform/demo/moving_base_trajectory_config.py`。普通算法实验只修改 YAML；
只有确实要改变图形尺寸、笔画或绘图面几何时才修改这个 Python 配置。

#### 3.5.5 gust/fault 的 RViz 可视化

统一 launch 会把 `/moving_base/disturbance_markers` 加入 RViz，所有 Marker 都在
`world` 坐标系中：

| RViz 元素 | 颜色 | 含义 |
|---|---|---|
| 名义基座球 | cyan | 未叠加实验扰动的基座位置 |
| 真实基座球 | red | 叠加完整扰动后的基座真值位置 |
| 总扰动箭头 | cyan | 从名义基座指向真实基座，表示当前总扰动 |
| 事件分量箭头 | magenta | `gust` 的当前阵风分量，或 `fault` 窗口内的恒定偏置分量 |
| 状态文本 | 随状态更新 | 显示 profile、事件是否 active、总扰动和事件分量的大小 |

`gust` 的 magenta 箭头只在阵风脉冲期间出现，`fault` 的 magenta 箭头在偏置窗口内
保持。切换运行 epoch 或重新启动时会清除上一轮 Marker，避免旧的基座点和箭头留在
RViz 中。

为了不被基座 mesh 遮挡，两颗球和两支箭整体等量上移 `0.12 m` 作为显示锚点；箭头
起终点之间的差仍是未放大的真实扰动向量，文本数值也是真值。RViz 路径只对显示副本
限点和降采样，CSV、退出时 PNG 和误差计算仍保留全率样本。

专用 RViz 中不显示黄色/橙色 command-FK 轨迹，只显示绿色期望轨迹和红色实际轨迹；
command-FK 仍在后台写入 CSV/PNG，用于区分控制命令与执行器反馈误差。红色实际轨迹
按 `stroke_id` 分成多个 `LINE_STRIP`：同一落笔段内部连续，抬笔、换字和换面处一定
断开。统一实验配置对每个笔画的 RViz 显示副本使用 7 点居中平滑，且保持各笔画首尾
点不变；该平滑不修改原始采样、CSV、退出时 PNG 或误差指标。

实验结束后可从最新一组完整数据生成四幅独立离线图：

```bash
cd ~/code/windylab_ws
python3 src/arm-platform/demo/plot_moving_base_report.py
```

脚本默认从 `${ROS_HOME:-~/.ros}/moving_base_uncertainty` 选择最新主轨迹 CSV 及其
控制器参数快照，输出三维轨迹、三轴期望/实际跟踪、误差变化和基座频谱四幅 PNG。
它不会读取 command-FK；三轴图的扰动窗和箭头使用物理 `time_sec`，不使用可能被
在线降速的 `path_time_sec`。基座频谱中的物理真值由参数快照和同源扰动模型重建，
不会把 model belief 下的 `base_pos_*` 误标为真实基座。

RViz 的 `Global Options -> Fixed Frame` 必须是 `world`。若固定在 `base_link`，
网格和 world 目标会呈现反向运动，很容易被误判为末端漂移。这里的扰动直接施加到
基座平移真值，不是直接施加给末端；末端位置变化是基座运动经过机械臂运动学后的结果。

#### 3.5.6 算法、双时钟与安全记录

生产配置使用二阶运动学 NMPC：状态为 `[q, dq]`，控制量为关节加速度，预测步数
`N=10`。控制器向 `/student/joint_command` 同时发布关节位置和速度，执行器模型按
`3 rad/s`、`20 rad/s²` 限幅；OCP 以 `2 rad/s`、`15 rad/s²` 留出跟踪和安全余量。
求解器权重、信任度算法和安全过滤器的完整参数都在 YAML 中。

基座运动与绘图进度使用两个时钟：

```text
base_phase    = base_time / base_motion_time_scale
drawing_phase = path_time / trajectory_time_scale
```

`base_time` 始终按真实时间推进。在线时间缩放开启后，`path_time` 按 `path_scale`
积分；信任度阶梯可以减慢或冻结绘图，但不会让物理基座停止运动。控制器通过
`/moving_base/path_progress` 发布同一组 `base_time/path_time` 映射，可视化器以
`path_clock_mode: required` 插值每条反馈；时钟缺失或陈旧时暂停 RViz/CSV 采样，
不会退回到不一致的 wall time。

world 系实际末端和期望末端按下式计算：

```text
actual_world(base_time, path_time) =
    p_wb_true(base_time) + R_wb(base_time, path_time) @ FK_base(q)

expected_world(path_time) = p_ref_world(path_time)
```

因此保存的误差是 world 系真实末端相对 world 系目标的误差。作图程序只从
`moving_base_track` 阶段开始记录，并用 `pen_down` / `stroke_id` 区分真实笔画和悬空
转场；command-FK 数据则用来区分“NMPC 命令本身”与“执行器反馈”造成的误差。

控制节点使用单线程确定性 executor 和深度为 1 的反馈 QoS，求解期间只保留最新
`/joint_states`。transport delay guard 会检查 DDS 投递延迟和反馈断流；触发后立即
保持最后一条安全命令、冻结 `path_time`、停止继续求解，并把该次参数快照和 CSV 标记为
故障运行。录屏或桌面渲染造成的调度延迟也会被这一安全门检测，不能通过放宽机械臂约束
掩盖。

## 4. 作图模块 — `trajectory_visualizer_demo.py`

下表是节点单独运行时的代码默认值；3.5 的统一不确定性实验会用
`moving_base_uncertainty.yaml` 覆盖为 `moving_base_circle + world + required path clock`
等正式配置。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trajectory_mode` | `circle` | 期望轨迹模式:`circle` / `horizontal_circle` / `line` / `moving_base_circle`,要和运动 demo 保持一致 |
| `frame_id` | `base_link` | RViz Marker 使用的坐标系 |
| `actual_topic` | `/joint_states` | 实际关节状态输入 topic |
| `command_topic` | `/student/joint_command` | 轨迹命令 topic,用来判断轨迹开始 |
| `control_signal_topic` | `/student/joint_command` | 控制信号记录 topic |
| `record_control_signal` | `False` | 是否记录控制信号 CSV/图 |
| `record_command_fk` | `True` | 是否把 `/student/joint_command` 的关节角做 FK,记录命令末端轨迹 CSV/图 |
| `show_command_fk_marker` | `True` | 是否在 RViz 显示 command-FK 轨迹；3.5 统一实验设为 `False`，不影响其 CSV/PNG 记录 |
| `publish_rate_hz` | `10.0` | RViz Marker 发布频率 |
| `max_actual_points` | `12000` | RViz 最多保留的实际轨迹采样点数;默认可完整保留至少一轮 `whole`,避免早期红色笔画在任务结束前被滚动删除 |
| `max_rviz_path_points` | `2500` | 每条 RViz 路径的显示点预算；只降采样 Marker，不影响 CSV/PNG |
| `actual_rviz_smoothing_window` | `5` | 红色实际轨迹的逐笔画显示平滑窗口，必须为正奇数；3.5 统一实验设为 `7`，不改变原始数据和指标 |
| `expected_path_points` | `240` | 期望轨迹离散点数 |
| `show_matplotlib` | `False` | 是否打开 CPU 开销较高的 Matplotlib 实时曲线窗口;RViz/CSV/退出时 PNG 不受影响 |
| `start_on_command` | `True` | 是否等收到命令后再开始记录 |
| `use_message_stamp` | `True` | 是否使用消息时间戳计算轨迹时间 |
| `path_progress_topic` | `/moving_base/path_progress` | moving-base 控制器发布的双时钟映射 topic |
| `path_clock_mode` | `auto` | `auto` 检测发布者后强制同步;`required` 始终要求;`disabled` 仅用于旧控制器兼容 |
| `path_clock_max_extrapolation_sec` | `0.10` | 最后一个路径时钟锚点允许的最大有限外推时间 |
| `reset_gap_sec` | `2.0` | 命令间隔超过该值时重置记录 |
| `stale_command_timeout_sec` | `0.2` | 命令超时后暂停误差记录 |
| `sample_only_on_actual_change` | `True` | 只有实际关节变化时才采样 |
| `actual_change_epsilon` | `1e-9` | 实际关节变化判定阈值 |
| `control_signal_change_epsilon` | `1e-12` | 控制信号变化判定阈值 |
| `ignore_initial_sec` | `1.0` | 开始后忽略前多少秒数据 |
| `show_base_disturbance` | `True` | moving-base 模式下是否发布基座扰动 MarkerArray |
| `disturbance_marker_topic` | `/moving_base/disturbance_markers` | 基座扰动可视化 topic |
| `disturbance_display_height_m` | `0.12` | 扰动球/箭整体抬高的显示偏移；不改变向量和值 |
| `data_dir` | 节点脚本所在目录的 `data/` | 节点单独运行时的 CSV/PNG 保存目录；统一 launch 会改到 ROS 数据目录 |


## 5. MPC IK 模块 — `move_arm_mpc_ik_demo.py/move_arm_horizontal_circle_mpc_ik_demo.py`

参数介绍：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trajectory_mode` | `circle` | 运动轨迹:`circle` 垂直圆 / `horizontal_circle` 水平圆 / `line` 直线 |
| `terminal_tolerance` | `0.01` | MPC IK 末端位置误差容忍值,单位 m |
| `solver_backend` | `acados` | 求解器后端:`acados` |
| `horizon_steps` | `2` | MPC 预测步数 |
| `max_joint_step` | `0.02` | 每个控制周期最大关节步长,单位 rad |
| `lock_orientation` | `False` | 是否启用末端姿态软约束 |
| `orientation_target` | `initial` | 姿态目标:`initial` 保持初始姿态 / `vertical_down` 末端朝下 |
| `orientation_weight` | `0.375` | 姿态约束权重,仅 `lock_orientation:=true` 时生效 |
| `terminal_orientation_weight` | `5.0` | 终端姿态约束权重,仅 `lock_orientation:=true` 时生效 |

### 5.1 核心文件介绍

- `move_arm_mpc_ik_demo.py`:核心启动 demo —— 内部生成期望末端轨迹、按 50 Hz 调用 MPC 求解、把关节命令发布到 `/student/joint_command`,并负责 reset→track 切换与求解失败降级。
- `move_arm_horizontal_circle_mpc_ik_demo.py`:上面那个节点类的水平圆封装(同一个类,仅把默认 `trajectory_mode` 设为 `horizontal_circle`)。
- `MpcPinocchioIK`(`mpc_pinocchio_ik.py`):NMPC 公式与求解器调用(Pinocchio 从 URDF 建模 + acados 后端)。
- 期望轨迹:固定臂没有单独的轨迹发布节点,`circle`/`horizontal_circle`/`line` 由 demo 内部的 `_target()` 直接生成。


### 5.2 话题的使用

- **MPC 接受的当前状态话题**:订阅 `/joint_states` 中的 `joint1`~`joint7`,作为 NMPC 闭环求解的当前状态(求解起点)。
- **MPC 接收的期望状态话题**:无 —— demo 内部直接生成笛卡尔轨迹(`circle`/`horizontal_circle`/`line`),不订阅外部期望话题。
- **MPC 输出的控制量话题**:发布 `/student/joint_command`(`sensor_msgs/JointState`,`position` = 下一拍关节角 `q_next`,`velocity` = 关节速度前馈 `dq_cmd`)。

## 6. 调参

统一参数闭环离线调参脚本(固定臂 + 浮动基座臂四场景, 一套参数):

```bash
cd ~/code/windylab_ws/src/arm-platform/demo
source ../scripts/setup_acados_env.sh
python3 tune_unified_mpc_params.py
```

详细说明见 `MPC_TUNING_README.md`。

## 7. 常见问题

### 找不到 `acados_template`

先 source 环境:

```bash
source ~/code/windylab_ws/src/arm-platform/scripts/setup_acados_env.sh
```

如果仍失败,运行:

```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/check_nmpc_env.sh
```

### 找不到 `libacados.so`

通常是没有 source `setup_acados_env.sh`,或者 ACADOS 没有 `make install`。修复:

```bash
cd ~/code/windylab_ws/src/arm-platform
scripts/install_acados.sh
source scripts/setup_acados_env.sh
```

### 第一次运行很慢

ACADOS 第一次会生成 C 代码并编译求解器。这个是启动/建模开销,不是每帧在线求解开销。只要 `horizon_steps`、权重维度、模型结构等配置不变,后续会复用生成目录。

### 端载设备上不够快

先降 `horizon_steps`,例如:

```bash
python3 move_arm_mpc_ik_demo.py --ros-args -p horizon_steps:=4
```

再根据跟踪误差调整 `max_joint_step` 和姿态权重。机械臂 demo 默认 50 Hz,每个控制周期是 20 ms。

### ACADOS 后端失败

本仓库已裁剪为只部署 acados 后端。先确认已完成部署并激活环境:

```bash
cd ~/code/windylab_ws/src/arm-platform
source scripts/setup_acados_env.sh
scripts/check_nmpc_env.sh
```
