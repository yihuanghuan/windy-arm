"""死区时间补偿（Smith 预测）：把滞后的反馈状态往前推，换回环路增益上限。

为什么需要
==========
2026-08-04 通宵 15 组实物调参的结论：精度卡在交接后 RMS 2.95~3.17 mm，
从闭环数据反解的四项误差里 86% 的方差在 0.3 Hz 以下 —— 是准静态摩擦残余，
不是抖振。压它只有一条路：抬环路增益 c0 = (K+η/φ)·Λ。而 c0 被回路延迟锁死：

    速度路 T_v = 25 ms（力矩→速度一阶导互相关，j4/j6 各自 r=0.857/0.906，硬数）
    位置路 T_p = 10 ms（力矩→位置二阶导，r=0.434/0.450，⚠️ 证据弱）

本模块补的是【速度路那 25 ms】。


==========================================================================
1  预测什么：不是经典 Smith，是【测量锚定的有限时域外推】
==========================================================================
经典 Smith 预测器在控制器外面挂一个无延迟的并联模型，用
``x̂ = x_model(t) + [x_meas(t) − x_model(t−T)]`` 修正。**本系统绝不能用它**：

计算力矩抵消之后被控对象是 ``ẍ = ν``，一个【纯双积分器】。而"积分型对象上
Smith 预测器不成立"是教科书结论 —— 并联模型开环无阻尼，任何直流建模偏差或
扰动都会让它无界漂移，修正项 ``x_meas − x_model(t−T)`` 随时间线性/二次发散，
扰动抑制被彻底破坏。本系统恰好同时具备两个触发条件：

  * 对象是双积分器（不是自衡对象）
  * 已知存在直流量级的未建模力矩（j2 静摩擦标定高了 2.3 倍：实测启动力矩
    0.767 Nm vs 标定 1.775 Nm，差 1.0 Nm；折合 j2 加速度 5.1 rad/s²）

改用【测量锚定】形式：每一拍都从最新测量重新起算，模型只负责提供
horizon 内的【增量】：

    q̇̂(t) = q̇_meas(t) + β_v · Σ_{最近 T_v 秒} q̈_cmd·dt
    q̂(t)  = q_meas(t)  + β_p · [ T_p·q̇̂(t−T_p) + ΣΣ_{最近 T_p 秒} q̈_cmd·dt² ]

两者在【模型精确且无扰动】时数学等价（对双积分器，模型状态的 horizon 增量
就等于二次积分增量）；差别全在模型不精确时：

    经典 Smith   偏差按 t 无界累积
    本形式       偏差只在 horizon 内累积，上界 ½·a_err·T²，每拍清零

用实测的等效任务扰动 d_eff = 0.411 m/s²（从 hw03 闭环数据反解，配置里已记）
算这个上界：T = 25 ms 时位置预测误差 ½·0.411·0.025² = **0.128 mm**。
经典 Smith 在同样条件下 1 秒就漂 ½·0.411·1² = 206 mm。

⚠️ 还有一个次要但决定性的好处：β=0 时本形式【逐位退回当前控制器】
（q̂ ≡ q_meas，q̇̂ ≡ q̇_meas）。经典 Smith 没有这个性质，做不出安全分档。


==========================================================================
2  用什么推：已经算出来的 q̈_cmd，不新增任何 pinocchio 调用
==========================================================================
控制回路是 Python @200 Hz、5 ms 预算，今晚实测丢拍 0.08~0.83%、p99 周期
5.2 ms、最大间隙 15.4 ms。曾经因为可视化标记每拍重建折线把周期挤爆
（丢拍 1.07%、p99 10 ms，操作者直接看到"一顿一顿"）。所以预测器必须便宜。

``whole_task_smc_ros_runner`` 每拍本来就已经算出：

    rr.qdd_cmd      resolve_acceleration 解出的指令关节加速度
    torque_scale    统一饱和缩放系数

而 RNEA 给出 ``τ = M(q)·q̈_cmd + h``、``τ_bias = h``，所以
``τ − τ_bias = M·q̈_cmd`` 精确成立，饱和缩放之后生效的加速度就是

    q̈_eff = torque_scale · rr.qdd_cmd          ← 一次乘法，零额外动力学调用

实测本模块的每拍开销 **4.3 µs**（6 关节、回溯 ≤8 拍），占 5 ms 预算的 0.09%。

三个必须显式处理的建模选择：

* **不把 τ_ff（静摩擦前馈）算进预测。** 它按设计是去抵消真实摩擦的，净加速度
  贡献理想为零；算进去等于预测一段不会发生的运动。代价是"ff 抵消不完的那部分"
  变成预测误差 —— 已包含在上面的 d_eff 里。
* **必须乘 torque_scale。** 不乘的话饱和时预测器以为机械臂在按未饱和的指令走。
* **不用 LESO 的扩张状态外推。** 那是另一条路线（C/D），误差来源不同：
  本模块的误差来自"模型不知道扰动"，LESO 的误差来自"扰动估计本身有噪声与滞后"。
  ⚠️ 见下方第 5 节 —— 两者【应该叠加】，Smith 单用有一个结构性缺陷。


==========================================================================
3  预测多久：两条路【分开】补，而且都不依赖那个证据弱的 T_p
==========================================================================
速度路 T_v = 25 ms、位置路 T_p = 10 ms 是两条不同的通路，分开补：

  * 速度路 horizon = T_v = 25 ms，锚点是电机自报速度
  * 位置路 horizon = T_p = 10 ms，锚点是最新位置测量

**为什么不共用一个锚点。** 试过"锚在最旧时刻（t−T_v）、两条路一起推 25 ms"，
它的好处是完全不用 T_p（只用 T_v−T_p 去对齐缓冲下标，误差 <1 个编码器计数）。
但它有个致命性质：β<1 时位置路的等效滞后【劣于】现状（用 t−T_v 的位置而不是
t−T_p 的），退不回基线，做不出安全分档。分路补则 β=0 逐位等于现状。

**T_p 那 10 ms 的弱证据不影响结论**（数值扫描，模型 A、β_v=1、β_p=0、
对 Δ=±0.3/±0.5 取最坏）：

    T_p =  8 ms   基线 c1_max 50.2   补偿后最坏 208.6
    T_p = 10 ms   基线 c1_max 50.0   补偿后最坏 189.2
    T_p = 15 ms   基线 c1_max 50.0   补偿后最坏 174.6
    T_p = 20 ms   基线 c1_max 50.7   补偿后最坏 176.3

T_p 在 8~20 ms 之间变化，收益只动 ±10%。原因是**位置路根本不是约束方**：
单独补位置路（β_v=0, β_p=1）的最坏 c1 是 34.1，比不补的 35.6 还低。
⚠️ 结论：**只补速度路就够了**，位置路补不补是二阶效应。这条正好把方案与
那个 r=0.43 的弱数据【解耦】。

轨迹很慢也帮了大忙：跟踪段关节参考速度 0.0033~0.029 rad/s，一拍 5 ms 走
1.5e-4 rad，比编码器量化步长 3.81e-4 rad 还小。所以 T_p 判错一整拍造成的
位置对齐误差 <1 个计数 ≈ 末端 0.045 mm。


==========================================================================
4  ⚠️ 模型误差敏感性：定量结果
==========================================================================
把不确定性挂在【被控对象】上（真实增益 G = g·(1+Δ)，预测器只有名义模型 g），
数值解特征方程

    s² + G·c1·s·W_v(s) + G·c0·W_p(s) = 0
    W_v = e^{−sT_v} + μ_v(1 − e^{−sT_v}),                 μ = β/(1+Δ)
    W_p = e^{−sT_p} + β_p T_p s[e^{−sT_v} + μ_v(e^{−sT_p}−e^{−sT_v})]
          + μ_p(1 − e^{−sT_p} − sT_p e^{−sT_p})

沿 α 族（λ,k,η 同比例缩、φ 不变 -> c0 = 0.24·c1²）二分求最大 c1。

【引擎先自校验两个独立锚点，都精确复现】
    旧增益 c1=68.4 c0=1010.9 -> σ=+9.98/s, 10.40 Hz   （配置/测试记 +9.98, 10.40）
    姿态通道 G_ff 临界附加增益 -> 739 1/s²             （配置注释记 739）

【c1 上限，只补速度路 β_p=0，取 Δ∈{0,±0.3,±0.5} 的最坏】

    模型 A (T_v=25ms, T_p=10ms, g=1.0)     模型 B (T_v=10.5ms, g=2.2)
    β_v=0.0   35.6   ×1.00                 43.1   ×1.00     <- 基线
    β_v=0.2   43.2   ×1.21                 54.6   ×1.27
    β_v=0.3   48.4   ×1.36                 63.1   ×1.46
    β_v=0.4   54.9   ×1.54                 75.0   ×1.74
    β_v=0.5   63.3   ×1.78                 93.0   ×2.16
    β_v=0.6   74.5   ×2.09                123.4   ×2.86
    β_v=0.8  112.0   ×3.14                284.8   ×6.60
    β_v=1.0  189.2   ×5.31                445.6   ×10.33

⚠️【一个意外的验证】模型 A 在 Δ=±0.5 下的基线上限 **35.6**，而实物实测
   Λ=21 那组 c1=36、Λ=25 那组 c1=35.5 【都当场触发看门狗】。也就是说：
   一旦不再假设被控对象增益精确、允许它有 ±50% 不确定，这套解析判据就
   **不再高估**（Δ=0 时它给 50，高估 40%）。这是本次分析最有价值的副产品：
   以后用这个判据必须【带 Δ=±0.5 取最坏】，不能用 Δ=0 那一列。

【危险方向是 Δ>0，即实物比模型更"强"】
   模型 A、β_v=1：Δ=+0.5 -> 189.2；Δ=−0.5 -> 1000+（搜索上界）。
   而模型 B 的 g=2.2 说的正是"实物比名义模型强 2.2 倍" —— 属于危险方向。
   所以上表一律取最坏值，不用 Δ=0。

【horizon 判错（Smith 的第二个经典失效）——本结构对它不敏感】
   模型 A、β_v=1、真实 T_v=25 ms，预测器却用 T̂_v：

       T̂_v/T_v   0.40   0.60   0.80   1.00   1.20   1.40   1.60   2.00
       最坏 c1    54.4   75.5  118.3  189.2  147.3   86.7   61.2   41.4
       ×基线      1.53   2.12   3.32   5.31   4.14   2.44   1.72   1.16

   **T̂ 错到 2.5 倍都还优于不补。** 原因是本形式的修正项是指令的有限差分
   （一个超前项），不是靠不稳定并联模型去对消延迟。β_v=0.5 时更平：
   T̂_v/T_v 从 0.4 到 2.0，收益在 ×1.21~×1.87 之间，谷底极宽。
   欠补比过补安全（0.8 -> ×3.32 vs 1.2 -> ×4.14 是特例；两端 0.4 -> ×1.53
   vs 1.6 -> ×1.72 基本对称），取 T̂_v = 25 ms 且不外加一拍 ZOH 余量。

   真实 T_v 若不是 25 ms（预测器仍按 25 ms 补）：
       真实 20 ms  基线 44.5 -> 补偿后 168.6  ×3.79
       真实 25 ms  基线 35.6 -> 补偿后 189.2  ×5.31
       真实 30 ms  基线 29.7 -> 补偿后 113.4  ×3.82

【那个没有模型的 24.8 Hz 模态】
   它属于"模型在某频率处幅值与相位都错了"。上表的 Δ 是实数增益偏差，覆盖不到
   相位。另做了复数 δ 的最坏摄动计算（δ 允许任意相位，等价于一般乘性不确定）：
   模型 A 全补时，把闭环推到虚轴所需的最小 |δ| 随 c1 变化 ——
       c1= 30 -> 1.43 | 50 -> 0.90 | 80 -> 0.60 | 100 -> 0.50 | 150 -> 0.37
   而不补时上限由延迟本身定死在 c1=50（模型 A）。两者相交在 |δ|≈0.88。
   ⚠️ **所以本方案的成立条件是：24.8 Hz 处的相对模型误差 < 0.88。**
   这个数【现在无法验证】，因为那个模态根本没有模型。这是本方向最大的未知，
   也是必须从 β_v=0.3 起步分档、而不是直接 β_v=1 的唯一理由。

【摩擦前馈那条隐式位置环增益 G_ff 反而被松绑】
   姿态通道（约束方）能容纳的附加位置增益，取 Δ∈{0,±0.3,±0.5} 最坏：
       无预测      621 1/s²   当前 G_ff=425 占 68.4%   ⚠️ 比配置记的 57.5% 还紧
       只补速度路   900 1/s²   占 47.2%
       全补 β=1   2144 1/s²   占 19.8%
   即预测把 G_ff 的预算放大 1.45~3.5 倍。这条很重要：G_ff 是压那 2.429 mm
   动摩擦残余的主力，原先它已经吃掉三分之二的裕度，没有余地再加。


==========================================================================
5  ⚠️⚠️ Smith 单用有一个结构性缺陷：稳态扰动抑制会【变差】
==========================================================================
这是本设计里最容易踩、也最容易被误读为"方案无效"的一点。

稳态（等速跟踪 + 常值等效扰动 d）解析解：

    无预测                |e| = d/c0 + ẋ_ref·T_p
    Smith                 |e| = (d/c0)·[1 + β_v·c1·T_v + β_p·c0·T_p·(T_v+T_p/2)]
    Smith + 理想扰动观测   |e| = d/c0

中间那个 ``1 + c1·T_v`` 的惩罚项来源很直接：预测器用【指令】加速度积分，
而指令里没有 d，所以它把速度低估了 d·T_v。控制器于是以为机械臂比实际慢，
多给一份 c1·d·T_v 的加速度 —— 恰好推着扰动走。

代入 d_eff = 0.411 m/s²、模型 A（数值，单位 mm）：

    c1    c0     无预测    Smith(β=1)   Smith+理想扰动观测
     30   216     2.103      3.453          1.903
     40   384     1.270      2.264          1.070
     50   600     0.885      1.665          0.685
     60   864     0.676      1.313          0.476
     80  1536     0.468      0.926          0.268
    120  3456     0.319      0.599          0.119

⚠️ **在 c1 不变（=30）的情况下上机，Smith 会让准静态误差从 2.10 mm 变成
   3.45 mm —— 比现在差 64%。** 追平现状需要 c1 ≥ 42.2（只补速度路 40.5）。
   所以【绝不能先只开预测、不动增益地跑一次"对照组"】：那次一定更差，
   而且会被读成"Smith 没用"。预测与提增益必须同一次改动一起上。

   （模型 B 下惩罚小得多：c1=30 时 2.57 mm vs 2.11 mm，追平只需 c1=33.7。
     因为惩罚项 ∝ T_v，模型 B 的 T_v 只有 10.5 ms。）

=> 这直接给出路线优先级：**D（LESO + Smith）> E（Smith 单用）**。
   LESO 提供 d̂，把 ``1 + c1·T_v`` 里的 d 换成残差 d−d̂；只要 LESO 精度到 30%，
   惩罚项就缩到 1/3。到 1.0 mm 所需的 c1：
       Smith 单用（全补）74.9 | Smith 只补速度路 68.0 | Smith+理想扰动观测 41.4
   本模块把 ``disturbance_accel`` 留成入参就是为了以后接 LESO，不用改结构。


==========================================================================
6  为什么这不会重蹈 50 Hz 低通的覆辙
==========================================================================
2026-08-03 第 12 次上机：速度反馈加 50 Hz 低通，存活 0.47 -> 0.35 s，更早发散。
降噪 ~30% 却花掉 26.6° 相位 —— 证明该回路【相位受限】而非噪声受限。

本模块在这条轴上是【反号】的：它不动噪声底（测量原样进来，增益恒为 1），
只加一个超前项。白噪声下闭环噪声增益的数值结果（模型 A，相对基线 c1=30 归一）：

    β_v=0 β_p=0 c1= 30   位置 ×1.000   指令 ×1.000     <- 基线
    β_v=1 β_p=0 c1= 30   位置 ×0.926   指令 ×0.858     <- 同增益下噪声反而更低
    β_v=1 β_p=1 c1= 30   位置 ×0.954   指令 ×0.858
    β_v=0 β_p=0 c1= 45   位置 ×1.214   指令 ×2.186     <- 只提增益不补偿
    β_v=1 β_p=0 c1= 56   位置 ×0.706   指令 ×1.557     <- 补偿 + 提更多增益
    β_v=1 β_p=0 c1= 80   位置 ×0.611   指令 ×2.165

读法：要拿到同样的指令噪声（≈×2.2），不补偿只能到 c1=45，补偿能到 c1=80。
位置噪声则一路下降。**低通是拿相位换噪声，预测是拿噪声（几乎不花）换相位。**

⚠️ 唯一的噪声通道是位置路的死推项 β_p·T_p·q̇̂，它把速度噪声折进位置估计
（闭环分子从 c1 变成 c1 + c0·β_p·T_p，当前 30 -> 32.2，+7%）。这也是
默认 ``beta_position = 0.0`` 的第二个理由。


==========================================================================
7  安全边界
==========================================================================
预测出错时的判据（全部在本模块内，不依赖调用方）：

1. **限幅**。逐关节钳住修正量：
       |q̇̂ − q̇_meas| <= max_velocity_correction
       |q̂  − q_meas | <= max_position_correction
   缺省值的来历：动力学一致伪逆的 J̄ 行范数腕部达 11.07 /m，跟踪段任务加速度
   量级 |ν| ≈ 1 m/s²，25 ms 内合法的速度修正上限 ≈ 11.07 × 1 × 0.025
   = 0.28 rad/s。取 [0.20,0.20,0.20,0.50,0.50,0.50]（大臂行范数只有 3.3~4.9）。
   位置修正取 0.005 rad ≈ 13 个编码器计数 ≈ 末端 1.5 mm，与当前误差同量级。

2. **连续限幅即退回**。连续 ``fallback_after_clamped_ticks`` 拍触顶就把
   β 锁成 0（= 逐位退回当前控制器）并置 ``tripped``，需显式 ``reset()`` 才恢复。
   缺省 10 拍 = 50 ms。理由：单拍触顶可能是一次饱和或一次丢帧，连续 50 ms
   触顶意味着模型与实物系统性不一致，此时预测量已不可信。

3. **非有限值直通**。q̈_cmd 缓冲里出现 NaN/Inf 时该拍不预测，返回测量值原样。

4. **看门狗必须继续用【测量值】，不能用预测值。** 预测器错的时候，看门狗是
   唯一的兜底；拿预测值喂它等于让被测对象自己判自己。协议无使能/失能帧、
   软件层无急停通道，急停只能物理断电 —— 兜底不能有第二个失效点。

5. **摩擦前馈的驱动量（tracking.friction_feedforward 的 dq_err）第一阶段
   继续用测量误差。** G_ff = 425 1/s² 是个隐式位置环增益，把预测误差喂进去
   会被放大 425 倍：限幅上限 0.005 rad ≈ 末端 1.5 mm，×425 = 0.64 m/s²，
   是实测等效扰动 d_eff=0.411 的 1.5 倍。而第 4 节已算出预测【放宽】了 G_ff
   的预算（621 -> 900），所以这里不需要动它就能得到好处。

6. **分档上机**。β_v 从 0.3 起（模型 A 给 ×1.36，模型 B ×1.46），确认存活与
   抖动后再 0.5、0.7。⚠️ 每一档都必须同时把 c1 抬到该档允许值的一半以内，
   否则会落进第 5 节那个"比现在差"的坑。


==========================================================================
⚠️ 本模块【只做估计】，不改控制律、不改增益、不碰配置里 position/orientation
   那 15 组实测出来的最优点。缺省 ``enabled=False`` 且 β 全零 -> 逐位直通，
   已验证的基线逐字节不变。
=========================================================================="""

from __future__ import annotations

from collections import deque

import numpy as np

__all__ = [
    "SmithPredictorConfig",
    "JointStatePredictor",
    "DEFAULT_MAX_VELOCITY_CORRECTION",
    "DEFAULT_MAX_POSITION_CORRECTION",
]

# 见模块 docstring 第 7 节第 1 条。
DEFAULT_MAX_VELOCITY_CORRECTION = (0.20, 0.20, 0.20, 0.50, 0.50, 0.50)
DEFAULT_MAX_POSITION_CORRECTION = 0.005


class SmithPredictorConfig:
    """预测器参数，带取值校验。

    ``beta_velocity`` / ``beta_position`` 是【预测强度】∈[0,1]：
    0 = 逐位退回当前控制器，1 = 按名义模型全额补偿。分档上机用它，
    不要去改 horizon（horizon 是实测的物理量，β 才是设计余量）。
    """

    __slots__ = ("enabled", "n_joints", "velocity_horizon_sec",
                 "position_horizon_sec", "beta_velocity", "beta_position",
                 "max_velocity_correction", "max_position_correction",
                 "fallback_after_clamped_ticks")

    def __init__(
        self,
        n_joints: int,
        *,
        enabled: bool = False,
        velocity_horizon_sec: float = 0.025,
        position_horizon_sec: float = 0.010,
        beta_velocity: float = 0.0,
        beta_position: float = 0.0,
        max_velocity_correction=DEFAULT_MAX_VELOCITY_CORRECTION,
        max_position_correction=DEFAULT_MAX_POSITION_CORRECTION,
        fallback_after_clamped_ticks: int = 10,
    ) -> None:
        n = int(n_joints)
        if n <= 0:
            raise ValueError(f"n_joints 必须为正，收到 {n_joints}")
        self.n_joints = n
        self.enabled = bool(enabled)

        for name, value in (("velocity_horizon_sec", velocity_horizon_sec),
                            ("position_horizon_sec", position_horizon_sec)):
            v = float(value)
            if not np.isfinite(v) or v < 0.0:
                raise ValueError(f"{name} 必须是非负有限值，收到 {value}")
            setattr(self, name, v)

        for name, value in (("beta_velocity", beta_velocity),
                            ("beta_position", beta_position)):
            v = float(value)
            if not np.isfinite(v) or not (0.0 <= v <= 1.0):
                # β>1 是过补偿：第 4 节的 horizon 扫描显示两端都掉收益，
                # 而 β>1 还会把噪声一起放大，没有任何理由允许。
                raise ValueError(f"{name} 必须落在 [0,1]，收到 {value}")
            setattr(self, name, v)

        self.max_velocity_correction = self._limit_vector(
            "max_velocity_correction", max_velocity_correction, n)
        self.max_position_correction = self._limit_vector(
            "max_position_correction", max_position_correction, n)

        ticks = int(fallback_after_clamped_ticks)
        if ticks < 0:
            raise ValueError(
                f"fallback_after_clamped_ticks 不能为负，收到 {fallback_after_clamped_ticks}")
        self.fallback_after_clamped_ticks = ticks

    @staticmethod
    def _limit_vector(name, value, n):
        arr = np.asarray(value, dtype=float).ravel()
        if arr.size == 1:
            arr = np.full(n, float(arr[0]))
        if arr.size != n:
            raise ValueError(f"{name} 需要 {n} 个值或 1 个标量，收到 {arr.size}")
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"{name} 含非有限值: {arr}")
        if np.any(arr < 0.0):
            raise ValueError(f"{name} 不能为负: {arr}")
        return arr

    @property
    def active(self) -> bool:
        """真正会改变输出的条件：开关打开【且】至少一路 β 非零。"""
        return self.enabled and (self.beta_velocity > 0.0
                                 or self.beta_position > 0.0)

    def describe(self) -> str:
        return (
            "Smith 预测: %s  速度路 β=%.2f horizon %.1f ms，位置路 β=%.2f horizon %.1f ms；"
            "限幅 Δq̇<=%s rad/s、Δq<=%s rad；连续 %d 拍触顶即退回"
            % ("启用" if self.active else "关闭（逐位直通）",
               self.beta_velocity, 1000.0 * self.velocity_horizon_sec,
               self.beta_position, 1000.0 * self.position_horizon_sec,
               np.array2string(self.max_velocity_correction, precision=2),
               np.array2string(self.max_position_correction, precision=4),
               self.fallback_after_clamped_ticks))


class JointStatePredictor:
    """把滞后的 (q_meas, qd_meas) 往前推，返回控制器该用的那一份。

    用法（每个控制周期恰好一次，与 ``_refresh_controller_feedback`` 同一位置）::

        predictor.push_command(torque_scale * rr.qdd_cmd, dt)   # 本拍指令生效后
        q_hat, qd_hat = predictor.predict(q_meas, qd_meas)      # 下一拍开头

    ⚠️ 顺序很重要：``push_command`` 记的是【已经发出去】的指令。本拍还没发的
    指令不能进缓冲，否则预测器会用到未来信息，在实物上表现为凭空多出的超前。
    """

    def __init__(self, config: SmithPredictorConfig) -> None:
        self.config = config
        self.n_joints = config.n_joints
        # 缓冲深度：按最大 horizon 加余量。丢拍时单拍 dt 会变大（实测最大间隙
        # 15.4 ms），所以按【秒】累加而不是按拍数截断，深度只是内存上界。
        depth = int(max(config.velocity_horizon_sec,
                        config.position_horizon_sec) / 0.005 + 0.5) + 8
        self._buf: deque = deque(maxlen=max(depth, 8))
        self._clamped_ticks = 0
        self.tripped = False
        self.last_diagnostics = {}

    # -- 状态 ---------------------------------------------------------------

    @property
    def active(self) -> bool:
        return self.config.active and not self.tripped

    def reset(self) -> None:
        """清空缓冲并解除退回锁存。切换阶段（交接、看门狗复位）时调用。"""
        self._buf.clear()
        self._clamped_ticks = 0
        self.tripped = False
        self.last_diagnostics = {}

    # -- 输入 ---------------------------------------------------------------

    def push_command(self, qdd_cmd, dt: float) -> None:
        """记录【已发出】的这一拍指令关节加速度与它作用的时长。

        ``qdd_cmd`` 必须是饱和缩放【之后】的有效加速度
        （= ``torque_scale * rr.qdd_cmd``）。不乘 torque_scale 的话，饱和时
        预测器会以为机械臂在按未饱和的指令走 —— 而饱和恰恰发生在误差最大、
        最需要预测正确的时刻。
        """
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            # 时钟异常时不入缓冲：宁可少补，也不能把错误的 dt 乘进积分。
            return
        a = np.asarray(qdd_cmd, dtype=float).reshape(self.n_joints)
        self._buf.append((a.copy(), dt))

    # -- 输出 ---------------------------------------------------------------

    def _integrate(self, horizon: float):
        """回溯累加最近 ``horizon`` 秒的指令，返回 (Σa·dt, ΣΣa·dt²)。

        ⚠️ 按【秒】累加而非按拍数：控制回路实测丢拍 0.08~0.83%、最大间隙
        15.4 ms，用固定拍数会在丢拍时把 horizon 算成 25~75 ms 之间的随机值。
        延迟是物理量（传输 + 电机内部估计器），单位是秒，不随丢拍变化。

        二次项用【梯形/中点】口径：一段 dt 内加速度恒定，对位置的贡献是
        ½a·dt² 再加上这段之后已累积速度的 dt 倍。从最新往回走时等价于
            ΣΣ = Σ_k [ ½a_k dt_k² + v_after_k · dt_k ]
        其中 v_after_k 是第 k 段结束后（更晚那些段）累积的速度增量。
        """
        dv = np.zeros(self.n_joints)
        dx = np.zeros(self.n_joints)
        elapsed = 0.0
        for a, dt in reversed(self._buf):
            if elapsed >= horizon:
                break
            step = min(dt, horizon - elapsed)
            # dx 先加"这一段之后已有速度"走过的距离，再加本段自身的 ½a·dt²
            dx += dv * step + 0.5 * a * step * step
            dv += a * step
            elapsed += step
        return dv, dx, elapsed

    def predict(self, position, velocity):
        """返回 (q̂, q̇̂)。未启用 / 已退回时**原样返回入参对象**（逐位直通）。"""
        cfg = self.config
        if not self.active:
            self.last_diagnostics = {"active": 0.0, "tripped": float(self.tripped)}
            return position, velocity

        q = np.asarray(position, dtype=float).reshape(self.n_joints)
        qd = np.asarray(velocity, dtype=float).reshape(self.n_joints)
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd))):
            # 反馈本身坏了：这不是预测器该处理的故障，原样透传给看门狗。
            self.last_diagnostics = {"active": 0.0, "feedback_nonfinite": 1.0}
            return position, velocity

        dv_v, _, span_v = self._integrate(cfg.velocity_horizon_sec)
        _, dx_p, span_p = self._integrate(cfg.position_horizon_sec)
        if not (np.all(np.isfinite(dv_v)) and np.all(np.isfinite(dx_p))):
            self.last_diagnostics = {"active": 0.0, "command_nonfinite": 1.0}
            return position, velocity

        # 速度路：q̇̂ = q̇_meas + β_v·Σa·dt
        corr_v = cfg.beta_velocity * dv_v
        # 位置路：q̂ = q_meas + β_p·[T_p·q̇̂ + ΣΣa·dt²]
        # 死推项用【预测后】的速度 q̇̂（含 β_v 修正）—— 位置路的 horizon 只有
        # 10 ms，而它要外推的那段时间里的真实速度更接近 q̇̂ 而不是 q̇_meas。
        qd_hat_raw = qd + corr_v
        corr_p = cfg.beta_position * (cfg.position_horizon_sec * qd_hat_raw + dx_p)

        clamp_v = np.clip(corr_v, -cfg.max_velocity_correction,
                          cfg.max_velocity_correction)
        clamp_p = np.clip(corr_p, -cfg.max_position_correction,
                          cfg.max_position_correction)
        hit = bool(np.any(clamp_v != corr_v) or np.any(clamp_p != corr_p))

        if hit:
            self._clamped_ticks += 1
            if (cfg.fallback_after_clamped_ticks > 0
                    and self._clamped_ticks >= cfg.fallback_after_clamped_ticks):
                # 连续触顶 = 模型与实物系统性不一致，预测量不可信 -> 锁存退回。
                self.tripped = True
                self.last_diagnostics = {
                    "active": 0.0, "tripped": 1.0,
                    "clamped_ticks": float(self._clamped_ticks)}
                return position, velocity
        else:
            self._clamped_ticks = 0

        self.last_diagnostics = {
            "active": 1.0,
            "tripped": 0.0,
            "clamped": 1.0 if hit else 0.0,
            "clamped_ticks": float(self._clamped_ticks),
            "velocity_span_sec": float(span_v),
            "position_span_sec": float(span_p),
            "max_velocity_correction_used": float(np.max(np.abs(clamp_v))),
            "max_position_correction_used": float(np.max(np.abs(clamp_p))),
        }
        return q + clamp_p, qd + clamp_v
