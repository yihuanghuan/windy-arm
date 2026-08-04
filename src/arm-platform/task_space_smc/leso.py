"""关节速度的线性扩张状态观测器（LESO / ADRC 观测器本体）。

只做一件事：**从量化的位置反馈估计关节速度**，取代电机自报速度。
不改控制律、不改增益、不做扰动补偿（扩张态 z3 只输出、不回灌）。

==========================================================================
为什么要有它：速度路的 15 ms 是全部裕度的来源
==========================================================================
2026-08-03 第 15 次上机直连实测（同一份数据、两种测法）：

    关节   力矩 -> 位置二阶导        力矩 -> 速度一阶导
     j4     2 拍 = 10.0 ms  r=+0.434   5 拍 = 25.0 ms  r=+0.857
     j6     2 拍 = 10.0 ms  r=+0.450   5 拍 = 25.0 ms  r=+0.906

速度信号比位置信号多滞后 3 拍 = 15 ms —— 电机内部速度估计器的群延迟。
位置路 10 ms 是传输/执行延迟，谁也去不掉；那 15 ms 是**可以不要的**，
只要我们自己从位置估速度。24.8 Hz 处这 15 ms 值 133.9°。

⚠️ 位置那 10 ms 的相关系数只有 0.43/0.45，**没有坐实**；25 ms 那个
   （r=0.86/0.91）才是硬数。整套收益的前提是"速度比位置多滞后 15 ms"，
   这一条由 25 ms 与 10 ms 之差给出，见下方【风险】。

==========================================================================
⚠️ 凭什么不重蹈 velocity_filter 的覆辙
==========================================================================
2026-08-03 第 12 次上机否定了反馈路低通：截止 50 Hz，降噪 ~30% 却在 25 Hz
花掉 26.6° 相位，存活 0.47 -> 0.35 s。结论是**该回路相位受限、不是噪声受限**。
所以任何"加滤波器"的方案都必须先回答：你的相位从哪来。

答案是**三阶 LESO 的低频相位是三次抵消的，不是一次的**。
把 n 阶观测器的"速度估计 / 真速度"闭式写出来（三重极点 s=-ω_o）：

    H_n(s) = 1 - s^(n-1)·(s + n·ω_o) / (s + ω_o)^n

    n=2:  H = ω²/(s+ω)²                     低频相位 ≈ -2·(Ω/ω)     【一次 = 低通】
    n=3:  H = ω²(3s+ω)/(s+ω)³               低频相位 ≈ -8·(Ω/ω)³    【三次】
    n=4:  H = (6ω²s²+4ω³s+ω⁴)/(s+ω)⁴        低频相位 ≈ +4·(Ω/ω)³    【超前】

二阶 LESO 就是一个二阶低通，和 velocity_filter 同一个陷阱。三阶把分子那个
零点 3ω 放进去，恰好抵掉三重极点的前两阶相位 —— 这就是差别所在，不是调参
调出来的，是阶数带来的结构性质。

实际数字（200 Hz 精确离散，ω_o=200 rad/s，已与时域逐点核对到 1e-4）：

    频率        1Hz    3Hz    5Hz   10Hz   15Hz   24.8Hz   30Hz
    本 LESO   -0.01  -0.26  -1.12  -6.74 -16.04  -36.45  -46.05   deg
    50Hz 低通 -1.15  -3.43  -5.71 -11.31 -16.70  -26.38  -30.96   deg
    电机自报  -5.40 -16.20 -27.00 -54.00 -81.00 -133.92 -162.00   deg  <- 要替掉的
    净收益    +5.39 +15.94 +25.88 +47.26 +64.96  +97.47 +115.95   deg

⚠️ 诚实地说：**15 Hz 以上本 LESO 的相位比那个被否定的低通还大**
   （24.8 Hz：36.45° vs 26.38°）。孤立地比这两个数没有意义，因为两者在
   回路里的位置根本不同：

     低通是【叠加】—— 26.38° 加在电机自报的 133.92° 上，总共 160.3°，
                       净变化 −26.4°，纯亏，所以实测更早发散。
     LESO 是【替代】—— 36.45° **取代** 133.92°，净变化 +97.5°。

   这就是本方案与 velocity_filter 的结构性差别：一个往相位账上加，一个把
   账上最大的那一笔换掉。5 Hz 以下 LESO 的相位还比低通小 5~100 倍（1 Hz
   处 0.01° vs 1.15°），10 Hz 处仍小 1.7 倍 —— 而抖动指标带就在 0.8~6 Hz。

==========================================================================
⚠️ 电机自报速度是 12 bit 量化的（2026-08-04 从实机 CSV 反解，新发现）
==========================================================================
此前的判断"j1/j2/j3 速度噪声 ≈ 0.0000，大臂反馈很干净"是**静止态的假象**。
从 3 份实机记录（065819 / 184404 / 225550）逐关节检验自报速度的取值集合，
三份独立给出同一个结论：自报速度落在均匀网格上。

    j1/j2/j3   LSB = 0.0097675 rad/s = 40/4095   -> ±20 rad/s 满量程 12 bit
    j4/j5/j6   LSB = 0.0244179 rad/s = 100/4095  -> ±50 rad/s 满量程 12 bit

（判据：逐关节取唯一值集合，相邻差全部是最小间隔的整数倍，残差 <0.004。）

对照本轨迹的**参考**关节速度：

    大臂 0.013~0.029 rad/s  =  1.3 ~ 3.0 个速度 LSB
    腕部 0.0033~0.0133 rad/s = 0.14 ~ 0.54 个速度 LSB   <- 不足一格

也就是说腕部的速度反馈**在数值上无法表示本轨迹的参考速度**，只能报 0 或 ±0.0244。
而"大臂噪声为零"实际是"静止时停在同一个码上"—— 184404/225550 两份记录里
j1/j2/j3 位置 22~38 s 内一个 LSB 都没动，自报速度却恒为 -0.00488 rad/s
（正好一个码的偏置）。这不是干净，是**分辨率不够 + 有偏**。

纯量化给出的 RMS 下限：大臂 0.00282、腕部 0.00705 rad/s。
本 LESO 在末端抖动指标带（0.8~6 Hz）的输出噪声是 0.00035 / 0.00069 rad/s ——
**比自报速度的量化下限还低 4~10 倍**。所以"大臂电机噪声接近零、不该换"
这个理由不成立，六个关节应当用同一套方案。

==========================================================================
设计决定
==========================================================================
1. 阶数 = 3（位置、速度、扩张的总扰动）。
   * 不能用 2 阶：2 阶对**常值**扰动的稳态速度偏置是 2d/ω_o。本机静摩擦折合
     的关节加速度 d = f_s/M_eff = 4.8~10.2 rad/s²（j2 最大 9.0，j4 10.2），
     ω_o=200 时偏置 0.048~0.10 rad/s —— 是参考速度的 3~30 倍，直接压垮信号。
     3 阶的扩张态吸收常值扰动，稳态速度误差恒为 0。
   * 不用 4 阶：4 阶多跟一阶扰动导数，而实测误差分解里 **86% 的方差在 0.3 Hz
     以下**（准静态摩擦残余），3 阶的常值吸收已经够；4 阶把噪声传递再抬一档
     （|H| 峰值 1.23 -> 1.44）换来的只是 3 阶已经拿到的东西。
   * 3 阶对**斜坡**扰动的残差 3·ḋ/ω_o²：0.3 Hz、幅值 9 rad/s² 时是 0.0012 rad/s
     （ω_o=200），可忽略。

2. 带宽 ω_o = 200 rad/s（31.8 Hz）**六个关节同一个值**。
   * 下界由闭环裕度定，不是由噪声定。把 LESO 的真实传递函数代进现有判据
     s² + c1·s·H(s)·e^(-s·T_p) + c0·e^(-s·T_p) = 0：
         模型A(T_v=25,T_p=10,g=1)   基线增益裕度 ×5.72、G_ff 预算 739
                                     ω_o=80 时 ×5.82/1034（≈持平，无收益）
                                     ω_o=200 时 ×8.92/1032（+56% / +40%）
         模型B(T_v=10.5,T_p=10.5,g=2.2) 基线 ×6.41、G_ff 预算 1957
                                     ω_o=100 时 σ=+1.56 **发散**
                                     ω_o=200 时 σ=-13.97、×3.93、G_ff 2055
     => **ω_o < 120 在模型 B 下会发散**，ω_o=200 是 G_ff 预算在两个模型下
        都不低于基线的最小值。（G_ff 预算 739 这个数与既有判据完全吻合，
        是本计算与团队既有结论的对齐锚点。）
   * 上界由噪声定，但这个约束**很松**：0.8~6 Hz 带内噪声几乎与 ω_o 无关
     （0.00035~0.00057 rad/s，ω_o 从 100 到 250 只变 1.6 倍），因为 LESO 在
     低频就是理想微分器，量化噪声在那里被微分特性压掉。ω_o 真正换的是
     20~30 Hz 的噪声（0.0033 -> 0.0058），而即便 ω_o=250 那也仍低于电机
     自报的 0.0107~0.0128。**在 100~250 全区间内，本 LESO 在两个关心的
     频带上都比电机自报安静。**
   * 大臂与腕部**不分开**：LESO 只读位置，六轴位置量化同为 0.000381 rad，
     输出噪声与关节无关；30 倍的大臂/腕部不对称完全存在于电机的速度估计器
     里，而那正是被丢弃的东西。且位置通道（要靠抬 c0 压掉 2.95 mm 的那个）
     由大臂主导（|Jv| j1/j3 = 0.17/0.24 vs 腕部 0.05/0.08/0.00），只换腕部
     等于没动到约束方。

3. 离散化 = **精确 ZOH 预测 + 当前观测器校正**，不是前向欧拉。
   三重连续极点 s=-ω_o 映射到离散极点：
       前向欧拉  1-ω_o·dt     ZOH  exp(-ω_o·dt)     双线性 (2-ω_o dt)/(2+ω_o dt)
   ω_o=200、dt=5 ms 时分别是 0.000 / 0.368 / 0.333 —— 欧拉此时已经踩在
   deadbeat 上，dt 再大一点（丢拍）就出单位圆：dt=10 ms 给 -1.0，dt=15.4 ms
   （实测最大间隙）给 -2.08，**发散**。ZOH 的极点 exp(-ω_o·dt) 对任意 dt
   恒在 (0,1)，无条件稳定 —— 这正是丢拍要的性质。
   预测矩阵 [[1,dt,dt²/2],[0,1,dt],[0,0,1]] 对双积分链是**精确**的（b0=0
   时没有输入项），所以 ZOH 这里不带任何近似。
   校正用**当前观测器**（先预测再用本拍的 y 校正），不是预测观测器 ——
   后者会再赔进一整拍 5 ms，把要省的 15 ms 花掉三分之一。
   闭式增益（β = exp(-ω_o·dt)，已与 Ackermann 数值解核对到 1e-10）：
       l1 = 1 - β³        l2 = 1.5/dt·(1-β)²(1+β)      l3 = (1-β)³/dt²

4. 丢拍 = **用实际 dt，增益跟着 dt 重算**。
   增益是 dt 的函数（上面三个闭式），所以"用实际 dt"与"增益跟着变"是同一件事，
   不是两个选项。合成信号实测（真速度 0.020 rad/s，一次长拍）：
       长拍       跟 dt 变      固定标称 dt
       2×5ms      0.0193        0.0192   rad/s 峰值误差
       3×5ms      0.0192        0.0289
       4×5ms      0.0192        0.0313
   固定 dt 的误差随长拍长度线性增长，跟 dt 变的则平掉。
   ⚠️ 更重要的一条：**本观测器挂在 /joint_states 回调上，不挂在控制周期上。**
      实测丢拍 0.08~0.83%、最大间隙 15.4 ms 是**控制循环**（Python 调度）的
      现象，串口上行帧率实测 200.7 Hz、校验错 0 —— 传感器流本身没有断。
      挂在回调上就根本见不到那些长拍，还顺带处理了 runner 里
      "_joint_state_cb 覆盖 self.q、控制循环丢掉中间采样"的问题。
      （对比：loop_delay 的 FeedbackDelayModel 必须挂在控制周期上，因为它的
      单位就是"拍"。两者性质不同，不要照搬。）
   dt 取 /joint_states 的 header.stamp 之差（student_arm_node.cc:364 用
   this->now() 打的时标，比订阅侧收包时刻少一层调度抖动），夹在
   [0.2·T, 4·T]；超过 resync_sec 视为真掉线，重置 z1=y 并保留 z2/z3。

5. **不用模型**（b0 = 0，纯运动学双积分链）。
   ADRC 的常规建议是把已知的 RNEA 项作为 b0·u 喂进去让扩张态少干活。这里
   反过来，理由是硬件的静摩擦工况：
   * 第 16 次上机实测跟踪段净力矩 |Δτ| 中位数 [0.453,0.369,0.480,0.081,0.124,0.051] Nm，
     静摩擦 [0.725,1.775,0.625,0.125,0.100,0.100] Nm —— **六轴里四轴的指令
     力矩低于静摩擦，超过静摩擦的时间占比 j1/j2/j4/j6 全是 0%**。
     此时真实 q̈ ≈ 0，而 b0·τ_cmd = 0.369/0.197 = 1.87 rad/s²（j2）。喂进去
     等于让扩张态去追一个**比真扰动更大、更快**的假信号，纯亏。
   * 3 阶 LESO 对常值扰动的稳态速度误差本来就是 0，而 86% 的扰动方差在
     0.3 Hz 以下。模型项要替扩张态省掉的正是这一段 —— 扩张态免费就能吃掉。
   * 喂 b0·u 会把控制器自己的输出（含切换项与摩擦前馈的过零翻转）引到反馈
     估计里。第 17 次上机已经因为一条"瞬时正反馈"（前馈驱动量取净力矩）
     当场剧烈抖振、操作者断电 —— 不要再造第二条控制器 -> 估计器的通路。
   * 顺带省掉 5 ms 预算里的一次 M⁻¹(τ−h)。本模块每拍每关节约 25 flop + 1 次
     exp，六轴向量化后是微秒量级。

6. 本模块**不碰摩擦前馈的隐式增益 G_ff**。
   跟踪段前馈的驱动量是 q̇_ref + error_gain·Δq（whole_task_smc_ros_runner.py
   的 tracking_friction 分支），取的是**参考**速度与任务误差，不是实测速度。
   所以 G_ff = J·M⁻¹·diag(scale·f_s/κ)·J⁻¹ 的值不变，仍是 425 1/s²。
   变的是它的**预算**：见上面第 2 条，739 -> 1032（模型A）/ 1957 -> 2055（模型B），
   即 G_ff 从占用 57.5% 降到 41%。这是净收益，不是新风险。

==========================================================================
⚠️ 风险与它的证伪方式
==========================================================================
* 整套收益的前提是"速度比位置多滞后 15 ms"。若实际不是（模型 B 就假定
  T_v ≈ T_p ≈ 10.5 ms），LESO 只剩下自己的相位滞后，是**纯亏**：模型 B 下
  增益裕度 6.41 -> 3.93。ω_o=200 是在这个前提被推翻时损失最小、且 G_ff
  预算仍不低于基线的取法，但它救不了"前提本来就错"。
  证伪方法：上机后拿同一份 CSV 重做那个互相关（力矩 -> 位置二阶导 /
  力矩 -> 速度一阶导）。若换成 LESO 后两条路的滞后合并成同一个数，前提成立。
* 合成信号假设位置反馈除量化外没有额外噪声。实机 184404/225550 跟踪段实测
  逐拍 Δq 的 p99 只有 1 个 LSB、中位数 0 个，与"纯阶梯、几乎无 dither"一致；
  但那两段是保位状态。真正跟踪时若位置本底噪声远大于此，本模块输出噪声
  按比例线性放大 —— 上机第一件事应当是量 Δq 的分布。
* ω_o→∞ 时本 LESO 退化成位置裸后向差分，即**路线 B 是路线 C 的端点**，
  不是两个独立架构。建议把 B 与 C 合并成一次 ω_o ∈ {120, 200, 300, ∞} 的
  扫描，省一组上机。
* 扩张态 z3 是每关节的总扰动加速度估计，**本模块只输出、绝不回灌**。
  回灌就是 ADRC 的扰动抑制回路，会直接改变环路增益，必须作为**另一条独立
  路线**单独上机 —— 与本次混在一起测就分不清是谁的功劳。
"""

from __future__ import annotations

import numpy as np

__all__ = ["JointLESO"]


class JointLESO:
    """逐关节三阶线性扩张状态观测器：量化位置 -> 速度 + 总扰动。

    状态 z = [ẑ_pos, ẑ_vel, ẑ_dist]，每关节独立。b0 = 0（不用模型，见模块头）。

    ``omega_o`` 逐关节给定（rad/s）；<= 0 表示该关节不用观测器。
    **全部 <= 0 时整个对象是逐位直通**：``__call__`` 原样返回传进来的那个
    速度对象（连拷贝都不做），所以没写这个配置的运行逐字节不变 —— 与
    ``FeedbackDelayModel`` / ``JointVelocityFilter`` 同一个约定。
    """

    def __init__(
        self,
        omega_o,
        n_joints: int,
        nominal_dt: float = 0.005,
        dt_min_ratio: float = 0.2,
        dt_max_ratio: float = 4.0,
        resync_sec: float = 0.05,
        b0=0.0,
    ) -> None:
        w = np.asarray(omega_o, dtype=float).ravel()
        if w.size == 1:
            w = np.full(int(n_joints), float(w[0]))
        if w.size != int(n_joints):
            raise ValueError(
                f"omega_o 需要 {n_joints} 个值或 1 个标量，收到 {w.size}")
        if not np.all(np.isfinite(w)):
            raise ValueError(f"omega_o 含非有限值: {w}")
        if np.any(w < 0.0):
            raise ValueError(f"omega_o 不能为负: {w}")
        nominal_dt = float(nominal_dt)
        if not np.isfinite(nominal_dt) or nominal_dt <= 0.0:
            raise ValueError(f"nominal_dt 必须为正，收到 {nominal_dt}")
        if not 0.0 < float(dt_min_ratio) < float(dt_max_ratio):
            raise ValueError(
                f"需要 0 < dt_min_ratio < dt_max_ratio，收到 "
                f"{dt_min_ratio} / {dt_max_ratio}")

        self.omega_o = w
        self.n_joints = int(n_joints)
        self.nominal_dt = nominal_dt
        self.dt_lo = float(dt_min_ratio) * nominal_dt
        self.dt_hi = float(dt_max_ratio) * nominal_dt
        self.resync_sec = float(resync_sec)
        # 关闭的关节 ω_o=0 -> β=1 -> 三个增益全 0 -> 观测器不动，输出由
        # passthrough 掩码直接换成入参。这样算术上也不会产生 NaN。
        self._active = self.omega_o > 0.0
        self._z = np.zeros((3, self.n_joints))
        self._primed = False

        # ------------------------------------------------------------------
        # b0：模型输入增益。**缺省 0.0 = 模块头第 5 条决定的那个"不用模型"**。
        #
        # ⚠️ 这条通路是【后加的、缺省关闭的】，加它不是因为改了主意，而是因为
        # 存在一条与第 5 条直接冲突的独立论证，必须让实物去裁决：
        #
        #   本模块（第 5 条）：实测跟踪段净力矩 |Δτ| 中位数
        #     [0.453,0.369,0.480,0.081,0.124,0.051] Nm 对静摩擦
        #     [0.725,1.775,0.625,0.125,0.100,0.100] Nm —— 六轴里四轴的指令力矩
        #     低于静摩擦，超过静摩擦的时间占比 j1/j2/j4/j6 全是 0%。此时真实
        #     q̈ ≈ 0 而 b0·u ≈ 1.87 rad/s²（j2）。**模型输入是一句谎话**，
        #     喂进去只会让 z3 去抵消它。
        #
        #   集成方设计：b0·u 一旦接上，速度估计对真速度的传函在
        #     "plant 恰为 q̈=b0·u" 时【恒等于 1、全频段零相位】——
        #       Z2 = (β2 s+β3)·sY/D + b0·U·s(s+β1)/D，代入 U=s²Y/b0 得 Z2 = sY。
        #     观测器增益只作用在【模型残差】上。反过来说，**模型输入不准时，
        #     增益作用在残差上的那条路就是全部相位**，本 LESO 退化成纯微分器，
        #     24.8 Hz 处 ω_o=200 花 36.5°、ω_o=15Hz 花 98°，而位置裸后向差分
        #     只花 22.3°（半拍）。集成方由此推出：**无模型输入时没有任何 ω_o
        #     能让 LESO 同时在相位和噪声上赢过裸差分**，即路线 C 被路线 B 支配。
        #
        # 两条论证不矛盾 —— 它们合起来说的是：模型输入准就赢，不准就输，而本
        # 机的模型输入准不准正是未知数。所以代码两条路都留，缺省走本模块原来
        # 的 b0=0，由实物做一次消融（C 与 "C 但 input:none"）分胜负。
        # 逐位保证：b0 全零或调用时不传 u -> 走的算术表达式与加这条通路之前
        # 完全相同（见 __call__ 里的 acc 分支）。
        # ------------------------------------------------------------------
        b = np.asarray(b0, dtype=float).ravel()
        if b.size == 1:
            b = np.full(self.n_joints, float(b[0]))
        if b.size != self.n_joints:
            raise ValueError(f"b0 需要 {n_joints} 个值或 1 个标量，收到 {b.size}")
        if not np.all(np.isfinite(b)):
            raise ValueError(f"b0 含非有限值: {b}")
        self.b0 = b
        self._has_input = bool(np.any(b != 0.0))

    # -- 查询 ---------------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return bool(np.any(self._active))

    @property
    def position_estimate(self) -> np.ndarray:
        return self._z[0].copy()

    @property
    def velocity_estimate(self) -> np.ndarray:
        return self._z[1].copy()

    @property
    def disturbance_estimate(self) -> np.ndarray:
        """扩张态 ẑ_dist [rad/s²]：每关节的总扰动加速度估计。

        ⚠️ 只用于记录与诊断。回灌进控制律就是 ADRC 的扰动抑制回路，会改变
        环路增益，必须作为独立路线单独上机 —— 见模块头【风险】最后一条。
        """
        return self._z[2].copy()

    def gains(self, dt: float) -> np.ndarray:
        """本采样间隔下的当前观测器增益 (3, n)。

        三重离散极点置于 β = exp(-ω_o·dt)：
            l1 = 1 - β³,  l2 = 1.5/dt·(1-β)²(1+β),  l3 = (1-β)³/dt²
        与 Ackermann 数值解一致（test_leso.py 逐点核对到 1e-10）。
        ω_o = 0 的关节 β = 1，三个增益全为 0。
        """
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt 必须为正的有限值，收到 {dt}")
        b = np.exp(-self.omega_o * dt)
        one_b = 1.0 - b
        return np.vstack([
            1.0 - b ** 3,
            1.5 / dt * one_b ** 2 * (1.0 + b),
            one_b ** 3 / (dt * dt),
        ])

    def discrete_pole(self, dt: float) -> np.ndarray:
        """离散三重极点 exp(-ω_o·dt)。恒在 (0,1] —— 对任意 dt 无条件稳定。"""
        return np.exp(-self.omega_o * float(dt))

    def response(self, frequency_hz, dt: float | None = None) -> np.ndarray:
        """速度估计 / 真速度 的精确离散频响（复数，逐关节）。

        用于把"这个滤波器到底花了多少相位"摆到台面上 —— 与
        ``JointVelocityFilter.phase_lag_deg`` 同样的用途。不进控制律。
        """
        dt = self.nominal_dt if dt is None else float(dt)
        f = float(frequency_hz)
        if f == 0.0:
            return np.ones(self.n_joints, dtype=complex)
        z = np.exp(2j * np.pi * f * dt)
        out = np.ones(self.n_joints, dtype=complex)
        Phi = np.array([[1.0, dt, 0.5 * dt * dt], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])
        L_all = self.gains(dt)
        for j in range(self.n_joints):
            if not self._active[j]:
                continue                       # 直通：估计值就是入参，H = 1
            L = L_all[:, j]
            A = (np.eye(3) - np.outer(L, [1.0, 0.0, 0.0])) @ Phi
            gy = z * (np.linalg.solve(z * np.eye(3) - A, L)[1])
            out[j] = gy / (2j * np.pi * f)
        return out

    def phase_lag_deg(self, frequency_hz, dt: float | None = None) -> np.ndarray:
        """给定频率处的相位滞后 [deg]（正数 = 滞后）。"""
        return -np.degrees(np.angle(self.response(frequency_hz, dt)))

    # -- 使用 ---------------------------------------------------------------

    def reset(self, position=None, velocity=None) -> None:
        self._z[:] = 0.0
        if position is not None:
            self._z[0] = np.asarray(position, dtype=float).reshape(self.n_joints)
        if velocity is not None:
            self._z[1] = np.asarray(velocity, dtype=float).reshape(self.n_joints)
        self._primed = position is not None

    def __call__(self, position, velocity, dt: float | None = None, u=None):
        """推进一拍，返回本拍的速度估计。

        ⚠️ **每收到一帧 /joint_states 调用一次**，不是每个控制周期调用一次。
        观测器是挂在传感器流上的连续时间滤波器，"拍"对它没有意义；而
        ``FeedbackDelayModel`` 恰好相反（它的单位就是控制拍）。见模块头第 4 条。

        参数
        ----
        position : 本帧的关节位置（量化后的原始测量值）
        velocity : 本帧电机自报速度。**只在观测器关闭的关节上使用**（直通），
                   以及首帧 priming 时作为初值。开启的关节完全不用它。
        dt       : 距上一帧的间隔 [s]。None 表示用标称周期。
        u        : 本步生效的模型输入（缺省 None = 不用模型，b0=0 那条路）。
                   只有 ``b0`` 非零【且】传了 u 才走模型通路；两者任一缺席，
                   算术表达式与本参数加入之前逐位相同。
                   调用方负责把 u 与位置测量对齐（指令发出到体现为关节加速度
                   之间有一段延迟，见 state_estimator 的 input_delay_sec）。

        返回
        ----
        速度估计 ndarray。整体未启用时**原样返回入参 velocity 对象**。
        """
        if not self.enabled:
            return velocity
        q = np.asarray(position, dtype=float).reshape(self.n_joints)
        v_in = np.asarray(velocity, dtype=float).reshape(self.n_joints)

        if not np.all(np.isfinite(q)):
            # 反馈损坏：不把 NaN 灌进状态，保持上一拍的估计。
            out = self._z[1].copy()
            out[~self._active] = v_in[~self._active]
            return out

        if not self._primed:
            # 首帧：位置直接采纳，速度用电机自报的那一份当初值（本机上电即
            # 静止，通常是 0；但不写死 0 —— 万一是热启动，写死会造成假冲击）。
            self._z[0] = q
            self._z[1] = np.where(np.isfinite(v_in), v_in, 0.0)
            self._z[2] = 0.0
            self._primed = True
            out = self._z[1].copy()
            out[~self._active] = v_in[~self._active]
            return out

        raw_dt = self.nominal_dt if dt is None else float(dt)
        if not np.isfinite(raw_dt) or raw_dt <= 0.0:
            raw_dt = self.nominal_dt
        if raw_dt > self.resync_sec:
            # 真掉线：位置可能已经跑出去很远，创新量没有意义。重新落位到
            # 测量位置，**保留** z2/z3 —— 本机运动很慢（0.003~0.03 rad/s），
            # 掉线前的速度仍是比 0 好的猜测；置 0 会造成一次假的减速冲击。
            self._z[0] = q
            out = self._z[1].copy()
            out[~self._active] = v_in[~self._active]
            return out

        step = float(np.clip(raw_dt, self.dt_lo, self.dt_hi))
        z0, z1, z2 = self._z
        # 预测：双积分链的 ZOH 转移矩阵。b0=0 时是【精确】的，无离散化误差；
        # 带模型输入时对"这一步内 u 恒定"（ZOH，正是实际的指令保持方式）
        # 同样精确 —— 加速度换成 z2 + b0·u 即可，转移矩阵结构不变。
        # ⚠️ acc 在缺省路径上【就是】z2 这个数组对象本身，下面三行的算术
        #    表达式与加入模型通路之前逐位相同。
        acc = z2
        if self._has_input and u is not None:
            acc = z2 + self.b0 * np.asarray(u, dtype=float).reshape(self.n_joints)
        p0 = z0 + step * z1 + 0.5 * step * step * acc
        p1 = z1 + step * acc
        p2 = z2
        # 校正：当前观测器（用本拍的 y，不赔额外一拍）。
        l1, l2, l3 = self.gains(step)
        innovation = q - p0
        self._z = np.vstack([
            p0 + l1 * innovation,
            p1 + l2 * innovation,
            p2 + l3 * innovation,
        ])
        out = self._z[1].copy()
        # 关闭的关节【逐位】直通，不走观测器的算术（那条路上增益虽为 0，
        # 但 p1 = z1 + step·z2 仍会累积浮点误差，对"不启用"这个承诺不干净）。
        out[~self._active] = v_in[~self._active]
        self._z[0][~self._active] = q[~self._active]
        self._z[1][~self._active] = v_in[~self._active]
        self._z[2][~self._active] = 0.0
        return out

    def describe(self) -> str:
        if not self.enabled:
            return "关节速度 LESO: 全部关闭（逐位直通）"
        f_list = (1.0, 10.0, 24.8)
        lag = {f: np.round(self.phase_lag_deg(f), 2).tolist() for f in f_list}
        return (
            "关节速度 LESO(3阶, ZOH+当前观测器): ω_o=%s rad/s (%s Hz)，"
            "离散极点 %s @%.1f ms，相位滞后 1Hz %s° / 10Hz %s° / 24.8Hz %s°"
            "（对照：电机自报的额外 15 ms 在 24.8 Hz 是 133.9°）"
            % (np.round(self.omega_o, 1).tolist(),
               np.round(self.omega_o / (2 * np.pi), 1).tolist(),
               np.round(self.discrete_pole(self.nominal_dt), 4).tolist(),
               1000.0 * self.nominal_dt,
               lag[1.0], lag[10.0], lag[24.8]))
