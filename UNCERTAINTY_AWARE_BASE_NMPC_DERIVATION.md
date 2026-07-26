# 把无人机基座建模为"有界不确定合作者"的末端精确控制——完整推导

> 2026-07-26。基于 10 篇顶刊顶会文献的精读提取（Fisac RSS18 / Fridovich-Keil IJRR20 /
> Hu IJRR23 / Lindemann RA-L23 / Luo IJRR23 / Haninger ICRA22 / Hakobyan-Yang T-RO21 /
> Raković TAC22 / Chen DCEE Automatica21 / Vahs RA-L23），统一到本项目符号体系。
> 现有基线：解析已知基座下 50Hz 二阶 NMPC + Smith 预测器 + 命令逆变换，whole 轨迹
> 末端 world RMS 0.27–0.67mm（见 MPC_TRACKING_HANDOFF_20260726.md §0）。

---

## 0. 记号

| 符号 | 含义 |
|---|---|
| $\xi_b=(p_b,R_b)\in SE(3)$ | 基座（无人机）真实位姿 |
| $\xi_b^*=(p_b^*,R_b^*)$ | 发给基座的命令位姿（**我们选择，精确已知**） |
| $\delta=(\delta_p,\delta_\theta)\in\mathbb{R}^6$ | 实现误差：$p_b=p_b^*+\delta_p$, $R_b=R_b^*\exp([\delta_\theta]_\times)$ |
| $q,\dot q,\ddot q\in\mathbb{R}^7$ | 机械臂关节状态 |
| $\varphi(q),\ J_v(q)$ | 臂基座系 EE 位置正运动学及其雅可比 |
| $p_E$ | EE 世界位置；$r(t)$ 任务参考；$\varepsilon_{\rm task}$ 任务容差 |
| $\Delta t=0.02$s, $N=10$ | 控制周期与 NMPC 时域 |
| $\psi$ | 基座闭环响应模型参数（增益/滞后/偏置/谐波响应） |
| $\beta$ | 模型置信度（Fisac 意义：模型此刻的可信程度，非基座性能） |
| $\hat\Delta_\tau$ | conformal 校准的 $\tau$ 步前瞻误差界（分位数） |

---

## 1. 系统建模

### 1.1 整机动力学与级联分解（为什么可以只商定"契约"）

整机广义坐标 $Q=(p_b,R_b,q)\in SE(3)\times\mathbb{R}^7$，标准拉格朗日形式

$$M(Q)\ddot Q + C(Q,\dot Q)\dot Q + g(Q) = B\,u,\qquad
u=(f_z, \tau_b, \tau_q)\in\mathbb{R}^{1+3+7},$$

基座 6 自由度只有 4 个输入（总推力 + 三轴力矩）→ **欠驱动**。但无人机自带飞控闭环：
位置/姿态环把基座动力学封闭成一个跟踪系统

$$\xi_b = \mathcal{G}_{\rm cl}[\xi_b^*] + w,$$

$\mathcal{G}_{\rm cl}$ 是闭环响应算子（有限带宽、有滞后），$w$ 是风扰/未建模项。
臂控制器**不直接接触推力**，接口只有命令 $\xi_b^*$——这正是"合作者"语义的物理来源：
我们不能控制队友的肌肉，只能给它指令并观察它的执行。

时标分离依据：飞控带宽（外环 ~2–5Hz）≪ 臂控制 50Hz；臂对基座的反作用力
$\lesssim m_{\rm arm}\,a_{\rm arm}$，作为 $w$ 的一部分被飞控闭环吸收（后续可作为
前馈告知飞控，见 §3.7）。

### 1.2 契约形式化（老师的 $x^*\pm\Delta x$）

**定义 1（执行契约）**：基座承诺对任意可行命令 $\xi_b^*(\cdot)$，实现误差分解为

$$\boxed{\ \delta(t) = \underbrace{\mu_\psi(t)}_{\text{可预测部分}} + \underbrace{w(t)}_{\text{残差}},\qquad
\mu_\psi = (\mathcal{G}_\psi - \mathrm{Id})[\xi_b^*],\quad \|w\|_{W} \le \Delta\ }$$

- $\mathcal{G}_\psi$：参数化响应模型。最小可用形式 = 线性参数化
  $$\delta_{t+1} = F(h_t)\,\psi + w_t,\qquad
  h_t = (\text{近期 } \xi^*_b \text{ 及其导数、上一拍 } \delta),$$
  $F(h_t)$ 的列可取：常值偏置基、一阶/二阶滞后响应基、基座激励频率处的
  $\sin/\cos$ 谐波基（正是 2Hz 螺旋工作中辨识过的执行链结构）。
- $\Delta$：**不是先验给定**，而是在线校准量（§2.3）。契约的"部分可信"就体现在
  $\Delta$ 会随基座表现张缩。

与 Fisac 设定的关键区别（也是我们的优势）：他们的"意图" $\theta$（人要去哪）未知需
推断；我们的 $\theta\equiv\xi_b^*$ **是我们自己发的**，完全已知——不确定性只剩
"执行得怎么样"，推断问题严格更简单。

### 1.3 EE 误差通道（扰动如何进入任务）

$p_E = p_b + R_b\,\varphi(q)$。小姿态误差一阶展开：

$$p_E \approx \big(p_b^* + R_b^*\varphi(q)\big) + \delta_p + [\delta_\theta]_\times R_b^*\varphi(q)
= \bar p_E(q) + L(q)\,\delta,$$

$$\boxed{\ L(q) := \big[\,I_3\ \ -[R_b^*\varphi(q)]_\times\,\big]\in\mathbb{R}^{3\times6},\qquad
\kappa(q):=\|L(q)\|_2 \le \sqrt{1+\|\varphi(q)\|^2}\ }$$

$\kappa(q)$ 是**杠杆增益**：臂伸得越长，基座姿态误差被放大得越厉害。它把"基座不
确定性"翻译成"末端不确定性"，也是后面冗余调度的直接优化对象。

---

## 2. 信任度建模（三层，三个数学对象）

> 直觉：$(\mu_\psi,\Sigma_\psi)$ = "我了解你的脾气"；$b(\beta)$ = "你现在还按脾气
> 来吗"；$\hat\Delta$ = "不管怎样，我给你的误差立一个有统计保证的界"。

### 2.1 参数层：$\psi$ 的共轭贝叶斯辨识（Hu et al. IJRR23, Lemma 1）

高斯先验 $\psi\sim\mathcal{N}(\mu_\psi,\Sigma_\psi)$，观测 $\delta_{t+1}=F_t\psi+w_t$,
$w_t\sim\mathcal{N}(0,\Sigma_w)$，闭式更新

$$\Sigma_\psi^+ = \big(\Sigma_\psi^{-1} + F_t^\top\Sigma_w^{-1}F_t\big)^{-1},\qquad
\mu_\psi^+ = \Sigma_\psi^+\big(F_t^\top\Sigma_w^{-1}(\delta_{t+1}-\bar f_t) + \Sigma_\psi^{-1}\mu_\psi\big).$$

注意 $\Sigma_\psi^+$ 依赖 $F_t$ 即依赖**我们发的命令**——这是 §3.6 主动试探
（dual control effect, Hu Thm 1）的数学入口：激励越丰富，$\Sigma_\psi$ 收缩越快。

### 2.2 置信层：$\beta$ 滤波（Fisac RSS18 Eq.3/5 + FK IJRR20 Eq.4 连续观测版）

Fisac 的 Boltzmann 观测模型 $P(u|x;\beta)\propto e^{\beta Q}$ 在连续观测下取
$Q=-\tfrac12\|\delta_t-F\mu_\psi\|^2_{\Sigma_{\rm pred}^{-1}}$，即**β 缩放精度的高斯**：

$$P(\delta_t \mid h_t;\beta) = \mathcal{N}\!\big(\delta_t;\ F_t\mu_\psi,\ \Sigma_{\rm pred}/\beta\big),
\qquad \Sigma_{\rm pred}=F_t\Sigma_\psi F_t^\top+\Sigma_w.$$

$\beta\!\to\!\infty$：完全信模型；$\beta\!\to\!0$：似然趋平，模型不提供信息。
离散网格 $\beta\in\{\beta_1..\beta_M\}$（~10 个对数均匀点），带遗忘的 HMM 滤波
（FK Eq.4-5）：

$$\boxed{\ b^+(\beta)\ \propto\ P(\delta_t|h_t;\beta)\,\big[(1-\epsilon)b(\beta)+\epsilon\,p_0(\beta)\big]\ }$$

$\epsilon$-遗忘保证置信度**跌得快也恢复得了**（阵风/故障后几拍内 $\beta$ 崩到低位，
恢复正常后重建）。信任标量取 $\bar\beta_t=\mathbb{E}_b[\beta]$ 或 $P(\beta\ge\beta_{\min})$。

**置信加权预测**（Fisac Eq.6 的对应物）：未来基座误差的预测分布是 $\beta$-混合：
高 $\beta$ 分量集中于 $F\mu_\psi$ 附近；低 $\beta$ 分量展宽到基座**一步可达集**
$\mathcal{R}_{\Delta t}$（由基座速度/加速度上限界定，FK §7：$\beta\to0$ 极限
恰好恢复前向可达集）。这实现"模型失效时预测自动变保守"。

### 2.3 界校准层：$\hat\Delta_\tau$ 的 conformal 构造（Lindemann RA-L23 + ACI）

非一致性得分（$\tau$ 步前瞻预测误差，按通道加权）：

$$s_t(\tau) := \big\|W_L\big(\delta_{t+\tau} - \hat\delta_{t+\tau|t}\big)\big\|,\qquad
\hat\delta_{t+\tau|t} = F_{t+\tau|t}\,\mu_\psi\ (\text{后验均值预测}).$$

滑窗校准集 $D_{\rm cal}=\{s_{t-i}(\tau)\}_{i=1}^{n}$，取分位数（Lindemann Lemma 1）：

$$\boxed{\ \hat\Delta_\tau = s^{(p)}(\tau),\quad p=\lceil (n+1)(1-\bar\alpha)\rceil,\quad
\bar\alpha = \alpha/N\ (\text{时域联合界, Thm 1})\ }$$

**覆盖保证**（可交换性下，有限样本、分布无关）：
$P\big(\|\delta_{t+\tau}-\hat\delta_{t+\tau|t}\|\le\hat\Delta_\tau,\ \forall\tau\in\{1..N\}\big)\ge 1-\alpha$。

分布漂移（基座进入新工况）由 **ACI** 在线修正（Gibbs–Candès；Luo IJRR23 引荐）：

$$\alpha_{t+1} = \alpha_t + \gamma\,(\alpha^* - \mathrm{err}_t),\qquad
\mathrm{err}_t=\mathbb{1}\{s_t>\hat\Delta\},$$

长期覆盖率收敛到 $1-\alpha^*$ 而无需分布假设。**这是"信任侵蚀与重建"的有保证版本**。

工程细节：校准窗口 $n\approx 100\text{–}500$（2–10s@50Hz）；$\hat\Delta$ 更新做速率
限制（每拍变化 $\le\rho_\Delta$），供 §4.2 递归可行性使用；落笔段与抬笔段分开
校准（不同激励分布，保护可交换性——Luo 的 Mondrian 分组思想）。

---

## 3. 不确定性感知 NMPC

### 3.1 预测替换（Smith 预测器的推广）

现有代码用解析 $\xi_b(t)$ 生成逐级基座系参考；改为**后验均值预测**

$$\hat\xi_{b,\tau|t} = \xi^*_{b}(\tau)\ \oplus\ F_{\tau|t}\,\mu_\psi ,$$

即把"臂侧执行链 Smith 预测器"的同一模式复制到"基座通道"。可预测部分
（偏置、滞后、谐波响应）**进前馈**，只有真正的残差交给鲁棒层——这直接复用
2Hz 螺旋修复的方法论：先消系统性失配，再给随机部分立界。

### 3.2 硬约束收紧（homothetic tube 形式，Raković TAC22）

任务约束 $\|p_E(\tau)-r(\tau)\|\le\varepsilon_{\rm task}$。代入 §1.3 误差通道，
对 $\|W_L\tilde\delta\|\le\hat\Delta_\tau$ 的一切实现：

$$\boxed{\ \big\|\bar p_E(\tau)-r(\tau)\big\| \ \le\ \varepsilon_{\rm task} - \kappa(q_\tau)\,\hat\Delta_\tau\ }$$

这正是 Raković 收紧式 $c^\top z_k+\alpha_k h_S(c)\le1$ 的任务空间形态：
$\hat\Delta_\tau$ 扮演同伦缩放 $\alpha_k$（截面随前瞻深度增长），$\kappa(q)h_S$ 扮演
支持函数。**关键工程选择**（Raković 迁移笔记）：$\mathbb{W}$ 取**一步（20ms）不可
预测增量**的界，而非全幅 $\pm\Delta x$ 包络——基座位姿 50Hz 可测，误差的低频部分
已被 $\mu_\psi$ 前馈吸收，tube 只需覆盖高频残差，保守性可低一个量级。

关节侧同理：为管内反馈律 $\pi=v+\nu(y-z)$ 预留修正权限，
$\ddot q$ 预算拆分 $\ddot q_{\max} = \ddot q_{\rm nom} + \|\nu\|\,\hat\Delta_1/\Delta t^2$
（把一部分已放开的硬件加速度余量指定给"替队友擦屁股"）。

### 3.3 概率软化（二选一/组合，按数据可得性）

**(a) 高斯头**（GP-MPC 模板, Haninger ICRA22 Eq.18）：用 $\Sigma_{\rm pred}$ 时

$$\|\bar p_E-r\| \le \varepsilon_{\rm task} - \mathrm{erf}^{-1}(1-2\epsilon)\,
\sqrt{L\,\Sigma_{\rm pred}\,L^\top}.$$

**(b) DR-CVaR 头**（Hakobyan–Yang T-RO21）：保留样本缓冲 $\{\hat w^{(i)}\}_{i=1}^{N_s}$，
Wasserstein 球半径挂钩信任 $\theta(\bar\beta)$（$\bar\beta$ 低 → 球大），约束

$$\sup_{\mu: W(\mu,\hat\nu)\le\theta(\bar\beta)}\ \mathrm{CVaR}_\alpha^\mu\big[\text{任务超差损失}\big]\le0,$$

经 Kantorovich 对偶化为有限维凸约束
$z+\frac{1}{1-\alpha}\big[\lambda\theta+\frac1{N_s}\sum_i s_i\big]\le\delta_\ell$ +
逐样本线性不等式（Prop.1），可直接进 acados 外的 QP 层或作离线设计工具。
首选路线 **conformal(§2.3)+tube(§3.2)**：无分布假设、实现最轻；(a)(b) 作消融。

### 3.4 可行性退化阶梯（安全屏蔽，Hu Eq.15 的最小限制形式）

当 $\varepsilon_{\rm task}-\kappa\hat\Delta_\tau < \varepsilon_{\rm solver}$（求解器可达
残差）时按序降级，带迟滞防抖：

1. **时间缩放**：路径速度 $\times\sigma\in(0,1]$（代码已有
   `online_time_scaling` 钩子）——需求加速度 $\propto v^2$，最先松弛；
2. **构型优化**：切换到低 $\kappa(q)$/高操纵度姿态参考（§3.5）；
3. **抬笔保持**：$\dot r=0$ 且 EE 退到安全高度——这是控制不变的退避集
   （屏蔽定理的落点：$x_0\in\Omega\Rightarrow x_t\in\Omega$）。

### 3.5 信任驱动的冗余调度（7-DOF 零空间）

任务占 3(+2) 自由度，零空间维数 ≥2。姿态参考由信任状态调度：

$$q_{\rm ref}(\bar\beta) = \arg\min_{q\in\mathcal{N}(\text{task})}\
\underbrace{\kappa(q)^2\,\hat\Delta^2}_{\text{杠杆暴露}}
+ \frac{w_m}{\sigma_{\min}(J_v(q))^2}
+ w_j\,\mathrm{dist}^2(q,\ \text{关节余量中心}),$$

信任高 → 权重回到舒适姿态（现 `reset_nominal` 行为）；信任低 → 收臂降 $\kappa$、
避奇异保修正权限。实现即现有 `posture_reference` + 权重参数的在线调度，
NMPC 结构不动。

### 3.6 主动试探（DCEE 项，仅抬笔段激活）

DCEE 精确分解（Chen Thm 1）：$J = \|\text{exploitation}\|^2 + P_{k+1|k}$（未来信息
条件下的后验方差）。对应到基座命令设计：抬笔过渡段在契约允许幅度内叠加微激励
$\xi^*_b \leftarrow \xi^*_b + a\sin(\omega_j t)$，选 $a,\omega_j$ 最小化

$$J_{\rm probe} = J_{\rm transition} + \mathrm{tr}\big(W_E\,\Sigma_\psi^+(a,\omega)\big),\qquad
\Sigma_\psi^+ \text{ 由 §2.1 更新式对候选激励前推},$$

即"用最小的额外晃动买最多的队友信息"（Hu Thm 1 的 dual control effect）。
落笔段禁用（不能拿画图精度换信息）。

### 3.7 协商层（合作者语义闭环，慢速率 1–5Hz）

基座参考再规划：给定任务 $r(t)$ 与当前信任 $(\mu_\psi,\hat\Delta)$，选 $\xi_b^*$
最小化臂的需求，同时尊重基座自身能力：

$$\min_{\xi_b^*}\ \int \big\|W_a\,\ddot\varphi_{\rm req}(t)\big\|^2 dt
\quad\text{s.t.}\quad \ddot\varphi_{\rm req} = R_b^{*\top}\big(\ddot r - \ddot p_b^* - \widehat{\ddot\delta}\big),\ \
\xi_b^*\in\text{基座可行集}.$$

频域直觉：基座接低频大位移，臂接高频小残差（互补滤波分工）；信任降级时分工
向臂侧移动（配合 §3.4 降速）。另将臂反作用力前馈 $\hat F_{\rm arm}=m_{\rm arm}\ddot p_{\rm com}$
发给飞控，缩小 $w$ 本身。

---

## 4. 保证

### 4.1 复合精度证书

条件：(C1) conformal 覆盖成立（可交换/ACI 长期版）；(C2) NMPC 每拍可行且名义
残差 $\le\varepsilon_{\rm solver}$；(C3) 臂侧执行链误差 $\le\varepsilon_{\rm exec}$
（实测 ~0.1mm，命令逆变换已验证）。则

$$P\Big(\|p_E(\tau)-r(\tau)\|\le \varepsilon_{\rm solver}+\kappa\hat\Delta_\tau+\varepsilon_{\rm exec}
\ \ \forall\tau\in\{1..N\}\Big)\ \ge\ 1-\alpha .$$

（把 §3.2 收紧后的名义约束与通道界相加即得；与 Fisac 的"概率预测 × 最坏情况
跟踪"复合证书同构：FaSTrack 的 $\mathcal{E}$ ↔ 我们的 $\varepsilon_{\rm exec}$。）

### 4.2 递归可行性（草图）

采用 Raković 候选解构造：平移上一拍解 + 终端项。需两个附加条件：
(i) $\hat\Delta$ 拍间变化 $\le\rho_\Delta$（§2.3 的速率限制），收紧量的扰动进入
候选解的富余度；(ii) 终端集取抬笔退避集（控制不变，§3.4 梯级 3 恒可行）。
则可行性在梯级切换语义下递归保持：NMPC 不可行 ⇒ 落入更低梯级，而梯级 3
恒可行。（严格证明留论文附录；机制同 Raković Prop.3 + 屏蔽 Thm 2。）

### 4.3 精度预算表（设计公式）

$$\text{EE 误差} \le \underbrace{\varepsilon_{\rm solver}}_{0.1\text{–}0.2\,\rm mm}
+ \underbrace{\varepsilon_{\rm exec}}_{\approx0.1\,\rm mm}
+ \underbrace{\kappa(q)\,\hat\Delta}_{\text{设计变量}} .$$

例：目标 1mm ⇒ 允许 $\kappa\hat\Delta\approx0.7$mm ⇒ $\kappa\!\approx\!1.4$（臂展
0.5m、收臂姿态）时需 $\hat\Delta\lesssim0.5$mm ⇒ 响应模型须吃掉基座扰动的
~90%（当前 2Hz 工况下 Smith+逆变换链已证明此量级可达）。**这个公式反向给出
"对基座提什么契约要求"**：给定精度目标与姿态，可解出可容忍的 $\Delta$。

---

## 5. 落地路线（映射到现有代码）

| 阶段 | 内容 | 代码落点 |
|---|---|---|
| P0 | 契约仿真器：解析基座 + AR(1) 色噪 + 阵风脉冲 + "抽风"故障注入 | `moving_base_trajectory_config.py` 加扰动项；`hifi_offline_loop.py` 扩展 |
| P1 | 信任模块（纯观测，shadow）：$\psi$ 递推 + $\beta$ 滤波 + conformal $\hat\Delta$，全部进 loop CSV | 新文件 `base_trust_estimator.py`（~200 行，无 ROS 依赖） |
| P2 | 收紧接入：基座预测换 $\hat\xi_b$；守卫 `r_allow`←$\varepsilon-\kappa\hat\Delta$；posture 权重按 $\bar\beta$ 调度 | `move_base_circle_mpc_ik_demo.py`（参数已具备：guard_residual_abs_m、posture 权重） |
| P3 | 退化阶梯：$\hat\Delta$ 触发 time-scaling / 姿态切换 / 抬笔 | `online_time_scaling` 钩子 + 新梯级状态机 |
| P4 | 试探与协商：抬笔段微激励；基座参考再规划接口 | 契约仿真器先行，真机基座后接 |

**实验矩阵**（论文用）：扰动谱 {低频漂移, 2Hz 谐波, 宽带, 突变故障} ×
方法 {忽略 δ, 纯鲁棒(固定 Δ), 纯自适应(μ_ψ 无界), 本方法(三层)} ×
指标 {EE RMS/P95, 覆盖率实测 vs 1−α, 降级触发次数, 恢复时间}。
预期卖点曲线：固定 Δ 在小扰动时过保守（无谓降速）、纯自适应在故障时超差，
三层方法两端都占优。

## 6. 与最近邻工作的差异（论文 claim 备用）

1. vs Fisac/FK confidence-aware：伙伴从"不可指挥的人"变为"可指挥的载具"——
   意图 $\theta$ 精确已知，且我们**主动设计激励**（§3.6）与**再协商命令**（§3.7），
   置信推断严格更强；任务从米级避碰变为毫米级输出调节。
2. vs 空中操作鲁棒系（ESO/TASE23,24）：基座不再是被压制的扰动，而是带契约、
   带信任状态、可协商的合作者；且给出有限样本覆盖保证而非仅渐近收敛。
3. vs tube/DR-MPC：$\mathbb{W}$/球半径不是离线常数，而是由双层信任
   （$\beta$ 滤波 + conformal/ACI）在线校准并带保证地张缩。
4. vs trust-aware HRC（NeuTRL 等）：trust 不是启发式标量，而是
   （后验、置信、分位界）三元组，每一元都有明确的数学语义和更新定理。
