"""关节状态估计器：换掉反馈的【速度来源】，并可选地把滞后的状态往前推。

本模块是 ``leso.JointLESO``（三阶扩张状态观测器）与
``smith_predictor.JointStatePredictor``（测量锚定的有限时域外推）之上的
**组合层 + 配置解析层**。它自己不做新的滤波数学，只负责：

  1. 逐关节选速度来源：``motor`` / ``finite_difference`` / ``leso``
  2. 选位置来源：``measured`` / ``leso``
  3. 位置路与速度路【分开】的 Smith 预测
  4. 把这些接成一条与实物同构的代码路径，并在配置里可切换

⚠️ 本模块【只改估计】，不碰控制律、不碰增益。缺省（配置里没有 ``estimator:``
   段）是**逐位直通** —— ``__call__`` 原样返回传进来的那两个对象，连拷贝都不做，
   与 ``FeedbackDelayModel`` / ``JointVelocityFilter`` / ``JointLESO`` 同一个约定。


==========================================================================
为什么做这件事：2.95 mm 卡住的是【环路增益】，环路增益卡在【回路延迟】
==========================================================================
2026-08-04 通宵 15 组实物调参：精度卡在交接后 RMS 2.95~3.17 mm（目标 <1 mm），
末端 0.8~6 Hz 抖动 0.856~0.983 mm（目标 <0.35 mm）。闭环数据反解的四项误差
（RSS 占实测 90%）：

    动摩擦残余（与 sign(v) 相关）  2.429 mm   <- 主项
    非速度相关残余（重力/模型误差） 1.758 mm
    0.3~10 Hz 粘滑                 1.270 mm
    >10 Hz                         0.121 mm

86% 的方差在 0.3 Hz 以下 —— 是准静态摩擦残余，不是抖振。压它只能靠环路增益
c0 = (K+η/φ)·Λ，而 c0 被回路延迟锁死。**降延迟是唯一治本方向**，本模块就是
降延迟的那几个候选实现。

实测的两段延迟（2026-08-03 第 15 次上机直连，同一份数据两种测法）：

    关节   力矩 -> 位置二阶导        力矩 -> 速度一阶导
     j4     2 拍 = 10.0 ms  r=+0.434   5 拍 = 25.0 ms  r=+0.857
     j6     2 拍 = 10.0 ms  r=+0.450   5 拍 = 25.0 ms  r=+0.906

速度比位置多滞后 3 拍 = 15 ms，是【电机内部速度估计器】的群延迟。
⚠️ 位置那 10 ms 的 r 只有 0.43/0.45，**没有坐实**；25 ms（r=0.86/0.91）才是硬数。


==========================================================================
⚠️ 凭什么这不是又一个 velocity_filter（2026-08-03 第 12 次上机的否定结论）
==========================================================================
那次：速度反馈加 50 Hz 低通，降噪 ~30%，在 25 Hz 花掉 26.6° 相位，存活时间
0.47 -> 0.35 s，**更早发散**。结论是该回路【相位受限】而非噪声受限。所以任何
"加个滤波器"的方案都必须先回答：你的相位从哪来、跟谁比。

本模块三条路各自的答案（24.8 Hz 处，相位滞后越小越好）：

    电机自报速度（当前基线）  133.9°   <- 那多出来的 15 ms
    位置裸后向差分            22.3°    = 180·f·T，半拍
    LESO ω_o=200 rad/s        36.5°    （代码实测，见 test_state_estimator）
    ——— 被否定的 50 Hz 低通    26.6°    但它是【叠加】在 133.9° 上 => 净 −26.4°

关键区别是**叠加 vs 替代**：低通把 26.6° 加在电机自报的 133.9° 上，纯亏；
裸差分与 LESO 是把那 133.9° 整个【换掉】，净赚 111.6° / 97.4°。
这不是"更高级的滤波器"，是回路里位置不同。

代价在噪声上，而且是可量的（本模块 test 里用合成信号实测，位置量化
0.000381 rad @200 Hz、参考速度取实测的腕部 0.0133 rad/s）：

    速度估计的输出标准差 rad/s
      裸后向差分                0.0289
      LESO ω_o=2π·12 Hz         0.0066
      LESO ω_o=2π·15 Hz         0.0094
      LESO ω_o=2π·31.8 Hz(=200) 0.0239
      LESO ω_o=2π·40 Hz         0.0290   <- 到这里就与裸差分持平

    对照电机自报速度的【量化下限】（不是噪声，是分辨率）：
      j1~j3 LSB 0.009768 rad/s（40/4095，±20 满量程 12 bit）-> RMS 下限 0.00282
      j4~j6 LSB 0.024418 rad/s（100/4095，±50 满量程）      -> RMS 下限 0.00705


==========================================================================
⚠️⚠️ 一条【未解决的分歧】，必须由实物裁决，不要在代码里假装它已经解决
==========================================================================
两份独立设计对"路线 C（LESO）值不值得做"给出相反结论，两边的算都对：

  * LESO 侧：ω_o 的下界【由闭环裕度定，不由噪声定】。把 LESO 的真实传函代进
    既有判据 s²+c1·s·H(s)·e^(−sT_p)+c0·e^(−sT_p)=0（该求根器在基线上重现出
    团队已发布的 G_ff 预算 739，作为对齐锚点）：
        模型A(T_v=25,T_p=10,g=1.0)   基线 ×5.72 / G_ff 预算 739
                                     ω_o=80  -> ×5.82/1034（等于没收益）
                                     ω_o=200 -> ×8.92/1032
        模型B(T_v=10.5,T_p=10.5,g=2.2) 基线 ×6.41 / 1957
                                     ω_o=100 -> σ=+1.56 **发散**
                                     ω_o=200 -> σ=−13.97 / ×3.93 / 2055
    => **ω_o < 120 rad/s（19.1 Hz）在模型 B 下发散**，取 ω_o=200 rad/s。

  * 集成侧：**无模型输入（b0=0）的 LESO 就是一个纯微分器**，观测器增益的全部
    相位都花在残差通路上。要在噪声上赢裸差分（0.0289）需要 ω_o ≲ 17 Hz，
    而那里它要付 98°~122° 相位、裸差分只付 22.3°。
    => **没有任何 ω_o 能让无模型输入的 LESO 同时赢过裸差分**，路线 C 被 B 支配。

两者合起来的结论是：**路线 C 的全部价值押在 b0·u 这条模型通路上**，而这条
通路在本机是否成立又是未知的（第 16 次实测：六轴里四轴的指令力矩低于静摩擦，
超过静摩擦的时间占比 j1/j2/j4/j6 全是 0%，此时真实 q̈≈0 而 b0·u≈1.87 rad/s²
—— 模型输入是一句谎话）。

所以本模块：
  * ``leso.input`` 缺省 ``none``（LESO 设计方的取法），可切到
    ``commanded_acceleration`` 做**消融对照**；
  * ``bandwidth_hz`` < 19.1 Hz 时发警告（模型 B 发散区），但不禁止；
  * **不在代码里替实物做这个判断**。

⚠️ 与此对应的实验次序：**路线 B（位置裸差分）必须第一个跑，它是判别实验。**
   它零新参数、行为完全确定，而 CSV 新增的 ``joint_N_qd_meas_rad_s`` 与出口列
   ``joint_N_velocity_rad_s`` 的互相关能直接读出"速度路是否真的提前了 3 拍"：
     滞后降 3 拍 且 闭环有变化 -> 模型A 成立，C/D 值得投；
     滞后降 3 拍 但 闭环无变化 -> 模型B 成立，B/C 整条支线可砍，直接跳 D/E；
     滞后没降               -> T_v=25 ms 在当前接线下不成立，先查测量。
   三种结果都能砍掉或确认一整条支线 —— 这才值得占用串行的实物时段。


==========================================================================
⚠️ 增益被冻结的前提下，本模块的【预期 RMS 改善约等于零】
==========================================================================
降延迟不直接减小误差，它只是【允许】把 c0 抬上去，而 c0 由 λ/k/η/φ 决定，
那四个数是 15 组实测出来的最优点、本次不动。所以：

    预期 RMS 变化 ≈ 0（腕部可能小幅改善 —— 跟踪段腕部参考关节速度
                       0.0033~0.0133 rad/s 低于腕部电机速度噪声底
                       0.037~0.041 rad/s 3~10 倍，那条反馈当前基本是噪声）
    真正的读数是【裕度】：互相关滞后降了几拍、噪声、闭环阻尼指标

**实验记录表的主列必须是"滞后拍数 / 噪声 std / 阻尼"，RMS 只能当次列。**
否则一个【被预测到的】零结果会被读成"估计器没用"，把正确的方向否掉。
（对照：velocity_filter 那次是指标对了、方案错了；这次的风险恰好相反。）


==========================================================================
接在哪：``_refresh_controller_feedback()`` 之内、FeedbackDelayModel 的【下游】
==========================================================================
    plant 真值 -> FeedbackDelayModel（仿真信道：延迟+量化） -> 本估计器 -> 控制器
                  hardware 下全零直通                          两种模式同一条路径

三条理由，任何一条单独成立：

  1. **仿真必须与实物同构。** 放在 feedback_delay 的【上游】（作用在 plant 真值
     上）时，仿真里估计器会看到无延迟、无量化的完美信号 —— 等于让仿真替方案
     作弊。这与当初加位置量化要防的是同一个错误（不建量化的话，裸差分在仿真里
     会"假赢"，把一个实物上更差的方案送上机）。
  2. **模型输入 u 只在控制回路一侧存在。** ``/joint_states`` 回调里拿不到与本拍
     对齐的指令加速度。
  3. ``velocity_filter`` 现在就在回调里 —— 那是**反面先例**：它的等效 dt 一直是
     名义周期，与回调实际间隔无关。别复制它。

⚠️ 有一条【真实的代价】必须写下来：实测丢拍 0.08~0.83%、最大间隙 15.4 ms 是
   **控制循环**（Python 调度）的现象，串口上行帧率实测 200.7 Hz、校验错 0 ——
   传感器流本身没断。挂在控制周期上就会见到那些长拍，挂在回调上则见不到。
   我们仍然挂在控制周期上（理由 1 压倒性），代价由"用实际 dt + ZOH 离散"抵消：

     三重连续极点映射到离散极点，ω_o=200：
        dt        5 ms     10 ms    15.4 ms（实测最大间隙）
        前向欧拉  0.000   −1.000   −2.080   <- 出单位圆，**一次丢拍就打飞**
        ZOH       0.368    0.135    0.046   <- 恒在 (0,1)，无条件稳定
   ``JointLESO`` 用的就是精确 ZOH + 当前观测器，增益是 dt 的闭式函数，
   所以"用实际 dt"与"增益跟着 dt 变"是同一件事，不是两个选项。
   （集成侧设计里的 ``substep_omega_dt`` 是给前向欧拉打的补丁；ZOH 不需要它，
     这里没有这个键 —— 见 leso.py 第 3 条。）


==========================================================================
⚠️ 每拍开销：实测，比设计阶段估的贵一个数量级
==========================================================================
6 关节 @200 Hz，含 ``push_command``，min-of-5 × 40000 拍（本机整夜在跑实验，
测量噪声不小，给的是区间）：

    路线                      名义拍        含 0.83% 丢拍
    A  motor（直通）           1.3 µs        1.3 µs
    B  finite_difference      12 µs         12 µs
    B' 混合                    12 µs         12 µs
    C  leso(input=none)       34 µs         34 µs
    C' leso + 模型输入         40 µs         40 µs
    D  leso + Smith           80~130 µs     80~170 µs   <- 最贵
    E  motor + Smith          45~90 µs      45~90 µs

⚠️ 设计阶段估的是 0.09~6.3 µs，实测差一个数量级。原因不是算法，是 numpy 在
6 元素小数组上的 Python 调用开销：profile 显示没有单一热点，时间平摊在几十次
``np.all(np.isfinite(...))`` / ``np.clip`` / ``np.exp`` 的调用开销上
（每次 1~2 µs，一拍有几十次）。所以"这个算法很便宜"是对的，"这个实现很便宜"
不完全对。

对 5 ms 预算：最贵的路线 D 占 1.6~3.4%。对照 runner 里既有的每拍开销
（compute_kinematics 8.44 µs 每拍两次、lstsq 6×6 8.41 µs 每拍两次、
pin.rnea 0.92 / crba 1.35），它是同一量级里偏大的一项，**不是可忽略的**。

⚠️ 上机纪律：首次开路线 D/E 时必须同时看丢拍率与周期 p99（今晚基线是丢拍
0.08~0.83%、p99 5.2 ms、最大间隙 15.4 ms）。曾经因为可视化标记每拍重建折线
把周期挤爆（丢拍 1.07%、p99 10 ms，操作者直接看到"一顿一顿"）—— 那是数百
微秒量级的事件，路线 D 的 0.1 ms 比它小，但不像设计阶段以为的那样"完全不用管"。
判别实验路线 B 只有 12 µs，这一条不构成任何顾虑。
缓冲全部是 ``deque(maxlen=...)``，长跑 12 万拍单拍成本不上涨（已验证）。

==========================================================================
G_ff：摩擦前馈那条隐式位置环增益，本模块【不改变它的值，只放大它的预算】
==========================================================================
    G_ff = J·M⁻¹·diag(scale·f_s/κ)·J⁻¹,  κ = velocity_epsilon/error_gain
    当前最大特征值 425 1/s²，姿态通道（约束方）临界 739 1/s²（Δ=0），已占 57.5%。
    ⚠️ 带 ±50% 对象增益不确定取最坏时临界只有 621，占比其实是 68.4%。

它从来没进过 c1 = Λ+K+η/φ 的判据 —— 这就是解析裕度高估硬件的原因
（预测 σ=−11.8"很安全"的 Λ=25 实测直接触发看门狗）。

本模块不动它，因为跟踪段前馈的驱动量是 ``q̇_ref + error_gain·Δq``（取【参考】
速度与任务误差，不是实测速度），换速度源不改变 G_ff 的数值。变的是预算：
模型A 739->1032、模型B 1957->2055（LESO 路），Smith 只补速度路时姿态通道
621->900。这是净收益。

⚠️ 但预测量绝不能喂进 friction_feedforward 的 dq_err：G_ff=425 会把预测误差
   放大 425 倍（限幅上限 0.005 rad ≈ 末端 1.5 mm，×425 = 0.64 m/s²，是实测
   等效扰动 d_eff=0.411 的 1.5 倍）。第一阶段那里继续用测量误差。

==========================================================================
⚠️ 看门狗必须继续用【测量值】，不能用估计值/预测值
==========================================================================
协议无使能/失能帧，软件层无急停通道，急停只能物理断电 —— 兜底不能有第二个
失效点。拿估计值喂看门狗等于让被测对象自己判自己。

==========================================================================
⚠️ 实物那个 24.8 Hz 极限环仍然【没有任何模型能解释】
==========================================================================
纯延迟模型最高 10.4 Hz，速度路换一阶低通（τ=15 ms）最高 13.2 Hz。所以本模块
的一切裕度结论只能说"在两套已知不完备的模型下都不变坏"，不能说"一定不炸"。
两套参数必须都检验：(A) T_v=25ms, T_p=10ms, g=1.0  (B) T_v=10.5ms, T_p=10.5ms, g=2.2
首次上机仍按最短时长 + 看门狗全程盯。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from task_space_smc.core.leso import JointLESO
from task_space_smc.core.smith_predictor import (
    DEFAULT_MAX_POSITION_CORRECTION,
    DEFAULT_MAX_VELOCITY_CORRECTION,
    JointStatePredictor,
    SmithPredictorConfig,
)

__all__ = [
    "JointStateEstimator",
    "EstimatorPlan",
    "resolve_estimator",
    "VELOCITY_SOURCES",
    "POSITION_SOURCES",
    "PREDICT_SOURCES",
    "LESO_INPUTS",
    "MODEL_B_MIN_OMEGA_RAD_S",
    "MODEL_B_MIN_BANDWIDTH_HZ",
    "ESTIMATOR_KEYS",
]

VELOCITY_SOURCES = ("motor", "finite_difference", "leso")
POSITION_SOURCES = ("measured", "leso")
PREDICT_SOURCES = ("commanded_acceleration", "acceleration_estimate")
LESO_INPUTS = ("none", "commanded_acceleration")

# 模型 B（T_v=T_p=10.5 ms、有效增益 2.2）下 ω_o=100 rad/s 给出 σ=+1.56 —— 发散。
# 低于这条线就发警告。⚠️ 方向是反的：直觉上"低带宽更平滑更安全"，这里不成立。
MODEL_B_MIN_OMEGA_RAD_S = 120.0
MODEL_B_MIN_BANDWIDTH_HZ = MODEL_B_MIN_OMEGA_RAD_S / (2.0 * np.pi)   # ≈ 19.10 Hz

ESTIMATOR_KEYS = (
    "source",
    "position_source",
    "predict_position_sec",
    "predict_velocity_sec",
    "predict_source",
    "beta_position",
    "beta_velocity",
    "predict_clamp_rad",
    "predict_clamp_rad_s",
    "predict_fallback_ticks",
    "hold_on_invalid",
    "leso",
)
_LESO_KEYS = ("bandwidth_hz", "input", "b0", "input_delay_sec")


# --------------------------------------------------------------------------
# 解析辅助
# --------------------------------------------------------------------------

def _per_joint_choice(value, n, name, allowed):
    """把标量或 n 元素列表展开成逐关节的字符串数组，并校验取值。

    ⚠️ 逐关节是必须的，不是锦上添花。实测的物理不对称：
      * 电机自报速度噪声 j1~j3 = 0.0000 / j4~j6 = 0.041~0.044（30 倍）
      * 跟踪段参考速度   大臂 0.013~0.029 / 腕部 0.0033~0.0133（3~10 倍）
    两端处境相反：腕部的速度反馈当前基本是噪声（换掉稳赚），大臂看起来很干净。
    单一全局 source 必然是折中。
    """
    if isinstance(value, str):
        items = [value] * int(n)
    else:
        items = list(value)
        if len(items) == 1:
            items = items * int(n)
    if len(items) != int(n):
        raise ValueError(f"{name} 需要 {n} 个值或 1 个标量，收到 {len(items)}")
    out = []
    for i, item in enumerate(items):
        s = str(item).strip().lower()
        if s not in allowed:
            raise ValueError(
                f"{name}[{i}] = {item!r} 不是合法取值，可选 {list(allowed)}")
        out.append(s)
    return tuple(out)


def _as_vector(value, n):
    """转成 (n,) float 向量，**已经是的话原样返回同一个对象**。

    ⚠️ 不能无脑写 ``np.asarray(x, float).reshape(n)``：即便形状已经对，
    ``reshape`` 也会造出一个新的 view 对象，于是 ``out is inp`` 不成立。
    Smith 预测锁存退回之后承诺的是"逐位退回当前控制器"，那条路上必须保住
    对象恒等 —— 这与 ``FeedbackDelayModel`` 的直通承诺是同一个标准。
    """
    arr = np.asarray(value, dtype=float)
    return arr if arr.shape == (n,) else arr.reshape(n)


def _per_joint_float(value, n, name, minimum=None):
    arr = np.asarray(value, dtype=float).ravel()
    if arr.size == 1:
        arr = np.full(int(n), float(arr[0]))
    if arr.size != int(n):
        raise ValueError(f"{name} 需要 {n} 个值或 1 个标量，收到 {arr.size}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} 含非有限值: {arr}")
    if minimum is not None and np.any(arr < minimum):
        raise ValueError(f"{name} 不能小于 {minimum}: {arr}")
    return arr


# --------------------------------------------------------------------------
# 估计器本体
# --------------------------------------------------------------------------

class JointStateEstimator:
    """反馈通路出口 -> 控制器入口的关节状态估计。

    用法（每个控制周期恰好一次，与 ``_refresh_controller_feedback`` 同一位置）::

        q_ctrl, qd_ctrl = estimator(q_meas, qd_meas, dt)     # 拍首
        ...
        estimator.push_command(qdd_eff, dt)                  # 拍尾，指令【已发出】之后

    ⚠️ ``push_command`` 的时序很重要：它记的是【已经发出去】的指令。本拍还没发
    的指令进了缓冲，预测器就用到了未来信息，在实物上表现为凭空多出的超前。

    ⚠️ approach 段也必须调 ``push_command``。否则 30 s 的 approach 期间模型输入
    全是零，交接那一刻观测器正处在最差状态 —— 而交接恰好是当前 RMS 2.95~3.17 mm
    的起点。

    未启用时 ``__call__`` **原样返回入参对象**（逐位直通，连拷贝都不做）。
    """

    def __init__(
        self,
        n_joints: int,
        control_period: float,
        *,
        velocity_source=None,
        position_source: str = "measured",
        leso: JointLESO | None = None,
        predictor: JointStatePredictor | None = None,
        predict_source: str = "commanded_acceleration",
        leso_input: str = "none",
        leso_input_delay_sec: float = 0.0,
        hold_on_invalid: bool = True,
    ) -> None:
        n = int(n_joints)
        if n <= 0:
            raise ValueError(f"n_joints 必须为正，收到 {n_joints}")
        control_period = float(control_period)
        if not np.isfinite(control_period) or control_period <= 0.0:
            raise ValueError(f"control_period 必须为正，收到 {control_period}")
        self.n_joints = n
        self.control_period = control_period

        self.velocity_source = tuple(
            velocity_source if velocity_source is not None else ("motor",) * n)
        if len(self.velocity_source) != n:
            raise ValueError(
                f"velocity_source 需要 {n} 项，收到 {len(self.velocity_source)}")
        for s in self.velocity_source:
            if s not in VELOCITY_SOURCES:
                raise ValueError(f"未知的速度来源 {s!r}，可选 {list(VELOCITY_SOURCES)}")
        self.position_source = str(position_source)
        if self.position_source not in POSITION_SOURCES:
            raise ValueError(
                f"未知的位置来源 {self.position_source!r}，可选 {list(POSITION_SOURCES)}")
        self.predict_source = str(predict_source)
        if self.predict_source not in PREDICT_SOURCES:
            raise ValueError(
                f"未知的 predict_source {self.predict_source!r}，可选 {list(PREDICT_SOURCES)}")
        self.leso_input = str(leso_input)
        if self.leso_input not in LESO_INPUTS:
            raise ValueError(
                f"未知的 leso.input {self.leso_input!r}，可选 {list(LESO_INPUTS)}")

        self.leso = leso if leso is not None else JointLESO(
            np.zeros(n), n, nominal_dt=control_period)

        self._fd_mask = np.array(
            [s == "finite_difference" for s in self.velocity_source])
        # ⚠️ leso 掩码要与【观测器真的在跑】的关节一致，不能只看 source 字符串：
        # ``source: leso`` 但 bandwidth_hz=0 的关节实际是直通电机自报速度。
        # 若掩码只按字符串算，这种配置会让 enabled 变成 True，于是走一遍
        # asarray/reshape 的算术，破坏"什么都没开就逐位直通"的承诺 ——
        # 而它恰恰是最容易写出来的一种误配。（resolve_estimator 会另发一条警告。）
        self._leso_mask = (np.array([s == "leso" for s in self.velocity_source])
                           & (self.leso.omega_o > 0.0))
        self.predictor = predictor if predictor is not None else JointStatePredictor(
            SmithPredictorConfig(n))
        self.hold_on_invalid = bool(hold_on_invalid)

        self.leso_input_delay_sec = float(leso_input_delay_sec)
        if (not np.isfinite(self.leso_input_delay_sec)
                or self.leso_input_delay_sec < 0.0):
            raise ValueError(
                f"leso.input_delay_sec 必须是非负有限值，收到 {leso_input_delay_sec}")

        # 模型输入对齐用的环形缓冲。深度按秒算所需拍数 + 余量（丢拍时单拍
        # dt 会变大，缓冲深度只是内存上界，真正的截断按【秒】做）。
        depth = int(self.leso_input_delay_sec / control_period + 0.5) + 8
        self._cmd_buf: deque = deque(maxlen=max(depth, 8))

        # fd 的"上一份位置"。⚠️ 是【本估计器上一次被调用时】的位置，不是
        # /joint_states 的上一帧 —— 两者在丢拍时不同，而 dt 也是按前者量的，
        # 所以 (Δq/Δt) 自洽。
        self._prev_q = None
        self._last_q_out = None
        self._last_qd_out = None

        # 诊断计数。选累计计数而非每帧布尔：clamp / hold 是"发生了就必须知道"
        # 的稀有事件，计数列的单调性一眼可读，每帧布尔要扫全表。
        self.hold_count = 0
        self.predict_clamp_hits = 0
        self.updates = 0

    # -- 查询 ---------------------------------------------------------------

    @property
    def velocity_active(self) -> bool:
        return bool(self._fd_mask.any() or self._leso_mask.any())

    @property
    def position_from_leso(self) -> bool:
        return self.position_source == "leso" and bool(self._leso_mask.any())

    @property
    def predict_active(self) -> bool:
        return bool(self.predictor.config.active)

    @property
    def enabled(self) -> bool:
        """会不会改变输出。False 时 ``__call__`` 是逐位直通。"""
        return bool(self.velocity_active or self.position_from_leso
                    or self.predict_active)

    @property
    def leso_input_active(self) -> bool:
        return (self.leso_input == "commanded_acceleration"
                and bool(self._leso_mask.any()))

    @property
    def wants_commanded_acceleration(self) -> bool:
        """调用方是否需要每拍算一次有效指令加速度喂 ``push_command``。

        为 False 时 runner 可以整段跳过那次 M⁻¹(τ−h)（约 5 µs/拍）——
        缺省配置下就是 False。
        """
        return bool(
            self.leso_input_active
            or (self.predict_active
                and self.predict_source == "commanded_acceleration"))

    @property
    def acceleration_estimate(self) -> np.ndarray:
        """逐关节的加速度估计 [rad/s²]，只用于记录与诊断。

        LESO 关节给扩张态 z3（= 总扰动加速度；b0=0 时它吸收的是【全部】q̈，
        包含指令的效果，不只是"扰动"）；其余关节给 NaN —— 那里根本没有估计
        这个量，写 0 会被读成"扰动为零"。

        ⚠️ **绝不回灌控制律。** 回灌就是 ADRC 的扰动抑制回路，会直接改变环路
        增益，与本次要测的"换速度源"混在一起就分不清是谁的功劳。必须作为另一
        条独立路线单独上机。
        （``predict_source: acceleration_estimate`` 是个例外，但它走的是 Smith
          预测那条【有限时域 + 限幅 + 锁存退回】的通路，不是无界的扰动前馈。）
        """
        out = np.full(self.n_joints, np.nan)
        if self._leso_mask.any():
            out[self._leso_mask] = self.leso.disturbance_estimate[self._leso_mask]
        return out

    # -- 生命周期 -----------------------------------------------------------

    def reset(self) -> None:
        """清空全部内部状态（含 Smith 的锁存退回）。

        ⚠️ 不在交接处自动调用。理由：交接那一刻 q_ctrl/qd_ctrl 若跳变，正好落在
        历次上机最脆弱的时刻（当前 RMS 2.95~3.17 mm 的起点）。估计器全程启用 +
        首拍用测量值预热，比"只在跟踪段启用"更安全。
        """
        self.leso.reset()
        self.predictor.reset()
        self._cmd_buf.clear()
        self._prev_q = None
        self._last_q_out = None
        self._last_qd_out = None

    # -- 输入 ---------------------------------------------------------------

    def push_command(self, qdd_cmd, dt: float) -> None:
        """记录【已发出】的这一拍有效指令关节加速度与它作用的时长。

        ``qdd_cmd`` 必须是饱和/限幅【之后】真正生效的加速度。不是的话，饱和时
        预测器会以为机械臂在按未饱和的指令走 —— 而饱和恰恰发生在误差最大、
        最需要预测正确的时刻。

        不需要这个量时（``wants_commanded_acceleration`` 为 False）立即返回，
        调用方可以连算都不用算。
        """
        if not self.wants_commanded_acceleration:
            return
        d = float(dt)
        if not np.isfinite(d) or d <= 0.0:
            # 时钟异常：宁可少补，也不能把错误的 dt 乘进积分。
            return
        a = np.asarray(qdd_cmd, dtype=float).reshape(self.n_joints)
        if not np.all(np.isfinite(a)):
            return
        if self.leso_input_active:
            self._cmd_buf.append((a.copy(), d))
        if self.predict_active and self.predict_source == "commanded_acceleration":
            self.predictor.push_command(a, d)

    def _aligned_command(self):
        """取 ``input_delay_sec`` 之前发出的那一条指令加速度。

        物理含义：指令在 t 发出，要过一段传输/执行延迟才体现为关节加速度。
        观测器的预测步用的必须是【本步内真正作用在对象上】的那个 u。

        ⚠️ 这个延迟理论上等于位置路 T_p，而 T_p=10 ms 的 r 只有 0.43/0.45、
        明确未坐实。整条路线 C/D 的收益都挂在它上面 —— 首次上机应当把它当
        扫描变量（8/10/15 ms），不要先固定。
        """
        if not self._cmd_buf:
            return None
        lag = self.leso_input_delay_sec
        if lag <= 0.0:
            return self._cmd_buf[-1][0]
        elapsed = 0.0
        for a, d in reversed(self._cmd_buf):
            elapsed += d
            if elapsed >= lag:
                return a
        return self._cmd_buf[0][0]

    # -- 输出 ---------------------------------------------------------------

    def _sanitized_dt(self, dt):
        if dt is None:
            return self.control_period
        d = float(dt)
        if not np.isfinite(d) or d <= 0.0:
            return self.control_period
        return d

    def __call__(self, position, velocity, dt: float | None = None):
        """反馈通路出口 -> 控制器看到的那一份。返回 ``(q, qd)``。

        ⚠️ 未启用时**原样返回入参对象**（``is`` 意义上的同一个对象）。判据用
        对象恒等而不是数值相等：走一遍 ``x + 0.0`` 的算术也会在 −0.0 上留痕，
        对"逐位不变"这个承诺不干净。

        ⚠️ 必须**每个控制周期恰好调用一次**，且在 FeedbackDelayModel 之后。
        空转周期（等反馈、等 plant）不要调 —— 同一个 q 被反复喂进去，创新量
        恒为 0，等于把观测器停摆一拍，实际带宽被悄悄压低。
        """
        if not self.enabled:
            return position, velocity

        n = self.n_joints
        q = _as_vector(position, n)
        qd = _as_vector(velocity, n)

        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd))):
            # 反馈损坏。不把 NaN 灌进观测器状态，也不推进任何积分。
            self.hold_count += 1
            if self.hold_on_invalid and self._last_q_out is not None:
                return self._last_q_out.copy(), self._last_qd_out.copy()
            return position, velocity

        step = self._sanitized_dt(dt)
        # fd 的 dt 单独夹一次。LESO 自己会夹（并且靠 raw dt > resync_sec 判断
        # 真掉线），所以给它的是未夹的 step；而裸差分在一个异常长的间隔上会
        # 算出无意义的小速度，必须夹。
        step_fd = float(np.clip(step, self.leso.dt_lo, self.leso.dt_hi))

        # 1) LESO。未启用的关节由 JointLESO 内部逐位直通成入参 qd。
        u = self._aligned_command() if self.leso_input_active else None
        qd_est = self.leso(q, qd, step, u)

        # 2) 裸后向差分。⚠️ 它【减去】15 ms 滞后、不加任何滞后，付的是噪声不是
        #    相位 —— 与被否定的 50 Hz 低通交易方向【相反】。24.8 Hz 处相位从
        #    电机自报的 133.9° 降到 22.3°（半拍）。
        #    首帧没有上一份位置 -> 保留电机自报值（不是 0：从 0 起步会在第一拍
        #    造成一次假的减速冲击，与 JointVelocityFilter 的 _primed 同一个坑）。
        if self._fd_mask.any():
            qd_est = np.array(qd_est, dtype=float)      # 绝不就地改到入参数组
            if self._prev_q is not None:
                qd_est[self._fd_mask] = (
                    (q[self._fd_mask] - self._prev_q[self._fd_mask]) / step_fd)

        # 3) 位置来源。缺省 measured —— 位置路只有 10 ms 且量化仅
        #    0.000381 rad(0.022°)，用 LESO 的 z1 只会给这条快路白加滞后。
        q_est = q
        if self.position_from_leso:
            q_est = q.copy()
            q_est[self._leso_mask] = self.leso.position_estimate[self._leso_mask]

        # 4) Smith 的影子积分器输入。predict_source=acceleration_estimate 时
        #    喂【总】加速度，而 z3 只在 b0=0 时等于总加速度。
        #    b0≠0（input: commanded_acceleration）时 z3 是【模型残差】，
        #    真加速度 = z3 + b0·u —— 与 leso.py 预测步里的 acc = z2 + b0·u 同式。
        #    ⚠️ 2026-08-04 两个独立评审同时抓到这里漏加 b0·u：模型越准 z3 越接近
        #    0，预测量就越小，路线 D 会塌成"Smith 没用"的假结论。实测（恒定指令
        #    A=1.87 rad/s²、horizon 10 ms）：漏加时修正 +0.00467 rad/s，
        #    应有 A·T = 0.0187 rad/s —— 只拿到 25%，而本轨迹参考关节速度只有
        #    0.013~0.029 rad/s，丢掉的量与信号同量级。
        #    没有 LESO 的关节没有这个量 -> 喂 0，即那些关节不预测（安全侧）。
        if self.predict_active and self.predict_source == "acceleration_estimate":
            acc = np.zeros(n)
            if self._leso_mask.any():
                acc[self._leso_mask] = (
                    self.leso.disturbance_estimate[self._leso_mask])
                if self.leso_input_active and u is not None:
                    u_arr = np.asarray(u, dtype=float).reshape(n)
                    acc[self._leso_mask] += (
                        self.leso.b0[self._leso_mask] * u_arr[self._leso_mask])
            self.predictor.push_command(acc, step)

        # 5) Smith 预测。未启用/已锁存退回时原样返回入参对象。
        q_out, qd_out = self.predictor.predict(q_est, qd_est)
        if self.predictor.last_diagnostics.get("clamped", 0.0):
            self.predict_clamp_hits += 1

        self._prev_q = q.copy()
        self._last_q_out = np.asarray(q_out, dtype=float).copy()
        self._last_qd_out = np.asarray(qd_out, dtype=float).copy()
        self.updates += 1
        return q_out, qd_out

    # -- 说明 ---------------------------------------------------------------

    def describe(self) -> str:
        if not self.enabled:
            return "关节状态估计器: 全部关闭（逐位直通，= 路线 A 电机自报速度基线）"
        parts = ["速度源 %s" % (list(self.velocity_source),)]
        if self._leso_mask.any():
            w = self.leso.omega_o
            parts.append(
                "LESO ω_o=%s rad/s (%s Hz)，24.8 Hz 相位 %s°（对照：电机自报那多出来的 15 ms 是 133.9°，裸差分半拍 22.3°），input=%s"
                % (np.round(w, 1).tolist(),
                   np.round(w / (2 * np.pi), 2).tolist(),
                   np.round(self.leso.phase_lag_deg(24.8), 1).tolist(),
                   self.leso_input))
        if self.position_from_leso:
            parts.append("位置源 leso(z1)")
        if self.predict_active:
            parts.append(self.predictor.config.describe()
                         + "，驱动量 " + self.predict_source)
        return "关节状态估计器: " + "；".join(parts)


# --------------------------------------------------------------------------
# 配置解析（可脱离 ROS 单测，写法对齐 loop_delay.resolve_simulation_delay）
# --------------------------------------------------------------------------

@dataclass
class EstimatorPlan:
    """一份配置解析出来的完整估计方案。"""

    estimator: JointStateEstimator
    messages: tuple = ()
    warnings: tuple = ()

    @property
    def enabled(self) -> bool:
        return self.estimator.enabled


def resolve_estimator(estimator_cfg, n_joints, control_period,
                      mode: str = "simulation",
                      feedback_delay_enabled: bool | None = None) -> EstimatorPlan:
    """从配置的 ``estimator`` 段解析出估计器 + 提示 + 警告。

    ⚠️【与 ``resolve_simulation_delay`` 的关键区别：本模块不做 hardware 门控。】
    延迟模型只在仿真生效（实物自带那些延迟，叠加即双重计入）；估计器则**两种
    模式都要生效，而且必须是同一条代码路径**，否则实验结论无法从仿真带到实物。
    串联顺序 feedback_delay -> estimator 已经保证了同构：
      * 仿真：估计器收到 quantize 后延迟 2 拍的位置、延迟 5 拍的速度
      * 实物：resolve_simulation_delay(mode="hardware") 返回全零模型，
              ``__call__`` 原样返回入参，估计器收到裸 /joint_states
    估计器内部**不需要**任何 mode 分支，也不应该有。

    ⚠️ 缺省【全部关闭】，不是实测值。理由与 ``FeedbackDelayModel`` /
       ``freeze_trajectory_during_blend`` 相同：smc_params_whole6.yaml 与
       smc_params_hwtest_simbest.yaml 是已验证基线，代码缺省若取实测值，
       它们不改一个字行为就变了。要用就在配置里显式写。

    ``feedback_delay_enabled`` 只用于生成一条提示（仿真里没建延迟却开了预测 ->
    预测是净扰动），传 None 表示不检查。
    """
    cfg = dict(estimator_cfg or {})
    n = int(n_joints)
    control_period = float(control_period)
    messages = []
    warnings = []

    unknown = [k for k in cfg if k not in ESTIMATOR_KEYS]
    if unknown:
        # 不静默忽略：实验配置里一个拼错的键会让整次上机的记录无法解释，
        # 而实物时段是串行的稀缺资源。
        warnings.append(
            "estimator 段有无法识别的键 %s（拼错？可用键：%s）—— 它们【不生效】"
            % (sorted(unknown), list(ESTIMATOR_KEYS)))

    source = _per_joint_choice(
        cfg.get("source", "motor"), n, "estimator.source", VELOCITY_SOURCES)
    position_source = str(cfg.get("position_source", "measured")).strip().lower()
    if position_source not in POSITION_SOURCES:
        raise ValueError(
            f"estimator.position_source = {position_source!r} 不合法，"
            f"可选 {list(POSITION_SOURCES)}")

    leso_cfg = dict(cfg.get("leso") or {})
    unknown_leso = [k for k in leso_cfg if k not in _LESO_KEYS]
    if unknown_leso:
        warnings.append(
            "estimator.leso 段有无法识别的键 %s（可用键：%s）—— 它们【不生效】"
            % (sorted(unknown_leso), list(_LESO_KEYS)))
    bandwidth_hz = _per_joint_float(
        leso_cfg.get("bandwidth_hz", 0.0), n, "estimator.leso.bandwidth_hz",
        minimum=0.0)
    leso_input = str(leso_cfg.get("input", "none")).strip().lower()
    if leso_input not in LESO_INPUTS:
        raise ValueError(
            f"estimator.leso.input = {leso_input!r} 不合法，可选 {list(LESO_INPUTS)}")
    b0_cfg = _per_joint_float(
        leso_cfg.get("b0", 1.0), n, "estimator.leso.b0")
    input_delay_sec = float(leso_cfg.get("input_delay_sec", 0.0) or 0.0)

    uses_leso = any(s == "leso" for s in source)
    omega_o = np.where(
        [s == "leso" for s in source], 2.0 * np.pi * bandwidth_hz, 0.0)
    # b0 只在模型通路真的打开时才非零 —— 否则 JointLESO 会走那条带 u 的算术，
    # 与"不用模型"的逐位承诺不符。
    b0 = b0_cfg if leso_input == "commanded_acceleration" else np.zeros(n)
    leso = JointLESO(omega_o, n, nominal_dt=control_period, b0=b0)

    # ---- Smith 预测 -------------------------------------------------------
    predict_position_sec = float(cfg.get("predict_position_sec", 0.0) or 0.0)
    predict_velocity_sec = float(cfg.get("predict_velocity_sec", 0.0) or 0.0)
    predict_source = str(
        cfg.get("predict_source", "commanded_acceleration")).strip().lower()
    if predict_source not in PREDICT_SOURCES:
        raise ValueError(
            f"estimator.predict_source = {predict_source!r} 不合法，"
            f"可选 {list(PREDICT_SOURCES)}")
    # β 是【同一 horizon 上的预测强度】，horizon 是实测的物理量。
    # 两个旋钮分工明确：horizon 写实测的 T_v/T_p 不动，分档上机只动 β。
    # 缺省 1.0 —— 只写 predict_velocity_sec 的配置就是"按名义模型全额补偿"，
    # 与集成侧那份最小 schema 行为一致。
    beta_velocity = float(cfg.get("beta_velocity", 1.0))
    beta_position = float(cfg.get("beta_position", 1.0))
    if predict_velocity_sec <= 0.0:
        beta_velocity = 0.0
    if predict_position_sec <= 0.0:
        beta_position = 0.0
    predictor_cfg = SmithPredictorConfig(
        n,
        enabled=(beta_velocity > 0.0 or beta_position > 0.0),
        velocity_horizon_sec=predict_velocity_sec,
        position_horizon_sec=predict_position_sec,
        beta_velocity=beta_velocity,
        beta_position=beta_position,
        # 缺省逐关节 [0.20,0.20,0.20,0.50,0.50,0.50] rad/s 的来历：动力学一致
        # 伪逆的 J̄ 行范数腕部达 11.07 /m、大臂 3.3~4.9，跟踪段 |ν|≈1 m/s²，
        # 25 ms 内【合法】的速度修正上限 ≈ 11.07×1×0.025 = 0.28 rad/s。
        # 关节数不是 6 时退回该向量的第一项（大臂档，更紧）。
        max_velocity_correction=cfg.get(
            "predict_clamp_rad_s",
            DEFAULT_MAX_VELOCITY_CORRECTION
            if n == len(DEFAULT_MAX_VELOCITY_CORRECTION)
            else DEFAULT_MAX_VELOCITY_CORRECTION[0]),
        max_position_correction=cfg.get(
            "predict_clamp_rad", DEFAULT_MAX_POSITION_CORRECTION),
        fallback_after_clamped_ticks=int(cfg.get("predict_fallback_ticks", 10)),
    )

    estimator = JointStateEstimator(
        n, control_period,
        velocity_source=source,
        position_source=position_source,
        leso=leso,
        predictor=JointStatePredictor(predictor_cfg),
        predict_source=predict_source,
        leso_input=leso_input,
        leso_input_delay_sec=input_delay_sec,
        hold_on_invalid=bool(cfg.get("hold_on_invalid", True)),
    )

    # ---- 提示与警告 -------------------------------------------------------
    # ⚠️ 这一条必须在"整体未启用就早返回"之【前】发：``source: leso`` 配
    # ``bandwidth_hz: 0`` 恰好就落在未启用那一支，而它是最容易写出来的一种误配
    # —— 操作者以为 LESO 在跑，实际整段是电机自报速度直通。静默通过的话，
    # 那次上机的记录会被完全误读。
    if uses_leso and np.any(bandwidth_hz[[s == "leso" for s in source]] <= 0.0):
        warnings.append(
            "estimator.source 里有关节选了 leso，但它的 bandwidth_hz 是 0 "
            "-> 该关节实际【直通电机自报速度】，不是 LESO。")

    if not estimator.enabled:
        if cfg:
            messages.append(
                "关节状态估计器: 配置存在但全部为缺省 -> 逐位直通"
                "（= 路线 A 电机自报速度基线，与不写 estimator 段【逐位相同】）")
        return EstimatorPlan(estimator, tuple(messages), tuple(warnings))

    messages.append(estimator.describe())

    if uses_leso:
        active_hz = bandwidth_hz[[s == "leso" for s in source]]
        low = active_hz[(active_hz > 0.0)
                        & (active_hz < MODEL_B_MIN_BANDWIDTH_HZ)]
        if low.size:
            warnings.append(
                "estimator.leso.bandwidth_hz 有 %s Hz 低于 %.1f Hz "
                "(ω_o=%.0f rad/s)：模型 B(T_v=T_p=10.5ms, g=2.2) 下 ω_o=100 给出 "
                "σ=+1.56【发散】，ω_o=80 即便在模型 A 下也只有 ×5.82 对基线 ×5.72"
                "（等于没收益）。⚠️ 方向是反的 —— 直觉上『低带宽更平滑更安全』"
                "在这里不成立。建议 ω_o=200 rad/s = 31.8 Hz。"
                % (np.round(low, 2).tolist(), MODEL_B_MIN_BANDWIDTH_HZ,
                   MODEL_B_MIN_OMEGA_RAD_S))
        if leso_input == "none":
            warnings.append(
                "estimator.leso.input = none（无模型输入）：此时三阶 LESO 退化成"
                "纯微分器，观测器增益的全部相位都花在残差通路上。集成侧的分析"
                "认为这种配置下【没有任何 ω_o 能同时在相位和噪声上赢过位置裸差分】"
                "（要在噪声上赢 0.0289 需 ω_o ≲ 17 Hz，那里要付 98°~122° 相位，"
                "而裸差分只付 22.3°）—— 即路线 C 被路线 B 支配。"
                "这是【可证伪的预测】，值得作为消融对照跑一次，但不要指望它赢。")
        else:
            warnings.append(
                "estimator.leso.input = commanded_acceleration（b0=%s）："
                "模型输入准时速度估计对真速度全频段零相位，不准时纯亏。"
                "⚠️ 第 16 次上机实测跟踪段净力矩 |Δτ| 中位数 "
                "[0.453,0.369,0.480,0.081,0.124,0.051] Nm 对静摩擦 "
                "[0.725,1.775,0.625,0.125,0.100,0.100] Nm —— 六轴里四轴的指令"
                "力矩低于静摩擦，超过静摩擦的时间占比 j1/j2/j4/j6 全是 0%%，"
                "此时真实 q̈≈0 而 b0·u≈1.87 rad/s²(j2)。这条通路在本机是否成立"
                "是【未知数】，必须与 input:none 做消融对照。"
                % np.round(b0_cfg, 3).tolist())
        if input_delay_sec <= 0.0:
            messages.append(
                "estimator.leso.input_delay_sec = 0：模型输入未做延迟对齐。"
                "它的物理含义是『指令发出到体现为关节加速度』，理论上 = 位置路 T_p，"
                "而 T_p=10 ms 的 r 只有 0.43/0.45【未坐实】—— 首次上机建议当扫描"
                "变量（8/10/15 ms），不要先固定。")

    fd_idx = [i + 1 for i, s in enumerate(source) if s == "finite_difference"]
    if fd_idx:
        messages.append(
            "位置裸后向差分关节 j%s：省掉电机速度估计器那 15 ms（24.8 Hz 处 "
            "133.9° -> 22.3°），代价是量化噪声。实测参考速度 0.0133 rad/s 时"
            "裸差分 std 0.0289 rad/s；⚠️ loop_delay 里那个 0.0762 是"
            "『每拍跨一个量化步』的上界，本轨迹太慢达不到。"
            % (fd_idx,))
        if any(i <= 3 for i in fd_idx):
            warnings.append(
                "j1~j3 选了 finite_difference。表面上大臂自报速度噪声实测 0.0000、"
                "换成裸差分是『无中生有』地注入 0.0287~0.0370 rad/s；"
                "⚠️ 但 2026-08-04 从 3 份实机 CSV 反解出自报速度是【12 bit 量化】的"
                "（大臂 LSB 0.009768 = 40/4095，腕部 0.024418 = 100/4095），"
                "『大臂 σ=0.0000』是静止时停在同一个码上的假象，同期还带一个整码"
                "偏置（j1/j2/j3 位置 22~38 s 一个 LSB 没动，自报速度恒为 "
                "−0.00488 rad/s）。所以大臂不是干净，是分辨率不够且有偏。"
                "两种读法都要在实物上验，别当成已经解决。")

    if estimator.predict_active:
        warnings.append(
            "⚠️ Smith 预测已开启，而 position/orientation 增益是冻结的。"
            "稳态解析解：无预测 |e| = d/c0 + ẋ_ref·T_p；Smith |e| = (d/c0)·"
            "[1 + β_v·c1·T_v + ...]。代入 d_eff=0.411 m/s²、模型A、c1=30："
            "无预测 2.103 mm vs Smith 3.453 mm —— **差 64%，更糟**。"
            "惩罚项来源明确：预测器用指令加速度积分，指令里没有扰动 d，于是把"
            "速度低估了 d·T_v，控制器多给 c1·d·T_v 的加速度，恰好推着扰动走。"
            "追平现状需 c1>=42.2（只补速度路 40.5）。"
            "=> **绝不能先只开预测、不动增益地跑一次『对照组』** —— 那次一定更差，"
            "而且会被读成『Smith 没用』。预测与提增益必须同一次改动一起上。")
        if feedback_delay_enabled is False and mode != "hardware":
            messages.append(
                "⚠️ 仿真模式且 simulation 段没建延迟（plant 是 "
                "step_on_effort_command 锁步、零延迟），此时 Smith 预测补的是一段"
                "【不存在】的延迟 —— 在这上面它是净扰动，不是补偿。冒烟测试可以，"
                "任何性能判断都无效。")

    return EstimatorPlan(estimator, tuple(messages), tuple(warnings))


def estimator_from_config(estimator_cfg, n_joints, control_period,
                          mode: str = "simulation") -> JointStateEstimator:
    """``resolve_estimator`` 的便捷包装，只要估计器本身。"""
    return resolve_estimator(estimator_cfg, n_joints, control_period, mode).estimator


# 给测试与外部脚本用的简短别名（与集成侧设计里的用法一致）。
JointStateEstimator.from_config = staticmethod(estimator_from_config)
