#!/usr/bin/env python3
"""阶段 1：6 关节构型的正运动学 / 关节零位校验（位置模式，不发力矩）。

原理
----
没有外部测量设备时，重力力矩本身就是完整正运动学的函数：连杆长度、关节
轴向、关节零位全部编码在 g(q) 里。把其余关节锁死、只扫一个关节 j 时，
该关节的重力力矩是一条纯正弦：

    tau_j(q_j) = A * sin(q_j - phi) + b

  A    下游一阶质量矩 x g  -> 偏差来自质量误差，阶段 2 的标定能修
  phi  关节零位偏差        -> 偏差来自几何/装配，必须先修
  b    常数偏置            -> 力矩零漂

A 与 phi 是正弦的幅值和相位，彼此独立可辨识，所以这一个实验就能把
"质量错"和"零位错"分开 —— 这正是换用 link6 末端之后必须先确认的事。

安全
----
全程位置模式，只发布 /student/joint_command。本脚本不会 import 也不会
发布任何力矩话题。每个目标点在下发前都做三重检查：关节限位余量、模型
预测力矩占额定的比例、以及与当前位姿的单步跨度。

用法
----
    # 先看计划，不动机械臂
    ros2 run manipulator fk_gravity_verify.py --dry-run

    # 只做方向校验（最小幅度，最先跑这个）
    ros2 run manipulator fk_gravity_verify.py --direction-only

    # 完整扫描
    ros2 run manipulator fk_gravity_verify.py --joints 1,2,3,4,5
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from dummy_interface.msg import MotorState
except ImportError as exc:  # pragma: no cover - offline planning still works
    rclpy = None
    _IMPORT_ERROR = exc

import pinocchio as pin


N_ARM = 6
JOINT_NAMES = [f"joint{i}" for i in range(1, N_ARM + 1)]

JOINT_LOWER = np.array([-4.0, -3.1415, -1.5708, -4.0, -2.0, -4.0])
JOINT_UPPER = np.array([4.0, 2.0, 1.5708, 4.0, 2.0, 4.0])
RATED_TORQUE = np.array([9.0, 9.0, 9.0, 1.6, 1.6, 1.6])

# 实测力矩噪声 std（/dev/ttyUSB0 静止保持抓包）
TAU_NOISE = np.array([0.0251, 0.0645, 0.0627, 0.0071, 0.0060, 0.0060])

LIMIT_MARGIN = 0.15       # rad，任何目标点离限位至少这么远
# 预测力矩不得超过额定的这个比例。
#
# 0.50 -> 0.65：URDF 按实机标定修正(连杆重了 23%)后，joint1 原扫描区间的
# 峰值重力从额定的 44% 变为 59.5%。这【不是新增风险】—— 实物一直就是这个
# 质量，该扫描也已在实机安全跑完；旧的 44% 是拿偏轻 30% 的模型算出的虚高余量。
#
# 取 0.65 的依据：9 Nm 是【连续热额定】，峰值是 27 Nm，0.65x 额定仅为峰值的
# 21.7%。且扫描是 13 点 x 2s dwell 共约 26 s 的间歇保持，远短于热时间常数。
# 若改为收窄 joint1 区间来满足 0.50，会把跨度从 2.28 压回 1.83 rad，
# 而该区间当初正是为改善相位拟合条件数才加宽的（见 SWEEP_PLAN 注释）。
TORQUE_BUDGET = 0.65

# 桌面几何。基座装在桌沿，安装平面与桌面同高，所以 base_link 的 z=0 就是桌面。
# joint1 转轴指向 +x，那一侧是桌沿之外的空区；-x 一侧才有桌面。
#
# 这条约束原本没有，代价是实机把整条臂压到了桌面上：joint4 的扫描位姿
# 需要 joint1 = -1.5 rad，把机械臂绕基座轴甩下 90 度，末端落到 z = -82 mm。
# 当时的安全检查只看关节限位和力矩预算，几何上完全没有防线。
DESK_PLANE_Z = 0.0        # m，桌面高度（= 基座安装平面）
DESK_EDGE_X = 0.02        # m，桌沿位置；x 小于此值的一侧有桌面
DESK_CLEARANCE = 0.08

# 桌面上的围栏：竖直挡板，沿 -x 方向延伸，位于 +y 侧。
#   FENCE_Y      实测约「基座->joint4 距离(394mm) 再多几厘米」= 420~440mm。
#                取 400mm 偏保守（比实际更靠近基座，会提前拦下）。
#   FENCE_HEIGHT 实测约「基座->joint3 距离」= 331mm，自桌面起算。
# 基座已向外(+x)移动约 20mm，DESK_EDGE_X 保持 0.02 不变 —— 那使模型认为桌面
# 比实际更靠近机械臂，属偏保守，安全。
FENCE_Y = 0.40
FENCE_HEIGHT = 0.331
FENCE_CLEARANCE = 0.05     # m，离桌面至少留这么高
# 30mm 不够：desk_violations 只采样关节原点和末端，电机壳与线缆的实际外形
# 比这些点低得多。实测撞桌那次，点采样只报侵入 11mm，实物却整条压在桌面上。
MAX_STEP = 0.35           # rad，单步最大跨度，限制两次安全检查之间扫过的体积
MAX_MOVE_STEPS = 200      # move_through 的死循环兜底

# 高位安全中转姿态。两个扫描位姿之间直接插值会经过贴近桌面的构型：实测
# joint3 扫描结束点直奔 joint5 起点时，中途 joint3 原点降到 z=75mm 被拒
# （要求 80mm）。经这个中转点走，全部转移的最小余量是 102~265mm。
#
# 注意离线仿真无法精确预测实际路径：move_through 每步从【当前已下垂的】
# 位置重新瞄准目标，实现出来的轨迹会偏离直线，而下垂量无法完全预测。
# 所以中转点是【增大余量】的手段，运行时的 check_setpoint 才是权威。
SAFE_WAYPOINT = np.array([-0.2968, 1.0676, 0.7130, -0.5496, -0.3997, 0.7471])

# 每个关节的测试姿态、扫描区间与采样配置。姿态由随机搜索给出：固定其余关节，
# 使被测关节的重力力矩摆幅最大。
#
#   amp    拟合正弦的真实幅值 A（tau = A*sin(q-phi)+b）
#          ⚠️ 该值由【已按实机标定修正的】arm6.urdf 算出，比 CAD 名义值
#          大约 1.28~1.31 倍。sigma 是在旧(较小)幅值下做蒙特卡洛得到的，
#          幅值变大只会提高信噪比，所以 sigma 是保守值，未重算。
#   swing  扫描区间内实际观测到的摆幅一半；区间不足一个周期时 swing < amp
#   points 扫描点数
#   dwell  每点静止采样秒数
#   sigma  该配置下零位估计的 1-sigma（度），由 400 次蒙特卡洛实测
#
# points/dwell 是逐关节调的，不是统一值：joint3 的力矩噪声 std 是 0.0627 Nm
# （joint4/5 的十倍）而幅值只有 0.89 Nm，用 joint1 那套配置它的 1-sigma 会到
# 0.36 度，比判据阈值还大，必然误判。
SWEEP_PLAN = {
    # 区间从 [-1.024, 0.976] 改为 [-0.88, 1.40]。原区间的【起点】会让 joint6
    # 伸到 y=468.7mm，撞上桌面围栏（限值 350mm）—— 实机上确实撞到了，
    # 而围栏当时完全不在碰撞模型里。新区间在安全区 [-0.900, 1.425] 内留
    # 20 mrad 余量，跨度反而更宽（2.28 vs 2.00 rad）。
    1: dict(posture=[-0.0239, 0.0482, 1.4088, 1.5000, 0.8844, -0.8976],
            lo=-0.8800, hi=1.4000, amp=5.4649, swing=4.8322,
            points=13, dwell=2.0, sigma=0.0233),
    2: dict(posture=[-0.0384, -0.0359, 1.1034, -1.5000, -1.5000, -1.5000],
            lo=-1.0359, hi=0.9641, amp=4.3304, swing=3.6436,
            points=13, dwell=5.0, sigma=0.0620),
    # 区间从 2.00 加宽到 2.42 rad（覆盖 32%->39% 个周期）。相位拟合的精度由
    # 拟合条件数决定，区间越窄相位与幅值越强耦合、拟合越病态。
    3: dict(posture=[0.0998, 1.2224, -0.3036, 1.0787, 0.2736, 0.2837],
            lo=-1.4200, hi=1.0000, amp=1.1306, swing=1.0556,
            points=21, dwell=5.0, sigma=0.1427),
    # joint4 原来的位姿是 [-1.5, 0.8315, ...]，joint1 = -86 度把整条臂绕基座轴
    # 甩到水平，压在桌面上（实机发生过）。改用下面这个 joint1 近零的位姿，代价是
    # 重力摆幅从 0.1122 降到 0.0513 Nm，零位分辨率相应从 0.10 度退化到 0.37 度。
    # 区间从 0.80 加宽到 7.00 rad（覆盖 13%->111% 个周期）。
    # 原来的 0.80 rad 是撞桌事件后为避开桌面设的，但【转 joint4 根本不影响
    # 桌面】—— 实测桌面余量恒为 0.1732 m，与 joint4 角度完全无关。
    # 窄区间使拟合病态：实测两遍 phi 相差 82.3 度，末端贡献 56.96 mm；
    # 加宽后降到 1.66 mm。
    4: dict(posture=[0.0161, 0.7043, -0.7045, 1.4000, 1.4000, 1.4000],
            lo=-3.5000, hi=3.5000, amp=0.1716, swing=0.1706,
            points=13, dwell=7.5, sigma=0.0348),
    # 原位姿 [-0.0766, 1.5, -0.2867, -1.5, -0.367, 0.4498] 的【21 个采样点
    # 全部违反桌面约束】—— 它是撞桌事件后唯一没有重新校验过的位姿。
    # 换成 21/21 点全安全的位姿：桌面余量恒 +0.234 m，限位最小余量 0.342 rad。
    # 信号强度不变（amp 是连杆质量决定的固有值，与位姿无关；swing 也相同）。
    # 区间从 2.00 加宽到 3.70 rad（覆盖 32%->59%）。同样地，转 joint5 不影响
    # 桌面（余量恒 0.2344 m）。
    # 区间从 ±1.85 收窄到 ±1.70。±1.85 时端点的【过冲点】会超出限位
    # (1.85+0.15=2.00)，approach() 三档过冲全部不安全，退化为直接进入 ——
    # 该点摩擦符号不受控，单向进入协议失效（实测 4 次违规），而扫描端点
    # 恰恰是相位拟合最敏感的位置。±1.70 时 1.70+0.15=1.85 仍安全。
    # 幅值不变（A 是连杆质量的固有属性，与区间无关）。
    5: dict(posture=[-0.0069, 0.3348, 1.2292, -1.3767, 0.0000, -0.3033],
            lo=-1.7000, hi=1.7000, amp=0.1723, swing=0.1723,
            points=21, dwell=5.0, sigma=0.0493),
    # joint6 无重力信号：link6 质心落在 joint6 转轴上（y/z 偏移 0.05/0.10 mm），
    # 转动它既不移动质心也不改变末端位置。零位只能靠方向校验或装上末端工具后再测。
}

# 判定关节零位偏差是否显著的阈值：工程阈值 0.5 度与该关节 3-sigma 取大者。
# 只用固定阈值会让 joint3 这类噪声大的关节频繁误报。
#
# 注意：零位偏差本身不是故障。URDF 的建模零位与电机编码器零位本就不是同一个
# 零点，所以 phi != 0 是预期结果，量出来加进模型即可。真正的几何错（连杆长度
# 或轴向错误）会破坏正弦的形状，表现为拟合残差变大 —— 那才是必须停下来的信号。
ZERO_OFFSET_ENGINEERING_THRESHOLD_DEG = 0.5

# 归一化拟合残差超过这个比例就认为正弦形状被破坏 = 几何错。
# 无噪声时模型残差精确为 0；实测噪声下 resid/A 约等于 sigma_tau/A。
SHAPE_RESIDUAL_LIMIT = 0.05

# 判稳窗口内允许的位置变化幅度 (rad)。位置编码器 LSB 是 3.834e-4，
# 实测静止 15 s 时 joint1-3 的 span 精确为 0、joint4-6 为 2 个 LSB。
POSITION_SETTLE_SPAN = 1.5e-3

# 跟踪误差超过这个值就提示一次。位置环 kp 有限，承受重力的关节必然有
# 稳态误差（joint2: 2.2 Nm / kp 30 = 0.073 rad），这不是故障。
TRACKING_ERROR_WARN = 0.01

# 单向进入的过冲量 (rad)。每个测量点先过冲再退回，保证最后一次运动方向恒定，
# 把库仑摩擦变成被拟合常数项吸收的同号偏置。过冲必须明显大于摩擦造成的
# 位置滞回（实测 joint2 端点 0.866 Nm / kp 30 = 0.029 rad）。
APPROACH_OVERSHOOT = 0.15

# 方向校验的幅值容差（相对命令值）。下垂量随重力力矩变化，实测约 10%。
DIRECTION_SCALE_MIN = 0.7
DIRECTION_SCALE_MAX = 1.3


def offset_threshold_deg(joint: int) -> float:
    sigma = SWEEP_PLAN[joint]["sigma"]
    return max(ZERO_OFFSET_ENGINEERING_THRESHOLD_DEG, 3.0 * sigma)


def urdf_path() -> str:
    here = Path(__file__).resolve()
    local = here.parents[2] / "config" / "arm6.urdf"
    if local.exists():
        return str(local)
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("manipulator"), "arm6.urdf")


class GravityModel:
    """6 关节固定基座重力模型。基座假定水平。"""

    def __init__(self, path: str | None = None):
        self.path = path or urdf_path()
        self.model = pin.buildModelFromUrdf(self.path)
        if self.model.nq != N_ARM:
            raise ValueError(
                f"{self.path} has nq={self.model.nq}, expected {N_ARM}. "
                "This tool is for the 6-joint hardware configuration."
            )
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId("link6")

    def gravity(self, q: np.ndarray) -> np.ndarray:
        return pin.computeGeneralizedGravity(
            self.model, self.data, np.asarray(q, dtype=float)).copy()

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id].translation.copy()

    def link_points(self, q: np.ndarray) -> np.ndarray:
        """各关节原点 + 末端在 base_link 系的位置，用于碰撞检查。

        这是对真实外形的粗略采样：只查关节原点和末端，连杆中段的凸出
        部分（电机壳、线缆）没有建模，所以 DESK_CLEARANCE 要留够。
        """
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pts = [self.data.oMi[self.model.getJointId(f"joint{j}")].translation.copy()
               for j in range(1, N_ARM + 1)]
        pts.append(self.data.oMf[self.ee_frame_id].translation.copy())
        return np.array(pts)

    def desk_violations(self, q: np.ndarray) -> list[tuple[str, np.ndarray]]:
        """返回会撞到桌面【或围栏】的点。

        桌面：水平面，只存在于 x < DESK_EDGE_X 一侧。
        围栏：竖直挡板，沿 -x 方向延伸，位于 +y 侧，高度自桌面起算。
              实机上 joint6 曾打到它，而它此前完全不在模型里 —— 之前几次
              「卡住」中可能有一次实际是撞围栏而非撞桌面。
        """
        names = [f"joint{j}" for j in range(1, N_ARM + 1)] + ["ee"]
        bad = []
        for name, p in zip(names, self.link_points(q)):
            over_desk = p[0] < DESK_EDGE_X
            if over_desk and p[2] < DESK_PLANE_Z + DESK_CLEARANCE:
                bad.append((name, p))
            elif (over_desk and p[1] > FENCE_Y - FENCE_CLEARANCE
                    and p[2] < DESK_PLANE_Z + FENCE_HEIGHT):
                bad.append((name, p))
        return bad


# --------------------------------------------------------------------------
# 安全检查
# --------------------------------------------------------------------------

@dataclass
class SafetyReport:
    ok: bool
    reasons: list[str] = field(default_factory=list)


def check_setpoint(q: np.ndarray, model: GravityModel,
                   q_from: np.ndarray | None = None) -> SafetyReport:
    rep = SafetyReport(ok=True)
    q = np.asarray(q, dtype=float)

    if q.shape != (N_ARM,):
        rep.ok = False
        rep.reasons.append(f"expected {N_ARM} joints, got {q.shape}")
        return rep
    if not np.all(np.isfinite(q)):
        rep.ok = False
        rep.reasons.append("contains NaN/Inf")
        return rep

    lo_slack = q - (JOINT_LOWER + LIMIT_MARGIN)
    hi_slack = (JOINT_UPPER - LIMIT_MARGIN) - q
    for j in range(N_ARM):
        if lo_slack[j] < 0 or hi_slack[j] < 0:
            rep.ok = False
            rep.reasons.append(
                f"joint{j+1}={q[j]:.4f} is within {LIMIT_MARGIN} rad of its limit "
                f"[{JOINT_LOWER[j]}, {JOINT_UPPER[j]}]")

    tau = model.gravity(q)
    frac = np.abs(tau) / RATED_TORQUE
    if np.max(frac) > TORQUE_BUDGET:
        j = int(np.argmax(frac))
        rep.ok = False
        rep.reasons.append(
            f"predicted gravity torque on joint{j+1} is {tau[j]:.3f} Nm = "
            f"{frac[j]*100:.0f}% of rated, over the {TORQUE_BUDGET*100:.0f}% budget")

    for name, p in model.desk_violations(q):
        rep.ok = False
        rep.reasons.append(
            f"{name} would be at x={p[0]*1000:.0f} y={p[1]*1000:.0f} "
            f"z={p[2]*1000:.0f} mm, i.e. over the desk (x<{DESK_EDGE_X*1000:.0f}mm) "
            f"and below the {DESK_CLEARANCE*1000:.0f}mm clearance")

    if q_from is not None:
        step = np.max(np.abs(q - np.asarray(q_from, dtype=float)))
        if step > MAX_STEP:
            rep.ok = False
            rep.reasons.append(
                f"single step of {step:.3f} rad exceeds MAX_STEP={MAX_STEP}")
    return rep


class FlushingWriter:
    """每写一行就落盘的 csv writer。

    默认的 csv.writer 写进 Python 的缓冲区，直到文件关闭才落盘。一次
    扫描要跑半小时以上，中途崩溃或被中断就会丢掉全部数据 —— 实测发生过：
    被中止的那次运行只在磁盘上留下 77 字节的表头。
    """

    def __init__(self, handle):
        self._handle = handle
        self._writer = csv.writer(handle)

    def writerow(self, row):
        self._writer.writerow(row)
        self._handle.flush()
        os.fsync(self._handle.fileno())


def plan_next_step(q_now: np.ndarray, q_target: np.ndarray) -> np.ndarray:
    """从【当前实际位置】朝目标推进一步，跨度不超过 MAX_STEP。

    必须每步现算，不能预先切好整条路径。位置环有 kp 限制的跟踪滞后，
    预算好的第 k 个点与滞后后的实际位置之间可能超过 MAX_STEP，安全检查
    会把一次完全正常的移动判为违规（实测：滞后 0.1859 rad 时报出
    "single step of 0.353 rad exceeds MAX_STEP=0.35" 并中止扫描）。
    """
    q_now = np.asarray(q_now, dtype=float)
    q_target = np.asarray(q_target, dtype=float)
    span = float(np.max(np.abs(q_target - q_now)))
    if span <= MAX_STEP:
        return q_target.copy()
    return q_now + (q_target - q_now) * (MAX_STEP / span)


def interpolate(q_from: np.ndarray, q_to: np.ndarray) -> list[np.ndarray]:
    """把一次大跨度拆成不超过 MAX_STEP 的若干小步（离线规划用）。

    在线运动请用 plan_next_step：本函数假定机械臂精确跟随，不成立于
    有稳态跟踪误差的位置环。
    """
    q_from = np.asarray(q_from, dtype=float)
    q_to = np.asarray(q_to, dtype=float)
    span = np.max(np.abs(q_to - q_from))
    n = max(1, int(np.ceil(span / MAX_STEP)))
    return [q_from + (q_to - q_from) * (k + 1) / n for k in range(n)]


# --------------------------------------------------------------------------
# 正弦拟合
# --------------------------------------------------------------------------

def fit_sine(q: np.ndarray, tau: np.ndarray) -> dict:
    """拟合 tau = A*sin(q - phi) + b。

    展开成 tau = a1*sin(q) + a2*cos(q) + b 后是线性最小二乘，无需初值。
    """
    q = np.asarray(q, dtype=float)
    tau = np.asarray(tau, dtype=float)
    if q.size < 3:
        return dict(ok=False, reason="need at least 3 points")

    M = np.column_stack([np.sin(q), np.cos(q), np.ones_like(q)])
    coef, *_ = np.linalg.lstsq(M, tau, rcond=None)
    a1, a2, b = coef
    amplitude = float(np.hypot(a1, a2))
    phi = float(np.arctan2(-a2, a1))
    resid = tau - M @ coef
    return dict(
        ok=True,
        amplitude=amplitude,
        phi=phi,
        offset=float(b),
        rms_residual=float(np.sqrt(np.mean(resid ** 2))),
        max_residual=float(np.max(np.abs(resid))),
        n_points=int(q.size),
    )


def model_reference(model: GravityModel, joint: int,
                    posture: list[float], qs: np.ndarray) -> dict:
    """标称位姿下的参考相位。只在离线规划时用。

    在线分析请用 model_reference_measured：位置环有 kp 限制的下垂，机械臂
    实际停的位置不是标称位姿，而参考相位对其余关节的位置极其敏感。
    """
    j = joint - 1
    taus = []
    for v in qs:
        q = np.array(posture, dtype=float)
        q[j] = v
        taus.append(model.gravity(q)[j])
    return fit_sine(qs, np.array(taus))


def model_reference_measured(model: GravityModel, joint: int,
                             q_full: np.ndarray) -> dict:
    """用每个采样点【实测的完整关节角】算参考相位。

    必须这样做，不能用标称位姿。位置环的稳态下垂让机械臂停在标称之外，
    而参考相位对其余关节的位置非常敏感：实测 joint3 的扫描位姿里 joint2
    下垂 0.073 rad，就能让表观零位偏差漂 4.1 度 —— 是力矩噪声给出的
    1-sigma (0.11 度) 的 37 倍。

    这个错误被一次重复测量抓到：joint3 两次独立扫描的零位偏差差了
    2.243 度，而按力矩噪声算应该只差 0.11 度。A、ratio、残差、静摩擦
    四项都重复得很好，唯独相位对不上，指向的正是位姿复现问题。
    """
    q_full = np.asarray(q_full, dtype=float)
    if q_full.ndim != 2 or q_full.shape[1] != N_ARM:
        raise ValueError(f"expected an (n, {N_ARM}) array of measured poses")
    j = joint - 1
    taus = [model.gravity(q)[j] for q in q_full]
    return fit_sine(q_full[:, j], np.array(taus))


# --------------------------------------------------------------------------
# ROS 执行
# --------------------------------------------------------------------------

class FkGravityVerifier(Node):
    def __init__(self, args):
        super().__init__('fk_gravity_verify')
        self.args = args
        self.model = GravityModel()
        self.q = None
        self.qd = None
        self.tau = None
        self.joint_order = None
        # 安全检查拒绝时置位。这类事件必须中止整轮运行：继续做下一个关节
        # 等于在一个已知违规的状态上接着动，而且被拒关节的数据会被静默丢弃。
        self.safety_abort = False
        # 过冲点不安全、退化为直接进入的次数。非零说明该次扫描里有点的
        # 摩擦符号不受控，结果可疑。
        self.unidirectional_violations = 0

        self.pub = self.create_publisher(JointState, '/student/joint_command', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(MotorState, '/student/joint_feedback',
                                 self._on_feedback, 10)

    def _on_joint_state(self, msg: JointState):
        if len(msg.position) != N_ARM:
            self.get_logger().error(
                f"/joint_states has {len(msg.position)} joints, expected {N_ARM}. "
                "Are you running student_arm6.launch.py?", throttle_duration_sec=5.0)
            return
        self.joint_order = list(msg.name)
        self.q = np.array(msg.position, dtype=float)
        self.qd = np.array(msg.velocity, dtype=float) if len(msg.velocity) == N_ARM \
            else np.zeros(N_ARM)
        if len(msg.effort) == N_ARM:
            self.tau = np.array(msg.effort, dtype=float)

    def _on_feedback(self, msg: MotorState):
        if len(msg.current) == N_ARM:
            self.tau = np.array(msg.current, dtype=float)

    # -- helpers ---------------------------------------------------------

    def wait_for_state(self, timeout=10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.q is not None and self.tau is not None:
                return True
        return False

    def publish(self, q_target: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in q_target]
        msg.velocity = [0.0] * N_ARM
        self.pub.publish(msg)

    def goto(self, q_target: np.ndarray, timeout=15.0) -> bool:
        """走到目标点并等待运动停止。

        判稳看的是"位置不再变化"，不是"到达目标"，也不看速度反馈：

        * 速度反馈的量化 LSB 是 0.0244 rad/s。实测静止时 joint4 仍报
          0.0855 rad/s，纯属量化噪声，任何合理的速度阈值都判不了稳。
        * 位置存在 kp 限制的稳态误差。joint2 承受 2.2 Nm 重力而 kp=30，
          稳态误差 2.2/30 = 0.073 rad，实测 0.0801 rad 吻合。要求
          |q - q_target| 很小是不可能满足的，会把正常状态误判为失败。

        因此收敛判据是位置在一个时间窗内不再变化；到不了目标只记录跟踪
        误差，不算失败。
        """
        q_target = np.asarray(q_target, dtype=float)
        t0 = time.time()
        window = []
        while time.time() - t0 < timeout:
            self.publish(q_target)
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.q is None:
                continue
            window.append((time.time(), self.q.copy()))
            window = [(t, q) for t, q in window
                      if time.time() - t <= self.args.settle_sec]
            if len(window) >= 20 and time.time() - t0 > self.args.settle_sec:
                span = np.max(np.ptp(np.array([q for _, q in window]), axis=0))
                if span <= POSITION_SETTLE_SPAN:
                    err = float(np.max(np.abs(self.q - q_target)))
                    if err > TRACKING_ERROR_WARN:
                        self.get_logger().warn(
                            f"settled with tracking error {err:.4f} rad "
                            f"(kp-limited sag, expected on gravity-loaded joints)")
                    return True
        self.get_logger().warn(
            f"goto: motion did not settle within {timeout:g}s, "
            f"residual {np.max(np.abs(self.q - q_target)):.4f} rad")
        return False

    def approach(self, q_target: np.ndarray, joint_idx: int) -> bool:
        """带过冲的单向进入：保证测量前最后一次运动方向恒定。

        双向平均本应消掉库仑摩擦（正反方向摩擦符号相反），但它要求正反两次
        的摩擦【对称】。扫描的第一个点破坏了这个前提：机械臂是从转移过来的，
        最后一次运动方向由转移决定，不是扫描方向。实测该点的表观摩擦达
        0.866 Nm（中段仅 0.02），仿真显示这种不对称会造成约 -6.8 度的系统性
        相位偏差 —— 量级正好覆盖 joint2 两次测量 9.4 度的差异。

        每个点都先过冲再退回，摩擦就变成同号常数，被拟合的 b 项吸收，
        幅值 A 与相位 phi 都不受影响。代价是每点多一次移动。
        """
        q_target = np.asarray(q_target, dtype=float)
        for overshoot in (APPROACH_OVERSHOOT, APPROACH_OVERSHOOT * 0.5,
                          APPROACH_OVERSHOOT * 0.25):
            q_over = q_target.copy()
            q_over[joint_idx] += overshoot
            if check_setpoint(q_over, self.model).ok:
                if not self.goto(q_over):
                    return False
                return self.goto(q_target)
        # 过冲点全都不安全时退回直接进入。该点的摩擦符号无法保证，
        # 计数记录下来，分析时视为可疑。
        self.get_logger().warn(
            f"no safe overshoot for joint{joint_idx + 1} at "
            f"{q_target[joint_idx]:+.4f}; approaching directly")
        self.unidirectional_violations += 1
        return self.goto(q_target)

    def dwell_record(self, q_target: np.ndarray, seconds: float) -> dict | None:
        """静止采样，返回均值与标准差。"""
        qs, taus = [], []
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.publish(q_target)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.q is not None and self.tau is not None:
                qs.append(self.q.copy())
                taus.append(self.tau.copy())
        if len(qs) < 10:
            return None
        qs = np.array(qs)
        taus = np.array(taus)
        return dict(
            q_mean=qs.mean(axis=0), q_std=qs.std(axis=0),
            tau_mean=taus.mean(axis=0), tau_std=taus.std(axis=0),
            n=len(qs),
        )

    def move_through(self, q_target: np.ndarray,
                     allow_waypoint: bool = True) -> bool:
        """分步走到目标，每一步都从【当前实际位置】重新计算。

        不能预先把整条路径切好再逐个下发：位置环有 kp 限制的跟踪滞后
        （实测最大 0.1859 rad），预算好的第 k 个点与滞后后的实际位置之间
        可能超过 MAX_STEP，于是安全检查把一次完全正常的移动判为违规。
        每步现算则按构造恒等于 MAX_STEP，永远不会误触发。
        """
        q_target = np.asarray(q_target, dtype=float)
        rep = check_setpoint(q_target, self.model)
        if not rep.ok:
            self.get_logger().error("REFUSED target: " + "; ".join(rep.reasons))
            self.safety_abort = True
            return False

        if self._direct_move(q_target):
            return True
        if not allow_waypoint or self.safety_abort:
            return False

        # 直达被拒时先抬到高位中转姿态再走。两个扫描位姿之间的直线会经过
        # 贴近桌面的构型（实测 joint3 -> joint5 的转移在 z=75mm 被拒），
        # 而经中转点的每一段余量都在 100mm 以上。
        self.get_logger().warn(
            "direct transit refused; retrying via the safe waypoint")
        self.safety_abort = False
        # 过冲点不安全、退化为直接进入的次数。非零说明该次扫描里有点的
        # 摩擦符号不受控，结果可疑。
        self.unidirectional_violations = 0
        if not self._direct_move(SAFE_WAYPOINT):
            self.get_logger().error("could not reach the safe waypoint either")
            self.safety_abort = True
            return False
        if not self._direct_move(q_target):
            self.get_logger().error("still refused after the waypoint")
            self.safety_abort = True
            return False
        return True

    def _direct_move(self, q_target: np.ndarray) -> bool:
        q_target = np.asarray(q_target, dtype=float)
        for _ in range(MAX_MOVE_STEPS):
            if float(np.max(np.abs(q_target - self.q))) <= MAX_STEP:
                return self.goto(q_target)
            step = plan_next_step(self.q, q_target)
            rep = check_setpoint(step, self.model)
            if not rep.ok:
                self.get_logger().warn("refused step: " + "; ".join(rep.reasons))
                return False
            if not self.goto(step):
                return False
        self.get_logger().error(
            f"move_through made no progress in {MAX_MOVE_STEPS} steps; "
            f"still {np.max(np.abs(q_target - self.q)):.4f} rad away")
        return False


# --------------------------------------------------------------------------
# 步骤
# --------------------------------------------------------------------------

def run_direction_check(node: FkGravityVerifier, joints: list[int],
                        delta: float, q_home: np.ndarray) -> list[dict]:
    """最小幅度方向 / 标度校验：验证命令 -> 旋向翻转 -> 固件 -> 编码器 整条符号链。

    q_home 由调用方采集一次并全程复用。绝不能在这里重新读取当前位置：
    承受重力的关节有 kp 限制的稳态下垂，把下垂后的位置当成新的 home
    会逐轮累积（实测 joint2 因此从 1.3235 一路爬到 1.5661）。
    """
    results = []
    q_home = np.asarray(q_home, dtype=float).copy()
    for j in joints:
        idx = j - 1
        node.get_logger().info(f"--- direction check joint{j} (delta = +{delta} rad) ---")
        q_plus = q_home.copy()
        q_plus[idx] += delta
        rep = check_setpoint(q_plus, node.model, q_from=q_home)
        if not rep.ok:
            q_plus[idx] = q_home[idx] - delta
            rep = check_setpoint(q_plus, node.model, q_from=q_home)
            if not rep.ok:
                node.get_logger().error(f"joint{j} SKIPPED: " + "; ".join(rep.reasons))
                results.append(dict(joint=j, ok=False, reason="; ".join(rep.reasons)))
                continue
        commanded = q_plus[idx] - q_home[idx]

        # 两个端点必须在同样的受控条件下测量。位置环有 kp 限制的稳态下垂，
        # 若基准取自"机械臂未受本节点命令时"的位置，测得的位移会等于
        # delta + 下垂 —— 实测 joint2 因此得到 0.1896 而非 0.1000
        # (0.1 + 0.089 下垂)，joint3 得到 0.0668 (0.1 - 0.039)。
        # 先命令 q_home 停稳测基准，下垂在相减时抵消。
        if not node.move_through(q_home):
            results.append(dict(joint=j, ok=False, reason="could not reach baseline"))
            continue
        base = node.dwell_record(q_home, 1.0)
        if base is None:
            results.append(dict(joint=j, ok=False, reason="no baseline samples"))
            continue
        q_a = float(base["q_mean"][idx])

        if not node.move_through(q_plus):
            results.append(dict(joint=j, ok=False, reason="move failed"))
            node.move_through(q_home)
            continue
        rec = node.dwell_record(q_plus, 1.0)
        measured = float(rec["q_mean"][idx] - q_a) if rec else float("nan")
        node.move_through(q_home)

        # 这一步的首要目的是验符号。幅值只能粗查：移动 delta 会改变该关节的
        # 重力力矩，kp 限制的下垂量随之变化（joint2 约 0.3 Nm / kp 30 = 0.01 rad，
        # 即 delta=0.1 时约 10%），所以用相对容差而非绝对容差。
        scale_err = abs(measured - commanded)
        ratio = float(measured / commanded) if commanded else float("nan")
        sign_ok = bool(np.isfinite(measured)
                       and np.sign(measured) == np.sign(commanded))
        scale_ok = bool(np.isfinite(ratio)
                        and DIRECTION_SCALE_MIN <= abs(ratio) <= DIRECTION_SCALE_MAX)
        ok = sign_ok and scale_ok
        results.append(dict(joint=j, ok=ok, sign_ok=sign_ok, scale_ok=scale_ok,
                            commanded=commanded, measured=measured,
                            scale_error=scale_err, scale_ratio=ratio))
        node.get_logger().info(
            f"    commanded {commanded:+.4f} -> measured {measured:+.4f} rad  "
            f"ratio {ratio:+.3f}  sign {'OK' if sign_ok else 'INVERTED'}  "
            f"scale {'OK' if scale_ok else 'OFF'}")
    return results


def run_sweep(node: FkGravityVerifier, joint: int, n_points: int | None,
              dwell: float | None, csv_writer, is_warmup: bool = False) -> dict:
    plan = SWEEP_PLAN[joint]
    idx = joint - 1
    posture = list(plan["posture"])
    n_points = n_points or plan["points"]
    dwell = dwell if dwell is not None else plan["dwell"]
    qs = np.linspace(plan["lo"], plan["hi"], n_points)

    node.get_logger().info(
        ("=== WARM-UP " if is_warmup else "=== ") +
        f"joint{joint} sweep: {n_points} points x {dwell:g}s over "
        f"[{plan['lo']:.4f}, {plan['hi']:.4f}], model amplitude {plan['amp']:.4f} Nm, "
        f"expected 1-sigma {plan['sigma']:.4f} deg ===")

    q_start = np.array(posture, dtype=float)
    q_start[idx] = qs[0]
    if not node.move_through(q_start):
        return dict(joint=joint, ok=False, reason="could not reach start posture")

    samples = {"fwd": [], "rev": []}
    for direction, order in (("fwd", qs), ("rev", qs[::-1])):
        node.get_logger().info(f"  sweeping {direction}")
        for k, v in enumerate(order):
            q_t = np.array(posture, dtype=float)
            q_t[idx] = v
            # 第一个点用 move_through（可能是跨位姿转移，需要经中转点），
            # 之后每个点都用带过冲的单向进入 —— 两遍扫描的所有点都从
            # 同一方向进入，摩擦因此变成被 b 吸收的同号常数。
            ok = node.move_through(q_t) if k == 0 else node.approach(q_t, idx)
            if not ok:
                return dict(joint=joint, ok=False, reason=f"move failed at {v:.4f}")
            rec = node.dwell_record(q_t, dwell)
            if rec is None:
                return dict(joint=joint, ok=False, reason=f"no samples at {v:.4f}")
            # 记录【全部六个关节】的实测角，不只是被扫的那个。参考相位对
            # 其余关节的位置极其敏感，只留被扫关节会让分析退回到用标称位姿，
            # 那正是让 joint3 两次测量差 2.243 度的原因。
            samples[direction].append((rec["q_mean"].copy(), rec["tau_mean"][idx],
                                       rec["tau_std"][idx], rec["n"]))
            csv_writer.writerow([joint, f"{direction}{'_warmup' if is_warmup else ''}",
                                 f"{rec['q_mean'][idx]:.6f}",
                                 f"{rec['tau_mean'][idx]:.6f}",
                                 f"{rec['tau_std'][idx]:.6f}", rec["n"]]
                                + [f"{v:.6f}" for v in rec["q_mean"]])
            node.get_logger().info(
                f"    q={rec['q_mean'][idx]:+.4f}  tau={rec['tau_mean'][idx]:+.4f}"
                f" +/- {rec['tau_std'][idx]:.4f} Nm  (n={rec['n']})")

    fwd_q = np.array([s[0] for s in samples["fwd"]])
    rev_q = np.array([s[0] for s in samples["rev"]])[::-1]
    fwd_tau = np.array([s[1] for s in samples["fwd"]])
    rev_tau = np.array([s[1] for s in samples["rev"]])[::-1]

    q_full_avg = 0.5 * (fwd_q + rev_q)            # 六个关节的实测角，双向平均
    q_avg = q_full_avg[:, idx]
    tau_avg = 0.5 * (fwd_tau + rev_tau)           # 双向平均消掉静摩擦
    stiction = 0.5 * (fwd_tau - rev_tau)          # 半差就是静摩擦估计

    meas = fit_sine(q_avg, tau_avg)
    ref = model_reference_measured(node.model, joint, q_full_avg)
    ref_nominal = model_reference(node.model, joint, posture, q_avg)
    posture_bias_deg = float(np.degrees(zero_offset(ref_nominal["phi"], ref["phi"])))

    out = dict(joint=joint, ok=True, warmup=is_warmup, posture=posture,
               posture_measured_mean=q_full_avg.mean(axis=0).tolist(),
               posture_bias_deg=posture_bias_deg,
               unidirectional_violations=node.unidirectional_violations,
               points=n_points, dwell=dwell,
               expected_sigma_deg=plan["sigma"],
               threshold_deg=offset_threshold_deg(joint),
               measured=meas, model=ref,
               stiction_mean=float(np.mean(np.abs(stiction))),
               stiction_max=float(np.max(np.abs(stiction))))
    if meas["ok"] and ref["ok"]:
        out["amplitude_ratio"] = meas["amplitude"] / ref["amplitude"] \
            if ref["amplitude"] > 1e-9 else float("nan")
        out["zero_offset_rad"] = zero_offset(meas["phi"], ref["phi"])
        out["zero_offset_deg"] = float(np.degrees(out["zero_offset_rad"]))
    return out


def zero_offset(phi_measured: float, phi_model: float) -> float:
    """把拟合相位差换算成关节零位偏差，约定为 实际角 = 命令角 + offset。

    若真实关节比命令多转了 delta，则 tau(q_cmd) = A*sin(q_cmd + delta - phi_true)，
    拟合得到 phi_meas = phi_true - delta，即 phi_meas - phi_model = -delta。
    所以这里取负号。
    """
    wrapped = np.arctan2(np.sin(phi_measured - phi_model),
                         np.cos(phi_measured - phi_model))
    return float(-wrapped)


def print_plan(model: GravityModel, joints: list[int],
               n_points: int | None, dwell: float | None):
    print("=" * 78)
    print("PLAN (dry run — nothing is commanded)")
    print("=" * 78)
    total = 0.0
    for j in joints:
        if j not in SWEEP_PLAN:
            print(f"\n  joint{j}: NO GRAVITY SIGNAL — link6 CoM sits on the joint6 axis "
                  f"(y/z offset 0.05/0.10 mm), so rotating it neither moves the centre "
                  f"of mass nor the end effector. Its zero offset is unobservable this "
                  f"way; it only gets the direction/scale check.")
            continue
        plan = SWEEP_PLAN[j]
        pts = n_points or plan["points"]
        dw = dwell if dwell is not None else plan["dwell"]
        qs = np.linspace(plan["lo"], plan["hi"], pts)
        worst = 0.0
        bad = []
        for v in qs:
            q = np.array(plan["posture"], dtype=float)
            q[j - 1] = v
            rep = check_setpoint(q, model)
            if not rep.ok:
                bad.append((v, rep.reasons))
            worst = max(worst, float(np.max(np.abs(model.gravity(q)) / RATED_TORQUE)))
        secs = pts * 2 * (dw + 2.0)
        total += secs
        # sigma 只在使用该关节自带配置时才是标定过的
        tuned = (pts == plan["points"] and abs(dw - plan["dwell"]) < 1e-9)
        sigma_txt = (f"{plan['sigma']:.4f} deg (Monte-Carlo)" if tuned
                     else "unknown (config differs from the calibrated one)")
        print(f"\n  joint{j}: {pts} pts x 2 directions x {dw:g}s over "
              f"[{plan['lo']:.4f}, {plan['hi']:.4f}]")
        print(f"     other joints held at   : {plan['posture']}")
        print(f"     model amplitude        : {plan['amp']:.4f} Nm "
              f"(observed swing {plan['swing']:.4f})")
        print(f"     peak torque            : {worst*100:.1f}% of rated")
        print(f"     zero-offset 1-sigma    : {sigma_txt}")
        print(f"     decision threshold     : {offset_threshold_deg(j):.3f} deg")
        print(f"     estimated time         : {secs/60:.1f} min")
        if bad:
            print(f"     *** {len(bad)} SETPOINTS REFUSED ***")
            for v, reasons in bad[:3]:
                print(f"         q={v:.4f}: {'; '.join(reasons)}")
    print(f"\n  TOTAL ESTIMATED TIME: {total/60:.1f} min")
    print("=" * 78)


def check_position_path_active(node, timeout: float = 5.0) -> str | None:
    """确认节点在【位置模式】—— 否则本工具发的指令全部被无声忽略。

    StudentArmNode::ControlLoop 在 enable_direct_effort_command 为真时走力矩
    分支并 return，/student/joint_command 整条位置路径被旁路。本工具发位置
    指令，于是每个采样点都"settled with tracking error = 命令量"（关节纹丝
    不动），却照常跑完整个扫描并产出一份全是噪声的数据。

    实机上这个错误消耗了一次 25 分钟的扫描才被发现，而且它的表象
    （关节不动）与静摩擦、与通道失效都很像，很容易误诊。
    """
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, '/student_arm_node/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None          # 拿不到参数服务时不阻塞（可能是仿真/离线）
    req = GetParameters.Request()
    req.names = ['enable_direct_effort_command']
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.result() is None or not fut.result().values:
        return None
    if fut.result().values[0].bool_value:
        return ("student_arm_node is in DIRECT EFFORT mode — it bypasses the "
                "position path entirely, so every command from this tool would "
                "be silently ignored. Relaunch with "
                "enable_direct_effort_command:=false")
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--joints', default='1,2,3,4,5',
                    help='逗号分隔的关节号，默认 1,2,3,4,5（joint6 无重力信号）')
    ap.add_argument('--points', type=int, default=None,
                    help='覆盖扫描点数；默认用每个关节各自标定过的配置')
    ap.add_argument('--dwell', type=float, default=None,
                    help='覆盖每点采样秒数；默认用每个关节各自标定过的配置')
    ap.add_argument('--settle-sec', type=float, default=0.6,
                    help='判稳窗口长度：位置在该窗口内不再变化即认为停稳')
    ap.add_argument('--direction-delta', type=float, default=0.10,
                    help='方向校验的幅度 (rad)')
    ap.add_argument('--direction-only', action='store_true', help='只做方向校验')
    ap.add_argument('--converge', action='store_true', default=True,
                    help='反复扫描直到连续两遍的零位偏差吻合到 2-sigma 内，'
                         '而不是固定热身遍数。实测该量在到达位姿后单调漂移，'
                         '两轮分别从 5.023/4.494 度爬向 9.75/9.41 度，收敛所需'
                         '遍数不同（一轮 2 遍、一轮 3 遍仍未到），固定遍数不可靠。'
                         '同期 joint2 温度升了 4 度，其余关节基本不变。')
    ap.add_argument('--no-converge', dest='converge', action='store_false',
                    help='关闭自动收敛，改用固定 --warmup 遍数')
    ap.add_argument('--max-sweeps', type=int, default=6,
                    help='收敛模式下单个关节最多扫多少遍')
    ap.add_argument('--warmup', type=int, default=1,
                    help='到达测试位姿后先跑几遍【丢弃】的扫描。实测：到达新'
                         '位姿后的首扫会被污染 —— joint2 四次首扫散布在 '
                         '-7.9~+5.6 度，而热身后的两遍吻合到 0.007 度。')
    ap.add_argument('--repeat', type=int, default=1,
                    help='把每个关节重复扫描 N 次并报告离散度。'
                         '单次拟合得再好也不能证明结果正确 —— 本轮的教训是'
                         'joint2/joint3 各自单次残差都很小，重复测量却差 9 度。')
    ap.add_argument('--goto', default=None,
                    help='只驱动到指定位姿后退出，如 "0,1.32,-1.37,0,0,0"')
    ap.add_argument('--dry-run', action='store_true', help='只打印计划，不动机械臂')
    ap.add_argument('--output-dir',
                    default=str(Path(__file__).resolve().parent / 'data' / 'fk_verify'))
    args = ap.parse_args()

    joints = [int(x) for x in args.joints.split(',') if x.strip()]
    model = GravityModel()

    if args.dry_run:
        print_plan(model, joints, args.points, args.dwell)
        q0 = np.zeros(N_ARM)
        print(f"\n  EE(link6) at q=0 : {np.round(model.ee_position(q0), 5)} m")
        return 0

    if rclpy is None:
        print(f"rclpy unavailable: {_IMPORT_ERROR}", file=sys.stderr)
        return 2

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')

    rclpy.init()
    node = FkGravityVerifier(args)
    try:
        if not node.wait_for_state():
            node.get_logger().error(
                "no /joint_states or torque feedback. Start student_arm6.launch.py "
                "with publish_joint_feedback:=true first.")
            return 3

        why = check_position_path_active(node)
        if why:
            node.get_logger().error(why)
            return 6

        node.get_logger().info(f"joint order from /joint_states: {node.joint_order}")
        if node.joint_order != JOINT_NAMES:
            node.get_logger().error(
                f"unexpected joint order {node.joint_order}, expected {JOINT_NAMES}")
            return 4

        q_home = node.q.copy()
        node.get_logger().info(f"start pose: {np.round(q_home, 4).tolist()}")

        # 起始位姿是"机械臂恰好停在哪"，没有任何保证。上一轮扫描被中断时
        # 它停在了一个违反桌面约束的构型，而每轮重复结束都要返回 q_home，
        # 于是第二轮直接被拒、整轮中止。不安全就改用高位中转姿态当 home。
        rep_home = check_setpoint(q_home, model)
        if not rep_home.ok:
            node.get_logger().warn(
                "start pose is unsafe (" + "; ".join(rep_home.reasons) +
                "); using the safe waypoint as home instead")
            if not node.move_through(SAFE_WAYPOINT):
                node.get_logger().error("could not reach the safe waypoint")
                return 6
            q_home = node.q.copy()
            node.safety_abort = False
            node.get_logger().info(
                f"home reset to {np.round(q_home, 4).tolist()}")
        node.get_logger().info(
            f"predicted EE(link6): {np.round(model.ee_position(q_home), 4).tolist()} m "
            f"| distance from base_link: {np.linalg.norm(model.ee_position(q_home)):.4f} m")

        report = dict(timestamp=stamp, start_pose=q_home.tolist(),
                      urdf=model.path, joints=joints)

        if args.goto:
            q_goal = np.array([float(x) for x in args.goto.split(',')], dtype=float)
            rep = check_setpoint(q_goal, model)
            if not rep.ok:
                node.get_logger().error("REFUSED: " + "; ".join(rep.reasons))
                return 5
            node.get_logger().info(f"driving to {np.round(q_goal, 4).tolist()}")
            node.move_through(q_goal)
            node.dwell_record(q_goal, 2.0)
            node.get_logger().info(
                f"arrived at {np.round(node.q, 4).tolist()}, "
                f"tracking error {np.round(node.q - q_goal, 4).tolist()}")
            return 0

        report['direction_check'] = run_direction_check(
            node, joints, args.direction_delta, q_home)
        if args.direction_only:
            node.move_through(q_home)
        else:
            csv_path = out_dir / f'fk_verify_{stamp}.csv'
            with open(csv_path, 'w', newline='') as fh:
                w_csv = FlushingWriter(fh)
                w_csv.writerow(['joint', 'direction', 'q', 'tau_mean', 'tau_std', 'n']
                           + [f'q{k}_meas' for k in range(1, N_ARM + 1)])
                sweeps = []
                for j in joints:
                    if j not in SWEEP_PLAN:
                        node.get_logger().warn(
                            f"joint{j} has no gravity signal, skipping sweep")
                        sweeps.append(dict(joint=j, ok=False,
                                           reason='no gravity signal'))
                        continue
                    if args.converge:
                        prev = None
                        accepted = False
                        for k in range(args.max_sweeps):
                            r = run_sweep(node, j, args.points, args.dwell,
                                          w_csv, is_warmup=(prev is None))
                            if node.safety_abort or not r.get('ok'):
                                sweeps.append(r)
                                break
                            off = r.get('zero_offset_deg', float('nan'))
                            tol = 2.0 * SWEEP_PLAN[j]['sigma']
                            if prev is not None and abs(off - prev) <= tol:
                                node.get_logger().info(
                                    f"joint{j} converged: {prev:+.3f} -> {off:+.3f} deg "
                                    f"(within {tol:.3f} deg)")
                                r['warmup'] = False
                                r['converged'] = True
                                sweeps.append(r)
                                accepted = True
                                break
                            if prev is not None:
                                node.get_logger().info(
                                    f"joint{j} sweep {k + 1}: {off:+.3f} deg "
                                    f"(prev {prev:+.3f}, drift {off - prev:+.3f}) "
                                    f"— not converged, continuing")
                            r['warmup'] = True      # 未收敛的都算热身
                            sweeps.append(r)
                            prev = off
                        if not accepted and not node.safety_abort:
                            node.get_logger().error(
                                f"joint{j} did not converge in {args.max_sweeps} "
                                f"sweeps; all of them are marked as warm-up so "
                                f"nothing unconverged reaches the calibration")
                        continue
                    for wi in range(args.warmup):
                        node.get_logger().info(
                            f"--- joint{j} WARM-UP {wi + 1}/{args.warmup} "
                            f"(data recorded but excluded from the fit) ---")
                        r = run_sweep(node, j, args.points, args.dwell, w_csv,
                                      is_warmup=True)
                        sweeps.append(r)
                        if node.safety_abort or not r.get('ok'):
                            break
                    for rep in range(args.repeat):
                        if args.repeat > 1:
                            node.get_logger().info(
                                f"--- joint{j} repeat {rep + 1}/{args.repeat} ---")
                        r = run_sweep(node, j, args.points, args.dwell, w_csv)
                        r['repeat_index'] = rep
                        sweeps.append(r)
                        if node.safety_abort or not r.get('ok'):
                            break
                    if node.safety_abort:
                        node.get_logger().error(
                            f"safety check refused during joint{j}; aborting the "
                            f"whole run instead of continuing to the next joint")
                        report['aborted_at_joint'] = j
                        break
            report['sweeps'] = sweeps
            report['csv'] = str(csv_path)
            node.get_logger().info("returning to start pose")
            node.move_through(q_home)

        json_path = out_dir / f'fk_verify_{stamp}.json'
        with open(json_path, 'w') as fh:
            json.dump(report, fh, indent=2, default=float)
        node.get_logger().info(f"report written to {json_path}")
        print_verdict(report)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def print_verdict(report: dict):
    print("\n" + "=" * 78)
    print("VERDICT")
    print("=" * 78)
    for r in report.get('direction_check', []):
        if r.get('ok'):
            print(f"  joint{r['joint']} direction/scale : OK "
                  f"(cmd {r['commanded']:+.4f} -> meas {r['measured']:+.4f})")
        else:
            print(f"  joint{r['joint']} direction/scale : FAIL — "
                  f"{r.get('reason', 'sign or scale mismatch')}")

    if 'sweeps' not in report:
        return
    print()
    print(f"  {'joint':<8} {'A meas':>9} {'A model':>9} {'ratio':>7} "
          f"{'zero offset':>12} {'+/-1sig':>8} {'resid/A':>8} "
          f"{'stiction':>9}  verdict")
    offsets = {}
    for s in report['sweeps']:
        if not s.get('ok'):
            print(f"  joint{s['joint']:<3} -- {s.get('reason')}")
            continue
        m, ref = s['measured'], s['model']
        phi_deg = s.get('zero_offset_deg', float('nan'))
        sigma = s.get('expected_sigma_deg', float('nan'))
        # 纯零位偏差只平移相位，正弦形状仍然完美；只有几何错才会破坏形状。
        # 所以判几何故障要看归一化残差，不是看相位。
        rel_resid = (m['rms_residual'] / m['amplitude']
                     if m['amplitude'] > 1e-9 else float('inf'))
        if rel_resid > SHAPE_RESIDUAL_LIMIT:
            verdict = '*** GEOMETRY FAULT (shape) ***'
        elif abs(phi_deg) < 3.0 * sigma:
            verdict = 'zero offset not significant'
        else:
            verdict = 'zero offset -> apply as calibration'
            offsets[s['joint']] = phi_deg
        print(f"  joint{s['joint']:<3} {m['amplitude']:>9.4f} {ref['amplitude']:>9.4f} "
              f"{s.get('amplitude_ratio', float('nan')):>7.3f} "
              f"{phi_deg:>9.3f}deg {sigma:>8.4f} {rel_resid:>8.4f} "
              f"{s['stiction_mean']:>9.4f}  {verdict}")

    print("\n  怎么读这张表：")
    print("    resid/A 大   -> 正弦形状被破坏 = 真正的几何错（连杆长度/轴向），必须停下来查")
    print("    resid/A 小   -> 形状正确，模型结构没问题")
    print("    zero offset  -> 关节零位与 URDF 零位之差，是标定值不是故障，")
    print("                    URDF 零位与实际关节零位本就不同，这里把它量出来")
    print("    ratio != 1   -> 质量误差，阶段 2 标定，此前实测约 1.32~1.40")
    if offsets:
        print("\n  测得的零位偏差（实际角 = 命令角 + offset，单位 rad）：")
        vec = [round(np.radians(offsets.get(j, 0.0)), 6) for j in range(1, N_ARM + 1)]
        print(f"    {vec}")
        print("    joint6 无重力信号，其偏差保持 0")
    print("=" * 78)


if __name__ == '__main__':
    sys.exit(main())
