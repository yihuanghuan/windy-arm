"""桥接入 Vicon 原生 twist/accel 与 arm_ee 之后的行为。

【这个测试的存在理由】
桥此前只订阅 pose、自己差分。改成优先用 Vicon 原生 twist/accel 后，几件事
必须钉死：

1. **必须按【实机的到达顺序】喂数据。** 这是上一版测试全绿却功能空转的根因：
   老测试先喂 twist、再喂同戳 pose，而实机是 100% 反过来的 ——

       同一 Vicon 帧的 pose/twist 源时间戳完全相同   3000/3000
       同帧 twist 【晚于】 pose 到达                  3000/3000
       晚到多少                        中位 +0.100 ms, max 0.992 ms
       每帧最后到达的通道                accel, 2342/2342 恒定
       同帧三通道到达时间跨度            中位 0.158 ms, p95 0.424 ms, max 1.232 ms

   于是 pose 回调执行时缓冲区里最新的是【上一帧】，源错位恰好一个帧周期
   8.028 ms，超过 4 ms 判据，配对 100% 失败，四个 prefer_vicon_* 开关全是
   装饰品。测试构造了一个现实中从不发生的顺序，所以什么也没测住。
   下面所有配对测试一律用 ``_feed_frame``，它按实机顺序喂。

2. **stamp=0 的通道必须被整条丢弃。** 这就是 vrpn_listener 未修复时的现状
   （实测 5156/5156 条 twist/accel 都是 stamp=0、frame_id=''）。丢弃后自动
   退回本地差分，于是"listener 没修也能部署"这句话才成立。
3. **杠杆臂用基座外参（33.1 mm），不是末端标记杠杆臂（17.9 mm）。**
4. **arm_ee 必须走同一个桥换算世界系。** zero_on_first_pose 用第一帧基座位姿
   定义了控制器世界系，另起节点会落在不同世界系里。
"""

import importlib.util
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BRIDGE = os.path.join(ROOT, "task_space_smc", "hw", "vicon_floating_base_bridge.py")

R_BASE = np.array([0.019621, -0.026358, -0.003764])      # 33.07 mm


def _load():
    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("没有 ROS 环境")
    spec = importlib.util.spec_from_file_location("_vicon_bridge_channels", BRIDGE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def bridge_module():
    return _load()


@pytest.fixture
def bridge(bridge_module):
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    node = bridge_module.ViconFloatingBaseBridge()
    # 缺省参数下四条辅助话题都是空的，装配器里就只有 pose。这些用例要测的
    # 正是"配了辅助通道之后"的行为，所以把话题名填上再重建装配器。
    # "没配置的通道不等待"由 test_an_unconfigured_channel_adds_no_wait 单独守。
    node.twist_topic = "/vrpn/arm_base/twist"
    node.accel_topic = "/vrpn/arm_base/accel"
    node.ee_twist_topic = "/vrpn/arm_ee/twist"
    node.ee_accel_topic = "/vrpn/arm_ee/accel"
    node._build_assemblers()
    node._fake_now = 1000.0

    class _Clock:
        def __init__(self, owner):
            self.owner = owner

        def now(self):
            ns = int(round(self.owner._fake_now * 1e9))
            return types.SimpleNamespace(
                nanoseconds=ns,
                to_msg=lambda: bridge_module.Time(nanoseconds=ns).to_msg())

    node.get_clock = lambda: _Clock(node)
    node.published = []
    node.pub = types.SimpleNamespace(publish=node.published.append)
    node.ee_published = []
    node.ee_pub = types.SimpleNamespace(publish=node.ee_published.append)
    node.diag = []
    node.diag_pub = types.SimpleNamespace(publish=node.diag.append)
    node.t_rb = R_BASE.copy()
    yield node
    node.destroy_node()


def _pose(x=0.0, y=0.0, z=0.0, stamp=0.0):
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.w = 1.0
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    return msg


def _twist(lin, ang, stamp):
    from geometry_msgs.msg import TwistStamped
    msg = TwistStamped()
    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = [float(v) for v in lin]
    msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = [float(v) for v in ang]
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    return msg


def _accel(lin, ang, stamp):
    from geometry_msgs.msg import AccelStamped
    msg = AccelStamped()
    msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z = [float(v) for v in lin]
    msg.accel.angular.x, msg.accel.angular.y, msg.accel.angular.z = [float(v) for v in ang]
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    return msg


FRAME = 0.008021          # 实测帧周期（源时钟）
SPREAD = 0.0001           # 实测同帧通道到达跨度的量级：中位 0.100 ms


def _feed_pose(node, x, arrival, source):
    node._fake_now = arrival
    node._pose_cb(_pose(x=x, stamp=source))


def _feed_frame(node, source, arrival, x=0.0, twist=None, accel=None,
                spread=SPREAD):
    """按【实机到达顺序】喂基座的一帧：pose 先到，同戳 twist/accel 后到。

    实测 3000/3000 帧都是这个顺序，晚到中位 +0.100 ms。老测试反着喂（先
    twist 后 pose），于是构造出一个现实中从不发生的顺序 —— 那正是配对
    100% 失败却全绿的原因。
    """
    node._fake_now = arrival
    node._pose_cb(_pose(x=x, stamp=source))
    if twist is not None:
        node._fake_now = arrival + spread
        node._twist_cb(_twist(twist[0], twist[1], stamp=source))
    if accel is not None:
        node._fake_now = arrival + 2.0 * spread
        node._accel_cb(_accel(accel[0], accel[1], stamp=source))


def _feed_ee_frame(node, source, arrival, x=0.0, twist=None, accel=None,
                   orientation=None, spread=SPREAD):
    """按实机到达顺序喂 arm_ee 的一帧。"""
    node._fake_now = arrival
    msg = _pose(x=x, stamp=source)
    if orientation is not None:
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = orientation
    node._ee_cb(msg)
    if twist is not None:
        node._fake_now = arrival + spread
        node._ee_twist_cb(_twist(twist[0], twist[1], stamp=source))
    if accel is not None:
        node._fake_now = arrival + 2.0 * spread
        node._ee_accel_cb(_accel(accel[0], accel[1], stamp=source))


def _warmup(node, feed, source0=500.0, arrival0=1000.0, frames=3, **kw):
    """喂前几帧让装配器认识各条通道。

    装配器只把【已经来过】的通道算进"齐全"条件 —— 否则一条永远不来的
    stamp=0 通道会把每一帧都拖到超时。代价是头两三帧各通道还没被认全，
    这几帧照旧走本地差分。实机上就是 25 ms 的起步期。
    """
    for k in range(frames):
        feed(node, source=source0 + k * FRAME, arrival=arrival0 + k * FRAME, **kw)
    return source0 + frames * FRAME, arrival0 + frames * FRAME


def _feed_ee(node, arrival, source, x=0.0, y=0.0, z=0.0, orientation=None):
    """喂一帧 arm_ee 位姿。

    第一帧【不会】发出任何东西：还没有第二帧就没有导数，而
    FloatingBaseState 里的速度/加速度补零是绝对不允许的 —— 0 是下游任何守卫
    都识别不出来的合法值。所以要拿到输出至少得喂两帧。
    """
    node._fake_now = arrival
    msg = _pose(x=x, y=y, z=z, stamp=source)
    if orientation is not None:
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = orientation
    node._ee_cb(msg)


def _twist_of(out):
    return np.array([out.twist.linear.x, out.twist.linear.y, out.twist.linear.z])


def _ee_twist_of(out):
    return np.array([out.twist.linear.x, out.twist.linear.y, out.twist.linear.z])


def _ee_omega_of(out):
    return np.array([out.twist.angular.x, out.twist.angular.y,
                     out.twist.angular.z])


# ---------------- stamp=0 的通道必须被丢弃

def test_a_twist_with_a_zero_stamp_is_discarded(bridge):
    """stamp=0 = 没人填时间戳，无法与位姿同帧对齐，只能整条丢。

    这正是 vrpn_listener 未修复时的现状。丢弃之后桥自动退回本地差分，
    行为与改造前逐位相同 —— 这就是"listener 没修也能先部署桥"的依据。
    这样的通道【永远不会进入装配器】，所以也永远不会把帧拖到超时。
    """
    bridge._twist_cb(_twist([9.0, 0, 0], [0, 0, 0], stamp=0.0))
    bridge._accel_cb(_accel([9.0, 0, 0], [0, 0, 0], stamp=0.0))
    assert bridge._base_frames.required() == {"pose"}, \
        "stamp=0 的通道不该被算进齐全条件，否则每帧都要白等一个超时"


def test_a_twist_with_a_real_stamp_enters_the_assembler(bridge):
    """有真时间戳的 twist 才会被装配器收下，并从此计入齐全条件。"""
    bridge._fake_now = 1000.0
    bridge._twist_cb(_twist([0.1, 0.2, 0.3], [0.0, 0.0, 0.4], stamp=500.0))
    assert "twist" in bridge._base_frames.required()


def test_non_finite_channel_samples_are_dropped(bridge):
    bridge._fake_now = 1000.0
    bridge._twist_cb(_twist([float("nan"), 0, 0], [0, 0, 0], stamp=500.0))
    assert bridge._base_frames.required() == {"pose"}


# ---------------- 优先用 Vicon、否则回退

def test_vicon_twist_is_paired_in_the_real_arrival_order(bridge):
    """**这是这次修复的核心用例。**

    实测同帧 twist 恒【晚于】pose 到达（3000/3000，中位 +0.100 ms）。老实现
    在 pose 回调里就地去缓冲区找同帧 twist，找到的永远是上一帧，源错位
    8.028 ms > 4 ms 判据，配对 100% 失败 —— 而老测试先喂 twist 再喂 pose，
    把这个失败完全掩盖了。这里按实机顺序喂，必须配上。
    """
    bridge.tau_filter = 0.0
    src, arr = _warmup(bridge, _feed_frame, x=0.0,
                       twist=([7.0, 0.0, 0.0], [0.0, 0.0, 0.0]))
    _feed_frame(bridge, source=src, arrival=arr, x=0.01,
                twist=([7.0, 0.0, 0.0], [0.0, 0.0, 0.0]))
    assert bridge._used_vicon_twist, "按实机顺序喂之后仍然没配上"
    # 本地差分会给 1.247 m/s；Vicon 给 7.0，无角速度所以杠杆臂项为零。
    assert _twist_of(bridge.published[-1])[0] == pytest.approx(7.0, rel=1e-9)


def test_the_assembled_frame_waits_only_the_channel_arrival_spread(bridge):
    """额外延迟只有"同帧各通道的到达跨度"，不是一个帧周期。

    实测跨度中位 0.158 ms、max 1.232 ms；而"整体延迟一帧"要付 8 ms，在
    200 Hz 控制环上是 1.6 个控制拍。这条用例把这个数量级钉住。
    """
    bridge.tau_filter = 0.0
    src, arr = _warmup(bridge, _feed_frame, x=0.0,
                       twist=([1.0, 0, 0], [0, 0, 0]),
                       accel=([0.5, 0, 0], [0, 0, 0]))
    _feed_frame(bridge, source=src, arrival=arr, x=0.01,
                twist=([1.0, 0, 0], [0, 0, 0]),
                accel=([0.5, 0, 0], [0, 0, 0]), spread=0.0001)
    assert bridge._used_vicon_twist and bridge._used_vicon_accel
    # pose 到达 -> accel 到达 = 2 * spread = 0.2 ms
    assert bridge._frame_wait == pytest.approx(0.0002, abs=1e-9)
    assert bridge._frame_wait < 0.001, "额外延迟应在 0.2 ms 量级"


def test_it_falls_back_to_local_differentiation_when_the_channel_is_silent(bridge):
    """没有 Vicon twist 就回退本地差分，行为与改造前一致。"""
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.00, arrival=1000.000, source=500.000)
    _feed_pose(bridge, 0.01, arrival=1000.008, source=500.008)
    assert not bridge._used_vicon_twist
    assert _twist_of(bridge.published[-1])[0] == pytest.approx(1.25, rel=1e-6)


def test_a_channel_that_stops_arriving_is_released_on_timeout(bridge):
    """通道断了必须靠超时照常发布，绝不能卡住不发。

    没有这条兜底，一条时断时续的通道会把整条控制反馈拖死；而"卡住不发"
    比"回退本地差分"坏得多 —— 后者是既定的降级路径，前者是停摆。
    """
    bridge.tau_filter = 0.0
    src, arr = _warmup(bridge, _feed_frame, x=0.0,
                       twist=([7.0, 0, 0], [0, 0, 0]))
    assert "twist" in bridge._base_frames.required()
    before = len(bridge.published)

    # 这一帧 twist 再也不来
    _feed_frame(bridge, source=src, arrival=arr, x=0.01, twist=None)
    assert len(bridge.published) == before, "还没到超时就不该放行"
    bridge._fake_now = arr + 0.004          # > frame_completion_timeout 3 ms
    bridge._flush_frames()
    assert len(bridge.published) == before + 1, "超时后必须照常发布"
    assert not bridge._used_vicon_twist, "缺的那条通道要回退本地差分"
    assert bridge._base_frames.timeouts >= 1, "超时放行必须被计数"


def test_a_long_dead_channel_stops_costing_a_timeout(bridge):
    """通道长时间不来就移出必需集合，稳态回到零等待。

    否则每一帧都要白等一个 3 ms 超时 —— 在 5 ms 的控制周期上这不是小数。
    它一旦恢复，下一次 submit 会自动把它重新算进来。
    """
    bridge.tau_filter = 0.0
    src, arr = _warmup(bridge, _feed_frame, x=0.0,
                       twist=([7.0, 0, 0], [0, 0, 0]))
    for k in range(12):
        _feed_frame(bridge, source=src + k * FRAME, arrival=arr + k * FRAME,
                    x=0.01, twist=None)
        bridge._fake_now = arr + k * FRAME + 0.004
        bridge._flush_frames()
    assert bridge._base_frames.required() == {"pose"}, \
        "断了很久的通道仍在必需集合里，每帧都会白等一个超时"


# ---------------- 杠杆臂

def test_the_lever_arm_uses_the_base_extrinsic(bridge):
    """带角速度时，输出速度必须含 w x r，且 r 是 33.1 mm 的基座外参。"""
    bridge.tau_filter = 0.0
    # 采信开关现在默认【关闭】（上游 listener 发硬零，见 smc_params_hold.yaml
    # 的闸门算术）。这条用例验的是杠杆臂数学，需要角速度真的走 Vicon 那一路，
    # 所以显式打开；同时把对照容差放开，单独隔离杠杆臂本身。
    bridge.prefer_vicon_angular_velocity = True
    bridge.angular_cross_check_tol = 1.0e6
    omega = np.array([0.0, 0.0, 2.0])
    src, arr = _warmup(bridge, _feed_frame, x=0.0,
                       twist=([0.0, 0.0, 0.0], omega))
    _feed_frame(bridge, source=src, arrival=arr, x=0.0,
                twist=([0.0, 0.0, 0.0], omega))
    assert bridge._used_vicon_twist
    expected = np.cross(omega, R_BASE)          # 世界系旋转此时为单位阵
    assert np.allclose(_twist_of(bridge.published[-1]), expected, atol=1e-12)
    assert np.linalg.norm(expected) > 1e-3, "这个算例的杠杆臂项太小，测不出东西"


def test_vicon_angular_velocity_is_used_by_default(bridge):
    """角速度默认【采信 Vicon】—— 字段是有数据的。

    实测原始字段（arm_base 静止）：
        vel_quat    = [0.000000, -0.026500, 0.178300, -0.033800]   ||q|| != 1
        vel_quat_dt = 1.000000000        <- vrpn_Tracker 构造函数的缺省值
    没有确立的是【单位与约定】，不是"有没有数据"。所以默认采信，再用位姿
    差分做裁判。
    """
    # 缺省值现在是 False（上游发硬零，闸门会在慢速区间放行它）。这条用例
    # 验的是"打开时这条路确实通"，所以显式打开 —— 缺省值本身由
    # test_hold_qp_runner.py 的配置用例把关。
    assert bridge.prefer_vicon_angular_velocity is True, "Base 缺省必须采信 Vicon"
    bridge.prefer_vicon_angular_velocity = True
    bridge.tau_filter = 0.0
    bridge.angular_cross_check_tol = 1.0e6      # 只验证"确实被采用了"
    src, arr = _warmup(bridge, _feed_frame, x=0.0,
                       twist=([0.0, 0.0, 0.0], [0.0, 0.0, 0.7]))
    _feed_frame(bridge, source=src, arrival=arr, x=0.0,
                twist=([0.0, 0.0, 0.0], [0.0, 0.0, 0.7]))
    out = bridge.published[-1]
    assert out.twist.angular.z == pytest.approx(0.7, rel=1e-9)
    assert bridge._vicon_angular_ok


def test_an_inconsistent_vicon_angular_rate_falls_back_to_pose_differencing(bridge):
    """与位姿差分粗量级不符就这一拍回退，并计数。

    vel_quat 的单位/约定尚未核实（||q|| != 1、dt 仍是缺省值 1.0），所以不能
    盲信；但也不能一刀切禁用 —— 那样就永远没法验证它。用本地差分当裁判是
    唯一能同时做到"能用"和"错了会被发现"的做法。
    """
    bridge.tau_filter = 0.0
    bridge.prefer_vicon_angular_velocity = True      # 闸只在采信时才跑
    # 刚体静止（本地差分≈0），却报来 2.7 rad/s：远超容差
    bad = ([0.0, 0.0, 0.0], [1.54, 0.49, 2.66])
    src, arr = _warmup(bridge, _feed_frame, x=0.0, twist=bad)
    _feed_frame(bridge, source=src, arrival=arr, x=0.0, twist=bad)
    out = bridge.published[-1]
    angular = np.array([out.twist.angular.x, out.twist.angular.y,
                        out.twist.angular.z])
    assert np.allclose(angular, 0.0, atol=1e-12), "不一致的角速度漏进了输出"
    assert not bridge._vicon_angular_ok
    assert bridge._angular_reject >= 1, "回退必须被计数，否则没人知道它发生过"


def test_vicon_angular_acceleration_is_used_but_cross_checked(bridge):
    """角加速度也采信 Vicon，但同样要过位姿二阶差分的对照。

    它直接进 compute_task_bias_acceleration 变力矩：QA_record 量过，
    16 rad/s² 的伪角加速度在 0.3 m 力臂上就是 4.8 m/s² 的伪末端需求 ——
    所以"用"和"敢不敢盲信"是两件事，对照那道闸不能省。
    """
    assert bridge.prefer_vicon_angular_accel is True, "Base 缺省必须采信 Vicon"
    bridge.prefer_vicon_angular_accel = True         # 闸只在采信时才跑
    bridge.tau_filter = 0.0
    # 刚体静止（二阶差分≈0），却报来 40 rad/s²：远超容差，必须回退并计数
    bad = ([0.0, 0, 0], [40.0, 0, 0])
    src, arr = _warmup(bridge, _feed_frame, x=0.0, accel=bad)
    _feed_frame(bridge, source=src, arrival=arr, x=0.0, accel=bad)
    out = bridge.published[-1]
    assert out.acceleration.angular.x == pytest.approx(0.0, abs=1e-12)
    assert not bridge._vicon_angular_accel_ok
    assert bridge._angular_accel_reject >= 1


def test_base_vicon_angular_channels_are_rotated_from_rigid_body_frame(bridge):
    """listener 的槽 1..3 是刚体系量，桥必须按同帧姿态换到控制器世界系。"""
    bridge.t_rb = np.zeros(3)
    bridge.angular_cross_check_tol = 1.0e6
    bridge.prefer_vicon_angular_velocity = True
    bridge.prefer_vicon_angular_accel = True
    q_z_90 = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    # 刚体系 +x 经 90° 绕 z 旋转后是控制器世界系 +y。
    local_v = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    local_a = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    v, a, used_twist, used_accel, _ = bridge._vicon_twist_accel(
        np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
        q_z_90, local_v, local_a)
    assert used_twist and used_accel
    assert np.allclose(v[3:], [0.0, 1.0, 0.0], atol=1e-12)
    assert np.allclose(a[3:], [0.0, 1.0, 0.0], atol=1e-12)


# ---------------- arm_ee 转发

def test_arm_ee_lands_in_the_same_world_frame_as_base_link(bridge):
    """arm_ee 必须落在【base_link 原点】的世界系里，不是标记簇原点的。

    这是整个"用同一个桥"的理由。zero_on_first_pose 捕获的原点是基座位姿
    【经过 rigid_from_base 外参之后】的 base_link，所以世界原点比标记簇原点
    往前挪了 19.6 mm。另起一个节点转发 arm_ee 就会少这一步，两条数据落在
    相差 33 mm 的两个世界系里，而末端残差本身才 1.77 mm —— 错的比要测的大
    一个数量级。
    """
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 1.00, arrival=1000.000, source=500.000)   # 定义世界原点
    _feed_ee(bridge, arrival=1000.008, source=500.008, x=1.20)
    assert len(bridge.ee_published) == 0, "第一帧没有导数，整条不发才对"
    _feed_ee(bridge, arrival=1000.016, source=500.016, x=1.20)
    assert len(bridge.ee_published) == 1
    out = bridge.ee_published[-1]
    # 世界原点 = 1.0 + t_rb，所以 arm_ee 在控制器世界系里是 0.20 - t_rb
    expected = 0.20 - R_BASE
    assert out.pose.position.x == pytest.approx(expected[0], abs=1e-9)
    assert out.header.frame_id == "world"


def test_arm_ee_is_withheld_until_the_world_origin_exists(bridge):
    """基座还没定义世界原点之前不许发 arm_ee，否则它落在另一个世界系里。"""
    bridge._fake_now = 1000.0
    bridge._ee_cb(_pose(x=1.2, stamp=500.0))
    assert len(bridge.ee_published) == 0


def test_arm_ee_orientation_is_passed_through_for_logging_only(bridge):
    """姿态原样透传，只供记录残差；控制回路永不使用（见 ee_estimator）。"""
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)
    quat = (0.0, 0.0, 0.7071, 0.7071)
    _feed_ee(bridge, arrival=1000.008, source=500.008, orientation=quat)
    _feed_ee(bridge, arrival=1000.016, source=500.016, orientation=quat)
    out = bridge.ee_published[-1]
    assert out.pose.orientation.z == pytest.approx(0.7071)


# ---------------- arm_ee 的 twist/accel 通道

def test_arm_ee_publishes_a_floating_base_state_not_a_pose(bridge):
    """消息类型必须是 FloatingBaseState —— 位姿/速度/加速度共用一个时间戳。

    拆成三个话题会把跨话题时间对齐这个问题原封不动地丢给下游，而那正是这个
    项目在基座上花大力气解决掉的：源时钟与本地时钟相差 11.992 h，本地到达
    间隔在 0.09~180 ms 之间散开，靠到达时刻配对必然配错帧。
    """
    from dummy_interface.msg import FloatingBaseState
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)
    _feed_ee(bridge, arrival=1000.008, source=500.008, x=0.00)
    _feed_ee(bridge, arrival=1000.016, source=500.016, x=0.01)
    out = bridge.ee_published[-1]
    assert isinstance(out, FloatingBaseState)
    # 本地差分：10 mm / 8 ms = 1.25 m/s
    assert _ee_twist_of(out)[0] == pytest.approx(1.25, rel=1e-6)


def test_arm_ee_twist_is_paired_in_the_real_arrival_order(bridge):
    """末端通路同样必须在【实机顺序】下配上。基座与末端各守一条。

    末端和基座走的是同一个 FrameAssembler 类 —— 理由与 SourceClockAligner
    相同：两份相似但不同的实现正是这个项目反复踩过的坑。所以两条通路都要
    有自己的实机顺序用例，不能只测一条。
    """
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)   # 定义世界原点
    src, arr = _warmup(bridge, _feed_ee_frame, source0=500.008, arrival0=1000.008,
                       x=0.00, twist=([3.0, 0.0, 0.0], [0, 0, 0]))
    _feed_ee_frame(bridge, source=src, arrival=arr, x=0.01,
                   twist=([3.0, 0.0, 0.0], [0, 0, 0]))
    assert bridge._ee_used_vicon_twist, "末端按实机顺序喂之后仍然没配上"
    assert _ee_twist_of(bridge.ee_published[-1])[0] == pytest.approx(3.0, rel=1e-9)
    assert bridge._ee_frame_wait == pytest.approx(SPREAD, abs=1e-9)


def test_arm_ee_pairing_uses_source_time_not_arrival_time(bridge):
    """配对键是【源时间戳】，不是到达时间。

    源时钟的帧间隔是 8.021 ms（精确），而本地到达间隔被 WiFi 压成串，
    从 0.09 ms 到 180 ms 散开。用到达时间配对会把一整串挤在一起投递的帧
    配到同一拍上。这里把到达时刻整体推迟 484 ms，源戳不变，仍须配上。
    """
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)
    src, arr = _warmup(bridge, _feed_ee_frame, source0=500.008, arrival0=1000.008,
                       x=0.00, twist=([3.0, 0.0, 0.0], [0, 0, 0]))
    # 到达时刻突然晚了 484 ms（WiFi 成串投递），源戳照旧
    _feed_ee_frame(bridge, source=src, arrival=arr + 0.484, x=0.01,
                   twist=([3.0, 0.0, 0.0], [0, 0, 0]))
    assert bridge._ee_used_vicon_twist, "源时间对得上就必须配上"
    assert _ee_twist_of(bridge.ee_published[-1])[0] == pytest.approx(3.0, rel=1e-9)


def test_an_arm_ee_channel_with_a_zero_stamp_is_discarded_whole(bridge):
    """stamp=0 的末端通道也是整条丢弃，自动退回位姿差分。

    与基座同一条规则、同一个理由：vrpn_listener 历史上 twist/accel 全是
    stamp=0（实测 5156/5156）。现已修复，但这条路径必须留着 —— 它保证了
    listener 万一没更新，这套改动照样能部署。

    关键回归点：这样的通道【永远不会 submit】，所以永远不会进必需集合，
    帧靠 pose 一到就齐全，不会被拖到超时。
    """
    bridge._fake_now = 1000.0
    bridge._ee_twist_cb(_twist([9.0, 0, 0], [0, 0, 0], stamp=0.0))
    bridge._ee_accel_cb(_accel([9.0, 0, 0], [0, 0, 0], stamp=0.0))
    assert bridge._ee_frames.required() == {"pose"}

    # 丢掉之后行为与只有位姿时逐位相同，且【不需要】等超时
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)
    _feed_ee_frame(bridge, source=500.008, arrival=1000.008, x=0.00)
    _feed_ee_frame(bridge, source=500.016, arrival=1000.016, x=0.01)
    assert not bridge._ee_used_vicon_twist
    assert bridge._ee_frame_wait == pytest.approx(0.0, abs=1e-12)
    assert _ee_twist_of(bridge.ee_published[-1])[0] == pytest.approx(1.25, rel=1e-6)


def test_arm_ee_velocity_is_rotated_into_the_controller_world_frame(bridge):
    """Vicon 原生量在 VRPN 世界系里，必须先转进控制器世界系。

    本地差分出来的量天然就在控制器世界系里（它差的是已经变换+归零过的位姿）。
    两者不是同一个系；直接混用等于悄悄换了坐标系，而且一致性对照会变成跨
    坐标系相减 —— 那道闸本身就是错的。这个 bug 在基座通路上真实发生过。
    """
    bridge.tau_filter = 0.0
    # 让基座第一帧带一个 90° 偏航，于是 zero_q 非单位，控制器世界系确实转了。
    bridge._fake_now = 1000.000
    msg = _pose(x=0.0, stamp=500.000)
    msg.pose.orientation.z = msg.pose.orientation.w = np.sqrt(0.5)
    bridge._pose_cb(msg)
    assert bridge.zero_q is not None

    src, arr = _warmup(bridge, _feed_ee_frame, source0=500.008, arrival0=1000.008,
                       x=0.0, twist=([1.0, 0.0, 0.0], [0, 0, 0]))
    _feed_ee_frame(bridge, source=src, arrival=arr, x=0.0,
                   twist=([1.0, 0.0, 0.0], [0, 0, 0]))
    assert bridge._ee_used_vicon_twist
    # R(conj(zero_q)) 是绕 z 转 -90°：vicon 系的 +x 变成控制器系的 -y
    got = _ee_twist_of(bridge.ee_published[-1])
    assert np.allclose(got, [0.0, -1.0, 0.0], atol=1e-9), got


def test_the_arm_ee_path_adds_no_lever_arm(bridge):
    """末端通路【不加】杠杆臂 —— 加了就是搬了两次。

    Vicon 的 arm_ee 刚体报的 pose/twist 就在标记簇原点，而控制点就是这个标记
    原点，两点重合，w x r 恒等于零。config 里 arm_ee_from_link6 的 17.9 mm 是
    给【FK 一侧】用的（把 FK 从 link6 挪到标记点，实现在
    EeFeedbackEstimator.control_point_* 里）。

    这个算例特意给了 2 rad/s 的角速度：若误用基座外参 33.1 mm 当杠杆臂，
    输出会多出 |w x r| = 66 mm/s，测得出来。
    """
    bridge.tau_filter = 0.0
    bridge.prefer_vicon_angular_velocity = True
    bridge.angular_cross_check_tol = 1.0e6      # 单独验证杠杆臂数学本身
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)
    omega = np.array([0.0, 0.0, 2.0])
    src, arr = _warmup(bridge, _feed_ee_frame, source0=500.008, arrival0=1000.008,
                       x=0.0, twist=([0.0, 0.0, 0.0], omega))
    _feed_ee_frame(bridge, source=src, arrival=arr, x=0.0,
                   twist=([0.0, 0.0, 0.0], omega))
    assert bridge._ee_used_vicon_twist
    got = _ee_twist_of(bridge.ee_published[-1])
    assert np.allclose(got, np.zeros(3), atol=1e-12), \
        "末端线速度里混进了杠杆臂项 %s" % got
    # 若误加了基座杠杆臂，这里会是 66 mm/s —— 确认这个算例真的测得出来
    assert np.linalg.norm(np.cross(omega, R_BASE)) > 1e-3


def test_an_inconsistent_arm_ee_angular_rate_falls_back_and_counts(bridge):
    """末端角速度也要与位姿差分对照，不符就这一拍回退并计数。

    arm_ee 姿态相对 link6 差 114°，但那是【常量】旋转，常量旋转不改变世界系
    角速度 —— 所以位姿差分出来的 ω 即使姿态本身不可用，当裁判依然成立。
    """
    bridge.tau_filter = 0.0
    bridge.prefer_ee_vicon_angular_velocity = True   # 末端角量测试显式开启
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)
    # 刚体静止（差分≈0），却报来 2.7 rad/s：远超容差
    bad = ([0.0, 0, 0], [1.54, 0.49, 2.66])
    src, arr = _warmup(bridge, _feed_ee_frame, source0=500.008, arrival0=1000.008,
                       x=0.0, twist=bad)
    _feed_ee_frame(bridge, source=src, arrival=arr, x=0.0, twist=bad)
    assert np.allclose(_ee_omega_of(bridge.ee_published[-1]), 0.0, atol=1e-12), \
        "不一致的角速度漏进了末端输出"
    assert not bridge._ee_vicon_angular_ok
    assert bridge._ee_angular_reject >= 1, "回退必须被计数，否则没人知道它发生过"


def test_an_inconsistent_arm_ee_angular_accel_falls_back_and_counts(bridge):
    """末端角加速度同样过对照闸。"""
    bridge.tau_filter = 0.0
    bridge.prefer_ee_vicon_angular_accel = True      # 末端角量测试显式开启
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)
    bad = ([0.0, 0, 0], [40.0, 0, 0])
    src, arr = _warmup(bridge, _feed_ee_frame, source0=500.008, arrival0=1000.008,
                       x=0.0, accel=bad)
    _feed_ee_frame(bridge, source=src, arrival=arr, x=0.0, accel=bad)
    out = bridge.ee_published[-1]
    assert out.acceleration.angular.x == pytest.approx(0.0, abs=1e-12)
    assert not bridge._ee_vicon_angular_accel_ok
    assert bridge._ee_angular_accel_reject >= 1


def test_arm_ee_is_withheld_rather_than_publishing_zero_velocity(bridge):
    """导数建立不起来时整条不发，【绝不】把速度补零发下去。

    0 是下游任何守卫都识别不出来的合法值。整条不发，运行器既有且已测的
    陈旧性守卫（core/base_state.py 的 latest 抛 stale）就会自动接管。
    """
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.0, arrival=1000.000, source=500.000)
    _feed_ee(bridge, arrival=1000.008, source=500.008, x=0.0)
    assert len(bridge.ee_published) == 0
    assert bridge._ee_withheld >= 1, "停发必须被计数"


# ---------------- 诊断

def test_diagnostics_report_which_source_was_used(bridge):
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.00, arrival=1000.000, source=500.000)
    bridge._twist_cb(_twist([7.0, 0, 0], [0, 0, 0], stamp=500.008))
    _feed_pose(bridge, 0.01, arrival=1000.008, source=500.008)
    data = bridge.diag[-1].data
    assert data[4] == 1.0, "used_vicon_twist 必须被如实记录"
    assert data[6] == 0.0, "这一拍没有降级"


def test_diagnostics_carry_the_arm_ee_channel_state(bridge):
    """末端通路的诊断追加在【尾部】：已有看板按位置读前 14 个。"""
    bridge.tau_filter = 0.0
    _feed_pose(bridge, 0.00, arrival=1000.000, source=500.000)
    src, arr = _warmup(bridge, _feed_ee_frame, source0=500.008, arrival0=1000.008,
                       x=0.00, twist=([3.0, 0, 0], [0, 0, 0]))
    _feed_ee_frame(bridge, source=src, arrival=arr, x=0.01,
                   twist=([3.0, 0, 0], [0, 0, 0]))
    _feed_pose(bridge, 0.01, arrival=arr + 0.004, source=src + 0.004)
    data = bridge.diag[-1].data
    assert len(data) == 26, "诊断数组长度变了，读它的脚本会错位"
    assert data[14] == 1.0, "ee_used_vicon_twist 必须被如实记录"


def test_diagnostics_expose_the_assembly_wait(bridge):
    """索引 22/23 是装配等待，也就是这次修复引入的【全部】额外延迟。

    修之前 used_vicon_twist 恒为 0（配对 100% 失败）；修之后它必须恒为 1，
    而代价只有这个等待。把它放进诊断，是为了实机上能直接读出"到底多付了
    多少延迟"，而不是靠推断。
    """
    bridge.tau_filter = 0.0
    src, arr = _warmup(bridge, _feed_frame, x=0.0,
                       twist=([1.0, 0, 0], [0, 0, 0]),
                       accel=([0.5, 0, 0], [0, 0, 0]))
    _feed_frame(bridge, source=src, arrival=arr, x=0.01,
                twist=([1.0, 0, 0], [0, 0, 0]),
                accel=([0.5, 0, 0], [0, 0, 0]))
    data = bridge.diag[-1].data
    assert data[4] == 1.0, "used_vicon_twist 必须为 1 —— 这是本次修复的硬指标"
    assert data[22] == pytest.approx(0.0002, abs=1e-9)
    assert data[24] == 0.0, "正常链路不该有装配超时"


# ---------------- 零延迟回归：没配置的通道绝不引入等待

def test_an_unconfigured_channel_adds_no_wait(bridge_module):
    """只订阅 pose 时，pose 一到就发，额外延迟严格为 0。

    这是整个装配方案的回归底线：装配器只等【已配置且来过】的通道。如果
    没配置的通道也被算进"齐全"条件，那么每一帧都要白等一个完成超时 ——
    对只用 pose 的既有部署来说，那是纯亏。
    """
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    node = bridge_module.ViconFloatingBaseBridge()      # 四条辅助话题都为空
    try:
        node._fake_now = 1000.0

        class _Clock:
            def __init__(self, owner):
                self.owner = owner

            def now(self):
                ns = int(round(self.owner._fake_now * 1e9))
                return types.SimpleNamespace(
                    nanoseconds=ns,
                    to_msg=lambda: bridge_module.Time(nanoseconds=ns).to_msg())

        node.get_clock = lambda: _Clock(node)
        node.published = []
        node.pub = types.SimpleNamespace(publish=node.published.append)
        node.diag_pub = types.SimpleNamespace(publish=lambda m: None)
        node.tau_filter = 0.0

        assert node._base_frames.required() == {"pose"}
        _feed_pose(node, 0.00, arrival=1000.000, source=500.000)
        _feed_pose(node, 0.01, arrival=1000.008, source=500.008)
        assert len(node.published) == 1, "pose 到了就该立刻发，不该等任何东西"
        assert node._frame_wait == 0.0, "没配置辅助通道却引入了等待"
    finally:
        node.destroy_node()
