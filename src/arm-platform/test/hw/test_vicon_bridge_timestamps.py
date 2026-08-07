"""Vicon 桥的时间戳来源回归测试。

守住的核心事实（背景见 docs/VICON_TIMESTAMP_JITTER.md）：
  * 微分用【Vicon 源时钟】的差，不用本地到达间隔。WiFi 把 UDP 包压成串投递，
    实测最小到达间隔 0.087 ms，拿它当 dt 会把速度放大 ~90 倍；
  * 发布的 header.stamp 仍在【本地时钟】域（陈旧性判据要和本机 now 比），
    但它是【采集时刻】而不是【到达时刻】—— 否则一条飞了 180 ms 的样本会被
    报成"0 ms 新鲜"，安全判据形同虚设；
  * 源时间戳缺失（为 0）时必须回落到到达时刻，而不是崩掉或产出垃圾。
"""

import importlib.util
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
BRIDGE = os.path.join(ROOT, "task_space_smc", "hw",
                      "vicon_floating_base_bridge.py")


def _load_bridge_module():
    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("没有 ROS 环境")
    spec = importlib.util.spec_from_file_location(
        "_vicon_bridge_under_test", BRIDGE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def bridge_module():
    return _load_bridge_module()


@pytest.fixture
def bridge(bridge_module):
    """起一个真节点，但把时钟换成可控的假时钟。

    不打桩被测方法 —— 打桩就等于测试自己写了一遍实现。
    """
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    node = bridge_module.ViconFloatingBaseBridge()
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


def _feed(node, x, arrival, source):
    node._fake_now = arrival
    node._pose_cb(_pose(x=x, stamp=source))


def _last_velocity(node):
    out = node.published[-1]
    return np.array([out.twist.linear.x, out.twist.linear.y, out.twist.linear.z])


# ------------------------------------------------------ dt 的来源

def test_velocity_uses_source_dt_not_arrival_dt(bridge):
    """两帧真实间隔 8 ms，但被 WiFi 压成 0.1 ms 到达。

    速度必须按 8 ms 算（0.01/0.008 = 1.25 m/s），
    而不是按 0.1 ms 算（0.01/0.0001 = 100 m/s）—— 差 80 倍。
    """
    bridge.tau_filter = 0.0                      # 关滤波，直接看原始差分
    _feed(bridge, 0.00, arrival=1000.0000, source=500.000)
    _feed(bridge, 0.01, arrival=1000.0001, source=500.008)   # 到达只差 0.1 ms
    v = _last_velocity(bridge)
    assert v[0] == pytest.approx(0.01 / 0.008, rel=1e-6)
    assert abs(v[0]) < 10.0, "按到达间隔算会得到 ~100 m/s"


def test_long_delivery_gap_does_not_shrink_velocity(bridge):
    """反向也要对：投递卡了 200 ms，但真实只过了 8 ms。

    按到达间隔算会把速度【低估】25 倍，控制器会以为基座几乎没动。
    """
    bridge.tau_filter = 0.0
    _feed(bridge, 0.00, arrival=1000.000, source=500.000)
    _feed(bridge, 0.01, arrival=1000.200, source=500.008)
    assert _last_velocity(bridge)[0] == pytest.approx(0.01 / 0.008, rel=1e-6)


def test_sample_gap_is_judged_on_the_source_clock(bridge):
    """真实丢帧才该重置导数估计，投递打嗝不该。

    改动前按到达间隔判，90 秒里误触发 262 次。
    """
    bridge.tau_filter = 0.0
    _feed(bridge, 0.00, arrival=1000.000, source=500.000)
    # 到达间隔 300 ms（远超 max_gap），但源时钟只过了 8 ms -> 不算空档
    _feed(bridge, 0.01, arrival=1000.300, source=500.008)
    assert _last_velocity(bridge)[0] == pytest.approx(1.25, rel=1e-6)

    # 源时钟真的跳了 500 ms -> 算空档。
    published_before = len(bridge.published)
    _feed(bridge, 0.02, arrival=1000.310, source=500.508)
    assert bridge._stats_dropped >= 1, "真实空档必须被计入丢帧统计"
    assert len(bridge.published) == published_before, (
        "空档时必须【整条不发】，而不是发一条速度/加速度为零的消息")


def test_a_gap_withholds_the_message_instead_of_publishing_zeros(bridge):
    """空档时不许把零当成测量值发下去。

    这是相对旧行为的【有意改动】。旧实现在空档时发一条 v=a=0 的
    FloatingBaseState，而 0 是一个下游任何守卫都识别不出来的"合法值"：
    运行器会拿它当真实基座状态去算补偿力矩。QA_record 已经量过这条路的代价
    —— 伪造的基座加速度 16 rad/s² 在 0.3 m 力臂上就是 4.8 m/s² 的伪末端需求。

    整条不发反而让既有且已测的守卫自动做对的事：RosFeedbackBaseState.latest
    抛 "base state is stale"，控制环那一拍不发力矩，驱动按指令超时保位。
    """
    bridge.tau_filter = 0.0
    # 第一帧没有前一帧，导数无从谈起，同样【不发】—— 等一帧（约 8 ms）而已。
    _feed(bridge, 0.00, arrival=1000.000, source=500.000)
    assert len(bridge.published) == 0, "起步第一帧没有导数，也不该发零"
    _feed(bridge, 0.01, arrival=1000.008, source=500.008)
    assert len(bridge.published) == 1

    _feed(bridge, 0.02, arrival=1000.020, source=501.000)     # 1 s 真空档
    assert len(bridge.published) == 1, "空档那一拍不该有新消息"

    # 恢复之后必须继续正常发布，不能就此卡死。
    _feed(bridge, 0.03, arrival=1000.030, source=501.008)
    assert len(bridge.published) == 2


# ------------------------------------------------------ 发布的时间戳

def test_published_stamp_is_capture_time_not_arrival_time(bridge):
    """一条飞了 150 ms 的样本，必须被报成 150 ms 旧。

    到达打戳会把它报成 0 ms 新鲜，运行器的 base_state_timeout_sec 就白设了。
    """
    # 先喂一批低延迟样本，让钟差偏置收敛到"零延迟"那条
    for i in range(20):
        _feed(bridge, 0.0, arrival=1000.0 + 0.008 * i, source=500.0 + 0.008 * i)
    # 再喂一条延迟 150 ms 的
    _feed(bridge, 0.0, arrival=1000.310, source=500.160)

    out = bridge.published[-1]
    stamp = out.header.stamp.sec + 1e-9 * out.header.stamp.nanosec
    age = 1000.310 - stamp
    assert age == pytest.approx(0.150, abs=2e-3), (
        "样本年龄应约 150 ms，实得 %.1f ms" % (1e3 * age))


def test_low_latency_sample_is_reported_as_fresh(bridge):
    for i in range(20):
        _feed(bridge, 0.0, arrival=1000.0 + 0.008 * i, source=500.0 + 0.008 * i)
    out = bridge.published[-1]
    stamp = out.header.stamp.sec + 1e-9 * out.header.stamp.nanosec
    assert bridge._fake_now - stamp == pytest.approx(0.0, abs=1e-6)


def test_clock_offset_uses_the_minimum_not_the_mean(bridge):
    """偏置取窗口内的【最小】差 —— 延迟最低那条才最接近纯钟差。

    取均值会把常态延迟算进偏置，于是所有样本都被报成偏新鲜。
    """
    _feed(bridge, 0.0, arrival=1000.000, source=500.000)   # 延迟 0
    _feed(bridge, 0.0, arrival=1000.108, source=500.008)   # 延迟 100 ms
    _feed(bridge, 0.0, arrival=1000.216, source=500.016)   # 延迟 200 ms
    assert bridge._clock_offset(1000.224, 500.024) == pytest.approx(500.0)


def test_offset_window_lets_the_estimate_follow_clock_drift(bridge):
    """滑动双窗口：旧窗口过期后，偏置要能跟着时钟漂移走。

    一次性定标会被某条走运的早期包锁死，长跑时年龄会越报越错。
    """
    bridge.clock_offset_window = 1.0
    bridge._clock_offset(1000.0, 500.0)                      # 偏置 500.0
    for k in range(1, 6):
        offset = bridge._clock_offset(1000.0 + k, 499.0 + k)  # 偏置漂到 501.0
    assert offset == pytest.approx(501.0)


# ------------------------------------------------------ 回落与配置

def test_missing_source_stamp_falls_back_to_arrival(bridge):
    """源时间戳为 0 = 没人填。这时到达时刻确实是唯一可用的信息。"""
    bridge.tau_filter = 0.0
    _feed(bridge, 0.00, arrival=1000.000, source=0.0)
    _feed(bridge, 0.01, arrival=1000.008, source=0.0)
    assert _last_velocity(bridge)[0] == pytest.approx(1.25, rel=1e-6)
    out = bridge.published[-1]
    stamp = out.header.stamp.sec + 1e-9 * out.header.stamp.nanosec
    assert stamp == pytest.approx(1000.008, abs=1e-6)


def test_source_timestamp_can_be_switched_off_for_comparison(bridge_module):
    """留一个开关退回旧行为，方便做 A/B 对照。"""
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    node = bridge_module.ViconFloatingBaseBridge()
    node.use_source_timestamp = False
    assert node._source_time(_pose(stamp=500.0)) is None
    node.destroy_node()


def test_input_queue_is_deep_enough_for_burst_delivery(bridge_module):
    """深度 10 会在节点【内部】丢掉网络已经送达的帧：实测 2.4G 丢 4.5%、5G 丢 14%。"""
    src = open(BRIDGE, encoding="utf-8").read()
    i = src.index('self.declare_parameter("input_queue_depth"')
    assert "200" in src[i:i + 80], "订阅队列默认深度应为 200"
    assert "self._pose_cb, queue_depth" in src, "订阅必须用该参数，不能写死"


def test_hold_config_timeout_matches_capture_stamping():
    """改用采集打戳后年龄天然变大，0.1 s 会让 3.47% 的控制拍停发力矩。"""
    import yaml
    path = os.path.join(ROOT, "task_space_smc", "config", "smc_params_hold.yaml")
    cfg = yaml.safe_load(open(path, encoding="utf-8"))
    timeout = cfg["safety"]["base_state_timeout_sec"]
    assert timeout == pytest.approx(0.15)
    # 上界是硬的：190 ms 前的基座位姿拿去算补偿本身就是错的。
    assert timeout <= 0.2


# ------------------------------------------------------ 乱序保护

def test_base_state_ignores_an_older_sample():
    """桥排空积压时会连发一串真实很旧的帧。乱序时旧帧不能顶掉新帧。

    这条陈旧性判据看不出来：它比的是时间戳，而那个旧时间戳本身是诚实的。
    """
    from task_space_smc.core.base_state import BaseState, RosFeedbackBaseState

    def _sample(stamp):
        return BaseState(
            position_w=np.array([stamp, 0.0, 0.0]),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            rotation_wb=np.eye(3), velocity_lwa=np.zeros(6),
            acceleration_lwa=np.zeros(6), stamp=float(stamp))

    holder = RosFeedbackBaseState(timeout_sec=10.0)
    holder.update(_sample(100.0))
    holder.update(_sample(99.5))                 # 迟到的旧帧
    assert holder.latest(100.0).stamp == pytest.approx(100.0)
    holder.update(_sample(100.5))                # 新帧照常接受
    assert holder.latest(100.5).stamp == pytest.approx(100.5)
