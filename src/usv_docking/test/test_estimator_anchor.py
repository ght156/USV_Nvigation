#!/usr/bin/env python3
"""docking_pose_estimator 锚点逻辑单元测试（离线直调 _update_anchor）。

覆盖：首锚共识播种（中位数 + 离散度滑窗）、跳变拒绝簇吸附解锁、
接受帧清零拒绝簇、重置清空缓冲、TF 查询非阻塞约定。
"""

import math
import time

import pytest
import rclpy
from std_msgs.msg import Bool

from usv_docking.docking_pose_estimator import DockingPoseEstimator


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def est(ros_context):
    node = DockingPoseEstimator()
    yield node
    node.destroy_node()


def _seed_default_anchor(est, pose=(0.0, 0.0, 0.0)):
    for _ in range(8):
        est._update_anchor(pose)
    assert est._anchor is not None


def test_seed_requires_full_window(est):
    """未集满 seed_frames 帧不播种。"""
    for i in range(7):
        est._update_anchor((10.0, 5.0, 1.0))
        assert est._anchor is None
        assert len(est._seed_buf) == i + 1
    est._update_anchor((10.0, 5.0, 1.0))
    assert est._anchor is not None
    assert est._seed_buf == []


def test_seed_consensus_median(est):
    """首锚取位置中位数 / yaw 矢量平均，抗个别离群帧。"""
    for i in range(8):
        # x/y 各含一个 ±0.3 离群（< 离散门限 0.40），中位数应落在簇中心
        est._update_anchor(
            (10.0 + (0.3 if i == 2 else 0.0),
             5.0 + (-0.3 if i == 5 else 0.0),
             1.0)
        )
    ax, ay, ayaw = est._anchor
    assert ax == pytest.approx(10.0, abs=1e-6)
    assert ay == pytest.approx(5.0, abs=1e-6)
    assert ayaw == pytest.approx(1.0, abs=1e-6)


def test_seed_spread_gate_slides_window(est):
    """离散度超限不播种（双码切换场景），滑窗直到窗口一致。"""
    for i in range(8):
        # 两簇相距 1.2m（> 0.40 门限）交替：仿真单双码系统差
        est._update_anchor((10.0 + (1.2 if i % 2 else 0.0), 5.0, 0.0))
    assert est._anchor is None
    # 之后 8 帧一致观测 -> 播种
    for _ in range(8):
        est._update_anchor((11.0, 5.0, 0.0))
    assert est._anchor is not None
    assert est._anchor[0] == pytest.approx(11.0, abs=1e-6)


def test_reject_cluster_unlock_adsorbs_ema(est):
    """连续被拒且彼此一致的观测（锚点本身偏了）：解锁吸附到簇 EMA。"""
    _seed_default_anchor(est)
    for _ in range(4):
        est._update_anchor((0.7, 0.05, 0.0))  # dp≈0.70 > 0.60
        assert est._reject_count >= 1
        assert est._anchor[0] == pytest.approx(0.0)  # 未解锁前不动
    est._update_anchor((0.7, 0.05, 0.0))  # 第 5 帧 -> 解锁
    assert est._anchor[0] == pytest.approx(0.7, abs=1e-6)
    assert est._anchor[1] == pytest.approx(0.05, abs=1e-6)
    assert est._reject_count == 0
    assert est._reject_ema is None


def test_reloc_yaw_guard_rejects_grazing_cluster(est):
    """重定位守卫（2026-10-07）：拒绝簇 EMA 的 yaw 与锚点差超
    reloc_max_yaw_jump_deg(10°) -> 拒绝重定位，保持原锚点、清零计数
    （掠射角观测伪影：远距观测 yaw 系统性偏 20~33°，浮动坞真实朝向
    变化缓慢，瞬间大 yaw 跳变必是伪影），位置再偏也不重定位。"""
    _seed_default_anchor(est)  # 锚点 (0, 0, 0)
    for _ in range(10):  # 位置/艏向双超差的成簇观测
        est._update_anchor((0.7, 0.05, math.radians(25.0)))
    assert est._anchor[0] == pytest.approx(0.0, abs=1e-9)
    assert est._anchor[1] == pytest.approx(0.0, abs=1e-9)
    assert est._anchor[2] == pytest.approx(0.0, abs=1e-9)
    # 守卫触发即清零计数（第 5/10 帧各触发一次，之后重新累计）
    assert est._reject_count == 0
    assert est._reject_ema is None
    # 但簇首次出现时刻保留（持续窗口还在累积，供逃逸判定）
    assert est._reject_cluster_start is not None


def test_reloc_yaw_guard_escape_releases_persistent_cluster(est):
    """守卫持续窗口逃逸（2026-10-07 评审）：伪影是一过性的，拒绝簇持续超
    reloc_guard_escape_sec(3s) 判定真实移位（换泊位/重启），守卫放行重定位，
    防锚点永久锁死在旧处（reset_anchor_on_start=false 时无人救）。"""
    from rclpy.duration import Duration

    _seed_default_anchor(est)
    est._update_anchor((0.7, 0.05, math.radians(25.0)))  # 簇首帧
    assert est._reject_cluster_start is not None
    # 回拨簇首次出现时刻，模拟簇已持续 3.5s（> 3s 逃逸窗口）
    est._reject_cluster_start = est.get_clock().now() - Duration(seconds=3.5)
    for _ in range(4):  # 集满 5 帧触发解锁判定 -> 逃逸放行
        est._update_anchor((0.7, 0.05, math.radians(25.0)))
    assert est._anchor[0] == pytest.approx(0.7, abs=1e-6)
    assert est._anchor[1] == pytest.approx(0.05, abs=1e-6)
    assert est._anchor[2] == pytest.approx(math.radians(25.0), abs=1e-6)
    assert est._reject_count == 0
    assert est._reject_ema is None
    assert est._reject_cluster_start is None


def test_reloc_yaw_guard_allows_small_yaw_cluster(est):
    """簇 yaw 偏差在守卫门限内（5° < 10°）：正常重定位到簇 EMA（真实移位）。"""
    _seed_default_anchor(est)
    for _ in range(5):
        est._update_anchor((0.7, 0.05, math.radians(5.0)))  # dp>0.6，dyaw=5°
    assert est._anchor[0] == pytest.approx(0.7, abs=1e-6)
    assert est._anchor[1] == pytest.approx(0.05, abs=1e-6)
    assert est._anchor[2] == pytest.approx(math.radians(5.0), abs=1e-6)
    assert est._reject_count == 0
    assert est._reject_ema is None


def test_reloc_blocked_by_distance(est):
    """可信度门-距离（2026-10-07 现场）：远场（dist > reloc_max_dist_m=6m）
    的坞位姿不可信，拒绝改写锚点（实测远场重定位 dp 0.77~1.21m）。"""
    _seed_default_anchor(est)
    for _ in range(5):
        est._update_anchor((0.7, 0.05, math.radians(5.0)), dist=8.0)
    assert est._anchor[0] == pytest.approx(0.0, abs=1e-6), "远场不得重定位"
    assert est._reject_count == 0
    assert est._reject_ema is None


def test_reloc_blocked_when_single_tag(est, monkeypatch):
    """可信度门-双码：明确只看到一个码时拒绝重定位（单双码系统差可达 ~1m）。"""
    _seed_default_anchor(est)
    monkeypatch.setattr(est, "_visible_tag_count", lambda now: 1)
    for _ in range(5):
        est._update_anchor((0.7, 0.05, math.radians(5.0)), dist=3.0)
    assert est._anchor[0] == pytest.approx(0.0, abs=1e-6), "单码不得重定位"


def test_reloc_allowed_when_dual_tag_near(est, monkeypatch):
    """双码可见 + 近距离 ⇒ 允许重定位（真实移位场景不受影响）。"""
    _seed_default_anchor(est)
    monkeypatch.setattr(est, "_visible_tag_count", lambda now: 2)
    for _ in range(5):
        est._update_anchor((0.7, 0.05, math.radians(5.0)), dist=3.0)
    assert est._anchor[0] == pytest.approx(0.7, abs=1e-6)


def test_accepted_frame_resets_reject_cluster(est):
    """接受帧清零拒绝计数与簇 EMA（离群毛刺不累计解锁）。"""
    _seed_default_anchor(est)
    for _ in range(4):
        est._update_anchor((0.7, 0.0, 0.0))
    assert est._reject_count == 4
    est._update_anchor((0.1, 0.0, 0.0))  # 接受 -> 清零，锚点 EMA 微动
    assert est._reject_count == 0
    assert est._reject_ema is None
    assert est._anchor[0] == pytest.approx(0.025, abs=1e-6)
    # 再来 4 帧跳变：不解锁（计数重新开始）
    for _ in range(4):
        est._update_anchor((0.7, 0.0, 0.0))
    assert est._anchor[0] < 0.1
    est._update_anchor((0.7, 0.0, 0.0))  # 第 5 帧解锁
    assert est._anchor[0] == pytest.approx(0.7, abs=1e-6)


def test_reset_clears_seed_and_cluster(est):
    """锚点重置清空播种窗口与拒绝簇。"""
    for _ in range(3):
        est._update_anchor((10.0, 5.0, 0.0))
    assert len(est._seed_buf) == 3
    msg = Bool()
    msg.data = True
    est._reset_anchor_cb(msg)
    assert est._anchor is None
    assert est._seed_buf == []
    assert est._reject_ema is None
    assert est._reject_count == 0


def test_yaw_spread_gate(est):
    """yaw 离散度超门限不播种（防 ±π 附近振荡锚定）。"""
    for i in range(8):
        est._update_anchor((10.0, 5.0, math.pi - 0.5 + (1.0 if i % 2 else 0.0)))
    # 两簇 yaw 相差 1.0 rad（> 15° 门限）
    assert est._anchor is None


def test_lookup_missing_frame_does_not_block(est):
    """2026-09-11 回归：目标帧不在 TF 缓存里时必须立即返回，不得阻塞。

    阻塞式查询会占满 20Hz 定时器周期，单线程 executor 随即将 /tf 订阅回调
    饿死，自己的 tf buffer 落后实时流数秒，把新鲜 odom 误判成 INVALID
    （实船实测龄期恒 3.07s、锚点永建不起来）。
    """
    assert est._tf_timeout.nanoseconds == 0, "tf_timeout_sec 必须保持 0（非阻塞）"
    t0 = time.perf_counter()
    pose, age = est._lookup_2d("odom", "base_link", est.get_clock().now())
    cost = time.perf_counter() - t0
    assert pose is None
    assert age == float("inf")
    assert cost < 0.02, f"TF 查询阻塞了 {cost * 1e3:.0f}ms，会饿死 /tf 订阅回调"
