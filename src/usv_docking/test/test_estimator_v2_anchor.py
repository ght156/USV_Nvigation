#!/usr/bin/env python3
"""docking_pose_estimator_v2 锚点逻辑单元测试（离线直调 _update_anchor）。

覆盖：首锚共识播种（中位数 + 离散度滑窗）、跳变拒绝簇吸附解锁、
接受帧清零拒绝簇、重置清空缓冲。
"""

import math

import pytest
import rclpy
from std_msgs.msg import Bool

from usv_docking.docking_pose_estimator_v2 import DockingPoseEstimatorV2


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def est(ros_context):
    node = DockingPoseEstimatorV2()
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
