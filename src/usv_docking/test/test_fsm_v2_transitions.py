#!/usr/bin/env python3
"""docking_fsm_v2 状态转移单元测试（rclpy 离线驱动，不需要仿真）。

驱动方式：直接调输入回调注入合成消息，再手动调 _transitions(dt)。
"""

import json
import math

import pytest
import rclpy
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, String

from usv_docking.docking_fsm_v2 import (
    DockState,
    DockingFsmV2,
    MODE_BACK_IN,
    MODE_SEARCH,
)

SRC_VISION = "VISION"
SRC_INVALID = "INVALID"


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def fsm(ros_context):
    node = DockingFsmV2()
    yield node
    node.destroy_node()


def _pose(x, y, yaw):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def _feed(fsm, x=-5.0, y=0.0, yaw=math.pi, source=SRC_VISION, visible=True, age=0.0):
    fsm._dock_pose_cb(_pose(x, y, yaw))
    fsm._tag_visible_cb(Bool(data=visible))
    src = String()
    src.data = source
    fsm._pose_source_cb(src)
    fsm._measurement_age_cb(Float32(data=age))


def _start(fsm):
    fsm._start_cb(Bool(data=True))


def _backdate(fsm, seconds):
    fsm._state_enter_time = fsm.get_clock().now() - Duration(seconds=seconds)


def test_start_acquire_approach(fsm):
    _start(fsm)
    fsm._transitions(0.1)
    assert fsm._state == DockState.ACQUIRE_TAG
    # 连续 5 帧 VISION -> APPROACH_ENTRY
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_acquire_timeout_reports_reapproach(fsm):
    _start(fsm)
    fsm._transitions(0.1)
    _feed(fsm, source=SRC_INVALID, visible=False)
    _backdate(fsm, 61.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._needs_reapproach is True


def test_acquire_miss_tolerance(fsm):
    """集帧闪烁容忍：<=3 帧丢失不清零，补满 5 帧 VISION 即转移。"""
    _start(fsm)
    fsm._transitions(0.1)
    for _ in range(3):
        _feed(fsm)
        fsm._transitions(0.1)
    # 3 帧丢失（<= 容忍）：计数保留
    for _ in range(3):
        _feed(fsm, source=SRC_INVALID, visible=False)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ACQUIRE_TAG
    # 再补 2 帧 -> 满 5 帧 -> APPROACH_ENTRY
    for _ in range(2):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_acquire_miss_over_tolerance_resets(fsm):
    """连续丢失 >3 帧：集帧计数清零重来。"""
    _start(fsm)
    fsm._transitions(0.1)
    for _ in range(3):
        _feed(fsm)
        fsm._transitions(0.1)
    for _ in range(4):
        _feed(fsm, source=SRC_INVALID, visible=False)
        fsm._transitions(0.1)
    assert fsm._acquire_frames == 0
    # 重新集帧：4 帧不够，仍在 ACQUIRE；第 5 帧转移
    for _ in range(4):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ACQUIRE_TAG
    _feed(fsm)
    fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_approach_to_align_gate(fsm):
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 到达预备点容差内 -> ALIGN_ENTRY
    _feed(fsm, x=-2.5, y=0.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 满足对准门槛持续 1s -> BACK_IN
    for _ in range(11):
        _feed(fsm, x=-2.5, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.BACK_IN
    assert fsm._compute_mode() == MODE_BACK_IN


def test_align_y_abort_returns_to_approach(fsm):
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, x=-2.5)
    fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    _feed(fsm, x=-2.5, y=0.5)  # |e_y|>0.35
    fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_align_y_stuck_escapes_to_approach(fsm):
    """y 卡死带逃逸：艏向已准但 |y| 在 (y_tol, y_abort] 滞留超时回 APPROACH。"""
    from rclpy.parameter import Parameter

    # 与生产 yaml 一致：approach_y_tol=0.15（代码默认 0.5 会让逃逸后
    # APPROACH 立即满足 staging 条件弹回 ALIGN，测试无法观测）
    fsm.set_parameters(
        [Parameter("approach_y_tol", Parameter.Type.DOUBLE, 0.15)]
    )
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, x=-2.5, y=0.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 艏向已准（yaw=π -> e_yaw=0）但 |y|=0.25 落在卡死带 (0.20, 0.35]
    for _ in range(50):  # 5.0s < 6.0s：不逃逸
        _feed(fsm, x=-2.5, y=0.25, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    for _ in range(15):  # 累计 6.5s > 6.0s：逃逸
        _feed(fsm, x=-2.5, y=0.25, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_align_tag_loss_grace(fsm):
    """ALIGN 丢 Tag 宽限：短暂推算不弹回，超宽限才进 REACQUIRE。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, x=-2.5)
    fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 宽限内（1.0s < 5s）的推算闪烁：保持 ALIGN
    for _ in range(10):
        _feed(fsm, x=-2.5, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 视觉恢复：计时清零，ALIGN 继续正常推进（gate 满足直接进 BACK_IN）
    for _ in range(11):
        _feed(fsm, x=-2.5, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.BACK_IN
    # 回到 ALIGN 场景：超过宽限（6s）-> REACQUIRE
    fsm._enter(DockState.ALIGN_ENTRY)
    for _ in range(60):
        _feed(fsm, x=-2.5, source="ODOM_PREDICTION", visible=False, age=6.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG


def test_back_in_corridor_violation_aborts(fsm):
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    fsm._enter(DockState.BACK_IN)
    # 连续 violation_cycles 周期走廊违规 -> ABORT_EXIT
    for _ in range(10):
        _feed(fsm, x=-1.5, y=0.5)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ABORT_EXIT
    # 驶出到 exit_complete_x -> IDLE + needs_reapproach
    _feed(fsm, x=-4.5, y=0.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._needs_reapproach is True


def test_approach_invalid_routes_reacquire_and_back(fsm):
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    assert fsm._compute_mode() == MODE_SEARCH
    # 重捕获 5 帧且位于入口窗口 -> ALIGN_ENTRY
    for _ in range(5):
        _feed(fsm, x=-2.0, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY


def test_failed_maps_dock_abort_in_status(fsm):
    _start(fsm)
    fsm._transitions(0.1)
    req = String()
    req.data = "TOPIC_TIMEOUT:POSE"
    fsm._abort_request_cb(req)
    assert fsm._state == DockState.FAILED

    captured = {}

    class _Pub:
        def publish(self, msg):
            captured["json"] = msg.data

    fsm._status_pub = _Pub()
    fsm._publish("HOLD")
    status = json.loads(captured["json"])
    assert status["state"] == "DOCK_ABORT"
    assert status["v2_state"] == "FAILED"
    assert status["needs_reapproach"] is True
    assert status["needs_manual_takeover"] is True
    assert status["tag_age_sec"] is None  # inf -> null（严格 JSON）


def test_undock_timeout_fails_instead_of_fake_success(fsm):
    fsm._undock_cb(Bool(data=True))
    fsm._transitions(0.1)
    assert fsm._state == DockState.UNDOCK_EXIT
    # 位姿 INVALID 且超时 -> FAILED（不报 undock_success）
    _feed(fsm, source=SRC_INVALID, visible=False)
    _backdate(fsm, 61.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.FAILED
    assert fsm._undock_success is False


def test_undock_position_completion(fsm):
    fsm._undock_cb(Bool(data=True))
    fsm._transitions(0.1)
    _feed(fsm, x=-4.5)
    fsm._transitions(0.1)
    assert fsm._state == DockState.UNDOCK_SETTLE
    assert fsm._undock_success is True
    _backdate(fsm, 2.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
