#!/usr/bin/env python3
"""docking_safety 单元测试（离线直调 _check，捕获 abort_request 发布）。

覆盖：ALIGN_ENTRY 独立推算窗口（不得抢跑 FSM align_tag_loss_grace_sec
使 REACQUIRE 路径走不到）、APPROACH 维持 3s 竞速语义、撤离通道位姿
INVALID 持续告警 EXIT_POSE_LOST。
"""

import math

import pytest
import rclpy
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String

from usv_docking.docking_safety import DockingSafety


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def safety(ros_context):
    node = DockingSafety()
    captured = []

    class _Pub:
        def publish(self, msg):
            captured.append(msg.data)

    node._abort_request_pub = _Pub()
    node._captured = captured
    yield node
    node.destroy_node()


def _feed(safety, state, source="VISION", age=0.0, x=-2.5, y=0.0, yaw=math.pi):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    safety._dock_pose_cb(msg)
    src = String()
    src.data = source
    safety._pose_source_cb(src)
    safety._measurement_age_cb(Float32(data=age))
    st = String()
    st.data = state
    safety._state_cb(st)


def test_align_prediction_not_aborted_before_align_window(safety):
    """ALIGN 推算 4s（> 普通窗口 3s，< ALIGN 窗口 7s；FSM 宽限 5s）：
    safety 不得抢跑 ODOM_TIMEOUT，ALIGN 的 REACQUIRE 重捕获路径须走得到。"""
    for _ in range(15):  # > violation_cycles=10，若按 3s 窗口早已触发
        _feed(safety, "ALIGN_ENTRY", source="ODOM_PREDICTION", age=4.0)
        safety._check()
    assert safety._captured[-1] == ""


def test_approach_prediction_timeout_unchanged(safety):
    """APPROACH 保持 3s 窗口 + 0.5s 防抖竞速语义：推算 4s 超窗 -> ODOM_TIMEOUT。"""
    for _ in range(10):
        _feed(safety, "APPROACH_ENTRY", source="ODOM_PREDICTION", age=4.0)
        safety._check()
    assert safety._captured[-1] == "ODOM_TIMEOUT"


def test_align_prediction_beyond_align_window_aborts(safety):
    """ALIGN 推算 8s > 7s 窗口：ODOM_TIMEOUT 兜底仍然生效。"""
    for _ in range(10):
        _feed(safety, "ALIGN_ENTRY", source="ODOM_PREDICTION", age=8.0)
        safety._check()
    assert safety._captured[-1] == "ODOM_TIMEOUT"


def test_exit_invalid_pose_warns_after_threshold(safety):
    """撤离通道位姿 INVALID：阈值内静默（FSM 超时兜底驶出），持续超
    exit_pose_invalid_warn_sec 发 EXIT_POSE_LOST 提示性告警（不抢 FAILED）。"""
    # 刚进入 ABORT_EXIT：阈值内不告警
    for _ in range(3):
        _feed(safety, "ABORT_EXIT", source="INVALID", age=float("inf"))
        safety._check()
    assert safety._captured[-1] == ""
    # 位姿丢失已持续 6s > 5s（回拨起始时刻）-> EXIT_POSE_LOST
    safety._exit_invalid_since = safety.get_clock().now() - Duration(seconds=6.0)
    _feed(safety, "ABORT_EXIT", source="INVALID", age=float("inf"))
    safety._check()
    assert safety._captured[-1] == "EXIT_POSE_LOST"
    # 位姿恢复推算 -> 告警消失、计时清零
    _feed(safety, "ABORT_EXIT", source="ODOM_PREDICTION", age=1.0)
    safety._check()
    assert safety._captured[-1] == ""
    assert safety._exit_invalid_since is None
