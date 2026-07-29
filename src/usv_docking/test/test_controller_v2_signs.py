#!/usr/bin/env python3
"""docking_motion_controller_v2 控制律符号/门控/安全单元测试。

只调内部计算函数与 _control_loop（捕获发布），不需要仿真。
"""

import math

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from usv_docking.docking_motion_controller_v2 import (
    DockingMotionControllerV2,
    MODE_BACK_IN,
    MODE_EXIT_FORWARD,
    MODE_HOLD,
)


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def ctrl(ros_context):
    node = DockingMotionControllerV2()
    yield node
    node.destroy_node()


def _feed_pose(ctrl, x, y, yaw):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    ctrl._pose_cb(msg)
    src = String()
    src.data = "VISION"
    ctrl._src_cb(src)


def test_back_in_centered_reverses_straight(ctrl):
    """坞外中轴线上对准：v<0（倒入），ω≈0。"""
    v, w = ctrl._compute_back_in(-2.0, 0.0, 0.0, 0.2, 0.18)
    assert v < 0.0
    assert abs(w) < 1e-9


def test_back_in_lateral_error_yaw_sign(ctrl):
    """e_y>0 时倒船 ω 必须为负（艏向左摆使船尾右移消横偏）。"""
    _, w = ctrl._compute_back_in(-2.0, 0.15, 0.0, 0.2, 0.18)
    assert w < 0.0
    _, w = ctrl._compute_back_in(-2.0, -0.15, 0.0, 0.2, 0.18)
    assert w > 0.0


def test_back_in_final_settle_kills_yaw(ctrl):
    """终局消艏偏：x/y 均达标但 e_yaw 超差时 v=0，ω 与 e_yaw 反号。"""
    v, w = ctrl._compute_back_in(0.05, 0.05, 0.05, 0.15, 0.35)
    assert v == 0.0
    assert w < 0.0
    v, w = ctrl._compute_back_in(0.05, 0.05, -0.05, 0.15, 0.35)
    assert v == 0.0
    assert w > 0.0
    # x 未达标（坞外 2m）：不进入消艏偏分支，仍倒车
    v, _ = ctrl._compute_back_in(-2.0, 0.05, 0.05, 0.15, 0.35)
    assert v < 0.0


def test_approach_bidirectional_by_side(ctrl):
    """APPROACH 双向就位：船在预备点外侧倒退（v<0），内侧前进倒出（v>0）。"""
    import math as m
    # 外侧（x=-5.0 < staging_x=-2.5）：倒退入位
    v, _ = ctrl._compute_approach(-5.0, 0.0, 0.0, m.pi)
    assert v < 0.0
    # 内侧（x=-2.0 > staging_x=-2.5）：前进倒出（不再要求 180° 调头）
    v, _ = ctrl._compute_approach(-2.0, 0.0, 0.0, m.pi)
    assert v > 0.0


def test_approach_forward_crab_capped(ctrl):
    """前进倒出蟹行角限幅：大 e_y 下艏向角速度有界（防横移过冲荡秋千）。"""
    import math as m
    crab = m.radians(8.0)
    _, w_small = ctrl._compute_approach(-2.0, 0.05, 0.0, m.pi)
    _, w_large = ctrl._compute_approach(-2.0, 0.50, 0.0, m.pi)
    # e_y=0.5 时 ky*e_y=0.2rad 超 8° 上限：w 应与限幅值相当而非 0.2rad 满输出
    assert abs(w_large) <= 1.0 * crab + 1e-6
    # 小 e_y 未触限幅，保持比例
    assert abs(w_small) < abs(w_large)


def test_approach_speed_tapers_near_axis(ctrl):
    """接近轴线锥形降速：|e_y| 越小 v 越低，下限 min_speed 蠕行不停。"""
    import math as m
    # 前进倒出分支（船在预备点内侧）
    v_full, _ = ctrl._compute_approach(-2.0, 0.60, 0.0, m.pi)   # |e_y|>=slow_y 全速
    v_mid, _ = ctrl._compute_approach(-2.0, 0.25, 0.0, m.pi)
    v_low, _ = ctrl._compute_approach(-2.0, 0.05, 0.0, m.pi)
    assert v_full > v_mid > v_low
    assert v_low >= 0.08 - 1e-6  # 下限蠕行
    # 倒退入位分支（船在外侧）
    rv_full, _ = ctrl._compute_approach(-5.0, 0.60, 0.0, m.pi)
    rv_low, _ = ctrl._compute_approach(-5.0, 0.05, 0.0, m.pi)
    assert abs(rv_full) > abs(rv_low)
    assert abs(rv_low) >= 0.08 - 1e-6
    assert rv_full < 0.0 and rv_low < 0.0


def test_back_in_gate2_stops_translation(ctrl):
    """超出门控2：v=0 只修艏向。"""
    v, w = ctrl._compute_back_in(-2.0, 0.40, 0.0, 0.2, 0.18)
    assert v == 0.0
    assert w != 0.0


def test_back_in_yaw_error_sign(ctrl):
    """e_yaw>0（艏向偏左）-> ω<0 修正。"""
    _, w = ctrl._compute_back_in(-2.0, 0.0, 0.05, 0.2, 0.18)
    assert w < 0.0


def test_exit_forward_signs(ctrl):
    """前进驶出：v>0；e_y>0 -> ω>0（艏向右摆带船身左移消横偏）。"""
    v, w = ctrl._compute_exit_forward(0.15, 0.0, 0.15)
    assert v > 0.0
    assert w > 0.0
    _, w = ctrl._compute_exit_forward(-0.15, 0.0, 0.15)
    assert w < 0.0


def test_exit_turn_threshold_stops_translation(ctrl):
    """驶出中 |e_yaw| 超阈值：先原地转正。"""
    v, w = ctrl._compute_exit_forward(0.0, math.radians(45.0), 0.15)
    assert v == 0.0
    assert w < 0.0  # e_yaw>0 -> 需要 ω<0 转回


def test_safety_stop_zeroes_immediately(ctrl):
    """safety_stop=true：输出立即清零（绕过斜坡）。"""
    ctrl._v_cmd = -0.3
    ctrl._w_cmd = 0.2
    ctrl._safety_cb(Bool(data=True))
    _feed_pose(ctrl, -2.0, 0.0, math.pi)
    mode = String()
    mode.data = MODE_BACK_IN
    ctrl._mode_cb(mode)

    published = {}

    class _Pub:
        def publish(self, twist):
            published["twist"] = twist

    ctrl._cmd_pub = _Pub()
    ctrl._control_loop()
    assert published["twist"].linear.x == 0.0
    assert published["twist"].angular.z == 0.0
    assert ctrl._v_cmd == 0.0
    assert ctrl._w_cmd == 0.0


def test_invalid_pose_holds_motion_modes(ctrl):
    """位姿 INVALID：运动态全部输出零。"""
    ctrl._safety_cb(Bool(data=False))
    _feed_pose(ctrl, -2.0, 0.0, math.pi)
    src = String()
    src.data = "INVALID"
    ctrl._src_cb(src)
    mode = String()
    mode.data = MODE_BACK_IN
    ctrl._mode_cb(mode)

    published = {}

    class _Pub:
        def publish(self, twist):
            published["twist"] = twist

    ctrl._cmd_pub = _Pub()
    ctrl._control_loop()
    assert published["twist"].linear.x == 0.0
    assert published["twist"].angular.z == 0.0


def test_approach_is_stern_first(ctrl):
    """APPROACH 全程倒船：目标在船尾方向时 v<0，ω≈0。"""
    _feed_pose(ctrl, -5.5, 0.0, math.pi)  # 船尾朝坞
    v, w = ctrl._compute_approach(-5.5, 0.0, 0.0, math.pi)
    assert v < 0.0
    assert abs(w) < 1e-6


def test_hold_mode_zero(ctrl):
    published = {}

    class _Pub:
        def publish(self, twist):
            published["twist"] = twist

    ctrl._cmd_pub = _Pub()
    mode = String()
    mode.data = MODE_HOLD
    ctrl._mode_cb(mode)
    ctrl._control_loop()
    assert published["twist"].linear.x == 0.0
    assert published["twist"].angular.z == 0.0
