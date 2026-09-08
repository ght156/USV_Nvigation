#!/usr/bin/env python3
"""docking_motion_controller 控制律符号/门控/安全单元测试。

只调内部计算函数与 _control_loop（捕获发布），不需要仿真。
"""

import math

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from usv_docking.docking_motion_controller import (
    DockingMotionController,
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
    node = DockingMotionController()
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
    """锥形降速：|e_y| 或距预备点越小 v 越低，下限 min_speed 蠕行不停。"""
    import math as m
    # 外侧倒入（船在预备点外）：远场全速，y 进锥形区降速，接近 0 蠕行
    v_far, _ = ctrl._compute_approach(-6.0, 1.5, 0.0, m.pi)
    v_mid, _ = ctrl._compute_approach(-6.0, 0.6, 0.0, m.pi)
    v_low, _ = ctrl._compute_approach(-6.0, 0.05, 0.0, m.pi)
    assert abs(v_far - (-0.45)) < 1e-6  # 全速 = approach_speed 默认值（与 yaml 一致）
    assert abs(v_mid) < abs(v_far)
    assert abs(v_low) >= 0.08 - 1e-6
    assert v_far < 0.0 and v_mid < 0.0 and v_low < 0.0
    # 前进倒出分支：距预备点越近（dist<slow_dist）速度越低
    v_in_near, _ = ctrl._compute_approach(-1.5, 0.7, 0.0, m.pi)  # dist≈1.22m
    v_in_far, _ = ctrl._compute_approach(-1.1, 0.7, 0.0, m.pi)   # dist≈1.57m
    assert v_in_near < v_in_far


def test_approach_decelerates_before_half_boat_length(ctrl):
    """减速提前（08-10 实测）：|e_y|<1.0 即降速，而非旧 0.5m 阈值。"""
    import math as m
    # 远场 y=1.5（>=slow_y）且距离>slow_dist：全速
    v_far, _ = ctrl._compute_approach(-6.0, 1.5, 0.0, m.pi)
    # 旧阈值 0.5m 处（y=0.6）现在必须已经降速
    v_old_threshold, _ = ctrl._compute_approach(-6.0, 0.6, 0.0, m.pi)
    # y=0.9 已进入 slow_y=1.0 锥形区：速度低于全速
    v_y, _ = ctrl._compute_approach(-6.0, 0.9, 0.0, m.pi)
    assert abs(v_far - (-0.45)) < 1e-6  # 全速 = approach_speed 默认值（与 yaml 一致）
    assert abs(v_old_threshold) < abs(v_far)
    assert abs(v_y) < abs(v_far)


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
    """进入 HOLD 后在停稳窗口内仍发布零速（2026-09-01 起停稳+发够即静默，
    见 test_hold_goes_silent_after_settle）；初始 HOLD 无切换沿不发布。"""
    published = {}

    class _Pub:
        def publish(self, twist):
            published["twist"] = twist

    ctrl._cmd_pub = _Pub()
    mode = String()
    # 先给一个非保持态制造模式切换沿
    mode.data = MODE_EXIT_FORWARD
    ctrl._mode_cb(mode)
    ctrl._control_loop()
    mode.data = MODE_HOLD
    ctrl._mode_cb(mode)
    ctrl._control_loop()
    assert published["twist"].linear.x == 0.0
    assert published["twist"].angular.z == 0.0


# ══════════════ ALIGN 弧线化（2026-08-22）══════════════


def test_align_arc_reverses_with_small_speed(ctrl):
    """ALIGN 弧线化：x 在门限外时带小倒速（v<0）消艏向，ω 符号不变。"""
    v, w = ctrl._compute_align(-2.5, 0.1)
    assert v < 0.0
    assert abs(v) <= 0.08 + 1e-9  # 默认 align_arc_speed=0.08
    assert w < 0.0  # e_yaw>0 -> ω<0 修正
    _, w = ctrl._compute_align(-2.5, -0.1)
    assert w > 0.0


def test_align_arc_x_limit_guard(ctrl):
    """x 超过门限（近坞）：退回 v=0 纯旋转，防 ALIGN 卡死时倒船滑入坞。"""
    v, w = ctrl._compute_align(-1.0, 0.1)  # > align_arc_x_limit=-1.5
    assert v == 0.0
    assert w < 0.0


def test_align_arc_disabled_when_speed_zero(ctrl):
    """align_arc_speed=0：完全回退旧版 v=0 纯原地转行为。"""
    from rclpy.parameter import Parameter

    ctrl.set_parameters(
        [Parameter("align_arc_speed", value=0.0)]
    )
    v, w = ctrl._compute_align(-2.5, 0.1)
    assert v == 0.0
    assert w < 0.0


# ══════════════ 终局横向修正蠕行（x 达位 y 未收敛不清零 v）══════════════


def test_back_in_creeps_for_lateral_when_x_done(ctrl):
    """x 已达位但 |e_y| 超 docked_y_tol：保持小倒速让横向律有纵向行程
    （倒船横向律 ẏ≈|v|·e_yaw，v=0 则 y 数学上收敛不到位，干等 final_dock
    超时）；纵向行程以 docked 窗口远缘为界，到界 v->0。"""
    # x=0.05（|dist|=0.05 < docked_x_tol=0.15），e_y=0.12 ∈ (0.10, 0.35]：
    # v<0 蠕行，横向律符号不变（e_y>0 -> ω<0）
    v, w = ctrl._compute_back_in(0.05, 0.12, 0.0, 0.15, 0.18)
    assert v < 0.0
    assert abs(v) <= 0.15 + 1e-9
    assert w < 0.0
    # 到达窗口远缘（x = final_target_x + docked_x_tol = 0.15）：剩余行程 0
    v, _ = ctrl._compute_back_in(0.15, 0.12, 0.0, 0.15, 0.18)
    assert v == 0.0
    # x/y 均达标：仍是终局消艏偏（v=0 原地转），不受蠕行分支影响
    v, w = ctrl._compute_back_in(0.05, 0.05, 0.05, 0.15, 0.18)
    assert v == 0.0
    assert w < 0.0


# ══════════════ 保持态静默（停稳后让出 cmd_vel，2026-09-01）══════════════


class _CapturePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append((msg.linear.x, msg.angular.z))


def test_hold_goes_silent_after_settle(ctrl):
    """HOLD/DOCKED_HOLD 停稳且零速帧发够 hold_zero_publish_sec 后静默，
    不再占用 /cmd_vel_nav（让位 teleop；实船桥有看门狗兜底清零）。"""
    cap = _CapturePub()
    ctrl._cmd_pub = cap
    # 初始即 HOLD（无切换沿，余量 0，v=w=0）：直接静默
    ctrl._control_loop()
    assert not cap.sent
    # 切入 DOCKED_HOLD：发一小段零速后静默
    ctrl._mode_cb(String(data="DOCKED_HOLD"))
    for _ in range(30):  # 1.0s 余量 + 富余
        ctrl._control_loop()
    n_after_settle = len(cap.sent)
    assert 0 < n_after_settle <= 21
    for _ in range(10):
        ctrl._control_loop()
    assert len(cap.sent) == n_after_settle
    assert all(v == 0.0 and w == 0.0 for v, w in cap.sent)


def test_motion_mode_keeps_publishing_when_pose_invalid(ctrl):
    """运动态位姿不可用停车时仍常发零速，不受保持态静默逻辑影响。"""
    cap = _CapturePub()
    ctrl._cmd_pub = cap
    ctrl._mode_cb(String(data="APPROACH"))
    for _ in range(30):
        ctrl._control_loop()
    assert len(cap.sent) == 30


def test_hold_safety_stop_republishes_zero(ctrl):
    """保持态静默期间收到 safety_stop：补发零速帧。"""
    cap = _CapturePub()
    ctrl._cmd_pub = cap
    ctrl._control_loop()  # 初始 HOLD，静默
    assert not cap.sent
    ctrl._safety_cb(Bool(data=True))
    ctrl._control_loop()
    assert len(cap.sent) == 1
    assert cap.sent[0] == (0.0, 0.0)
