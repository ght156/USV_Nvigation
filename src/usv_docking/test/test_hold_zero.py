#!/usr/bin/env python3
"""HOLD/DOCKED_HOLD 进入沿当帧零速回归测试（2026-10-13 三审 P0）。

切到保持态前 _v_cmd/_w_cmd 可能带着上一模式的残余指令（如 BACK_IN 的
-0.3 m/s 倒船指令、+0.1 rad/s 转艏指令）；进入保持态当帧必须直接清零
发布，不得经加速度斜坡慢收敛（斜坡 ~1.5s 内仍在主动推进）。
"""

import pytest
import rclpy

from usv_docking.docking_motion_controller import (
    DockingMotionController,
    MODE_BACK_IN,
    MODE_DOCKED_HOLD,
    MODE_HOLD,
    MODE_SEARCH,
)


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def ctrl(ros_context):
    node = DockingMotionController()
    published = []

    class _Pub:
        def publish(self, msg):
            published.append((msg.linear.x, msg.angular.z))

    node._cmd_pub = _Pub()
    node._published = published
    yield node
    node.destroy_node()


@pytest.mark.parametrize("hold_mode", [MODE_HOLD, MODE_DOCKED_HOLD])
def test_hold_entry_zeroes_residual_command(ctrl, hold_mode):
    """倒船残余指令（v=-0.3, w=+0.1）切保持态：第一帧必须是严格零速，
    不是斜坡收敛后的 -0.285 / +0.075。"""
    ctrl._v_cmd = -0.3
    ctrl._w_cmd = 0.1
    ctrl._mode = hold_mode
    ctrl._last_mode = MODE_BACK_IN  # 触发模式切换沿

    ctrl._control_loop()

    assert ctrl._published, "进入保持态当帧必须发布零速帧"
    v, w = ctrl._published[0]
    assert v == 0.0
    assert w == 0.0


def test_hold_zero_burst_then_silent(ctrl):
    """进入 HOLD 后连续发 hold_zero_publish_sec（默认 1s）零速帧，随后
    静默让出 /cmd_vel_nav；全程不得出现非零指令。"""
    ctrl._v_cmd = -0.3
    ctrl._w_cmd = 0.1
    ctrl._mode = MODE_HOLD
    ctrl._last_mode = MODE_BACK_IN

    for _ in range(100):
        ctrl._control_loop()

    msgs = ctrl._published
    assert msgs, "进入保持态必须先发一段零速帧"
    assert all(v == 0.0 and w == 0.0 for v, w in msgs)
    # 默认 1s / 20Hz ≈ 20 帧，远小于 100——之后必须静默让权
    assert len(msgs) <= 25


def test_non_hold_modes_keep_accel_ramp(ctrl):
    """非保持态不受当帧清零影响：SEARCH 开环旋转仍经斜坡从零加速，
    验证 P0 修复只作用于 HOLD 进入沿，不改其他模式的斜坡行为。"""
    ctrl._v_cmd = 0.0
    ctrl._w_cmd = 0.0
    ctrl._mode = MODE_SEARCH
    ctrl._last_mode = MODE_SEARCH  # 无切换沿
    # SEARCH 属开环找 Tag，不卡位姿门；但 mode/safety 断流会触发 must-stop，
    # 这里伪造新鲜输入绕过看门狗，专验斜坡
    now = ctrl.get_clock().now()
    ctrl._mode_msg_time = now
    ctrl._safety_msg_time = now

    ctrl._control_loop()

    assert len(ctrl._published) == 1
    _, w = ctrl._published[0]
    search_rate = float(ctrl.get_parameter("search_angular_speed").value)
    # 第一帧是斜坡起步：0 < w < search_rate（绝非当帧到目标，也非零）
    assert 0.0 < w < search_rate
