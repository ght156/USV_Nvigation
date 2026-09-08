#!/usr/bin/env python3
"""docking_fsm 状态转移单元测试（rclpy 离线驱动，不需要仿真）。

驱动方式：直接调输入回调注入合成消息，再手动调 _transitions(dt)。
"""

import json
import math

import pytest
import rclpy
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, String

from usv_docking.docking_fsm import (
    DockState,
    DockingFsm,
    MODE_BACK_IN,
    MODE_HOLD,
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
    node = DockingFsm()
    yield node
    node.destroy_node()


def _pose(x, y, yaw):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def _feed(
    fsm,
    x=-5.0,
    y=0.0,
    yaw=math.pi,
    source=SRC_VISION,
    visible=True,
    age=0.0,
    speed=0.0,
):
    fsm._dock_pose_cb(_pose(x, y, yaw))
    fsm._tag_visible_cb(Bool(data=visible))
    src = String()
    src.data = source
    fsm._pose_source_cb(src)
    fsm._measurement_age_cb(Float32(data=age))
    odom = Odometry()
    odom.twist.twist.linear.x = speed
    fsm._odom_cb(odom)


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
    # 到达预备点容差内且艏向/速度满足门槛，持续 0.5s -> ALIGN_ENTRY
    _feed(fsm, x=-2.5, y=0.0)
    for _ in range(6):
        _feed(fsm, x=-2.5, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 满足对准门槛持续 1s -> BACK_IN
    for _ in range(11):
        _feed(fsm, x=-2.5, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.BACK_IN
    assert fsm._compute_mode() == MODE_BACK_IN


def test_approach_to_align_requires_yaw(fsm):
    """大艏偏（实测 55°）不得放行 ALIGN：基线只查 x/y 会原地旋转甩 y 冲线。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # x/y 达标但 e_yaw=40°：保持 APPROACH
    for _ in range(10):
        _feed(fsm, x=-2.5, y=0.0, yaw=math.pi + math.radians(40.0))
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 艏向收进门槛后持续 0.5s -> ALIGN_ENTRY
    for _ in range(6):
        _feed(fsm, x=-2.5, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY


def test_approach_to_align_requires_stop(fsm):
    """进 ALIGN 前实际速度须接近 0：实测切换时仍有 0.27m/s，旋转+滑行放大漂移。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # x/y/艏向达标但速度 0.3：保持 APPROACH
    for _ in range(10):
        _feed(fsm, x=-2.5, y=0.0, yaw=math.pi, speed=0.3)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 速度降到 0 后持续 0.5s -> ALIGN_ENTRY
    for _ in range(6):
        _feed(fsm, x=-2.5, y=0.0, yaw=math.pi, speed=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY


def test_approach_tag_loss_grace_keeps_approach(fsm):
    """APPROACH 丢 Tag 宽限：闪烁（推算期）宽限内不弹 REACQUIRE，超时才弹。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 推算期 2.9s（<3s 宽限）：保持 APPROACH
    for _ in range(29):
        _feed(fsm, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 超过宽限 -> REACQUIRE_TAG
    for _ in range(3):
        _feed(fsm, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    # 视觉恢复清零宽限计时
    fsm._enter(DockState.APPROACH_ENTRY)
    for _ in range(5):
        _feed(fsm, x=-5.0, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    for _ in range(29):
        _feed(fsm, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_align_y_abort_returns_to_approach(fsm):
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    for _ in range(6):
        _feed(fsm, x=-2.5, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    _feed(fsm, x=-2.5, y=0.5)  # |e_y|>0.35
    fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_align_y_stuck_escapes_to_approach(fsm):
    """y 卡死带逃逸：艏向已准但 |y| 在 (y_tol, y_abort] 滞留超时回 APPROACH。"""
    from rclpy.parameter import Parameter

    # 显式对齐生产 yaml（approach_y_tol=0.15），防默认值再漂移影响本用例：
    # 若 approach_y_tol > align_y_tol，逃逸后 APPROACH 立即满足 staging
    # 条件弹回 ALIGN，卡死带无法观测
    fsm.set_parameters(
        [Parameter("approach_y_tol", Parameter.Type.DOUBLE, 0.15)]
    )
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    for _ in range(6):
        _feed(fsm, x=-2.5, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 艏向已准（yaw=π -> e_yaw=0）但 |y|=0.25 落在卡死带 (0.15, 0.35]
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
    for _ in range(6):
        _feed(fsm, x=-2.5, y=0.0)
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


def test_reacquire_holds_when_prediction_valid(fsm):
    """REACQUIRE 位姿有效（odom 推算）时 HOLD 不自转，避免 SEARCH 扫漂 y。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    # 位姿有效 -> HOLD
    _feed(fsm, source="ODOM_PREDICTION", visible=False, age=1.0)
    fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_HOLD
    # 位姿无效 -> SEARCH
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_SEARCH


def test_reacquire_routes_approach_when_yaw_or_speed_large(fsm):
    """重捕获路由与 APPROACH->ALIGN 同一门槛：
    08-10 实测 REACQUIRE 以 e_yaw≈-58° 直接进 ALIGN 复现冲线，必须拦下。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    # 位置/横向达标但 e_yaw=45°：路由 APPROACH
    for _ in range(5):
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi + math.radians(45.0))
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 艏向达标但速度 0.3：仍路由 APPROACH
    fsm._enter(DockState.REACQUIRE_TAG)
    for _ in range(5):
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi, speed=0.3)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 全部达标 -> ALIGN_ENTRY
    fsm._enter(DockState.REACQUIRE_TAG)
    for _ in range(5):
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi, speed=0.0)
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


def test_reacquire_return_resets_approach_grace(fsm):
    """_enter 须复位 _approach_tag_loss/_staging_hold：REACQUIRE 集帧路由回
    APPROACH 后若残留旧推算计时，遇短暂新闪烁会立即复弹 REACQUIRE
    （08-10 实测活锁的弱化版）。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 推算 2.9s（宽限 3.0s 未满）：保持 APPROACH
    for _ in range(29):
        _feed(fsm, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    assert fsm._approach_tag_loss == pytest.approx(2.9, abs=1e-6)
    # INVALID 尖峰立即弹 REACQUIRE
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    # 集帧 5 帧 VISION（x=-5.0 在入口窗口外 -> 路由回 APPROACH）
    for _ in range(5):
        _feed(fsm, x=-5.0, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 计时已复位：2.9s 新闪烁不得复弹（修复前残留 2.9s，0.2s 即复弹）
    assert fsm._approach_tag_loss == 0.0
    assert fsm._staging_hold == 0.0
    for _ in range(29):
        _feed(fsm, x=-5.0, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_abort_exit_timeout_invalid_pose_not_fake_reapproach(fsm):
    """ABORT_EXIT 超时且位姿 INVALID：控制器只能停车，不得谎报"已驶出"
    （needs_reapproach=True 会让 dock_mission 在船仍卡坞内时立刻重试）。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    fsm._enter(DockState.BACK_IN)
    # 连续走廊违规 -> ABORT_EXIT
    for _ in range(10):
        _feed(fsm, x=-1.5, y=0.5)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ABORT_EXIT
    # 位姿 INVALID + 超时 -> IDLE，上报"驶出未确认"
    _feed(fsm, source=SRC_INVALID, visible=False)
    _backdate(fsm, 46.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._needs_reapproach is False
    assert fsm._abort_reason == "ABORT_EXIT_UNCONFIRMED"
    assert fsm._needs_manual_takeover is True
    # /dock/status 契约面：needs_reapproach 保持 false（不触发上层盲重试）
    captured = {}

    class _Pub:
        def publish(self, msg):
            captured["json"] = msg.data

    fsm._status_pub = _Pub()
    fsm._publish("HOLD")
    status = json.loads(captured["json"])
    assert status["state"] == "IDLE"
    assert status["needs_reapproach"] is False
    assert status["abort_reason"] == "ABORT_EXIT_UNCONFIRMED"


def test_abort_exit_timeout_valid_pose_still_reapproach(fsm):
    """ABORT_EXIT 超时但位姿有效（位置已知）：维持原 needs_reapproach=True。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    fsm._enter(DockState.ABORT_EXIT)
    _feed(fsm, x=-2.0, source="ODOM_PREDICTION", visible=False, age=1.0)
    _backdate(fsm, 46.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._needs_reapproach is True
    assert fsm._abort_reason is None


def test_exit_pose_lost_advisory_does_not_change_state(fsm):
    """safety 的 EXIT_POSE_LOST 是提示性告警：撤离/出泊中不改变状态
    （驶出走各自超时兜底，不提前 FAILED）。"""
    fsm._undock_cb(Bool(data=True))
    fsm._transitions(0.1)
    assert fsm._state == DockState.UNDOCK_EXIT
    req = String()
    req.data = "EXIT_POSE_LOST"
    fsm._abort_request_cb(req)
    assert fsm._state == DockState.UNDOCK_EXIT
    fsm._enter(DockState.ABORT_EXIT)
    fsm._abort_request_cb(req)
    assert fsm._state == DockState.ABORT_EXIT


def test_back_in_final_dock_requires_y_gate(fsm):
    """进 FINAL_DOCK 增加 |e_y| 门控（final_dock_entry_y_tol=0.15）：
    y 超门控留在 BACK_IN 继续修正，避免终局段修不动 y 干等 60s 超时。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    fsm._enter(DockState.BACK_IN)
    # x 进过渡距离（|x|=0.5 < 0.8）但 |e_y|=0.25 > 0.15：保持 BACK_IN
    _feed(fsm, x=-0.5, y=0.25)
    fsm._transitions(0.1)
    assert fsm._state == DockState.BACK_IN
    # y 收敛到门控内 -> FINAL_DOCK
    _feed(fsm, x=-0.5, y=0.10)
    fsm._transitions(0.1)
    assert fsm._state == DockState.FINAL_DOCK


# ══════════════ 二次归港：success 残留与 FAILED 重启（2026-09-01）══════════════


def test_success_cleared_on_idle_entry(fsm):
    """DOCKED 置 success 后回 IDLE 必须清掉：残留 success=true 会让二次归港
    进 MONITOR_DOCK 时读到在途旧帧"假成功"，dock_mission 随即 /dock/cancel
    把刚启动的新任务掐掉。"""
    _start(fsm)
    fsm._transitions(0.1)
    fsm._enter(DockState.DOCKED)
    assert fsm._success is True
    # DOCKED 下 start 仍被忽略（须先 undock/cancel）
    _start(fsm)
    assert fsm._state == DockState.DOCKED
    fsm._cancel_cb(Empty())
    assert fsm._state == DockState.IDLE
    assert fsm._success is False
    # IDLE 下可再次 start（二次归港不被残留标志污染）
    _start(fsm)
    assert fsm._state == DockState.ACQUIRE_TAG
    assert fsm._success is False


def test_undock_success_cleared_on_idle_entry(fsm):
    """undock_success 回 IDLE 后同样须清掉（与 success 对称的二次任务竞态）。"""
    fsm._undock_cb(Bool(data=True))
    fsm._transitions(0.1)
    _feed(fsm, x=-4.5)
    fsm._transitions(0.1)
    assert fsm._state == DockState.UNDOCK_SETTLE
    assert fsm._undock_success is True
    _backdate(fsm, 2.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._undock_success is False


def test_start_accepted_from_failed(fsm):
    """FAILED 状态应接受 /dock/start 重新开始（与接口文档一致）。"""
    _start(fsm)
    fsm._transitions(0.1)
    req = String()
    req.data = "TOPIC_TIMEOUT:POSE"
    fsm._abort_request_cb(req)
    assert fsm._state == DockState.FAILED
    _start(fsm)
    assert fsm._state == DockState.ACQUIRE_TAG
    assert fsm._success is False
    assert fsm._needs_manual_takeover is False


# ══════════════ 船坞夹爪交互（dock_claw_enabled，默认关闭）══════════════


def _enable_claw(fsm):
    from rclpy.parameter import Parameter

    fsm.set_parameters(
        [Parameter("dock_claw_enabled", Parameter.Type.BOOL, True)]
    )


def test_claw_disabled_default_paths_unchanged(fsm):
    """默认 dock_claw_enabled=false：start 直进 ACQUIRE_TAG，undock 直进
    UNDOCK_EXIT，BACK_IN 无位移停滞检测，FINAL_DOCK 仍按位置判 DOCKED。"""
    _start(fsm)
    assert fsm._state == DockState.ACQUIRE_TAG
    # BACK_IN 20s 无位移：不触发停滞中止（检测未开启）
    fsm._enter(DockState.BACK_IN)
    for _ in range(200):
        _feed(fsm, x=-1.0, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.BACK_IN
    # FINAL_DOCK 位置到位保持 1s -> DOCKED（位置判据仍在）
    fsm._enter(DockState.FINAL_DOCK)
    for _ in range(11):
        _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.DOCKED
    # undock 直进 UNDOCK_EXIT（无 WAIT_DOCK_RELEASE）
    fsm._cancel_cb(Empty())
    fsm._undock_cb(Bool(data=True))
    assert fsm._state == DockState.UNDOCK_EXIT


def test_start_requests_dock_open_when_claw_enabled(fsm):
    """夹爪模式：/dock/start -> WAIT_DOCK_OPEN（HOLD），ControlDO 成功才归港。"""
    _enable_claw(fsm)
    _start(fsm)
    assert fsm._state == DockState.WAIT_DOCK_OPEN
    assert fsm._compute_mode() == MODE_HOLD
    fsm._control_do_result = (True, "opened")
    fsm._transitions(0.1)
    assert fsm._state == DockState.ACQUIRE_TAG


def test_dock_open_failure_reports_reapproach(fsm):
    """夹爪打开失败：船尚未动，回 IDLE 上报 needs_reapproach（上层可重试）。"""
    _enable_claw(fsm)
    _start(fsm)
    fsm._control_do_result = (False, "io error")
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._needs_reapproach is True
    assert "DOCK_OPEN_FAILED" in fsm._abort_reason


def test_dock_open_timeout(fsm):
    """ControlDO server 长时间不就绪：dock_open_timeout_sec 超时回 IDLE。"""
    _enable_claw(fsm)
    fsm._send_control_do = lambda: None  # 隔离 Action 环境依赖
    _start(fsm)
    _backdate(fsm, 16.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._abort_reason == "DOCK_OPEN_TIMEOUT"
    assert fsm._needs_reapproach is True


def test_claw_grabbed_in_back_in_docks(fsm):
    """夹爪模式：BACK_IN 中 WaitIO 报抓住 -> 直接 DOCKED（不看位置）。"""
    _enable_claw(fsm)
    _start(fsm)
    fsm._control_do_result = (True, "ok")
    fsm._transitions(0.1)
    assert fsm._state == DockState.ACQUIRE_TAG
    fsm._enter(DockState.BACK_IN)
    _feed(fsm, x=-1.0, y=0.0)
    fsm._claw_grabbed = True  # 模拟 WaitIO 结果回调置位
    fsm._transitions(0.1)
    assert fsm._state == DockState.DOCKED
    assert fsm._success is True


def test_final_dock_position_alone_not_success_when_claw_enabled(fsm):
    """夹爪模式：FINAL_DOCK 位置/姿态达标不判成功，只有 WaitIO 抓住才 DOCKED。"""
    _enable_claw(fsm)
    fsm._enter(DockState.FINAL_DOCK)
    for _ in range(20):  # 2s > docked_hold_sec(1s)：位置判据被屏蔽则保持
        _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.FINAL_DOCK
    fsm._claw_grabbed = True
    _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._state == DockState.DOCKED


def test_back_in_stuck_aborts_when_claw_enabled(fsm):
    """夹爪模式：BACK_IN 位移停滞超 backin_stuck_timeout_sec(15s)
    -> ABORT_EXIT（驶出后由 dock_mission 重试）。"""
    _enable_claw(fsm)
    fsm._enter(DockState.BACK_IN)
    for _ in range(160):  # 16s 无位移
        _feed(fsm, x=-1.0, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ABORT_EXIT
    assert fsm._abort_reason == "BACK_IN_STUCK"


def test_back_in_progress_resets_stuck_timer(fsm):
    """夹爪模式：倒船持续推进（每 10s 进展 0.05m > progress_m）不触发停滞。"""
    _enable_claw(fsm)
    fsm._enter(DockState.BACK_IN)
    x = -2.0
    for _ in range(8):  # 共 80s < back_in_timeout(90s)
        for _ in range(100):  # 10s
            _feed(fsm, x=x, y=0.0)
            fsm._transitions(0.1)
        assert fsm._state == DockState.BACK_IN
        x += 0.05


def test_undock_requests_release_when_claw_enabled(fsm):
    """夹爪模式出泊：先 WAIT_DOCK_RELEASE 请求松开夹爪，成功才 UNDOCK_EXIT。"""
    _enable_claw(fsm)
    fsm._undock_cb(Bool(data=True))
    assert fsm._state == DockState.WAIT_DOCK_RELEASE
    assert fsm._compute_mode() == MODE_HOLD
    fsm._control_do_result = (True, "released")
    fsm._transitions(0.1)
    assert fsm._state == DockState.UNDOCK_EXIT


def test_undock_release_failure_fails(fsm):
    """夹爪松开失败：FAILED 停车（爪子可能仍扣着，禁止盲动）。"""
    _enable_claw(fsm)
    fsm._undock_cb(Bool(data=True))
    fsm._control_do_result = (False, "motor stuck")
    fsm._transitions(0.1)
    assert fsm._state == DockState.FAILED
    assert "DOCK_RELEASE_FAILED" in fsm._abort_reason
    assert fsm._undock_success is False
    assert fsm._needs_manual_takeover is True


def test_wait_io_result_outside_corridor_ignored(fsm):
    """WaitIO 迟到的成功结果只在坞内状态生效（离开后不判抓住）。"""

    class _Res:
        success = True
        message = "grabbed"

    class _Future:
        def result(self):
            return type("_R", (), {"result": _Res()})()

    _enable_claw(fsm)
    fsm._enter(DockState.ABORT_EXIT)  # 非坞内状态
    fsm._on_wait_io_result_cb(_Future())
    assert fsm._claw_grabbed is False
