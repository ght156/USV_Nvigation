#!/usr/bin/env python3
"""docking_fsm 状态转移单元测试（rclpy 离线驱动，不需要仿真）。

驱动方式：直接调输入回调注入合成消息，再手动调 _transitions(dt)。
"""

import json
import math

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, String

from usv_docking.docking_fsm import (
    DockState,
    DockingFsm,
    MODE_APPROACH,
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
    # 本文件的历史用例都按"无夹爪"路径编写（位置到位判据、start 直进
    # ACQUIRE_TAG）。代码默认值 2026-10-07 起 dock_claw_enabled=true
    # （兴飞船带夹爪，与 config/docking.yaml 对齐），故在此**显式**关闭，
    # 不依赖默认值；夹爪路径见文件末尾「船坞夹爪交互」小节。
    node.set_parameters(
        [Parameter("dock_claw_enabled", Parameter.Type.BOOL, False)]
    )
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
    _feed(fsm, x=-2.0, y=0.0)
    for _ in range(6):
        _feed(fsm, x=-2.0, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 满足对准门槛持续 2s（align_hold_sec 2026-10-05 起 2.0s）-> BACK_IN
    for _ in range(22):
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi)
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
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi + math.radians(40.0))
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 艏向收进门槛后持续 0.5s -> ALIGN_ENTRY
    for _ in range(6):
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi)
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
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi, speed=0.3)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 速度降到 0 后持续 0.5s -> ALIGN_ENTRY
    for _ in range(6):
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi, speed=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY


def test_approach_stuck_escapes_to_reapproach(fsm):
    """预备点退化区打转逃逸（2026-10-07 现场 bag 实锤）：APPROACH 中大艏偏
    （|e_yaw|>approach_yaw_tol_deg）持续累计超 approach_stuck_sec(25s)
    收不进 -> ABORT 回 IDLE + needs_reapproach（比干等 120s 超时快）。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    big_yaw = math.pi + math.radians(40.0)  # > 25° 放行门槛
    # 24s 大艏偏（< 25s）：保持 APPROACH，不逃逸
    for _ in range(240):
        _feed(fsm, x=-2.0, y=0.3, yaw=big_yaw)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY
    # 艏向短暂收进门槛：计时清零（毛刺不累计）
    _feed(fsm, x=-2.0, y=0.3, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._approach_stuck == 0.0
    # 再次大艏偏 25s+：逃逸 -> IDLE + needs_reapproach
    for _ in range(255):
        _feed(fsm, x=-2.0, y=0.3, yaw=big_yaw)
        fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._needs_reapproach is True
    assert fsm._abort_reason == "APPROACH_STUCK"


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
        _feed(fsm, x=-2.0, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    _feed(fsm, x=-2.0, y=0.5)  # |e_y| > align_y_abort(0.45)
    fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_align_y_stuck_escapes_to_approach(fsm):
    """y 卡死带逃逸：艏向已准但 |y| 在 (align_y_tol, align_y_abort] 滞留超时回
    APPROACH。

    用生产默认值（align_y_tol=0.25 / align_y_abort=0.45，2026-10-07 起代码默认
    与 yaml 一致），取 y=0.35 落在卡死带内。原实现钉住 approach_y_tol=0.15 是
    为对齐当时的 yaml；现在两者都是 0.25，逃逸后 APPROACH 也不满足
    approach_y_tol，不会立刻弹回，故不再需要钉参数。
    """
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    for _ in range(6):
        _feed(fsm, x=-2.0, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 艏向已准（yaw=π -> e_yaw=0）但 |y|=0.35 落在卡死带 (0.25, 0.45]
    for _ in range(50):  # 5.0s < 6.0s：不逃逸
        _feed(fsm, x=-2.0, y=0.35, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    for _ in range(15):  # 累计 6.5s > 6.0s：逃逸
        _feed(fsm, x=-2.0, y=0.35, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.APPROACH_ENTRY


def test_align_tag_loss_grace(fsm):
    """ALIGN 丢 Tag 宽限：短暂推算不弹回，超宽限才进 REACQUIRE。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    for _ in range(6):
        _feed(fsm, x=-2.0, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 宽限内（1.0s < 5s）的推算闪烁：保持 ALIGN
    for _ in range(10):
        _feed(fsm, x=-2.0, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ALIGN_ENTRY
    # 视觉恢复：计时清零，ALIGN 继续正常推进（gate 持续 2s 满足才进 BACK_IN）
    for _ in range(22):
        _feed(fsm, x=-2.0, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.BACK_IN
    # 回到 ALIGN 场景：超过宽限（6s）-> REACQUIRE
    fsm._enter(DockState.ALIGN_ENTRY)
    for _ in range(60):
        _feed(fsm, x=-2.0, source="ODOM_PREDICTION", visible=False, age=6.0)
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


def test_reacquire_drives_to_virtual_dock_when_prediction_valid(fsm):
    """会话级虚拟坞点回归（2026-10-07）：REACQUIRE 锚点活着（odom 推算）时
    朝推算预备点开回去（MODE_APPROACH），不再原地 HOLD/SEARCH；
    位姿 INVALID（锚点也丢了）保持原行为 -> SEARCH。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    # odom 推算中（锚点活着）且距离在门内 -> 朝虚拟坞点开回去
    _feed(fsm, x=-4.0, source="ODOM_PREDICTION", visible=False, age=1.0)
    fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_APPROACH
    # 位姿无效 -> SEARCH（原行为）
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_SEARCH


def test_reacquire_virtual_dock_gated_by_distance(fsm):
    """虚拟坞位的距离门（2026-10-07 现场）：远场视觉的坞位姿必跳（实测重定位
    dp 0.77~1.21m），拿它当真值去盲开等于朝幻影开、把船带走。故
    |dock_x| > reacquire_drive_max_dist_m(5m) -> HOLD（旧行为），
    进到门内才 MODE_APPROACH。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    # 远场（x=-8，超 5m 门）：不驾驶，退回 HOLD
    _feed(fsm, x=-8.0, source="ODOM_PREDICTION", visible=False, age=1.0)
    fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_HOLD
    # 拉进 5m 门内：允许朝虚拟坞位受控对准/接近
    _feed(fsm, x=-4.0, source="ODOM_PREDICTION", visible=False, age=1.0)
    fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_APPROACH


def test_reacquire_vision_flicker_stays_hold(fsm):
    """REACQUIRE 中 VISION 但 tag 闪烁（visible=False）：HOLD 等其恢复，
    不开车也不自转（SEARCH 会扫漂刚修好的横向偏差，08-10 实测活锁）。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    _feed(fsm, source=SRC_VISION, visible=False)
    fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_HOLD


def test_reacquire_drive_budget_exhausted_falls_back_to_hold(fsm):
    """推算驾驶预算保护：同一归港会话内 ODOM_PREDICTION 驾驶累计超
    reacquire_drive_max_sec(10s) 退回 HOLD（防锚点有偏时盲开到底）。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    _feed(fsm, source=SRC_INVALID, visible=False)
    fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    # 累计 10.0s（<=预算）：仍在推算驾驶
    for _ in range(100):
        _feed(fsm, source="ODOM_PREDICTION", visible=False, age=5.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    assert fsm._compute_mode() == MODE_APPROACH
    # 再 0.2s：累计 10.2s 超预算 -> HOLD
    for _ in range(2):
        _feed(fsm, source="ODOM_PREDICTION", visible=False, age=5.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.REACQUIRE_TAG
    assert fsm._compute_mode() == MODE_HOLD


def test_reacquire_drive_budget_is_session_scoped(fsm):
    """盲驾预算是会话级（2026-10-07 评审）：APPROACH↔REACQUIRE 乒乓不重置
    （否则每轮重获全额 10s，累计不受控）；回 IDLE（会话结束）清零。"""
    _start(fsm)
    for _ in range(5):
        _feed(fsm)
        fsm._transitions(0.1)
    fsm._enter(DockState.REACQUIRE_TAG)
    for _ in range(60):  # 6s 推算驾驶
        _feed(fsm, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_APPROACH
    # 乒乓：弹回 APPROACH 再进 REACQUIRE——_enter 不重置会话级预算
    fsm._enter(DockState.APPROACH_ENTRY)
    fsm._enter(DockState.REACQUIRE_TAG)
    assert fsm._reacquire_drive_elapsed == pytest.approx(6.0, abs=1e-6)
    # 再 4.5s：累计 10.5s 超预算 -> HOLD
    for _ in range(45):
        _feed(fsm, source="ODOM_PREDICTION", visible=False, age=1.0)
        fsm._transitions(0.1)
    assert fsm._compute_mode() == MODE_HOLD
    # 会话结束（回 IDLE）清零，下次会话重新获得全额预算
    fsm._enter(DockState.IDLE)
    assert fsm._reacquire_drive_elapsed == 0.0


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


# ══════════════ 船坞夹爪交互（dock_claw_enabled）══════════════


def _enable_claw(fsm):
    fsm.set_parameters(
        [Parameter("dock_claw_enabled", Parameter.Type.BOOL, True)]
    )


def test_claw_disabled_paths_unchanged(fsm):
    """dock_claw_enabled=false（本文件 fixture 显式设定）：start 直进
    ACQUIRE_TAG，undock 直进 UNDOCK_EXIT，BACK_IN 无位移停滞检测。
    FINAL_DOCK 自 2026-10-05 压紧段设计起不再以位置到位判 DOCKED
    （见 test_final_push_timeout_aborts）。"""
    _start(fsm)
    assert fsm._state == DockState.ACQUIRE_TAG
    # BACK_IN 20s 无位移：不触发停滞中止（检测未开启）
    fsm._enter(DockState.BACK_IN)
    for _ in range(200):
        _feed(fsm, x=-1.0, y=0.0)
        fsm._transitions(0.1)
    assert fsm._state == DockState.BACK_IN
    # undock 直进 UNDOCK_EXIT（无 WAIT_DOCK_RELEASE）
    fsm._cancel_cb(Empty())
    fsm._undock_cb(Bool(data=True))
    assert fsm._state == DockState.UNDOCK_EXIT


def test_claw_enabled_gates_start_and_undock(fsm):
    """dock_claw_enabled=true（兴飞生产配置）：
    - /dock/start 必须先经 WAIT_DOCK_OPEN 等船坞开夹爪，成功才 ACQUIRE_TAG；
    - /dock/undock 必须先经 WAIT_DOCK_RELEASE，成功才 UNDOCK_EXIT；
    - usv_rs485_driver 不可用时降级：开夹爪超时回 IDLE 上报可重试（不盲动），
      松夹爪失败停 FAILED（爪子可能仍扣着）。
    回归背景：2026-10-07 代码默认值由 false 改 true 后，单测若仍假设"默认
    无夹爪"，start 路径会被静默改道。"""
    _enable_claw(fsm)
    _start(fsm)
    assert fsm._state == DockState.WAIT_DOCK_OPEN
    # Action 包在单测环境不可导入 -> 不会误进 ACQUIRE_TAG；超时后回 IDLE
    _backdate(fsm, 20.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._needs_reapproach is True
    # 出泊：需先从 DOCKED 触发
    fsm._state = DockState.DOCKED
    fsm._undock_cb(Bool(data=True))
    assert fsm._state == DockState.WAIT_DOCK_RELEASE
    _backdate(fsm, 20.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.FAILED


# ══════════════ FINAL_DOCK 压紧段 + 无线充电判据（2026-10-05）══════════════


class _ChargeMsg:
    def __init__(self, charging_status=1, error_code=0):
        self.charging_status = charging_status
        self.error_code = error_code


def test_final_push_timeout_aborts(fsm):
    """压紧段超时：到死位置后位置不再判成功；无充电监测（服务不可用降级）
    且无夹爪时，压紧超 final_push_time_sec -> ABORT_EXIT 重试。"""
    _start(fsm)
    fsm._enter(DockState.FINAL_DOCK)
    # 首次到死位置：进入压紧段（记时间戳），不判 DOCKED
    _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._state == DockState.FINAL_DOCK
    assert fsm._final_push_start is not None
    # 压紧 5.9s（< 6.0s）：保持 FINAL_DOCK（旧逻辑 1s 即 DOCKED）
    for _ in range(58):
        _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.FINAL_DOCK
    # 累计 6.0s+：压紧超时 -> ABORT_EXIT
    for _ in range(3):
        _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.ABORT_EXIT
    assert fsm._abort_reason == "FINAL_PUSH_TIMEOUT"
    # 驶出完成回 IDLE：压紧状态已复位
    _feed(fsm, x=-4.5, y=0.0)
    fsm._transitions(0.1)
    assert fsm._state == DockState.IDLE
    assert fsm._needs_reapproach is True
    assert fsm._final_push_start is None


def test_charge_stable_docks(fsm):
    """充电判据（主）：压紧段内好样本（charging_status∈{1,5} 且 error_code==0）
    连续稳定 charge_stable_sec -> DOCKED；坏样本清零计时。"""
    _start(fsm)
    fsm._enter(DockState.FINAL_DOCK)
    fsm._charge_monitor_on = True  # 模拟监测已打开
    _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._final_push_start is not None
    # 好样本累计 1.9s（< 2.0s）：不判成功
    for _ in range(19):
        fsm._charge_cb(_ChargeMsg())
        _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.FINAL_DOCK
    # 坏样本清零：再多 1.9s 好样本仍不够（累计重新从 0 起）
    fsm._charge_cb(_ChargeMsg(charging_status=0, error_code=1))
    for _ in range(19):
        fsm._charge_cb(_ChargeMsg())
        _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.FINAL_DOCK
    # 再补 0.2s 好样本：连续满 2.0s -> DOCKED
    for _ in range(3):
        fsm._charge_cb(_ChargeMsg(charging_status=5))
        _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
        fsm._transitions(0.1)
    assert fsm._state == DockState.DOCKED
    assert fsm._success is True


def test_charge_signal_lost_aborts(fsm):
    """无夹爪模式：压紧段内充电话题超 charge_status_timeout_sec 无消息
    （监测已打开但始终无帧）-> ABORT_EXIT（CHARGE_SIGNAL_LOST）。"""
    _start(fsm)
    fsm._enter(DockState.FINAL_DOCK)
    fsm._charge_monitor_on = True
    _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._final_push_start is not None
    # 回拨压紧起点模拟 3.5s 无消息（> 断流判据 3s，< 压紧超时 6s，先触发断流）
    fsm._final_push_start = fsm.get_clock().now() - Duration(seconds=3.5)
    _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._state == DockState.ABORT_EXIT
    assert fsm._abort_reason == "CHARGE_SIGNAL_LOST"


def test_charge_signal_lost_degrades_when_claw_enabled(fsm):
    """夹爪模式：充电话题断流不中止（WaitIO 在跑），压紧段继续，
    夹爪 DI 后置位仍应 DOCKED（2026-10-06 现场：DI 比断流中止晚 0.1s）。"""
    _enable_claw(fsm)
    _start(fsm)
    fsm._control_do_result = (True, "ok")
    fsm._transitions(0.1)
    fsm._enter(DockState.FINAL_DOCK)
    fsm._charge_monitor_on = True
    _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._final_push_start is not None
    # 断流超 3s：不中止，仅降级
    fsm._final_push_start = fsm.get_clock().now() - Duration(seconds=3.5)
    _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._state == DockState.FINAL_DOCK
    assert fsm._charge_signal_dead is True
    # 随后夹爪 DI 置位（WaitIO 成功）-> DOCKED
    fsm._claw_grabbed = True
    _feed(fsm, x=0.05, y=0.0, yaw=math.pi)
    fsm._transitions(0.1)
    assert fsm._state == DockState.DOCKED


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
