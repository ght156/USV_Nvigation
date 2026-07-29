#!/usr/bin/env python3
"""docking_fsm_v2 — 完整归港状态机（V2）。

状态流：
    IDLE -> ACQUIRE_TAG -> APPROACH_ENTRY -> ALIGN_ENTRY
        -> BACK_IN -> FINAL_DOCK -> DOCKED
    异常：REACQUIRE_TAG（入口外丢 Tag）/ ABORT_EXIT（坞内失败驶出）/ FAILED
    出泊：UNDOCK_EXIT -> UNDOCK_SETTLE（/dock/undock 触发）

输入：
  /docking_v2/dock_pose        (PoseStamped, base_link 在 dock_est 系位姿)
  /docking_v2/tag_visible      (Bool)
  /docking_v2/pose_source      (String: VISION/ODOM_PREDICTION/INVALID)
  /docking_v2/measurement_age  (Float32)
  /docking_v2/abort_request    (String, 来自 safety，空串=无请求)
  /dock/start                  (Bool, dock_mission 契约)
  /dock/cancel                 (Empty, dock_mission 契约)
  /dock/undock                 (Bool, dock_mission 契约)

输出：
  /docking_v2/state            (String)
  /docking_v2/target_mode      (String, 控制器阶段目标，见 MODE_*)
  /docking_v2/reset_anchor     (Bool, 可选在进入 ACQUIRE_TAG 时重置锚点)
  /dock/status                 (String JSON, dock_mission 兼容契约：
                                 success/needs_reapproach/abort_reason/
                                 undock_success/state；FAILED 映射为
                                 state="DOCK_ABORT" 且 needs_reapproach=true，
                                 原始状态见 v2_state)

关键约定：
  - 误差在 dock_est 系表达：x<0 在坞外、x≈0 在 bay 中心；对准 = yaw≈±pi
    （e_yaw = wrap(yaw - pi)，船尾朝坞）；e_y = y（横向偏差）。
  - 入口外状态（ACQUIRE/APPROACH/ALIGN/REACQUIRE）失败 -> IDLE + needs_reapproach；
    坞内状态（BACK_IN/FINAL_DOCK）失败 -> ABORT_EXIT 驶出后再上报。
  - 内部不自动重试：ABORT_EXIT 完成回 IDLE，重试由 dock_mission 决定。
  - ABORT_EXIT / UNDOCK 完成判据：位姿有效时用 dock 系 x，无效时退化为超时兜底
    （控制器另行用 odom 距离跟踪，见任务5）。
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, Float32, String


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(z: float, w: float) -> float:
    """平面四元数（仅 yaw 分量）-> yaw。"""
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


class DockState:
    IDLE = "IDLE"
    ACQUIRE_TAG = "ACQUIRE_TAG"
    APPROACH_ENTRY = "APPROACH_ENTRY"
    ALIGN_ENTRY = "ALIGN_ENTRY"
    BACK_IN = "BACK_IN"
    FINAL_DOCK = "FINAL_DOCK"
    DOCKED = "DOCKED"
    REACQUIRE_TAG = "REACQUIRE_TAG"
    ABORT_EXIT = "ABORT_EXIT"
    FAILED = "FAILED"
    UNDOCK_EXIT = "UNDOCK_EXIT"
    UNDOCK_SETTLE = "UNDOCK_SETTLE"


# 坞内状态（失败须 ABORT_EXIT 驶出）与入口外状态（失败直接回 IDLE 上报）
CORRIDOR_STATES = (DockState.BACK_IN, DockState.FINAL_DOCK)
OUTSIDE_STATES = (
    DockState.ACQUIRE_TAG,
    DockState.APPROACH_ENTRY,
    DockState.ALIGN_ENTRY,
    DockState.REACQUIRE_TAG,
)

# target_mode 取值（控制器任务5消费）
MODE_HOLD = "HOLD"                    # 零速
MODE_SEARCH = "SEARCH"                # 按 odom 预测方位的扇形搜索（无锚点则自转）
MODE_APPROACH = "APPROACH"            # 弧线到入口外预备点
MODE_ALIGN = "ALIGN"                  # v=0 只修航向
MODE_BACK_IN = "BACK_IN"              # 沿中心线倒入
MODE_FINAL_DOCK = "FINAL_DOCK"        # 低速倒向充电位
MODE_SEARCH_LIMITED = "SEARCH_LIMITED"  # 坞内小角度搜索（±backin_max_search_angle）
MODE_EXIT_FORWARD = "EXIT_FORWARD"    # ABORT_EXIT 沿轴线前进驶出
MODE_UNDOCK_FORWARD = "UNDOCK_FORWARD"  # 出泊前进
MODE_DOCKED_HOLD = "DOCKED_HOLD"      # 入泊保持

SRC_VISION = "VISION"
SRC_PREDICTION = "ODOM_PREDICTION"
SRC_INVALID = "INVALID"


class DockingFsmV2(Node):
    """归港状态机 V2（任务 4：状态切换 + 契约输出，不发 cmd_vel）。"""

    def __init__(self):
        super().__init__("docking_fsm_v2")

        # ── 输入话题 ──
        self.declare_parameter("dock_pose_topic", "/docking_v2/dock_pose")
        self.declare_parameter("tag_visible_topic", "/docking_v2/tag_visible")
        self.declare_parameter("pose_source_topic", "/docking_v2/pose_source")
        self.declare_parameter("measurement_age_topic", "/docking_v2/measurement_age")
        self.declare_parameter("abort_request_topic", "/docking_v2/abort_request")
        self.declare_parameter("start_topic", "/dock/start")
        self.declare_parameter("cancel_topic", "/dock/cancel")
        self.declare_parameter("undock_topic", "/dock/undock")

        # ── 输出话题 ──
        self.declare_parameter("state_topic", "/docking_v2/state")
        self.declare_parameter("target_mode_topic", "/docking_v2/target_mode")
        self.declare_parameter("reset_anchor_topic", "/docking_v2/reset_anchor")
        self.declare_parameter("status_topic", "/dock/status")

        # ── 运行 ──
        self.declare_parameter("state_rate", 10.0)
        self.declare_parameter("reset_anchor_on_start", False)

        # ── ACQUIRE_TAG ──
        self.declare_parameter("tag_acquire_frames", 5)
        self.declare_parameter("acquire_timeout_sec", 60.0)
        # 集帧闪烁容忍：视野边缘逐帧丢检是常态，零容忍会反复清零
        # 集帧计数并伴随 HOLD<->SEARCH 角速度忽高忽低（2026-07-29 实测捕获 6 分钟）
        self.declare_parameter("acquire_miss_tolerance", 3)

        # ── APPROACH_ENTRY：预备点与到点容差（dock 系）──
        self.declare_parameter("staging_x", -2.5)
        self.declare_parameter("approach_x_tol", 0.5)
        self.declare_parameter("approach_y_tol", 0.5)
        self.declare_parameter("approach_timeout_sec", 90.0)

        # ── ALIGN_ENTRY：进 BACK_IN 门槛 / 退回门槛 ──
        self.declare_parameter("align_y_tol", 0.20)
        self.declare_parameter("align_yaw_tol_deg", 5.0)
        self.declare_parameter("align_hold_sec", 1.0)
        self.declare_parameter("align_y_abort", 0.35)
        self.declare_parameter("align_timeout_sec", 45.0)
        # 艏向已准但 y 滞留卡死带 (y_tol, y_abort] 的逃逸时限：
        # 超时确定性回 APPROACH 修 y（替代干等噪声/总超时）
        self.declare_parameter("align_y_stuck_sec", 6.0)
        # 允许进 BACK_IN 的纵向窗口（坞外）
        self.declare_parameter("entry_window_min_x", -3.5)
        self.declare_parameter("entry_window_max_x", -1.0)

        # ── BACK_IN ──
        self.declare_parameter("final_target_x", 0.0)  # 最终充电位（bay 中心 P）
        self.declare_parameter("final_dock_entry_dist", 0.8)
        self.declare_parameter("back_in_timeout_sec", 90.0)
        self.declare_parameter("back_in_tag_loss_hold_sec", 0.5)
        self.declare_parameter("back_in_tag_loss_search_sec", 2.0)
        # 走廊违规（与控制器门控2一致；连续 violation_cycles 周期 -> ABORT_EXIT）
        self.declare_parameter("back_in_gate2_y", 0.35)
        self.declare_parameter("back_in_gate2_yaw_deg", 10.0)
        self.declare_parameter("violation_cycles", 10)

        # ── FINAL_DOCK ──
        self.declare_parameter("docked_x_tol", 0.15)
        self.declare_parameter("docked_y_tol", 0.10)
        self.declare_parameter("docked_yaw_tol_deg", 3.0)
        self.declare_parameter("docked_hold_sec", 1.0)
        self.declare_parameter("final_dock_timeout_sec", 60.0)
        self.declare_parameter("final_dock_tag_loss_timeout_sec", 2.0)

        # ── ALIGN_ENTRY 丢 Tag 宽限 ──
        # 视野边缘检测逐帧闪烁（估计器 0.3s 无帧即切推算），零容忍会与 REACQUIRE
        # 高频互弹（2026-07-29 实测 40s 弹 6 次）；宽限期内靠 odom 推算继续对准
        self.declare_parameter("align_tag_loss_grace_sec", 5.0)

        # ── REACQUIRE_TAG ──
        self.declare_parameter("reacquire_frames", 5)
        self.declare_parameter("reacquire_miss_tolerance", 3)
        self.declare_parameter("reacquire_timeout_sec", 30.0)

        # ── ABORT_EXIT ──
        self.declare_parameter("exit_complete_x", -4.0)
        self.declare_parameter("abort_exit_timeout_sec", 45.0)

        # ── UNDOCK ──
        self.declare_parameter("undock_complete_x", -4.0)
        self.declare_parameter("undock_timeout_sec", 60.0)
        self.declare_parameter("undock_settle_sec", 1.0)

        # ── 全局 ──
        self.declare_parameter("max_docking_duration_sec", 150.0)

        p = self.get_parameter
        qos = QoSProfile(depth=10)

        self._state_pub = self.create_publisher(String, p("state_topic").value, qos)
        self._target_mode_pub = self.create_publisher(
            String, p("target_mode_topic").value, qos
        )
        self._reset_anchor_pub = self.create_publisher(
            Bool, p("reset_anchor_topic").value, qos
        )
        self._status_pub = self.create_publisher(String, p("status_topic").value, qos)

        self.create_subscription(
            PoseStamped, p("dock_pose_topic").value, self._dock_pose_cb, qos
        )
        self.create_subscription(
            Bool, p("tag_visible_topic").value, self._tag_visible_cb, qos
        )
        self.create_subscription(
            String, p("pose_source_topic").value, self._pose_source_cb, qos
        )
        self.create_subscription(
            Float32, p("measurement_age_topic").value, self._measurement_age_cb, qos
        )
        self.create_subscription(
            String, p("abort_request_topic").value, self._abort_request_cb, qos
        )
        self.create_subscription(Bool, p("start_topic").value, self._start_cb, qos)
        self.create_subscription(Empty, p("cancel_topic").value, self._cancel_cb, qos)
        self.create_subscription(Bool, p("undock_topic").value, self._undock_cb, qos)

        # ── 运行状态 ──
        self._state = DockState.IDLE
        self._state_enter_time = None
        self._mission_start_time = None
        self._last_tick_time = None

        # 最新位姿（dock 系）
        self._dock_x = None
        self._dock_y = None
        self._dock_yaw = None
        self._tag_visible = False
        self._pose_source = SRC_INVALID
        self._measurement_age = float("inf")

        # 计数器 / 计时器
        self._acquire_frames = 0
        self._acquire_miss = 0
        self._align_hold = 0.0
        self._align_tag_loss = 0.0
        self._align_y_stuck = 0.0
        self._docked_hold = 0.0
        self._violation_count = 0
        self._reacquire_frames = 0
        self._reacquire_miss = 0

        # 上报标志（dock_mission 契约）
        self._success = False
        self._undock_success = False
        self._needs_reapproach = False
        self._needs_manual_takeover = False
        self._abort_reason = None

        rate = float(p("state_rate").value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info("docking_fsm_v2 已启动，状态 IDLE，等待 /dock/start")

    # ══════════════ 输入回调 ══════════════
    def _dock_pose_cb(self, msg: PoseStamped):
        self._dock_x = msg.pose.position.x
        self._dock_y = msg.pose.position.y
        self._dock_yaw = yaw_from_quat(
            msg.pose.orientation.z, msg.pose.orientation.w
        )

    def _tag_visible_cb(self, msg: Bool):
        self._tag_visible = bool(msg.data)

    def _pose_source_cb(self, msg: String):
        self._pose_source = msg.data

    def _measurement_age_cb(self, msg: Float32):
        self._measurement_age = float(msg.data)

    def _abort_request_cb(self, msg: String):
        if not msg.data or self._state in (DockState.IDLE, DockState.FAILED):
            return
        reason = msg.data
        self.get_logger().warn(f"安全退出请求: {reason}（当前 {self._state}）")
        if reason.startswith(("ODOM", "TOPIC_TIMEOUT")):
            # odom 真异常 / 上游节点断流：不能盲驶，直接 FAILED 停车
            self._fail(f"SAFETY:{reason}")
        elif self._state in (DockState.UNDOCK_EXIT, DockState.UNDOCK_SETTLE):
            # 出泊中异常：无再撤离必要，直接 FAILED 上报
            self._fail(f"SAFETY:{reason}")
        elif self._state in CORRIDOR_STATES:
            self._abort_reason = f"SAFETY:{reason}"
            self._enter(DockState.ABORT_EXIT)
        elif self._state in OUTSIDE_STATES:
            self._abort_and_report(f"SAFETY:{reason}")

    def _start_cb(self, msg: Bool):
        if not msg.data or self._state != DockState.IDLE:
            return
        self.get_logger().info("收到 /dock/start，进入 ACQUIRE_TAG")
        self._reset_flags()
        self._mission_start_time = self.get_clock().now()
        if self.get_parameter("reset_anchor_on_start").value:
            self._reset_anchor_pub.publish(Bool(data=True))
        self._enter(DockState.ACQUIRE_TAG)

    def _cancel_cb(self, _msg: Empty):
        if self._state == DockState.IDLE:
            return
        self.get_logger().info(f"收到 /dock/cancel：{self._state} -> IDLE")
        self._enter(DockState.IDLE)

    def _undock_cb(self, msg: Bool):
        if not msg.data or self._state not in (DockState.IDLE, DockState.DOCKED):
            return
        self.get_logger().info(f"收到 /dock/undock：{self._state} -> UNDOCK_EXIT")
        self._reset_flags()
        self._mission_start_time = self.get_clock().now()
        self._enter(DockState.UNDOCK_EXIT)

    # ══════════════ 状态工具 ══════════════
    def _enter(self, state: str):
        self.get_logger().info(f"状态: {self._state} -> {state}")
        self._state = state
        self._state_enter_time = self.get_clock().now()
        self._acquire_frames = 0
        self._acquire_miss = 0
        self._align_hold = 0.0
        self._align_tag_loss = 0.0
        self._align_y_stuck = 0.0
        self._docked_hold = 0.0
        self._violation_count = 0
        self._reacquire_frames = 0
        self._reacquire_miss = 0
        if state == DockState.DOCKED:
            self._success = True
        if state == DockState.IDLE and not self._undock_success:
            # 回 IDLE 时保留 needs_reapproach / success 供 dock_mission 读取，
            # 仅清任务进行态
            pass

    def _reset_flags(self):
        self._success = False
        self._undock_success = False
        self._needs_reapproach = False
        self._needs_manual_takeover = False
        self._abort_reason = None

    def _fail(self, reason: str):
        self.get_logger().error(f"FAILED: {reason}")
        self._abort_reason = reason
        self._needs_manual_takeover = True
        self._enter(DockState.FAILED)

    def _abort_and_report(self, reason: str):
        """入口外失败：直接回 IDLE 并上报 needs_reapproach。"""
        self.get_logger().warn(f"任务中止（{self._state}）: {reason}")
        self._abort_reason = reason
        self._needs_reapproach = True
        self._enter(DockState.IDLE)

    def _abort_corridor(self, reason: str):
        """坞内失败：先 ABORT_EXIT 驶出。"""
        self.get_logger().warn(f"坞内中止（{self._state}）: {reason} -> ABORT_EXIT")
        self._abort_reason = reason
        self._enter(DockState.ABORT_EXIT)

    def _state_elapsed(self) -> float:
        if self._state_enter_time is None:
            return 0.0
        return (self.get_clock().now() - self._state_enter_time).nanoseconds * 1e-9

    def _mission_elapsed(self) -> float:
        if self._mission_start_time is None:
            return 0.0
        return (self.get_clock().now() - self._mission_start_time).nanoseconds * 1e-9

    def _errors(self):
        """返回 (x, e_y, e_yaw)；位姿不可用返回 (None, None, None)。"""
        if self._dock_x is None:
            return None, None, None
        return (
            self._dock_x,
            self._dock_y,
            wrap_angle(self._dock_yaw - math.pi),
        )

    # ══════════════ 主循环 ══════════════
    def _tick(self):
        now = self.get_clock().now()
        dt = 0.0
        if self._last_tick_time is not None:
            dt = (now - self._last_tick_time).nanoseconds * 1e-9
        self._last_tick_time = now

        self._transitions(dt)
        mode = self._compute_mode()
        self._publish(mode)

    def _global_timeout_hit(self) -> bool:
        limit = float(self.get_parameter("max_docking_duration_sec").value)
        return limit > 0 and self._mission_elapsed() > limit

    def _transitions(self, dt: float):
        s = self._state
        p = self.get_parameter
        x, e_y, e_yaw = self._errors()

        # 全局总超时：坞内 -> ABORT_EXIT；坞外 -> IDLE 上报
        if s in CORRIDOR_STATES + OUTSIDE_STATES and self._global_timeout_hit():
            if s in CORRIDOR_STATES:
                self._abort_corridor("GLOBAL_TIMEOUT")
            else:
                self._abort_and_report("GLOBAL_TIMEOUT")
            return

        if s == DockState.ACQUIRE_TAG:
            if self._pose_source == SRC_VISION and self._tag_visible:
                self._acquire_frames += 1
                self._acquire_miss = 0
            else:
                self._acquire_miss += 1
                if self._acquire_miss > int(p("acquire_miss_tolerance").value):
                    self._acquire_frames = 0
            if self._acquire_frames >= int(p("tag_acquire_frames").value):
                self._enter(DockState.APPROACH_ENTRY)
            elif self._state_elapsed() > float(p("acquire_timeout_sec").value):
                self._abort_and_report("ACQUIRE_TIMEOUT")

        elif s == DockState.APPROACH_ENTRY:
            if self._pose_source == SRC_INVALID:
                self._enter(DockState.REACQUIRE_TAG)
            elif self._measurement_age > float(
                self._age_timeout()
            ) and self._pose_source != SRC_VISION:
                self._enter(DockState.REACQUIRE_TAG)
            elif x is not None and self._staging_reached(x, e_y):
                self._enter(DockState.ALIGN_ENTRY)
            elif self._state_elapsed() > float(p("approach_timeout_sec").value):
                self._abort_and_report("APPROACH_TIMEOUT")

        elif s == DockState.ALIGN_ENTRY:
            # 丢 Tag 宽限计时：VISION 清零，推算期累积（INVALID 立即弹）
            if self._pose_source == SRC_VISION:
                self._align_tag_loss = 0.0
            elif self._pose_source != SRC_INVALID:
                self._align_tag_loss += dt
            if self._pose_source == SRC_INVALID:
                self._enter(DockState.REACQUIRE_TAG)
            elif self._pose_source != SRC_VISION:
                # 视野边缘检测逐帧闪烁属常态，宽限期内靠推算继续对准
                if self._align_tag_loss > float(
                    p("align_tag_loss_grace_sec").value
                ):
                    self._enter(DockState.REACQUIRE_TAG)
            elif x is None:
                pass
            elif abs(e_y) > float(p("align_y_abort").value):
                # 差速船 v=0 修不了横向，退回弧线阶段
                self.get_logger().warn(
                    f"ALIGN 中 |e_y|={abs(e_y):.2f} 超限，退回 APPROACH_ENTRY"
                )
                self._enter(DockState.APPROACH_ENTRY)
            elif self._align_gate_ok(x, e_y, e_yaw):
                self._align_hold += dt
                if self._align_hold >= float(p("align_hold_sec").value):
                    self._enter(DockState.BACK_IN)
            else:
                self._align_hold = 0.0
                # y 卡死带 (align_y_tol, align_y_abort]：艏向已准但 y 超差，
                # ALIGN v=0 修不了横向只能干等到超时（2026-07-29 实测卡满 45s）。
                # 滞留超时确定性退回 APPROACH 弧线修 y，再到ALIGN复核
                yaw_tol = math.radians(float(p("align_yaw_tol_deg").value))
                if (
                    abs(e_yaw) <= yaw_tol
                    and abs(e_y) > float(p("align_y_tol").value)
                ):
                    self._align_y_stuck += dt
                    if self._align_y_stuck > float(
                        p("align_y_stuck_sec").value
                    ):
                        self.get_logger().warn(
                            f"ALIGN 艏向已准但 |e_y|={abs(e_y):.2f} 滞留"
                            f" {self._align_y_stuck:.1f}s，回 APPROACH 修 y"
                        )
                        self._enter(DockState.APPROACH_ENTRY)
                else:
                    self._align_y_stuck = 0.0
                if self._state_elapsed() > float(p("align_timeout_sec").value):
                    self._abort_and_report("ALIGN_TIMEOUT")

        elif s == DockState.BACK_IN:
            self._tick_back_in(dt, x, e_y, e_yaw)

        elif s == DockState.FINAL_DOCK:
            self._tick_final_dock(dt, x, e_y, e_yaw)

        elif s == DockState.REACQUIRE_TAG:
            if self._pose_source == SRC_VISION and self._tag_visible:
                self._reacquire_frames += 1
                self._reacquire_miss = 0
            else:
                self._reacquire_miss += 1
                if self._reacquire_miss > int(
                    p("reacquire_miss_tolerance").value
                ):
                    self._reacquire_frames = 0
            if self._reacquire_frames >= int(p("reacquire_frames").value):
                self._route_after_reacquire(x, e_y)
            elif self._state_elapsed() > float(p("reacquire_timeout_sec").value):
                self._abort_and_report("REACQUIRE_TIMEOUT")

        elif s == DockState.ABORT_EXIT:
            done = False
            if x is not None and self._pose_source != SRC_INVALID:
                done = x <= float(p("exit_complete_x").value)
            if done or self._state_elapsed() > float(
                p("abort_exit_timeout_sec").value
            ):
                self.get_logger().warn(
                    f"ABORT_EXIT 完成（pose={'有效' if self._pose_source != SRC_INVALID else '无效/超时'}），"
                    "回 IDLE 上报 needs_reapproach"
                )
                self._needs_reapproach = True
                self._enter(DockState.IDLE)

        elif s == DockState.UNDOCK_EXIT:
            done = False
            if x is not None and self._pose_source != SRC_INVALID:
                done = x <= float(p("undock_complete_x").value)
            if done:
                self._undock_success = True
                self._enter(DockState.UNDOCK_SETTLE)
            elif self._state_elapsed() > float(p("undock_timeout_sec").value):
                # 超时不报假成功：船未确认驶出，FAILED（JSON 映射 DOCK_ABORT）
                self._fail("UNDOCK_TIMEOUT")

        elif s == DockState.UNDOCK_SETTLE:
            if self._state_elapsed() > float(p("undock_settle_sec").value):
                self._enter(DockState.IDLE)

        # IDLE / DOCKED / FAILED：等待外部触发（start/cancel/undock）

    def _age_timeout(self) -> float:
        # 与估计器 tag_timeout 对齐的“视为丢失”阈值（稍宽，避免竞态）
        return 0.5

    def _staging_reached(self, x, e_y) -> bool:
        p = self.get_parameter
        return (
            abs(x - float(p("staging_x").value))
            <= float(p("approach_x_tol").value)
            and abs(e_y) <= float(p("approach_y_tol").value)
        )

    def _align_gate_ok(self, x, e_y, e_yaw) -> bool:
        p = self.get_parameter
        return (
            abs(e_y) < float(p("align_y_tol").value)
            and abs(e_yaw) < math.radians(float(p("align_yaw_tol_deg").value))
            and float(p("entry_window_min_x").value)
            <= x
            <= float(p("entry_window_max_x").value)
        )

    def _corridor_violation(self, e_y, e_yaw) -> bool:
        p = self.get_parameter
        return (
            abs(e_y) > float(p("back_in_gate2_y").value)
            or abs(e_yaw) > math.radians(float(p("back_in_gate2_yaw_deg").value))
        )

    def _tick_back_in(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        # Tag 丢失分级（measurement_age 为 inf 时直接进 ABORT_EXIT）
        if self._pose_source != SRC_VISION:
            age = self._measurement_age
            if age > float(p("back_in_tag_loss_search_sec").value):
                self._abort_corridor("TAG_LOST_IN_BACK_IN")
                return
        # 走廊违规（连续 N 周期）
        if x is not None and self._corridor_violation(e_y, e_yaw):
            self._violation_count += 1
            if self._violation_count >= int(p("violation_cycles").value):
                self._abort_corridor("CORRIDOR_VIOLATION")
                return
        else:
            self._violation_count = 0
        # 到 FINAL_DOCK 过渡位
        if x is not None and (
            abs(x - float(p("final_target_x").value))
            < float(p("final_dock_entry_dist").value)
        ):
            self._enter(DockState.FINAL_DOCK)
            return
        if self._state_elapsed() > float(p("back_in_timeout_sec").value):
            self._abort_corridor("BACK_IN_TIMEOUT")

    def _tick_final_dock(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        # 最后阶段不主动搜索：停车等待短时重识别，超时退出
        if self._pose_source != SRC_VISION and (
            self._measurement_age
            > float(p("final_dock_tag_loss_timeout_sec").value)
        ):
            self._abort_corridor("TAG_LOST_IN_FINAL_DOCK")
            return
        if x is not None and self._corridor_violation(e_y, e_yaw):
            self._violation_count += 1
            if self._violation_count >= int(p("violation_cycles").value):
                self._abort_corridor("CORRIDOR_VIOLATION")
                return
        else:
            self._violation_count = 0
        # 到位判据（无充电/接触传感器的临时版本）
        if x is not None and (
            abs(x - float(p("final_target_x").value))
            < float(p("docked_x_tol").value)
            and abs(e_y) < float(p("docked_y_tol").value)
            and abs(e_yaw) < math.radians(float(p("docked_yaw_tol_deg").value))
        ):
            self._docked_hold += dt
            if self._docked_hold >= float(p("docked_hold_sec").value):
                self._enter(DockState.DOCKED)
                return
        else:
            self._docked_hold = 0.0
        if self._state_elapsed() > float(p("final_dock_timeout_sec").value):
            self._abort_corridor("FINAL_DOCK_TIMEOUT")

    def _route_after_reacquire(self, x, e_y):
        """重捕获后按位置路由：入口窗口内且横向已达 ALIGN 通过门槛 -> ALIGN，
        否则 -> APPROACH。y 必须用 align_y_tol 而非 align_y_abort：
        (align_y_tol, align_y_abort] 区间内 ALIGN 无法通过（原地转修不了横向），
        只能等超时（2026-07-29 实测 y=0.29 卡死带）。"""
        p = self.get_parameter
        if (
            x is not None
            and float(p("entry_window_min_x").value)
            <= x
            <= float(p("entry_window_max_x").value)
            and abs(e_y) <= float(p("align_y_tol").value)
        ):
            self._enter(DockState.ALIGN_ENTRY)
        else:
            self._enter(DockState.APPROACH_ENTRY)

    # ══════════════ target_mode 映射 ══════════════
    def _compute_mode(self) -> str:
        s = self._state
        if s == DockState.ACQUIRE_TAG:
            return MODE_HOLD if self._tag_visible else MODE_SEARCH
        if s == DockState.APPROACH_ENTRY:
            return MODE_APPROACH
        if s == DockState.ALIGN_ENTRY:
            return MODE_ALIGN
        if s == DockState.BACK_IN:
            if self._pose_source != SRC_VISION:
                if self._measurement_age <= float(
                    self.get_parameter("back_in_tag_loss_hold_sec").value
                ):
                    return MODE_HOLD  # 0~0.5s：停车等待
                return MODE_SEARCH_LIMITED  # 0.5~2s：小角度搜索
            return MODE_BACK_IN
        if s == DockState.FINAL_DOCK:
            if self._pose_source != SRC_VISION:
                return MODE_HOLD  # 最后阶段只停车等待
            return MODE_FINAL_DOCK
        if s == DockState.REACQUIRE_TAG:
            # 看到 Tag 就停车集帧（与 ACQUIRE 同构）：边转边集帧锚点不收敛，
            # 且集帧通过瞬间的朝向会把 Tag 甩出视野造成 ALIGN<->REACQUIRE 互弹
            return MODE_HOLD if self._tag_visible else MODE_SEARCH
        if s == DockState.ABORT_EXIT:
            return MODE_EXIT_FORWARD
        if s == DockState.UNDOCK_EXIT:
            return MODE_UNDOCK_FORWARD
        if s == DockState.DOCKED:
            return MODE_DOCKED_HOLD
        return MODE_HOLD  # IDLE / FAILED / UNDOCK_SETTLE

    # ══════════════ 发布 ══════════════
    def _publish(self, mode: str):
        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)

        mode_msg = String()
        mode_msg.data = mode
        self._target_mode_pub.publish(mode_msg)

        x, e_y, e_yaw = self._errors()
        # dock_mission 契约：
        #   MONITOR_DOCK   只看 success / needs_reapproach(+abort_reason)
        #   MONITOR_UNDOCK 看 undock_success / state=="DOCK_ABORT"(+abort_reason)
        # FAILED 必须：
        #   - state 映射为 "DOCK_ABORT"（出泊失败能被上层捕获）
        #   - needs_reapproach 强制 true（入泊失败让上层走重试->人工接管流水线，
        #     否则 MONITOR_DOCK 会挂死）
        failed = self._state == DockState.FAILED
        status = {
            "state": "DOCK_ABORT" if failed else self._state,
            "v2_state": self._state,
            "target_mode": mode,
            "success": self._success,
            "undock_success": self._undock_success,
            "abort_reason": self._abort_reason,
            "needs_reapproach": self._needs_reapproach or failed,
            "needs_manual_takeover": self._needs_manual_takeover,
            "pose_valid": self._pose_source != SRC_INVALID,
            "pose_source": self._pose_source,
            "tag_visible": self._tag_visible,
            "tag_age_sec": (
                round(self._measurement_age, 2)
                if math.isfinite(self._measurement_age)
                else None
            ),
            "mission_elapsed_sec": round(self._mission_elapsed(), 1),
        }
        if x is not None:
            status["dock_x"] = round(x, 3)
            status["dock_y"] = round(e_y, 3)
            status["e_yaw_deg"] = round(math.degrees(e_yaw), 2)
        status_msg = String()
        status_msg.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DockingFsmV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
