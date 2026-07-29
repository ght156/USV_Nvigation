#!/usr/bin/env python3
"""docking_safety_v2 — 归港安全监督（V2）。

检查项（check_rate 周期执行）：
  1. 横向误差 |e_y| > max_lateral_error        -> abort_request("LATERAL_ERROR")
  2. 艏向误差 |e_yaw| > max_yaw_error_deg       -> abort_request("YAW_ERROR")
  3. odom 推算超时（正常窗口 odom_prediction_timeout；
     ABORT_EXIT/UNDOCK 期间放宽到 abort_exit_odom_timeout；
     ACQUIRE_TAG/REACQUIRE_TAG 搜索状态豁免，由 FSM 超时兜底）
                                                -> abort_request("ODOM_TIMEOUT")
  4. estimator 话题断流 > pose_topic_timeout_sec -> safety_stop + abort_request("TOPIC_TIMEOUT:POSE")
  5. FSM 话题断流 > state_topic_timeout_sec      -> safety_stop（FSM 已死，无法走异常流）
  6. 任务总时长 > max_docking_duration_sec       -> abort_request("GLOBAL_TIMEOUT")

适用规则：
  - 误差类检查（1/2）仅在运动状态且位姿有效时启用；IDLE/DOCKED/FAILED 不查；
    ABORT_EXIT/UNDOCK 期间不查（已在撤离通道上）。
  - 连续 violation_cycles 周期超限才触发（防抖）；触发后保持请求直到条件消失。
  - safety_stop=true 期间控制器立即输出零速（绕过斜坡）。

设计约束：不直接发布 cmd_vel，只发信号。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, String

# 误差检查适用的状态：仅坞内走廊（BACK_IN/FINAL_DOCK）。
# APPROACH/ACQUIRE/REACQUIRE 在开阔水域，横向偏差由控制器自行收敛；
# ALIGN 的职责就是从大艏向误差开始修正，初始误差是正常输入而非故障，
# 过早检查会误中止（2026-07-29 偏轴线入场实测两次教训）
ERROR_CHECK_STATES = (
    "BACK_IN",
    "FINAL_DOCK",
)
# 撤离通道状态：放宽 odom 窗口、不做误差中止
EXIT_STATES = ("ABORT_EXIT", "UNDOCK_EXIT", "UNDOCK_SETTLE")
# 搜索状态：本质就是"靠 odom 推算找 Tag"，豁免 odom 推算时长检查
# （由 FSM 的 acquire/reacquire 超时兜底，2026-07-29 实测：否则 REACQUIRE 活不过 3s）
SEARCH_STATES = ("ACQUIRE_TAG", "REACQUIRE_TAG")
# 完全静默状态
QUIET_STATES = ("IDLE", "DOCKED", "FAILED")

SRC_INVALID = "INVALID"


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


class DockingSafetyV2(Node):
    def __init__(self):
        super().__init__("docking_safety_v2")

        # ── 输入话题 ──
        self.declare_parameter("dock_pose_topic", "/docking_v2/dock_pose")
        self.declare_parameter("tag_visible_topic", "/docking_v2/tag_visible")
        self.declare_parameter("pose_source_topic", "/docking_v2/pose_source")
        self.declare_parameter("measurement_age_topic", "/docking_v2/measurement_age")
        self.declare_parameter("state_topic", "/docking_v2/state")

        # ── 输出话题 ──
        self.declare_parameter("safety_stop_topic", "/docking_v2/safety_stop")
        self.declare_parameter("abort_request_topic", "/docking_v2/abort_request")

        # ── 运行 ──
        self.declare_parameter("check_rate", 20.0)

        # ── 阈值 ──
        self.declare_parameter("max_lateral_error", 0.50)
        self.declare_parameter("max_yaw_error_deg", 15.0)
        self.declare_parameter("odom_prediction_timeout", 3.00)
        self.declare_parameter("abort_exit_odom_timeout", 60.0)
        self.declare_parameter("pose_topic_timeout_sec", 1.0)
        self.declare_parameter("state_topic_timeout_sec", 1.0)
        self.declare_parameter("max_docking_duration_sec", 150.0)
        self.declare_parameter("violation_cycles", 10)

        qos = QoSProfile(depth=10)

        self._safety_stop_pub = self.create_publisher(
            Bool, self.get_parameter("safety_stop_topic").value, qos
        )
        self._abort_request_pub = self.create_publisher(
            String, self.get_parameter("abort_request_topic").value, qos
        )

        self.create_subscription(
            PoseStamped,
            self.get_parameter("dock_pose_topic").value,
            self._dock_pose_cb,
            qos,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("tag_visible_topic").value,
            self._tag_visible_cb,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("pose_source_topic").value,
            self._pose_source_cb,
            qos,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("measurement_age_topic").value,
            self._measurement_age_cb,
            qos,
        )
        self.create_subscription(
            String, self.get_parameter("state_topic").value, self._state_cb, qos
        )

        self._pose_x = None
        self._pose_y = None
        self._pose_yaw = None
        self._tag_visible = False
        self._pose_source = SRC_INVALID
        self._measurement_age = float("inf")
        self._state = "IDLE"

        # 话题活性（节点死亡检测）
        now = self.get_clock().now()
        self._last_pose_msg_time = now
        self._last_state_msg_time = now
        self._mission_start_time = None
        self._last_state = "IDLE"

        # 防抖计数
        self._lateral_count = 0
        self._yaw_count = 0
        self._odom_count = 0
        self._global_count = 0

        rate = float(self.get_parameter("check_rate").value)
        self.create_timer(1.0 / rate, self._check)

        self.get_logger().info("docking_safety_v2 已启动（6 项检查）")

    # ── 输入回调 ──
    def _dock_pose_cb(self, msg: PoseStamped):
        self._pose_x = msg.pose.position.x
        self._pose_y = msg.pose.position.y
        self._pose_yaw = yaw_from_quat(
            msg.pose.orientation.z, msg.pose.orientation.w
        )
        self._last_pose_msg_time = self.get_clock().now()

    def _tag_visible_cb(self, msg: Bool):
        self._tag_visible = bool(msg.data)

    def _pose_source_cb(self, msg: String):
        self._pose_source = msg.data

    def _measurement_age_cb(self, msg: Float32):
        self._measurement_age = float(msg.data)

    def _state_cb(self, msg: String):
        self._state = msg.data
        self._last_state_msg_time = self.get_clock().now()

    # ── 主循环 ──
    def _check(self):
        p = self.get_parameter
        now = self.get_clock().now()
        state = self._state

        # 任务计时：离开 IDLE/DOCKED 视为任务开始
        if state != self._last_state:
            if state not in QUIET_STATES and self._last_state in QUIET_STATES:
                self._mission_start_time = now
            self._last_state = state

        stop = False
        abort_reason = ""

        # ── 检查4/5：话题断流（任何非静默状态都查；静默态节点死亡无所谓）──
        pose_silent = (
            now - self._last_pose_msg_time
        ).nanoseconds * 1e-9 > float(p("pose_topic_timeout_sec").value)
        fsm_silent = (
            now - self._last_state_msg_time
        ).nanoseconds * 1e-9 > float(p("state_topic_timeout_sec").value)

        if state not in QUIET_STATES:
            if fsm_silent:
                # FSM 死亡：无法走异常流，只能力保停车
                stop = True
                self._warn_throttle("FSM 话题断流 -> safety_stop")
            elif pose_silent:
                stop = True
                abort_reason = "TOPIC_TIMEOUT:POSE"
                self._warn_throttle("位姿话题断流 -> safety_stop + abort")

        # ── 检查6：全局任务时长 ──
        if (
            not stop
            and not abort_reason
            and state not in QUIET_STATES
            and self._mission_start_time is not None
        ):
            elapsed = (now - self._mission_start_time).nanoseconds * 1e-9
            if elapsed > float(p("max_docking_duration_sec").value):
                self._global_count += 1
                if self._global_count >= int(p("violation_cycles").value):
                    abort_reason = "GLOBAL_TIMEOUT"
            else:
                self._global_count = 0

        # ── 检查3：odom 推算超时（搜索状态豁免，见 SEARCH_STATES 注释）──
        if (
            not stop
            and not abort_reason
            and state not in QUIET_STATES
            and state not in SEARCH_STATES
        ):
            if state in EXIT_STATES:
                odom_limit = float(p("abort_exit_odom_timeout").value)
            else:
                odom_limit = float(p("odom_prediction_timeout").value)
            if self._pose_source == SRC_INVALID or (
                self._pose_source != "VISION"
                and self._measurement_age > odom_limit
            ):
                # 撤离通道里 INVALID 同理不可久留
                if self._pose_source == SRC_INVALID and state in EXIT_STATES:
                    pass  # 撤离中位姿丢失：FSM 用超时兜底驶出，不另报
                else:
                    self._odom_count += 1
                    if self._odom_count >= int(p("violation_cycles").value):
                        abort_reason = "ODOM_TIMEOUT"
            else:
                self._odom_count = 0

        # ── 检查1/2：误差类（运动状态 + 位姿有效；撤离通道不查）──
        if (
            not abort_reason
            and state in ERROR_CHECK_STATES
            and self._pose_source != SRC_INVALID
            and self._pose_x is not None
        ):
            e_y = self._pose_y
            e_yaw = abs(wrap_angle(self._pose_yaw - math.pi))

            if abs(e_y) > float(p("max_lateral_error").value):
                self._lateral_count += 1
                if self._lateral_count >= int(p("violation_cycles").value):
                    abort_reason = "LATERAL_ERROR"
            else:
                self._lateral_count = 0

            if not abort_reason:
                if e_yaw > math.radians(float(p("max_yaw_error_deg").value)):
                    self._yaw_count += 1
                    if self._yaw_count >= int(p("violation_cycles").value):
                        abort_reason = "YAW_ERROR"
                else:
                    self._yaw_count = 0
        else:
            self._lateral_count = 0
            self._yaw_count = 0

        stop_msg = Bool()
        stop_msg.data = stop
        self._safety_stop_pub.publish(stop_msg)

        abort_msg = String()
        abort_msg.data = abort_reason
        self._abort_request_pub.publish(abort_msg)

    def _warn_throttle(self, text: str):
        self.get_logger().warn(text, throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DockingSafetyV2()
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
