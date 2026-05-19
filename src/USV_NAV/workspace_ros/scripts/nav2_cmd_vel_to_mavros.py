#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# nav2_cmd_vel_to_mavros.py
#
# 功能：
#   将 Nav2 速度 Twist 转为 MAVROS /mavros/setpoint_raw/local (PositionTarget)。
#   Nav2 bringup：controller → /cmd_vel_nav；velocity_smoother → /cmd_vel。
#   默认 cmd_vel_src=/cmd_vel_nav（绕过 smoother）；可设为 /cmd_vel 用平滑后指令。
#
# 适用对象：
#   差速无人船 / 双推进器无人船。
#   船体不能倒退，不能横向平移。
#
# 固件自定义协议：
#   本船固件虽然使用 /mavros/setpoint_raw/local 和 FRAME_LOCAL_NED，
#   但对 PositionTarget.velocity.y 做了自定义解释：
#
#       velocity.x  ← cmd_vel.linear.x     前向线速度，单位 m/s
#       velocity.y  ← cmd_vel.angular.z    偏航角速度 yaw rate，单位 rad/s
#       velocity.z  = 0.0                  不使用
#
#   注意：
#     这里的 velocity.y 不是标准 NED 坐标系下的 y 方向线速度，
#     而是船厂固件复用为“偏航角速度输入”。
#     因此本节点不使用 PositionTarget.yaw_rate 字段，并在 type_mask 中
#     设置 IGNORE_YAW_RATE。
#
# 安全机制：
#   1. forbid_reverse=true 时，负 linear.x 会被钳制为 0，禁止倒船。
#   2. cmd_vel 超时后持续发布零速。
#   3. estop_topic 收到 True 后持续发布零速。
#   4. require_offboard_for_motion=true 时，未进入 OFFBOARD 或 MAVROS 未连接，
#      持续发布零速，不转发运动指令。
#   5. 节点退出前连续发送多帧零速，降低飞控/MAVROS 保持最后一帧非零命令的风险。
#
# 输入（参数 cmd_vel_src，默认 /cmd_vel_nav）：
#   geometry_msgs/msg/Twist
#
# 输出：
#   /mavros/setpoint_raw/local           mavros_msgs/msg/PositionTarget
#
# 关键映射：
#   msg.header.stamp = 当前 ROS 时间，避免 MAVROS / 飞控按旧消息丢弃
#   msg.velocity.x  = cmd.linear.x
#   msg.velocity.y  = cmd.angular.z      直接映射，不乘增益
#   msg.velocity.z  = 0.0
#   msg.yaw_rate    = 0.0，并被 type_mask 忽略
#
# 依赖：
#   rclpy, geometry_msgs, mavros_msgs, std_msgs
# -----------------------------------------------------------------------------

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool


# PX4/MAVROS 有些环境会把 OFFBOARD 显示成 CMODE(393216)。
# 0x60000 == 393216，这里只保留十六进制写法，便于和 custom mode 位值对应。
_DEFAULT_OFFBOARD_CMODES = frozenset({0x60000})


def _as_bool(value) -> bool:
    """兼容 bool / int / string 形式的参数值。"""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _parse_cmode_allowlist(text: str) -> frozenset[int]:
    """解析类似 '393216,0x60000' 的 OFFBOARD custom mode 白名单。"""
    text = (text or "").strip()
    if not text:
        return _DEFAULT_OFFBOARD_CMODES

    values: set[int] = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            values.add(int(part, 0))
        except ValueError:
            continue

    return frozenset(values) if values else _DEFAULT_OFFBOARD_CMODES


def _flight_mode_implies_offboard(
    mode_str: str,
    cmode_allow: frozenset[int],
) -> bool:
    """判断 mavros_msgs/State.mode 是否代表 OFFBOARD。"""
    raw = (mode_str or "").strip()
    if not raw:
        return False

    upper = raw.upper()

    if upper == "OFFBOARD" or "OFFBOARD" in upper:
        return True

    if upper.startswith("CMODE(") and upper.endswith(")"):
        inner = raw[6:-1].strip()
        try:
            code = int(inner, 0)
        except ValueError:
            return False
        return code in cmode_allow

    return False


class Nav2CmdVelToMavrosRaw(Node):
    """
    Nav2 cmd_vel 到 MAVROS PositionTarget 的转换节点。

    固件自定义直接映射：
        Twist.linear.x   -> PositionTarget.velocity.x
        Twist.angular.z  -> PositionTarget.velocity.y

    标准 yaw_rate 字段不使用：
        PositionTarget.yaw_rate = 0.0
        type_mask 包含 IGNORE_YAW_RATE
    """

    def __init__(self) -> None:
        super().__init__("nav2_cmd_vel_to_mavros_raw")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        # 话题参数
        self.declare_parameter("cmd_vel_src", "/cmd_vel_nav")
        self.declare_parameter("mavros_raw_dst", "/mavros/setpoint_raw/local")
        self.declare_parameter("mavros_state_src", "/mavros/state")
        self.declare_parameter("estop_topic", "")

        # 前向速度 linear.x 限幅
        self.declare_parameter("min_linear_x", 0.0)
        self.declare_parameter("max_linear_x", 1.2)

        # angular.z 限幅；限幅后的 angular.z 会直接写入 velocity.y
        self.declare_parameter("min_angular_z", -1.2)
        self.declare_parameter("max_angular_z", 1.2)

        # 行为和安全参数
        self.declare_parameter("forbid_reverse", True)
        self.declare_parameter("linear_deadband", 0.03)
        self.declare_parameter("angular_deadband", 0.01)
        self.declare_parameter("cmd_timeout_sec", 0.3)
        self.declare_parameter("publish_hz", 20.0)

        # 退出前连续发布零速
        self.declare_parameter("shutdown_zero_count", 10)
        self.declare_parameter("shutdown_zero_dt_sec", 0.05)

        # Nav2 cmd_vel 常见 QoS 是 best effort；这里默认设为 True，避免订阅不到。
        self.declare_parameter("cmd_vel_sub_qos_best_effort", True)

        # OFFBOARD 门控
        self.declare_parameter("require_offboard_for_motion", True)
        self.declare_parameter("offboard_cmode_allowlist", "")

        allow_text = (
            self.get_parameter("offboard_cmode_allowlist")
            .get_parameter_value()
            .string_value
        )
        self._offboard_cmode_allow = _parse_cmode_allowlist(allow_text)

        # 仅启用 velocity.x 和 velocity.y。
        # velocity.y 在本船固件中表示偏航角速度 yaw rate。
        # 标准 yaw_rate 字段明确忽略。
        self._type_mask_vx_vy = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            # 不设置 IGNORE_VX -> 使用 velocity.x
            # 不设置 IGNORE_VY -> 使用 velocity.y，固件自定义为 yaw rate
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )

        self._lock = threading.Lock()
        self._shutting_down = False

        self._cmd_linear_x = 0.0
        self._cmd_angular_z = 0.0
        self._have_cmd = False
        self._last_cmd_sec: float | None = None

        self._estop = False
        self._timed_out_log = False

        self._px4_offboard = False
        self._mavros_connected = False
        self._last_mavros_mode_str = ""
        self._offboard_warn_time: float | None = None

        self._timer = None

        self._warned_bad_linear_limits = False
        self._warned_bad_angular_limits = False
        self._warned_bad_timeout = False

        st_src = (
            self.get_parameter("mavros_state_src")
            .get_parameter_value()
            .string_value
            .strip()
        )

        req_ob = bool(
            self.get_parameter("require_offboard_for_motion")
            .get_parameter_value()
            .bool_value
        )
        self._gate_offboard = bool(req_ob and st_src)

        cmd_src = (
            self.get_parameter("cmd_vel_src")
            .get_parameter_value()
            .string_value
            .strip()
        )
        raw_dst = (
            self.get_parameter("mavros_raw_dst")
            .get_parameter_value()
            .string_value
            .strip()
        )

        self._pub = self.create_publisher(PositionTarget, raw_dst, 10)

        sub_qos = self._cmd_vel_sub_qos()
        self.create_subscription(Twist, cmd_src, self._on_cmd_vel, sub_qos)

        kind = (
            "best_effort"
            if sub_qos.reliability == ReliabilityPolicy.BEST_EFFORT
            else "reliable"
        )
        self.get_logger().info(f"Subscribe {cmd_src} ({kind}) -> {raw_dst}")

        estop_topic = (
            self.get_parameter("estop_topic")
            .get_parameter_value()
            .string_value
            .strip()
        )
        if estop_topic:
            self.create_subscription(Bool, estop_topic, self._on_estop, 10)
            self.get_logger().info(f"Subscribe estop topic: {estop_topic}")

        if st_src:
            self.create_subscription(State, st_src, self._on_mavros_state, 10)
            self.get_logger().info(f"Subscribe MAVROS state: {st_src}")

        hz = float(
            self.get_parameter("publish_hz")
            .get_parameter_value()
            .double_value
        )
        if hz <= 0.0:
            self.get_logger().warning("publish_hz <= 0, fallback to 20.0 Hz")
            hz = 20.0

        self._timer = self.create_timer(1.0 / hz, self._on_timer)

    def _cmd_vel_sub_qos(self) -> QoSProfile:
        best_effort = bool(
            self.get_parameter("cmd_vel_sub_qos_best_effort")
            .get_parameter_value()
            .bool_value
        )

        return QoSProfile(
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if best_effort
                else ReliabilityPolicy.RELIABLE
            ),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _clamp_checked(
        self,
        value: float,
        low: float,
        high: float,
        name: str,
    ) -> float:
        """
        将 value 限制在 [low, high]，并在配置反向时给出 warning。

        为了不中断运行，检测到 low > high 时仍会交换上下限。
        """
        if low > high:
            if name == "linear_x" and not self._warned_bad_linear_limits:
                self.get_logger().warning(
                    "min_linear_x > max_linear_x, swapped internally; "
                    "please check parameter configuration"
                )
                self._warned_bad_linear_limits = True

            if name == "angular_z" and not self._warned_bad_angular_limits:
                self.get_logger().warning(
                    "min_angular_z > max_angular_z, swapped internally; "
                    "please check parameter configuration"
                )
                self._warned_bad_angular_limits = True

            low, high = high, low

        return max(low, min(high, value))

    def _on_estop(self, msg: Bool) -> None:
        if self._shutting_down:
            return

        with self._lock:
            self._estop = _as_bool(msg.data)

    def _on_mavros_state(self, msg: State) -> None:
        if self._shutting_down:
            return

        with self._lock:
            self._mavros_connected = bool(msg.connected)
            self._last_mavros_mode_str = str(msg.mode or "")
            self._px4_offboard = bool(
                msg.connected
                and _flight_mode_implies_offboard(
                    msg.mode,
                    self._offboard_cmode_allow,
                )
            )

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._shutting_down:
            return

        with self._lock:
            self._cmd_linear_x = float(msg.linear.x)
            self._cmd_angular_z = float(msg.angular.z)
            self._have_cmd = True
            self._last_cmd_sec = self._now_sec()
            self._timed_out_log = False

    def _sanitize_and_map(
        self,
        linear_x: float,
        angular_z: float,
    ) -> tuple[float, float]:
        """
        限幅、死区、禁止倒船，并完成直接映射。

        Returns:
            (vx, vy_custom):
                vx        -> PositionTarget.velocity.x，前向速度
                vy_custom -> PositionTarget.velocity.y，固件自定义为偏航角速度

        注意：
            这里不乘任何 yaw_rate_gain。
            angular_z 限幅、死区处理后直接写入 velocity.y。
        """
        p = self.get_parameter

        min_lx = p("min_linear_x").get_parameter_value().double_value
        max_lx = p("max_linear_x").get_parameter_value().double_value

        min_az = p("min_angular_z").get_parameter_value().double_value
        max_az = p("max_angular_z").get_parameter_value().double_value

        forbid_reverse = bool(
            p("forbid_reverse").get_parameter_value().bool_value
        )

        if forbid_reverse:
            linear_x = max(0.0, linear_x)
            # 防止误把 min_linear_x 配成正数后，Nav2 发 0 时仍被抬成前进速度。
            min_lx = max(0.0, min_lx)

        linear_x = self._clamp_checked(linear_x, min_lx, max_lx, "linear_x")
        angular_z = self._clamp_checked(angular_z, min_az, max_az, "angular_z")

        linear_deadband = abs(
            p("linear_deadband").get_parameter_value().double_value
        )
        angular_deadband = abs(
            p("angular_deadband").get_parameter_value().double_value
        )

        if abs(linear_x) < linear_deadband:
            linear_x = 0.0

        if abs(angular_z) < angular_deadband:
            angular_z = 0.0

        # 严格直接映射：cmd_vel.angular.z -> PositionTarget.velocity.y
        return linear_x, angular_z

    def _make_position_target(
        self,
        vx: float,
        vy_custom_yaw_rate: float,
    ) -> PositionTarget:
        """
        构造 PositionTarget。

        注意：
            velocity.y 在本船固件中不是侧向速度，而是偏航角速度输入。
        """
        msg = PositionTarget()

        # MAVROS / 飞控侧通常会根据 header.stamp 判断 setpoint 新鲜度。
        # frame_id 主要用于调试和工具显示；coordinate_frame 才是 MAVLink 控制坐标系。
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = self._type_mask_vx_vy

        msg.velocity.x = float(vx)
        msg.velocity.y = float(vy_custom_yaw_rate)
        msg.velocity.z = 0.0

        msg.acceleration_or_force.x = 0.0
        msg.acceleration_or_force.y = 0.0
        msg.acceleration_or_force.z = 0.0

        msg.yaw = 0.0
        msg.yaw_rate = 0.0

        return msg

    def _publish_zero(self) -> None:
        self._pub.publish(self._make_position_target(0.0, 0.0))

    def _on_timer(self) -> None:
        if self._shutting_down:
            return

        with self._lock:
            estop = self._estop
            have_cmd = self._have_cmd
            last_sec = self._last_cmd_sec
            lx_in = self._cmd_linear_x
            az_in = self._cmd_angular_z
            offboard = self._px4_offboard
            connected = self._mavros_connected
            mode_str = self._last_mavros_mode_str

        now = self._now_sec()

        if estop:
            self._publish_zero()
            return

        timeout = float(
            self.get_parameter("cmd_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        if timeout <= 0.0:
            if not self._warned_bad_timeout:
                self.get_logger().warning(
                    "cmd_timeout_sec <= 0, fallback to 0.3 seconds"
                )
                self._warned_bad_timeout = True
            timeout = 0.3

        stale = (
            not have_cmd
            or last_sec is None
            or (now - last_sec) > timeout
        )

        if stale:
            self._publish_zero()

            log_now = False
            with self._lock:
                if not self._timed_out_log:
                    self._timed_out_log = True
                    log_now = True

            if log_now:
                self.get_logger().warning(
                    f"cmd_vel timeout > {timeout:.3f}s, publish zero"
                )
            return

        with self._lock:
            self._timed_out_log = False

        if self._gate_offboard and (not connected or not offboard):
            self._publish_zero()

            if lx_in != 0.0 or az_in != 0.0:
                log_now = False
                with self._lock:
                    if (
                        self._offboard_warn_time is None
                        or (now - self._offboard_warn_time) >= 5.0
                    ):
                        self._offboard_warn_time = now
                        log_now = True

                if log_now:
                    self.get_logger().warning(
                        "OFFBOARD not active, block motion and publish zero "
                        f"(connected={connected}, "
                        f"offboard={offboard}, "
                        f"mode={mode_str}); "
                        "this can be normal during startup"
                    )
            return

        vx, vy_custom_yaw_rate = self._sanitize_and_map(lx_in, az_in)
        self._pub.publish(self._make_position_target(vx, vy_custom_yaw_rate))

    def publish_stop(self) -> None:
        """
        节点退出前连续发布多帧零速。

        只发一帧零速可能因为 DDS/MAVROS/节点销毁时序而丢失。
        """
        self._shutting_down = True

        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:
                pass

        count = int(
            self.get_parameter("shutdown_zero_count")
            .get_parameter_value()
            .integer_value
        )
        dt = float(
            self.get_parameter("shutdown_zero_dt_sec")
            .get_parameter_value()
            .double_value
        )

        count = max(1, count)
        dt = max(0.0, dt)

        for _ in range(count):
            self._publish_zero()
            try:
                rclpy.spin_once(self, timeout_sec=0.0)
            except Exception as exc:
                self.get_logger().debug(
                    f"spin_once during shutdown ignored: {exc}"
                )
            if dt > 0.0:
                time.sleep(dt)

        self.get_logger().info("zero velocity commands published before shutdown")


def main(args=None) -> None:
    rclpy.init(args=args)

    node = Nav2CmdVelToMavrosRaw()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
