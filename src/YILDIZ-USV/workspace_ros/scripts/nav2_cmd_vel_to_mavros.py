#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# Nav2 controller raw /cmd_vel_nav → MAVROS /mavros/setpoint_velocity/cmd_vel_unstamped (Twist).
# 绕过 velocity_smoother，直接取 controller_server 输出，由本节点自行限幅/死区/超时。
# 与仿真 converter.py 订阅同一话题 /cmd_vel_nav，控制链一致。
# - Surge + yaw only (差速船常用；无 lateral)。PX4 OFFBOARD 侧再做闭环与推进分配。
# - 可选「只前进、禁止原地转向」：forbid_reverse + min_surge_for_turn（见参数）。
# - Nav2 在恢复 / RotateToGoal 等情况下仍可能短暂给出 linear.x<0 或低速大角速度；勿假定恒非负。
# - Saturation, deadband, optional estop; timer republish for PX4; optional OFFBOARD gating.
# ----------------------------------------------------------------------------------------------- #

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from std_msgs.msg import Bool


def _as_bool(val) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).strip().lower() in ('true', '1', 'yes', 'on')


_DEFAULT_OFFBOARD_CMODES = frozenset({393216, 0x60000})


def _parse_cmode_allowlist(s: str) -> frozenset[int]:
    """Comma-separated decimal or 0x hex integers; empty uses defaults."""
    s = (s or '').strip()
    if not s:
        return _DEFAULT_OFFBOARD_CMODES
    out: set[int] = set()
    for part in s.split(','):
        part = part.strip()
        if not part:
            continue
        try:
            out.add(int(part, 0))
        except ValueError:
            continue
    return frozenset(out) if out else _DEFAULT_OFFBOARD_CMODES


def _flight_mode_implies_offboard(mode_str: str, cmode_allow: frozenset[int]) -> bool:
    """MAVROS sometimes reports 'OFFBOARD', sometimes 'CMODE(<px4_custom>)' for PX4."""
    raw = (mode_str or '').strip()
    if not raw:
        return False
    up = raw.upper()
    if up == 'OFFBOARD' or 'OFFBOARD' in up:
        return True
    if up.startswith('CMODE(') and up.endswith(')'):
        inner = raw[6:-1].strip()
        try:
            code = int(inner, 0)
        except ValueError:
            return False
        return code in cmode_allow
    return False


class Nav2CmdVelToMavros(Node):
    def __init__(self) -> None:
        super().__init__('nav2_cmd_vel_to_mavros')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.declare_parameter('cmd_vel_src', '/cmd_vel_nav')
        self.declare_parameter('mavros_cmd_vel_dst', '/mavros/setpoint_velocity/cmd_vel_unstamped')
        self.declare_parameter('min_linear_x', 0.0)
        self.declare_parameter('max_linear_x', 0.2)
        self.declare_parameter('min_angular_z', -0.15)
        self.declare_parameter('max_angular_z', 0.15)
        self.declare_parameter('forbid_reverse', True)
        self.declare_parameter('min_surge_for_turn', 0.05)
        self.declare_parameter('linear_deadband', 0.03)
        self.declare_parameter('angular_deadband', 0.03)
        self.declare_parameter('cmd_timeout_sec', 0.3)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('estop_topic', '')
        self.declare_parameter('cmd_vel_sub_qos_best_effort', False)
        self.declare_parameter('mavros_state_src', '/mavros/state')
        self.declare_parameter('require_offboard_for_motion', True)
        self.declare_parameter('offboard_cmode_allowlist', '')

        allow_s = self.get_parameter('offboard_cmode_allowlist').get_parameter_value().string_value
        self._offboard_cmode_allow = _parse_cmode_allowlist(allow_s)
        self.get_logger().info(
            'OFFBOARD detection: name OFFBOARD or CMODE in {%s}'
            % ', '.join(str(x) for x in sorted(self._offboard_cmode_allow))
        )

        self._lock = threading.Lock()
        self._cmd_linear_x = 0.0
        self._cmd_angular_z = 0.0
        self._have_cmd = False
        self._last_cmd_sec: float | None = None
        self._estop = False
        self._timed_out_log = False
        self._px4_offboard = False
        self._mavros_connected = False
        self._offboard_warn_time: float | None = None
        self._last_mavros_mode_str = ''

        st_src = self.get_parameter('mavros_state_src').get_parameter_value().string_value.strip()
        req_ob_raw = _as_bool(
            self.get_parameter('require_offboard_for_motion').get_parameter_value().bool_value
        )
        if req_ob_raw and not st_src:
            self.get_logger().error(
                'require_offboard_for_motion is true but mavros_state_src is empty; '
                'OFFBOARD gating disabled (set mavros_state_src or turn gate off).'
            )
        self._gate_offboard = bool(req_ob_raw and st_src)

        self._validate_limit_params()
        self._log_kinematics_hint()

        cmd_src = self.get_parameter('cmd_vel_src').get_parameter_value().string_value
        cmd_dst = self.get_parameter('mavros_cmd_vel_dst').get_parameter_value().string_value
        self._pub = self.create_publisher(Twist, cmd_dst, 10)

        sub_qos = self._cmd_vel_sub_qos()
        self.create_subscription(Twist, cmd_src, self._on_cmd_vel, sub_qos)
        kind = 'best_effort' if sub_qos.reliability == ReliabilityPolicy.BEST_EFFORT else 'reliable'
        self.get_logger().info(
            'Subscribe %s (%s), publish %s' % (cmd_src, kind, cmd_dst)
        )

        estop = self.get_parameter('estop_topic').get_parameter_value().string_value.strip()
        if estop:
            self.create_subscription(Bool, estop, self._on_estop, 10)
            self.get_logger().info('E-stop on topic %s (data=true → zero setpoint)' % estop)

        if st_src:
            self.create_subscription(State, st_src, self._on_mavros_state, 10)
            self.get_logger().info(
                'MAVROS state %s (require_offboard_for_motion=%s, gate_active=%s)'
                % (st_src, req_ob_raw, self._gate_offboard)
            )

        hz = float(self.get_parameter('publish_hz').get_parameter_value().double_value)
        if hz <= 0.0:
            hz = 20.0
        if hz > 200.0:
            self.get_logger().warning('publish_hz capped at 200 (was %.1f)' % hz)
            hz = 200.0
        self.create_timer(1.0 / hz, self._on_timer)

    def _cmd_vel_sub_qos(self) -> QoSProfile:
        if _as_bool(self.get_parameter('cmd_vel_sub_qos_best_effort').get_parameter_value().bool_value):
            return QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _validate_limit_params(self) -> None:
        p = self.get_parameter
        min_lx = p('min_linear_x').get_parameter_value().double_value
        max_lx = p('max_linear_x').get_parameter_value().double_value
        min_az = p('min_angular_z').get_parameter_value().double_value
        max_az = p('max_angular_z').get_parameter_value().double_value
        if min_lx > max_lx:
            self.get_logger().warning(
                'min_linear_x (%.4f) > max_linear_x (%.4f); clamp will use the effective range'
                % (min_lx, max_lx)
            )
        if min_az > max_az:
            self.get_logger().warning(
                'min_angular_z (%.4f) > max_angular_z (%.4f); clamp will use the effective range'
                % (min_az, max_az)
            )

    def _log_kinematics_hint(self) -> None:
        fr = _as_bool(self.get_parameter('forbid_reverse').get_parameter_value().bool_value)
        mst = float(self.get_parameter('min_surge_for_turn').get_parameter_value().double_value)
        self.get_logger().info(
            'Kinematics: forbid_reverse=%s min_surge_for_turn=%.4f (m/s; ≤0 disables no-inplace-yaw)'
            % (fr, mst)
        )

    def _on_estop(self, msg: Bool) -> None:
        with self._lock:
            self._estop = _as_bool(msg.data)

    def _on_mavros_state(self, msg: State) -> None:
        with self._lock:
            self._mavros_connected = bool(msg.connected)
            self._last_mavros_mode_str = str(msg.mode or '')
            self._px4_offboard = bool(
                msg.connected
                and _flight_mode_implies_offboard(msg.mode, self._offboard_cmode_allow)
            )

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._cmd_linear_x = float(msg.linear.x)
            self._cmd_angular_z = float(msg.angular.z)
            self._have_cmd = True
            self._last_cmd_sec = self.get_clock().now().nanoseconds * 1e-9
            self._timed_out_log = False

    def _saturate_and_deadband(self, lx: float, az: float) -> Twist:
        p = self.get_parameter
        forbid_rev = _as_bool(p('forbid_reverse').get_parameter_value().bool_value)
        if forbid_rev:
            lx = max(0.0, lx)

        min_lx = p('min_linear_x').get_parameter_value().double_value
        max_lx = p('max_linear_x').get_parameter_value().double_value
        min_az = p('min_angular_z').get_parameter_value().double_value
        max_az = p('max_angular_z').get_parameter_value().double_value
        ld = p('linear_deadband').get_parameter_value().double_value
        ad = p('angular_deadband').get_parameter_value().double_value

        lo_lx, hi_lx = (min_lx, max_lx) if min_lx <= max_lx else (max_lx, min_lx)
        lo_az, hi_az = (min_az, max_az) if min_az <= max_az else (max_az, min_az)
        if forbid_rev:
            lo_lx = max(0.0, lo_lx)
        lx = max(lo_lx, min(hi_lx, lx))
        az = max(lo_az, min(hi_az, az))
        if abs(lx) < ld:
            lx = 0.0
        if abs(az) < ad:
            az = 0.0

        min_turn = float(p('min_surge_for_turn').get_parameter_value().double_value)
        if min_turn > 0.0 and abs(lx) < min_turn:
            az = 0.0

        out = Twist()
        out.linear.x = lx
        out.angular.z = az
        return out

    def _on_timer(self) -> None:
        with self._lock:
            estop = self._estop
            have_cmd = self._have_cmd
            last_sec = self._last_cmd_sec
            lx_in = self._cmd_linear_x
            az_in = self._cmd_angular_z
            offboard = self._px4_offboard
            connected = self._mavros_connected
            mode_str = self._last_mavros_mode_str
        now = self.get_clock().now().nanoseconds * 1e-9

        if estop:
            self._pub.publish(Twist())
            return

        timeout = float(self.get_parameter('cmd_timeout_sec').get_parameter_value().double_value)

        stale = (
            not have_cmd
            or last_sec is None
            or (now - last_sec) > timeout
        )
        if stale:
            self._pub.publish(Twist())
            log_now = False
            with self._lock:
                if not self._timed_out_log:
                    self._timed_out_log = True
                    log_now = True
            if log_now:
                self.get_logger().warning(
                    'cmd_vel stale or missing → publishing zero (timeout %.3fs)' % timeout
                )
            return

        with self._lock:
            self._timed_out_log = False

        if self._gate_offboard:
            if not connected or not offboard:
                self._pub.publish(Twist())
                if (lx_in != 0.0 or az_in != 0.0):
                    log_blk = False
                    with self._lock:
                        if self._offboard_warn_time is None or (
                            now - self._offboard_warn_time
                        ) >= 5.0:
                            self._offboard_warn_time = now
                            log_blk = True
                    if log_blk:
                        self.get_logger().warning(
                            'Non-zero cmd_vel suppressed: MAVROS connected=%s '
                            'flight_mode_offboard=%s (mode=%r). '
                            'Bench: require_offboard_for_motion:=false. '
                            'Unknown CMODE: set offboard_cmode_allowlist.'
                            % (connected, offboard, mode_str)
                        )
                return

        self._pub.publish(self._saturate_and_deadband(lx_in, az_in))

    def publish_stop(self) -> None:
        """Best-effort zero setpoint before teardown (OFFBOARD safety)."""
        self._pub.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Nav2CmdVelToMavros()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
