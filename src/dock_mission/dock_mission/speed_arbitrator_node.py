"""cmd_vel speed authority arbitration (Phase 2 skeleton)."""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from dock_mission.types import SpeedAuthority


class SpeedArbitratorNode(Node):
    def __init__(self) -> None:
        super().__init__("speed_arbitrator")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_nav_topic", "/cmd_vel_nav_raw")
        self.declare_parameter("cmd_vel_dock_topic", "/cmd_vel_dock")
        self.declare_parameter("authority_topic", "/dock/speed_authority")
        self.declare_parameter("watchdog_sec", 1.0)

        out_topic = self.get_parameter("cmd_vel_out_topic").value
        self._authority = SpeedAuthority.NAVIGATION
        self._last_out_time = self.get_clock().now()

        self._pub = self.create_publisher(Twist, out_topic, 10)
        self._nav_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_nav_topic").value,
            self._nav_cb,
            10,
        )
        self._dock_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_dock_topic").value,
            self._dock_cb,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("authority_topic").value,
            self._authority_cb,
            10,
        )

        self._last_nav = Twist()
        self._last_dock = Twist()
        period = 0.05
        self.create_timer(period, self._timer_cb)
        self.get_logger().info(f"speed_arbitrator publishing {out_topic} (skeleton)")

    def _authority_cb(self, msg: String) -> None:
        try:
            self._authority = SpeedAuthority(msg.data.strip())
        except ValueError:
            self.get_logger().warning(f"Unknown authority '{msg.data}'")

    def _nav_cb(self, msg: Twist) -> None:
        self._last_nav = msg

    def _dock_cb(self, msg: Twist) -> None:
        self._last_dock = msg

    def _zero_twist(self) -> Twist:
        return Twist()

    def _select_cmd(self) -> Twist:
        if self._authority == SpeedAuthority.NAVIGATION:
            return self._last_nav
        if self._authority == SpeedAuthority.DOCKING:
            return self._last_dock
        return self._zero_twist()

    def _timer_cb(self) -> None:
        cmd = self._select_cmd()
        if self._authority in (
            SpeedAuthority.STAGING_VERIFY,
            SpeedAuthority.FAILED,
        ):
            cmd = self._zero_twist()
        self._pub.publish(cmd)
        self._last_out_time = self.get_clock().now()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedArbitratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero = Twist()
        node._pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
