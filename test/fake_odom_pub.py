#!/usr/bin/env python3
"""Publish minimal odometry so mission_bridge can send waypoints."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FakeOdomPub(Node):
    def __init__(self) -> None:
        super().__init__("fake_odom_pub")
        self._pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self._timer = self.create_timer(0.2, self._tick)

    def _tick(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = FakeOdomPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
