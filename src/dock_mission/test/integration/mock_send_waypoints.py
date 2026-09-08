#!/usr/bin/env python3
"""Minimal mock for mission_bridge SendWaypoints used in smoke tests."""

import rclpy
from rclpy.node import Node

from m_common.srv import SendWaypoints


class MockSendWaypoints(Node):
    def __init__(self):
        super().__init__("mock_send_waypoints")
        self.create_service(
            SendWaypoints, "/mission_bridge/send_waypoints", self._handle
        )

    def _handle(self, req, resp):
        self.get_logger().info(
            f"mock send_waypoints mission_id={req.mission_id!r} "
            f"waypoints={len(req.waypoints)}"
        )
        resp.success = True
        resp.message = "mock ok"
        return resp


def main():
    rclpy.init()
    node = MockSendWaypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
