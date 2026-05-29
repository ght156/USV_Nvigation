#!/usr/bin/env python3
"""Minimal FollowWaypoints action server for service integration tests."""

import time

import rclpy
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class MockFollowWaypoints(Node):
    def __init__(self) -> None:
        super().__init__("mock_follow_waypoints")
        cg = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            FollowWaypoints,
            "follow_waypoints",
            self._execute,
            callback_group=cg,
        )
        self.get_logger().info("Mock FollowWaypoints ready on follow_waypoints")

    def _execute(self, goal_handle):
        n = len(goal_handle.request.poses)
        self.get_logger().info(f"Goal: {n} pose(s) — succeed in 0.3s")
        time.sleep(0.3)
        goal_handle.succeed()
        result = FollowWaypoints.Result()
        result.missed_waypoints = []
        for attr, val in (("error_code", 0), ("error_msg", "")):
            if hasattr(result, attr):
                setattr(result, attr, val)
        return result


def main() -> None:
    rclpy.init()
    node = MockFollowWaypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
