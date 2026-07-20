"""Client for mission_bridge SendWaypoints service."""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from m_common.srv import SendWaypoints


class MissionBridgeClient:
    def __init__(self, node: Node) -> None:
        self._node = node
        service = str(node.get_parameter("send_waypoints_service").value)
        self._client = node.create_client(SendWaypoints, service)

    def wait_ready(self, timeout_sec: float = 5.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def send_staging(
        self,
        pose: PoseStamped,
        mission_id: str = "dock_staging",
        command_id: str = "",
    ) -> tuple[bool, str]:
        if not self._client.service_is_ready():
            return False, "send_waypoints service not ready"
        req = SendWaypoints.Request()
        req.waypoints = [pose]
        req.mission_id = mission_id
        req.command_id = command_id
        future = self._client.call_async(req)
        import rclpy

        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        if not future.done():
            return False, "send_waypoints timeout"
        resp = future.result()
        if resp is None:
            return False, "send_waypoints no response"
        return bool(resp.success), str(resp.message)
