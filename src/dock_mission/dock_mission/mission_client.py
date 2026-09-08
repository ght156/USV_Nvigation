"""Client for mission_bridge SendWaypoints service.

实船契约：m_common/srv/SendWaypoints 的 waypoints 是
m_common/MissionWaypoint[]（WGS84 经纬度 + ENU 航向度 + seq），
经纬度→map 换算由 mission_bridge 按地图 datum 内部完成。
（仿真仓的同名 srv 是 geometry_msgs/PoseStamped[] map 系，勿混用。）
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from m_common.msg import MissionWaypoint
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
        *,
        latitude: float,
        longitude: float,
        yaw_deg: float,
        mission_id: str = "dock_staging",
        command_id: str = "",
    ) -> tuple[bool, str]:
        if not self._client.service_is_ready():
            return False, "send_waypoints service not ready"
        req = self._build_request(latitude, longitude, yaw_deg, mission_id, command_id)
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        if not future.done():
            return False, "send_waypoints timeout"
        resp = future.result()
        if resp is None:
            return False, "send_waypoints no response"
        return bool(resp.success), str(resp.message)

    def send_staging_async(
        self,
        *,
        latitude: float,
        longitude: float,
        yaw_deg: float,
        mission_id: str = "dock_staging",
        command_id: str = "",
        done_cb,
    ) -> None:
        """非阻塞发送；done_cb(ok, message) 在 executor 回调里触发。

        FSM 定时器回调里严禁用上面的同步版（spin_until_future_complete 嵌套
        自旋在单线程 executor 下收不到响应，必超时）。
        """
        if not self._client.service_is_ready():
            done_cb(False, "send_waypoints service not ready")
            return
        req = self._build_request(latitude, longitude, yaw_deg, mission_id, command_id)
        future = self._client.call_async(req)

        def _done(fut) -> None:
            resp = fut.result()
            if resp is None:
                done_cb(False, "send_waypoints no response")
                return
            done_cb(bool(resp.success), str(resp.message))

        future.add_done_callback(_done)

    def _build_request(
        self,
        latitude: float,
        longitude: float,
        yaw_deg: float,
        mission_id: str,
        command_id: str,
    ) -> SendWaypoints.Request:
        wp = MissionWaypoint()
        wp.latitude = float(latitude)
        wp.longitude = float(longitude)
        wp.yaw = float(yaw_deg)
        wp.seq = 0
        req = SendWaypoints.Request()
        req.waypoints = [wp]
        req.mission_id = mission_id
        req.command_id = command_id
        return req
