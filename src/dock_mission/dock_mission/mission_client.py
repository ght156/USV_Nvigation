"""Client for mission_bridge SendWaypoints service.

实船契约：m_common/srv/SendWaypoints 的 waypoints 是
m_common/MissionWaypoint[]（WGS84 经纬度 + ENU 航向度 + seq），
经纬度→map 换算由 mission_bridge 按地图 datum 内部完成。
（仿真仓的同名 srv 是 geometry_msgs/PoseStamped[] map 系，勿混用。）
"""

from __future__ import annotations

from rclpy.node import Node

from m_common.msg import MissionWaypoint
from m_common.srv import CancelMission, SendWaypoints


class MissionBridgeClient:
    def __init__(self, node: Node) -> None:
        self._node = node
        service = str(node.get_parameter("send_waypoints_service").value)
        self._client = node.create_client(SendWaypoints, service)
        cancel_service = str(node.get_parameter("cancel_mission_service").value)
        self._cancel_client = node.create_client(CancelMission, cancel_service)

    def wait_ready(self, timeout_sec: float = 5.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

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

        只提供异步版：FSM 定时器回调里嵌套 spin_until_future_complete 在
        单线程 executor 下收不到响应，必超时。
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

    def cancel_async(self, done_cb=None) -> None:
        """非阻塞取消 mission_bridge 当前任务（撤销 Nav2 FollowWaypoints goal）。

        与 send_staging_async 同款异步模式；调用失败只经 done_cb 回日志，
        不阻塞主流程。mission_bridge 侧对 IDLE 幂等（"Already idle"）。
        """
        if not self._cancel_client.service_is_ready():
            if done_cb is not None:
                done_cb(False, "cancel_mission service not ready")
            return
        future = self._cancel_client.call_async(CancelMission.Request())

        def _done(fut) -> None:
            if done_cb is None:
                return
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001
                done_cb(False, f"cancel_mission exception: {exc}")
                return
            if resp is None:
                done_cb(False, "cancel_mission no response")
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
