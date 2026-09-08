#!/usr/bin/env python3
"""数据流测试用 mock mission_bridge：实船 WGS84 SendWaypoints 契约。

收到航线 → 打印航点 → success；2s 后发 /task_event TASK_COMPLETED
（dock_mission 据此从 NAV_TO_STAGING 进入 SETTLE）。
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from m_common.srv import SendWaypoints


class MockMissionBridge(Node):
    def __init__(self):
        super().__init__("mock_mission_bridge")
        self.create_service(
            SendWaypoints, "/mission_bridge/send_waypoints", self._handle
        )
        self._event_pub = self.create_publisher(String, "/task_event", 10)
        self._pending = None
        self.create_timer(0.1, self._tick)
        self.get_logger().info("mock mission_bridge up (WGS84 SendWaypoints)")

    def _handle(self, req, resp):
        for wp in req.waypoints:
            self.get_logger().info(
                f"send_waypoints: mission_id={req.mission_id!r} "
                f"lat={wp.latitude:.7f} lon={wp.longitude:.7f} "
                f"yaw={wp.yaw:.2f} seq={wp.seq}"
            )
        resp.success = True
        resp.message = "mock ok"
        self._pending = (req.mission_id, 2.0)
        return resp

    def _tick(self):
        if self._pending is None:
            return
        mission_id, remain = self._pending
        remain -= 0.1
        if remain > 0.0:
            self._pending = (mission_id, remain)
            return
        self._pending = None
        payload = {
            "schema_version": 1,
            "task_id": mission_id,
            "command_id": "",
            "event": "TASK_COMPLETED",
            "detail": {"task_id": mission_id, "elapsed_sec": 2.0},
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._event_pub.publish(msg)
        self.get_logger().info(f"/task_event TASK_COMPLETED task_id={mission_id!r}")


def main():
    rclpy.init()
    node = MockMissionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
