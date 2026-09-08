#!/usr/bin/env python3
"""数据流测试用 mock 上层事件 server：实现 /dock_task/event 并回 ACK。

对应嵌软状态机的服务端契约（m_common/srv/DockTaskEvent）：
船端(dock_mission)作为 client 上报关键事件，本节点回 success=true（ACK）。
仅用于无嵌软时的数据流联调；生产由嵌软实现同一服务。
"""

import time
import uuid

import rclpy
from rclpy.node import Node

from m_common.srv import DockTaskEvent


class MockEventServer(Node):
    def __init__(self):
        super().__init__("mock_event_server")
        self.create_service(DockTaskEvent, "/dock_task/event", self._handle)
        self._count = 0
        self.get_logger().info("mock event server up (/dock_task/event -> ACK)")

    def _handle(self, req, resp):
        self._count += 1
        self.get_logger().info(
            f"[event#{self._count}] {req.event} "
            f"mission_id={req.mission_id or ''!r} "
            f"command_id={req.command_id or ''!r} "
            f"detail={req.detail or '{}'} "
            f"stamp_sec={req.stamp_sec:.3f}"
        )
        resp.success = True
        resp.message = "event accepted"
        resp.ack_id = str(uuid.uuid4())
        resp.ack_stamp_sec = time.time()
        return resp


def main():
    rclpy.init()
    node = MockEventServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
