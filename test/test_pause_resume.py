#!/usr/bin/env python3
"""Pause/resume test: send waypoints then pause before mock completes."""
import sys
import time
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from m_common.srv import SendWaypoints, SetPause


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("pause_test")
    send_cli = node.create_client(SendWaypoints, "mission_bridge/send_waypoints")
    pause_cli = node.create_client(SetPause, "mission_bridge/set_pause")

    for cli, name in ((send_cli, "send"), (pause_cli, "pause")):
        if not cli.wait_for_service(timeout_sec=15.0):
            print(f"FAIL: {name} service unavailable")
            return 1

    def spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    t = threading.Thread(target=spin, daemon=True)
    t.start()

    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = 100.0
    ps.pose.position.y = 100.0
    ps.pose.orientation.w = 1.0

    req = SendWaypoints.Request()
    req.waypoints = [ps, ps]
    req.mission_id = "pause_py_test"
    req.command_id = "c1"
    fut = send_cli.call_async(req)
    while not fut.done():
        time.sleep(0.05)
    resp = fut.result()
    print(f"send: success={resp.success} msg={resp.message}")
    if not resp.success:
        return 1

    time.sleep(1.5)  # allow RUNNING + goal in flight

    req = SetPause.Request()
    req.pause = True
    fut = pause_cli.call_async(req)
    while not fut.done():
        time.sleep(0.05)
    resp = fut.result()
    print(f"pause: success={resp.success} msg={resp.message}")
    if not resp.success:
        return 1

    req.pause = False
    fut = pause_cli.call_async(req)
    while not fut.done():
        time.sleep(0.05)
    resp = fut.result()
    print(f"resume: success={resp.success} msg={resp.message}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if resp.success else 1


if __name__ == "__main__":
    sys.exit(main())
