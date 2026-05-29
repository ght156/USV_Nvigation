#!/usr/bin/env python3
"""USV_NAV mission 迁移 smoke test（mock TF/odom/Nav2，不启真 MAVROS）.

在仿真工作区运行，source **实船** USV_NAV 的 install 后验证迁移后的 mission 栈。

环境变量:
  USV_WS  实船工作区根目录，默认 /home/ght/USV_NAV
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

try:
    from m_common.srv import CancelMission, EmergencyStop, SendWaypoints
except ImportError:
    print("FAIL: m_common 未安装。请在 USV_WS 执行: colcon build --packages-select m_common workspace_nav")
    sys.exit(2)

SIM_TEST = Path(__file__).resolve().parent
USV_WS = Path(os.environ.get("USV_WS", "/home/ght/USV_NAV")).expanduser().resolve()
NAV_PKG = USV_WS / "src/USV_NAV/workspace_nav"
MAP = NAV_PKG / "config/map.yaml"
PARAMS = NAV_PKG / "config/mission_stack.real_boat.yaml"

PASS = 0
FAIL = 0
procs: List[subprocess.Popen] = []


def check(cond: bool, label: str) -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [PASS] {label}")
    else:
        FAIL += 1
        print(f"  [FAIL] {label}")


def spawn(cmd: List[str]) -> subprocess.Popen:
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    procs.append(p)
    return p


def cleanup() -> None:
    for p in procs:
        try:
            p.terminate()
        except Exception:
            pass
    for p in procs:
        try:
            p.wait(timeout=3)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass


class Tester(Node):
    def __init__(self) -> None:
        super().__init__("usv_migrate_tester")
        self._nav_status: dict = {}
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        self.create_subscription(String, "/nav_status", self._cb_nav, qos)
        self._odom_pub = self.create_publisher(Odometry, "/mavros/local_position/odom", 10)
        self._wp_pub = self.create_publisher(String, "/waypoint", 10)
        self._cancel_pub = self.create_publisher(Empty, "/gcs_mission/cancel", 10)
        self.create_timer(0.2, self._pub_odom)
        self._send = self.create_client(SendWaypoints, "/mission_bridge/send_waypoints")
        self._emerg = self.create_client(EmergencyStop, "/mission_bridge/emergency_stop")
        self._cancel = self.create_client(CancelMission, "/mission_bridge/cancel_mission")

    def _cb_nav(self, msg: String) -> None:
        try:
            self._nav_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _pub_odom(self) -> None:
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"
        m.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(m)

    def task_state(self) -> str:
        t = self._nav_status.get("task") or {}
        return str(t.get("state", ""))

    def task_id(self) -> str:
        t = self._nav_status.get("task") or {}
        return str(t.get("task_id") or "")

    def wait_state(self, want: str, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.task_state() == want:
                return True
        return False

    def wait_task_id(self, want: str, timeout: float = 10.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.task_id() == want:
                return True
        return False


def main() -> int:
    if not MAP.is_file():
        print(f"FAIL: 找不到实船地图 {MAP}")
        return 2
    if not PARAMS.is_file():
        print(f"FAIL: 找不到 {PARAMS}")
        return 2

    print("========== USV_NAV mission 迁移模拟测试 ==========")
    print(f"  USV_WS={USV_WS}")
    print(f"  MAP={MAP}")
    print(f"  PARAMS={PARAMS}")

    spawn(["ros2", "run", "tf2_ros", "static_transform_publisher", "0", "0", "0", "0", "0", "0", "map", "base_link"])
    time.sleep(0.8)
    spawn(["python3", str(SIM_TEST / "mock_follow_waypoints_server.py")])
    time.sleep(0.8)
    spawn(
        [
            "ros2",
            "launch",
            "workspace_nav",
            "mission_bridge.launch.py",
            "use_sim_time:=false",
            f"map_yaml_path:={MAP}",
            f"params_file:={PARAMS}",
            "odom_topic:=/mavros/local_position/odom",
        ]
    )
    time.sleep(4.0)

    rclpy.init()
    node = Tester()
    try:
        check(node._send.wait_for_service(timeout_sec=8.0), "send_waypoints service 就绪")
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)

        req = SendWaypoints.Request()
        req.waypoints = [_pose(5.0, 5.0), _pose(15.0, 15.0)]
        req.mission_id = "svc_migrate_test"
        req.command_id = "c1"
        fut = node._send.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        resp = fut.result()
        check(resp is not None and resp.success, f"send_waypoints success ({getattr(resp, 'message', '')})")
        check(node.wait_state("RUNNING"), f"nav_status RUNNING (got {node.task_state()})")

        wp = {
            "waypoints": [
                {"latitude": 31.48673, "longitude": 120.36832},
                {"latitude": 31.48680, "longitude": 120.36840},
            ],
            "mission_id": "gcs_migrate",
            "explicit_replan": True,
        }
        msg = String()
        msg.data = json.dumps(wp)
        node._wp_pub.publish(msg)
        check(node.wait_task_id("gcs_migrate"), f"GCS /waypoint 换线 task_id (got {node.task_id()})")

        node._cancel_pub.publish(Empty())
        check(node.wait_state("IDLE"), f"GCS cancel → IDLE (got {node.task_state()})")

        req2 = SendWaypoints.Request()
        req2.waypoints = [_pose(1.0, 1.0)]
        req2.mission_id = "emerg_test"
        fut2 = node._send.call_async(req2)
        rclpy.spin_until_future_complete(node, fut2, timeout_sec=10.0)
        node.wait_state("RUNNING", timeout=6.0)
        fut3 = node._emerg.call_async(EmergencyStop.Request())
        rclpy.spin_until_future_complete(node, fut3, timeout_sec=10.0)
        check(node.wait_state("EMERGENCY"), f"emergency_stop → EMERGENCY (got {node.task_state()})")
        fut4 = node._cancel.call_async(CancelMission.Request())
        rclpy.spin_until_future_complete(node, fut4, timeout_sec=10.0)
        check(node.wait_state("IDLE"), f"cancel_mission 清急停 → IDLE (got {node.task_state()})")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        cleanup()

    print(f"========== 结果: PASS={PASS} FAIL={FAIL} ==========")
    return 0 if FAIL == 0 else 1


def _pose(x: float, y: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.w = 1.0
    return ps


if __name__ == "__main__":
    sys.exit(main())
