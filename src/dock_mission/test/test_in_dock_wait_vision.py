#!/usr/bin/env python3

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from m_common.srv import QueryInDock, SetChannelEnabled
import pytest
import rclpy
from std_msgs.msg import Float32, String


os.environ["ROS_DOMAIN_ID"] = "224"
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
EXECUTABLE = Path(get_package_prefix("dock_mission")) / "lib" / "dock_mission" / "dock_mission_node"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_wait_vision_refreshes_dock_pose_cache_during_service_call():
    tempdir = tempfile.TemporaryDirectory()
    env = os.environ.copy()
    env["USV_HOME"] = tempdir.name
    process = subprocess.Popen(
        [
            str(EXECUTABLE), "--ros-args",
            "-p", "in_dock_backlight_mask:=0",
            "-p", "in_dock_vision_timeout_sec:=1.0",
            "-p", "use_goal_checker_selector:=false",
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    node = rclpy.create_node("wait_vision_cache_integration_test")

    def channel_callback(request, response):
        response.success = True
        response.message = "ok"
        return response

    node.create_service(SetChannelEnabled, "/apriltag_node/set_channel_enabled", channel_callback)
    pose_pub = node.create_publisher(PoseStamped, "/docking/dock_pose", 10)
    source_pub = node.create_publisher(String, "/docking/pose_source", 10)
    anchor_pub = node.create_publisher(String, "/docking/anchor_status", 10)
    age_pub = node.create_publisher(Float32, "/docking/measurement_age", 10)
    client = node.create_client(QueryInDock, "/dock/in_dock")

    def feed(source, age):
        pose_pub.publish(PoseStamped())
        source_pub.publish(String(data=source))
        anchor_pub.publish(String(data="VALID"))
        age_pub.publish(Float32(data=age))

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not client.service_is_ready():
            feed("INVALID", float("inf"))
            rclpy.spin_once(node, timeout_sec=0.02)
        assert client.service_is_ready()

        query = QueryInDock.Request()
        query.wait_vision = True
        future = client.call_async(query)
        time.sleep(0.2)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not future.done():
            feed("VISION", 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert future.done(), "wait_vision did not complete while VISION was being published"
        response = future.result()
        assert response.pose_valid is True, response.message
        assert response.in_dock is True, response.message
        assert response.pose_source == "VISION"
    finally:
        node.destroy_node()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            _, stderr = process.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            _, stderr = process.communicate(timeout=2)
        print(stderr)
        tempdir.cleanup()
        assert process.returncode in (0, -signal.SIGINT)
