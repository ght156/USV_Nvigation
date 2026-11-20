#!/usr/bin/env python3

import json
import os
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from m_common.msg import GeoFence, GeoPoint, NavSafetyEvent
from m_common.srv import GetGeoFence, SetGeoFence
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def wait_until(predicate, timeout=10.0, message="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail(f"timed out waiting for {message}")


def call_service(client, request):
    wait_until(client.wait_for_service, message=client.srv_name)
    future = client.call_async(request)
    wait_until(future.done, message=f"{client.srv_name} response")
    return future.result()


def test_zone_manager_and_monitor_publish_mask_and_paired_safety_events(tmp_path):
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text("origin: [0, 0, 0]\nref_gnss_10: [120, 31]\n", encoding="utf-8")
    persist = tmp_path / "nav_zones.json"
    os.environ["ROS_DOMAIN_ID"] = "86"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "86"
    prefix = Path(get_package_prefix("workspace_nav")) / "lib" / "workspace_nav"
    manager = subprocess.Popen(
        [str(prefix / "zone_manager"), "--ros-args",
         "-p", f"map_yaml_path:={map_yaml}", "-p", f"zones_persist_path:={persist}"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    monitor = subprocess.Popen(
        [str(prefix / "zone_monitor"), "--ros-args",
         "-p", f"map_yaml_path:={map_yaml}", "-p", "odom_topic:=/test_zone_odom",
         "-p", "check_period_sec:=0.05", "-p", "violation_threshold:=2"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    rclpy.init()
    node = Node("zone_cpp_test_harness")
    latched = QoSProfile(depth=5)
    latched.reliability = ReliabilityPolicy.RELIABLE
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
    masks = []
    currents = []
    safety = []
    node.create_subscription(OccupancyGrid, "/keepout_filter/keepout_filter_mask", lambda msg: masks.append(msg), latched)
    node.create_subscription(String, "/nav_zones/current", lambda msg: currents.append(json.loads(msg.data)), latched)
    node.create_subscription(NavSafetyEvent, "/mission_bridge/safety_event", lambda msg: safety.append(msg), 10)
    map_pub = node.create_publisher(OccupancyGrid, "/map", latched)
    odom_pub = node.create_publisher(Odometry, "/test_zone_odom", 10)
    set_client = node.create_client(SetGeoFence, "mission_bridge/set_geofence")
    get_client = node.create_client(GetGeoFence, "mission_bridge/get_geofence")
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    grid = OccupancyGrid()
    grid.info.resolution = 1.0
    grid.info.width = 20
    grid.info.height = 20
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 400
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.position.x = 100.0
    odom.pose.pose.position.y = 100.0

    def publish_inputs():
        map_pub.publish(grid)
        odom_pub.publish(odom)

    timer = node.create_timer(0.05, publish_inputs)
    try:
        request = SetGeoFence.Request()
        fence = GeoFence()
        fence.fence_id = "work_area"
        fence.type = "inclusion"
        fence.shape = "polygon"
        fence.points = [
            GeoPoint(lat=31.0, lon=120.0), GeoPoint(lat=31.0, lon=120.0001),
            GeoPoint(lat=31.0001, lon=120.0001), GeoPoint(lat=31.0001, lon=120.0)]
        request.fences = [fence]
        response = call_service(set_client, request)
        assert response.success, response.message

        wait_until(lambda: masks and currents, message="zone mask and current snapshot")
        assert masks[-1].data[0] == 0
        assert masks[-1].data[19 * 20 + 19] == 100
        assert currents[-1]["fences"][0]["fence_id"] == "work_area"
        assert persist.exists()
        get_response = call_service(get_client, GetGeoFence.Request())
        assert get_response.success
        assert len(get_response.fences) == 1

        wait_until(lambda: any(event.enabled for event in safety), message="geofence violation")
        alarm = next(event for event in safety if event.enabled)
        assert alarm.fence_id == "work_area"
        assert alarm.transition == "EXIT"

        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 1.0
        wait_until(lambda: any(not event.enabled for event in safety), message="geofence clear")
        cleared = next(event for event in safety if not event.enabled)
        assert cleared.fence_id == alarm.fence_id
        assert cleared.transition == alarm.transition
    finally:
        executor.shutdown()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()
        for process in (monitor, manager):
            process.terminate()
        for process in (monitor, manager):
            process.communicate(timeout=5)
