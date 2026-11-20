#!/usr/bin/env python3

import json
import os
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


def wait_until(predicate, timeout=10.0, message="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail(f"timed out waiting for {message}")


def executable(name):
    return Path(get_package_prefix("workspace_nav")) / "lib" / "workspace_nav" / name


class FollowDouble(Node):
    def __init__(self):
        super().__init__("waypoint_with_state_test_follow")
        self.goals = []
        self.server = ActionServer(
            self, FollowWaypoints, "follow_waypoints", self.execute,
            callback_group=ReentrantCallbackGroup())

    def execute(self, goal_handle):
        self.goals.append(goal_handle.request)
        goal_handle.succeed()
        return FollowWaypoints.Result()


def test_waypoint_transform_converts_and_shuts_down(tmp_path):
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(
        "origin: [10.0, 20.0, 0.0]\nref_gnss_10: [120.0, 31.0]\n", encoding="utf-8")
    output = tmp_path / "waypoints.json"
    os.environ["ROS_DOMAIN_ID"] = "84"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "84"
    env["USV_HOME"] = str(tmp_path)
    env["WAYPOINT_OUTPUT_PATH"] = str(output)
    process = subprocess.Popen(
        [str(executable("waypoint_transform")), "--ros-args", "-p", f"map_yaml_path:={map_yaml}"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = Node("waypoint_transform_test_publisher")
    publisher = node.create_publisher(String, "/waypoint", 10)
    try:
        payload = String(data=json.dumps({
            "waypoints": [{"latitude": 31.0, "longitude": 120.001}]}))
        for _ in range(5):
            publisher.publish(payload)
            rclpy.spin_once(node, timeout_sec=0.1)
            if output.exists():
                break
        wait_until(output.exists, message="converted waypoints.json")
        document = json.loads(output.read_text(encoding="utf-8"))
        assert document["waypoints"][0]["x"] == pytest.approx(105.4194, abs=0.0002)
        assert document["waypoints"][0]["y"] == pytest.approx(20.0, abs=0.0002)
        assert document["datum_source_meta"]["datum_source"] == "map_yaml"
        assert process.wait(timeout=5) == 0
        log_file = tmp_path / "logs" / "navigation" / "waypoint_transform.log"
        assert log_file.is_file(), "mlogger navigation log was not created"
        assert "Waypoints written" in log_file.read_text(encoding="utf-8")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.terminate()
            process.communicate(timeout=5)


def test_waypoint_with_state_executes_file_and_clears_it(tmp_path):
    waypoint_file = tmp_path / "waypoints.json"
    waypoint_file.write_text(json.dumps({
        "waypoints": [{"x": 10.0, "y": 0.0}, {"x": 20.0, "y": 0.0}]}), encoding="utf-8")
    os.environ["ROS_DOMAIN_ID"] = "84"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "84"
    env["WAYPOINT_FILE_PATH"] = str(waypoint_file)
    process = subprocess.Popen(
        [str(executable("waypoint_with_state")), "--ros-args", "-p", "odom_topic:=/test_wp_odom"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    rclpy.init()
    follow = FollowDouble()
    odom_node = Node("waypoint_with_state_test_odom")
    odom_pub = odom_node.create_publisher(Odometry, "/test_wp_odom", 10)
    odom_timer = odom_node.create_timer(0.05, lambda: odom_pub.publish(Odometry()))
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(follow)
    executor.add_node(odom_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(lambda: len(follow.goals) >= 2, timeout=12, message="two FollowWaypoints goals")
        wait_until(
            lambda: json.loads(waypoint_file.read_text(encoding="utf-8")) == {},
            message="legacy waypoint file cleared")
        assert follow.goals[0].poses[0].pose.position.x == pytest.approx(10.0)
        assert follow.goals[1].poses[0].pose.position.x == pytest.approx(20.0)
    finally:
        executor.shutdown()
        odom_node.destroy_timer(odom_timer)
        follow.destroy_node()
        odom_node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        process.communicate(timeout=5)
