#!/usr/bin/env python3

import json
import os
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped, Twist
from m_common.msg import NavStatus
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


def wait_until(predicate, timeout=10.0, message="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail(f"timed out waiting for {message}")


def test_nav_status_aggregates_health_events_and_mission():
    os.environ["ROS_DOMAIN_ID"] = "85"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "85"
    executable = Path(get_package_prefix("workspace_nav")) / "lib" / "workspace_nav" / "nav_status_aggregator"
    process = subprocess.Popen(
        [str(executable), "--ros-args",
         "-p", "publish_rate:=10.0",
         "-p", "odom_topic:=/test_status_odom",
         "-p", "gps_topic:=/test_status_gps",
         "-p", "cmd_vel_topic:=/test_status_cmd",
         "-p", "status_detail_topic:=/test_status_detail",
         "-p", "nav_status_topic:=/test_nav_status",
         "-p", "task_event_topic:=/test_task_event",
         "-p", "follow_waypoints_status_topic:=/test_fw_status",
         "-p", "planner_status_topic:=/test_planner_status",
         "-p", "follow_path_status_topic:=/test_fp_status",
         "-p", "sync_stuck_from_nav2:=false",
         "-p", "auto_recover_costmap_current:=false"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    rclpy.init()
    node = Node("nav_status_cpp_test_harness")
    statuses = []
    events = []
    qos = QoSProfile(depth=20)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(NavStatus, "/test_nav_status", lambda msg: statuses.append(msg), qos)
    node.create_subscription(
        String, "/test_task_event", lambda msg: events.append(json.loads(msg.data)), 50)
    odom_pub = node.create_publisher(Odometry, "/test_status_odom", 10)
    gps_pub = node.create_publisher(NavSatFix, "/test_status_gps", 10)
    cmd_pub = node.create_publisher(Twist, "/test_status_cmd", 10)
    detail_pub = node.create_publisher(String, "/test_status_detail", 10)
    planner_pub = node.create_publisher(GoalStatusArray, "/test_planner_status", 10)
    fw_pub = node.create_publisher(GoalStatusArray, "/test_fw_status", 10)

    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"
    transform.transform.rotation.w = 1.0
    tf = StaticTransformBroadcaster(node)
    tf.sendTransform(transform)

    mission_detail = String(data=json.dumps({
        "state": "RUNNING", "task_id": "task-cpp", "command_id": "cmd-cpp",
        "waypoint_total": 2, "waypoint_completed": 0,
        "waypoint_current_index": 0, "elapsed_sec": 1.2, "error_code": None}))
    executing = GoalStatusArray()
    executing.status_list = [GoalStatus(status=GoalStatus.STATUS_EXECUTING)]

    def publish_inputs():
        detail_pub.publish(mission_detail)
        fw_pub.publish(executing)
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        gps = NavSatFix()
        gps.status.status = 0
        gps_pub.publish(gps)
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd_pub.publish(cmd)

    timer = node.create_timer(0.05, publish_inputs)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: any(s.task_id == "task-cpp" and s.localization_overall == "GOOD" for s in statuses),
            message="healthy RUNNING NavStatus")
        status = next(s for s in statuses if s.task_id == "task-cpp")
        assert status.nav_phase == "TRACKING"
        assert status.alerts_mission_bridge_alive
        assert status.total_waypoints == 2

        aborted = GoalStatusArray()
        aborted.status_list = [GoalStatus(status=GoalStatus.STATUS_ABORTED)]
        planner_pub.publish(aborted)
        wait_until(lambda: statuses and statuses[-1].planner_status == "FAILED", message="planner failure")
        wait_until(
            lambda: any(e["event"] == "ALARM_RAISED" and e["detail"]["alarm_code"] == "PLAN_FAILED" for e in events),
            message="PLAN_FAILED alarm")

        succeeded = GoalStatusArray()
        succeeded.status_list = [GoalStatus(status=GoalStatus.STATUS_SUCCEEDED)]
        planner_pub.publish(succeeded)
        wait_until(lambda: statuses[-1].planner_status == "OK", message="planner recovery")
        wait_until(
            lambda: any(e["event"] == "ALARM_CLEARED" and e["detail"]["alarm_code"] == "PLAN_FAILED" for e in events),
            message="PLAN_FAILED clear")

        detail_pub.publish(String(data=json.dumps({
            "state": "COMPLETED", "task_id": "task-cpp", "command_id": "cmd-cpp",
            "waypoint_total": 2, "waypoint_completed": 2,
            "waypoint_current_index": 2, "elapsed_sec": 8.5, "error_code": None})))
        wait_until(
            lambda: any(e["event"] == "TASK_COMPLETED" and e["task_id"] == "task-cpp" for e in events),
            message="TASK_COMPLETED event")
    finally:
        executor.shutdown()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        process.communicate(timeout=5)
