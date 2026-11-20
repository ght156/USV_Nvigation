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
from m_common.srv import (
    CancelMission,
    DockTaskEvent,
    QueryInDock,
    SendWaypoints,
    SetChannelEnabled,
)
from nav_msgs.msg import Odometry
import pytest
from rcl_interfaces.srv import SetParameters
import rclpy
from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import Trigger


os.environ["ROS_DOMAIN_ID"] = "224"
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
EXECUTABLE = (
    Path(get_package_prefix("dock_mission")) / "lib" / "dock_mission" / "dock_mission_node"
)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(node, predicate, timeout, callback=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if callback:
            callback()
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return predicate()


def test_cpp_mission_full_service_chain_and_in_dock_query():
    tempdir = tempfile.TemporaryDirectory()
    env = os.environ.copy()
    env["USV_HOME"] = tempdir.name
    parameters = {
        "use_goal_checker_selector": False,
        "settle_sec": 0.3,
        "settle_still_hold_sec": 0.2,
        "settle_min_still_frames": 3,
        "settle_timeout_sec": 2.0,
        "complete_settle_sec": 0.2,
        "status_publish_rate": 20.0,
        "in_dock_backlight_mask": 0,
    }
    command = [str(EXECUTABLE), "--ros-args"]
    for key, value in parameters.items():
        rendered = str(value).lower() if isinstance(value, bool) else str(value)
        command += ["-p", f"{key}:={rendered}"]
    process = subprocess.Popen(
        command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True
    )
    node = rclpy.create_node("cpp_mission_integration_test")
    sent_waypoints = []
    cancelled_missions = []
    dock_starts = []
    dock_cancels = []
    task_statuses = []
    channel_calls = []
    ack_events = []

    def send_waypoints_callback(request, response):
        sent_waypoints.append(request)
        response.success = True
        response.message = "accepted"
        return response

    def cancel_mission_callback(request, response):
        cancelled_missions.append(request)
        response.success = True
        response.message = "cancelled"
        return response

    def set_parameters_callback(request, response):
        from rcl_interfaces.msg import SetParametersResult
        response.results = [SetParametersResult(successful=True) for _ in request.parameters]
        return response

    def event_ack_callback(request, response):
        ack_events.append(request)
        response.success = True
        response.message = "ack"
        response.ack_id = "test-ack"
        return response

    def channel_callback(request, response):
        channel_calls.append((request.channel, request.enabled))
        response.success = True
        response.message = "ok"
        return response

    node.create_service(SendWaypoints, "/mission_bridge/send_waypoints", send_waypoints_callback)
    node.create_service(CancelMission, "/mission_bridge/cancel_mission", cancel_mission_callback)
    node.create_service(SetParameters, "/controller_server/set_parameters", set_parameters_callback)
    node.create_service(DockTaskEvent, "/dock_task/event", event_ack_callback)
    node.create_service(SetChannelEnabled, "/apriltag_node/set_channel_enabled", channel_callback)
    node.create_subscription(Bool, "/dock/start", lambda msg: dock_starts.append(msg.data), 10)
    node.create_subscription(Empty, "/dock/cancel", lambda msg: dock_cancels.append(msg), 10)
    node.create_subscription(
        String, "/dock_task/status", lambda msg: task_statuses.append(json.loads(msg.data)), 10
    )
    task_event_pub = node.create_publisher(String, "/task_event", 10)
    dock_status_pub = node.create_publisher(String, "/dock/status", 10)
    odom_pub = node.create_publisher(Odometry, "/mavros/gps_input/local", 10)
    pose_pub = node.create_publisher(PoseStamped, "/docking/dock_pose", 10)
    source_pub = node.create_publisher(String, "/docking/pose_source", 10)
    anchor_pub = node.create_publisher(String, "/docking/anchor_status", 10)
    age_pub = node.create_publisher(Float32, "/docking/measurement_age", 10)
    start_client = node.create_client(Trigger, "/dock/mission/start")
    cancel_client = node.create_client(Trigger, "/dock/mission/cancel")
    in_dock_client = node.create_client(QueryInDock, "/dock/in_dock")
    try:
        assert spin_until(
            node,
            lambda: start_client.service_is_ready() and in_dock_client.service_is_ready(),
            3.0,
        )
        start_future = start_client.call_async(Trigger.Request())
        assert spin_until(node, start_future.done, 2.0)
        assert start_future.result().success is True

        def feed_odom():
            odom_pub.publish(Odometry())

        assert spin_until(node, lambda: bool(sent_waypoints), 3.0, feed_odom)
        request = sent_waypoints[0]
        assert request.mission_id == "dock_staging"
        assert request.command_id == "retry0"
        assert len(request.waypoints) == 1
        assert request.waypoints[0].latitude != 0.0
        assert request.waypoints[0].longitude != 0.0
        def feed_settle():
            feed_odom()
            task_event_pub.publish(String(data=json.dumps({
                "event": "TASK_COMPLETED",
                "detail": {"task_id": "dock_staging", "command_id": "retry0"},
            })))

        assert spin_until(node, lambda: bool(dock_starts), 3.0, feed_settle)
        assert dock_starts[-1] is True
        assert any(status["phase"] == "STAGING" for status in task_statuses)
        assert any(event.event == "DOCK_STAGING_STARTED" for event in ack_events)
        assert any(enabled for _, enabled in channel_calls)

        def feed_success():
            feed_odom()
            dock_status_pub.publish(String(data=json.dumps({"success": True})))

        assert spin_until(
            node,
            lambda: bool(task_statuses)
            and task_statuses[-1]["run_state"] == "IDLE"
            and not task_statuses[-1]["dock_active"],
            3.0,
            feed_success,
        )
        assert spin_until(
            node, lambda: any(event.event == "DOCK_SUCCEEDED" for event in ack_events), 1.0
        )
        assert dock_cancels
        assert spin_until(
            node, lambda: any(not enabled for _, enabled in channel_calls), 1.0
        )

        def feed_vision():
            pose_pub.publish(PoseStamped())
            source_pub.publish(String(data="VISION"))
            anchor_pub.publish(String(data="VALID"))
            age_pub.publish(Float32(data=0.0))

        spin_until(node, lambda: False, 0.2, feed_vision)
        query = QueryInDock.Request()
        query.wait_vision = False
        query_future = in_dock_client.call_async(query)
        assert spin_until(node, query_future.done, 2.0, feed_vision)
        query_response = query_future.result()
        assert query_response.pose_valid is True
        assert query_response.in_dock is True
        assert query_response.pose_source == "VISION"

        cancel_future = cancel_client.call_async(Trigger.Request())
        assert spin_until(node, cancel_future.done, 2.0)
        assert cancel_future.result().success is True
        assert cancel_future.result().message == "already idle"
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            _, process_stderr = process.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            _, process_stderr = process.communicate(timeout=2)
        print(process_stderr)
        node.destroy_node()
        tempdir.cleanup()
        assert process.returncode in (0, -signal.SIGINT)
