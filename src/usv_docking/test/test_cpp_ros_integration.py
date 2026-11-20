#!/usr/bin/env python3

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import pytest
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, String
from tf2_ros import TransformBroadcaster
from rclpy.action import ActionServer
from usv_rs485_driver.action import ControlDO, WaitIO


os.environ["ROS_DOMAIN_ID"] = "223"
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
EXE_DIR = Path(get_package_prefix("usv_docking")) / "lib" / "usv_docking"


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class NodeProcess:
    def __init__(self, name, parameters):
        self.tempdir = tempfile.TemporaryDirectory()
        env = os.environ.copy()
        env["USV_HOME"] = self.tempdir.name
        command = [str(EXE_DIR / name), "--ros-args"]
        for key, value in parameters.items():
            command += ["-p", f"{key}:={str(value).lower() if isinstance(value, bool) else value}"]
        self.process = subprocess.Popen(
            command,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        try:
            self.process.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.communicate(timeout=2)
        self.tempdir.cleanup()
        assert self.process.returncode in (0, -signal.SIGINT)


def spin_for(node, seconds, callback=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if callback:
            callback()
        rclpy.spin_once(node, timeout_sec=0.01)


def pose_message(x, y, yaw):
    message = PoseStamped()
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def test_cpp_fsm_normal_chain_status_and_cancel():
    process = NodeProcess("docking_fsm", {
        "dock_claw_enabled": False,
        "backlight_enabled": False,
        "reset_anchor_on_start": False,
        "state_rate": 20.0,
        "status_publish_rate": 20.0,
        "approach_stop_hold_sec": 0.2,
        "align_hold_sec": 0.3,
    })
    node = rclpy.create_node("cpp_fsm_integration_test")
    states = []
    statuses = []
    node.create_subscription(String, "/docking/state", lambda msg: states.append(msg.data), 10)
    node.create_subscription(String, "/dock/status", lambda msg: statuses.append(json.loads(msg.data)), 10)
    start_pub = node.create_publisher(Bool, "/dock/start", 10)
    cancel_pub = node.create_publisher(Empty, "/dock/cancel", 10)
    pose_pub = node.create_publisher(PoseStamped, "/docking/dock_pose", 10)
    source_pub = node.create_publisher(String, "/docking/pose_source", 10)
    visible_pub = node.create_publisher(Bool, "/docking/tag_visible", 10)
    age_pub = node.create_publisher(Float32, "/docking/measurement_age", 10)
    odom_pub = node.create_publisher(Odometry, "/mavros/gps_input/local", 10)
    try:
        spin_for(node, 0.4)
        start_pub.publish(Bool(data=True))

        def feed():
            x = -0.5 if "BACK_IN" in states else -2.0
            pose_pub.publish(pose_message(x, 0.0, math.pi))
            source_pub.publish(String(data="VISION"))
            visible_pub.publish(Bool(data=True))
            age_pub.publish(Float32(data=0.0))
            odom_pub.publish(Odometry())

        spin_for(node, 3.0, feed)
        for expected in ("ACQUIRE_TAG", "APPROACH_ENTRY", "ALIGN_ENTRY", "BACK_IN", "FINAL_DOCK"):
            assert expected in states, f"missing {expected}; states={states}"
        assert statuses
        status = statuses[-1]
        for key in (
            "state", "v2_state", "target_mode", "success", "undock_success",
            "needs_reapproach", "needs_manual_takeover", "pose_source", "dock_x",
            "dock_y", "e_yaw_deg", "dock_claw_enabled", "backlight_session_active",
        ):
            assert key in status
        assert status["pose_valid"] is True
        assert status["dock_claw_enabled"] is False
        cancel_pub.publish(Empty())
        spin_for(node, 0.2, feed)
        assert states[-1] == "IDLE"
        assert statuses[-1]["success"] is False
    finally:
        node.destroy_node()
        process.stop()


def test_cpp_controller_reverse_and_immediate_safety_zero():
    process = NodeProcess("docking_motion_controller", {
        "test_only": True,
        "control_rate": 20.0,
    })
    node = rclpy.create_node("cpp_controller_integration_test")
    commands = []
    node.create_subscription(Twist, "/docking/cmd_vel_test", lambda msg: commands.append(msg), 10)
    mode_pub = node.create_publisher(String, "/docking/target_mode", 10)
    pose_pub = node.create_publisher(PoseStamped, "/docking/dock_pose", 10)
    source_pub = node.create_publisher(String, "/docking/pose_source", 10)
    safety_pub = node.create_publisher(Bool, "/docking/safety_stop", 10)
    try:
        spin_for(node, 0.4)

        def feed(mode="BACK_IN", stopped=False, source="VISION"):
            mode_pub.publish(String(data=mode))
            pose_pub.publish(pose_message(-2.0, 0.0, math.pi))
            source_pub.publish(String(data=source))
            safety_pub.publish(Bool(data=stopped))

        spin_for(node, 0.8, feed)
        assert commands
        moving = [msg for msg in commands if abs(msg.linear.x) > 0.01]
        assert moving and moving[-1].linear.x < 0.0
        spin_for(node, 0.15, lambda: feed(stopped=True))
        assert commands[-1].linear.x == 0.0
        assert commands[-1].angular.z == 0.0
        spin_for(node, 0.15, lambda: feed(source="INVALID"))
        assert commands[-1].linear.x == 0.0
        assert commands[-1].angular.z == 0.0
    finally:
        node.destroy_node()
        process.stop()


def test_cpp_estimator_builds_anchor_from_real_tf():
    process = NodeProcess("docking_pose_estimator", {
        "publish_rate": 20.0,
        "seed_frames": 3,
        "broadcast_tf": True,
    })
    node = rclpy.create_node("cpp_estimator_integration_test")
    poses = []
    sources = []
    anchors = []
    node.create_subscription(PoseStamped, "/docking/dock_pose", lambda msg: poses.append(msg), 10)
    node.create_subscription(String, "/docking/pose_source", lambda msg: sources.append(msg.data), 10)
    node.create_subscription(String, "/docking/anchor_status", lambda msg: anchors.append(msg.data), 10)
    broadcaster = TransformBroadcaster(node)

    def broadcast():
        now = node.get_clock().now().to_msg()
        odom = TransformStamped()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        dock = TransformStamped()
        dock.header.stamp = now
        dock.header.frame_id = "base_link"
        dock.child_frame_id = "dock_frame"
        dock.transform.translation.x = 2.0
        broadcaster.sendTransform([odom, dock])

    try:
        spin_for(node, 1.2, broadcast)
        assert sources and sources[-1] == "VISION", sources
        assert anchors and anchors[-1] == "VALID", anchors
        assert poses
        assert poses[-1].header.frame_id == "dock_est"
        assert poses[-1].pose.position.x == pytest.approx(-2.0, abs=1e-6)
        assert poses[-1].pose.position.y == pytest.approx(0.0, abs=1e-6)
    finally:
        node.destroy_node()
        process.stop()


def test_component_container_loads_three_core_nodes_and_estimator_separately():
    tempdir = tempfile.TemporaryDirectory()
    env = os.environ.copy()
    env["USV_HOME"] = tempdir.name
    process = subprocess.Popen(
        ["ros2", "launch", "usv_docking", "docking.launch.py", "test_only:=true"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    node = rclpy.create_node("cpp_container_integration_test")
    expected = {
        "docking_pose_estimator",
        "docking_core_container",
        "docking_fsm",
        "docking_motion_controller",
        "docking_safety",
    }
    try:
        deadline = time.monotonic() + 10.0
        names = set()
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            names = set(node.get_node_names())
            if expected <= names:
                break
            if process.poll() is not None:
                break
        assert expected <= names, f"expected={expected}, actual={names}"
        assert node.count_publishers("/docking/cmd_vel_test") == 1
    finally:
        node.destroy_node()
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            stdout, stderr = process.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate(timeout=2)
        print(stdout)
        print(stderr)
        log_dir = Path(tempdir.name) / "logs" / "docking"
        log_names = {path.name for path in log_dir.glob("*.log")}
        tempdir.cleanup()
        assert process.returncode in (0, -signal.SIGINT)
        assert {
            "docking_pose_estimator.log",
            "docking_fsm.log",
            "docking_motion_controller.log",
            "docking_safety.log",
        } <= log_names


def test_cpp_fsm_rs485_actions_dock_and_undock():
    process = NodeProcess("docking_fsm", {
        "dock_claw_enabled": True,
        "dock_claw_open_on_start": True,
        "dock_claw_release_on_undock": True,
        "backlight_enabled": False,
        "reset_anchor_on_start": False,
        "state_rate": 20.0,
        "status_publish_rate": 20.0,
        "tag_acquire_frames": 1,
        "approach_stop_hold_sec": 0.1,
        "align_hold_sec": 0.1,
        "undock_settle_sec": 0.1,
    })
    node = rclpy.create_node("cpp_fsm_rs485_integration_test")
    control_goals = []
    wait_goals = []

    def control_execute(goal_handle):
        control_goals.append(goal_handle.request)
        goal_handle.succeed()
        return ControlDO.Result(success=True, message="io ok")

    def wait_execute(goal_handle):
        wait_goals.append(goal_handle.request)
        goal_handle.publish_feedback(WaitIO.Feedback(query_count=1, digital_input=3))
        goal_handle.succeed()
        return WaitIO.Result(success=True, message="grabbed", query_count=1)

    control_server = ActionServer(node, ControlDO, "control_do", control_execute)
    wait_server = ActionServer(node, WaitIO, "wait_io", wait_execute)
    states = []
    statuses = []
    node.create_subscription(String, "/docking/state", lambda msg: states.append(msg.data), 10)
    node.create_subscription(String, "/dock/status", lambda msg: statuses.append(json.loads(msg.data)), 10)
    start_pub = node.create_publisher(Bool, "/dock/start", 10)
    undock_pub = node.create_publisher(Bool, "/dock/undock", 10)
    pose_pub = node.create_publisher(PoseStamped, "/docking/dock_pose", 10)
    source_pub = node.create_publisher(String, "/docking/pose_source", 10)
    visible_pub = node.create_publisher(Bool, "/docking/tag_visible", 10)
    age_pub = node.create_publisher(Float32, "/docking/measurement_age", 10)
    odom_pub = node.create_publisher(Odometry, "/mavros/gps_input/local", 10)
    try:
        spin_for(node, 0.4)
        start_pub.publish(Bool(data=True))

        def feed_dock():
            pose_pub.publish(pose_message(-2.0, 0.0, math.pi))
            source_pub.publish(String(data="VISION"))
            visible_pub.publish(Bool(data=True))
            age_pub.publish(Float32(data=0.0))
            odom_pub.publish(Odometry())

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and "DOCKED" not in states:
            feed_dock()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert "WAIT_DOCK_OPEN" in states
        assert "DOCKED" in states, states
        assert control_goals and control_goals[0].start is True
        assert wait_goals
        spin_for(node, 0.2, feed_dock)
        assert any(status["success"] for status in statuses)

        undock_pub.publish(Bool(data=True))

        def feed_undock():
            feed_dock()
            pose_pub.publish(pose_message(-3.5, 0.0, math.pi))

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            feed_undock()
            rclpy.spin_once(node, timeout_sec=0.02)
            if states and states[-1] == "IDLE" and any(
                status["undock_success"] for status in statuses
            ):
                break
        assert "WAIT_DOCK_RELEASE" in states
        assert "UNDOCK_EXIT" in states
        assert "UNDOCK_SETTLE" in states
        assert states[-1] == "IDLE"
        assert any(status["undock_success"] for status in statuses)
        assert len(control_goals) >= 3
        assert any(goal.mask == 1 for goal in control_goals)
    finally:
        control_server.destroy()
        wait_server.destroy()
        node.destroy_node()
        process.stop()
