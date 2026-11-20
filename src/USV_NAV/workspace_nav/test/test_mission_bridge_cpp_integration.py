#!/usr/bin/env python3

import json
import os
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from m_common.action import NavigateTask
from m_common.msg import MissionWaypoint
from m_common.srv import CancelMission, PlanMission, SendWaypoints, SetPause
from nav2_msgs.action import ComputePathThroughPoses, FollowWaypoints
from nav_msgs.msg import Odometry, Path as NavPath
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


def _wait_until(predicate, timeout=8.0, message="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail(f"timed out waiting for {message}")


def _call_service(client, request, timeout=8.0):
    _wait_until(client.wait_for_service, timeout=timeout, message=client.srv_name)
    future = client.call_async(request)
    _wait_until(future.done, timeout=timeout, message=f"{client.srv_name} response")
    return future.result()


class Nav2Double(Node):
    def __init__(self):
        super().__init__("mission_bridge_test_nav2_double")
        self.callback_group = ReentrantCallbackGroup()
        self.release_follow_goals = False
        self.follow_goal_count = 0
        self.last_follow_goal = None
        self.follow_server = ActionServer(
            self,
            FollowWaypoints,
            "test_follow_waypoints",
            self._execute_follow,
            callback_group=self.callback_group,
        )
        self.path_server = ActionServer(
            self,
            ComputePathThroughPoses,
            "test_compute_path",
            self._execute_path,
            callback_group=self.callback_group,
        )
        self.odom_pub = self.create_publisher(Odometry, "/test_mission_odom", 10)
        self.create_timer(0.05, self._publish_odom)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster.sendTransform(transform)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        self.odom_pub.publish(msg)

    def _execute_follow(self, goal_handle):
        self.follow_goal_count += 1
        self.last_follow_goal = goal_handle.request
        while not self.release_follow_goals and not goal_handle.is_cancel_requested:
            time.sleep(0.01)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return FollowWaypoints.Result()

    def _execute_path(self, goal_handle):
        result = ComputePathThroughPoses.Result()
        result.path = NavPath()
        result.path.header.frame_id = "map"
        result.path.poses = list(goal_handle.request.goals)
        goal_handle.succeed()
        return result


@pytest.fixture(scope="module")
def running_bridge(tmp_path_factory):
    executable = Path(get_package_prefix("workspace_nav")) / "lib" / "workspace_nav" / "mission_bridge"
    if not executable.is_file():
        pytest.fail(f"C++ mission_bridge executable is missing: {executable}")
    with executable.open("rb") as stream:
        if stream.read(4) != b"\x7fELF":
            pytest.fail(f"mission_bridge is not the C++ ELF executable: {executable}")

    root = tmp_path_factory.mktemp("mission_bridge_cpp")
    map_yaml = root / "map.yaml"
    map_yaml.write_text(
        "image: map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n"
        "ref_gnss_10: [120.0, 31.0]\n",
        encoding="utf-8",
    )
    nav2_yaml = root / "nav2.yaml"
    nav2_yaml.write_text(
        "controller_server:\n  ros__parameters:\n"
        "    goal_checker_plugins: [general_goal_checker]\n"
        "    general_goal_checker:\n      xy_goal_tolerance: 0.25\n",
        encoding="utf-8",
    )
    waypoints_json = root / "waypoints.json"
    target_json = root / "target_buoy.json"

    os.environ["ROS_DOMAIN_ID"] = "83"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env = os.environ.copy()
    env["RCUTILS_LOGGING_BUFFERED"] = "0"
    command = [
        str(executable),
        "--ros-args",
        "-p", f"map_yaml_path:={map_yaml}",
        "-p", f"nav2_params_path:={nav2_yaml}",
        "-p", f"waypoints_json_path:={waypoints_json}",
        "-p", f"target_buoy_json_path:={target_json}",
        "-p", "follow_waypoints_action:=test_follow_waypoints",
        "-p", "compute_path_through_poses_action:=test_compute_path",
        "-p", "odom_topic:=/test_mission_odom",
        "-p", "clear_costmap_on_mission_start:=false",
        "-p", "reinflate_after_costmap_clear:=false",
        "-p", "tf_check_period_sec:=0.1",
        "-p", "localization_timeout_sec:=5.0",
        "-p", "ctrl_stuck_timeout_sec:=30.0",
    ]
    process = subprocess.Popen(
        command,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    rclpy.init()
    nav2 = Nav2Double()
    observer = Node("mission_bridge_test_observer")
    states = []
    details = []
    observer.create_subscription(
        String, "/mission_bridge/state", lambda msg: states.append(msg.data), 20)
    observer.create_subscription(
        String, "/mission_bridge/status_detail", lambda msg: details.append(json.loads(msg.data)), 50)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(nav2)
    executor.add_node(observer)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        _wait_until(lambda: "IDLE" in states, message="C++ bridge to become IDLE")
        yield {
            "process": process,
            "observer": observer,
            "nav2": nav2,
            "states": states,
            "details": details,
            "waypoints": waypoints_json,
            "target": target_json,
        }
    finally:
        executor.shutdown()
        nav2.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            output = process.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            process.kill()
            output = process.communicate(timeout=5)[0]
        print(output)


def _waypoint(seq=1):
    return MissionWaypoint(latitude=31.0, longitude=120.001, yaw=90.0, seq=seq)


def test_cpp_bridge_plan_pause_resume_cancel_and_action(running_bridge):
    observer = running_bridge["observer"]
    nav2 = running_bridge["nav2"]
    states = running_bridge["states"]

    plan_client = observer.create_client(PlanMission, "mission_bridge/plan_mission")
    plan_request = PlanMission.Request()
    plan_request.waypoints = [_waypoint(1)]
    plan_response = _call_service(plan_client, plan_request, timeout=20.0)
    assert plan_response.success, plan_response.message
    assert len(plan_response.path.poses) == 1
    assert states[-1] == "IDLE"
    assert not running_bridge["waypoints"].exists()

    send_client = observer.create_client(SendWaypoints, "mission_bridge/send_waypoints")
    pause_client = observer.create_client(SetPause, "mission_bridge/set_pause")
    cancel_client = observer.create_client(CancelMission, "mission_bridge/cancel_mission")

    nav2.release_follow_goals = False
    send_request = SendWaypoints.Request()
    send_request.waypoints = [_waypoint(7)]
    send_request.mission_id = "service-mission"
    send_request.command_id = "command-7"
    send_response = _call_service(send_client, send_request)
    assert send_response.success, send_response.message
    _wait_until(lambda: "RUNNING" in states, message="service mission RUNNING")
    _wait_until(lambda: nav2.follow_goal_count >= 1, message="FollowWaypoints goal")

    pause_request = SetPause.Request()
    pause_request.pause = True
    pause_response = _call_service(pause_client, pause_request)
    assert pause_response.success, pause_response.message
    _wait_until(lambda: states[-1] == "PAUSED", message="PAUSED state")

    pause_request.pause = False
    resume_response = _call_service(pause_client, pause_request)
    assert resume_response.success, resume_response.message
    _wait_until(lambda: states[-1] == "RUNNING", message="resumed RUNNING state")

    cancel_response = _call_service(cancel_client, CancelMission.Request())
    assert cancel_response.success, cancel_response.message
    _wait_until(lambda: states[-1] == "IDLE", message="cancelled IDLE state")
    assert json.loads(running_bridge["waypoints"].read_text(encoding="utf-8")) == {"waypoints": []}

    nav2.release_follow_goals = True
    action_client = ActionClient(observer, NavigateTask, "/mission_bridge/navigate")
    _wait_until(action_client.wait_for_server, message="NavigateTask server")
    goal = NavigateTask.Goal()
    goal.waypoints = [_waypoint(42)]
    goal_future = action_client.send_goal_async(goal)
    _wait_until(goal_future.done, message="NavigateTask goal response")
    goal_handle = goal_future.result()
    assert goal_handle.accepted
    result_future = goal_handle.get_result_async()
    _wait_until(result_future.done, message="NavigateTask result")
    wrapped_result = result_future.result()
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped_result.result.result == NavigateTask.Result.RESULT_SUCCESS
    assert wrapped_result.result.final_reached_seq == 42

    color_pub = observer.create_publisher(String, "/color_code", 10)
    color_pub.publish(String(data="#00FF00"))
    _wait_until(running_bridge["target"].exists, message="target_buoy.json")
    _wait_until(
        lambda: json.loads(running_bridge["target"].read_text(encoding="utf-8"))["color"] == "green",
        message="green target buoy",
    )
    assert any(detail.get("state") == "COMPLETED" for detail in running_bridge["details"])
