#!/usr/bin/env python3

import os
from pathlib import Path
import struct
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


def wait_until(predicate, timeout=10.0, message="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail(f"timed out waiting for {message}")


def executable(name):
    return Path(get_package_prefix("workspace_nav")) / "lib" / "workspace_nav" / name


def make_cloud():
    cloud = PointCloud2()
    cloud.header.frame_id = "livox"
    cloud.height = 1
    cloud.width = 2
    cloud.point_step = 16
    cloud.row_step = 32
    cloud.is_dense = True
    for index, name in enumerate(("x", "y", "z", "intensity")):
        field = PointField()
        field.name = name
        field.offset = index * 4
        field.datatype = PointField.UINT32 if name == "intensity" else PointField.FLOAT32
        field.count = 1
        cloud.fields.append(field)
    cloud.data = (
        struct.pack("<fffI", 3.0, 0.0, 0.0, 7) +
        struct.pack("<fffI", 10.0, 0.0, 0.0, 42))
    return cloud


def test_livox_filter_drops_masked_point_over_ros():
    os.environ["ROS_DOMAIN_ID"] = "87"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "87"
    process = subprocess.Popen(
        [str(executable("livox_cloud_filter")), "--ros-args",
         "-p", "input_topic:=/test_livox_input", "-p", "output_topic:=/test_livox_output"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = Node("livox_filter_cpp_test")
    outputs = []
    node.create_subscription(PointCloud2, "/test_livox_output", lambda msg: outputs.append(msg), 10)
    qos = QoSProfile(depth=5)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    publisher = node.create_publisher(PointCloud2, "/test_livox_input", qos)
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "livox"
    transform.transform.rotation.w = 1.0
    tf = StaticTransformBroadcaster(node)
    tf.sendTransform(transform)
    timer = node.create_timer(0.1, lambda: publisher.publish(make_cloud()))
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(lambda: outputs, message="filtered PointCloud2")
        assert outputs[-1].width == 1
        assert bytes(outputs[-1].data) == struct.pack("<fffI", 10.0, 0.0, 0.0, 42)
    finally:
        executor.shutdown()
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        process.communicate(timeout=5)


class LifecycleDouble(Node):
    def __init__(self):
        super().__init__("map_guard_lifecycle_double")
        self.state = State.PRIMARY_STATE_UNCONFIGURED
        self.create_service(GetState, "/test_map_server/get_state", self.get_state)
        self.create_service(ChangeState, "/test_map_server/change_state", self.change_state)

    def get_state(self, _request, response):
        response.current_state.id = self.state
        return response

    def change_state(self, request, response):
        if request.transition.id == Transition.TRANSITION_CONFIGURE:
            self.state = State.PRIMARY_STATE_INACTIVE
            response.success = True
        elif request.transition.id == Transition.TRANSITION_ACTIVATE:
            self.state = State.PRIMARY_STATE_ACTIVE
            response.success = True
        else:
            response.success = False
        return response


def test_map_server_guard_configures_and_activates_lifecycle_node():
    os.environ["ROS_DOMAIN_ID"] = "87"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "87"
    process = subprocess.Popen(
        [str(executable("map_server_guard")),
         "--node-name", "/test_map_server", "--start-delay", "0",
         "--check-period", "0.05", "--timeout", "5"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    lifecycle = LifecycleDouble()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(lifecycle)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert process.wait(timeout=10) == 0
        assert lifecycle.state == State.PRIMARY_STATE_ACTIVE
    finally:
        executor.shutdown()
        lifecycle.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.terminate()
            process.communicate(timeout=5)
