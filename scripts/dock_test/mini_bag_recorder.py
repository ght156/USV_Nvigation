#!/usr/bin/env python3
"""极简 ROS 2 bag 录制器（ros2bag CLI 与 rosbag2_py 版本错配时的备胎）。

用法：python3 mini_bag_recorder.py <输出目录> <topic1> [topic2 ...]
话题出现后自动补订阅（QoS 跟随发布者），Ctrl-C 停止。
"""

import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py


class MiniRecorder(Node):
    def __init__(self, uri: str, topics: list[str]):
        super().__init__("mini_bag_recorder")
        self._want = set(topics)
        self._subs = {}
        self._types = {}
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        self._counts = {t: 0 for t in topics}
        self.create_timer(2.0, self._discover)
        self._discover()

    def _discover(self):
        known = dict(self.get_topic_names_and_types())
        for t in sorted(self._want - set(self._subs)):
            types = known.get(t)
            if not types:
                continue
            msg_cls = get_message(types[0])
            qos = QoSProfile(depth=100)
            infos = self.get_publishers_info_by_topic(t)
            if infos:
                qos.reliability = infos[0].qos_profile.reliability
                qos.durability = infos[0].qos_profile.durability
            if qos.durability == DurabilityPolicy.UNKNOWN:
                qos.durability = DurabilityPolicy.VOLATILE
            self._writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=t, type=types[0], serialization_format="cdr"
                )
            )
            self._subs[t] = self.create_subscription(
                msg_cls, t, lambda m, tt=t: self._on_msg(tt, m), qos
            )
            self._types[t] = types[0]
            self.get_logger().info(f"subscribed {t} ({types[0]})")

    def _on_msg(self, topic, msg):
        self._writer.write(
            topic, serialize_message(msg), self.get_clock().now().nanoseconds
        )
        self._counts[topic] += 1

    def summary(self):
        return ", ".join(f"{t}={n}" for t, n in self._counts.items())


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    uri, topics = sys.argv[1], sys.argv[2:]
    rclpy.init()
    node = MiniRecorder(uri, topics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("stopped.", node.summary())
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
