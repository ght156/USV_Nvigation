#!/usr/bin/env python3
"""数据流测试用假环境：广播 odom→base_link（静止）与 base_link→dock_frame（脚本化
轨迹，模拟船从坞外 x=-2.5 倒到 x=0），并发布两路零速度 Odometry。

只验证数据链：TF / 话题流向、FSM 状态迁移、cmd_vel 输出，不模拟动力学。
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FakeEnv(Node):
    def __init__(self):
        super().__init__("fake_dock_env")
        self._tfb = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self._mavros_pub = self.create_publisher(
            Odometry, "/mavros/gps_input/local", 10
        )
        self._t0 = self.get_clock().now()
        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info("fake env up: TF odom->base_link, base_link->dock_frame")

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _dock_x(self, t: float) -> float:
        """dock_frame 相对 base_link 的 x（=船在 dock 系的 x）：
        -2.5 保持 20s（等编排层走完预泊→验收→交接），再缓移到 0。"""
        if t < 20.0:
            return -2.5
        t -= 20.0
        x = -2.5 + 0.15 * t              # 段1：-2.5 → -0.7，约 12s
        if x >= -0.7:
            x = -0.7 + 0.06 * (t - 12.0)  # 段2：-0.7 → 0，约 12s
        return min(x, 0.0)

    def _tick(self):
        now = self.get_clock().now().to_msg()
        x = self._dock_x(self._elapsed())

        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = "odom"
        t1.child_frame_id = "base_link"
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "dock_frame"
        t2.transform.translation.x = x
        qx, qy, qz, qw = yaw_to_quat(math.pi)
        t2.transform.rotation.x = qx
        t2.transform.rotation.y = qy
        t2.transform.rotation.z = qz
        t2.transform.rotation.w = qw
        self._tfb.sendTransform([t1, t2])

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        self._odom_pub.publish(odom)

        # 给 dock_entry_validator 用的“船位”（它把该话题 pose 当 map 系）：
        # map(2.5, 0, yaw=π) → dock_enu ex=-3.0, ey=0, eyaw=0，落在占位走廊内
        m = Odometry()
        m.header.stamp = now
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"
        m.pose.pose.position.x = 2.5
        qx, qy, qz, qw = yaw_to_quat(math.pi)
        m.pose.pose.orientation.x = qx
        m.pose.pose.orientation.y = qy
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        self._mavros_pub.publish(m)


def main():
    rclpy.init()
    node = FakeEnv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
