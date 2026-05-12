#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# Bridges MAVROS (PX4) topics to the /roboboat/... sensor namespace expected by localization
# (imu_covariance_repub / gps_covariance_repub → navsat_transform → ekf → /odometry/filtered).
#
# QoS: subscriptions use SENSOR_DATA/BEST_EFFORT to match typical MAVROS publishers.
# ----------------------------------------------------------------------------------------------- #

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan, NavSatFix


class MavrosRoboatRelay(Node):
    def __init__(self) -> None:
        super().__init__('mavros_roboboat_relay')

        # launch parameters 会先声明 use_sim_time；避免重复 declare 触发 ParameterAlreadyDeclaredException
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.declare_parameter('imu_src', '/mavros/imu/data')
        self.declare_parameter('imu_dst', '/roboboat/sensors/imu/imu')

        self.declare_parameter('gps_src', '/mavros/global_position/raw/fix')
        self.declare_parameter('gps_dst', '/roboboat/sensors/gps/navsat')

        self.declare_parameter('scan_src', '')
        self.declare_parameter('scan_dst', '/roboboat/sensors/lidar/scan')

        self.declare_parameter('enable_nav2_to_mavros_cmd_vel', False)
        self.declare_parameter('cmd_vel_nav_src', '/cmd_vel_nav')
        self.declare_parameter('mavros_cmd_vel_dst', '/mavros/setpoint_velocity/cmd_vel_unstamped')

        qos = qos_profile_sensor_data

        def as_bool(pv) -> bool:
            """Launch files may substitute string \"true\"/\"false\"."""
            if isinstance(pv, bool):
                return pv
            return str(pv).strip().lower() in ('true', '1', 'yes', 'on')

        imu_dst = self.get_parameter('imu_dst').get_parameter_value().string_value
        imu_src = self.get_parameter('imu_src').get_parameter_value().string_value
        self._imu_pub = self.create_publisher(Imu, imu_dst, 10)
        self.create_subscription(Imu, imu_src, self._on_imu, qos)
        self.get_logger().info('IMU relay %s -> %s' % (imu_src, imu_dst))

        gps_dst = self.get_parameter('gps_dst').get_parameter_value().string_value
        gps_src = self.get_parameter('gps_src').get_parameter_value().string_value
        self._gps_pub = self.create_publisher(NavSatFix, gps_dst, 10)
        self.create_subscription(NavSatFix, gps_src, self._on_gps, qos)
        self.get_logger().info('GPS relay %s -> %s' % (gps_src, gps_dst))

        scan_src = self.get_parameter('scan_src').get_parameter_value().string_value.strip()
        scan_dst = self.get_parameter('scan_dst').get_parameter_value().string_value
        if scan_src:
            self._scan_pub = self.create_publisher(LaserScan, scan_dst, 10)
            self.create_subscription(LaserScan, scan_src, self._on_scan, qos)
            self.get_logger().info('LaserScan relay %s -> %s' % (scan_src, scan_dst))
        else:
            self._scan_pub = None
            self.get_logger().info(
                'LaserScan relay disabled (scan_src empty). '
                'Set param scan_src if you have a front lidar on the boat.'
            )

        if as_bool(self.get_parameter('enable_nav2_to_mavros_cmd_vel').value):
            cmd_src = self.get_parameter('cmd_vel_nav_src').get_parameter_value().string_value
            cmd_dst = self.get_parameter('mavros_cmd_vel_dst').get_parameter_value().string_value
            self._mavros_cmd_pub = self.create_publisher(Twist, cmd_dst, 10)
            self.create_subscription(Twist, cmd_src, self._on_cmd_vel, 10)
            self.get_logger().warning(
                f'Nav2 -> MAVROS cmd_vel bridge ON: {cmd_src} -> {cmd_dst}. '
                'Requires PX4 OFFBOARD (or equivalent) and coordinate frame agreement; '
                'do not run converter thruster bridge unless you intentionally merge both paths.'
            )
        else:
            self._mavros_cmd_pub = None

    def _on_imu(self, msg: Imu) -> None:
        self._imu_pub.publish(msg)

    def _on_gps(self, msg: NavSatFix) -> None:
        self._gps_pub.publish(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        if self._scan_pub is not None:
            self._scan_pub.publish(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._mavros_cmd_pub is not None:
            self._mavros_cmd_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MavrosRoboatRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
