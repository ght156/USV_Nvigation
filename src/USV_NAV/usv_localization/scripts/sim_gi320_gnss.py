#!/usr/bin/env python3
"""Simulate BW GI320 GNGGA + INSPVAA near a map.yaml ref_gnss anchor."""
from __future__ import annotations

import math

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from bw_gi320_driver.msg import Gi320Gngga, Gi320Inspvaa


WGS84_A = 6378137.0


def enu_to_latlon(east: float, north: float, lat0: float, lon0: float):
    dlat = north / WGS84_A
    dlon = east / (WGS84_A * math.cos(math.radians(lat0)))
    return lat0 + math.degrees(dlat), lon0 + math.degrees(dlon)


class SimGi320Gnss(Node):
    def __init__(self) -> None:
        super().__init__('sim_gi320_gnss')
        map_yaml = self.declare_parameter('map_config_yaml', '').value
        ref_key = self.declare_parameter('map_origin_ref_key', 'ref_gnss_10').value
        self._rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)
        self._radius_m = float(self.declare_parameter('path_radius_m', 20.0).value)
        self._speed_mps = float(self.declare_parameter('speed_mps', 1.0).value)
        self._fix_type = int(self.declare_parameter('fix_type', 4).value)
        self._hdop = float(self.declare_parameter('hdop', 0.8).value)
        # Offset from map anchor at t=0 (east, north) so lock is not exactly at origin.
        self._e0 = float(self.declare_parameter('start_east_m', 30.0).value)
        self._n0 = float(self.declare_parameter('start_north_m', 40.0).value)

        if not map_yaml:
            raise RuntimeError('map_config_yaml is required')
        with open(map_yaml, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        arr = cfg[ref_key]
        self._lon0 = float(arr[0])
        self._lat0 = float(arr[1])

        self._pub_gga = self.create_publisher(
            Gi320Gngga, '/gi320/gngga', qos_profile_sensor_data)
        self._pub_ins = self.create_publisher(
            Gi320Inspvaa, '/gi320/inspvaa', qos_profile_sensor_data)

        self._t0 = self.get_clock().now()
        self._odom_m = 0.0
        period = 1.0 / max(1.0, self._rate_hz)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'sim GI320 around anchor lat={self._lat0:.8f} lon={self._lon0:.8f} '
            f'start ENU=({self._e0:.1f},{self._n0:.1f}) r={self._radius_m:.1f}m'
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        # Circle centered at (e0, n0)
        omega = self._speed_mps / max(1e-3, self._radius_m)
        ang = omega * t
        east = self._e0 + self._radius_m * math.cos(ang)
        north = self._n0 + self._radius_m * math.sin(ang)
        # Heading: tangent, CCW from north toward east (matches node conversion)
        # velocity direction: (-r*sin, r*cos) → angle from north...
        ve = -self._radius_m * omega * math.sin(ang)
        vn = self._radius_m * omega * math.cos(ang)
        yaw_deg = math.degrees(math.atan2(ve, vn))  # from north, CCW to east

        lat, lon = enu_to_latlon(east, north, self._lat0, self._lon0)
        self._odom_m += self._speed_mps * (1.0 / self._rate_hz)
        stamp = now.to_msg()

        gga = Gi320Gngga()
        gga.stamp = stamp
        gga.latitude = lat
        gga.longitude = lon
        gga.altitude = 10.0
        gga.fix_type = self._fix_type
        gga.satellite_num = 18
        gga.hdop = float(self._hdop)
        gga.odometer = self._odom_m
        self._pub_gga.publish(gga)

        ins = Gi320Inspvaa()
        ins.stamp = stamp
        ins.latitude = lat
        ins.longitude = lon
        ins.altitude = 10.0
        ins.velocity_e = ve
        ins.velocity_n = vn
        ins.velocity_u = 0.0
        ins.roll = 0.0
        ins.pitch = 0.0
        ins.yaw = yaw_deg
        ins.odometer = self._odom_m
        self._pub_ins.publish(ins)


def main() -> None:
    rclpy.init()
    node = SimGi320Gnss()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
