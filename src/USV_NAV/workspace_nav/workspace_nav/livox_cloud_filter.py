#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# livox_cloud_filter：/livox/lidar 点云自用回波遮蔽（2026-10-21，兴飞船）。
#
# 背景：兴飞船雷达斜装船头（pitch≈-43.9°、朝后上方），船头正前方的清扫杆及其搅动水雾
#   持续被 voxel_layer 标成致命格（实测船体系 x 2.75~3.46 / y -1.0~+1.25 / z -0.15~0.25
#   一团，12s 内清图后重新标回 3300+ 格）。杆随船动，射线清除永远清不掉它
#   （杆/水雾就是射线端点），唯一办法是在进代价图前把这块点云遮掉。
#
# 做法：把点云变换到 mask_frame（默认 base_link，静态 TF 只查一次），丢弃落在任一
#   mask_boxes 内的点，原坐标系原样转发（只删点，不动坐标/字段/时间戳）。
#   盒外点一律不动 → 左右舷侧真实物体（坞壁/浮筒）不受影响。
#
# 参数：
#   input_topic   输入点云（默认 /livox/lidar）
#   output_topic  输出点云（默认 /livox/lidar_filtered，代价图观测源改订阅它）
#   mask_frame    遮蔽盒所在坐标系（默认 base_link）
#   mask_boxes    扁平 double 数组，每 6 个数一盒 [xmin,ymin,zmin,xmax,ymax,zmax]，
#                 支持在线改（立即生效）。默认盒覆盖船头正前清扫杆/水雾区：
#                 x 2.70~3.70（船头 2.7 之前 0~1m）、y -1.20~+1.40、z -0.40~0.65
#                 （下贴观测源 min_obstacle_height -0.35，上盖 voxel 层 max_obstacle_height 0.6）。
#
# 注意：本节点若挂掉，代价图观测源将无数据（无标记也无清除），launch 里已加 respawn。
# ----------------------------------------------------------------------------------------------- #
from __future__ import annotations

import math
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

_DEFAULT_BOXES = [2.70, -1.20, -0.40, 3.70, 1.40, 0.65]


def _quat_to_mat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LivoxCloudFilter(Node):
    def __init__(self) -> None:
        super().__init__('livox_cloud_filter')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar_filtered')
        self.declare_parameter('mask_frame', 'base_link')
        self.declare_parameter('mask_boxes', _DEFAULT_BOXES)

        self._mask_frame = self.get_parameter('mask_frame').value
        self._boxes = self._parse_boxes(self.get_parameter('mask_boxes').value)
        self._tf_mat: Optional[np.ndarray] = None  # 4x4，cloud_frame → mask_frame

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 5)
        self._sub = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self._on_cloud, sub_qos)
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(
            f'遮蔽 {len(self._boxes)} 盒（{self._mask_frame} 系）：'
            + '; '.join(f'x{b[0]:.2f}~{b[3]:.2f} y{b[1]:.2f}~{b[4]:.2f} z{b[2]:.2f}~{b[5]:.2f}'
                        for b in self._boxes))

    @staticmethod
    def _parse_boxes(flat: List[float]) -> List[tuple]:
        if len(flat) % 6 != 0:
            raise ValueError(f'mask_boxes 长度须为 6 的倍数，实得 {len(flat)}')
        return [tuple(flat[i:i + 6]) for i in range(0, len(flat), 6)]

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'mask_boxes':
                try:
                    self._boxes = self._parse_boxes(list(p.value))
                except ValueError as e:
                    return SetParametersResult(successful=False, reason=str(e))
                self.get_logger().info(f'mask_boxes 在线更新为 {len(self._boxes)} 盒')
        return SetParametersResult(successful=True)

    def _lookup_tf(self, cloud_frame: str) -> bool:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._mask_frame, cloud_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(
                f'等 {cloud_frame}→{self._mask_frame} TF：{e}', throttle_duration_sec=5.0)
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        m = np.eye(4)
        m[:3, :3] = _quat_to_mat(q.x, q.y, q.z, q.w)
        m[0, 3], m[1, 3], m[2, 3] = t.x, t.y, t.z
        self._tf_mat = m
        self.get_logger().info(
            f'TF {cloud_frame}→{self._mask_frame} 已锁定：'
            f't=({t.x:.3f},{t.y:.3f},{t.z:.3f})（静态外参，只查一次）')
        return True

    @staticmethod
    def _dtype_from_fields(msg: PointCloud2) -> np.dtype:
        # Livox 点云各字段类型不一（float32 + uint8 + uint64），read_points_numpy 不支持，
        # 直接按 fields 布局构造结构化 dtype 解析二进制，过滤后原样打包（字段/时间戳不丢失）。
        code = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
        endian = '>' if msg.is_bigendian else '<'
        names, formats, offsets = [], [], []
        for f in msg.fields:
            names.append(f.name)
            fmt = code[f.datatype]
            formats.append(fmt if f.datatype in (1, 2) else endian + fmt)
            offsets.append(f.offset)
        return np.dtype({'names': names, 'formats': formats,
                         'offsets': offsets, 'itemsize': msg.point_step})

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._tf_mat is None and not self._lookup_tf(msg.header.frame_id):
            return
        arr = np.frombuffer(msg.data, dtype=self._dtype_from_fields(msg),
                            count=msg.width * msg.height)
        arr = arr[np.isfinite(arr['x']) & np.isfinite(arr['y']) & np.isfinite(arr['z'])]
        if arr.shape[0] == 0 or not self._boxes:
            self._pub.publish(msg)
            return
        xyz = np.stack([arr['x'], arr['y'], arr['z'],
                        np.ones(arr.shape[0])]).astype(np.float64)
        base = (self._tf_mat @ xyz)[:3]
        keep = np.ones(arr.shape[0], dtype=bool)
        for xmin, ymin, zmin, xmax, ymax, zmax in self._boxes:
            inside = ((base[0] >= xmin) & (base[0] <= xmax)
                      & (base[1] >= ymin) & (base[1] <= ymax)
                      & (base[2] >= zmin) & (base[2] <= zmax))
            keep &= ~inside
        dropped = int((~keep).sum())
        if dropped == 0:
            self._pub.publish(msg)
            return
        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.is_dense = msg.is_dense
        kept = arr[keep]
        out.height = 1
        out.width = kept.shape[0]
        out.row_step = out.point_step * out.width
        out.data = kept.tobytes()
        self._pub.publish(out)
        self.get_logger().debug(f'滤除 {dropped}/{arr.shape[0]} 点')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxCloudFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
