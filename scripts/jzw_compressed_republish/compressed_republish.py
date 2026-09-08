#!/usr/bin/env python3
"""compressed_republish — 船上后端相机裸图 → JPEG 压缩流，供主机端低带宽订阅。

背景：
  cleaning_boat.launch.py（apriltag 容器）里的 video_camera 以
  /camera_back_video/color/image_raw（1920x1080@10fps，约 60MB/s 裸图）发布。
  主机端 rviz 一订阅这条裸图，船↔地面无线链路立刻被抽空，图传(RTSP)延迟暴涨。

本节点在「船上本地」订阅裸图 → 降采样 → JPEG 编码 → 以
  /camera_back_video/color/image_raw/compressed  (sensor_msgs/CompressedImage)
发布。apriltag 仍在船内订阅裸图（不跨网），不受影响；主机端改用压缩流即可，
带宽下降一个数量级，图传不被吃掉。

用法（船上，先 source ROS + 工作区）：
  python3 compressed_republish.py
  python3 compressed_republish.py --ros-args -p in_topic:=/camera_back_video/color/image_raw \
    -p out_topic:=/camera_back_video/color/image_raw/compressed -p scale:=0.5 -p quality:=80

主机端 rviz：把 Image/Camera 显示的 Topic 改成
  /camera_back_video/color/image_raw/compressed
（rviz 用 compressed transport 解码；若主机缺 compressed_image_transport，
  把本节点 publish_small_image:=true，rviz 订 out_topic 的 _small 版裸图即可。）
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CompressedRepublish(Node):
    def __init__(self):
        super().__init__("compressed_republish")

        # ── 输入/输出话题 ──
        self.declare_parameter("in_topic", "/camera_back_video/color/image_raw")
        self.declare_parameter("out_topic", "/camera_back_video/color/image_raw/compressed")

        # ── 压缩参数 ──
        self.declare_parameter("jpeg_quality", 80)     # JPEG 质量 1-100
        self.declare_parameter("scale", 1.0)           # 缩放因子（0.5=长宽各减半）
        self.declare_parameter("target_width", 0)      # >0 则覆盖 scale 的宽
        self.declare_parameter("target_height", 0)     # >0 则覆盖 scale 的高
        self.declare_parameter("max_fps", 0.0)         # >0 限帧率，降 CPU/带宽

        # ── 附加：无压缩插件时给 rviz 的降采样裸图 ──
        self.declare_parameter("publish_small_image", False)
        self.declare_parameter("small_topic", "/camera_back_video/color/image_small")
        self.declare_parameter("small_format", "raw")  # raw(bgr8)/jpeg；rviz 无插件时建议 raw

        self._in_topic = str(self.get_parameter("in_topic").value)
        self._out_topic = str(self.get_parameter("out_topic").value)
        self._quality = int(self.get_parameter("jpeg_quality").value)
        self._scale = float(self.get_parameter("scale").value)
        self._tw = int(self.get_parameter("target_width").value)
        self._th = int(self.get_parameter("target_height").value)
        self._max_fps = float(self.get_parameter("max_fps").value)
        self._publish_small = bool(self.get_parameter("publish_small_image").value)
        self._small_topic = str(self.get_parameter("small_topic").value)
        self._small_format = str(self.get_parameter("small_format").value)

        # 订阅：相机发布者 RELIABLE/KEEP_LAST(10)/VOLATILE（apriltag 同款订阅能收到）
        sub_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        # 发布：默认订阅者(ros2 topic hz/echo、rviz)同为 RELIABLE，避免 QoS 不兼容
        pub_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, self._in_topic, self._on_image, sub_qos)
        self._jpeg_pub = self.create_publisher(CompressedImage, self._out_topic, pub_qos)
        self._small_pub = None
        if self._publish_small:
            self._small_pub = self.create_publisher(Image, self._small_topic, pub_qos)

        # 帧率限流
        self._period = 1.0 / self._max_fps if self._max_fps > 0 else 0.0
        self._last = None
        self._frames = 0
        self._rx = 0
        self._enc = "bgr8"

        self.get_logger().info(
            "compressed_republish 启动：%s -> %s (jpeg q=%d, scale=%.2f%s)"
            % (self._in_topic, self._out_topic, self._quality, self._scale,
               ", fps<=%.1f" % self._max_fps if self._max_fps > 0 else "")
        )
        # 心跳诊断：每 5s 打印一次接收/转发数，便于确认链路活跃
        self.create_timer(5.0, self._diag)

    def _diag(self):
        self.get_logger().info(
            "诊断：收到裸图 %d 帧，已转发 %d 帧" % (self._rx, self._frames)
        )

    def _to_bgr(self, msg: Image):
        enc = msg.encoding
        h, w = msg.height, msg.width
        step = msg.step
        if enc == "bgr8":
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, step if step > 0 else w * 3)[:, : w * 3]
            return buf.reshape(h, w, 3)
        if enc == "rgb8":
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, step if step > 0 else w * 3)[:, : w * 3]
            return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        if enc == "mono8":
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, step if step > 0 else w)[:, :w]
            return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
        self.get_logger().warn("截获不支持的编码 %s，丢弃" % enc)
        return None

    def _target_size(self, w, h):
        if self._tw > 0 and self._th > 0:
            return self._tw, self._th
        nw, nh = int(w * self._scale), int(h * self._scale)
        if nw < 1 or nh < 1:
            return w, h
        return nw, nh

    def _on_image(self, msg: Image):
        self._rx += 1
        # 可选限帧率
        if self._period > 0:
            import time as _t
            now = _t.monotonic()
            if self._last is not None and (now - self._last) < self._period:
                return
            self._last = now

        bgr = self._to_bgr(msg)
        if bgr is None:
            return
        nw, nh = self._target_size(bgr.shape[1], bgr.shape[0])
        small = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if not ok:
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = jpg.tobytes()
        self._jpeg_pub.publish(out)

        if self._small_pub is not None:
            simg = Image()
            simg.header = msg.header
            simg.height = nh
            simg.width = nw
            simg.step = nw * 3
            if self._small_format == "raw":
                simg.encoding = "bgr8"
                simg.data = small.tobytes()
            else:
                simg.encoding = "jpeg"
                simg.data = jpg.tobytes()
            self._small_pub.publish(simg)

        self._frames += 1
        if self._frames % 30 == 0:
            self.get_logger().info(
                "已转发 %d 帧 -> %dx%d @ q%d (%.1f KB/帧)" % (
                    self._frames, nw, nh, self._quality, len(out.data) / 1024.0,
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = CompressedRepublish()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGTERM/SIGINT 可能已触发 rclpy 自动 shutdown，重复调用会报错，故忽略
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
