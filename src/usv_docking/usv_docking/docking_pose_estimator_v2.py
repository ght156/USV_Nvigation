#!/usr/bin/env python3
"""docking_pose_estimator_v2 — 船坞位姿估计器（V2）。

职责：
  - Tag 可见时：TF 查 base_link->dock_frame，换算为 odom->dock 并锁存为
    odom->dock_est 锚点（位置 EMA + yaw 环绕 EMA + 跳变拒绝）；
  - Tag 丢失后：锚点不动，用当前 odom->base_link 推算 dock_est->base_link；
  - 广播 odom->dock_est TF（RViz 调试与下游复用）。

发布：
  /docking_v2/dock_pose        (geometry_msgs/PoseStamped, frame_id=dock_est,
                                表示 base_link 在 dock_est 系中的位姿)
  /docking_v2/tag_visible      (std_msgs/Bool)
  /docking_v2/pose_source      (std_msgs/String: VISION / ODOM_PREDICTION / INVALID)
  /docking_v2/measurement_age  (std_msgs/Float32, 距最近视觉观测秒数)

订阅：
  /docking_v2/reset_anchor     (std_msgs/Bool, FSM 进入 ACQUIRE_TAG 时重置锚点)

设计要点：
  - 全部输入走 TF：odom->base_link 由 EKF 广播（30Hz），
    base_link->dock_frame 由 apriltag_localization 经 camera_rear_link 广播；
    不订阅 odom / detections topic。
  - 滤波与跳变拒绝只作用于 odom 系锚点（船坞在 odom 系是静态量）；
    不对相机系/船体系相对量滤波（船转头时相对量本就会跳米级）。
  - dock 朝向用「X 轴投影法」提取（dock_frame 若带 roll 翻转，直接取四元数
    yaw 会得到错误符号；投影法对翻转鲁棒，轴向结论见任务2实测）。
  - 连续 jump_reject_unlock_frames 帧观测都被拒，说明船已大幅移位（换泊位/
    重新 spawn），接受新观测重建锚点。
  - pose 恒按最新可得值发布，是否可用以 pose_source 为准（下游必须判 source）。
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool, Float32, String

import tf2_ros


# ══════════════════════ 平面数学（纯函数，供单测） ══════════════════════

def wrap_angle(angle: float) -> float:
    """弧度环绕到 [-pi, pi)。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compose_2d(ax, ay, ayaw, bx, by, byaw):
    """二维刚体复合 A*B：先 B 后 A（均为 (x, y, yaw)）。"""
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (
        ax + c * bx - s * by,
        ay + s * bx + c * by,
        wrap_angle(ayaw + byaw),
    )


def inverse_2d(x, y, yaw):
    """二维刚体逆变换。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, wrap_angle(-yaw))


def quat_to_planar_yaw(qx, qy, qz, qw) -> float:
    """坐标系朝向的平面 yaw：X 轴经四元数旋转后投影到 XY 平面取 atan2。

    对带 roll/pitch 翻转的 frame（如 dock_frame 可能 roll=±pi）比直接
    提取四元数 yaw 鲁棒。
    """
    # v = q * (1,0,0) * q^-1 的展开式
    vx = 1.0 - 2.0 * (qy * qy + qz * qz)
    vy = 2.0 * (qx * qy + qz * qw)
    return math.atan2(vy, vx)


def yaw_to_quat(yaw):
    """平面 yaw -> 四元数 (x, y, z, w)。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def transform_to_2d(t: TransformStamped):
    """TransformStamped -> (x, y, planar_yaw)。"""
    yaw = quat_to_planar_yaw(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w,
    )
    return (t.transform.translation.x, t.transform.translation.y, yaw)


# ══════════════════════ 节点 ══════════════════════

SRC_VISION = "VISION"
SRC_PREDICTION = "ODOM_PREDICTION"
SRC_INVALID = "INVALID"


class DockingPoseEstimatorV2(Node):
    """船坞位姿估计器 V2。"""

    def __init__(self):
        super().__init__("docking_pose_estimator_v2")

        # ── 坐标系 ──
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("dock_frame", "dock_frame")
        self.declare_parameter("dock_est_frame", "dock_est")
        self.declare_parameter("tf_timeout_sec", 0.1)
        # odom 帧龄期容忍：EKF 低帧率/抖动下 0.5s 会频繁误判 INVALID 连带废锚点；
        # 推算本就允许数秒陈旧 odom（USV 慢速），3s 远比掉 INVALID 强
        self.declare_parameter("odom_tf_timeout_sec", 3.0)
        self.declare_parameter("broadcast_tf", True)

        # ── 话题 ──
        self.declare_parameter("dock_pose_topic", "/docking_v2/dock_pose")
        self.declare_parameter("tag_visible_topic", "/docking_v2/tag_visible")
        self.declare_parameter("pose_source_topic", "/docking_v2/pose_source")
        self.declare_parameter("measurement_age_topic", "/docking_v2/measurement_age")
        self.declare_parameter("reset_anchor_topic", "/docking_v2/reset_anchor")

        # ── 运行 ──
        self.declare_parameter("publish_rate", 20.0)

        # ── 滤波 ──
        self.declare_parameter("ema_alpha_position", 0.25)
        self.declare_parameter("ema_alpha_yaw", 0.15)
        self.declare_parameter("max_position_jump", 0.60)
        self.declare_parameter("max_yaw_jump_deg", 20.0)
        self.declare_parameter("jump_reject_unlock_frames", 5)
        # 首锚共识：单码远距离噪声/单双码系统差可达 0.7m，首帧即锚会把偏差
        # 固化并被跳变拒绝自我强化（2026-07-29 实测 dp=0.62~0.72m 持续拒绝）
        self.declare_parameter("seed_frames", 8)
        self.declare_parameter("seed_max_spread_m", 0.40)
        self.declare_parameter("seed_max_yaw_spread_deg", 15.0)

        # ── 超时 ──
        self.declare_parameter("tag_timeout", 0.30)
        # 推算硬上限：ABORT_EXIT/UNDOCK 全靠推算驶出，须 >= 安全节点撤离窗口(60s)
        self.declare_parameter("odom_prediction_timeout", 70.0)

        p = self.get_parameter
        self._odom_frame = p("odom_frame").value
        self._base_frame = p("base_frame").value
        self._dock_frame = p("dock_frame").value
        self._dock_est_frame = p("dock_est_frame").value
        self._tf_timeout = Duration(seconds=p("tf_timeout_sec").value)
        self._odom_tf_timeout = float(p("odom_tf_timeout_sec").value)
        self._broadcast_tf = bool(p("broadcast_tf").value)
        self._alpha_pos = float(p("ema_alpha_position").value)
        self._alpha_yaw = float(p("ema_alpha_yaw").value)
        self._max_pos_jump = float(p("max_position_jump").value)
        self._max_yaw_jump = math.radians(float(p("max_yaw_jump_deg").value))
        self._unlock_frames = int(p("jump_reject_unlock_frames").value)
        self._seed_frames = int(p("seed_frames").value)
        self._seed_max_spread = float(p("seed_max_spread_m").value)
        self._seed_max_yaw_spread = math.radians(
            float(p("seed_max_yaw_spread_deg").value)
        )
        self._tag_timeout = float(p("tag_timeout").value)
        self._prediction_timeout = float(p("odom_prediction_timeout").value)

        qos = QoSProfile(depth=10)
        self._dock_pose_pub = self.create_publisher(
            PoseStamped, p("dock_pose_topic").value, qos
        )
        self._tag_visible_pub = self.create_publisher(
            Bool, p("tag_visible_topic").value, qos
        )
        self._pose_source_pub = self.create_publisher(
            String, p("pose_source_topic").value, qos
        )
        self._measurement_age_pub = self.create_publisher(
            Float32, p("measurement_age_topic").value, qos
        )

        self.create_subscription(
            Bool, p("reset_anchor_topic").value, self._reset_anchor_cb, qos
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 锚点 (x, y, yaw)（odom 系，滤波后）；None = 未建立
        self._anchor = None
        self._reject_count = 0
        self._seed_buf = []   # 首锚共识窗口 [(x,y,yaw), ...]
        self._reject_ema = None  # 被拒观测簇的 EMA（解锁时吸附目标）
        self._last_vision_stamp = None  # rclpy.time.Time，最近有效视觉观测
        self._prev_source = SRC_INVALID
        # 最近一次输出（x, y, yaw）（dock_est 系下的 base_link）
        self._last_output = None

        rate = float(p("publish_rate").value)
        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f"docking_pose_estimator_v2 已启动：{self._odom_frame}/"
            f"{self._base_frame}/{self._dock_frame} -> {self._dock_est_frame}"
        )

    # ── 锚点重置 ──
    def _reset_anchor_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info("锚点重置：清空 odom->dock_est 锁存与滤波状态")
            self._anchor = None
            self._reject_count = 0
            self._seed_buf = []
            self._reject_ema = None
            self._last_vision_stamp = None
            self._last_output = None

    # ── TF 查询（返回 (2d_pose, stamp_age_sec)，失败/过期返回 (None, inf)）──
    def _lookup_2d(self, target: str, source: str, now: Time):
        try:
            t = self._tf_buffer.lookup_transform(
                target, source, Time(), timeout=self._tf_timeout
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None, float("inf")
        age = (now - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        return transform_to_2d(t), age

    # ── 主循环 ──
    def _update(self):
        now = self.get_clock().now()

        odom_pose, odom_age = self._lookup_2d(
            self._odom_frame, self._base_frame, now
        )
        odom_ok = odom_pose is not None and odom_age <= self._odom_tf_timeout
        if not odom_ok:
            self.get_logger().warn(
                f"odom TF 不可用（age={odom_age:.2f}s），输出 INVALID",
                throttle_duration_sec=2.0,
            )

        dock_pose, dock_age = self._lookup_2d(
            self._base_frame, self._dock_frame, now
        )
        vision_ok = dock_pose is not None and dock_age <= self._tag_timeout

        # ── 锚点更新（仅视觉新鲜且 odom 正常时）──
        if vision_ok and odom_ok:
            raw = compose_2d(*odom_pose, *dock_pose)
            self._update_anchor(raw)
            self._last_vision_stamp = now - Duration(seconds=dock_age)

        # ── 观测龄期与来源分级 ──
        if self._last_vision_stamp is None:
            measurement_age = float("inf")
        else:
            measurement_age = (now - self._last_vision_stamp).nanoseconds * 1e-9

        if not odom_ok or self._anchor is None:
            source = SRC_INVALID
        elif measurement_age <= self._tag_timeout:
            source = SRC_VISION
        elif measurement_age <= self._prediction_timeout:
            source = SRC_PREDICTION
        else:
            source = SRC_INVALID

        if source != self._prev_source:
            self.get_logger().info(
                f"pose_source: {self._prev_source} -> {source}"
                f"（age={measurement_age:.2f}s）"
            )
            self._prev_source = source

        # ── 推算 dock_est->base_link 并发布 ──
        if odom_ok and self._anchor is not None:
            inv = inverse_2d(*self._anchor)
            self._last_output = compose_2d(*inv, *odom_pose)

        self._publish(now, measurement_age, vision_ok, source)

        if self._broadcast_tf and self._anchor is not None:
            self._broadcast_anchor(now)

    # ── 锚点滤波：共识播种 + EMA + 跳变拒绝（含簇吸附解锁）──
    def _update_anchor(self, raw):
        if self._anchor is None:
            # 首锚共识：收满 seed_frames 帧后取位置中位数/yaw 矢量平均；
            # 离散度超门限则滑窗继续收（双码切换/远距离抖动期不播种）
            self._seed_buf.append(raw)
            if len(self._seed_buf) < self._seed_frames:
                return
            xs = sorted(r[0] for r in self._seed_buf)
            ys = sorted(r[1] for r in self._seed_buf)
            mx = xs[len(xs) // 2]
            my = ys[len(ys) // 2]
            spread = max(
                math.hypot(r[0] - mx, r[1] - my) for r in self._seed_buf
            )
            cy = math.atan2(
                sum(math.sin(r[2]) for r in self._seed_buf),
                sum(math.cos(r[2]) for r in self._seed_buf),
            )
            yaw_spread = max(
                abs(wrap_angle(r[2] - cy)) for r in self._seed_buf
            )
            if spread > self._seed_max_spread or yaw_spread > self._seed_max_yaw_spread:
                self._seed_buf.pop(0)
                self.get_logger().warn(
                    f"首锚离散度超限（dp={spread:.2f}m dyaw={math.degrees(yaw_spread):.1f}°），"
                    "滑窗继续集帧",
                    throttle_duration_sec=1.0,
                )
                return
            self._anchor = (mx, my, cy)
            self._seed_buf = []
            self._reject_count = 0
            self._reject_ema = None
            self.get_logger().info(
                f"锚点建立（{self._seed_frames}帧共识，离散 {spread:.2f}m/"
                f"{math.degrees(yaw_spread):.1f}°）：odom ({mx:.2f}, {my:.2f}, "
                f"{math.degrees(cy):.1f}°)"
            )
            return

        dp = math.hypot(raw[0] - self._anchor[0], raw[1] - self._anchor[1])
        dyaw = abs(wrap_angle(raw[2] - self._anchor[2]))

        if dp > self._max_pos_jump or dyaw > self._max_yaw_jump:
            self._reject_count += 1
            # 被拒观测若彼此一致成簇，说明锚点本身偏了；解锁时吸附到簇 EMA
            # （比直接吞最新单帧平滑，且正中双码均值真值）
            if self._reject_ema is None:
                self._reject_ema = raw
            else:
                self._reject_ema = (
                    self._reject_ema[0] + 0.4 * (raw[0] - self._reject_ema[0]),
                    self._reject_ema[1] + 0.4 * (raw[1] - self._reject_ema[1]),
                    wrap_angle(
                        self._reject_ema[2]
                        + 0.4 * wrap_angle(raw[2] - self._reject_ema[2])
                    ),
                )
            if self._reject_count >= self._unlock_frames:
                self.get_logger().warn(
                    f"连续 {self._reject_count} 帧跳变，锚点重定位到拒绝簇均值 "
                    f"({self._reject_ema[0]:.2f}, {self._reject_ema[1]:.2f})"
                    "——原锚点有偏或船已大幅移位"
                )
                self._anchor = self._reject_ema
                self._reject_count = 0
                self._reject_ema = None
            else:
                self.get_logger().warn(
                    f"拒绝锚点跳变：dp={dp:.2f}m dyaw={math.degrees(dyaw):.1f}°"
                    f"（{self._reject_count}/{self._unlock_frames}）",
                    throttle_duration_sec=1.0,
                )
            return

        self._reject_count = 0
        self._reject_ema = None
        ax, ay, ayaw = self._anchor
        self._anchor = (
            ax + self._alpha_pos * (raw[0] - ax),
            ay + self._alpha_pos * (raw[1] - ay),
            wrap_angle(ayaw + self._alpha_yaw * wrap_angle(raw[2] - ayaw)),
        )

    # ── 发布 ──
    def _publish(self, now: Time, age: float, visible: bool, source: str):
        visible_msg = Bool()
        visible_msg.data = bool(visible)
        self._tag_visible_pub.publish(visible_msg)

        source_msg = String()
        source_msg.data = source
        self._pose_source_pub.publish(source_msg)

        age_msg = Float32()
        age_msg.data = float(age)
        self._measurement_age_pub.publish(age_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = self._dock_est_frame
        if self._last_output is not None:
            x, y, yaw = self._last_output
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            qx, qy, qz, qw = yaw_to_quat(yaw)
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
        self._dock_pose_pub.publish(pose_msg)

    def _broadcast_anchor(self, now: Time):
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._dock_est_frame
        t.transform.translation.x = self._anchor[0]
        t.transform.translation.y = self._anchor[1]
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(self._anchor[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DockingPoseEstimatorV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
