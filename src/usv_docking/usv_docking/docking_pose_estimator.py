#!/usr/bin/env python3
"""docking_pose_estimator — 船坞位姿估计器。

职责：
  - Tag 可见时：TF 查 base_link->dock_frame，换算为 odom->dock 并锁存为
    odom->dock_est 锚点（位置 EMA + yaw 环绕 EMA + 跳变拒绝）；
  - Tag 丢失后：锚点不动，用当前 odom->base_link 推算 dock_est->base_link；
  - 广播 odom->dock_est TF（RViz 调试与下游复用）。

发布：
  /docking/dock_pose        (geometry_msgs/PoseStamped, frame_id=dock_est,
                                表示 base_link 在 dock_est 系中的位姿)
  /docking/tag_visible      (std_msgs/Bool)
  /docking/pose_source      (std_msgs/String: VISION / ODOM_PREDICTION / INVALID)
  /docking/measurement_age  (std_msgs/Float32, 距最近视觉观测秒数)

订阅：
  /docking/reset_anchor     (std_msgs/Bool, FSM 进入 ACQUIRE_TAG 时重置锚点)

设计要点：
  - 全部输入走 TF：odom->base_link 由 EKF 广播（30Hz），
    base_link->dock_frame 由 apriltag_localization 经 camera_rear_link 广播；
    不订阅 odom / detections topic。
  - 滤波与跳变拒绝只作用于 odom 系锚点（船坞在 odom 系是静态量）；
    不对相机系/船体系相对量滤波（船转头时相对量本就会跳米级）。
  - dock 朝向用「X 轴投影法」提取（dock_frame 若带 roll 翻转，直接取四元数
    yaw 会得到错误符号；投影法对翻转鲁棒，轴向结论见任务2实测）。
  - 连续 jump_reject_unlock_frames 帧观测都被拒，说明船已大幅移位（换泊位/
    重新 spawn），接受新观测重建锚点；但簇 yaw 与锚点差超
    reloc_max_yaw_jump_deg 时判定为掠射角观测伪影，拒绝重定位保持原锚点
    （伪影是一过性的：簇持续超 reloc_guard_escape_sec 仍判定真实移位，
    守卫放行）。
  - pose 恒按最新可得值发布，是否可用以 pose_source 为准（下游必须判 source）。
"""

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool, Float32, String

import tf2_ros

# wrap_angle / SRC_* 统一在 usv_docking.docking_common 定义（此处 import 复用，
# 保持本模块命名空间可导入 wrap_angle：test_planar_math 直接引用）
from usv_docking.docking_common import (  # noqa: F401
    SRC_INVALID,
    SRC_PREDICTION,
    SRC_VISION,
    wrap_angle,
)


# ══════════════════════ 平面数学（纯函数，供单测） ══════════════════════


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


class DockingPoseEstimator(Node):
    """船坞位姿估计器。"""

    def __init__(self):
        super().__init__("docking_pose_estimator")

        # ── 坐标系 ──
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("dock_frame", "dock_frame")
        self.declare_parameter("dock_est_frame", "dock_est")
        # TF 查询最长等待：必须保持 0（非阻塞）。本节点是 20Hz 定时器内查询，
        # 目标帧不在缓存里时（如未检测到 Tag 的 base_link->dock_frame）阻塞等待
        # 会占满定时器周期→单线程 executor 被定时器回调占满→/tf 订阅回调被
        # 饿死→自己的 tf buffer 落后数秒→新鲜 odom 被误判 INVALID
        # （2026-09-11 实船实测 age 恒 3.07s，锚点永建不起来）。
        self.declare_parameter("tf_timeout_sec", 0.0)
        # odom 帧龄期容忍：EKF 低帧率/抖动下 0.5s 会频繁误判 INVALID 连带废锚点；
        # 推算本就允许数秒陈旧 odom（USV 慢速），3s 远比掉 INVALID 强
        self.declare_parameter("odom_tf_timeout_sec", 3.0)
        self.declare_parameter("broadcast_tf", True)

        # ── 话题 ──
        self.declare_parameter("dock_pose_topic", "/docking/dock_pose")
        self.declare_parameter("tag_visible_topic", "/docking/tag_visible")
        self.declare_parameter("pose_source_topic", "/docking/pose_source")
        self.declare_parameter("measurement_age_topic", "/docking/measurement_age")
        self.declare_parameter("reset_anchor_topic", "/docking/reset_anchor")

        # ── 运行 ──
        self.declare_parameter("publish_rate", 20.0)

        # ── 滤波 ──
        self.declare_parameter("ema_alpha_position", 0.25)
        self.declare_parameter("ema_alpha_yaw", 0.15)
        self.declare_parameter("max_position_jump", 0.60)
        self.declare_parameter("max_yaw_jump_deg", 20.0)
        # 重定位守卫（2026-10-07）：远距/掠射角观测的 yaw 系统性偏 20~33°
        # （透视畸变），成簇后会触发跳变解锁把锚点带到错误朝向，控制器追
        # phantom 越走越歪。浮动坞真实朝向变化缓慢，拒绝簇 EMA 的 yaw 与
        # 锚点差超此值必是观测伪影：拒绝重定位（保持原锚点，清零计数），
        # 位置再偏也不重定位
        self.declare_parameter("reloc_max_yaw_jump_deg", 10.0)
        # 守卫持续窗口逃逸（2026-10-07 评审）：伪影是船转过该方位的一过性现象，
        # 真实移位（换泊位/重启，reset_anchor_on_start=false 时无人救）会产生
        # 持续簇——拒绝簇持续存在超此秒数则判定真实移位，守卫放行重定位
        # 2026-10-08：3.0→8.0，缓转向时伪影簇常持续 3s+ 被误放行（当日 8 次）
        self.declare_parameter("reloc_guard_escape_sec", 8.0)
        # 重定位可信度门（2026-10-07 现场）：远场/单码观测的坞位姿不可信——
        # 实测远场重定位 dp 0.77~1.21m 而 dyaw < 10°（守卫只判 yaw，全放过），
        # 锚点被搬走 1m ⇒ 整个 dock 系错 ⇒ 报出 5~7m 假横偏把船带走。
        # ① 距离门：船-坞距离超此值不重定位（相机 9~10m 极限内也留余量）
        # 2026-10-08：6.0→4.0，4~6m 中场噪声仍 ±0.5m（当日 91s 重定位 25 次）
        self.declare_parameter("reloc_max_dist_m", 4.0)
        # ② 双码门：只在"明确只看到一个码"时拦（看不到码时观测本身也无效，
        # 不据此拦，避免影响离线单测与无 apriltag 节点的场景）
        self.declare_parameter("reloc_require_dual_tag", True)
        # 2026-10-08：5→12（8.5Hz 下 0.6s→1.4s），远场噪声簇本就连续，
        # 5 帧等于噪声稍持续就放行；真移位簇无限持续，12 帧照样解锁
        self.declare_parameter("jump_reject_unlock_frames", 12)
        # 首锚共识：单码远距离噪声/单双码系统差可达 0.7m，首帧即锚会把偏差
        # 固化并被跳变拒绝自我强化（2026-07-29 实测 dp=0.62~0.72m 持续拒绝）
        self.declare_parameter("seed_frames", 8)
        self.declare_parameter("seed_max_spread_m", 0.40)
        self.declare_parameter("seed_max_yaw_spread_deg", 15.0)

        # ── 超时 ──
        self.declare_parameter("tag_timeout", 0.30)
        # 推算硬上限：ABORT_EXIT/UNDOCK 全靠推算驶出，须 >= 安全节点撤离窗口
        # （safety.abort_exit_odom_timeout=60s）。2026-10-07 改名：本参数与 safety 的
        # odom_prediction_timeout(3s=正常运行窗口) 同名但语义完全不同，容易被误
        # "统一"成同一个值（改小瘫痪撤离、改大废掉安全网）；不等式由
        # test/test_param_consistency.py 兜底
        self.declare_parameter("anchor_prediction_timeout_sec", 70.0)

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
        self._reloc_max_yaw_jump = math.radians(
            float(p("reloc_max_yaw_jump_deg").value)
        )
        self._reloc_guard_escape = float(p("reloc_guard_escape_sec").value)
        self._reloc_max_dist = float(p("reloc_max_dist_m").value)
        self._reloc_require_dual = bool(p("reloc_require_dual_tag").value)
        self._unlock_frames = int(p("jump_reject_unlock_frames").value)
        self._seed_frames = int(p("seed_frames").value)
        self._seed_max_spread = float(p("seed_max_spread_m").value)
        self._seed_max_yaw_spread = math.radians(
            float(p("seed_max_yaw_spread_deg").value)
        )
        self._tag_timeout = float(p("tag_timeout").value)
        self._prediction_timeout = float(
            p("anchor_prediction_timeout_sec").value
        )

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
        # 拒绝簇首次出现时刻（重定位守卫持续窗口逃逸用；簇清零时同步清 None，
        # 但守卫拒绝路径保留——簇还在累积持续时长）
        self._reject_cluster_start = None
        self._last_vision_stamp = None  # rclpy.time.Time，最近有效视觉观测
        self._prev_source = SRC_INVALID
        # 最近一次输出（x, y, yaw）（dock_est 系下的 base_link）
        self._last_output = None

        rate = float(p("publish_rate").value)
        self._period = 1.0 / rate
        self._overrun_warned = 0
        self.create_timer(self._period, self._update)

        self.get_logger().info(
            f"docking_pose_estimator 已启动：{self._odom_frame}/"
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
            self._reject_cluster_start = None
            self._last_vision_stamp = None
            self._last_output = None

    # ── TF 查询（返回 (2d_pose, stamp_age_sec)，失败/过期返回 (None, inf)）──
    def _lookup_2d(self, target: str, source: str, now: Time):
        """查询 target->source 的平面位姿与时间戳龄期。

        必须非阻塞（tf_timeout_sec 默认 0）：本节点在 20Hz 定时器里查询，若目标
        帧不在缓存里，阻塞等待会占满整个定时器周期，单线程 executor 随即被定时器
        回调占满、/tf 订阅回调被饿死，自己的 tf buffer 就会落后实时流数秒，把
        新鲜的 odom 误判成 INVALID（2026-09-11 实测龄期恒 3.07s）。定时器本身
        20Hz 重试，阻塞等待没有任何收益。main() 还会用多线程 executor 兜底。
        """
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
        t_cycle = time.perf_counter()
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
            # 船-坞距离（dock 在 base_link 系里的模长）用于重定位可信度门
            self._update_anchor(raw, dist=math.hypot(dock_pose[0], dock_pose[1]))
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

        # 周期超时看门狗：回调一旦拖过两个周期，/tf 订阅回调就会开始挨饿，
        # 早点报出来，别等到 odom 被误判 INVALID 才发现（2026-09-11 缺陷）
        cost = time.perf_counter() - t_cycle
        if cost > 2.0 * self._period:
            self._overrun_warned += 1
            self.get_logger().warn(
                f"本周期耗时 {cost * 1e3:.0f}ms 超过定时器周期 "
                f"{self._period * 1e3:.0f}ms ×2，TF 回调可能挨饿（已 {self._overrun_warned} 次）",
                throttle_duration_sec=5.0,
            )

    # ── 锚点滤波：共识播种 + EMA + 跳变拒绝（含簇吸附解锁）──
    # apriltag_localization 为每个码各发一个 TF 帧（实测 2026-10-07：8.5Hz），
    # 用它们判断"当前能看到几个码"，不需要引入 m_common 消息依赖
    _TAG_FRAMES = ("apriltag_0", "apriltag_1")

    def _visible_tag_count(self, now: Time) -> int:
        """当前可见的单码数（非阻塞查 TF；查不到/过期不算）。"""
        n = 0
        for frame in self._TAG_FRAMES:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._base_frame, frame, Time(), timeout=Duration(seconds=0.0)
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            age = (now - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
            if age <= self._tag_timeout:
                n += 1
        return n

    def _reloc_trustworthy(self, dist) -> bool:
        """重定位可信度门：远场 / 明确单码都拒绝改写锚点。

        - 距离门：dist 有效且 > reloc_max_dist_m ⇒ 拒绝（远场坞位姿不可信）
        - 双码门：只在"明确只看到一个码"（n==1）时拒绝；n==0 表示查不到码
          （离线单测 / 无 apriltag 节点），此时观测本身也无效，不据此拦
        """
        if dist is not None and dist > self._reloc_max_dist:
            self.get_logger().warn(
                f"拒绝锚点重定位：距坞 {dist:.1f}m > "
                f"{self._reloc_max_dist:.1f}m，远场坞位姿不可信",
                throttle_duration_sec=2.0,
            )
            return False
        if self._reloc_require_dual:
            n = self._visible_tag_count(self.get_clock().now())
            if n == 1:
                self.get_logger().warn(
                    "拒绝锚点重定位：当前仅单码可见（单双码系统差可达 ~1m），"
                    "保持原锚点",
                    throttle_duration_sec=2.0,
                )
                return False
        return True

    def _update_anchor(self, raw, dist=None):
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
            self._reject_cluster_start = None
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
                if self._reject_cluster_start is None:
                    self._reject_cluster_start = self.get_clock().now()
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
                # 重定位守卫（2026-10-07）：簇 yaw 与锚点差过大疑似掠射角/远距
                # 观测伪影（浮动坞真实朝向变化缓慢）。持续窗口逃逸：伪影是
                # 一过性的，簇持续超 reloc_guard_escape_sec 判定真实移位
                # （换泊位/重启），守卫放行；否则保持原锚点、清零计数与簇 EMA
                # （但保留 _reject_cluster_start，簇还在累积持续时长）
                dyaw_reloc = abs(
                    wrap_angle(self._reject_ema[2] - self._anchor[2])
                )
                if dyaw_reloc > self._reloc_max_yaw_jump:
                    cluster_age = (
                        self.get_clock().now() - self._reject_cluster_start
                    ).nanoseconds * 1e-9
                    if cluster_age > self._reloc_guard_escape:
                        self.get_logger().warn(
                            f"簇持续 {cluster_age:.1f}s（> "
                            f"{self._reloc_guard_escape:.1f}s 守卫窗口），"
                            "判定真实移位，守卫放行重定位"
                        )
                    else:
                        self.get_logger().warn(
                            f"拒绝锚点重定位：簇艏向偏差 "
                            f"{math.degrees(dyaw_reloc):.1f}° > "
                            f"{math.degrees(self._reloc_max_yaw_jump):.1f}°，"
                            "疑似掠射角观测伪影，保持原锚点",
                            throttle_duration_sec=2.0,
                        )
                        self._reject_count = 0
                        self._reject_ema = None
                        return
                else:
                    self.get_logger().warn(
                        f"连续 {self._reject_count} 帧跳变，锚点重定位到拒绝簇均值 "
                        f"({self._reject_ema[0]:.2f}, {self._reject_ema[1]:.2f})"
                        "——原锚点有偏或船已大幅移位"
                    )
                # 可信度门（2026-10-07 现场，见参数处注释）：远场 / 明确单码的观测
                # 不足以改写锚点（锚点一歪，dock 系全歪，控制律会追假目标把船带走）
                if not self._reloc_trustworthy(dist):
                    self._reject_count = 0
                    self._reject_ema = None
                    return
                self._anchor = self._reject_ema
                self._reject_count = 0
                self._reject_ema = None
                self._reject_cluster_start = None
            else:
                self.get_logger().warn(
                    f"拒绝锚点跳变：dp={dp:.2f}m dyaw={math.degrees(dyaw):.1f}°"
                    f"（{self._reject_count}/{self._unlock_frames}）",
                    throttle_duration_sec=1.0,
                )
            return

        self._reject_count = 0
        self._reject_ema = None
        self._reject_cluster_start = None
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
    node = DockingPoseEstimator()
    # 多线程 executor：tf2_ros 的 /tf、/tf_static 订阅建在 ReentrantCallbackGroup，
    # 本节点定时器在默认互斥组，多线程下定时器回调再慢也不会饿死 TF 订阅。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
