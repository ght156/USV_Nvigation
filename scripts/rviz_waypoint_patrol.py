#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rviz_waypoint_patrol.py

在 RViz2 里多点发目标时，把每次点击的点位记录下来，等你确认“完成”后，按顺序下发，完成巡逻（可循环）。

重要：Nav2 面板（Navigation 2）的「2D Goal Pose」是把目标作为 navigate_to_pose / follow_waypoints
ACTION 直接发给 BT Navigator，不会发布 /goal_pose 话题，因此无法被本节点记录。
请改用 RViz 的「Publish Point」工具（点击发 /clicked_point 话题）来记录点位。

两种下发方式（dispatch_mode）：
  batch（默认）
      把记录的点整串丢给 Nav2 的 follow_waypoints 动作。follow_waypoints 内部就是
      “到达当前点才推进下一个”（顺序+到达判定由 Nav2 完成），跑完一圈后由本节点
      重新下发实现循环巡逻。最省心、最贴合“按顺序发布”。

  single
      单点模式：用 navigate_to_pose 逐个下发，依赖里程计“当前位置是否到达当前点”
      判定（arrival_tolerance），到达后才推进下一个。适合你想自定义到达容差/停稳再走。

  说明：Nav2 没有现成的“循环”动作，follow_waypoints / navigate_through_poses 跑完一圈就会
  结束。循环巡逻的标准做法就是客户端在成功结果后重新下发（本节点已内置 loop）。

交互命令（终端里敲）：
  go / g      开始巡逻：从第 1 个点开始
  list / l    列出当前记录的点
  clear / c   清空已记录的点
  loop on/off 开/关循环巡逻（默认开）
  count N     巡逻 N 圈后自动停止（0 = 无限循环）
  stop/s      取消当前正在执行的任务
  quit / q    退出节点

话题控制（不依赖终端）：
  /rviz_patrol/control  std_msgs/String  同上面的命令文本
  /rviz_patrol/start    std_msgs/Empty   等价于 go

可视化：
  已记录的点与巡逻路径会作为 MarkerArray 发布到 /waypoints（可改用 marker_topic 参数）。

持久化：
  记录/编辑/清空后自动保存到脚本同目录的 patrol_waypoints.json；启动时自动加载（autoload）。
  这样把脚本+该 json 一起拷到 NX/别台机器，上次的点位仍然可用。
  手工命令：save | load | savepath PATH

带朝向的点位来源（避免“点没有航向、靠 auto_yaw 猜”）：
  1) ros_map_tool 航点规划器导出 YAML（waypoint_i: [x, y, yaw(弧度)]），
     用命令 importyaml PATH 或启动参数 -p waypoint_yaml:=PATH 导入；
  2) RViz 工具栏「2D Goal Pose」点击+拖拽本身带朝向。注意：默认话题 /goal_pose
     会被 bt_navigator 直接执行（船会立刻自己开过去）。建议在 RViz 工具属性里把
     Goal Topic 改为 /rviz_patrol/goal_pose，并以 -p goal_topic:=/rviz_patrol/goal_pose
     启动本节点，这样只记录不触发导航。
  注：auto_yaw 只对无朝向来源（Publish Point 的 /clicked_point）生效，
     不会覆盖 2D Goal Pose / YAML 导入点自带的朝向。

坐标说明：RViz 里 2D Goal Pose 发的 /goal_pose 已经是 map 坐标系（frame_id: map），
因此本节点直接把这些 PoseStamped 交给 Nav2，无需再做经纬度转换。

用法示例（先 source 环境）：
  python3 scripts/rviz_waypoint_patrol.py
  或指定整串 / 单点模式：
  python3 scripts/rviz_waypoint_patrol.py \
    --ros-args -p dispatch_mode:=batch -p goal_topic:=/goal_pose
  python3 scripts/rviz_waypoint_patrol.py \
    --ros-args -p dispatch_mode:=single \
    -p arrival_tolerance:=1.5
"""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray


GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
CYAN = "\x1b[36m"
RED = "\x1b[31m"
RESET = "\x1b[0m"

# 默认持久化文件：与脚本同目录，方便拷贝到 NX 后仍能加载上次的点
DEFAULT_WAYPOINTS_FILE = str(Path(__file__).resolve().with_name("patrol_waypoints.json"))


def _green(s: str) -> str:
    return f"{GREEN}{s}{RESET}"


def _yellow(s: str) -> str:
    return f"{YELLOW}{s}{RESET}"


def _cyan(s: str) -> str:
    return f"{CYAN}{s}{RESET}"


def _red(s: str) -> str:
    return f"{RED}{s}{RESET}"


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """四元数 → 绕 z 轴偏航角（rad），用于显示。"""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class RecordedPoint:
    """记录一次 /goal_pose 点击。"""

    __slots__ = ("x", "y", "z", "qx", "qy", "qz", "qw", "frame_id")

    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        frame_id: str,
    ) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        self.frame_id = frame_id

    @property
    def yaw(self) -> float:
        return quat_to_yaw(self.qx, self.qy, self.qz, self.qw)

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw)

    def distance_to(self, other: "RecordedPoint") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def to_pose_stamped(self, timestamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = timestamp
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.x = self.qx
        pose.pose.orientation.y = self.qy
        pose.pose.orientation.z = self.qz
        pose.pose.orientation.w = self.qw
        return pose

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "qx": self.qx,
            "qy": self.qy,
            "qz": self.qz,
            "qw": self.qw,
            "frame_id": self.frame_id,
        }


class RvizPatrolRecorder(Node):
    """记录 RViz 多点点位，并按顺序下发（batch 整串 / single 单点）实现循环巡逻。"""

    def __init__(self) -> None:
        super().__init__("rviz_patrol_recorder")

        # ---- 参数 ------------------------------------------------------------
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/mavros/gps_input/local")
        self.declare_parameter("dispatch_mode", "batch")  # batch | single
        self.declare_parameter("follow_action_name", "follow_waypoints")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter(
            "use_clicked_point", True
        )  # 主通道：Publish Point 工具点一下发一条 /clicked_point
        self.declare_parameter("clicked_topic", "/clicked_point")
        self.declare_parameter(
            "use_move_base_simple", True
        )  # 同时监听 /move_base_simple/goal（2D Nav Goal 工具）作为备用
        self.declare_parameter("move_base_simple_topic", "/move_base_simple/goal")
        self.declare_parameter("control_topic", "/rviz_patrol/control")
        self.declare_parameter("start_topic", "/rviz_patrol/start")
        self.declare_parameter("marker_topic", "/waypoints")
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter(
            "dedup_distance", 0.1
        )  # 相邻两次点击距离小于该值时视为重复，跳过（0=不去重）
        self.declare_parameter(
            "auto_yaw", True
        )  # 记录点时自动把朝向设为“上一→本点行进方向”（无上一点则用船位指向本点）
        self.declare_parameter("loop", True)  # 巡逻：完成一圈后循环
        self.declare_parameter("loop_count", 0)  # 0 = 无限循环
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("waypoints_file", DEFAULT_WAYPOINTS_FILE)
        self.declare_parameter(
            "waypoint_yaml", ""
        )  # 启动时额外导入 ros_map_tool 导出的航点 YAML（waypoint_i: [x, y, yaw]），非空则覆盖已有点位
        self.declare_parameter("autosave", True)  # 记录/编辑后自动保存到文件
        self.declare_parameter("autoload", True)  # 启动时自动加载已保存点位
        self.declare_parameter("arrival_tolerance", 1.5)  # single 模式到达判定 (m)
        self.declare_parameter("arrival_confirm_ticks", 1)  # single 模式连续判定次数
        self.declare_parameter("watch_period", 0.5)  # single 模式巡检周期 (s)
        self.declare_parameter("point_timeout_sec", 0.0)  # single 模式单点超时 (s)，0=禁用
        self.declare_parameter("stall_timeout_sec", 30.0)  # batch 模式：某点位超过该时长未推进到下一标识则判卡滞
        self.declare_parameter("max_retries", 3)  # 卡滞/失败的自动重发次数
        self.declare_parameter("retry_delay_sec", 2.0)  # 重发前的等待 (s)

        self._goal_topic = self.get_parameter("goal_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._mode = (
            self.get_parameter("dispatch_mode").get_parameter_value().string_value
        ).strip().lower() or "batch"
        if self._mode not in ("batch", "single"):
            self.get_logger().fatal(
                f"dispatch_mode 仅支持 batch / single，收到 {self._mode!r}"
            )
            raise SystemExit(1)
        self._follow_action = self.get_parameter("follow_action_name").value
        self._nav_action = self.get_parameter("nav_action_name").value
        self._use_clicked = bool(self.get_parameter("use_clicked_point").value)
        self._clicked_topic = self.get_parameter("clicked_topic").value
        self._use_move_base_simple = bool(
            self.get_parameter("use_move_base_simple").value
        )
        self._simple_goal_topic = self.get_parameter(
            "move_base_simple_topic"
        ).value
        self._control_topic = self.get_parameter("control_topic").value
        self._start_topic = self.get_parameter("start_topic").value
        self._marker_topic = self.get_parameter("marker_topic").value
        self._expected_frame = self.get_parameter("expected_frame").value
        self._dedup = float(self.get_parameter("dedup_distance").value)
        self._auto_yaw = bool(self.get_parameter("auto_yaw").value)
        self._loop = bool(self.get_parameter("loop").value)
        self._loop_count = int(self.get_parameter("loop_count").value)
        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        wf = (
            self.get_parameter("waypoints_file").get_parameter_value().string_value
        ).strip()
        self._waypoints_file = wf or DEFAULT_WAYPOINTS_FILE
        self._waypoint_yaml = (
            self.get_parameter("waypoint_yaml").get_parameter_value().string_value
        ).strip()
        self._autosave = bool(self.get_parameter("autosave").value)
        self._autoload = bool(self.get_parameter("autoload").value)
        self._arrival_tolerance = float(self.get_parameter("arrival_tolerance").value)
        self._arrival_confirm_ticks = int(
            self.get_parameter("arrival_confirm_ticks").value
        )
        self._watch_period = float(self.get_parameter("watch_period").value)
        self._point_timeout = float(self.get_parameter("point_timeout_sec").value)
        self._stall_timeout = float(self.get_parameter("stall_timeout_sec").value)
        self._max_retries = int(self.get_parameter("max_retries").value)
        self._retry_delay = float(self.get_parameter("retry_delay_sec").value)
        self._retries_left = self._max_retries
        self._retry_pending = False
        self._retry_timer = None
        self._last_wp_idx = -1
        self._last_wp_change_wall = time.monotonic()

        # ---- 状态 ------------------------------------------------------------
        self._points: List[RecordedPoint] = []
        self._lock = threading.Lock()

        # 巡逻执行状态；phase: idle | navigating | transition
        self._phase = "idle"
        self._cur = 0  # 当前目标点索引（single 模式）
        self._active_goal = None  # 当前活动 action goal handle
        self._active_client = None  # 当前使用的 action client
        self._near_count = 0
        self._odom_reached = False
        self._point_start_wall = 0.0
        self._last_progress_log = 0.0
        self._pass_count = 0
        self._shutdown = False
        self._no_point_reminder_wall = 0.0

        self._robot = (0.0, 0.0)
        self._have_odom = False
        self._odom_lock = threading.Lock()

        # ---- 订阅 ------------------------------------------------------------
        # 放宽到 best_effort：既能接 RViz 可靠(RELIABLE)发布，也能接 best_effort 发布，
        # 避免因 QoS 不匹配导致“点了没反应”。
        broad_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PoseStamped, self._goal_topic, self._cb_goal_pose, qos_profile=broad_qos
        )
        if self._use_clicked:
            from geometry_msgs.msg import PointStamped

            self.create_subscription(
                PointStamped,
                self._clicked_topic,
                self._cb_clicked_point,
                qos_profile=broad_qos,
            )
        if self._use_move_base_simple:
            self.create_subscription(
                PoseStamped,
                self._simple_goal_topic,
                self._cb_simple_goal,
                qos_profile=broad_qos,
            )
        self.create_subscription(String, self._control_topic, self._cb_control, 10)
        self.create_subscription(Empty, self._start_topic, self._cb_start, 10)
        self.create_subscription(
            Odometry, self._odom_topic, self._cb_odom, qos_profile=broad_qos
        )

        # ---- Action clients --------------------------------------------------
        self._fw_client = ActionClient(self, FollowWaypoints, self._follow_action)
        self._nav_client = ActionClient(self, NavigateToPose, self._nav_action)

        self._watch_timer = self.create_timer(self._watch_period, self._watch_tick)

        # ---- Marker 发布 ----------------------------------------------------
        if self._publish_markers:
            self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)
            self._marker_timer = self.create_timer(1.0, self._publish_route_markers)
        else:
            self._marker_pub = None
            self._marker_timer = None

        # 启动时若存在上次保存的文件，自动加载（支持换机/NX 二次运行仍记得目标点）
        if self._autoload:
            self._load_points()

        # 启动时若指定了地图工具导出的航点 YAML，导入并覆盖当前点位
        if self._waypoint_yaml:
            self._import_waypoint_yaml(self._waypoint_yaml)

        # ---- 启动信息 --------------------------------------------------------
        mode_desc = (
            "batch：整串丢给 follow_waypoints，循环由客户端重新下发"
            if self._mode == "batch"
            else f"single：逐点 navigate_to_pose + 里程计到达判定（<{self._arrival_tolerance}m）"
        )
        self.get_logger().info(_green("多点点位记录·循环巡逻节点已启动"))
        self.get_logger().info(
            f"  目标输入话题   : {self._goal_topic}"
            + (f" / {self._clicked_topic}" if self._use_clicked else "")
            + f"  (frame={self._expected_frame})"
        )
        self.get_logger().info(f"  里程计话题     : {self._odom_topic}")
        self.get_logger().info(f"  下发方式       : {mode_desc}")
        self.get_logger().info(
            f"  控制话题       : {self._control_topic} / {self._start_topic}(可发 Empty)"
        )
        self.get_logger().info(
            f"  循环巡逻       : {'开' if self._loop else '关'} (loop_count={self._loop_count})"
        )
        self.get_logger().info(
            f"  卡滞/重发      : 卡滞判定 {self._stall_timeout:.0f}s，失败/卡住自动重发 {self._max_retries} 次"
        )
        self.get_logger().info(
            f"  自动朝向       : {'开（仅对无朝向来源生效：Publish Point 用上一→本点方向，首点用船位指向；2D Goal Pose/YAML 自带朝向不覆盖）' if self._auto_yaw else '关（保持下发 yaw）'}"
        )
        self.get_logger().info(
            f"  持久化文件     : {self._waypoints_file}"
            + (
                "（自动加载/保存已开）"
                if self._autoload and self._autosave
                else "（自动加载/保存已关）"
            )
        )
        self.get_logger().info(
            _cyan(
                "  记录方式：在 RViz 里切换到「Publish Point」工具（工具栏的十字/点工具），"
                "点击即记录并自动算朝向，无需先 go；go 只负责开始执行。"
                "  ⚠ Nav2 面板的「2D Goal Pose」走 action 不发布话题，无法被记录。"
            )
        )
        self._print_help()

        if sys.stdin is not None and sys.stdin.isatty():
            self._stdin_thread = threading.Thread(
                target=self._stdin_loop, daemon=True, name="patrol-stdin"
            )
            self._stdin_thread.start()

    # ----------------------------------------------------------------------- #
    # 人机交互
    # ----------------------------------------------------------------------- #
    def _print_help(self) -> None:
        self.get_logger().info(
            _yellow(
                "命令：go/g 巡逻 | list/l 列点 | clear/c 清点 | "
                "move X Y | yaw DEG | pnt X Y DEG | edit IDX X Y DEG | "
                "loop on/off | count N | stop/s 取消 | quit/q 退出"
            )
        )
        self.get_logger().info(
            _yellow(
                "  move X Y        改最后一个点的位置（朝向不变）\n"
                "  yaw DEG         改最后一个点的朝向（度；可写 90d 或 1.57r）\n"
                "  yaw auto        取上一→本点的行进方向作为朝向\n"
                "  pnt X Y DEG     追加一个手动点（度）\n"
                "  edit IDX X Y DEG 修改第 IDX 个点（1 起）"
            )
        )
        self.get_logger().info(
            _yellow(
                "  save            立即保存点位到文件\n"
                "  load            从文件重新加载点位\n"
                "  savepath PATH   改用新的持久化文件路径并保存\n"
                "  importyaml PATH 导入 ros_map_tool 导出的航点 YAML（waypoint_i: [x, y, yaw]，替换当前点位）"
            )
        )
        self.get_logger().info(
            _yellow(
                "带朝向打点：RViz 工具属性里把 2D Goal Pose 的话题改为 /rviz_patrol/goal_pose，\n"
                "  并以 -p goal_topic:=/rviz_patrol/goal_pose 启动本节点，即可点击+拖拽记录朝向，\n"
                "  且不触发 bt_navigator 立即导航（/goal_pose 默认会被 Nav2 直接执行）。"
            )
        )

    def _stdin_loop(self) -> None:
        try:
            while not self._shutdown:
                line = sys.stdin.readline()
                if line == "":
                    break
                cmd = line.strip()
                if cmd:
                    self._handle_command(cmd)
        except Exception:
            pass

    def _cb_control(self, msg: String) -> None:
        cmd = msg.data.strip()
        if cmd:
            self._handle_command(cmd)

    def _cb_start(self, _msg: Empty) -> None:
        self._handle_command("go")

    def _handle_command(self, cmd: str) -> None:
        c = cmd.strip().lower()
        parts = c.split()
        if c in ("go", "g", "start", "run"):
            self._start_patrol()
        elif c in ("list", "l"):
            self._print_points()
        elif c in ("clear", "c"):
            with self._lock:
                self._points.clear()
            self.get_logger().info(_yellow(f"已清空记录（现有 {len(self._points)} 点）"))
            self._publish_route_markers()
            self._save_points()
        elif c in ("loop on", "loop"):
            self._loop = True
            self.get_logger().info(_green("循环巡逻：开（回到第 1 个点继续）"))
        elif c in ("loop off", "noloop"):
            self._loop = False
            self.get_logger().info(_green("循环巡逻：关（只跑一圈）"))
        elif c in ("stop", "s", "cancel"):
            self._cancel_current()
        elif c in ("quit", "q", "exit"):
            self.get_logger().info("退出节点…")
            self._shutdown = True
            try:
                rclpy.shutdown()
            except Exception:
                pass
        elif c.startswith("count "):
            try:
                self._loop_count = int(c.split(" ", 1)[1])
                self._loop = True
                self.get_logger().info(_green(f"巡逻圈数 = {self._loop_count}（0=无限）"))
            except (ValueError, IndexError):
                self.get_logger().warning("用法：count N（N≥0，0=无限）")
        elif c.startswith("move ") and len(parts) >= 3:
            self._cmd_move(parts[1], parts[2])
        elif c.startswith("yaw ") and len(parts) >= 2:
            self._cmd_yaw(parts[1])
        elif c.startswith(("pnt ", "add ")) and len(parts) >= 4:
            self._cmd_add_point(parts[1], parts[2], parts[3])
        elif c.startswith("edit ") and len(parts) >= 5:
            self._cmd_edit_point(parts[1], parts[2], parts[3], parts[4])
        elif c in ("save", "sav"):
            self._save_points(force=True)
        elif c in ("load", "reload"):
            self._load_points()
        elif c.startswith(("importyaml ", "loadyaml ", "imp ")) and len(parts) >= 2:
            self._import_waypoint_yaml(cmd.split(" ", 1)[1])
        elif c.startswith(("savepath ", "savefile ")) and len(parts) >= 2:
            self._cmd_save_path(cmd.split(" ", 1)[1])
        elif c in ("help", "h", "?"):
            self._print_help()
        else:
            self.get_logger().warning(f"未知命令：{cmd!r}，输入 help 查看")

    @staticmethod
    def _parse_angle(text: str, default_unit: str = "deg") -> float:
        """把度/弧度字符串解析成弧度。默认按度；可用后缀 d/deg（度）、r/rad（弧度）。"""
        t = text.strip().lower()
        unit = None
        if t.endswith("rad"):
            unit = "rad"
            t = t[: -len("rad")]
        elif t.endswith("deg"):
            unit = "deg"
            t = t[: -len("deg")]
        elif t.endswith("d"):
            unit = "deg"
            t = t[: -len("d")]
        elif t.endswith("r"):
            unit = "rad"
            t = t[: -len("r")]
        val = float(t)
        unit = unit or default_unit
        return math.radians(val) if unit == "deg" else val

    @staticmethod
    def _q_from_yaw(yaw_rad: float) -> Tuple[float, float, float, float]:
        return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))

    def _cmd_move(self, xs: str, ys: str) -> None:
        try:
            x, y = float(xs), float(ys)
        except (ValueError, TypeError):
            self.get_logger().warning("用法：move X Y（X、Y 为数字）")
            return
        with self._lock:
            if not self._points:
                self.get_logger().warning(_yellow("还没有点，无法移动。可用 pnt X Y DEG 追加。"))
                return
            p = self._points[-1]
            p.x, p.y = x, y
            idx = len(self._points)
            yawstr = f"{p.yaw_deg:.1f}°"
        self.get_logger().info(
            _green(f"已移动点 #{idx} 到 ({x:.2f}, {y:.2f})，朝向 {yawstr}")
        )
        self._print_points()
        self._publish_route_markers()
        self._save_points()

    def _cmd_yaw(self, yaws: str) -> None:
        if yaws.strip().lower() in ("auto", "a", "path", "along"):
            with self._lock:
                pts = list(self._points)
            if len(pts) < 2:
                self.get_logger().warning(
                    _yellow("yaw auto 需要至少 2 个点（用上一→当前的行进方向）。")
                )
                return
            cur = pts[-1]
            prev = pts[-2]
            yaw = math.atan2(cur.y - prev.y, cur.x - prev.x)
            qx, qy, qz, qw = self._q_from_yaw(yaw)
            with self._lock:
                if not self._points:
                    return
                p = self._points[-1]
                p.qx, p.qy, p.qz, p.qw = qx, qy, qz, qw
                idx = len(self._points)
            self.get_logger().info(
                _green(
                    f"点 #{idx} 朝向 = auto（行进方向）: "
                    f"{math.degrees(yaw):.1f}° ({yaw:.3f} rad)"
                )
            )
            self._print_points()
            self._publish_route_markers()
            self._save_points()
            return

        try:
            yaw = self._parse_angle(yaws)
        except (ValueError, TypeError):
            self.get_logger().warning("用法：yaw DEG（度；也可 90d 或 1.57r）")
            return
        qx, qy, qz, qw = self._q_from_yaw(yaw)
        with self._lock:
            if not self._points:
                self.get_logger().warning(_yellow("还没有点，无法设置朝向。可用 pnt X Y DEG 追加。"))
                return
            p = self._points[-1]
            p.qx, p.qy, p.qz, p.qw = qx, qy, qz, qw
            idx = len(self._points)
            deg = math.degrees(yaw)
        self.get_logger().info(
            _green(f"点 #{idx} 朝向 = {deg:.1f}° ({yaw:.3f} rad)")
        )
        self._print_points()
        self._publish_route_markers()
        self._save_points()

    def _cmd_add_point(self, xs: str, ys: str, yaws: str) -> None:
        try:
            x, y = float(xs), float(ys)
            yaw = self._parse_angle(yaws)
        except (ValueError, TypeError):
            self.get_logger().warning("用法：pnt X Y DEG（DEG 为度）")
            return
        qx, qy, qz, qw = self._q_from_yaw(yaw)
        with self._lock:
            self._points.append(RecordedPoint(x, y, 0.0, qx, qy, qz, qw, self._expected_frame))
            idx = len(self._points)
        self.get_logger().info(
            _green(f"追加手动点 #{idx}: ({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.1f}°")
        )
        self._print_points()
        self._publish_route_markers()
        self._save_points()

    def _cmd_edit_point(self, idss: str, xs: str, ys: str, yaws: str) -> None:
        try:
            idx = int(idss)
            x, y = float(xs), float(ys)
            yaw = self._parse_angle(yaws)
        except (ValueError, TypeError):
            self.get_logger().warning("用法：edit IDX X Y DEG")
            return
        qx, qy, qz, qw = self._q_from_yaw(yaw)
        with self._lock:
            if not (1 <= idx <= len(self._points)):
                self.get_logger().warning(
                    _yellow(f"点编号越界：1..{len(self._points)}")
                )
                return
            p = self._points[idx - 1]
            p.x, p.y = x, y
            p.qx, p.qy, p.qz, p.qw = qx, qy, qz, qw
        self.get_logger().info(
            _green(f"已修改点 #{idx}: ({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.1f}°")
        )
        self._print_points()
        self._publish_route_markers()
        self._save_points()

    # ----------------------------------------------------------------------- #
    # 目标记录
    # ----------------------------------------------------------------------- #
    def _cb_goal_pose(self, msg: PoseStamped) -> None:
        frame = msg.header.frame_id
        self.get_logger().info(
            f"  [收到] {self._goal_topic}: x={msg.pose.position.x:.2f} "
            f"y={msg.pose.position.y:.2f} frame={frame} "
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )
        if self._expected_frame and frame and frame != self._expected_frame:
            self.get_logger().warning(
                f"目标点 frame 为 {frame!r}，与配置 {self._expected_frame!r} 不同，仍记录（以该 frame 为准）。"
            )
        self._add_pose_point(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
            frame,
            has_heading=True,  # 2D Goal Pose 拖拽给出真实朝向，不被 auto_yaw 覆盖
        )

    def _cb_simple_goal(self, msg: PoseStamped) -> None:
        """备用通道：经典 2D Nav Goal 发的 /move_base_simple/goal（PoseStamped）。"""
        frame = msg.header.frame_id
        self.get_logger().info(
            f"  [收到] {self._simple_goal_topic}: x={msg.pose.position.x:.2f} "
            f"y={msg.pose.position.y:.2f} frame={frame}"
        )
        if self._expected_frame and frame and frame != self._expected_frame:
            self.get_logger().warning(
                f"目标点 frame 为 {frame!r}，与配置 {self._expected_frame!r} 不同，仍记录（以该 frame 为准）。"
            )
        self._add_pose_point(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
            frame,
            has_heading=True,  # 备用通道同样带朝向
        )

    def _cb_clicked_point(self, msg) -> None:
        """主通道：Publish Point 工具发的 /clicked_point（无朝向，yaw=0）。"""
        frame = msg.header.frame_id
        self.get_logger().info(
            f"  [收到] {self._clicked_topic}: x={msg.point.x:.2f} "
            f"y={msg.point.y:.2f} frame={frame}"
        )
        self._add_pose_point(
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
            0.0,
            0.0,
            0.0,
            1.0,
            frame,
            has_heading=False,  # Publish Point 不带朝向，允许 auto_yaw 推算
        )

    def _add_pose_point(
        self, x, y, z, qx, qy, qz, qw, frame, has_heading: bool = False
    ) -> None:
        """去重 + （可选）自动朝向 + 追加记录 + 打印 + 刷新 marker。返回是否记录成功。

        has_heading=True 表示来源本身携带朝向（/goal_pose、/move_base_simple/goal），
        即使 auto_yaw 开启也尊重原始朝向，不再用行进方向覆盖。
        """
        p = RecordedPoint(x, y, z, qx, qy, qz, qw, frame)
        auto_note = ""
        if self._auto_yaw and not has_heading:
            # 参考点：上一个点；若无，用船当前位置指向本点
            ref = None
            with self._lock:
                if self._points:
                    ref = self._points[-1]
            if ref is None and self._have_odom:
                with self._odom_lock:
                    ref = self._robot
            if ref is not None:
                if isinstance(ref, RecordedPoint):
                    rx, ry = ref.x, ref.y
                else:
                    rx, ry = ref[0], ref[1]
                yaw = math.atan2(p.y - ry, p.x - rx)
                qx, qy, qz, qw = self._q_from_yaw(yaw)
                p = RecordedPoint(x, y, z, qx, qy, qz, qw, frame)
                auto_note = f"  朝向auto={math.degrees(yaw):.1f}°"
            else:
                auto_note = "  朝向auto(无参考点→0°)"
        with self._lock:
            if self._points and self._dedup > 0.0:
                last = self._points[-1]
                if p.distance_to(last) < self._dedup:
                    self.get_logger().info(
                        _yellow(
                            f"忽略重复点（与上一点距离 < {self._dedup:.2f}m）："
                            f"({p.x:.2f}, {p.y:.2f})"
                        )
                    )
                    return False
            self._points.append(p)
            idx = len(self._points)
        self.get_logger().info(
            _green(
                f"记录点位 #{idx}: ({p.x:.2f}, {p.y:.2f}) yaw={p.yaw:.3f} rad "
                f"({p.yaw_deg:.1f}°) frame={frame}{auto_note}"
            )
        )
        self._publish_route_markers()
        self._save_points()
        return True

    def _print_points(self) -> None:
        with self._lock:
            pts = list(self._points)
        if not pts:
            self.get_logger().info(_yellow("当前没有记录的点。请在 RViz 里点击目标。"))
            return
        self.get_logger().info(_cyan(f"已记录 {len(pts)} 个点："))
        for i, p in enumerate(pts):
            self.get_logger().info(
                f"  #{i + 1}: ({p.x:.2f}, {p.y:.2f}) yaw={p.yaw:.3f} rad "
                f"({p.yaw_deg:.1f}°) frame={p.frame_id}"
            )
        self.get_logger().info(
            _cyan("输入 go 开始按此顺序巡逻；可用 yaw DEG / edit IDX X Y DEG 微调朝向。")
        )

    # ----------------------------------------------------------------------- #
    # 持久化
    # ----------------------------------------------------------------------- #
    def _save_points(self, force: bool = False) -> bool:
        """把当前记录的点保存到 JSON 文件（原子写入）。force=True 即使 autosave 关闭也保存。"""
        if not force and not self._autosave:
            return False
        with self._lock:
            pts = list(self._points)
        data = {
            "version": 1,
            "frame_id": self._expected_frame,
            "count": len(pts),
            "points": [p.to_dict() for p in pts],
        }
        path = Path(self._waypoints_file)
        tmp = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix="patrol_", suffix=".tmp")
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            os.replace(tmp, str(path))
            tmp = None
            self.get_logger().info(_green(f"已保存 {len(pts)} 个点到 {path}"))
            return True
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(_yellow(f"保存点位失败：{e}"))
            return False
        finally:
            if tmp is not None:
                try:
                    os.remove(tmp)
                except Exception:
                    pass

    def _load_points(self) -> bool:
        """启动/load 命令：从 JSON 文件载入保存的点并替换当前记录。"""
        path = Path(self._waypoints_file)
        try:
            if not path.exists() or path.stat().st_size == 0:
                return False
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(_yellow(f"读取点位文件失败：{e}"))
            return False
        raw = data.get("points", [])
        loaded: List[RecordedPoint] = []
        for it in raw:
            try:
                loaded.append(
                    RecordedPoint(
                        float(it["x"]),
                        float(it["y"]),
                        float(it.get("z", 0.0)),
                        float(it.get("qx", 0.0)),
                        float(it.get("qy", 0.0)),
                        float(it.get("qz", 0.0)),
                        float(it.get("qw", 1.0)),
                        str(it.get("frame_id", self._expected_frame)),
                    )
                )
            except (KeyError, ValueError, TypeError):
                continue
        if not loaded:
            self.get_logger().warning(_yellow("点文件里没有可用点位。"))
            return False
        with self._lock:
            self._points = loaded
        self.get_logger().info(
            _green(f"已从 {path} 加载 {len(loaded)} 个点。")
        )
        self._publish_route_markers()
        self._print_points()
        return True

    def _import_waypoint_yaml(self, path_str: str) -> bool:
        """导入 ros_map_tool 导出的航点 YAML（waypoint_i: [x, y, yaw(弧度)]），替换当前点位。

        导入的航点自带真实朝向，直接下发；auto_yaw 只影响之后新点击记录的点。
        """
        import re

        path = Path(path_str.strip()).expanduser()
        try:
            text = path.read_text(encoding="utf-8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(_yellow(f"读取航点 YAML 失败：{path}（{e}）"))
            return False
        matches = re.findall(
            r"^\s*waypoint_(\d+)\s*:\s*\[\s*([-+\d.eE]+)\s*,"
            r"\s*([-+\d.eE]+)\s*,\s*([-+\d.eE]+)\s*\]",
            text,
            flags=re.MULTILINE,
        )
        if not matches:
            self.get_logger().warning(
                _yellow(f"{path} 里没有 waypoint_i: [x, y, yaw] 条目。")
            )
            return False
        loaded: List[RecordedPoint] = []
        for _idx, xs, ys, yaws in sorted(matches, key=lambda m: int(m[0])):
            try:
                x, y, yaw = float(xs), float(ys), float(yaws)
            except ValueError:
                continue
            qx, qy, qz, qw = self._q_from_yaw(yaw)
            loaded.append(
                RecordedPoint(x, y, 0.0, qx, qy, qz, qw, self._expected_frame)
            )
        if not loaded:
            self.get_logger().warning(_yellow("航点 YAML 解析结果为空。"))
            return False
        with self._lock:
            self._points = loaded
        self.get_logger().info(
            _green(f"已从 {path} 导入 {len(loaded)} 个带朝向航点（yaw 弧度）。")
        )
        self._publish_route_markers()
        self._print_points()
        self._save_points()
        return True

    def _cmd_save_path(self, new_path: str) -> None:
        self._waypoints_file = new_path.strip()
        self.get_logger().info(
            _green(f"持久化路径已改为：{self._waypoints_file}")
        )
        self._save_points(force=True)

    # ----------------------------------------------------------------------- #
    # 里程计（single 模式用）
    # ----------------------------------------------------------------------- #
    def _cb_odom(self, msg: Odometry) -> None:
        with self._odom_lock:
            self._robot = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            )
            self._have_odom = True

    def _robot_xy(self) -> Tuple[float, float]:
        with self._odom_lock:
            return self._robot

    def _dist_to_target(self) -> Optional[float]:
        with self._lock:
            pts = list(self._points)
        if self._phase != "navigating" or not (0 <= self._cur < len(pts)):
            return None
        with self._odom_lock:
            if not self._have_odom:
                return None
            rx, ry = self._robot
        tx, ty = pts[self._cur].x, pts[self._cur].y
        return math.hypot(tx - rx, ty - ry)

    # ----------------------------------------------------------------------- #
    # 巡逻启动
    # ----------------------------------------------------------------------- #
    def _start_patrol(self) -> None:
        with self._lock:
            pts = list(self._points)
        if not pts:
            self.get_logger().warning(_yellow("还没有记录任何点位，无法开始巡逻。"))
            return
        if self._phase != "idle":
            self.get_logger().warning(
                _yellow("巡逻已在进行中。若需重新下发，先执行 stop 再 go。")
            )
            return

        client = self._fw_client if self._mode == "batch" else self._nav_client
        if not client.wait_for_server(timeout_sec=5.0):
            name = self._follow_action if self._mode == "batch" else self._nav_action
            self.get_logger().error(_red(f"action server 不可用：{name}。"))
            return

        self._pass_count = 0
        self._retries_left = self._max_retries
        self._retry_pending = False
        self._phase = "navigating"
        if self._mode == "single" and not self._have_odom:
            self.get_logger().warning(
                _yellow("尚未收到里程计数据，single 模式将仅依赖 Nav2 完成结果判断到达。")
            )
        self.get_logger().info(
            _green(
                f"开始巡逻：共 {len(pts)} 个点  "
                f"（{'循环' if self._loop else '单次'}，mode={self._mode}）"
            )
        )
        if self._mode == "batch":
            self._dispatch_batch(pts)
        else:
            self._cur = 0
            self._send_current_goal()

    # ----------------------------------------------------------------------- #
    # batch：整串 follow_waypoints + 客户端循环
    # ----------------------------------------------------------------------- #
    def _dispatch_batch(self, pts: List[RecordedPoint]) -> None:
        goal = FollowWaypoints.Goal()
        goal.poses = [p.to_pose_stamped(self.get_clock().now().to_msg()) for p in pts]
        self._last_wp_idx = -1
        self._last_wp_change_wall = time.monotonic()
        self.get_logger().info(
            _green(f"整串下发 {len(pts)} 个点给 follow_waypoints …")
        )
        future = self._fw_client.send_goal_async(
            goal, feedback_callback=self._fw_fb_cb
        )
        future.add_done_callback(self._fw_accepted_cb)

    def _fw_accepted_cb(self, future) -> None:
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"发送 FollowWaypoints 目标失败：{e}")
            self._phase = "idle"
            return
        if not handle.accepted:
            self.get_logger().error(
                _red("整串目标被 follow_waypoints 拒绝（可能已有其它任务在运行）。")
            )
            self._phase = "idle"
            return
        self._active_goal = handle
        self._active_client = self._fw_client
        self.get_logger().info(_cyan("整串目标已被接受，开始巡航…"))
        res = handle.get_result_async()
        res.add_done_callback(self._fw_result_cb)

    def _fw_fb_cb(self, feedback_msg) -> None:
        try:
            idx = int(feedback_msg.feedback.current_waypoint_index)
            self.get_logger().info(f"  Nav2 正在前往点位 #{idx + 1}")
            if idx != self._last_wp_idx:
                self._last_wp_idx = idx
                self._last_wp_change_wall = time.monotonic()
        except Exception:
            pass

    def _fw_result_cb(self, future) -> None:
        try:
            status = future.result().status
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"读取整串结果失败：{e}")
            self._phase = "idle"
            return
        self._active_goal = None
        self._active_client = None
        if self._retry_pending:
            # 因为重发而取消/失败的旧目标，不再叠加重发
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._pass_count += 1
            self._retries_left = self._max_retries
            self.get_logger().info(_green(f"巡逻第 {self._pass_count} 圈完成。"))
            if self._loop and (
                self._loop_count == 0 or self._pass_count < self._loop_count
            ):
                with self._lock:
                    pts = list(self._points)
                if pts:
                    self.get_logger().info(_yellow("循环巡逻，重新下发整串点位…"))
                    self._dispatch_batch(pts)
                    return
            self._finish()
        else:
            self._batch_retry(f"整串巡航非正常终止（状态码 {status}）")

    def _batch_retry(self, reason: str) -> None:
        """卡滞/失败后重新整串下发；超出重发次数则停止。"""
        if self._retries_left <= 0:
            self.get_logger().error(
                _red(f"重发次数已用尽（{self._max_retries} 次），停止巡逻。原因：{reason}")
            )
            self._phase = "idle"
            self._retry_pending = False
            self._finish()
            return
        self._retries_left -= 1
        self._retry_pending = True
        self.get_logger().warning(
            _yellow(
                f"卡滞/失败：{reason}。{self._retry_delay:.1f}s 后重新下发整串"
                f"（剩余重发 {self._retries_left} 次）…"
            )
        )
        self._cancel_current()
        self._schedule_retry()

    def _schedule_retry(self) -> None:
        if self._retry_timer is not None:
            try:
                self._retry_timer.cancel()
            except Exception:
                pass
        self._retry_timer = self.create_timer(self._retry_delay, self._retry_timer_cb)

    def _retry_timer_cb(self) -> None:
        t = self._retry_timer
        self._retry_timer = None
        if t is not None:
            try:
                t.cancel()
            except Exception:
                pass
        self._retry_pending = False
        with self._lock:
            pts = list(self._points)
        if self._active_goal is not None:
            self.get_logger().warning("仍有活动目标，跳过本次重发。")
            self._phase = "idle"
            return
        if not pts:
            self.get_logger().warning("点位为空，停止重发。")
            self._phase = "idle"
            return
        self._phase = "navigating"
        if self._mode == "batch":
            self.get_logger().info(_green("重新下发整串点位…"))
            self._dispatch_batch(pts)
        else:
            self.get_logger().info(_green(f"重新下发当前点 #{self._cur + 1}…"))
            self._send_current_goal()

    # ----------------------------------------------------------------------- #
    # single：逐点 navigate_to_pose + 里程计到达判定
    # ----------------------------------------------------------------------- #
    def _send_current_goal(self) -> None:
        with self._lock:
            pts = list(self._points)
        if not (0 <= self._cur < len(pts)):
            self._finish()
            return
        tgt = pts[self._cur]
        goal = NavigateToPose.Goal()
        goal.pose = tgt.to_pose_stamped(self.get_clock().now().to_msg())
        self.get_logger().info(
            _green(
                f"下发改点 #{self._cur + 1}/{len(pts)}: "
                f"({tgt.x:.2f}, {tgt.y:.2f}) yaw={tgt.yaw:.3f} rad"
            )
        )
        self._point_start_wall = time.monotonic()
        self._near_count = 0
        self._odom_reached = False
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_fb_cb
        )
        future.add_done_callback(self._nav_accepted_cb)

    def _nav_accepted_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"发送目标失败：{e}")
            self._phase = "idle"
            return
        if not goal_handle.accepted:
            self.get_logger().error(
                _red(f"目标被 {self._nav_action} 拒绝（可能已有其它任务在运行）。")
            )
            self._phase = "idle"
            return
        self._active_goal = goal_handle
        self._active_client = self._nav_client
        self.get_logger().info(_cyan("目标已被接受，开始导航…"))
        res = goal_handle.get_result_async()
        res.add_done_callback(self._nav_result_cb)

    def _nav_fb_cb(self, feedback_msg) -> None:
        try:
            pose = feedback_msg.feedback.current_pose.pose.position
            self.get_logger().info(f"  Nav2 当前位姿: ({pose.x:.2f}, {pose.y:.2f})")
        except Exception:
            pass

    def _nav_result_cb(self, future) -> None:
        try:
            status = future.result().status
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"读取导航结果失败：{e}")
            self._phase = "idle"
            return
        self._active_goal = None
        self._active_client = None
        self._on_goal_terminal(status)

    def _on_goal_terminal(self, status: int) -> None:
        if self._phase not in ("navigating", "transition"):
            return
        reached = status == GoalStatus.STATUS_SUCCEEDED or self._odom_reached
        if reached:
            self._log_arrived()
            self._phase = "transition"
            self._advance()
            return
        dist = self._dist_to_target()
        if dist is not None and dist <= self._arrival_tolerance:
            self._log_arrived()
            self._phase = "transition"
            self._advance()
            return
        # 未到达即终止：尝试重发当前点，重发次数用尽才停止
        self._single_retry(status)

    def _single_retry(self, status: int) -> None:
        if self._retry_pending:
            return
        if self._retries_left <= 0:
            self.get_logger().error(
                _red(
                    f"点 #{self._cur + 1} 重发次数已用尽（{self._max_retries} 次），停止巡逻。"
                )
            )
            self._phase = "idle"
            self._finish()
            return
        self._retries_left -= 1
        self._retry_pending = True
        self.get_logger().warning(
            _yellow(
                f"点 #{self._cur + 1} 终止（状态码 {status}）且未到达，"
                f"{self._retry_delay:.1f}s 后重发（剩余 {self._retries_left} 次）…"
            )
        )
        self._cancel_current()
        self._schedule_retry()

    def _watch_tick(self) -> None:
        now = time.monotonic()
        # 空闲时定期提醒：若一直没收到点位，多半是话题没对上或节点后启动
        if self._phase == "idle" and not self._points:
            if now - self._no_point_reminder_wall >= 10.0:
                self._no_point_reminder_wall = now
                self.get_logger().info(
                    _yellow(
                        f"仍在监听 {self._goal_topic}"
                        + (f" / {self._clicked_topic}" if self._use_clicked else "")
                        + "。请在 RViz 用「Publish Point」点击目标。"
                        "若仍无 [收到] 日志，请确认 RViz 选中的是 Publish Point 工具"
                        "且与脚本处于同一 ROS_DOMAIN_ID。"
                    )
                )
            return

        if self._phase != "navigating":
            return
        if self._mode == "batch":
            # 卡滞看门狗：很多圈内某点迟迟不到下一个“正在前往点位”标识
            if self._active_goal is not None and (
                now - self._last_wp_change_wall
            ) > self._stall_timeout:
                self._batch_retry(
                    f"卡滞：{self._stall_timeout:.1f}s 未推进到下一个点位"
                )
            return

        dist = self._dist_to_target()
        if dist is None:
            return
        if now - self._last_progress_log >= 1.0:
            self.get_logger().info(f"  距目标 #{self._cur + 1} 距离: {dist:.2f} m")
            self._last_progress_log = now
        if (
            self._point_timeout > 0.0
            and now - self._point_start_wall > self._point_timeout
        ):
            self.get_logger().warning(
                _yellow(f"点 #{self._cur + 1} 超过 {self._point_timeout:.1f}s 未到达…")
            )
            self._cancel_current()
            self._on_goal_terminal(GoalStatus.STATUS_ABORTED)
            return
        if dist <= self._arrival_tolerance:
            self._near_count += 1
            if self._near_count >= self._arrival_confirm_ticks:
                self._odom_reached = True
                self.get_logger().info(
                    _green(
                        f"  已到达点 #{self._cur + 1}（距离 {dist:.2f} m，"
                        f"连续 {self._near_count} 次判定）。"
                    )
                )
                self._cancel_current()
        else:
            self._near_count = 0

    def _log_arrived(self) -> None:
        dist = self._dist_to_target()
        dstr = f"（距离 {dist:.2f} m）" if dist is not None else ""
        self.get_logger().info(_green(f"到达点 #{self._cur + 1} {dstr}"))

    def _advance(self) -> None:
        with self._lock:
            pts = list(self._points)
        if not pts:
            self._phase = "idle"
            self._finish()
            return
        if self._cur + 1 < len(pts):
            self._cur += 1
            self._phase = "navigating"
            self._send_current_goal()
            return
        self._pass_count += 1
        self._retries_left = self._max_retries
        self.get_logger().info(_green(f"巡逻第 {self._pass_count} 圈完成。"))
        if self._loop and (self._loop_count == 0 or self._pass_count < self._loop_count):
            self._cur = 0
            self._phase = "navigating"
            self.get_logger().info(_yellow("循环巡逻，回到第 1 个点继续…"))
            self._send_current_goal()
            return
        self._phase = "idle"
        self._finish()

    def _finish(self) -> None:
        self._active_goal = None
        self._active_client = None
        self._near_count = 0
        self._odom_reached = False
        self._retry_pending = False
        if self._retry_timer is not None:
            try:
                self._retry_timer.cancel()
            except Exception:
                pass
            self._retry_timer = None
        self.get_logger().info(_green("巡逻结束。"))

    def _cancel_current(self) -> None:
        if self._active_goal is not None:
            self.get_logger().info(_yellow("取消当前导航目标…"))
            try:
                cf = self._active_goal.cancel_goal_async()
                cf.add_done_callback(self._cancel_cb)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"取消目标异常：{e}")
                self._phase = "idle"
        else:
            self.get_logger().info(_yellow("当前没有正在执行的导航目标。"))

    def _cancel_cb(self, future) -> None:
        try:
            future.result()
        except Exception:
            pass

    # ----------------------------------------------------------------------- #
    # 可视化
    # ----------------------------------------------------------------------- #
    def _publish_route_markers(self) -> None:
        if not self._publish_markers:
            return
        with self._lock:
            pts = list(self._points)
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, p in enumerate(pts):
            mk = Marker()
            mk.header.frame_id = p.frame_id
            mk.header.stamp = now
            mk.ns = "patrol_points"
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = p.x
            mk.pose.position.y = p.y
            mk.pose.position.z = p.z
            mk.pose.orientation.x = p.qx
            mk.pose.orientation.y = p.qy
            mk.pose.orientation.z = p.qz
            mk.pose.orientation.w = p.qw
            mk.scale.x = mk.scale.y = mk.scale.z = 1.2
            mk.color.r = 0.0
            mk.color.g = 1.0
            mk.color.b = 0.0
            mk.color.a = 1.0
            arr.markers.append(mk)

        if len(pts) >= 2:
            line = Marker()
            line.header.frame_id = pts[0].frame_id
            line.header.stamp = now
            line.ns = "patrol_path"
            line.id = 9000
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.25
            line.color.r = 0.0
            line.color.g = 1.0
            line.color.b = 1.0
            line.color.a = 1.0
            for p in pts:
                pt = Point()
                pt.x = p.x
                pt.y = p.y
                pt.z = p.z
                line.points.append(pt)
            if self._loop:
                pt = Point()
                pt.x = pts[0].x
                pt.y = pts[0].y
                pt.z = pts[0].z
                line.points.append(pt)
            arr.markers.append(line)

        if self._marker_pub is not None:
            self._marker_pub.publish(arr)

    def destroy_node(self) -> None:
        # 退出前保存一次，避免最后编辑丢失
        try:
            self._save_points(force=True)
        except Exception:
            pass
        self._shutdown = True
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizPatrolRecorder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("被中断退出。")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
