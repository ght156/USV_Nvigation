#!/usr/bin/env python3
"""
归港在线调参 + 录 bag + 取消工具（tkinter + rclpy），仿 nav2_tune_gui.py。

用法（先 source 好 ROS 环境，再运行）：
    python3 scripts/dock_tune_gui.py
    python3 scripts/dock_tune_gui.py --docking-yaml <docking.yaml> --mission-yaml <dock_mission.yaml>

说明：
  - 滑块只通过 set_parameters 服务写到【运行中的节点】，不修改任何参数文件；
    重启 launch 后恢复参数文件里的值。调好后记下数值，手动写回 YAML。
  - 只收录【每周期实时读取】的参数（set_parameters 即时生效）：
      dock_mission_node / docking_fsm / docking_motion_controller / docking_safety
    的阈值、速度、增益类参数。
    ⚠️ docking_pose_estimator（滤波/跳变门限）
    的参数是启动时缓存的，在线改了不生效，须改 YAML 重启节点，故未收录。
  - staging_x / final_target_x / back_in_gate2_* / max_docking_duration_sec 这类
    FSM 与控制器/安全双份保持一致的参数，滑块会【同时写两个节点】。
  - 滑块范围围绕当前值 ±50% 自适应（支持负值参数）；数值框可键入精确值，
    超范围自动扩展。
  - 「取消归港」四连：/dock/mission/cancel(Trigger) + /dock/cancel(Empty)
    + mission_bridge/cancel_mission + Nav2 三 action CancelGoal。
    注意 dock_mission 取消会话本身【不会】撤掉已下发的 Nav2 预泊目标，
    必须走 mission_bridge/cancel_mission，所以这里一并调。
  - 「录 bag」用 ros2 bag record 录归港关键话题到 ./dock_bags/<时间戳>/，
    停止时发 SIGINT 让 bag 正常收尾（不要 kill -9，否则 metadata 损坏）。
    录哪些话题可在按钮下方按组勾选（含二维码定位 /apriltag_node/detections），
    增删话题/分组改脚本顶部 BAG_TOPIC_GROUPS。
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
import yaml
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID

try:
    from m_common.srv import CancelMission
except ImportError:  # m_common 未 build 时降级：跳过 mission_bridge 取消
    CancelMission = None


# (分组, 中文名, 目标节点(可多个，同时写), 参数名, 小数位, 文件默认值, 类型)
N_MISSION = ("/dock_mission_node",)
N_FSM = ("/docking_fsm",)
N_CTRL = ("/docking_motion_controller",)
N_SAFETY = ("/docking_safety",)
N_FSM_CTRL = ("/docking_fsm", "/docking_motion_controller")      # 双份参数同步写
N_FSM_SAFETY = ("/docking_fsm", "/docking_safety")

PARAMS = [
    # ── 预泊编排（dock_mission）──
    ("预泊编排", "预泊停稳时间 (s)",        N_MISSION, "settle_sec",                  2, 2.5, "double"),
    ("预泊编排", "完成收尾时间 (s)",        N_MISSION, "complete_settle_sec",         2, 2.0, "double"),
    ("预泊编排", "预泊重试上限",            N_MISSION, "staging_retry_max",           0, 3, "int"),
    ("预泊编排", "预泊XY容差 (m)",          N_MISSION, "docking_xy_goal_tolerance",   2, 0.6, "double"),
    ("预泊编排", "预泊艏向容差 (rad)",      N_MISSION, "docking_yaw_goal_tolerance",  3, 0.15, "double"),
    ("预泊编排", "事件ACK超时 (s)",         N_MISSION, "event_ack_timeout",           2, 3.0, "double"),

    # ── FSM 门槛（docking_fsm）──
    ("FSM 门槛", "预备点 staging_x (m)",    N_FSM_CTRL, "staging_x",                  2, -2.5, "double"),
    ("FSM 门槛", "APPROACH x容差 (m)",      N_FSM, "approach_x_tol",                  2, 0.4, "double"),
    ("FSM 门槛", "APPROACH y容差 (m)",      N_FSM, "approach_y_tol",                  3, 0.15, "double"),
    ("FSM 门槛", "APPROACH 艏向门槛 (deg)", N_FSM, "approach_yaw_tol_deg",            1, 10.0, "double"),
    ("FSM 门槛", "APPROACH 停稳速度 (m/s)", N_FSM, "approach_stop_speed",             3, 0.15, "double"),
    ("FSM 门槛", "APPROACH 丢Tag宽限 (s)",  N_FSM, "approach_tag_loss_grace_sec",     2, 3.0, "double"),
    ("FSM 门槛", "APPROACH 超时 (s)",       N_FSM, "approach_timeout_sec",            0, 120.0, "double"),
    ("FSM 门槛", "ALIGN y门槛 (m)",         N_FSM, "align_y_tol",                     3, 0.15, "double"),
    ("FSM 门槛", "ALIGN 艏向门槛 (deg)",    N_FSM, "align_yaw_tol_deg",               1, 5.0, "double"),
    ("FSM 门槛", "ALIGN 保持时间 (s)",      N_FSM, "align_hold_sec",                  2, 1.0, "double"),
    ("FSM 门槛", "ALIGN y中止回退 (m)",     N_FSM, "align_y_abort",                   3, 0.35, "double"),
    ("FSM 门槛", "ALIGN y卡死逃逸 (s)",     N_FSM, "align_y_stuck_sec",               1, 6.0, "double"),
    ("FSM 门槛", "ALIGN 丢Tag宽限 (s)",     N_FSM, "align_tag_loss_grace_sec",        2, 5.0, "double"),
    ("FSM 门槛", "ALIGN 超时 (s)",          N_FSM, "align_timeout_sec",               0, 45.0, "double"),
    ("FSM 门槛", "BACK_IN 走廊y (m)",       N_FSM_CTRL, "back_in_gate2_y",            3, 0.35, "double"),
    ("FSM 门槛", "BACK_IN 走廊艏向 (deg)",  N_FSM_CTRL, "back_in_gate2_yaw_deg",      1, 10.0, "double"),
    ("FSM 门槛", "BACK_IN 超时 (s)",        N_FSM, "back_in_timeout_sec",             0, 90.0, "double"),
    ("FSM 门槛", "FINAL进入y容差 (m)",      N_FSM, "final_dock_entry_y_tol",          3, 0.15, "double"),
    ("FSM 门槛", "停靠 x容差 (m)",          N_FSM, "docked_x_tol",                    3, 0.15, "double"),
    ("FSM 门槛", "停靠 y容差 (m)",          N_FSM, "docked_y_tol",                    3, 0.10, "double"),
    ("FSM 门槛", "停靠 艏向容差 (deg)",     N_FSM, "docked_yaw_tol_deg",              1, 3.0, "double"),
    ("FSM 门槛", "停靠 保持时间 (s)",       N_FSM, "docked_hold_sec",                 2, 1.0, "double"),
    ("FSM 门槛", "FINAL 超时 (s)",          N_FSM, "final_dock_timeout_sec",          0, 60.0, "double"),
    ("FSM 门槛", "集帧数 acquire",          N_FSM, "tag_acquire_frames",              0, 5, "int"),
    ("FSM 门槛", "ACQUIRE 超时 (s)",        N_FSM, "acquire_timeout_sec",             0, 60.0, "double"),
    ("FSM 门槛", "UNDOCK 超时 (s)",         N_FSM, "undock_timeout_sec",              0, 60.0, "double"),
    ("FSM 门槛", "全局总超时 (s)",          N_FSM_SAFETY, "max_docking_duration_sec", 0, 150.0, "double"),

    # ── 控制器（docking_motion_controller）──
    ("控制器速度", "APPROACH 倒速上限 (m/s)", N_CTRL, "approach_speed",              2, 0.45, "double"),
    ("控制器速度", "APPROACH 原地转门槛 (deg)", N_CTRL, "approach_turn_deg",         1, 25.0, "double"),
    ("控制器速度", "APPROACH 蟹行角限 (deg)", N_CTRL, "approach_crab_deg",           1, 8.0, "double"),
    ("控制器速度", "锥形降速 |y| (m)",       N_CTRL, "approach_slow_y",               2, 1.0, "double"),
    ("控制器速度", "锥形降速距离 (m)",       N_CTRL, "approach_slow_dist",            2, 2.5, "double"),
    ("控制器速度", "蠕行下限 (m/s)",         N_CTRL, "approach_min_speed",            3, 0.08, "double"),
    ("控制器速度", "ALIGN 弧线倒速 (m/s)",   N_CTRL, "align_arc_speed",               3, 0.08, "double"),
    ("控制器速度", "BACK_IN 常速 (m/s)",     N_CTRL, "back_in_speed_normal",          2, 0.40, "double"),
    ("控制器速度", "BACK_IN 慢速 (m/s)",     N_CTRL, "back_in_speed_slow",            2, 0.20, "double"),
    ("控制器速度", "BACK_IN 门控1 y (m)",    N_CTRL, "back_in_gate1_y",               3, 0.20, "double"),
    ("控制器速度", "BACK_IN 门控1艏向 (deg)", N_CTRL, "back_in_gate1_yaw_deg",        1, 5.0, "double"),
    ("控制器速度", "BACK_IN 角速度上限 (rad/s)", N_CTRL, "back_in_max_yaw_rate",      2, 0.18, "double"),
    ("控制器速度", "FINAL 倒入速度 (m/s)",   N_CTRL, "final_dock_speed",              2, 0.15, "double"),
    ("控制器速度", "ABORT 驶出速度 (m/s)",   N_CTRL, "abort_exit_speed",              2, 0.30, "double"),
    ("控制器速度", "UNDOCK 速度 (m/s)",      N_CTRL, "undock_speed",                  2, 0.45, "double"),
    ("控制器速度", "最大倒速 (m/s)",         N_CTRL, "max_reverse_speed",             2, 0.8, "double"),
    ("控制器速度", "最大角速度 (rad/s)",     N_CTRL, "max_yaw_rate",                  2, 0.35, "double"),
    ("控制器速度", "搜索角速度 (rad/s)",     N_CTRL, "search_angular_speed",          2, 0.35, "double"),
    ("控制器速度", "保持态发零时长 (s)",     N_CTRL, "hold_zero_publish_sec",         2, 1.0, "double"),

    # ── 控制器增益 ──
    ("控制器增益", "前进横向增益 ky",        N_CTRL, "ky_approach",                   2, 0.40, "double"),
    ("控制器增益", "前进艏向增益 kyaw",      N_CTRL, "kyaw_approach",                 2, 0.85, "double"),
    ("控制器增益", "ALIGN 艏向增益",         N_CTRL, "kyaw_align",                    2, 0.90, "double"),
    ("控制器增益", "倒船横向增益 ky_back",   N_CTRL, "ky_back",                       2, 0.80, "double"),
    ("控制器增益", "倒船艏向增益 kyaw_back", N_CTRL, "kyaw_back",                     2, 0.60, "double"),
    ("控制器增益", "纵向距离增益 kx_final",  N_CTRL, "kx_final",                      2, 0.25, "double"),

    # ── 安全监督（docking_safety）──
    ("安全监督", "横向安全限 (m)",           N_SAFETY, "max_lateral_error",           2, 0.50, "double"),
    ("安全监督", "艏向安全限 (deg)",         N_SAFETY, "max_yaw_error_deg",           1, 15.0, "double"),
    ("安全监督", "推算允许时长 (s)",         N_SAFETY, "odom_prediction_timeout",     2, 3.0, "double"),
    ("安全监督", "ALIGN 推算窗口 (s)",       N_SAFETY, "align_odom_prediction_timeout", 2, 7.0, "double"),
    ("安全监督", "超限防抖周期数",           N_SAFETY, "violation_cycles",            0, 10, "int"),
]

CANCEL_ACTIONS = ["navigate_to_pose", "follow_waypoints", "navigate_through_poses"]

# 录 bag 话题分组：GUI 里按组勾选，要增删话题/分组直接改这里。
# 注意 /apriltag_node/detections 是自定义消息 m_common/msg/DetectionObjList，
# 录它必须在 source 了工作空间（install/setup.bash）的环境里运行本脚本。
BAG_TOPIC_GROUPS = {
    "归港核心": [
        "/docking/dock_pose", "/docking/tag_visible", "/docking/pose_source",
        "/docking/measurement_age", "/docking/state", "/docking/target_mode",
        "/docking/safety_stop", "/docking/abort_request", "/docking/cmd_vel_test",
        "/cmd_vel_nav",
    ],
    "任务/指令": [
        "/dock/status", "/dock/start", "/dock/cancel", "/dock/undock",
        "/dock/entry_status", "/dock_task/status", "/dock_task/event_log",
        "/gcs_dock/command",
    ],
    "二维码定位": [
        "/apriltag_node/detections",
    ],
    "定位/TF": [
        "/mavros/gps_input/local", "/odometry/filtered",
        "/tf", "/tf_static",
    ],
}
# 默认勾选的分组（不想默认录的组从这里去掉即可，GUI 里仍可手动勾上）
BAG_DEFAULT_GROUPS = list(BAG_TOPIC_GROUPS)


def selected_bag_topics(group_vars):
    """按勾选项展开成话题列表，保持分组声明顺序、去重。"""
    topics, seen = [], set()
    for group, var in group_vars.items():
        if not var.get():
            continue
        for t in BAG_TOPIC_GROUPS[group]:
            if t not in seen:
                seen.add(t)
                topics.append(t)
    return topics

_WS = Path(__file__).resolve().parents[1]


def _first_existing(*candidates: Path) -> str:
    """两个仓库布局不同（USV_NAV 仓 src/<pkg>，usv 仓 src/USV_NAV/<pkg>），按序取首个存在的。"""
    for c in candidates:
        if c.is_file():
            return str(c)
    return str(candidates[0])


DEFAULT_DOCKING_YAML = _first_existing(
    _WS / "src/usv_docking/config/docking.yaml",
    _WS / "src/USV_NAV/usv_docking/config/docking.yaml",
)
DEFAULT_MISSION_YAML = _first_existing(
    _WS / "src/dock_mission/config/dock_mission.yaml",
    _WS / "src/USV_NAV/dock_mission/config/dock_mission.yaml",
)


def load_file_defaults(paths):
    """从归港参数 YAML 读取各参数当前值；返回 {(node, param): float}。"""
    result = {}
    docs = []
    for path in paths:
        try:
            with Path(path).open(encoding="utf-8") as f:
                docs.append(yaml.safe_load(f) or {})
        except Exception:  # noqa: BLE001
            continue
    for _, _, nodes, param, _, _, _ in PARAMS:
        for node in nodes:
            for data in docs:
                try:
                    val = data
                    for seg in node.strip("/").split("/"):
                        val = val[seg]
                    val = val["ros__parameters"]
                    for seg in param.split("."):
                        val = val[seg]
                    result[(node, param)] = float(val)
                    break
                except (KeyError, TypeError, ValueError):
                    continue
    return result


class DockTuneApp:
    def __init__(self, root: tk.Tk, defaults: dict, bag_dir: str):
        self.root = root
        self._defaults = defaults
        self.node = Node("dock_tune_gui")
        self._spinner = threading.Thread(target=self._spin_guard, daemon=True)
        self._spinner.start()

        self._clients = {}          # node -> SetParameters client
        self._get_clients = {}      # node -> GetParameters client
        self._cancel_clients = {}   # action -> CancelGoal client
        self._after_ids = {}        # slider idx -> after() id（防抖）
        self._bag_proc = None
        self._bag_dir = bag_dir

        root.title("归港在线调参")
        root.geometry("680x980")

        ttk.Label(
            root,
            text="滑块只改运行中的节点，不写参数文件；重启 launch 后恢复文件值。"
                 "estimator 参数为启动缓存，请改 YAML 重启。",
            foreground="#a05000",
        ).pack(anchor="w", padx=10, pady=(8, 0))

        # —— 状态行 ——
        self._status_var = tk.StringVar(value="状态：等待 /docking/state …")
        ttk.Label(root, textvariable=self._status_var, foreground="#0060a0").pack(
            anchor="w", padx=10, pady=(2, 0))

        # —— 滑块区（可滚动） ——
        body = ttk.Frame(root)
        body.pack(fill="both", expand=True, padx=0, pady=0)
        canvas = tk.Canvas(body, highlightthickness=0)
        vsb = ttk.Scrollbar(body, orient="vertical", command=canvas.yview)
        self._slider_frame = ttk.Frame(canvas)
        self._slider_frame.bind(
            "<Configure>",
            lambda _e: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        _win = canvas.create_window((0, 0), window=self._slider_frame, anchor="nw")
        canvas.configure(yscrollcommand=vsb.set)
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(_win, width=e.width))
        canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        canvas.bind("<MouseWheel>", lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))
        canvas.bind("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

        self._sliders = []
        self._value_entries = []
        last_section = None
        for section, label, nodes, param, digits, default, ptype in PARAMS:
            if section != last_section:
                ttk.Label(
                    self._slider_frame, text=f"── {section} ──", foreground="#444"
                ).pack(anchor="w", padx=10, pady=(8, 0))
                last_section = section
            idx = len(self._sliders)
            initial = self._defaults.get((nodes[0], param), default)
            self._add_slider(idx, label, nodes, param, digits, initial, ptype)

        # —— 按钮区 ——
        btns = ttk.Frame(root)
        btns.pack(anchor="w", padx=10, pady=6)
        ttk.Button(btns, text="触发 dock_only", command=self.trigger_dock_only).pack(side="left", padx=4)
        ttk.Button(btns, text="一键归港", command=self.trigger_one_click).pack(side="left", padx=4)
        ttk.Button(btns, text="取消归港", command=self.cancel_all).pack(side="left", padx=4)
        self._bag_btn = ttk.Button(btns, text="● 开始录 bag", command=self.toggle_bag)
        self._bag_btn.pack(side="left", padx=4)

        # —— 录 bag 话题分组勾选 ——
        topic_bar = ttk.Frame(root)
        topic_bar.pack(anchor="w", padx=10)
        ttk.Label(topic_bar, text="录 bag 话题：", foreground="#444").pack(side="left")
        self._bag_group_vars = {}
        for group in BAG_TOPIC_GROUPS:
            var = tk.BooleanVar(value=group in BAG_DEFAULT_GROUPS)
            self._bag_group_vars[group] = var
            ttk.Checkbutton(topic_bar, text=group, variable=var).pack(side="left", padx=2)

        self._log = tk.Text(root, height=7, state="disabled", wrap="word")
        self._log.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        # 状态订阅（只读显示）
        self.node.create_subscription(String, "/docking/state", self._on_state, 10)
        self.node.create_subscription(String, "/dock/status", self._on_dock_status, 10)
        self.node.create_subscription(String, "/dock/entry_status", self._on_entry_status, 10)
        # 指令发布
        self._gcs_pub = self.node.create_publisher(String, "/gcs_dock/command", 10)
        self._dock_cancel_pub = self.node.create_publisher(Empty, "/dock/cancel", 10)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._warm_up_clients()
        root.after(300, self.load_current_values)

    # ── ROS 客户端预热 ──
    def _warm_up_clients(self):
        for _, _, nodes, _, _, _, _ in PARAMS:
            for node_name in nodes:
                self._get_clients.setdefault(
                    node_name, self.node.create_client(
                        GetParameters, f"{node_name}/get_parameters"))
                self._clients.setdefault(
                    node_name, self.node.create_client(
                        SetParameters, f"{node_name}/set_parameters"))
        for action in CANCEL_ACTIONS:
            self._cancel_clients.setdefault(
                action, self.node.create_client(
                    CancelGoal, f"/{action}/_action/cancel_goal"))

    # ── 滑块 ──
    @staticmethod
    def _range_around(value, ptype="double"):
        """滑块范围围绕实际工作点 ±50%（支持负值）；0 值给固定小量程。"""
        if value == 0:
            return (0.0, 1.0) if ptype == "double" else (0.0, 3.0)
        lo, hi = value * 0.5, value * 1.5
        if lo > hi:  # 负值
            lo, hi = hi, lo
        if ptype == "int":
            span = max(2.0, abs(value))
            lo, hi = value - span, value + span
        return lo, hi

    def _expand_range(self, idx, value):
        slider = self._sliders[idx]
        lo, hi = slider[5], slider[6]
        if lo <= value <= hi:
            return
        new_lo, new_hi = min(lo, value), max(hi, value)
        margin = (new_hi - new_lo) * 0.1
        slider[5] = new_lo - margin
        slider[6] = new_hi + margin
        slider[0].config(from_=slider[5], to=slider[6])

    def _add_slider(self, idx, label, nodes, param, digits, initial, ptype):
        row = ttk.Frame(self._slider_frame)
        row.pack(fill="x", padx=10, pady=2)
        ttk.Label(row, text=label, width=24, anchor="w").pack(side="left")
        lo, hi = self._range_around(initial, ptype)
        var = tk.DoubleVar(value=initial)
        scale = ttk.Scale(
            row, from_=lo, to=hi, orient="horizontal", variable=var,
            command=lambda _v, i=idx: self._on_change(i),
        )
        scale.pack(side="left", fill="x", expand=True, padx=6)
        entry = ttk.Entry(row, width=10, justify="right")
        entry.insert(0, f"{initial:.{digits}f}")
        entry.pack(side="right")
        entry.bind("<Return>", lambda _e, i=idx: self._commit_entry(i))
        entry.bind("<FocusOut>", lambda _e, i=idx: self._commit_entry(i))
        self._sliders.append([scale, var, nodes, param, digits, lo, hi, ptype])
        self._value_entries.append(entry)

    def _commit_entry(self, idx):
        slider = self._sliders[idx]
        entry = self._value_entries[idx]
        var, nodes, param, digits, ptype = slider[1], slider[2], slider[3], slider[4], slider[7]
        try:
            value = float(entry.get())
        except ValueError:
            entry.delete(0, "end")
            entry.insert(0, f"{var.get():.{digits}f}")
            return
        if ptype == "int":
            value = int(round(value))
        self._expand_range(idx, value)
        var.set(value)
        entry.delete(0, "end")
        entry.insert(0, f"{value:.{digits}f}")
        self._set_remote(nodes, param, value, ptype)

    def _on_change(self, idx):
        if idx in self._after_ids:
            self.root.after_cancel(self._after_ids[idx])
        self._after_ids[idx] = self.root.after(200, lambda: self._apply(idx))

    def _apply(self, idx):
        scale, var, nodes, param, digits, _, _, ptype = self._sliders[idx]
        value = round(var.get(), digits)
        if ptype == "int":
            value = int(value)
        entry = self._value_entries[idx]
        entry.delete(0, "end")
        entry.insert(0, f"{value:.{digits}f}")
        self._set_remote(nodes, param, value, ptype)

    def _set_remote(self, nodes, param, value, ptype="double"):
        pval = int(round(value)) if ptype == "int" else float(value)
        for node_name in nodes:
            client = self._clients.get(node_name)
            if client is None:
                client = self.node.create_client(SetParameters, f"{node_name}/set_parameters")
                self._clients[node_name] = client
            if not client.service_is_ready() and not client.wait_for_service(0.5):
                self.log(f"[{node_name}] set_parameters 服务未就绪，跳过 {param}={value}")
                continue
            req = SetParameters.Request()
            req.parameters = [Parameter(name=param, value=pval).to_parameter_msg()]
            future = client.call_async(req)
            future.add_done_callback(
                lambda f, n=node_name, p=param, v=value: self._on_set_done(f, n, p, v))

    def _on_set_done(self, future, node_name, param, value):
        try:
            resp = future.result()
            ok = resp.results[0].successful if resp.results else False
            msg = f"[{node_name}] {param} = {value} -> {'OK' if ok else '拒绝'}"
        except Exception as e:  # noqa: BLE001
            msg = f"[{node_name}] {param} 设置失败: {e}"
        self.ui_after(lambda: self.log(msg))

    # ── 启动时读取节点在线值 ──
    def load_current_values(self):
        by_node = {}
        for _, _, nodes, param, _, _, _ in PARAMS:
            for n in nodes:
                by_node.setdefault(n, set()).add(param)
        for node_name, params in by_node.items():
            client = self._get_clients.get(node_name)
            if client is None or not client.service_is_ready():
                continue
            req = GetParameters.Request()
            req.names = sorted(params)
            future = client.call_async(req)
            future.add_done_callback(
                lambda f, n=node_name, ps=sorted(params): self._on_get_done(f, n, ps))

    def _on_get_done(self, future, node_name, params):
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self.ui_after(lambda: self.log(f"[{node_name}] 读取参数失败: {e}"))
            return
        for idx, (_, _, nodes, param, _, _, _) in enumerate(PARAMS):
            if node_name not in nodes or param not in params:
                continue
            j = params.index(param)
            if j >= len(resp.values):
                continue
            v = resp.values[j]
            if v.type == ParameterType.PARAMETER_INTEGER:
                value = float(v.integer_value)
            elif v.type == ParameterType.PARAMETER_DOUBLE:
                value = v.double_value
            else:
                continue
            self.ui_after(lambda i=idx, val=value: self._set_from_node(i, val))

    def _set_from_node(self, idx, value):
        self._expand_range(idx, value)
        slider = self._sliders[idx]
        slider[1].set(value)
        entry = self._value_entries[idx]
        entry.delete(0, "end")
        entry.insert(0, f"{value:.{slider[4]}f}")

    # ── 状态显示 ──
    def _on_state(self, msg: String):
        self.ui_after(lambda: self._status_var.set(f"FSM 状态：{msg.data}"))

    def _on_dock_status(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        note = ""
        if d.get("abort_reason"):
            note = f" | abort: {d['abort_reason']}"
        if d.get("needs_reapproach"):
            note += " | needs_reapproach"
        self.ui_after(lambda: self.log(f"[/dock/status]{note or ' ok'}"))

    def _on_entry_status(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not d:
            return
        text = (
            f"入口验收：{d.get('action', '?')}/{d.get('reason', '?')} "
            f"ex={d.get('ex', 0):.2f} ey={d.get('ey', 0):.2f} eyaw={d.get('eyaw', 0):.2f}"
        )
        self.ui_after(lambda: self._status_var.set(text))

    # ── 触发 / 取消 ──
    def _send_gcs_command(self, action: str):
        msg = String()
        msg.data = json.dumps({"action": action})
        self._gcs_pub.publish(msg)
        self.log(f"[/gcs_dock/command] 已发 {action}")

    def trigger_dock_only(self):
        if messagebox.askyesno("确认", "触发 dock_only？（test_only=false 时船会动）"):
            self._send_gcs_command("dock_only")

    def trigger_one_click(self):
        if messagebox.askyesno("确认", "触发一键归港？（test_only=false 时船会动）"):
            self._send_gcs_command("one_click_dock")

    def cancel_all(self):
        threading.Thread(target=self._cancel_worker, daemon=True).start()

    def _cancel_worker(self):
        # 1) dock_mission 会话取消（编排层，幂等）
        cli = self.node.create_client(Trigger, "/dock/mission/cancel")
        if cli.service_is_ready() or cli.wait_for_service(1.0):
            cli.call_async(Trigger.Request())
            self.ui_after(lambda: self.log("[/dock/mission/cancel] 已请求"))
        else:
            self.ui_after(lambda: self.log("[/dock/mission/cancel] 服务不在线，跳过"))
        # 2) usv_docking 直接取消（编排不在时也能停精靠泊）
        self._dock_cancel_pub.publish(Empty())
        self.ui_after(lambda: self.log("[/dock/cancel] Empty 已发"))
        # 3) mission_bridge 取消（dock_mission 取消不会撤 Nav2 预泊目标，必须单独调）
        if CancelMission is not None:
            cli = self.node.create_client(CancelMission, "/mission_bridge/cancel_mission")
            if cli.service_is_ready() or cli.wait_for_service(1.0):
                cli.call_async(CancelMission.Request())
                self.ui_after(lambda: self.log("[mission_bridge/cancel_mission] 已请求"))
            else:
                self.ui_after(lambda: self.log("[mission_bridge/cancel_mission] 服务不在线，跳过"))
        # 4) Nav2 action 兜底取消
        for action in CANCEL_ACTIONS:
            client = self._cancel_clients.get(action)
            if client is None or not (
                client.service_is_ready() or client.wait_for_service(1.0)
            ):
                continue
            req = CancelGoal.Request()
            req.goal_info = GoalInfo()
            req.goal_info.goal_id = UUID(uuid=[0] * 16)  # 全零 = 取消全部目标
            client.call_async(req)
            self.ui_after(lambda a=action: self.log(f"[{a}] CancelGoal 已发"))

    # ── 录 bag ──
    def toggle_bag(self):
        if self._bag_proc is None:
            self._start_bag()
        else:
            self._stop_bag()

    def _start_bag(self):
        topics = selected_bag_topics(self._bag_group_vars)
        if not topics:
            self.log("未勾选任何录 bag 话题分组，请先勾选再开始")
            return
        out = Path(self._bag_dir) / time.strftime("dock_%Y%m%d_%H%M%S")
        out.mkdir(parents=True, exist_ok=True)
        cmd = ["ros2", "bag", "record", "-o", str(out)] + topics
        try:
            self._bag_proc = subprocess.Popen(
                cmd, preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.log("找不到 ros2 命令——请先 source ROS 环境再启动本工具")
            return
        self._bag_btn.config(text="■ 停止录 bag")
        groups = [g for g, v in self._bag_group_vars.items() if v.get()]
        self.log(f"开始录 bag → {out}（{len(topics)} 个话题，分组：{'/'.join(groups)}）")

    def _stop_bag(self):
        proc, self._bag_proc = self._bag_proc, None
        # SIGINT 让 rosbag2 正常收尾写 metadata；kill -9 会损坏 bag
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=10)
            self.log("bag 已停止并保存")
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            self.log("bag 停止超时，已强杀（metadata 可能不完整）")
        self._bag_btn.config(text="● 开始录 bag")

    # ── 基础 ──
    def _spin_guard(self):
        """rclpy.spin 包装：吞掉退出时的 ExternalShutdownException 竞态打印。"""
        try:
            rclpy.spin(self.node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    def ui_after(self, fn):
        self.root.after(0, fn)

    def log(self, msg):
        self._log.config(state="normal")
        self._log.insert("end", msg + "\n")
        self._log.see("end")
        self._log.config(state="disabled")

    def on_close(self):
        if self._bag_proc is not None:
            self._stop_bag()
        try:
            self.node.destroy_node()
        finally:
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="归港在线调参工具")
    parser.add_argument("--docking-yaml", default=str(DEFAULT_DOCKING_YAML),
                        help="usv_docking 参数 YAML（读滑块初始值）")
    parser.add_argument("--mission-yaml", default=str(DEFAULT_MISSION_YAML),
                        help="dock_mission 参数 YAML（读滑块初始值）")
    parser.add_argument("--bag-dir", default="dock_bags",
                        help="bag 输出目录（默认 ./dock_bags）")
    args = parser.parse_args()

    defaults = load_file_defaults([args.docking_yaml, args.mission_yaml])
    print(f"已从参数文件读取 {len(defaults)} 个默认值")

    rclpy.init()
    root = tk.Tk()
    DockTuneApp(root, defaults, args.bag_dir)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
