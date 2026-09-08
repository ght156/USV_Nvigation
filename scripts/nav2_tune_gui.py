#!/usr/bin/env python3
"""
Nav2 在线调参 + 取消任务小工具（tkinter + rclpy）。

用法（先 source 好 ROS 环境，再运行）：
    python3 scripts/nav2_tune_gui.py
    python3 scripts/nav2_tune_gui.py --params-file <nav2_params_yaml>

说明：
  - 滑块只通过 set_parameters 服务写到【运行中的节点】，不修改任何参数文件；
    重启 launch 后恢复参数文件里的值。
  - 滑块初始值默认读取 nav2_params_real_mavros.yaml 对应参数；节点在线时，
    启动后会用节点当前值再覆盖一次。
  - 滑块范围围绕参数文件里的值 ±50% 自适应（不再用写死的全局范围），
    数值框可直接键入精确值，键入超出滑块范围时会自动扩展范围。
  - 参数列表已按本版 Nav2（1.1.19/Humble）源码核对，只放支持动态更新的项；
    参数前缀与 launch 一致（controller_server / velocity_smoother / costmap 节点）。
  - 「取消任务」不依赖 mission 栈：直接对 navigate_to_pose / follow_waypoints /
    navigate_through_poses 三个 Nav2 action 发 CancelGoal（全零 goal_id = 取消全部），
    由 Nav2 自己收尾（停速、清理）。
"""

import argparse
from pathlib import Path
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
import yaml
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID


# (中文名, 目标节点, 参数名, 最小值, 最大值, 步长, 小数位, 文件默认值, 类型)
# 注：
#   - 最小/最大值为历史参考，滑块范围现按参数文件值 ±50% 自适应生成。
#   - 仅收录本版 Nav2 源码里「运行动态参数」项（set_parameters 即时生效）：
#     · costmap_2d::InflationLayer         -> inflation_radius, cost_scaling_factor
#     · costmap_2d::VoxelLayer             -> max_obstacle_height, mark_threshold,
#                                              z_resolution, z_voxels
#     · Costmap2DROS 顶层                   -> publish_frequency, resolution, width, height
#   - 观测源参数（livox_cloud.obstacle_max_range / min|max_obstacle_height /
#     raytrace_* / clearing / marking）仅 configure 时生效，运行时改无效，未收录。
#   - publish_voxel_map 源码里明确 return 拒绝，未收录。
# 类型: double / int
PARAMS = [
    ("期望线速 (m/s)",          "/controller_server",        "FollowPath.desired_linear_vel",                       0.0, 2.0, 0.05, 2, 1.0, "double"),
    ("最大角加速 (rad/s²)",     "/controller_server",        "FollowPath.max_angular_accel",                        0.0, 3.0, 0.05, 2, 0.45, "double"),
    ("前瞻时间 (s)",            "/controller_server",        "FollowPath.lookahead_time",                           1.0, 10.0, 0.1, 1, 5.0, "double"),
    ("最小前瞻 (m)",            "/controller_server",        "FollowPath.min_lookahead_dist",                       1.0, 10.0, 0.1, 1, 4.0, "double"),
    ("最大前瞻 (m)",            "/controller_server",        "FollowPath.max_lookahead_dist",                       2.0, 15.0, 0.1, 1, 8.0, "double"),
    ("曲率最低速 (m/s)",        "/controller_server",        "FollowPath.regulated_linear_scaling_min_speed",       0.0, 1.0, 0.05, 2, 0.1, "double"),
    ("曲率减速半径 (m)",        "/controller_server",        "FollowPath.regulated_linear_scaling_min_radius",      0.5, 10.0, 0.1, 1, 3.0, "double"),
    ("代价减速距离 (m)",        "/controller_server",        "FollowPath.cost_scaling_dist",                        1.0, 10.0, 0.1, 1, 5.0, "double"),
    ("代价减速增益",            "/controller_server",        "FollowPath.cost_scaling_gain",                        0.0, 1.0, 0.05, 2, 1.0, "double"),
    ("碰撞提前时间 (s)",        "/controller_server",        "FollowPath.max_allowed_time_to_collision_up_to_carrot", 0.5, 10.0, 0.1, 1, 2.0, "double"),
    ("local 膨胀半径 (m)",      "/local_costmap/local_costmap",  "inflation_layer.inflation_radius",             1.0, 10.0, 0.1, 1, 5.0, "double"),
    ("local 代价缩放因子",      "/local_costmap/local_costmap",  "inflation_layer.cost_scaling_factor",           0.0, 3.0, 0.05, 2, 1.0, "double"),
    ("local 体素高上限 (m)",    "/local_costmap/local_costmap",  "voxel_layer.max_obstacle_height",               0.5, 3.0, 0.05, 2, 1.8, "double"),
    ("local 体素标记阈值",      "/local_costmap/local_costmap",  "voxel_layer.mark_threshold",                    0, 6, 1, 0, 1, "int"),
    ("local 体素 z 分辨率 (m)", "/local_costmap/local_costmap",  "voxel_layer.z_resolution",                      0.05, 0.5, 0.05, 2, 0.1, "double"),
    ("local 体素 z 层数",       "/local_costmap/local_costmap",  "voxel_layer.z_voxels",                          2, 40, 1, 0, 16, "int"),
    ("local 发布频率 (Hz)",     "/local_costmap/local_costmap",  "publish_frequency",                             1.0, 10.0, 0.5, 1, 5.0, "double"),
    ("local 分辨率 (m)",        "/local_costmap/local_costmap",  "resolution",                                    0.05, 0.5, 0.05, 2, 0.1, "double"),
    ("local 窗口宽 (m)",        "/local_costmap/local_costmap",  "width",                                         5, 50, 1, 0, 20, "int"),
    ("local 窗口高 (m)",        "/local_costmap/local_costmap",  "height",                                        5, 50, 1, 0, 20, "int"),
    ("global 膨胀半径 (m)",     "/global_costmap/global_costmap", "inflation_layer.inflation_radius",             1.0, 10.0, 0.1, 1, 4.0, "double"),
    ("global 代价缩放因子",     "/global_costmap/global_costmap", "inflation_layer.cost_scaling_factor",           0.0, 3.0, 0.05, 2, 1.0, "double"),
    ("global 体素高上限 (m)",   "/global_costmap/global_costmap", "voxel_layer.max_obstacle_height",               0.5, 3.0, 0.05, 2, 1.8, "double"),
    ("global 体素标记阈值",     "/global_costmap/global_costmap", "voxel_layer.mark_threshold",                    0, 6, 1, 0, 3, "int"),
    ("global 发布频率 (Hz)",    "/global_costmap/global_costmap", "publish_frequency",                             0.5, 5.0, 0.1, 1, 1.0, "double"),
    ("平滑频率 (Hz)",           "/velocity_smoother",        "smoothing_frequency",                                 5.0, 50.0, 1.0, 0, 20.0, "double"),
]

CANCEL_ACTIONS = ["navigate_to_pose", "follow_waypoints", "navigate_through_poses"]
DEFAULT_PARAMS_FILE = (
    Path(__file__).resolve().parents[1]
    / "src/USV_NAV/workspace_nav/config/nav2_params_real_mavros.yaml"
)


def load_file_defaults(path: Path):
    """从 nav2 参数 YAML 读取各参数当前值；返回 {(node, param): float}。"""
    result = {}
    try:
        with path.open(encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception:  # noqa: BLE001
        return result
    for _, node, param, _, _, _, _, _, _ in PARAMS:
        try:
            val = data
            for seg in node.strip("/").split("/"):
                val = val[seg]
            val = val["ros__parameters"]
            for seg in param.split("."):
                val = val[seg]
            result[(node, param)] = float(val)
        except (KeyError, TypeError, ValueError):
            continue
    return result


class TuneApp:
    def __init__(self, root: tk.Tk, defaults: dict):
        self.root = root
        self._defaults = defaults
        self.node = Node("nav2_tune_gui")
        self._spinner = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._spinner.start()

        self._clients = {}          # node -> SetParameters client
        self._get_clients = {}      # node -> GetParameters client
        self._cancel_clients = {}   # action -> CancelGoal client
        self._after_ids = {}        # param id -> after() id（防抖）

        root.title("Nav2 在线调参")
        root.geometry("640x920")

        ttk.Label(
            root,
            text="滑块只改运行中的节点，不写参数文件；重启 launch 后恢复文件值。",
            foreground="#666",
        ).pack(anchor="w", padx=10, pady=(8, 0))

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
        canvas.bind(
            "<Configure>",
            lambda e: canvas.itemconfigure(_win, width=e.width),
        )
        canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")

        def _on_mousewheel(event):  # Windows
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        def _on_mousewheel_linux(event):  # Linux Button-4 / Button-5
            canvas.yview_scroll(-1 if event.num == 4 else 1, "units")

        canvas.bind("<MouseWheel>", _on_mousewheel)
        canvas.bind("<Button-4>", _on_mousewheel_linux)
        canvas.bind("<Button-5>", _on_mousewheel_linux)

        self._sliders = []
        self._value_entries = []
        for idx, (label, node, param, vmin, vmax, step, digits, default, ptype) in enumerate(PARAMS):
            initial = defaults.get((node, param), default)
            self._add_slider(idx, label, node, param, vmin, vmax, step, digits, initial, ptype)

        cancel_btn = ttk.Button(root, text="取消 Nav2 任务", command=self.cancel_mission)
        cancel_btn.pack(anchor="w", padx=10, pady=8)

        self._log = tk.Text(root, height=6, state="disabled", wrap="word")
        self._log.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        # 预热：提前创建全部服务客户端，让 DDS 有时间完成发现
        self._warm_up_clients()
        root.after(300, self.load_current_values)

    def _warm_up_clients(self):
        for _, _, node_name, param, _, _, _, _ in self._sliders:
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

    @staticmethod
    def _range_around(value, ptype="double"):
        """滑块范围围绕实际工作点 ±50%；整数项保证能覆盖几个离散档位。"""
        if value <= 0:
            hi = 1.0 if ptype == "double" else 3.0
            return 0.0, hi
        lo = max(0.0, value * 0.5)
        hi = value * 1.5
        if ptype == "int":
            span = max(2.0, lo)
            lo = max(0.0, value - span)
            hi = value + span
        return lo, hi

    def _expand_range(self, idx, value):
        """若值超出当前滑块范围，向外扩展并留 10% 余量。"""
        slider = self._sliders[idx]
        lo, hi = slider[5], slider[6]
        if lo <= value <= hi:
            return
        new_lo = min(lo, value)
        new_hi = max(hi, value)
        margin = (new_hi - new_lo) * 0.1
        slider[5] = max(0.0, new_lo - margin)
        slider[6] = new_hi + margin
        slider[0].config(from_=slider[5], to=slider[6])

    def _add_slider(self, idx, label, node, param, vmin, vmax, step, digits, initial, ptype):
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

        self._sliders.append([scale, var, node, param, digits, lo, hi, ptype])
        self._value_entries.append(entry)

    def _commit_entry(self, idx):
        """数值框回车/失焦：键入精确值，超范围时自动扩展滑块范围。"""
        slider = self._sliders[idx]
        entry = self._value_entries[idx]
        var, node_name, param, digits, ptype = slider[1], slider[2], slider[3], slider[4], slider[7]
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
        self._set_remote(node_name, param, value, ptype)

    def _on_change(self, idx):
        # 防抖：拖动停止 200ms 后才实际下发
        if idx in self._after_ids:
            self.root.after_cancel(self._after_ids[idx])
        self._after_ids[idx] = self.root.after(200, lambda: self._apply(idx))

    def _apply(self, idx):
        scale, var, node_name, param, digits, _, _, ptype = self._sliders[idx]
        value = round(var.get(), digits)
        if ptype == "int":
            value = int(value)
        entry = self._value_entries[idx]
        entry.delete(0, "end")
        entry.insert(0, f"{value:.{digits}f}")
        self._set_remote(node_name, param, value, ptype)

    def _set_remote(self, node_name, param, value, ptype="double"):
        client = self._clients.get(node_name)
        if client is None:
            client = self.node.create_client(SetParameters, f"{node_name}/set_parameters")
            self._clients[node_name] = client
        if not client.service_is_ready() and not client.wait_for_service(0.5):
            self.log(f"[{node_name}] set_parameters 服务未就绪，跳过 {param}={value}")
            return
        if ptype == "int":
            pval = int(round(value))
        elif ptype == "double":
            pval = float(value)
        else:
            pval = value
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

    def load_current_values(self):
        """启动时从节点读当前值填充滑块；节点没起来就保留默认值。"""
        by_node = {}
        for _, _, node_name, param, _, _, _, _ in self._sliders:
            by_node.setdefault(node_name, []).append(param)

        for node_name, params in by_node.items():
            client = self._get_clients.get(node_name)
            if client is None:
                client = self.node.create_client(GetParameters, f"{node_name}/get_parameters")
                self._get_clients[node_name] = client
            if not client.service_is_ready():
                continue
            req = GetParameters.Request()
            req.names = params
            future = client.call_async(req)
            future.add_done_callback(
                lambda f, n=node_name, ps=params: self._on_get_done(f, n, ps))

    def _on_get_done(self, future, node_name, params):
        try:
            resp = future.result()
            for idx, (_, _, n, p, _, _, _, _) in enumerate(self._sliders):
                if n != node_name or p not in params:
                    continue
                j = params.index(p)
                if j < len(resp.values):
                    v = resp.values[j]
                    if v.type == ParameterType.PARAMETER_INTEGER:
                        value = v.integer_value
                    elif v.type == ParameterType.PARAMETER_DOUBLE:
                        value = v.double_value
                    else:
                        value = v.double_value
                    self.ui_after(lambda i=idx, val=value: self._set_from_node(i, val))
        except Exception as e:  # noqa: BLE001
            self.ui_after(lambda: self.log(f"[{node_name}] 读取参数失败: {e}"))

    def _set_from_node(self, idx, value):
        """用节点在线值更新滑块；超出范围时扩展滑块范围。"""
        self._expand_range(idx, value)
        slider = self._sliders[idx]
        slider[1].set(value)
        entry = self._value_entries[idx]
        entry.delete(0, "end")
        entry.insert(0, f"{value:.{slider[4]}f}")

    def cancel_mission(self):
        threading.Thread(target=self._cancel_mission_worker, daemon=True).start()

    def _cancel_mission_worker(self):
        for action in CANCEL_ACTIONS:
            client = self._cancel_clients.get(action)
            if client is None:
                client = self.node.create_client(CancelGoal, f"/{action}/_action/cancel_goal")
                self._cancel_clients[action] = client
            if not client.service_is_ready() and not client.wait_for_service(2.0):
                self.ui_after(lambda a=action: self.log(
                    f"[{a}] 取消服务未就绪（确认 /{a}/_action/cancel_goal 存在）"))
                continue
            req = CancelGoal.Request()
            req.goal_info = GoalInfo()
            req.goal_info.goal_id = UUID(uuid=[0] * 16)  # 全零 = 取消该 action 的全部目标
            self.ui_after(lambda a=action: self.log(f"[{a}] 发送取消..."))
            future = client.call_async(req)
            future.add_done_callback(
                lambda f, a=action: self._on_cancel_done(f, a))

    def _on_cancel_done(self, future, action):
        try:
            resp = future.result()
            self.ui_after(lambda: self.log(
                f"[{action}] 取消请求返回，已取消 {len(resp.goals_canceling)} 个目标"))
        except Exception as e:  # noqa: BLE001
            self.ui_after(lambda: self.log(f"[{action}] 取消失败: {e}"))

    def ui_after(self, fn):
        self.root.after(0, fn)

    def log(self, msg):
        self._log.config(state="normal")
        self._log.insert("end", msg + "\n")
        self._log.see("end")
        self._log.config(state="disabled")

    def on_close(self):
        try:
            self.node.destroy_node()
        finally:
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Nav2 在线调参工具")
    parser.add_argument(
        "--params-file",
        default=str(DEFAULT_PARAMS_FILE),
        help="nav2 参数 YAML 路径（读取滑块初始值），默认取 nav2_params_real_mavros.yaml",
    )
    args = parser.parse_args()

    params_file = Path(args.params_file)
    defaults = load_file_defaults(params_file)
    print(f"已从参数文件读取 {len(defaults)} 个默认值: {params_file}")

    rclpy.init()
    root = tk.Tk()
    app = TuneApp(root, defaults)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
