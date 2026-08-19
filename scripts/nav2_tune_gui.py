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
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID


# (中文名, 目标节点, 参数名, 最小值, 最大值, 步长, 小数位, 文件默认值)
PARAMS = [
    ("期望线速 (m/s)",          "/controller_server",        "FollowPath.desired_linear_vel",                       0.0, 2.0, 0.05, 2, 1.0),
    ("最大角加速 (rad/s²)",     "/controller_server",        "FollowPath.max_angular_accel",                        0.0, 3.0, 0.05, 2, 0.45),
    ("前瞻时间 (s)",            "/controller_server",        "FollowPath.lookahead_time",                           1.0, 10.0, 0.1, 1, 5.0),
    ("最小前瞻 (m)",            "/controller_server",        "FollowPath.min_lookahead_dist",                       1.0, 10.0, 0.1, 1, 4.0),
    ("最大前瞻 (m)",            "/controller_server",        "FollowPath.max_lookahead_dist",                       2.0, 15.0, 0.1, 1, 8.0),
    ("曲率最低速 (m/s)",        "/controller_server",        "FollowPath.regulated_linear_scaling_min_speed",       0.0, 1.0, 0.05, 2, 0.1),
    ("曲率减速半径 (m)",        "/controller_server",        "FollowPath.regulated_linear_scaling_min_radius",      0.5, 10.0, 0.1, 1, 3.0),
    ("代价减速距离 (m)",        "/controller_server",        "FollowPath.cost_scaling_dist",                        1.0, 10.0, 0.1, 1, 5.0),
    ("代价减速增益",            "/controller_server",        "FollowPath.cost_scaling_gain",                        0.0, 1.0, 0.05, 2, 1.0),
    ("碰撞提前时间 (s)",        "/controller_server",        "FollowPath.max_allowed_time_to_collision_up_to_carrot", 0.5, 10.0, 0.1, 1, 2.0),
    ("local 膨胀半径 (m)",      "/local_costmap/local_costmap",  "inflation_layer.inflation_radius",             1.0, 10.0, 0.1, 1, 5.0),
    ("global 膨胀半径 (m)",     "/global_costmap/global_costmap", "inflation_layer.inflation_radius",             1.0, 10.0, 0.1, 1, 3.5),
    ("平滑频率 (Hz)",           "/velocity_smoother",        "smoothing_frequency",                                 5.0, 50.0, 1.0, 0, 20.0),
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
    for _, node, param, _, _, _, _, _ in PARAMS:
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
        root.geometry("560x680")

        ttk.Label(
            root,
            text="滑块只改运行中的节点，不写参数文件；重启 launch 后恢复文件值。",
            foreground="#666",
        ).pack(anchor="w", padx=10, pady=(8, 0))

        self._sliders = []
        self._value_labels = []
        for idx, (label, node, param, vmin, vmax, step, digits, default) in enumerate(PARAMS):
            initial = defaults.get((node, param), default)
            self._add_slider(idx, label, node, param, vmin, vmax, step, digits, initial)

        cancel_btn = ttk.Button(root, text="取消 Nav2 任务", command=self.cancel_mission)
        cancel_btn.pack(anchor="w", padx=10, pady=8)

        self._log = tk.Text(root, height=8, state="disabled", wrap="word")
        self._log.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        # 预热：提前创建全部服务客户端，让 DDS 有时间完成发现
        self._warm_up_clients()
        root.after(300, self.load_current_values)

    def _warm_up_clients(self):
        for _, _, node_name, param, _, _, _ in self._sliders:
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

    def _add_slider(self, idx, label, node, param, vmin, vmax, step, digits, initial):
        row = ttk.Frame(self.root)
        row.pack(fill="x", padx=10, pady=2)

        ttk.Label(row, text=label, width=24, anchor="w").pack(side="left")
        initial = max(vmin, min(vmax, initial))
        var = tk.DoubleVar(value=initial)
        scale = ttk.Scale(
            row, from_=vmin, to=vmax, orient="horizontal", variable=var,
            command=lambda _v, i=idx: self._on_change(i),
        )
        scale.pack(side="left", fill="x", expand=True, padx=6)
        val = ttk.Label(row, text=f"{initial:.{digits}f}", width=8, anchor="e")
        val.pack(side="right")

        self._sliders.append((scale, var, node, param, digits, vmin, vmax))
        self._value_labels.append(val)

    def _on_change(self, idx):
        # 防抖：拖动停止 200ms 后才实际下发
        if idx in self._after_ids:
            self.root.after_cancel(self._after_ids[idx])
        self._after_ids[idx] = self.root.after(200, lambda: self._apply(idx))

    def _apply(self, idx):
        scale, var, node_name, param, digits, _, _ = self._sliders[idx]
        value = round(var.get(), digits)
        self._value_labels[idx].config(text=f"{value:.{digits}f}")
        self._set_remote(node_name, param, value)

    def _set_remote(self, node_name, param, value):
        client = self._clients.get(node_name)
        if client is None:
            client = self.node.create_client(SetParameters, f"{node_name}/set_parameters")
            self._clients[node_name] = client
        if not client.service_is_ready() and not client.wait_for_service(0.5):
            self.log(f"[{node_name}] set_parameters 服务未就绪，跳过 {param}={value}")
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param, value=float(value)).to_parameter_msg()]
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
        for _, _, node_name, param, _, _, _ in self._sliders:
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
            for idx, (_, _, n, p, _, _, _) in enumerate(self._sliders):
                if n != node_name or p not in params:
                    continue
                j = params.index(p)
                if j < len(resp.values):
                    value = resp.values[j].double_value
                    _, var, _, _, digits, vmin, vmax = self._sliders[idx]
                    value = max(vmin, min(vmax, value))
                    var.set(value)
                    self.ui_after(lambda v=value, d=digits, i=idx: self._value_labels[i].config(
                        text=f"{v:.{d}f}"))
        except Exception as e:  # noqa: BLE001
            self.ui_after(lambda: self.log(f"[{node_name}] 读取参数失败: {e}"))

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
