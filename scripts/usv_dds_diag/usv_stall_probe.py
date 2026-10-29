#!/usr/bin/env python3
"""usv_stall_probe.py —— 抓 controller_server 卡死的现场（只读）。

针对 2026-10-29 08:04 故障：controller 进了 computeControl() 一次后停在循环顶部，
不报错、不发 /cmd_vel_nav，只剩 pending 目标被反复 terminate。

每秒记录一行 JSON 到 <out>/stall.jsonl：
  kind=topics   /local_costmap/costmap_updates 与 /cmd_vel_nav 的到达率
                （代价图更新停了 ⇒ mapUpdateLoop 线程死 ⇒ isCurrent() 不会恢复）
  kind=threads  nav2 容器每个线程 [tid, state, wchan, 自愿切换/秒]
                判读：
                  · 某线程持续 ~200 次/秒 + wchan=hrtimer_nanosleep
                    ⇒ 在 rclcpp::Rate(100) 里空转，即 controller 卡在
                       while (!costmap_ros_->isCurrent()) { r.sleep(); }
                  · 某线程 0 次/秒 + wchan=futex_wait_queue_me 且持续不变
                    ⇒ 在锁上死等（代价图互斥量 / executor 死锁）

用法:
  python3 -u usv_stall_probe.py --out ~/usv_diag_runs/stall_<时间> [--interval 1]
  Ctrl-C 结束。不发布任何话题，不改参数。
"""
from __future__ import annotations

import argparse
import json
import os
import time

import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def read(path, max_bytes=1 << 20):
    try:
        with open(path, "rb") as f:
            return f.read(max_bytes).decode("utf-8", "replace")
    except OSError:
        return ""


def find_container_pid():
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        cmd = read("/proc/%s/cmdline" % pid, 8192).replace("\x00", " ")
        if "component_container_isolated" in cmd and "__node:=nav2_container" in cmd:
            return int(pid)
    return None


def thread_rows(pid):
    """[[tid, state, wchan, vsw], ...]；vsw 为累计自愿上下文切换次数。"""
    out = []
    tdir = "/proc/%d/task" % pid
    try:
        tids = os.listdir(tdir)
    except OSError:
        return out
    for tid in tids:
        base = "%s/%s" % (tdir, tid)
        st = read(base + "/status", 8192)
        state, vsw = "", 0
        for line in st.splitlines():
            if line.startswith("State:"):
                state = line.split(":", 1)[1].strip()[:1]
            elif line.startswith("voluntary_ctxt_switches:"):
                try:
                    vsw = int(line.split(":", 1)[1])
                except ValueError:
                    pass
        wchan = read(base + "/wchan", 256).strip() or "-"
        out.append([int(tid), state, wchan[:26], vsw])
    return out


class Probe(Node):
    def __init__(self):
        super().__init__("usv_stall_probe")
        self.n_update = 0
        self.n_cmdvel = 0
        # published_footprint 由 Costmap2DROS::updateMap() 每个更新周期发一次
        # （默认 5Hz），是"代价图更新线程还活着"最轻量的信号。
        self.create_subscription(PolygonStamped, "/local_costmap/published_footprint",
                                 self._on_update,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_cmdvel,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _on_update(self, m):
        self.n_update += 1

    def _on_cmdvel(self, m):
        self.n_cmdvel += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--max-sec", type=float, default=900.0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "stall.jsonl")
    f = open(path, "a", buffering=1)

    rclpy.init()
    node = Probe()
    pid = find_container_pid()
    prev = {}
    t_prev = time.time()
    t_start = t_prev
    node.get_logger().info("container pid=%s out=%s" % (pid, path))
    try:
        while True:
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except rclpy.executors.ExternalShutdownException:
                break
            now = time.time()
            if now - t_prev < args.interval:
                continue
            dt = now - t_prev
            t_prev = now

            f.write(json.dumps({
                "t": round(now, 3), "kind": "topics",
                "local_costmap_update_hz": round(node.n_update / dt, 2),
                "cmd_vel_nav_hz": round(node.n_cmdvel / dt, 2),
            }, ensure_ascii=False) + "\n")
            node.n_update = 0
            node.n_cmdvel = 0

            if pid is None:
                pid = find_container_pid()
            if pid is None:
                continue
            rows = thread_rows(pid)
            out = []
            for tid, st, wchan, vsw in rows:
                pv = prev.get(tid)
                d = 0.0 if pv is None else (vsw - pv) / dt
                prev[tid] = vsw
                out.append([tid, st, wchan, round(d)])
            f.write(json.dumps({"t": round(now, 3), "kind": "threads", "pid": pid,
                                "n": len(out), "threads": out}, ensure_ascii=False) + "\n")
            if now - t_start > args.max_sec:
                break
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
