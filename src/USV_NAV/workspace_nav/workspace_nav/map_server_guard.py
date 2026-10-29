#!/usr/bin/env python3
"""map_server_guard.py —— 启动兜底：确保 /map_server 最终进入 active。

背景（2026-10-29 实船）：nav2_boat.launch.py 用 LoadComposableNodes 把 map_server 和
lifecycle_manager_localization 装进 nav2_container。容器忙时 load_node 的服务响应会超时

    [WARN] [nav2_container.rclcpp]: failed to send response to
    /nav2_container/_container/load_node (timeout)

同一次调用里的第二个组件会被静默丢弃 → map_server 停在 unconfigured、/map 没有发布者，
global_costmap 的静态层为空、日志每 5 秒刷一次 "Robot is out of bounds of the costmap!"，
全局规划不可用（33 次启动里复现 1 次）。

本节点在 --start-delay 秒后检查目标节点的 lifecycle 状态；如果它仍停在
unconfigured/inactive 且状态稳定（连续两次相同，避免和 lifecycle_manager 抢），
就自己调 change_state 走 CONFIGURE → ACTIVATE，并打 ERROR 日志便于事后统计。

不做别的：不改参数、不改代码，只在"本该 active 却没有"时动手。正常启动时它什么也不做。
"""
from __future__ import annotations

import argparse
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

ACTIVE = 3
INACTIVE = 2
UNCONFIGURED = 1
MAX_ATTEMPTS = 3


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--node-name', default='/map_server')
    ap.add_argument('--start-delay', type=float, default=20.0)
    ap.add_argument('--check-period', type=float, default=3.0)
    ap.add_argument('--timeout', type=float, default=90.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Node('map_server_guard')
    log = node.get_logger()
    target = args.node_name.rstrip('/')
    get_cli = node.create_client(GetState, f'{target}/get_state')
    chg_cli = node.create_client(ChangeState, f'{target}/change_state')

    def get_state():
        fut = get_cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        try:
            return int(fut.result().current_state.id)
        except Exception:  # noqa: BLE001
            return None

    def change(transition_id: int) -> bool:
        req = ChangeState.Request()
        req.transition = Transition(id=transition_id)
        fut = chg_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
        try:
            return bool(fut.result().success)
        except Exception:  # noqa: BLE001
            return False

    time.sleep(max(0.0, args.start_delay))
    deadline = time.monotonic() + max(1.0, args.timeout)
    prev_state = None
    attempts = 0
    rc = 0

    while rclpy.ok() and time.monotonic() < deadline:
        get_cli.wait_for_service(timeout_sec=1.0)
        chg_cli.wait_for_service(timeout_sec=1.0)
        if not (get_cli.service_is_ready() and chg_cli.service_is_ready()):
            time.sleep(args.check_period)
            continue

        state = get_state()
        if state is None:
            time.sleep(args.check_period)
            continue

        if state == ACTIVE:
            log.info(f'{target} 已处于 active（{target} 正常，兜底无需介入）')
            node.destroy_node()
            rclpy.shutdown()
            return

        # CONFIGURING / ACTIVATING / CLEANING 等过渡态：交给 lifecycle_manager，别抢
        if state not in (UNCONFIGURED, INACTIVE):
            prev_state = state
            time.sleep(args.check_period)
            continue

        if state != prev_state:
            prev_state = state
            time.sleep(args.check_period)
            continue

        # 连续两次同一非活跃状态 → 判定为卡住，兜底
        log.error(
            f'{target} 启动后仍停留在 state={state}（应为 active），'
            '执行兜底 configure/activate')
        if state == UNCONFIGURED:
            change(Transition.TRANSITION_CONFIGURE)
        change(Transition.TRANSITION_ACTIVATE)
        attempts += 1
        time.sleep(args.check_period)

        state2 = get_state()
        if state2 == ACTIVE:
            log.info(f'{target} 兜底激活成功')
            node.destroy_node()
            rclpy.shutdown()
            return
        log.error(f'{target} 兜底激活失败（state={state2}，第 {attempts} 次）')
        if attempts >= MAX_ATTEMPTS:
            rc = 1
            break
        prev_state = None

    if rc != 0:
        log.error(f'{target} 未能在超时内进入 active，请检查 map_server 与 lifecycle_manager')
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
