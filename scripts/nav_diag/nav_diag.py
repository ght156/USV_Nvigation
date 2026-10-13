#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""船端导航 / 代价图在线诊断台（只读，经 ssh；兴飞/澄丰通用）。

为什么单独一个入口：航点下发在 `scripts/map_dispatch/`（页面 + HTTP 桥），
而这套是**命令行诊断**，两者共用同一套"ssh 把脚本喂到船端执行"的机制
（实现复用 `scripts/map_dispatch/waypoint_dispatch_bridge.py` 的 run_remote / watch）。

**所有模式都只订阅、只读，不发任何指令**（不动船）。

用法：
    python3 scripts/nav_diag/nav_diag.py <模式> [参数...] [--host hkzl]

模式（都是只读）：
    check                  船端环境/服务//nav_status 自检
    status                 读一次 /nav_status
    watch 60               流式盯导航（状态/到点距离/朝向差/实际速度/指令速度/前瞻点）
    diag 10                雷达落图自检：TF 链 + 点云俯视形状 + 各过滤级存活数 + 代价图形状
    compare 5              点云 vs 局部代价图**逐格对比**：该标没标/多标/陈旧
    voxelmap 4             解码 /local_costmap/voxel_grid：每个格子被标了几个 z 体素
                           （# = 现在已标成障碍，1 = 只有 1 个 z 层、被 mark_threshold 挡掉）
    count [话题]            统计一张 OccupancyGrid 的格值分布（清代价图前后对比）
    params [节点]           列/读运行中节点的参数（默认 /local_costmap/local_costmap）

依赖：主机能 `ssh <host>`（默认 hkzl）；船端无需装任何东西（脚本每次由 ssh stdin 送入）。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BRIDGE = HERE.parent / "map_dispatch" / "waypoint_dispatch_bridge.py"

if not BRIDGE.is_file():
    sys.exit(f"找不到 {BRIDGE}（本工具复用它里面的 ssh/remote 机制）")

_spec = importlib.util.spec_from_file_location("_wpd_bridge", BRIDGE)
assert _spec and _spec.loader
bridge = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bridge)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="船端导航/代价图在线只读诊断台",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(__doc__ or "").split("用法：")[-1])
    ap.add_argument("mode",
                    choices=["check", "status", "watch", "diag", "compare",
                             "voxelmap", "tfcheck", "count", "params"])
    ap.add_argument("args", nargs="*", help="模式的附加参数（秒数 / 话题 / 节点名）")
    ap.add_argument("--host", default="hkzl", help="船端 ssh 别名（默认 hkzl）")
    ap.add_argument("--remote-setup", default="",
                    help="船端要 source 的工作区 setup.bash（默认 ~/usv-main/install/arm/setup.bash）")
    ap.add_argument("--compact", action="store_true", help="紧凑 JSON（默认缩进打印）")
    a = ap.parse_args()

    if a.mode == "watch":
        secs = float(a.args[0]) if a.args else 60.0
        period = float(a.args[1]) if len(a.args) > 1 else 0.5
        return bridge.watch(a.host, secs, a.remote_setup, period)

    timeouts = {"check": 60.0, "status": 40.0, "diag": 60.0, "compare": 60.0,
                "voxelmap": 60.0, "count": 40.0, "params": 45.0}
    res = bridge.run_remote(a.host, [a.mode] + [str(x) for x in a.args],
                            timeouts.get(a.mode, 60.0), a.remote_setup)
    print(json.dumps(res, ensure_ascii=False,
                     separators=(",", ":") if a.compact else None,
                     indent=None if a.compact else 2))
    rem = res.get("remote") or {}
    if not res.get("ok"):
        print(f"[失败] {res.get('error') or rem.get('error') or '未知'}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
