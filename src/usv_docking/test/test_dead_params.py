#!/usr/bin/env python3
"""死参数防累积：declare_parameter 声明后从未被读取的参数会越积越多
（如 2026-10-07 的 docked_yaw_tol_deg/docked_hold_sec/reacquire_search_yaw_deg
遗留），后来者无从分辨哪些还能调。

判定规则（轻量文本扫描，非 AST）：参数名以 "name" 带引号形式在源码中只出现
一次（即 declare_parameter 处）即视为死参数。p("name")/get_parameter("name")
等读取都含同名带引号出现，天然被算"已读"，不会误报。

白名单内的已知遗留参数（为兼容 dock_tune_gui 在线调参/旧配置而保留声明）
不算失败；白名单外出现新死参数即失败——要么用起来，要么删声明，要么
注明原因后加进白名单。
"""

import re
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
SOURCES = [
    PKG / "usv_docking" / "docking_fsm.py",
    PKG / "usv_docking" / "docking_motion_controller.py",
    PKG / "usv_docking" / "docking_pose_estimator.py",
    PKG / "usv_docking" / "docking_safety.py",
]

# 已知遗留（保留声明的原因见各 declare 处注释）
KNOWN_DEAD = {
    "docked_yaw_tol_deg",
    "docked_hold_sec",
    "reacquire_search_yaw_deg",
}

_DECLARE_RE = re.compile(r'declare_parameter\(\s*"([^"]+)"')


def test_no_new_dead_parameters():
    offenders = []
    for path in SOURCES:
        text = path.read_text()
        for name in _DECLARE_RE.findall(text):
            # 带引号出现次数：1 = 只有声明处，从未被读取
            if text.count(f'"{name}"') < 2 and name not in KNOWN_DEAD:
                offenders.append(f"{path.name}: {name}")
    assert not offenders, (
        "以下参数声明后从未被读取（死参数）。要么用起来，要么删声明，"
        f"要么注明保留原因后加进 KNOWN_DEAD 白名单：\n" + "\n".join(offenders)
    )


def test_known_dead_whitelist_is_accurate():
    """白名单防腐：条目必须仍是"已声明且未读取"的死参数（参数重新被用起来
    或声明被删后，白名单条目也要同步移除，防白名单无限膨胀）。"""
    declared_dead = set()
    for path in SOURCES:
        text = path.read_text()
        for name in _DECLARE_RE.findall(text):
            if text.count(f'"{name}"') < 2:
                declared_dead.add(name)
    stale = KNOWN_DEAD - declared_dead
    assert not stale, (
        f"KNOWN_DEAD 白名单条目已不再是死参数（重新被读取或声明已删），"
        f"请同步清理白名单：{sorted(stale)}"
    )
