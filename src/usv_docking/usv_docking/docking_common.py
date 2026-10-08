#!/usr/bin/env python3
"""docking_common — 四节点共享的常量与平面数学工具。

收编原先在 docking_fsm / docking_motion_controller / docking_safety /
docking_pose_estimator 中各自重复定义的：
  - wrap_angle / yaw_from_quat（平面角度工具）
  - DockState 状态名常量（docking_safety 原先用裸字符串，改为此处单一来源）
  - MODE_* target_mode 字符串（FSM 产出、控制器消费，原先两份拷贝）
  - SRC_* pose_source 字符串（estimator 产出、FSM/控制器/safety 消费）

字符串取值与原各文件完全一致，话题契约不变。
"""

import math


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(z: float, w: float) -> float:
    """平面四元数（仅 yaw 分量）-> yaw。"""
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


class DockState:
    IDLE = "IDLE"
    ACQUIRE_TAG = "ACQUIRE_TAG"
    APPROACH_ENTRY = "APPROACH_ENTRY"
    ALIGN_ENTRY = "ALIGN_ENTRY"
    BACK_IN = "BACK_IN"
    FINAL_DOCK = "FINAL_DOCK"
    DOCKED = "DOCKED"
    REACQUIRE_TAG = "REACQUIRE_TAG"
    ABORT_EXIT = "ABORT_EXIT"
    FAILED = "FAILED"
    UNDOCK_EXIT = "UNDOCK_EXIT"
    UNDOCK_SETTLE = "UNDOCK_SETTLE"
    # 船坞夹爪交互（仅 dock_claw_enabled=true）：归港前请求打开 / 出泊前请求松开
    WAIT_DOCK_OPEN = "WAIT_DOCK_OPEN"
    WAIT_DOCK_RELEASE = "WAIT_DOCK_RELEASE"


# target_mode 取值（FSM 产出，控制器消费）
MODE_HOLD = "HOLD"                    # 零速
MODE_SEARCH = "SEARCH"                # 按 odom 预测方位的扇形搜索（无锚点则自转）
MODE_APPROACH = "APPROACH"            # 弧线到入口外预备点
MODE_ALIGN = "ALIGN"                  # v=0 只修航向
MODE_BACK_IN = "BACK_IN"              # 沿中心线倒入
MODE_FINAL_DOCK = "FINAL_DOCK"        # 低速倒向充电位
MODE_SEARCH_LIMITED = "SEARCH_LIMITED"  # 坞内小角度搜索（±backin_max_search_angle）
MODE_EXIT_FORWARD = "EXIT_FORWARD"    # ABORT_EXIT 沿轴线前进驶出
MODE_UNDOCK_FORWARD = "UNDOCK_FORWARD"  # 出泊前进
MODE_DOCKED_HOLD = "DOCKED_HOLD"      # 入泊保持

# pose_source 取值（estimator 产出）
SRC_VISION = "VISION"
SRC_PREDICTION = "ODOM_PREDICTION"
SRC_INVALID = "INVALID"
