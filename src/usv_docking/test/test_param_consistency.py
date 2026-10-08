#!/usr/bin/env python3
"""跨节点同名参数一致性：把"必须相等 / 必须满足不等式"的隐性契约变成会失败的测试。

背景：`config/docking.yaml` 按节点分段，同一个语义量常出现在 2~4 个节点段里。
ROS 2 参数文件**不支持** YAML 锚点/别名（rcl_yaml_param_parser 直接报
"Will not support aliasing"），也没有跨节点共享参数的机制，所以"一份定义多处
复用"做不到；改一处漏一处不会报错，只会在现场表现为：

  - 目标点/门控不一致 -> 状态机卡死或抖动（如 FSM 与控制器 staging_x 差 0.5m
    大于 approach_x_tol，_staging_reached 永远不满足 -> APPROACH 等到超时）；
  - 话题名不一致 -> 订阅方静默收不到数据（表现为"节点都在跑但全是 INVALID"）；
  - 同名不同义被误"统一" -> 如 estimator/safety 的 odom_prediction_timeout
    （2026-10-07 已改名 anchor_prediction_timeout_sec 消除）。

本文件是纯静态检查（yaml + AST），不需要 rclpy/DDS，可随时快速跑。
"""

import ast
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
MODULES = {
    "docking_pose_estimator": PKG / "usv_docking" / "docking_pose_estimator.py",
    "docking_fsm": PKG / "usv_docking" / "docking_fsm.py",
    "docking_motion_controller": (
        PKG / "usv_docking" / "docking_motion_controller.py"
    ),
    "docking_safety": PKG / "usv_docking" / "docking_safety.py",
}

# ── 语义必须相等（漏改一处会直接现场故障）──
MUST_MATCH = [
    # 目标点/门控：FSM 判"到位/越界"，控制器用同一数值算控制律
    ("staging_x", ["docking_fsm", "docking_motion_controller"]),
    ("final_target_x", ["docking_fsm", "docking_motion_controller"]),
    ("back_in_gate2_y", ["docking_fsm", "docking_motion_controller"]),
    ("back_in_gate2_yaw_deg", ["docking_fsm", "docking_motion_controller"]),
    ("docked_x_tol", ["docking_fsm", "docking_motion_controller"]),
    ("docked_y_tol", ["docking_fsm", "docking_motion_controller"]),
    # 全局裁决：FSM 与 safety 各自计时/防抖，须同尺
    ("max_docking_duration_sec", ["docking_fsm", "docking_safety"]),
    ("violation_cycles", ["docking_fsm", "docking_safety"]),
]

# 撤离链路余量：estimator 锚点推算硬上限须比 safety 撤离窗口多留这么多秒
ABORT_EXIT_MARGIN_SEC = 5.0


def _read_yaml():
    return yaml.safe_load((PKG / "config" / "docking.yaml").read_text())


def _topic_arg(expr, aliases):
    """解析 create_publisher/create_subscription 的 topic 实参。

    返回 ("param", 参数名) / ("literal", 话题字符串) / ("?", None)。
    """
    if isinstance(expr, ast.Constant) and isinstance(expr.value, str):
        return "literal", expr.value
    if isinstance(expr, ast.Attribute) and expr.attr == "value":
        call = expr.value
        if isinstance(call, ast.Call):
            func = call.func
            is_get = (
                isinstance(func, ast.Attribute)
                and func.attr == "get_parameter"
            )
            is_alias = isinstance(func, ast.Name) and func.id in aliases
            if (
                (is_get or is_alias)
                and call.args
                and isinstance(call.args[0], ast.Constant)
            ):
                return "param", call.args[0].value
    return "?", None


def _topic_roles(path):
    """扫描一个节点源文件，返回 {参数名|字面量: {"pub","sub"}}。"""
    tree = ast.parse(path.read_text())
    aliases = set()
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Assign)
            and isinstance(node.targets[0], ast.Name)
            and isinstance(node.value, ast.Attribute)
            and node.value.attr == "get_parameter"
        ):
            aliases.add(node.targets[0].id)
    roles = {}
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)):
            continue
        if node.func.attr == "create_publisher":
            role = "pub"
        elif node.func.attr == "create_subscription":
            role = "sub"
        else:
            continue
        # topic 是第 2 个位置参数：create_*(msg_type, topic, ...)
        if len(node.args) < 2:
            continue
        kind, name = _topic_arg(node.args[1], aliases)
        if kind != "param":
            continue
        roles.setdefault(name, set()).add(role)
    return roles


def _topic_usage():
    """{参数名: {节点: {"pub","sub"}}}"""
    usage = {}
    for module, path in MODULES.items():
        for name, roles in _topic_roles(path).items():
            usage.setdefault(name, {})[module] = roles
    return usage


def _paired_topic_params():
    """既被发布又被订阅的话题参数名 -> 涉及的节点集合。"""
    paired = {}
    for name, per_module in _topic_usage().items():
        roles = [r for rs in per_module.values() for r in rs]
        if "pub" in roles and "sub" in roles:
            paired[name] = set(per_module)
    return paired


def test_pub_sub_topic_params_are_consistent():
    """发布方与订阅方同名话题参数，yaml 值必须相等。"""
    cfg = _read_yaml()
    paired = _paired_topic_params()
    # 防"配对解析失效导致本用例空转"：当前代码里应有 9 对
    assert len(paired) >= 9, (
        f"只解析出 {len(paired)} 组发布/订阅配对，解析逻辑可能已失效："
        f"{sorted(paired)}"
    )
    for name, modules in sorted(paired.items()):
        values = {}
        for module in sorted(modules):
            section = cfg.get(module, {}).get("ros__parameters", {})
            if name in section:
                values[module] = section[name]
        assert len(set(map(repr, values.values()))) <= 1, (
            f"话题参数 {name} 在各节点 yaml 里不一致（发布方收不到）：{values}"
        )


def test_must_match_params_are_consistent():
    """语义必须相等的量，各节点 yaml 值必须一致。"""
    cfg = _read_yaml()
    for key, modules in MUST_MATCH:
        values = {}
        for module in modules:
            section = cfg[module]["ros__parameters"]
            assert key in section, (
                f"{module} 的 yaml 段缺 {key}——该参数必须出现在本清单的每个节点里"
            )
            values[module] = section[key]
        assert len(set(map(repr, values.values()))) == 1, (
            f"{key} 在 {modules} 之间不一致：{values}\n"
            f"（这类参数必须同时改，或用 scripts/dock_tune_gui.py 的同步写）"
        )


def test_duplicated_param_names_are_classified():
    """同名参数必须"有归类"：话题配对 或 显式声明须相等。

    这是防"下一个 odom_prediction_timeout"：同名不同义的参数（estimator 70s
    硬上限 vs safety 3s 运行窗口）极易被后来者误"统一"成同一个值，两个方向
    都会出大事。任何新出现的跨节点同名参数都必须在这里被显式归类。
    """
    cfg = _read_yaml()
    seen = {}
    for module, section in cfg.items():
        for key in section["ros__parameters"]:
            seen.setdefault(key, set()).add(module)
    duplicated = {k: v for k, v in seen.items() if len(v) > 1}

    classified = set(_paired_topic_params())
    classified |= {key for key, _ in MUST_MATCH}

    unclassified = {k: sorted(v) for k, v in duplicated.items() if k not in classified}
    assert not unclassified, (
        "以下同名参数跨节点重复但未归类（要么加进 MUST_MATCH，要么证明独立后"
        f"改名/补注释）：{unclassified}"
    )


def test_time_window_inequalities():
    """时间窗类参数的不等式约束（写错任一方向都会现场故障）。"""
    cfg = _read_yaml()

    def value(module, key):
        return cfg[module]["ros__parameters"][key]

    # 撤离链路：ABORT_EXIT/UNDOCK 全靠锚点推算驶出（可达数十秒），estimator 的
    # 推算硬上限必须覆盖 safety 的撤离窗口，否则驶出中途 pose_source 变 INVALID
    anchor_limit = value("docking_pose_estimator", "anchor_prediction_timeout_sec")
    abort_exit_window = value("docking_safety", "abort_exit_odom_timeout")
    assert anchor_limit >= abort_exit_window + ABORT_EXIT_MARGIN_SEC, (
        f"anchor_prediction_timeout_sec({anchor_limit}) 必须 >= "
        f"abort_exit_odom_timeout({abort_exit_window}) + "
        f"{ABORT_EXIT_MARGIN_SEC}s，否则 ABORT_EXIT/UNDOCK 中途瘫痪"
    )

    # ALIGN 推算窗口必须大于 FSM 的丢 Tag 宽限，否则 safety 抢跑 ODOM_TIMEOUT，
    # ALIGN 的 REACQUIRE 重捕获路径永远走不到（2026-08-31 实测）
    align_window = value("docking_safety", "align_odom_prediction_timeout")
    align_grace = value("docking_fsm", "align_tag_loss_grace_sec")
    assert align_window > align_grace, (
        f"align_odom_prediction_timeout({align_window}) 必须 > "
        f"align_tag_loss_grace_sec({align_grace})，否则 safety 抢跑"
    )
