#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""船端一次性执行工具 —— 由地面站 `waypoint_dispatch_bridge.py` 经 ssh stdin 送入运行。

为什么这样设计：
  * 船端（兴飞 NX / MEC-MB002）没装 rosbridge_server，浏览器无法直连 DDS；
  * 远距离/4G 下船端 `ros2` CLI 偶发卡死（见 AGENTS.md），故一律用 rclpy 直连；
  * 本脚本不落盘、不改船端任何文件：地面站把源码通过 ssh stdin 喂给 `python3 -`，
    船端只多一个短命进程，版本永远跟地面站一致，不会出现船端旧副本。

用法（船端 shell，ROS 环境由调用方 source 好）：
    python3 - check                        # 自检：环境 / 服务 / 当前 /nav_status
    python3 - status                       # 只读一次 /nav_status
    python3 - dispatch service <b64json>   # 调 mission_bridge/send_waypoints（带朝向，立即执行）
    python3 - dispatch topic   <b64json>   # 发 /waypoint（经纬度 JSON，debounce ~0.45s 自动开始）
    python3 - cancel                       # cancel_mission 服务，退化到 /gcs_mission/cancel
    python3 - pause 1|0                    # 暂停 / 继续
    python3 - watch <秒> [周期s]            # 流式监控：/nav_status + /plan 末端(目标) + /cmd_vel_nav
    python3 - diag [秒]                    # 雷达落图自检：TF 链 + 点云落图/高度分布 + 代价图障碍统计

输出约定：stdout 最后一行 `@@RESULT@@ {json}`（前面可能有 source setup.bash 的杂音），
调用方只认这一行。
"""

from __future__ import annotations

import base64
import json
import math
import os
import sys
import time

MARK = "@@RESULT@@"
YAW_UNSPECIFIED = 65536.0
DEFAULT_STATUS_TOPIC = "/nav_status"


def emit(obj: dict, code: int = 0) -> None:
    # ensure_ascii=True：船端非交互 shell 的 locale 可能是 ASCII，
    # 直接输出中文会 UnicodeEncodeError；转义后地面站 json.loads 再还原。
    sys.stdout.write(MARK + " " + json.dumps(obj, ensure_ascii=True) + "\n")
    sys.stdout.flush()
    raise SystemExit(code)


def log(msg: str) -> None:
    try:
        sys.stderr.write(str(msg) + "\n")
    except Exception:  # noqa: BLE001
        pass


def _status_to_dict(msg) -> dict:
    """m_common/msg/NavStatus → 精简 dict（只看关心的字段，版本差异用 hasattr 兜）。"""
    keys = (
        "task_state", "task_id", "command_id", "nav_phase",
        "current_waypoint", "total_waypoints", "progress_percent",
        "elapsed_sec", "distance_to_goal_m", "last_error",
        "planner_status", "controller_status",
        "localization_overall", "localization_gps_fix", "localization_tf_ok",
        "localization_odom_hz", "pose_x", "pose_y", "pose_yaw", "pose_v", "pose_w",
        "flags_manual_override", "flags_emergency_stop", "flags_recovery_active",
        "alerts_odom_stale", "alerts_gps_stale", "alerts_mission_bridge_alive",
    )
    out = {}
    for k in keys:
        if not hasattr(msg, k):
            continue
        v = getattr(msg, k)
        # 只给数值做保留小数；字符串/布尔原样（对 str round 会直接 TypeError）
        if isinstance(v, bool):
            out[k] = v
        elif isinstance(v, (int, float)):
            out[k] = round(v, 4)
        else:
            out[k] = v
    return out


def _import_m_common():
    """返回 (SendWaypoints, SetPause, CancelMission, MissionWaypoint, NavStatus)。"""
    from m_common.srv import CancelMission, SendWaypoints, SetPause
    from m_common.msg import MissionWaypoint, NavStatus
    return SendWaypoints, SetPause, CancelMission, MissionWaypoint, NavStatus


def _qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def read_nav_status(node, timeout: float = 4.0, topic: str = DEFAULT_STATUS_TOPIC):
    """读一帧 /nav_status。同时挂 typed（NavStatus）与 String 两种订阅，谁先来用谁。"""
    from std_msgs.msg import String

    grabbed: dict = {}
    qos = _qos()

    try:
        _sw, _sp, _cm, _mw, NavStatus = _import_m_common()
        node.create_subscription(
            NavStatus, topic,
            lambda m: grabbed.setdefault("typed", _status_to_dict(m)), qos)
    except Exception as exc:  # noqa: BLE001
        grabbed["typed_import_error"] = str(exc)

    def _on_string(m):
        try:
            grabbed.setdefault("json", json.loads(m.data))
        except Exception:  # noqa: BLE001
            grabbed.setdefault("json", {"raw": m.data})

    node.create_subscription(String, topic, _on_string, qos)

    import rclpy
    t0 = time.time()
    while time.time() - t0 < timeout and not any(k in grabbed for k in ("typed", "json")):
        rclpy.spin_once(node, timeout_sec=0.2)
    if "typed" in grabbed:
        grabbed["source"] = "m_common/msg/NavStatus"
        return grabbed
    if "json" in grabbed:
        grabbed["source"] = "std_msgs/String(JSON)"
        return grabbed
    return None


def _client_wait(node, srv_type, name: str, timeout: float):
    import rclpy
    cl = node.create_client(srv_type, name)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cl.service_is_ready():
            return cl
        rclpy.spin_once(node, timeout_sec=0.1)
    return None


def cmd_check(node) -> None:
    import rclpy

    info = {
        "ok": True,
        "cmd": "check",
        "hostname": os.uname().nodename,
        "python": sys.version.split()[0],
        "env": {
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
            "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION"),
            "CYCLONEDDS_URI": os.environ.get("CYCLONEDDS_URI"),
            "AMENT_PREFIX_PATH": os.environ.get("AMENT_PREFIX_PATH", ""),
        },
    }
    try:
        from rclpy.utilities import get_rmw_implementation_identifier
        info["rmw"] = get_rmw_implementation_identifier()
    except Exception as exc:  # noqa: BLE001
        info["rmw_error"] = str(exc)

    SendWaypoints = SetPause = None
    try:
        SendWaypoints, SetPause, _CM, _MW, _NS = _import_m_common()
        info["m_common_import"] = True
    except Exception as exc:  # noqa: BLE001
        info["m_common_import"] = False
        info["m_common_error"] = str(exc)

    # 图（服务）探测：等 2s 让 discovery 收敛
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    names = dict(node.get_service_names_and_types())
    info["services"] = {
        n: (n in names) for n in (
            "/mission_bridge/send_waypoints",
            "/mission_bridge/set_pause",
            "/mission_bridge/cancel_mission",
        )
    }
    info["nodes_visible"] = sorted(node.get_node_names())
    info["topics_visible"] = sorted(
        n for n, _t in node.get_topic_names_and_types()
        if n in ("/waypoint", DEFAULT_STATUS_TOPIC, "/task_event", "/plan", "/map"))

    if SendWaypoints is not None:
        cl = _client_wait(node, SendWaypoints, "/mission_bridge/send_waypoints", 3.0)
        info["send_waypoints_ready"] = cl is not None

    info["nav_status"] = read_nav_status(node, timeout=3.0)
    if info["nav_status"] is None:
        info["ok"] = False
        info["hint"] = ("读不到 /nav_status：确认 mission_bridge.launch.py（含 nav_status_aggregator）"
                        "已启动，且 ROS_DOMAIN_ID / RMW / CYCLONEDDS_URI 与船端一致")
    emit(info)


def cmd_status(node) -> None:
    st = read_nav_status(node, timeout=5.0)
    emit({"ok": st is not None, "cmd": "status", "nav_status": st})


# mission_bridge / zone_manager 实际加载的那份优先（install 拷贝），src 兜底
MAP_YAML_CANDIDATES = (
    "~/usv-main/install/arm/workspace_nav/share/workspace_nav/config/map.yaml",
    "~/usv-main/src/USV_NAV/workspace_nav/config/map.yaml",
)


def cmd_datum() -> None:
    """读船端 map.yaml 的 ref_gnss_* 锚点 + origin（纯文件读取，不需要 ROS）。

    与地面站页面 dispParseAnchors() 同一套两种写法（行内 [lon,lat] / 列表式）。
    """
    import re

    path_used = None
    text = None
    for p in MAP_YAML_CANDIDATES:
        rp = os.path.expanduser(p)
        if os.path.isfile(rp):
            path_used = rp
            with open(rp, encoding="utf-8") as f:
                text = f.read()
            break
    if text is None:
        emit({"ok": False, "cmd": "datum",
              "error": "找不到 map_yaml: " + " | ".join(MAP_YAML_CANDIDATES)}, 3)

    anchors = {}
    re_anchors = re.compile(
        r"(?:^|\n)\s*(ref_gnss(?:_\d\d)?)\s*:\s*"
        r"(?:\[\s*([-+\deE.]+)\s*,\s*([-+\deE.]+)\s*\]"
        r"|\r?\n\s*-\s*([-+\deE.]+)\s*\r?\n\s*-\s*([-+\deE.]+))")
    for m in re_anchors.finditer(text):
        lon = m.group(2) if m.group(2) is not None else m.group(4)
        lat = m.group(3) if m.group(3) is not None else m.group(5)
        anchors[m.group(1)] = [float(lon), float(lat)]

    m_origin = re.search(
        r"(?:^|\n)\s*origin\s*:\s*\[\s*([-+\deE.]+)\s*,\s*([-+\deE.]+)\s*,\s*([-+\deE.]+)\s*\]",
        text)
    origin = ([float(m_origin.group(i)) for i in (1, 2, 3)] if m_origin else None)

    pick = anchors.get("ref_gnss_10") or anchors.get("ref_gnss")
    emit({
        "ok": pick is not None,
        "cmd": "datum",
        "path": path_used,
        "anchors": anchors,
        "origin": origin,
        "datum": ({"lon": pick[0], "lat": pick[1]} if pick else None),
        "error": (None if pick else "map.yaml 里没有 ref_gnss/ref_gnss_10"),
    }, 0 if pick else 4)


def _load_payload(b64: str) -> dict:
    raw = base64.b64decode(b64.encode("ascii")).decode("utf-8")
    data = json.loads(raw)
    if not isinstance(data, dict) or not data.get("waypoints"):
        raise ValueError("payload 缺少 waypoints")
    return data


def _validate(waypoints: list) -> None:
    for i, w in enumerate(waypoints):
        lat = float(w["lat"])
        lon = float(w["lon"])
        if not (math.isfinite(lat) and math.isfinite(lon)):
            raise ValueError(f"航点 {i} 经纬度非法: {lat}, {lon}")
        if lat == 0.0 and lon == 0.0:
            raise ValueError(f"航点 {i} 是 (0,0)，拒绝")
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f"航点 {i} 超出经纬度范围: {lat}, {lon}")


def cmd_dispatch(node, mode: str, b64: str) -> None:
    import rclpy

    payload = _load_payload(b64)
    wps = payload["waypoints"]
    _validate(wps)
    mission_id = str(payload.get("mission_id") or "")
    command_id = str(payload.get("command_id") or "")

    if mode == "topic":
        from std_msgs.msg import String
        topic = str(payload.get("topic") or "/waypoint")
        body = {
            "waypoints": [{"latitude": float(w["lat"]), "longitude": float(w["lon"])} for w in wps],
            "explicit_replan": True,
        }
        if mission_id:
            body["mission_id"] = mission_id
        if command_id:
            body["command_id"] = command_id
        text = json.dumps(body)
        pub = node.create_publisher(String, topic, 10)
        # 连发 3 帧（0.3s 间隔）抗丢包；debounce 窗口内等效一次点击（同 scripts/send_waypoint.py）
        for _ in range(3):
            msg = String()
            msg.data = text
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.3)
        emit({
            "ok": True, "cmd": "dispatch", "mode": "topic", "topic": topic,
            "count": len(wps),
            "message": f"已向 {topic} 连发 3 帧，debounce ≈0.45s 后自动开始（朝向由行进方向决定）",
            "payload": text,
        })

    try:
        SendWaypoints, _SP, _CM, MissionWaypoint, _NS = _import_m_common()
    except Exception as exc:  # noqa: BLE001
        emit({"ok": False, "cmd": "dispatch", "mode": "service",
              "error": f"导入 m_common 失败（需 source ~/usv-main/install/arm/setup.bash）: {exc}"}, 2)
    name = str(payload.get("service") or "/mission_bridge/send_waypoints")
    cl = _client_wait(node, SendWaypoints, name, 4.0)
    if cl is None:
        emit({"ok": False, "cmd": "dispatch", "mode": "service",
              "error": f"服务不可用: {name}（mission_bridge 没起？）"}, 3)

    req = SendWaypoints.Request()
    for i, w in enumerate(wps):
        mw = MissionWaypoint()
        mw.latitude = float(w["lat"])
        mw.longitude = float(w["lon"])
        yaw = w.get("yaw", YAW_UNSPECIFIED)
        mw.yaw = float(yaw) if yaw is not None else YAW_UNSPECIFIED
        mw.seq = int(i)
        req.waypoints.append(mw)
    req.mission_id = mission_id
    req.command_id = command_id

    future = cl.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    res = future.result()
    if res is None:
        emit({"ok": False, "cmd": "dispatch", "mode": "service",
              "error": f"服务 {name} 超时（8s）无响应"}, 4)
    emit({
        "ok": bool(res.success),
        "cmd": "dispatch",
        "mode": "service",
        "service": name,
        "count": len(wps),
        "success": bool(res.success),
        "message": str(res.message),
    }, 0 if res.success else 5)


def cmd_cancel(node) -> None:
    import rclpy
    from std_msgs.msg import Empty

    try:
        _SW, _SP, CancelMission, _MW, _NS = _import_m_common()
    except Exception:  # noqa: BLE001
        CancelMission = None

    if CancelMission is not None:
        cl = _client_wait(node, CancelMission, "/mission_bridge/cancel_mission", 3.0)
        if cl is not None:
            future = cl.call_async(CancelMission.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
            res = future.result()
            if res is None:
                emit({"ok": False, "cmd": "cancel", "error": "cancel_mission 超时"}, 4)
            emit({"ok": bool(res.success), "cmd": "cancel", "via": "service",
                  "success": bool(res.success), "message": str(res.message)},
                 0 if res.success else 5)

    pub = node.create_publisher(Empty, "/gcs_mission/cancel", 10)
    for _ in range(3):
        pub.publish(Empty())
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.2)
    emit({"ok": True, "cmd": "cancel", "via": "topic:/gcs_mission/cancel",
          "message": "已发取消脉冲（服务不可用，退化走 GCS topic）"})


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_matrix(x, y, z, w):
    """四元数 → 3x3 旋转矩阵（行主序 9 元列表）。"""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
        2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
        2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy),
    ]


def _apply(T, p):
    R, t = T
    return (R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + t[0],
            R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + t[1],
            R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + t[2])


def _compose(T_ab, T_bc):
    """T_ab ∘ T_bc（先 bc 后 ab）：返回 T_ac。"""
    Ra, ta = T_ab
    Rb, tb = T_bc
    R = [sum(Ra[3 * i + k] * Rb[3 * k + j] for k in range(3))
         for i in range(3) for j in range(3)]
    t = _apply(T_ab, tb)
    return (R, t)


def _rpy_of(T):
    R, _t = T
    pitch = math.asin(max(-1.0, min(1.0, -R[6])))
    roll = math.atan2(R[7], R[8])
    yaw = math.atan2(R[3], R[0])
    return [round(math.degrees(v), 2) for v in (roll, pitch, yaw)]


def _invert(T):
    R, t = T
    Rt = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]]
    nt = tuple(-(Rt[3 * i] * t[0] + Rt[3 * i + 1] * t[1] + Rt[3 * i + 2] * t[2])
               for i in range(3))
    return (Rt, nt)


def _ascii_count(pts, half=12.0, cell=1.0):
    """俯视图（上=船头 +x，左=左舷 +y）：每格点数，'.'=0，'1'~'9' 饱和计数。"""
    n = int(round(2 * half / cell))
    grid = [["." for _ in range(n)] for _ in range(n)]
    for p in pts:
        x, y = p[0], p[1]
        c = int((half - y) / cell)
        r = int((half - x) / cell)
        if 0 <= r < n and 0 <= c < n:
            v = grid[r][c]
            grid[r][c] = "1" if v == "." else ("9" if v == "9" else str(int(v) + 1))
    return ["".join(row) for row in grid]


def _ascii_sym(items, half=12.0, cell=1.0):
    """俯视图：items=[(x,y,ch)]，'#'(致命) 优先于 '+'(代价)，无则 '.'。"""
    n = int(round(2 * half / cell))
    grid = [["." for _ in range(n)] for _ in range(n)]
    for x, y, ch in items:
        c = int((half - y) / cell)
        r = int((half - x) / cell)
        if 0 <= r < n and 0 <= c < n and (grid[r][c] == "." or ch == "#"):
            grid[r][c] = ch
    return ["".join(row) for row in grid]


def cmd_diag(node, seconds: float = 8.0) -> None:
    """雷达落图自检：真实 TF 链 + 点云落图位置/高度分布 + 局部代价图障碍统计。只读。"""
    import rclpy
    import rclpy.time
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from tf2_msgs.msg import TFMessage

    try:
        from sensor_msgs_py import point_cloud2 as pc2
    except Exception:  # noqa: BLE001
        from sensor_msgs import point_cloud2 as pc2  # type: ignore

    edges: dict = {}
    samples: dict = {}

    def _store(tf_msg, dynamic):
        for t in tf_msg.transforms:
            key = f"{t.header.frame_id} -> {t.child_frame_id}"
            e = edges.setdefault(key, {"dynamic": False})
            e["dynamic"] = e["dynamic"] or dynamic
            e["xyz"] = [round(v, 4) for v in (t.transform.translation.x,
                                             t.transform.translation.y,
                                             t.transform.translation.z)]
            e["rpy_deg"] = _rpy_of((_quat_matrix(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                                    (0.0, 0.0, 0.0)))
            if dynamic and len(samples.get(key, [])) < 600:
                samples.setdefault(key, []).append(
                    (t.transform.translation.x, t.transform.translation.y,
                     t.transform.translation.z,
                     _quat_to_yaw(t.transform.rotation)))
            ages = samples.setdefault(key + "#age", [])
            if len(ages) < 600:
                ages.append(round(time.time()
                                  - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9), 3))

    qos_static = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, "/tf_static", lambda m: _store(m, False), qos_static)
    node.create_subscription(TFMessage, "/tf", lambda m: _store(m, True), 100)

    cloud_holder: dict = {}
    gots = {"cloud": 0}

    def _cloud_cb(msg):
        gots["cloud"] += 1
        if "msg" not in cloud_holder:
            cloud_holder["msg"] = msg
            cloud_holder["recv_at"] = time.time()

    node.create_subscription(PointCloud2, "/livox/lidar", _cloud_cb, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/livox/lidar", _cloud_cb, 10)

    grid_holder: dict = {}
    qos_latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                             lambda m: grid_holder.setdefault("m", m), qos_latched)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                             lambda m: grid_holder.setdefault("g", m), qos_latched)
    map_holder: dict = {}
    node.create_subscription(OccupancyGrid, "/map",
                             lambda m: map_holder.setdefault("m", m), qos_latched)

    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
        # 三样到齐后再多等一会（代价图 1~5Hz、/map 是 latch 语义）
        if time.time() - t0 > 3.0 and "msg" in cloud_holder \
                and "m" in grid_holder and "m" in map_holder:
            break

    out: dict = {"ok": True, "cmd": "diag"}

    # ---- 1) TF 链 ----
    def _T_from_edge(key):
        e = edges.get(key)
        if not e:
            return None
        xyz = e["xyz"]
        rpy = [math.radians(v) for v in e["rpy_deg"]]
        cr, cp, cy = math.cos(rpy[0]), math.cos(rpy[1]), math.cos(rpy[2])
        sr, sp, sy = math.sin(rpy[0]), math.sin(rpy[1]), math.sin(rpy[2])
        R = [cp * cy, cy * sp * sr - sy * cr, sy * sp * sr + cy * cr,
             sy * cp, cy * cr + sy * sp * sr, -cy * sr + sy * sp * cr,
             -sp, cp * sr, cp * cr]
        return (R, tuple(xyz))

    def _chain(root, target):
        """BFS 求 root→target 的复合变换。别"取第一个 child" —— 同一父节点有多个分支。"""
        if root == target:
            return ([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], (0.0, 0.0, 0.0))
        children: dict = {}
        for key in edges:
            a, b = key.split(" -> ")
            children.setdefault(a, []).append(key)
        queue = [(root, ([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], (0.0, 0.0, 0.0)))]
        seen = {root}
        while queue:
            cur, T = queue.pop(0)
            for key in children.get(cur, []):
                child = key.split(" -> ")[1]
                if child in seen:
                    continue
                Tc = _compose(T, _T_from_edge(key))
                if child == target:
                    return Tc
                seen.add(child)
                queue.append((child, Tc))
        return None

    rel = [k for k in edges if any(f in k for f in
                                   ("livox", "imu_link", "base_link", "odom", "map"))]
    out["tf_edges"] = {k: edges[k] for k in sorted(rel)}
    # 动态边多帧采样：值在跳 = 有多个发布者（或锚点在重置）
    tf_sampling = {}
    for key, vals in samples.items():
        if key.endswith("#age"):
            continue
        if not any(f in key for f in ("map -> odom", "odom -> base_link", "map -> base_link")):
            continue
        xs = [v[0] for v in vals]
        ys = [v[1] for v in vals]
        yaws = [math.degrees(v[3]) for v in vals]
        mx, my, myaw = sorted(xs)[len(xs) // 2], sorted(ys)[len(ys) // 2], sorted(yaws)[len(yaws) // 2]
        jump = 0.0
        for a, b in zip(vals, vals[1:]):
            jump = max(jump, math.hypot(b[0] - a[0], b[1] - a[1]))
        tf_sampling[key] = {
            "n": len(vals),
            "x_min_max": [round(min(xs), 2), round(max(xs), 2)],
            "y_min_max": [round(min(ys), 2), round(max(ys), 2)],
            "yaw_min_max_deg": [round(min(yaws), 1), round(max(yaws), 1)],
            "median": [round(mx, 3), round(my, 3), round(myaw, 1)],
            "max_jump_m": round(jump, 3),
            "spread_m": round(max(math.hypot(v[0] - mx, v[1] - my) for v in vals), 3),
        }
        ag = samples.get(key + "#age") or [0.0]
        tf_sampling[key]["stamp_age_sec_latest"] = ag[-1]
        tf_sampling[key]["stamp_age_sec_min_max"] = [min(ag), max(ag)]
    out["tf_sampling"] = tf_sampling
    for root, target, tag in (("map", "livox_frame", "map_to_livox"),
                              ("odom", "livox_frame", "odom_to_livox"),
                              ("base_link", "livox_frame", "base_to_livox"),
                              ("map", "odom", "map_to_odom"),
                              ("odom", "base_link", "odom_to_base")):
        T = _chain(root, target)
        out[tag] = None if T is None else {
            "xyz": [round(v, 3) for v in T[1]], "rpy_deg": _rpy_of(T)}

    # ---- 2) 点云落图 ----
    msg = cloud_holder.get("msg")
    if msg is None:
        out["ok"] = False
        out["error"] = "收不到 /livox/lidar（驱动没起 / QoS / 域不对）"
        emit(out)
    fields = [f.name for f in msg.fields]
    pts = []
    want = ("x", "y", "z") + (("tag",) if "tag" in fields else ())
    tags = []
    for i, p in enumerate(pc2.read_points(msg, field_names=want, skip_nans=True)):
        if i >= 40000:
            break
        pts.append((float(p[0]), float(p[1]), float(p[2])))
        if len(want) > 3:
            try:
                tags.append(int(p[3]))
            except (TypeError, ValueError):
                tags.append(-1)
    # 注意：用"收到时刻"算 age，不能用出循环后的现在（否则会虚高好几秒）
    age = cloud_holder.get("recv_at", time.time()) - (msg.header.stamp.sec
                                                      + msg.header.stamp.nanosec * 1e-9)
    out["cloud"] = {
        "frame_id": msg.header.frame_id,
        "stamp_age_sec": round(age, 3),
        "width_height": [msg.width, msg.height],
        "fields": fields,
        "points_sampled": len(pts),
        "is_dense": bool(msg.is_dense),
        "fps_observed": gots["cloud"],
    }
    if tags:
        hist = {}
        for t in tags:
            hist[t] = hist.get(t, 0) + 1
        out["cloud_tag_hist"] = {f"tag={k}": v for k, v in sorted(hist.items())}

    def _tf_pts(T, ps):
        return [_apply(T, p) for p in ps]

    def _bbox(ps):
        if not ps:
            return None
        return [[round(min(p[i] for p in ps), 2), round(max(p[i] for p in ps), 2)]
                for i in range(3)]

    if pts:
        out["bbox_cloud_frame"] = _bbox(pts)
        T_bl_livox = _chain("base_link", "livox_frame")
        T_odom_livox = _chain("odom", "livox_frame")
        pts_bl = [_apply(T_bl_livox, p) for p in pts] if T_bl_livox else []
        pts_odom = [_apply(T_odom_livox, p) for p in pts] if T_odom_livox else []
        if pts_bl:
            out["bbox_base_link_frame"] = _bbox(pts_bl)
            out["俯视_点云在船体系(上=船头+12m,下=船尾-12m,左=左舷+y,每格1m)"] = _ascii_count(pts_bl)
            far = [(p[0], p[1]) for p in pts_bl if math.hypot(p[0], p[1]) > 8.0]
            if far:
                out["far_centroid_base_link"] = {
                    "dx_前": round(sum(p[0] for p in far) / len(far), 2),
                    "dy_左": round(sum(p[1] for p in far) / len(far), 2),
                    "n": len(far)}
            near_idx = [(p, math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2)) for p in pts]
            near = [pb for pb, dd in zip(pts_bl, [x[1] for x in near_idx]) if dd < 1.4]
            if near:
                out["near_cluster_lt1.4m"] = {
                    "n": len(near),
                    "bbox_base_link": _bbox(near),
                    "bbox_livox_frame": _bbox([pl for pl, dd in near_idx if dd < 1.4]),
                }
            near_pl = [pl for pl, dd in near_idx if dd < 1.4]
            if near_pl:
                dd_hist = {"0~0.2m": 0, "0.2~0.5m": 0, "0.5~1.0m": 0, "1.0~1.4m": 0}
                for pl, dd in near_idx:
                    if dd >= 1.4:
                        continue
                    if dd < 0.2:
                        dd_hist["0~0.2m"] += 1
                    elif dd < 0.5:
                        dd_hist["0.2~0.5m"] += 1
                    elif dd < 1.0:
                        dd_hist["0.5~1.0m"] += 1
                    else:
                        dd_hist["1.0~1.4m"] += 1
                out["near_cluster_distance_hist"] = dd_hist
                out["near_cluster_x_sign_livox"] = {
                    "x<-0.3(轴后方)": sum(1 for p in near_pl if p[0] < -0.3),
                    "-0.3~0": sum(1 for p in near_pl if -0.3 <= p[0] < 0),
                    "x>0(轴前方)": sum(1 for p in near_pl if p[0] >= 0)}
            # 前方扇区最近障碍（1.4~15m 内，±30°）
            fwd = []
            for pb, dd in zip(pts_bl, [x[1] for x in near_idx]):
                if 1.4 <= dd <= 15.0:
                    ang = math.degrees(math.atan2(pb[1], pb[0]))
                    if abs(ang) <= 30.0:
                        fwd.append(dd)
            out["forward_sector_1.4_15m"] = {
                "n": len(fwd), "nearest_m": (round(min(fwd), 2) if fwd else None)}
            elev = {"<-8度": 0, "-8~0度": 0, "0~15度": 0, "15~30度": 0,
                    "30~50度": 0, ">50度": 0}
            for p in pts_bl:
                e = math.degrees(math.atan2(p[2], math.hypot(p[0], p[1])))
                if e < -8:
                    elev["<-8度"] += 1
                elif e < 0:
                    elev["-8~0度"] += 1
                elif e < 15:
                    elev["0~15度"] += 1
                elif e < 30:
                    elev["15~30度"] += 1
                elif e < 50:
                    elev["30~50度"] += 1
                else:
                    elev[">50度"] += 1
            out["elevation_hist_base_link_仰角"] = elev
        d = [math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2) for p in pts]
        out["range_hist"] = {
            "雷达1.4m内_被丢": sum(1 for v in d if v < 1.4),
            "1.4~15m_入图": sum(1 for v in d if 1.4 <= v <= 15.0),
            "15~30m_超窗口": sum(1 for v in d if 15.0 < v <= 30.0),
            "30m以上": sum(1 for v in d if v > 30.0),
            "max": round(max(d), 1) if d else None}
        if pts_odom:
            zs = [p[2] for p in pts_odom]
            out["odom_z_hist_水面约-0.48_体素层上限0.6"] = {
                "-0.35以下_被观测源丢": sum(1 for z in zs if z < -0.35),
                "-0.35~0": sum(1 for z in zs if -0.35 <= z < 0.0),
                "0~0.3": sum(1 for z in zs if 0.0 <= z < 0.3),
                "0.3~0.6_入体素": sum(1 for z in zs if 0.3 <= z < 0.6),
                "0.6~1.0_被voxel层丢": sum(1 for z in zs if 0.6 <= z < 1.0),
                "1.0以上_被观测源丢": sum(1 for z in zs if z >= 1.0)}
            kept = [p for p, dd in zip(pts_odom, d)
                    if 1.4 <= dd <= 15.0 and -0.35 <= p[2] <= 1.0]
            voxel = [p for p, dd in zip(pts_odom, d)
                     if 1.4 <= dd <= 15.0 and -0.35 <= p[2] <= 0.6]
            out["filter_survival"] = {
                "total": len(pts_odom),
                "after_obs_filter(距离+高度)": len(kept),
                "after_voxel_max_obstacle_height_0.6": len(voxel)}

    # ---- 3) 代价图障碍统计 ----
    for key, name in (("m", "local_costmap"), ("g", "global_costmap")):
        g = grid_holder.get(key)
        if not g:
            out[f"{name}_obstacles"] = None
            continue
        lethal = cost = free = unknown = 0
        xs, ys = [], []
        lcells, ccells = [], []
        info = g.info
        for idx, v in enumerate(g.data):
            cx = info.origin.position.x + (idx % info.width + 0.5) * info.resolution
            cy = info.origin.position.y + (idx // info.width + 0.5) * info.resolution
            if v >= 99:
                lethal += 1
                xs.append(cx)
                ys.append(cy)
                lcells.append((cx, cy))
            elif v > 0:
                cost += 1
                ccells.append((cx, cy))
            elif v == 0:
                free += 1
            else:
                unknown += 1
        out[f"{name}_obstacles"] = {
            "lethal": lethal, "cost": cost, "free": free, "unknown": unknown,
            "frame": g.header.frame_id, "res": round(info.resolution, 3),
            "size": [info.width, info.height],
            "lethal_bbox": None if not xs else [[round(min(xs), 2), round(max(xs), 2)],
                                                [round(min(ys), 2), round(max(ys), 2)]],
        }
        if name == "local_costmap" and lcells:
            T_odom_bl = _chain("odom", "base_link")
            if T_odom_bl:
                T_bl_odom = _invert(T_odom_bl)
                sym = [(_apply(T_bl_odom, (c[0], c[1], 0.0))[0],
                        _apply(T_bl_odom, (c[0], c[1], 0.0))[1], "+") for c in ccells]
                sym += [(_apply(T_bl_odom, (c[0], c[1], 0.0))[0],
                         _apply(T_bl_odom, (c[0], c[1], 0.0))[1], "#") for c in lcells]
                out["boat_pose_odom"] = {
                    "x": round(T_odom_bl[1][0], 2), "y": round(T_odom_bl[1][1], 2),
                    "yaw_deg": _rpy_of(T_odom_bl)[2]}
                out["俯视_局部代价图在船体系(上=船头,左=左舷,#=致命,+=膨胀代价,每格1m)"] = _ascii_sym(sym)

    st = None
    try:
        st = read_nav_status(node, timeout=2.0)
    except Exception:  # noqa: BLE001
        pass
    if st:
        typed = st.get("typed") or {}
        out["boat_pose_map"] = {"x": typed.get("pose_x"), "y": typed.get("pose_y"),
                                "yaw_deg": round(math.degrees(typed.get("pose_yaw") or 0.0), 1),
                                "loc": typed.get("localization_overall"),
                                "gps_fix": typed.get("localization_gps_fix")}
    emit(out)


def _ang_diff_deg(target_deg: float, cur_rad: float) -> float:
    d = math.radians(target_deg) - cur_rad
    return math.degrees(math.atan2(math.sin(d), math.cos(d)))


def cmd_params(node, target: str = "/local_costmap/local_costmap",
               prefixes: str = "", names: str = "") -> None:
    """只读：列/读运行中节点的参数（默认看局部代价图 voxel/inflation 的关键值）。"""
    import rclpy
    from rcl_interfaces.srv import GetParameters, ListParameters

    out = {"ok": True, "cmd": "params", "node": target}
    lp = node.create_client(ListParameters, f"{target}/list_parameters")
    if not lp.wait_for_service(timeout_sec=4.0):
        emit({"ok": False, "cmd": "params",
              "error": f"服务不可用: {target}/list_parameters"}, 3)
    req = ListParameters.Request()
    req.prefixes = [p for p in (prefixes or "voxel_layer,inflation_layer").split(",") if p]
    req.depth = 4
    fut = lp.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=6.0)
    res = fut.result()
    if res is None:
        emit({"ok": False, "cmd": "params", "error": "list_parameters 超时"}, 4)
    out["names"] = sorted(res.result.names)

    want = [n for n in (names or "").split(",") if n]
    if want:
        gp = node.create_client(GetParameters, f"{target}/get_parameters")
        if gp.wait_for_service(timeout_sec=4.0):
            req2 = GetParameters.Request()
            req2.names = want
            fut2 = gp.call_async(req2)
            rclpy.spin_until_future_complete(node, fut2, timeout_sec=6.0)
            r2 = fut2.result()
            if r2 is not None:
                vals = {}
                for n, v in zip(want, r2.values):
                    if v.type == 1:
                        vals[n] = bool(v.bool_value)
                    elif v.type == 2:
                        vals[n] = int(v.integer_value)
                    elif v.type == 3:
                        vals[n] = round(float(v.double_value), 3)
                    elif v.type == 4:
                        vals[n] = v.string_value
                    else:
                        vals[n] = f"(type={v.type} 未设置)"
                out["values"] = vals
    emit(out)


def cmd_count(node, topic: str = "/local_costmap/costmap", seconds: float = 4.0) -> None:
    """只读：统计一张 OccupancyGrid 的格值分布，用来确认"代价图清掉没有"。"""
    import rclpy
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from nav_msgs.msg import OccupancyGrid

    holder: dict = {}
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, topic,
                             lambda m: holder.__setitem__("m", m), qos)
    node.create_subscription(OccupancyGrid, topic,
                             lambda m: holder.__setitem__("m", m), 1)
    t0 = time.time()
    while time.time() - t0 < seconds and "m" not in holder:
        rclpy.spin_once(node, timeout_sec=0.1)
    g = holder.get("m")
    if g is None:
        emit({"ok": False, "cmd": "count", "topic": topic,
              "error": "收不到该 OccupancyGrid（话题名/QoS/是否在发）"}, 3)
    lethal = cost = free = unknown = 0
    for v in g.data:
        if v >= 99:
            lethal += 1
        elif v > 0:
            cost += 1
        elif v == 0:
            free += 1
        else:
            unknown += 1
    emit({"ok": True, "cmd": "count", "topic": topic,
          "frame": g.header.frame_id, "stamp_age_sec":
              round(time.time() - (g.header.stamp.sec + g.header.stamp.nanosec * 1e-9), 2),
          "size": [g.info.width, g.info.height], "res": round(g.info.resolution, 3),
          "lethal_99plus": lethal, "cost_1_98": cost, "free_0": free, "unknown_-1": unknown})


def _mk_edge_T(e):
    """TF 单边 → (R, t)。"""
    xyz = e["xyz"]
    rpy = [math.radians(v) for v in e["rpy_deg"]]
    cr, cp, cy = math.cos(rpy[0]), math.cos(rpy[1]), math.cos(rpy[2])
    sr, sp, sy = math.sin(rpy[0]), math.sin(rpy[1]), math.sin(rpy[2])
    R = [cp * cy, cy * sp * sr - sy * cr, sy * sp * sr + cy * cr,
         sy * cp, cy * cr + sy * sp * sr, -cy * sr + sy * sp * cr,
         -sp, cp * sr, cp * cr]
    return (R, tuple(xyz))


def _mk_chain(edges, root, target):
    """用 edges{key:{xyz,rpy_deg}} 求 root→target 复合变换（BFS）。"""
    if root == target:
        return ([1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0], (0.0, 0.0, 0.0))
    children = {}
    for key in edges:
        a, b = key.split(" -> ")
        children.setdefault(a, []).append(key)
    queue = [(root, ([1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0], (0.0, 0.0, 0.0)))]
    seen = {root}
    while queue:
        cur, T = queue.pop(0)
        for key in children.get(cur, []):
            child = key.split(" -> ")[1]
            if child in seen:
                continue
            Tc = _compose(T, _mk_edge_T(edges[key]))
            if child == target:
                return Tc
            seen.add(child)
            queue.append((child, Tc))
    return None


def cmd_tfcheck(node, seconds: float = 6.0) -> None:
    """只读：查 /tf、/tf_static 有没有**重复发布者**（同一段 TF 两个值会互相打架）。

    做法：同一 (parent, child) 键在采样窗口内出现 ≥2 个不同取值 = 有多个发布者/值不一致。
    典型症状：近景（以传感器为原点的团块）看着正常、远景整体偏几米。
    """
    import rclpy
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from tf2_msgs.msg import TFMessage

    qos_static = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    seen: dict = {}
    counts: dict = {}

    def _cb(m):
        for t in m.transforms:
            key = f"{t.header.frame_id} -> {t.child_frame_id}"
            val = (round(t.transform.translation.x, 4), round(t.transform.translation.y, 4),
                   round(t.transform.translation.z, 4),
                   round(t.transform.rotation.x, 4), round(t.transform.rotation.y, 4),
                   round(t.transform.rotation.z, 4), round(t.transform.rotation.w, 4))
            seen.setdefault(key, set()).add(val)
            counts[key] = counts.get(key, 0) + 1

    node.create_subscription(TFMessage, "/tf_static", _cb, qos_static)
    node.create_subscription(TFMessage, "/tf", _cb, 100)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    conflict = {k: sorted(v) for k, v in seen.items() if len(v) > 1}
    out = {"ok": True, "cmd": "tfcheck", "seconds": seconds,
           "edges_seen": len(seen),
           "冲突的边(同一键多个值)_个数": len(conflict),
           "冲突详情": {k: v for k, v in list(conflict.items())[:6]},
           "base_link->livox_frame 取值": sorted(seen.get("base_link -> livox_frame", [])),
           "base_link->livox_frame 出现次数": counts.get("base_link -> livox_frame")}
    for topic in ("/tf_static", "/tf"):
        try:
            pubs = node.get_publishers_info_by_topic(topic)
            out[f"{topic}_publishers"] = sorted({p.node_name for p in pubs})
        except Exception as exc:  # noqa: BLE001
            out[f"{topic}_publishers_error"] = str(exc)
    try:
        subs = node.get_subscriptions_info_by_topic("/tf_static")
        out["tf_static_subscriber_count"] = len(subs)
    except Exception:  # noqa: BLE001
        pass
    emit(out)


def cmd_voxelmap(node, seconds: float = 4.0,
                 topic: str = "/local_costmap/voxel_grid") -> None:
    """只读：解码 /local_costmap/voxel_grid（nav2_msgs/VoxelGrid），画出"每个格子被标过几个 z 体素"。


    这正是 nav2 与 `mark_threshold` 比较的那个量：
    列 data 是 uint32（低 16 位=known 掩码，高 16 位=marked 掩码，bit z 表示该 z 体素被标过）。
    """
    import rclpy
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from nav2_msgs.msg import VoxelGrid
    from tf2_msgs.msg import TFMessage

    edges, holder = {}, {}
    qos_static = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    qos_latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def _store(m):
        for t in m.transforms:
            key = f"{t.header.frame_id} -> {t.child_frame_id}"
            edges[key] = {"xyz": [t.transform.translation.x, t.transform.translation.y,
                                  t.transform.translation.z],
                          "rpy_deg": _rpy_of((_quat_matrix(t.transform.rotation.x,
                                                           t.transform.rotation.y,
                                                           t.transform.rotation.z,
                                                           t.transform.rotation.w),
                                              (0.0, 0.0, 0.0)))}

    node.create_subscription(TFMessage, "/tf_static", _store, qos_static)
    node.create_subscription(TFMessage, "/tf", _store, 100)
    node.create_subscription(VoxelGrid, topic, lambda m: holder.__setitem__("m", m), qos_latch)
    node.create_subscription(VoxelGrid, topic, lambda m: holder.__setitem__("m", m), 1)

    t0 = time.time()
    while time.time() - t0 < seconds and "m" not in holder:
        rclpy.spin_once(node, timeout_sec=0.1)
    vg = holder.get("m")
    if vg is None:
        emit({"ok": False, "cmd": "voxelmap", "topic": topic,
              "error": "收不到该 VoxelGrid（话题名 / publish_voxel_map 是否开）"}, 3)

    sx, sy, sz = int(vg.size_x), int(vg.size_y), int(vg.size_z)
    res, zres = float(vg.resolutions.x), float(vg.resolutions.z)
    ox, oy, oz = float(vg.origin.x), float(vg.origin.y), float(vg.origin.z)
    hist, zbins, cells = {}, {}, []
    for j in range(sy):
        for i in range(sx):
            marked = int(vg.data[j * sx + i]) >> 16
            if not marked:
                continue
            nb = bin(marked).count("1")
            hist[nb] = hist.get(nb, 0) + 1
            for k in range(sz):
                if marked & (1 << k):
                    z = round(oz + (k + 0.5) * zres, 2)
                    zbins[z] = zbins.get(z, 0) + 1
            cells.append((ox + (i + 0.5) * res, oy + (j + 0.5) * res, nb))

    out = {"ok": True, "cmd": "voxelmap", "topic": topic, "frame": vg.header.frame_id,
           "size": [sx, sy, sz], "res_xy_z": [round(res, 3), round(zres, 3)],
           "origin": [round(ox, 2), round(oy, 2), round(oz, 2)],
           "stamp_age_sec": round(time.time() - (vg.header.stamp.sec
                                                 + vg.header.stamp.nanosec * 1e-9), 2),
           "有标记的列数": sum(hist.values()),
           "每列被标z体素个数_直方图": {f"{k}个": v for k, v in sorted(hist.items())},
           "现在就会标成障碍(>=2层)": sum(v for k, v in hist.items() if k >= 2),
           "被阈值挡掉(只有1层)": hist.get(1, 0),
           "被标z体素的高度分布(odom z)": dict(sorted(zbins.items()))}

    Tb = _mk_chain(edges, "odom", "base_link")
    n, half = 24, 12.0
    grid = [["." for _ in range(n)] for _ in range(n)]
    if Tb:
        Tbi = _invert(Tb)
        for (x, y, nb) in cells:
            b = _apply(Tbi, (x, y, 0.0))
            c, r = int((half - b[1]) / 1.0), int((half - b[0]) / 1.0)
            if 0 <= r < n and 0 <= c < n:
                if nb >= 2:
                    grid[r][c] = "#"
                elif grid[r][c] == ".":
                    grid[r][c] = "1"
    out["俯视_船体系(上=船头,#=现在已标 1=只1个z层_被mark_threshold挡掉)"] = [
        "".join(r) for r in grid]
    emit(out)


def cmd_voxelcheck(node, seconds: float = 5.0,
                   vt: str = "/local_costmap/voxel_marked_cloud") -> None:
    """（旧）点云 vs voxel_marked_cloud；本版 nav2 不发布该话题，改用 cmd_voxelmap。"""
    _ = vt
    """只读：对比 /local_costmap/voxel_grid（voxel层内部网格）与当前 /livox/lidar，
    判断代价图里的标记是"当前点云标的"还是"旧位置残留"。"""
    import rclpy
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    from sensor_msgs.msg import PointCloud2
    from tf2_msgs.msg import TFMessage
    try:
        from sensor_msgs_py import point_cloud2 as pc2
    except Exception:  # noqa: BLE001
        from sensor_msgs import point_cloud2 as pc2  # type: ignore

    edges, vox, clouds = {}, {}, []

    def _store(tf_msg):
        for t in tf_msg.transforms:
            key = f"{t.header.frame_id} -> {t.child_frame_id}"
            edges[key] = {"xyz": [t.transform.translation.x, t.transform.translation.y,
                                  t.transform.translation.z],
                          "rpy_deg": _rpy_of((_quat_matrix(
                              t.transform.rotation.x, t.transform.rotation.y,
                              t.transform.rotation.z, t.transform.rotation.w), (0, 0, 0)))}

    qos_static = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, "/tf_static", _store, qos_static)
    node.create_subscription(TFMessage, "/tf", _store, 100)
    for topic, sink in (("/livox/lidar", "c"), (vt, "v")):
        cb = (lambda m: clouds.append(m)) if sink == "c" else (lambda m: vox.__setitem__("m", m))
        node.create_subscription(PointCloud2, topic, cb, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, topic, cb, 10)

    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    out = {"ok": True, "cmd": "voxelcheck"}
    if not clouds or "m" not in vox:
        emit({"ok": False, "cmd": "voxelcheck",
              "error": f"缺数据：livox={len(clouds)} 帧, voxel_grid={'有' if 'm' in vox else '无'}"})
    T = _mk_chain(edges, "odom", "livox_frame")
    vmsg = vox["m"]
    Tv = None if vmsg.header.frame_id in ("odom", "") else _mk_chain(edges, "odom", vmsg.header.frame_id)
    origin = T[1]
    # 当前点云的"可标记"点集合（1.4~15m、z 0~0.6）
    sup = set()
    pts_ok = 0
    for m in clouds[-3:]:
        for p in pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True):
            q = _apply(T, (float(p[0]), float(p[1]), float(p[2])))
            d3 = math.dist(q, origin)
            if 1.4 <= d3 <= 15.0 and 0.0 <= q[2] <= 0.6:
                pts_ok += 1
                sup.add((round(q[0] / 0.25), round(q[1] / 0.25), round(q[2] / 0.25)))
    vpts = []
    for p in pc2.read_points(vmsg, field_names=("x", "y", "z"), skip_nans=True):
        q = (float(p[0]), float(p[1]), float(p[2]))
        if Tv:
            q = _apply(Tv, q)
        vpts.append(q)
    supported = sum(1 for q in vpts
                    if (round(q[0] / 0.25), round(q[1] / 0.25), round(q[2] / 0.25)) in sup)
    stale = [q for q in vpts
             if (round(q[0] / 0.25), round(q[1] / 0.25), round(q[2] / 0.25)) not in sup]
    dz = sorted(round(q[2], 2) for q in vpts)
    out.update({
        "voxel_grid": {"frame": vmsg.header.frame_id, "points": len(vpts),
                       "stamp_age_sec": round(time.time() - (vmsg.header.stamp.sec
                                                             + vmsg.header.stamp.nanosec * 1e-9), 2),
                       "z_range": [dz[0], dz[-1]] if dz else None},
        "current_cloud": {"frames_used": len(clouds[-3:]), "points_markable(1.4~15m,z0~0.6)": pts_ok},
        "voxel_points_with_cloud_support": supported,
        "voxel_points_WITHOUT_support(疑似残留)": len(stale),
    })
    if vpts:
        out["voxel_distance_from_sensor_m"] = {
            "min": round(min(math.dist(q, origin) for q in vpts), 2),
            "median": round(sorted(math.dist(q, origin) for q in vpts)[len(vpts) // 2], 2),
            "max": round(max(math.dist(q, origin) for q in vpts), 2)}
    Tb = _mk_chain(edges, "odom", "base_link")
    if Tb:
        Tbi = _invert(Tb)
        n, half = 24, 12.0
        grid = [["." for _ in range(n)] for _ in range(n)]
        for q in vpts:                       # '#' = voxel 层里的标记
            b = _apply(Tbi, q)
            c, r = int((half - b[1]) / 1.0), int((half - b[0]) / 1.0)
            if 0 <= r < n and 0 <= c < n:
                grid[r][c] = "#"
        for m in clouds[-2:]:                # 'o' = 当前点云可标记点
            for p in pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True):
                q = _apply(T, (float(p[0]), float(p[1]), float(p[2])))
                if not (1.4 <= math.dist(q, origin) <= 15.0 and 0.0 <= q[2] <= 0.6):
                    continue
                b = _apply(Tbi, q)
                c, r = int((half - b[1]) / 1.0), int((half - b[0]) / 1.0)
                if 0 <= r < n and 0 <= c < n and grid[r][c] == ".":
                    grid[r][c] = "o"
        out["俯视_船体系(上=船头,#=voxel层标记,o=当前点云可标记点)"] = ["".join(r) for r in grid]
    emit(out)


def cmd_compare(node, seconds: float = 4.0, topic: str = "/local_costmap/costmap") -> None:
    """只读：把 /livox/lidar 的点云与局部代价图**逐格**对比，量化"该标没标 / 多标"。

    过滤口径按**船端 1.1.20 的旧逻辑**：点到传感器原点的 3D 距离 ∈ [1.4, 15]，
    z(odom) ∈ [-0.35, 1.0]（观测源），再过 voxel 层 z ≤ 0.6、mark_threshold ≥ 2。
    """
    import rclpy
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from tf2_msgs.msg import TFMessage
    try:
        from sensor_msgs_py import point_cloud2 as pc2
    except Exception:  # noqa: BLE001
        from sensor_msgs import point_cloud2 as pc2  # type: ignore

    edges, clouds, grids = {}, [], {}
    tf_age = {}

    def _store(tf_msg):
        for t in tf_msg.transforms:
            key = f"{t.header.frame_id} -> {t.child_frame_id}"
            edges[key] = {
                "xyz": [t.transform.translation.x, t.transform.translation.y,
                        t.transform.translation.z],
                "rpy_deg": _rpy_of((_quat_matrix(t.transform.rotation.x, t.transform.rotation.y,
                                                t.transform.rotation.z, t.transform.rotation.w),
                                    (0.0, 0.0, 0.0)))}
            tf_age[key] = round(time.time() - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9), 3)

    qos_static = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, "/tf_static", _store, qos_static)
    node.create_subscription(TFMessage, "/tf", _store, 100)
    node.create_subscription(PointCloud2, "/livox/lidar", lambda m: clouds.append(m),
                             qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/livox/lidar", lambda m: clouds.append(m), 10)
    qos_latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, topic,
                             lambda m: grids.__setitem__("m", m), qos_latched)
    node.create_subscription(OccupancyGrid, topic,
                             lambda m: grids.__setitem__("m", m), 1)

    t0 = time.time()
    while time.time() - t0 < seconds or not clouds:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > seconds + 6.0:
            break

    out = {"ok": True, "cmd": "compare", "topic": topic, "seconds": seconds}
    g = grids.get("m")
    if not clouds:
        out["ok"] = False
        out["error"] = "没收到 /livox/lidar（雷达驱动没跑？）"
    if g is None:
        out["ok"] = False
        out["error"] = (out.get("error", "") + " 没收到局部代价图").strip()
    if out["ok"]:
        T = _mk_chain(edges, "odom", "livox_frame")
        if T is None:
            out["ok"] = False
            out["error"] = "TF 里找不到 odom→…→livox_frame"
    if out["ok"]:
        out["tf_stamp_age_sec"] = {k: tf_age.get(k) for k in
                                   ("odom -> base_link", "map -> odom", "base_link -> livox_frame")}
        origin = T[1]
        out["sensor_origin_odom"] = [round(v, 2) for v in origin]
        T_ob = _mk_chain(edges, "odom", "base_link")
        if T_ob:
            out["boat_pose_odom"] = {"x": round(T_ob[1][0], 2), "y": round(T_ob[1][1], 2),
                                     "yaw_deg": _rpy_of(T_ob)[2]}
        try:
            pubs = node.get_publishers_info_by_topic("/livox/lidar")
            out["livox_lidar_publishers"] = [
                {"node": p.node_name, "qos": f"{p.qos_profile.reliability.name}/"
                                             f"{p.qos_profile.durability.name}"} for p in pubs]
        except Exception as exc:  # noqa: BLE001
            out["livox_lidar_publishers_error"] = str(exc)
        info = g.info
        out["costmap"] = {"frame": g.header.frame_id,
                          "stamp_age_sec": round(time.time() - (
                              g.header.stamp.sec + g.header.stamp.nanosec * 1e-9), 2),
                          "origin": [round(info.origin.position.x, 2),
                                     round(info.origin.position.y, 2)]}
        hits = {}
        hits_layer = {}
        zhist = {"<0（被layer.min_obstacle_height=0丢）": 0, "0~0.3（入体素）": 0,
                 "0.3~0.6（入体素）": 0, "0.6~1.0（被layer.max=0.6丢）": 0}
        pts_used = 0
        pts_kept = 0
        pts_layer = 0
        dists_kept = []
        per_frame = []
        for msg in clouds[-8:]:
            frame_hits = {}
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                pts_used += 1
                q = _apply(T, (float(p[0]), float(p[1]), float(p[2])))
                d3 = math.sqrt((q[0] - origin[0]) ** 2 + (q[1] - origin[1]) ** 2
                               + (q[2] - origin[2]) ** 2)
                if not (1.4 <= d3 <= 15.0 and -0.35 <= q[2] <= 1.0):
                    continue
                pts_kept += 1
                z = q[2]
                if z < 0.0:
                    zhist["<0（被layer.min_obstacle_height=0丢）"] += 1
                elif z < 0.3:
                    zhist["0~0.3（入体素）"] += 1
                elif z <= 0.6:
                    zhist["0.3~0.6（入体素）"] += 1
                else:
                    zhist["0.6~1.0（被layer.max=0.6丢）"] += 1
                u = int((q[0] - info.origin.position.x) / info.resolution)
                v = int((q[1] - info.origin.position.y) / info.resolution)
                if 0 <= u < info.width and 0 <= v < info.height:
                    hits[(u, v)] = hits.get((u, v), 0) + 1
                    dists_kept.append(d3)
                    if 0.0 <= z <= 0.6:
                        pts_layer += 1
                        hits_layer[(u, v)] = hits_layer.get((u, v), 0) + 1
                        frame_hits[(u, v)] = frame_hits.get((u, v), 0) + 1
            per_frame.append({c for c, n in frame_hits.items() if n >= 2})
        exp1 = set(hits)
        exp2 = {c for c, n in hits.items() if n >= 2}
        exp2_layer = {c for c, n in hits_layer.items() if n >= 2}
        perframe_union = set().union(*per_frame) if per_frame else set()
        lethal, costset = set(), set()
        for idx, val in enumerate(g.data):
            if val >= 99:
                lethal.add((idx % info.width, idx // info.width))
            elif val > 0:
                costset.add((idx % info.width, idx // info.width))
        miss = exp2 - lethal
        stale = lethal - exp2

        def _cell_center(c):
            return (info.origin.position.x + (c[0] + 0.5) * info.resolution,
                    info.origin.position.y + (c[1] + 0.5) * info.resolution)

        miss_d = sorted(
            math.hypot(_cell_center(c)[0] - origin[0], _cell_center(c)[1] - origin[1])
            for c in miss)

        def _pair(c):
            x, y = _cell_center(c)
            return (round(x, 1), round(y, 1))

        out["cloud"] = {"frames_used": len(clouds[-8:]), "points": pts_used,
                        "points_after_obs_buffer": pts_kept,
                        "points_after_layer_z_window(0~0.6)": pts_layer,
                        "per_frame_ge2_cells": sorted(len(s) for s in per_frame),
                        "z_hist_of_obs_filtered": zhist}
        out["cells"] = {
            "expected_ge1_point": len(exp1),
            "expected_ge2_points(mark_threshold=2)": len(exp2),
            "expected_ge2_AND_layer_z_window": len(exp2_layer),
            "per_frame_ge2_union(每帧≥2点_并集)": len(perframe_union),
            "perframe_union_AND_lethal(命中)": len(perframe_union & lethal),
            "perframe_union_NOT_marked": len(perframe_union - lethal),
            "costmap_lethal_99plus": len(lethal),
            "costmap_cost_1_98(inflation)": len(costset),
            "marked_as_expected": len(exp2 & lethal),
            "expected_but_NOT_marked": len(miss),
            "expected_layer_but_NOT_marked": len(exp2_layer - lethal),
            "lethal_without_cloud_support(stale)": len(stale),
        }
        if miss_d:
            out["expected_but_missing_distance_m"] = {
                "min": round(miss_d[0], 2), "median": round(miss_d[len(miss_d) // 2], 2),
                "max": round(miss_d[-1], 2)}
        out["expected_but_missing_cells_odom"] = [_pair(c) for c in sorted(miss)[:15]]
        out["stale_lethal_cells_odom"] = [_pair(c) for c in sorted(stale)[:10]]
        if exp2:
            out["expected_bbox_odom"] = [
                [round(min(c[0] for c in exp2) * info.resolution + info.origin.position.x, 1),
                 round(max(c[0] for c in exp2) * info.resolution + info.origin.position.x, 1)],
                [round(min(c[1] for c in exp2) * info.resolution + info.origin.position.y, 1),
                 round(max(c[1] for c in exp2) * info.resolution + info.origin.position.y, 1)]]
        if lethal:
            out["lethal_bbox_odom"] = [
                [round(min(c[0] for c in lethal) * info.resolution + info.origin.position.x, 1),
                 round(max(c[0] for c in lethal) * info.resolution + info.origin.position.x, 1)],
                [round(min(c[1] for c in lethal) * info.resolution + info.origin.position.y, 1),
                 round(max(c[1] for c in lethal) * info.resolution + info.origin.position.y, 1)]]
        if costset:
            out["cost_bbox_odom"] = [
                [round(min(c[0] for c in costset) * info.resolution + info.origin.position.x, 1),
                 round(max(c[0] for c in costset) * info.resolution + info.origin.position.x, 1)],
                [round(min(c[1] for c in costset) * info.resolution + info.origin.position.y, 1),
                 round(max(c[1] for c in costset) * info.resolution + info.origin.position.y, 1)]]

        # 船体系 1 m 俯视图：'#'=代价图致命、'o'=点云该标却没标、'+'=只有膨胀代价
        T_bl_odom = _mk_chain(edges, "odom", "base_link")
        if T_bl_odom:
            n = 24
            half = 12.0
            grid = [["." for _ in range(n)] for _ in range(n)]

            def _put(cells, ch, only_if_empty=False):
                Tb = _invert(T_bl_odom)
                for c in cells:
                    x, y = _cell_center(c)
                    bx, by, _ = _apply(Tb, (x, y, 0.0))
                    col = int((half - by) / 1.0)
                    row = int((half - bx) / 1.0)
                    if 0 <= row < n and 0 <= col < n:
                        cur = grid[row][col]
                        if only_if_empty and cur != ".":
                            continue
                        if ch == "#" or cur == "." or cur == "+":
                            grid[row][col] = ch

            _put(costset, "+")
            _put(lethal, "#")
            _put(miss, "o", only_if_empty=False)
            out["俯视_船体系(上=船头,左=左舷,#=代价图致命,o=点云该标未标,+=膨胀)"] = [
                "".join(r) for r in grid]
    emit(out)


def cmd_watch(node, seconds: float, period: float = 0.5) -> None:
    """流式监控终点对准行为：位置误差 / 朝向误差 / 实际速度 / 指令速度 / 前瞻点。

    只订阅，不发任何指令。每 period 秒打一行 `@@T@@ ...`，结束后打一行 `@@RESULT@@ {...}`。
    """
    import rclpy
    from std_msgs.msg import String
    from nav_msgs.msg import Path
    from geometry_msgs.msg import Twist, PointStamped

    store: dict = {"status": None, "status_wall": 0.0, "goal": None, "goal_n": 0,
                   "cmd": None, "carrot": None, "selector": None}

    qos_status = _qos()
    try:
        _sw, _sp, _cm, _mw, NavStatus = _import_m_common()
        node.create_subscription(
            NavStatus, DEFAULT_STATUS_TOPIC,
            lambda m: store.update(status=_status_to_dict(m), status_wall=time.time()), qos_status)
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(f"typed /nav_status unavailable: {exc}\n")

    def _on_string(m):
        if store.get("status") is None:
            try:
                store.update(status=json.loads(m.data), status_wall=time.time())
            except Exception:  # noqa: BLE001
                pass

    node.create_subscription(String, DEFAULT_STATUS_TOPIC, _on_string, qos_status)

    def _plan_cb(msg):
        if msg.poses:
            p = msg.poses[-1].pose
            store["goal"] = (p.position.x, p.position.y, math.degrees(_quat_to_yaw(p.orientation)))
            store["goal_n"] = len(msg.poses)

    node.create_subscription(Path, "/plan", _plan_cb, 10)
    node.create_subscription(Twist, "/cmd_vel_nav",
                             lambda m: store.update(cmd=(m.linear.x, m.angular.z)), 10)
    node.create_subscription(PointStamped, "/lookahead_point",
                             lambda m: store.update(carrot=(m.point.x, m.point.y)), 10)
    node.create_subscription(String, "/goal_checker_selector",
                             lambda m: store.update(selector=m.data), 10)

    t0 = time.time()
    last_print = 0.0
    lines = 0
    sys.stdout.write("@@T@@ t      state/phase            wp    d_goal d_mb   d_yaw    pos              yaw     v      w      | cmd(v,w)    | loc\n")
    sys.stdout.flush()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now - last_print < period:
            continue
        last_print = now
        st = store.get("status") or {}
        goal = store.get("goal")
        pos = (st.get("pose_x"), st.get("pose_y"))
        yaw_rad = st.get("pose_yaw")
        dgoal = mbd = dyaw = None
        if goal and pos[0] is not None:
            dgoal = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        if st.get("distance_to_goal_m") is not None:
            mbd = float(st["distance_to_goal_m"])
        if goal and yaw_rad is not None:
            dyaw = _ang_diff_deg(goal[2], yaw_rad)
        cmd = store.get("cmd") or (None, None)
        carrot = store.get("carrot")
        dcar = None
        if carrot and pos[0] is not None:
            dcar = math.hypot(carrot[0] - pos[0], carrot[1] - pos[1])
        fmt = lambda v, w, p=2: ("-" if v is None else f"{v:+.{p}f}")
        sys.stdout.write(
            "@@T@@ {t:>5.1f}s {state:<10}/{phase:<10} {wp:<5} "
            "{dg:>6} {dm:>6} {dy:>7}  ({px:>7},{py:>7}) {yw:>7} {v:>6} {w:>6} | "
            "{cv:>6},{cw:>6} | {loc}{fix}{car}\n".format(
                t=now - t0,
                state=str(st.get("task_state", "?")),
                phase=str(st.get("nav_phase", "?")),
                wp=f"{st.get('current_waypoint', '?')}/{st.get('total_waypoints', '?')}",
                dg=("-" if dgoal is None else f"{dgoal:.2f}m"),
                dm=("-" if mbd is None else f"{mbd:.2f}m"),
                dy=("-" if dyaw is None else f"{dyaw:+.1f}°"),
                px="-" if pos[0] is None else f"{pos[0]:.2f}",
                py="-" if pos[1] is None else f"{pos[1]:.2f}",
                yw="-" if yaw_rad is None else f"{math.degrees(yaw_rad):+.1f}°",
                v=fmt(st.get("pose_v"), 2, 2),
                w=fmt(st.get("pose_w"), 2, 2),
                cv=fmt(cmd[0], 2, 2),
                cw=fmt(cmd[1], 2, 2),
                loc=str(st.get("localization_overall", "?")),
                fix=f" fix{st.get('localization_gps_fix', '?')}",
                car=("" if dcar is None else f" carrot={dcar:.1f}m"),
            ))
        sys.stdout.flush()
        lines += 1

    st = store.get("status") or {}
    emit({
        "ok": True, "cmd": "watch", "seconds": seconds, "lines": lines,
        "goal_last_plan": store.get("goal"), "plan_poses": store.get("goal_n"),
        "goal_checker_selector": store.get("selector"),
        "final": {"task_state": st.get("task_state"), "nav_phase": st.get("nav_phase"),
                  "current_waypoint": st.get("current_waypoint"),
                  "total_waypoints": st.get("total_waypoints"),
                  "distance_to_goal_m": st.get("distance_to_goal_m"),
                  "pose_x": st.get("pose_x"), "pose_y": st.get("pose_y"),
                  "pose_yaw": st.get("pose_yaw"), "pose_v": st.get("pose_v"),
                  "pose_w": st.get("pose_w"), "last_error": st.get("last_error"),
                  "localization_overall": st.get("localization_overall")},
    })


def cmd_pause(node, flag: str) -> None:
    import rclpy

    want = str(flag).strip() not in ("0", "false", "False", "")
    try:
        _SW, SetPause, _CM, _MW, _NS = _import_m_common()
    except Exception as exc:  # noqa: BLE001
        emit({"ok": False, "cmd": "pause", "error": f"导入 m_common 失败: {exc}"}, 2)
    cl = _client_wait(node, SetPause, "/mission_bridge/set_pause", 3.0)
    if cl is None:
        emit({"ok": False, "cmd": "pause", "error": "服务不可用: /mission_bridge/set_pause"}, 3)
    req = SetPause.Request()
    req.pause = want
    future = cl.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
    res = future.result()
    if res is None:
        emit({"ok": False, "cmd": "pause", "error": "set_pause 超时"}, 4)
    emit({"ok": bool(res.success), "cmd": "pause", "pause": want,
          "success": bool(res.success), "message": str(res.message)},
         0 if res.success else 5)


def main() -> None:
    argv = sys.argv[1:]
    if not argv:
        emit({"ok": False, "error": "用法: python3 - <check|status|dispatch|cancel|pause> [...]"}, 64)
    mode = argv[0]

    try:
        import rclpy
    except Exception as exc:  # noqa: BLE001
        emit({"ok": False, "cmd": mode, "error": f"import rclpy 失败（ROS 环境没 source）: {exc}"}, 70)

    rclpy.init(args=None)
    node = rclpy.create_node("map_tool_dispatch_cli")
    try:
        if mode == "check":
            cmd_check(node)
        elif mode == "status":
            cmd_status(node)
        elif mode == "datum":
            cmd_datum()
        elif mode == "dispatch":
            if len(argv) < 3:
                emit({"ok": False, "error": "dispatch 需要 <service|topic> <b64json>"}, 64)
            cmd_dispatch(node, argv[1], argv[2])
        elif mode == "cancel":
            cmd_cancel(node)
        elif mode == "pause":
            cmd_pause(node, argv[1] if len(argv) > 1 else "1")
        elif mode == "watch":
            cmd_watch(node,
                      float(argv[1]) if len(argv) > 1 else 30.0,
                      float(argv[2]) if len(argv) > 2 else 0.5)
        elif mode == "diag":
            cmd_diag(node, float(argv[1]) if len(argv) > 1 else 8.0)
        elif mode == "params":
            cmd_params(node,
                       argv[1] if len(argv) > 1 else "/local_costmap/local_costmap",
                       argv[2] if len(argv) > 2 else "",
                       argv[3] if len(argv) > 3 else "")
        elif mode == "count":
            cmd_count(node,
                      argv[1] if len(argv) > 1 else "/local_costmap/costmap",
                      float(argv[2]) if len(argv) > 2 else 4.0)
        elif mode == "compare":
            cmd_compare(node,
                        float(argv[1]) if len(argv) > 1 else 4.0,
                        argv[2] if len(argv) > 2 else "/local_costmap/costmap")
        elif mode == "voxelcheck":
            cmd_voxelcheck(node, float(argv[1]) if len(argv) > 1 else 5.0,
                           argv[2] if len(argv) > 2 else "/local_costmap/voxel_marked_cloud")
        elif mode == "voxelmap":
            cmd_voxelmap(node, float(argv[1]) if len(argv) > 1 else 4.0,
                         argv[2] if len(argv) > 2 else "/local_costmap/voxel_grid")
        elif mode == "tfcheck":
            cmd_tfcheck(node, float(argv[1]) if len(argv) > 1 else 6.0)
        else:
            emit({"ok": False, "error": f"未知模式: {mode}"}, 64)
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        emit({"ok": False, "cmd": mode, "error": f"{type(exc).__name__}: {exc}"}, 1)
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
