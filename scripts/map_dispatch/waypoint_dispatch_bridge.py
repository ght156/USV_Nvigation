#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图工具 → 船端航点下发桥（地面站本机运行，纯标准库）。

作用：把 ros_map_tool 航点编辑器里选中的 **map 坐标航点** 转成 WGS84 经纬度，
再通过 `ssh <船端别名>` 把 `remote_nav_cli.py` 喂到船端执行，最终调用
`mission_bridge/send_waypoints`（默认，带朝向、立即执行）或发 `/waypoint` 话题。

为什么走 ssh 而不是 rosbridge：
  * 船端（兴飞 NX）没装 rosbridge_server；
  * 地面站只在浏览器里跑网页、不装 ROS，ssh 是唯一既通又可复现的通路；
  * 地面站与船端跨网段/4G 时，DDS 发现不通，ssh 只需 TCP 22。

坐标换算与船端 `workspace_nav/gps_map_conversion.py` 严格互逆（ENU + map.yaml origin）：
    east  = R * cos(lat0) * dlon_rad
    north = R * dlat_rad
    mx = ox + east*cos(yaw0) - north*sin(yaw0)
    my = oy + east*sin(yaw0) + north*cos(yaw0)
反解即本文件 `map_xy_to_latlon()`。datum 取 map.yaml 的 ref_gnss_10（默认），
与船端 mission_bridge 的 `map_datum_ref_key` 默认值一致。

启动（地面站）：
    python3 scripts/map_dispatch/waypoint_dispatch_bridge.py            # 127.0.0.1:8770
    python3 scripts/map_dispatch/waypoint_dispatch_bridge.py --host 0.0.0.0
网页里「航点编辑器 → 📤 下发到船端」填 http://127.0.0.1:8770 即可；
也可直接把本服务当静态页服务器用：浏览器打开 http://127.0.0.1:8770/

命令行自检（不用浏览器）：
    ... --check            # ssh 到船端跑 remote_nav_cli.py check（只读，不动船）
    ... --status           # 读一次 /nav_status
    ... --cancel / --pause 1
    ... --dispatch-file payload.json --dry-run   # 只做坐标换算，不发船端
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
REMOTE_CLI = HERE / "remote_nav_cli.py"
MARK = "@@RESULT@@"
R_EARTH = 6378137.0
DEFAULT_HOST = "hkzl"

# 2026-10-16：船端 RMW 默认从 CycloneDDS 改成 FastDDS。
#   兴飞（hkzl）自 2026-10-15 起全船走 rmw_fastrtps_cpp。客户端若还落 CycloneDDS，
#   会出现「发现层/话题照旧正常（check 里服务全 ✅、/nav_status 也读得到），
#   但**服务请求-应答静默超时**」——当天实测 A/B：老默认值下
#   /mission_bridge/get_geofence 无响应，改 rmw_fastrtps_cpp 立即正常返回；
#   页面上表现为「下发失败：服务 /mission_bridge/send_waypoints 超时（8s）无响应」。
#   所以判断客户端中间件是否一致，不能只看 check 的服务列表。
#   仍用 CycloneDDS 的船/栈（如澄丰 nx）：--rmw rmw_cyclonedds_cpp，或请求体带 "rmw"。
DEFAULT_RMW = "rmw_fastrtps_cpp"
# 命令行 --rmw 覆盖它；请求体里的 rmw 优先级最高
ACTIVE_RMW = DEFAULT_RMW
# 船端要 source 的工作区，从左到右试第一个存在的（换船不用改参数）
DEFAULT_SETUPS = (
    "~/usv-main/install/arm/setup.bash",   # 兴飞 hkzl
    "~/usv_nav_ws/install/setup.bash",     # 澄丰 nx
    "~/usv/install/arm/setup.bash",        # 船端 usv 工作区（GI320/RS485 那份）
)
DEFAULT_SETUP = DEFAULT_SETUPS[0]
DEFAULT_HTML = Path("/home/ght/ros_map_tool-main/ros_map_tool_v3_nav2_live.html")

_remote_lock = threading.Lock()


# --------------------------------------------------------------------------- #
# 坐标换算
# --------------------------------------------------------------------------- #
def map_xy_to_latlon(x: float, y: float, datum_lat: float, datum_lon: float,
                     ox: float = 0.0, oy: float = 0.0, origin_yaw: float = 0.0):
    """map 系 (x, y) → WGS84 (lat, lon)。与船端 gps_map_conversion 互逆。"""
    dx = float(x) - ox
    dy = float(y) - oy
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    east = dx * c + dy * s
    north = -dx * s + dy * c
    lat = datum_lat + math.degrees(north / R_EARTH)
    lon = datum_lon + math.degrees(east / (R_EARTH * math.cos(math.radians(datum_lat))))
    return lat, lon


def bbox_from_anchors(anchors: dict, margin_m: float = 50.0):
    """用 map.yaml 的 ref_gnss_* 锚点算经纬度包围盒（+margin），做下发前的粗安全校验。"""
    pts = []
    for key, val in (anchors or {}).items():
        try:
            lon, lat = float(val[0]), float(val[1])
        except Exception:  # noqa: BLE001
            continue
        if lat == 0.0 and lon == 0.0:
            continue
        pts.append((lat, lon))
    if len(pts) < 2:
        return None
    dlat = margin_m / R_EARTH * 180.0 / math.pi
    lat0 = sum(p[0] for p in pts) / len(pts)
    dlon = margin_m / (R_EARTH * math.cos(math.radians(lat0))) * 180.0 / math.pi
    return (min(p[0] for p in pts) - dlat, max(p[0] for p in pts) + dlat,
            min(p[1] for p in pts) - dlon, max(p[1] for p in pts) + dlon)


# --------------------------------------------------------------------------- #
# ssh → 船端
# --------------------------------------------------------------------------- #
def build_remote_cmd(remote_setup: str, argv: list, rmw: str = "") -> str:
    setups = ([remote_setup] if remote_setup else []) + [
        s for s in DEFAULT_SETUPS if s != remote_setup
    ]
    src_ws = "for s in " + " ".join(setups) + "; do if [ -f \"$s\" ]; then . \"$s\" >/dev/null 2>&1 && break; fi; done; "
    rmw = (rmw or ACTIVE_RMW or DEFAULT_RMW).strip()
    if not rmw.replace("_", "").isalnum():
        rmw = DEFAULT_RMW
    if rmw == "rmw_cyclonedds_cpp":
        # 这份 env 里是 CYCLONEDDS_URI=<…Interface eth1…>，只有 CycloneDDS 用得上
        rmw_env = "if [ -f ~/logs/xingfei_stack/.cyclone_env ]; then . ~/logs/xingfei_stack/.cyclone_env; fi; "
    else:
        # FastDDS 下留着它只会把中间件带偏
        rmw_env = "unset CYCLONEDDS_URI; "
    return (
        "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-5}; "
        f"export RMW_IMPLEMENTATION=${{RMW_IMPLEMENTATION:-{rmw}}}; "
        "export PYTHONIOENCODING=utf-8; "
        + rmw_env +
        ". /opt/ros/humble/setup.bash >/dev/null 2>&1; "
        + src_ws +
        "exec python3 - " + " ".join(str(a) for a in argv)
    )


def run_remote(host: str, argv: list, timeout: float, remote_setup: str = "",
               rmw: str = "") -> dict:
    """把 remote_nav_cli.py 通过 ssh stdin 送到船端执行，返回解析后的结果 dict。"""
    if not REMOTE_CLI.is_file():
        return {"ok": False, "error": f"缺少船端脚本 {REMOTE_CLI}"}
    script = REMOTE_CLI.read_text(encoding="utf-8")
    cmd = build_remote_cmd(remote_setup, argv, rmw)
    ssh_argv = [
        "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=10", "-o", "ServerAliveCountMax=3",
        host, cmd,
    ]
    t0 = time.time()
    try:
        with _remote_lock:
            proc = subprocess.run(
                ssh_argv, input=script, capture_output=True, text=True, timeout=timeout
            )
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": f"ssh {host} 超时（{timeout:.0f}s）",
                "ssh_cmd": " ".join(ssh_argv[:4]) + f" {host} ..."}
    except FileNotFoundError:
        return {"ok": False, "error": "本机找不到 ssh 命令"}

    elapsed = time.time() - t0
    out = proc.stdout or ""
    result = None
    noise = []
    for line in out.splitlines():
        if line.startswith(MARK):
            try:
                result = json.loads(line[len(MARK):].strip())
            except json.JSONDecodeError as exc:
                return {"ok": False, "error": f"船端返回的 JSON 解析失败: {exc}",
                        "stdout": out[-2000:], "stderr": (proc.stderr or "")[-2000:]}
        elif line.strip():
            noise.append(line)

    payload = {
        "ok": bool(result and result.get("ok")),
        "host": host,
        "elapsed_sec": round(elapsed, 2),
        "ssh_rc": proc.returncode,
        "remote": result,
    }
    if noise:
        payload["remote_noise"] = noise[-8:]
    if proc.returncode != 0 and result is None:
        payload["stderr"] = (proc.stderr or "").strip()[-2000:]
        payload["error"] = payload.get("error") or (
            f"ssh 执行失败（rc={proc.returncode}），船端没打印结果")
    return payload


def fetch_status(host: str, remote_setup: str = "", timeout: float = 40.0,
                 rmw: str = "") -> dict:
    return run_remote(host, ["status"], timeout, remote_setup, rmw)


def do_check(host: str, remote_setup: str = "", timeout: float = 60.0,
             rmw: str = "") -> dict:
    return run_remote(host, ["check"], timeout, remote_setup, rmw)


def watch(host: str, seconds: float, remote_setup: str = "", period: float = 0.5,
          rmw: str = "") -> int:
    """流式监控船端状态（本地打印，便于盯「终点对准」这类过程）。"""
    if not REMOTE_CLI.is_file():
        print(f"缺少船端脚本 {REMOTE_CLI}")
        return 1
    script = REMOTE_CLI.read_text(encoding="utf-8")
    cmd = build_remote_cmd(remote_setup, ["watch", f"{seconds:g}", f"{period:g}"], rmw)
    ssh_argv = [
        "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=10", host, cmd,
    ]
    err_file = Path("/tmp") / f"watch_{os.getpid()}.err"
    with err_file.open("w", encoding="utf-8") as errf:
        proc = subprocess.Popen(
            ssh_argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=errf, text=True, bufsize=1,
        )
        assert proc.stdin and proc.stdout
        try:
            proc.stdin.write(script)
            proc.stdin.close()
        except BrokenPipeError:
            pass
        seen = 0
        for raw in proc.stdout:
            line = raw.rstrip("\n")
            if line.startswith("@@T@@"):
                print(line[len("@@T@@"):].strip(), flush=True)
                seen += 1
            elif line.startswith(MARK):
                try:
                    print("\n[结束] " + json.dumps(json.loads(line[len(MARK):].strip()),
                                                  ensure_ascii=False), flush=True)
                except json.JSONDecodeError:
                    print("\n[结束] " + line[len(MARK):].strip(), flush=True)
            elif line.startswith("==="):
                print(line, flush=True)
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
    if seen <= 1:
        tail = err_file.read_text(encoding="utf-8", errors="replace").strip()[-1500:]
        print("[无监控数据] 船端 stderr：\n" + (tail or "(空)"), flush=True)
    try:
        err_file.unlink()
    except OSError:
        pass
    return 0


# --------------------------------------------------------------------------- #
# 下发核心（HTTP 与 CLI 共用）
# --------------------------------------------------------------------------- #
def convert_waypoints(req: dict, allow_out_of_bounds: bool = False):
    """请求 dict → (latlon 列表, 元信息)；坐标非法直接抛 ValueError。"""
    wps = req.get("waypoints") or []
    if not wps:
        raise ValueError("没有航点（先在『航点编辑器』里打点并勾选）")

    datum = req.get("datum") or {}
    datum_lat = float(datum.get("lat"))
    datum_lon = float(datum.get("lon"))
    if not (math.isfinite(datum_lat) and math.isfinite(datum_lon)) or (datum_lat == 0 and datum_lon == 0):
        raise ValueError("datum（map.yaml ref_gnss_10）未设置，无法把 map 坐标转经纬度")

    origin = req.get("origin") or {}
    ox = float(origin.get("x", 0.0))
    oy = float(origin.get("y", 0.0))
    oyaw = float(origin.get("yaw", 0.0))
    yaw_mode = str(req.get("yaw_mode") or "planned")  # planned | course

    latlon = []
    for i, w in enumerate(wps):
        lat, lon = map_xy_to_latlon(w["x"], w["y"], datum_lat, datum_lon, ox, oy, oyaw)
        yaw_rad = float(w.get("yaw") or 0.0)
        yaw_deg = yaw_mode == "planned" and math.degrees(yaw_rad) or 65536.0
        latlon.append({"lat": round(lat, 8), "lon": round(lon, 8),
                       "yaw": round(float(yaw_deg), 2), "x": float(w["x"]), "y": float(w["y"])})

    bbox = bbox_from_anchors(req.get("anchors") or {}, margin_m=float(req.get("bbox_margin_m", 50.0)))
    if bbox and not allow_out_of_bounds:
        (lat_min, lat_max, lon_min, lon_max) = bbox
        for i, p in enumerate(latlon):
            if not (lat_min <= p["lat"] <= lat_max and lon_min <= p["lon"] <= lon_max):
                raise ValueError(
                    f"航点 {i} 换算出的经纬度 ({p['lat']:.7f}, {p['lon']:.7f}) "
                    f"超出地图锚点范围 —— 多半是底图/原点/datum 不匹配，已拒绝下发")

    return latlon, {"datum": {"lat": datum_lat, "lon": datum_lon},
                    "origin": {"x": ox, "y": oy, "yaw": oyaw},
                    "yaw_mode": yaw_mode, "bbox_checked": bool(bbox) and not allow_out_of_bounds}


def dispatch(req: dict) -> dict:
    """完整下发：换算 → （可选）ssh 船端执行。"""
    host = req.get("host") or DEFAULT_HOST
    mode = str(req.get("mode") or "service")
    remote_setup = req.get("remote_setup") or ""
    rmw = str(req.get("rmw") or "")
    dry_run = bool(req.get("dry_run"))
    allow_obb = bool(req.get("allow_out_of_bounds"))

    latlon, meta = convert_waypoints(req, allow_obb)
    mission_id = str(req.get("mission_id") or "")
    if not mission_id:
        mission_id = "maptool_" + time.strftime("%Y%m%d_%H%M%S")

    payload = {"waypoints": latlon, "mission_id": mission_id}
    if req.get("topic"):
        payload["topic"] = req["topic"]

    out = {"ok": True, "count": len(latlon), "latlon": latlon, "meta": meta,
           "mode": mode, "host": host, "mission_id": mission_id}
    if dry_run:
        out["dry_run"] = True
        out["remote_cmd_preview"] = build_remote_cmd(
            remote_setup, ["dispatch", mode, "<base64 payload>"], rmw)
        return out

    body = base64.b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")
    res = run_remote(host, ["dispatch", mode, body], timeout=70.0,
                     remote_setup=remote_setup, rmw=rmw)
    out["remote"] = res.get("remote")
    out["elapsed_sec"] = res.get("elapsed_sec")
    out["ok"] = bool(res.get("ok"))
    if not out["ok"]:
        out["error"] = res.get("error") or (res.get("remote") or {}).get("error") \
            or (res.get("remote") or {}).get("message") or "船端下发失败"
    return out


# --------------------------------------------------------------------------- #
# HTTP
# --------------------------------------------------------------------------- #
class Handler(BaseHTTPRequestHandler):
    server_version = "USVMapToolDispatch/1.0"
    html_path = DEFAULT_HTML
    default_host = DEFAULT_HOST
    remote_setup = ""
    default_rmw = DEFAULT_RMW

    # ---- utils ----
    def log_message(self, fmt, *args):
        sys.stderr.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), fmt % args))

    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        # file:// 打开的网页请求本机服务时，Chrome 会走 Private Network Access 预检
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, obj: dict):
        self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                   "application/json; charset=utf-8")

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return {}
        raw = self.rfile.read(n)
        try:
            data = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"请求体不是合法 JSON: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError("请求体必须是 JSON 对象")
        return data

    # ---- routes ----
    def do_OPTIONS(self):  # noqa: N802
        self._send(204, b"", "text/plain")

    def do_GET(self):  # noqa: N802
        path = self.path.split("?")[0]
        if path in ("/", "/index.html", "/tool", "/tool.html"):
            p = Path(self.html_path)
            if not p.is_file():
                self._json(404, {"ok": False, "error": f"HTML 不存在: {p}"})
                return
            self._send(200, p.read_bytes(), "text/html; charset=utf-8")
            return
        if path in ("/healthz", "/api/ping"):
            self._json(200, {"ok": True, "service": "usv_map_dispatch",
                             "html": str(self.html_path), "default_host": self.default_host})
            return
        self._json(404, {"ok": False, "error": f"未知路径 {path}"})

    def do_POST(self):  # noqa: N802
        path = self.path.split("?")[0]
        try:
            req = self._body()
        except ValueError as exc:
            self._json(400, {"ok": False, "error": str(exc)})
            return
        host = req.get("host") or self.default_host
        rmw = str(req.get("rmw") or self.default_rmw)
        req.setdefault("remote_setup", self.remote_setup)
        try:
            if path == "/api/check":
                self._json(200, do_check(host, self.remote_setup, rmw=rmw))
            elif path == "/api/status":
                self._json(200, fetch_status(host, self.remote_setup, rmw=rmw))
            elif path == "/api/dispatch":
                self._json(200, dispatch(req))
            elif path == "/api/cancel":
                self._json(200, run_remote(host, ["cancel"], 45.0, self.remote_setup, rmw))
            elif path == "/api/pause":
                flag = "1" if req.get("pause", True) else "0"
                self._json(200, run_remote(host, ["pause", flag], 45.0, self.remote_setup, rmw))
            elif path == "/api/datum":
                self._json(200, run_remote(host, ["datum"], 30.0, self.remote_setup, rmw))
            elif path == "/api/convert":
                latlon, meta = convert_waypoints(req, bool(req.get("allow_out_of_bounds")))
                self._json(200, {"ok": True, "count": len(latlon), "latlon": latlon, "meta": meta})
            else:
                self._json(404, {"ok": False, "error": f"未知接口 {path}"})
        except ValueError as exc:
            self._json(200, {"ok": False, "error": str(exc)})
        except Exception as exc:  # noqa: BLE001
            self._json(500, {"ok": False, "error": f"{type(exc).__name__}: {exc}"})


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description="地图工具航点下发桥（地面站本机）")
    ap.add_argument("--host", default="127.0.0.1", help="HTTP 监听地址（默认 127.0.0.1）")
    ap.add_argument("--port", type=int, default=8770, help="HTTP 端口（默认 8770）")
    ap.add_argument("--ssh-host", default=DEFAULT_HOST, help="船端 ssh 别名/主机（默认 hkzl）")
    ap.add_argument("--remote-setup", default="",
                    help=f"船端要 source 的工作区 setup.bash（默认 {DEFAULT_SETUP}）")
    ap.add_argument("--html", default=str(DEFAULT_HTML), help="可选：顺带托管的工具 HTML 路径")
    ap.add_argument("--check", action="store_true", help="只做船端自检（只读）")
    ap.add_argument("--status", action="store_true", help="只读一次 /nav_status")
    ap.add_argument("--datum", action="store_true",
                    help="只读船端 map.yaml 的 ref_gnss 锚点/datum（只读）")
    ap.add_argument("--cancel", action="store_true", help="取消船端当前任务")
    ap.add_argument("--pause", choices=("1", "0"), help="暂停(1)/继续(0)")
    ap.add_argument("--watch", type=float, metavar="SECONDS",
                    help="流式监控船端 /nav_status + /plan + /cmd_vel_nav（只读）")
    ap.add_argument("--watch-period", type=float, default=0.5, help="监控打印周期（秒，默认 0.5）")
    ap.add_argument("--diag", type=float, nargs="?", const=8.0, metavar="SECONDS",
                    help="雷达落图自检：TF 链 + 点云落图/高度分布 + 代价图障碍统计（只读）")
    ap.add_argument("--count", nargs="?", const="/local_costmap/costmap", metavar="TOPIC",
                    help="统计一张 OccupancyGrid 的格值分布（默认 /local_costmap/costmap）")
    ap.add_argument("--compare", type=float, nargs="?", const=4.0, metavar="SECONDS",
                    help="点云 vs 局部代价图逐格对比（该标没标/多标），只读")
    ap.add_argument("--params", nargs="?", const="/local_costmap/local_costmap", metavar="NODE",
                    help="列出/读取运行中节点的参数（默认局部代价图）")
    ap.add_argument("--voxelcheck", type=float, nargs="?", const=5.0, metavar="SECONDS",
                    help="voxel层内部网格 vs 当前点云：判断代价图标记是新鲜还是旧位置残留")
    ap.add_argument("--voxelmap", type=float, nargs="?", const=4.0, metavar="SECONDS",
                    help="解码 VoxelGrid：每个格子被标了几个 z 体素（=mark_threshold 的比较量）")
    ap.add_argument("--params-prefixes", default="voxel_layer,inflation_layer",
                    help="--params 时列举哪些前缀下的参数（逗号分隔）")
    ap.add_argument("--params-names", default="",
                    help="--params 时要读值的参数名（逗号分隔）")
    ap.add_argument("--dispatch-file", help="按 JSON 文件下发航点（与网页同格式）")
    ap.add_argument("--dry-run", action="store_true", help="只做坐标换算，不发船端")
    ap.add_argument("--rmw", default=DEFAULT_RMW,
                    help=f"船端 DDS 中间件（默认 {DEFAULT_RMW}；"
                         "澄丰等仍跑 CycloneDDS 的船传 rmw_cyclonedds_cpp）")
    args = ap.parse_args()

    global ACTIVE_RMW
    ACTIVE_RMW = args.rmw

    Handler.html_path = args.html
    Handler.default_host = args.ssh_host
    Handler.remote_setup = args.remote_setup
    Handler.default_rmw = args.rmw

    if args.check:
        print(json.dumps(do_check(args.ssh_host, args.remote_setup, rmw=args.rmw),
                         ensure_ascii=False, indent=2))
        return 0
    if args.status:
        print(json.dumps(fetch_status(args.ssh_host, args.remote_setup, rmw=args.rmw),
                         ensure_ascii=False, indent=2))
        return 0
    if args.datum:
        print(json.dumps(run_remote(args.ssh_host, ["datum"], 30.0, args.remote_setup, args.rmw),
                         ensure_ascii=False, indent=2))
        return 0
    if args.cancel:
        print(json.dumps(run_remote(args.ssh_host, ["cancel"], 45.0, args.remote_setup, args.rmw),
                         ensure_ascii=False, indent=2))
        return 0
    if args.pause is not None:
        print(json.dumps(run_remote(args.ssh_host, ["pause", args.pause], 45.0,
                                    args.remote_setup, args.rmw),
                         ensure_ascii=False, indent=2))
        return 0
    if args.watch is not None:
        return watch(args.ssh_host, args.watch, args.remote_setup, args.watch_period, args.rmw)
    if args.diag is not None:
        print(json.dumps(run_remote(args.ssh_host, ["diag", f"{args.diag:g}"], 60.0,
                                    args.remote_setup),
                         ensure_ascii=False, indent=2))
        return 0
    if args.count:
        print(json.dumps(run_remote(args.ssh_host, ["count", args.count], 40.0,
                                    args.remote_setup),
                         ensure_ascii=False, indent=2))
        return 0
    if args.params is not None:
        print(json.dumps(run_remote(args.ssh_host,
                                    ["params", args.params, args.params_prefixes,
                                     args.params_names], 45.0, args.remote_setup),
                         ensure_ascii=False, indent=2))
        return 0
    if args.compare is not None:
        print(json.dumps(run_remote(args.ssh_host, ["compare", f"{args.compare:g}"], 60.0,
                                    args.remote_setup),
                         ensure_ascii=False, indent=2))
        return 0
    if args.voxelcheck is not None:
        print(json.dumps(run_remote(args.ssh_host, ["voxelcheck", f"{args.voxelcheck:g}"],
                                    60.0, args.remote_setup),
                         ensure_ascii=False, indent=2))
        return 0
    if args.voxelmap is not None:
        print(json.dumps(run_remote(args.ssh_host, ["voxelmap", f"{args.voxelmap:g}"],
                                    60.0, args.remote_setup),
                         ensure_ascii=False, indent=2))
        return 0
    if args.dispatch_file:
        req = json.loads(Path(args.dispatch_file).read_text(encoding="utf-8"))
        req.setdefault("host", args.ssh_host)
        req.setdefault("rmw", args.rmw)
        req.setdefault("dry_run", args.dry_run)
        print(json.dumps(dispatch(req), ensure_ascii=False, indent=2))
        return 0

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"下发桥已启动: http://{args.host}:{args.port}/   （船端 ssh: {args.ssh_host}）")
    print(f"网页打开:   http://{args.host}:{args.port}/     或直接把工具页填地址 "
          f"http://{args.host}:{args.port}")
    print("自检:       curl -s http://%s:%d/api/check -H 'Content-Type: application/json' -d '{}'"
          % (args.host, args.port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")
    return 0


if __name__ == "__main__":
    sys.exit(main())
