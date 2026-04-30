#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  地面站 /waypoint（经纬度）→ waypoints.json 中 Nav2 map 坐标 x,y（geometry_msgs / map 帧）。
#
#  与纯「相对 datum 的 UTM 米」不同：必须对齐 map_server 的 map.yaml：
#    - ref_gnss_*：地图角点经纬度（与 navsat.yaml datum 同源），表示该角在地图中的物理含义；
#    - origin: [ox, oy, yaw]：单元格 (0,0) 在 map 世界系中的位姿（米、弧度），Nav2 目标点应落在该系下。
#
#  转换步骤：经纬度差 → 以 datum 为原点的局部 ENU（东、北，米，与 robot_localization navsat 小范围平面一致）
#          → 按 origin.yaw 旋转并加 origin 平移，得到 map 世界 (x, y)。
#
#  datum 默认 map.yaml ref（与 navsat 固定 datum 一致）；datum_source:=first_gps 可恢复旧行为。
# ----------------------------------------------------------------------------------------------- #

import json
import os
import tempfile
from pathlib import Path
from typing import Optional, Tuple

import math
import utm
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

GREEN = '\x1b[32m'
RESET = '\x1b[0m'
TARGET_FILENAME = "waypoints.json"

def find_workspace_root() -> Optional[Path]:
    try:
        script_path = Path(__file__).resolve()
    except Exception:
        script_path = Path.cwd().resolve()
    candidates = [script_path, Path.cwd().resolve()]
    seen = set()
    for start in candidates:
        for p in [start] + list(start.parents):
            if p in seen:
                continue
            seen.add(p)
            if (p / "src" / "YILDIZ-USV" / "workspace_nav").is_dir():
                return p
            if (p / "YILDIZ-USV" / "workspace_nav").is_dir():
                return p
    return None

def make_output_paths() -> Tuple[Path, Path]:
    env_path = os.environ.get("WAYPOINT_OUTPUT_PATH")
    if env_path:
        p = Path(env_path).resolve()
        return p.parent, p
    try:
        base = get_package_share_directory("workspace_nav")
        candidate = Path(base) / "json" / TARGET_FILENAME
        if candidate.parent.exists():
            return candidate.parent.resolve(), candidate.resolve()
    except Exception:
        pass
    ws_root = find_workspace_root()
    if ws_root is not None:
        candidate1 = (ws_root / "src" / "YILDIZ-USV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
        if candidate1.parent.exists():
            return candidate1.parent, candidate1
        candidate2 = (ws_root / "YILDIZ-USV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
        if candidate2.parent.exists():
            return candidate2.parent, candidate2
        candidate3_dir = (ws_root / "src" / "YILDIZ-USV" / "workspace_nav" / "json").resolve()
        return candidate3_dir, (candidate3_dir / TARGET_FILENAME).resolve()
    cwd_candidate = (Path.cwd().resolve() / "src" / "YILDIZ-USV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
    alt_cwd = (Path.cwd().resolve() / "YILDIZ-USV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
    if cwd_candidate.parent.exists():
        return cwd_candidate.parent, cwd_candidate
    if alt_cwd.parent.exists():
        return alt_cwd.parent, alt_cwd
    fallback_dir = cwd_candidate.parent
    return fallback_dir, cwd_candidate

OUTPUT_DIR, OUTPUT_PATH = make_output_paths()


def _read_map_ref_lon_lat(cfg: dict, ref_key: str) -> Tuple[float, float]:
    """yaml 中 ref 列表为 [longitude, latitude]，返回 (latitude, longitude)。"""
    if ref_key not in cfg:
        raise ValueError(f"map.yaml 中未找到 {ref_key}")
    arr = cfg[ref_key]
    lon = float(arr[0])
    lat = float(arr[1])
    return lat, lon


def _datum_lat_lon_from_cfg(cfg: dict, ref_key: str) -> Tuple[float, float]:
    if ref_key and ref_key.strip() and ref_key in cfg:
        return _read_map_ref_lon_lat(cfg, ref_key)
    for key in ("ref_gnss_10", "ref_gnss"):
        if key not in cfg:
            continue
        arr = cfg[key]
        lon = float(arr[0])
        lat = float(arr[1])
        return lat, lon
    raise ValueError("map.yaml 中未找到 ref_gnss_10 / ref_gnss 或指定 datum_ref_key")


def _read_map_origin(cfg: dict) -> Tuple[float, float, float]:
    """map_server 风格 origin: [ox, oy, yaw_rad]。缺省为 (0,0,0)。"""
    origin = cfg.get("origin")
    if not origin:
        return 0.0, 0.0, 0.0
    if isinstance(origin, (list, tuple)):
        ox = float(origin[0]) if len(origin) > 0 else 0.0
        oy = float(origin[1]) if len(origin) > 1 else 0.0
        oyaw = float(origin[2]) if len(origin) > 2 else 0.0
        return ox, oy, oyaw
    return 0.0, 0.0, 0.0


def _geodetic_delta_enu_m(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """以 (lat0,lon0) 为切点的局部 ENU：x 东、y 北（米）。与 navsat 小范围平面逼近一致。"""
    r_earth = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    east = r_earth * math.cos(math.radians(lat0)) * dlon
    north = r_earth * dlat
    return east, north


def _enu_delta_to_map_xy(
    east: float, north: float, ox: float, oy: float, origin_yaw: float
) -> Tuple[float, float]:
    """ENU 米偏移（相对与地图 ref 重合的角点）→ map 世界坐标（与 map_server/costmap 公式一致）。"""
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    mx = ox + east * c - north * s
    my = oy + east * s + north * c
    return mx, my


class GPSToFile(Node):
    def __init__(self):
        super().__init__('waypoint_transform')
        self.datum_lat: Optional[float] = None
        self.datum_lon: Optional[float] = None
        self.datum_easting: Optional[float] = None
        self.datum_northing: Optional[float] = None
        self.waypoints: Optional[list] = None
        self.waypoint_received = False
        self.is_done = False

        self.declare_parameter('datum_source', 'map_yaml')
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('gps_topic', '/gps/filtered')
        self.declare_parameter('map_datum_ref_key', 'ref_gnss_10')
        self.declare_parameter('projection', 'enu')

        source = self.get_parameter('datum_source').get_parameter_value().string_value
        map_yaml_param = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        ref_key = self.get_parameter('map_datum_ref_key').get_parameter_value().string_value
        projection = self.get_parameter('projection').get_parameter_value().string_value.strip().lower()

        self._datum_source = source
        self._map_yaml_resolved = ''
        self._datum_ref_key = (ref_key or 'ref_gnss_10').strip() or 'ref_gnss_10'
        self._projection = projection if projection in ('enu', 'utm') else 'enu'
        self._map_ox = 0.0
        self._map_oy = 0.0
        self._map_origin_yaw = 0.0

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        if source == 'map_yaml':
            if map_yaml_param.strip():
                map_path = Path(map_yaml_param).expanduser().resolve()
            else:
                try:
                    share = Path(get_package_share_directory('workspace_nav'))
                    map_path = (share / 'config' / 'map.yaml').resolve()
                except Exception as e:
                    self.get_logger().fatal(f'无法解析 map.yaml 路径: {e}')
                    raise SystemExit(1) from e
            if not map_path.is_file():
                self.get_logger().fatal(f'map.yaml 不存在: {map_path}')
                raise SystemExit(1)
            try:
                with map_path.open("r", encoding="utf-8") as f:
                    cfg = yaml.safe_load(f)
                lat, lon = _datum_lat_lon_from_cfg(cfg, self._datum_ref_key)
                self._map_ox, self._map_oy, self._map_origin_yaw = _read_map_origin(cfg)
            except Exception as e:
                self.get_logger().fatal(f'读取地图 datum/origin 失败: {e}')
                raise SystemExit(1) from e
            easting, northing, _, _ = utm.from_latlon(lat, lon)
            self.datum_lat = lat
            self.datum_lon = lon
            self.datum_easting = easting
            self.datum_northing = northing
            self._log_info_green(
                f'map_yaml datum ref={self._datum_ref_key}: lat={lat}, lon={lon} | '
                f'map origin=({self._map_ox}, {self._map_oy}, yaw={self._map_origin_yaw:.6f} rad) | '
                f'projection={self._projection} | file={map_path}'
            )
            self._map_yaml_resolved = str(map_path)
        elif source == 'first_gps':
            self.gps_sub = self.create_subscription(
                NavSatFix, gps_topic, self.gps_callback, qos_profile=qos
            )
            self.get_logger().info(f'datum_source=first_gps，等待 {gps_topic} 首帧作为 datum')
        else:
            self.get_logger().fatal(f'未知 datum_source: {source}，请使用 map_yaml 或 first_gps')
            raise SystemExit(1)

        self.waypoint_sub = self.create_subscription(String, '/waypoint', self.waypoint_callback, 10)
        self.shutdown_timer = self.create_timer(1.0, self.check_shutdown)
        self.get_logger().info('GPS-to-file node 已启动；等待航点消息 /waypoint')

    def _log_info_green(self, text: str):
        self.get_logger().info(f"{GREEN}{text}{RESET}")

    def check_shutdown(self):
        if self.is_done:
            self._log_info_green('Processing complete. Stopping timer and initiating shutdown.')
            try:
                self.destroy_timer(self.shutdown_timer)
            except Exception:
                pass
            rclpy.shutdown()

    def waypoint_callback(self, msg: String):
        if self.waypoint_received:
            return
        try:
            data_str = str(msg.data).strip()
            if data_str.startswith('data: '):
                data_str = data_str[6:].strip()
            json_data = json.loads(data_str)
            waypoints_list = json_data.get('waypoints', [])
            parsed = []
            for wp in waypoints_list:
                lat = float(wp['latitude'])
                lon = float(wp['longitude'])
                parsed.append((lat, lon))
            self.waypoints = parsed
            self._log_info_green(f'Received {len(self.waypoints)} waypoint(s).')
            self.waypoint_received = True
            self.try_conversion()
        except Exception as e:
            self.get_logger().error(f'Failed to parse waypoint message: {e}')

    def gps_callback(self, msg: NavSatFix):
        if self.datum_lat is not None and self.datum_lon is not None:
            return
        try:
            datum_lat = float(msg.latitude)
            datum_lon = float(msg.longitude)
            easting, northing, _, _ = utm.from_latlon(datum_lat, datum_lon)
            self.datum_lat = datum_lat
            self.datum_lon = datum_lon
            self.datum_easting = easting
            self.datum_northing = northing
            self._log_info_green(f'Datum acquired: lat={datum_lat}, lon={datum_lon}')
            self.try_conversion()
        except Exception as e:
            self.get_logger().error(f'Error processing GPS message: {e}')

    def try_conversion(self):
        if self.datum_easting is None or self.datum_northing is None or self.waypoints is None:
            return
        try:
            OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
            output = {
                "waypoints": [],
                "datum": {
                    "latitude": float(self.datum_lat),
                    "longitude": float(self.datum_lon)
                },
                "datum_source_meta": {
                    "datum_source": self._datum_source,
                    "map_yaml": self._map_yaml_resolved or None,
                    "map_datum_ref_key": self._datum_ref_key if self._datum_source == 'map_yaml' else None,
                    "projection": self._projection,
                },
                "map_frame_meta": {
                    "frame_id": "map",
                    "description": (
                        "x,y 为 Nav2 map 坐标（米）。datum_source=map_yaml 时已应用 map.yaml 的 origin 平移与旋转；"
                        "first_gps 时为相对首帧 GPS 的平面坐标，不一定与 map 一致。"
                    ),
                    "origin_x": float(self._map_ox),
                    "origin_y": float(self._map_oy),
                    "origin_yaw_rad": float(self._map_origin_yaw),
                    "applied_origin_transform": bool(self._datum_source == 'map_yaml'),
                },
            }
            for lat, lon in self.waypoints:
                if self._projection == 'utm':
                    easting, northing, _, _ = utm.from_latlon(float(lat), float(lon))
                    east = float(easting - self.datum_easting)
                    north = float(northing - self.datum_northing)
                else:
                    east, north = _geodetic_delta_enu_m(
                        float(self.datum_lat),
                        float(self.datum_lon),
                        float(lat),
                        float(lon),
                    )
                if self._datum_source == 'map_yaml':
                    x, y = _enu_delta_to_map_xy(
                        east, north, self._map_ox, self._map_oy, self._map_origin_yaw
                    )
                else:
                    x, y = east, north
                output["waypoints"].append({
                    "latitude": float(lat),
                    "longitude": float(lon),
                    "x": round(x, 4),
                    "y": round(y, 4)
                })
            fd = None
            tmp_path = None
            try:
                fd, tmp_path = tempfile.mkstemp(dir=str(OUTPUT_DIR), prefix='waypoints_', suffix='.tmp')
                with os.fdopen(fd, 'w') as tf:
                    json.dump(output, tf, indent=2)
                    tf.flush()
                    os.fsync(tf.fileno())
                if OUTPUT_PATH.exists():
                    try:
                        OUTPUT_PATH.unlink()
                    except Exception:
                        pass
                os.replace(tmp_path, str(OUTPUT_PATH))
                tmp_path = None
            finally:
                try:
                    if tmp_path and os.path.exists(tmp_path):
                        os.remove(tmp_path)
                except Exception:
                    pass
            try:
                with open(OUTPUT_PATH, 'r') as vf:
                    loaded = json.load(vf)
                if not isinstance(loaded, dict) or 'waypoints' not in loaded or 'datum' not in loaded:
                    self.get_logger().error('Verification failed: written file content is invalid or missing required keys.')
                    return
                if not isinstance(loaded.get('waypoints'), list) or not isinstance(loaded.get('datum'), dict):
                    self.get_logger().error('Verification failed: invalid types in written file.')
                    return
            except Exception as e:
                self.get_logger().error(f'Failed to verify written file: {e}')
                return
            self._log_info_green(f'Waypoints written to {OUTPUT_PATH} ({len(self.waypoints)} entries).')
            self.is_done = True
        except Exception as e:
            self.get_logger().error(f'Conversion failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSToFile()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
