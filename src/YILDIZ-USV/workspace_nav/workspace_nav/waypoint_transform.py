#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  地面站 /waypoint（经纬度）→ waypoints.json 中 Nav2 map 坐标 x,y（geometry_msgs / map 帧）。
#
#  与纯「相对 datum 的 UTM 米」不同：必须对齐 map_server 的 map.yaml：
#    - ref_gnss_*：地图角点经纬度（与 navsat 固定 datum 同源），表示该角在地图中的物理含义；
#    - origin: [ox, oy, yaw]：单元格 (0,0) 在 map 世界系中的位姿（米、弧度），Nav2 目标点应落在该系下。
#
#  转换步骤：经纬度差 → 以 datum 为原点的局部 ENU（东、北，米）
#          → 按 origin.yaw 旋转并加 origin 平移，得到 map 世界 (x, y)。
#
#  datum 默认 map_yaml；datum_source:=first_gps 兼容旧仿真行为。
#  载荷格式与 mission_bridge / USV_NAV 一致：顶层 list 或 {"waypoints":[...]}，点为对象或 [lat,lon]。
#
#  坐标逻辑见 workspace_nav.gps_map_conversion（与 mission_bridge 共用）。
# ----------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from pathlib import Path
from typing import Optional

import utm
import yaml
from ament_index_python.packages import get_package_share_directory

from workspace_nav.gps_map_conversion import (
    atomic_write_json,
    datum_lat_lon_from_cfg,
    lat_lon_list_to_waypoints_document,
    parse_waypoint_payload,
    read_map_origin,
    verify_waypoints_file,
)

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
            if (p / "src" / "USV_NAV" / "workspace_nav").is_dir():
                return p
            if (p / "USV_NAV" / "workspace_nav").is_dir():
                return p
            if (p / "src" / "YILDIZ-USV" / "workspace_nav").is_dir():
                return p
            if (p / "YILDIZ-USV" / "workspace_nav").is_dir():
                return p
    return None


def make_output_paths() -> tuple:
    import os
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
        candidate1 = (ws_root / "src" / "USV_NAV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
        if candidate1.parent.exists():
            return candidate1.parent, candidate1
        candidate2 = (ws_root / "USV_NAV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
        if candidate2.parent.exists():
            return candidate2.parent, candidate2
        candidate_y1 = (ws_root / "src" / "YILDIZ-USV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
        if candidate_y1.parent.exists():
            return candidate_y1.parent, candidate_y1
        candidate_y2 = (ws_root / "YILDIZ-USV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
        if candidate_y2.parent.exists():
            return candidate_y2.parent, candidate_y2
        candidate3_dir = (ws_root / "src" / "USV_NAV" / "workspace_nav" / "json").resolve()
        if candidate3_dir.exists():
            return candidate3_dir, (candidate3_dir / TARGET_FILENAME).resolve()
        candidate_y_dir = (ws_root / "src" / "YILDIZ-USV" / "workspace_nav" / "json").resolve()
        return candidate_y_dir, (candidate_y_dir / TARGET_FILENAME).resolve()
    cwd_candidate = (Path.cwd().resolve() / "src" / "USV_NAV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
    alt_cwd = (Path.cwd().resolve() / "USV_NAV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
    cwd_y = (Path.cwd().resolve() / "src" / "YILDIZ-USV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
    alt_y = (Path.cwd().resolve() / "YILDIZ-USV" / "workspace_nav" / "json" / TARGET_FILENAME).resolve()
    if cwd_candidate.parent.exists():
        return cwd_candidate.parent, cwd_candidate
    if alt_cwd.parent.exists():
        return alt_cwd.parent, alt_cwd
    if cwd_y.parent.exists():
        return cwd_y.parent, cwd_y
    if alt_y.parent.exists():
        return alt_y.parent, alt_y
    fallback_dir = cwd_y.parent
    return fallback_dir, cwd_y


OUTPUT_DIR, OUTPUT_PATH = make_output_paths()


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
        self.declare_parameter('gps_topic', '/roboboat/sensors/gps/navsat')
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
        if self._projection == 'utm':
            self.get_logger().warning(
                'projection=utm is not recommended for Nav2 map-frame waypoint conversion. '
                'Use projection=enu unless you know the map datum and UTM zone are consistent.'
            )
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
                    map_path = (share / 'config' / 'map_hk.yaml').resolve()
                except Exception as e:
                    self.get_logger().fatal(f'无法解析地图 yaml 路径: {e}')
                    raise SystemExit(1) from e
            if not map_path.is_file():
                self.get_logger().fatal(f'map yaml 不存在: {map_path}')
                raise SystemExit(1)
            try:
                with map_path.open("r", encoding="utf-8") as f:
                    cfg = yaml.safe_load(f)
                lat, lon = datum_lat_lon_from_cfg(cfg, self._datum_ref_key)
                self._map_ox, self._map_oy, self._map_origin_yaw = read_map_origin(cfg)
            except Exception as e:
                self.get_logger().fatal(f'读取地图 datum/origin 失败: {e}')
                raise SystemExit(1) from e
            easting, northing, _, _ = utm.from_latlon(lat, lon)
            self.datum_lat = lat
            self.datum_lon = lon
            self.datum_easting = easting
            self.datum_northing = northing
            self._log_info_green(
                f'Using datum from map_yaml — '
                f'ref={self._datum_ref_key}: lat={lat}, lon={lon} | '
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
        parsed = parse_waypoint_payload(msg.data)
        if not parsed:
            self.get_logger().error(
                'Invalid waypoint message (expect list / {"waypoints":[...]}, lat/lon objects or tuples)'
            )
            return

        self.waypoints = parsed
        self._log_info_green(f'Received {len(self.waypoints)} waypoint(s)')
        self.get_logger().info(f'Using datum from {self._datum_source}')
        self.get_logger().info(f'Using projection {self._projection}')
        self.waypoint_received = True
        self.try_conversion()

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
            output = lat_lon_list_to_waypoints_document(
                self.waypoints,
                float(self.datum_lat),
                float(self.datum_lon),
                float(self.datum_easting),
                float(self.datum_northing),
                self._projection,
                self._datum_source,
                self._map_yaml_resolved or '',
                self._datum_ref_key,
                self._map_ox,
                self._map_oy,
                self._map_origin_yaw,
            )
            atomic_write_json(OUTPUT_DIR, OUTPUT_PATH, output)

            if not verify_waypoints_file(OUTPUT_PATH):
                self.get_logger().error(
                    'Verification failed: written file content is invalid or missing required keys.'
                )
                return
            self._log_info_green(f'Waypoints written to {OUTPUT_PATH}')
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
