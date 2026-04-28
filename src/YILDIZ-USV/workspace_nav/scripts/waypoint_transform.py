#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that subscribes to GPS fixes and waypoint messages, transforms geographic coordinates
#  into local UTM-relative coordinates using an acquired GPS datum, and atomically persists the
#  resulting waypoints and datum to `waypoints.json`. It resolves the output path from the
#  workspace or environment, validates incoming waypoint payloads, performs a safe write using a
#  temporary file with fsync and atomic replace, verifies the integrity and structure of the
#  written JSON, logs detailed progress and error states, and performs a graceful shutdown after
#  successful completion.
# ----------------------------------------------------------------------------------------------- #

import json
import os
import tempfile
from pathlib import Path
from typing import Optional, Tuple

import utm
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

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.gps_sub = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_callback, qos_profile=qos)
        self.waypoint_sub = self.create_subscription(String, '/waypoint', self.waypoint_callback, 10)
        self.shutdown_timer = self.create_timer(1.0, self.check_shutdown)
        self.get_logger().info('GPS-to-file node initialized; awaiting GPS datum and waypoint input.')

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
                }
            }
            for lat, lon in self.waypoints:
                easting, northing, _, _ = utm.from_latlon(float(lat), float(lon))
                x = easting - self.datum_easting
                y = northing - self.datum_northing
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