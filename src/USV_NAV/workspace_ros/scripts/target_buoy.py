#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that subscribes to color-code messages on '/color_code' and atomically persists the
#  selected target color to `target_buoy.json`. It locates the workspace root to resolve the
#  JSON file path, performs a safe write using a temporary file with fsync and atomic replace,
#  verifies the integrity of the written content, logs detailed waiting/success/error states,
#  and gracefully unsubscribes after a confirmed successful write.
# ----------------------------------------------------------------------------------------------- #

import json
import os
import time
import tempfile
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

GREEN = '\x1b[32m'
RESET = '\x1b[0m'

VALID_TARGETS = {"green", "red", "black"}
COLOR_TOPIC = '/color_code'
TARGET_JSON_FILENAME = "target_buoy.json"

def find_workspace_root() -> Optional[Path]:
    try:
        script_path = Path(__file__).resolve()
    except Exception:
        script_path = Path.cwd().resolve()
    candidates = [script_path, Path.cwd().resolve()]
    checked = set()
    for start in candidates:
        for p in [start] + list(start.parents):
            if p in checked:
                continue
            checked.add(p)
            if (p / 'src' / 'USV_NAV' / 'workspace_nav').is_dir():
                return p
            if (p / 'USV_NAV' / 'workspace_nav').is_dir():
                return p
    return None

def make_target_paths() -> Tuple[Path, Path]:
    env_path = os.environ.get("TARGET_JSON_PATH")
    if env_path:
        p = Path(env_path).resolve()
        return p.parent, p
    try:
        base = get_package_share_directory("workspace_nav")
        candidate = Path(base) / "json" / TARGET_JSON_FILENAME
        if candidate.is_file():
            return candidate.parent.resolve(), candidate.resolve()
    except Exception:
        pass
    ws_root = find_workspace_root()
    if ws_root is not None:
        candidate1 = (ws_root / "src" / "USV_NAV" / "workspace_nav" / "json" / TARGET_JSON_FILENAME).resolve()
        if candidate1.is_file():
            return candidate1.parent, candidate1
        candidate2 = (ws_root / "USV_NAV" / "workspace_nav" / "json" / TARGET_JSON_FILENAME).resolve()
        if candidate2.is_file():
            return candidate2.parent, candidate2
        candidate3_dir = (ws_root / "src" / "USV_NAV" / "workspace_nav" / "json").resolve()
        return candidate3_dir, (candidate3_dir / TARGET_JSON_FILENAME).resolve()
    fallback = (Path.cwd().resolve() / "src" / "USV_NAV" / "workspace_nav" / "json" / TARGET_JSON_FILENAME).resolve()
    alt = (Path.cwd().resolve() / "USV_NAV" / "workspace_nav" / "json" / TARGET_JSON_FILENAME).resolve()
    if fallback.is_file():
        return fallback.parent, fallback
    if alt.is_file():
        return alt.parent, alt
    return fallback.parent, fallback

TARGET_DIR, TARGET_PATH = make_target_paths()

class TargetBuoyNode(Node):
    def __init__(self):
        super().__init__('target_buoy')
        qos = QoSProfile(depth=10)
        self.color_sub = self.create_subscription(String, COLOR_TOPIC, self.camera_info_callback, qos)
        self.camera_info_received = False
        self.pending_msg_logged = False
        self.pending_timer = self.create_timer(1.0, self.pending_timer_callback)
        self.get_logger().info(f'TargetBuoy node initialized; awaiting color code messages on {COLOR_TOPIC}.')

    def _log_info_green(self, text: str):
        self.get_logger().info(f"{GREEN}{text}{RESET}")

    def pending_timer_callback(self):
        if not self.camera_info_received and not self.pending_msg_logged:
            self.get_logger().info(f'No color code received yet; awaiting messages on {COLOR_TOPIC}.')
            self.pending_msg_logged = True
        if self.pending_msg_logged or self.camera_info_received:
            try:
                self.pending_timer.cancel()
            except Exception:
                pass

    def camera_info_callback(self, msg: String):
        if self.camera_info_received:
            return
        try:
            raw = str(msg.data)
        except Exception:
            raw = ""
        normalized = raw.strip().lower()
        self.get_logger().info(f"Received {COLOR_TOPIC}: '{raw}' -> normalized: '{normalized}'")
        if normalized in VALID_TARGETS:
            try:
                TARGET_DIR.mkdir(parents=True, exist_ok=True)
                fd = None
                tmp_path = None
                try:
                    fd, tmp_path = tempfile.mkstemp(dir=str(TARGET_DIR), prefix='target_buoy_', suffix='.tmp')
                    with os.fdopen(fd, 'w') as tf:
                        target_data = {
                            "target": {
                                "color": normalized,
                                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
                            }
                        }
                        json.dump(target_data, tf, indent=2)
                        tf.flush()
                        os.fsync(tf.fileno())
                    os.replace(tmp_path, str(TARGET_PATH))
                    tmp_path = None
                finally:
                    try:
                        if tmp_path and os.path.exists(tmp_path):
                            os.remove(tmp_path)
                    except Exception:
                        pass
                try:
                    with open(TARGET_PATH, 'r') as vf:
                        loaded = json.load(vf)
                    if not isinstance(loaded, dict) or 'target' not in loaded or loaded['target'].get('color') != normalized:
                        self.get_logger().error('Verification failed: written file content is invalid or does not match expected target.')
                        return
                except Exception as e:
                    self.get_logger().error(f'Failed to verify written file: {e}')
                    return
            except Exception as e:
                self.get_logger().error(f'Failed to write target_buoy.json: {e}')
                return
            self._log_info_green(f"Target written to {TARGET_PATH}: {normalized}")
            try:
                if self.color_sub is not None:
                    self.destroy_subscription(self.color_sub)
            except Exception:
                pass
            self.color_sub = None
            self.camera_info_received = True
            try:
                self.pending_timer.cancel()
            except Exception:
                pass
        else:
            self.get_logger().debug(f"Ignored invalid color code: '{raw}'")

def main(args=None):
    rclpy.init(args=args)
    node = TargetBuoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested by user; exiting.')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()