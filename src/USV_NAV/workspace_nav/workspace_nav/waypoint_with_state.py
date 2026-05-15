#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that monitors a waypoint JSON file, loads and validates waypoint lists, and dispatches
#  each waypoint to the Nav2 FollowWaypoints action server. It observes odometry to skip waypoints
#  within tolerance, handles action responses and results, clears the waypoint file on completion,
#  and may launch a post-navigation "kamikaze" script as a subprocess while ensuring robust process-
#  group termination and cleanup. The node resolves the waypoint file path from environment,
#  package share or workspace locations, logs detailed state transitions and errors, and supports a
#  graceful shutdown sequence that terminates any spawned child processes.
# ----------------------------------------------------------------------------------------------- #

import os
import math
import json
import rclpy
import threading
import subprocess
import sys
import signal
from pathlib import Path
from typing import Optional, List, Tuple

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

GREEN = '\x1b[32m'
RESET = '\x1b[0m'
WAYPOINT_FILENAME = "waypoints.json"

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
            if (p / "src" / "USV_NAV").is_dir():
                return p
            if (p / "USV_NAV").is_dir():
                return p
    return None

def make_waypoint_path() -> Path:
    env_path = os.environ.get("WAYPOINT_FILE_PATH")
    if env_path:
        return Path(env_path).expanduser().resolve()
    try:
        base = get_package_share_directory("workspace_nav")
        candidate = Path(base) / "json" / WAYPOINT_FILENAME
        if candidate.exists():
            return candidate.resolve()
    except Exception:
        pass
    ws_root = find_workspace_root()
    if ws_root is not None:
        candidate1 = (ws_root / "src" / "USV_NAV" / "workspace_nav" / "json" / WAYPOINT_FILENAME).resolve()
        if candidate1.exists():
            return candidate1
        candidate2 = (ws_root / "USV_NAV" / "workspace_nav" / "json" / WAYPOINT_FILENAME).resolve()
        if candidate2.exists():
            return candidate2
        candidate3 = (ws_root / "src" / "USV_NAV" / "workspace_nav" / "json" / WAYPOINT_FILENAME).resolve()
        return candidate3
    home_candidate = Path.home() / "usv_nav_ws" / "src" / "USV_NAV" / "workspace_nav" / "json" / WAYPOINT_FILENAME
    if home_candidate.exists():
        return home_candidate.resolve()
    default = Path.cwd().resolve() / "src" / "USV_NAV" / "workspace_nav" / "json" / WAYPOINT_FILENAME
    return default

def find_kamikaze_script() -> Optional[Path]:
    env_path = os.environ.get("KAMIKAZE_SCRIPT_PATH")
    if env_path:
        p = Path(env_path).expanduser().resolve()
        if p.exists():
            return p
    try:
        base = get_package_share_directory("workspace_ros")
        candidate = Path(base) / "scripts" / "kamikaze.py"
        if candidate.exists():
            return candidate.resolve()
    except Exception:
        pass
    ws_root = find_workspace_root()
    if ws_root is not None:
        candidate1 = (ws_root / "src" / "USV_NAV" / "workspace_ros" / "scripts" / "kamikaze.py").resolve()
        if candidate1.exists():
            return candidate1
        candidate2 = (ws_root / "USV_NAV" / "workspace_ros" / "scripts" / "kamikaze.py").resolve()
        if candidate2.exists():
            return candidate2
    return None

class SimpleWaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_with_state')
        self.tolerance = 1.5
        self.waypoint_file: Path = make_waypoint_path()
        self.get_logger().info(f'Using waypoint file: {self.waypoint_file}')
        self.waypoints: List[Tuple[float, float, float]] = []
        self.waypoints_loaded = False
        self.file_check_timer = self.create_timer(1.0, self.check_waypoint_file)
        self._file_state = None
        self.current_index = 0
        self.navigating = False
        self.kamikaze_script_executed = False
        self.kamikaze_process = None
        self._current_pose = (0.0, 0.0)
        self._pose_lock = threading.Lock()
        self.waypoint_follower = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._send_timer = None

    def _log_info_green(self, text: str):
        self.get_logger().info(f"{GREEN}{text}{RESET}")

    def check_waypoint_file(self):
        if self.waypoints_loaded:
            try:
                self.destroy_timer(self.file_check_timer)
            except Exception:
                pass
            return
        if not self.waypoint_file.exists():
            if self._file_state != 'missing':
                self.get_logger().info('Waypoint file not found; awaiting creation.')
                self._file_state = 'missing'
            return
        try:
            size = self.waypoint_file.stat().st_size
        except Exception:
            size = 0
        if size == 0:
            if self._file_state != 'empty':
                self.get_logger().warning('Waypoint file present but empty; awaiting data.')
                self._file_state = 'empty'
            return
        loaded_waypoints = self.load_waypoints_from_file(self.waypoint_file)
        if loaded_waypoints:
            self.waypoints = loaded_waypoints
            if self._file_state != 'loaded':
                self._log_info_green(f'Loaded {len(self.waypoints)} waypoint(s). Starting navigation.')
                self._file_state = 'loaded'
            self.waypoints_loaded = True
            try:
                self.destroy_timer(self.file_check_timer)
            except Exception:
                pass
            self.start_navigation()
        else:
            if self._file_state != 'invalid':
                self.get_logger().warning('Failed to load waypoints; will retry.')
                self._file_state = 'invalid'

    def start_navigation(self):
        self._current_pose = (0.0, 0.0)
        self._pose_lock = threading.Lock()
        self._odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self._odom_callback, 10)
        self._send_timer = self.create_timer(2.0, self.send_next_waypoint)

    def load_waypoints_from_file(self, path: Path) -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = []
        try:
            with path.open('r') as f:
                json_data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Error reading waypoint file: {e}')
            return []
        candidate_list = None
        if isinstance(json_data, dict):
            if 'waypoints' in json_data and isinstance(json_data['waypoints'], (list, tuple)):
                candidate_list = json_data['waypoints']
            else:
                for v in json_data.values():
                    if isinstance(v, (list, tuple)):
                        candidate_list = v
                        break
                if candidate_list is None:
                    candidate_list = [json_data]
        elif isinstance(json_data, (list, tuple)):
            candidate_list = json_data
        else:
            return []
        for idx, wp in enumerate(candidate_list):
            try:
                if isinstance(wp, (list, tuple)) and len(wp) >= 2:
                    x = float(wp[0])
                    y = float(wp[1])
                    points.append((x, y, 0.0))
                elif isinstance(wp, dict):
                    if 'x' in wp and 'y' in wp:
                        x = float(wp['x'])
                        y = float(wp['y'])
                        points.append((x, y, 0.0))
                    else:
                        keys = list(wp.keys())
                        numeric_vals = []
                        for k in keys:
                            try:
                                numeric_vals.append(float(wp[k]))
                            except Exception:
                                pass
                        if len(numeric_vals) >= 2:
                            points.append((float(numeric_vals[0]), float(numeric_vals[1]), 0.0))
                        else:
                            self.get_logger().warning(f'Waypoint #{idx+1} in file ignored: missing x/y')
                else:
                    self.get_logger().warning(f'Waypoint #{idx+1} in file ignored: unsupported format')
            except (ValueError, TypeError) as e:
                self.get_logger().warning(f'Waypoint #{idx+1} in file ignored due to parse error: {e}')
                continue
        if not points:
            return []
        return points

    def _odom_callback(self, msg: Odometry):
        with self._pose_lock:
            self._current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def get_robot_position(self) -> Tuple[float, float]:
        with self._pose_lock:
            return self._current_pose

    def create_pose(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def send_next_waypoint(self):
        if self.navigating or self.current_index >= len(self.waypoints):
            return
        x, y, _ = self.waypoints[self.current_index]
        rx, ry = self.get_robot_position()
        dist = math.hypot(x - rx, y - ry)
        if dist <= self.tolerance:
            self.get_logger().info(f'Waypoint {self.current_index + 1} within tolerance; skipping.')
            self.current_index += 1
            try:
                if self._send_timer:
                    self._send_timer.cancel()
            except Exception:
                pass
            self._send_timer = self.create_timer(0.5, self.send_next_waypoint)
            return
        goal_poses = [self.create_pose(x, y, 0.0, 0.0)]
        goal = FollowWaypoints.Goal()
        goal.poses = goal_poses
        self._log_info_green(f'Sending waypoint {self.current_index + 1}/{len(self.waypoints)}: x={x:.2f}, y={y:.2f}')
        self.navigating = True
        try:
            if not self.waypoint_follower.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('FollowWaypoints action server not available.')
                self.navigating = False
                self._send_timer = self.create_timer(2.0, self.send_next_waypoint)
                return
        except Exception:
            pass
        send_goal_future = self.waypoint_follower.send_goal_async(goal)
        send_goal_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception:
            self.get_logger().error('Failed to get goal handle from future.')
            self.navigating = False
            self._send_timer = self.create_timer(2.0, self.send_next_waypoint)
            return
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal was rejected by the action server.')
            self.navigating = False
            self._send_timer = self.create_timer(2.0, self.send_next_waypoint)
            return
        self.get_logger().info('Waypoint goal accepted by action server.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        try:
            result = future.result()
        except Exception:
            self.get_logger().error('Failed to get goal result from future.')
            self.navigating = False
            self._send_timer = self.create_timer(2.0, self.send_next_waypoint)
            return
        status = result.status
        self.navigating = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._log_info_green(f'Waypoint {self.current_index + 1} reached successfully.')
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self._log_info_green('All waypoints completed. Initiating post-navigation sequence.')
                self.execute_kamikaze_script()
                return
        self._send_timer = self.create_timer(1.0, self.send_next_waypoint)

    def clear_waypoint_file(self):
        try:
            self.waypoint_file.parent.mkdir(parents=True, exist_ok=True)
            with self.waypoint_file.open('w') as f:
                f.write("{}")
            self.get_logger().info(f'Waypoint file cleared: {self.waypoint_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to clear waypoint file: {e}')

    def _terminate_process(self, proc: Optional[subprocess.Popen]):
        if proc is None:
            return
        try:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    except Exception:
                        pass
                try:
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except Exception:
                        pass
        except Exception:
            pass
        finally:
            try:
                if proc.poll() is None:
                    proc.kill()
            except Exception:
                pass

    def execute_kamikaze_script(self):
        if self.kamikaze_script_executed:
            return
        kamikaze_path = find_kamikaze_script()
        kamikaze_started = False
        try:
            if kamikaze_path and kamikaze_path.exists():
                self.get_logger().info('Starting kamikaze script.')
                self.kamikaze_process = subprocess.Popen(
                    [sys.executable, str(kamikaze_path)],
                    preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                kamikaze_started = True
            else:
                self.get_logger().error(f'kamikaze script not found at: {kamikaze_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to start kamikaze script: {e}')
            self.kamikaze_process = None
        self.kamikaze_script_executed = kamikaze_started
        if self.kamikaze_script_executed:
            self.clear_waypoint_file()
            self._log_info_green('Post-navigation kamikaze script launched. Navigator will remain running; press Ctrl+C to terminate and clean up child processes.')
        else:
            self.get_logger().error('No post-navigation scripts could be launched.')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Waypoint navigator interrupted by user.')
    finally:
        try:
            node.get_logger().info('Terminating child processes if any.')
        except Exception:
            pass
        try:
            node._terminate_process(node.kamikaze_process)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass

if __name__ == '__main__':
    main()