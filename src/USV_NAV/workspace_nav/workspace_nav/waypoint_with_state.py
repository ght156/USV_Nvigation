#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that monitors a waypoint JSON file, loads and validates waypoint lists, and dispatches
#  each waypoint to the Nav2 FollowWaypoints action server. It observes odometry to skip waypoints
#  within tolerance, handles action responses and results, and clears the waypoint file on completion.
#  The node resolves the waypoint file path from environment, package share or workspace locations.
# ----------------------------------------------------------------------------------------------- #

import os
import math
import json
import rclpy
import threading
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

        # odom topic parameter
        self.declare_parameter(
            'odom_topic',
            '/mavros/local_position/odom'
        )
        odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.get_logger().info(
            f'Using odom topic: {odom_topic}'
        )

        self._odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10
        )

        self._send_timer = self.create_timer(
            2.0,
            self.send_next_waypoint
        )

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
                self.clear_waypoint_file()
                self.get_logger().info(
                    'All waypoints completed.'
                )
                self.get_logger().info(
                    'Holding position.'
                )
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

def main(args=None):
    rclpy.init(args=args)
    node = SimpleWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Waypoint navigator interrupted by user.')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

if __name__ == '__main__':
    main()