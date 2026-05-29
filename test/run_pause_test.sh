#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ght/wuxihik_navigation/install/setup.bash
WS=/home/ght/wuxihik_navigation
MAP="${WS}/src/YILDIZ-USV/workspace_nav/config/map.yaml"

pkill -f "mission_bridge|nav_status_aggregator|mock_follow|fake_odom|static_transform" 2>/dev/null
sleep 2

# Slow mock: 15s per goal so pause can be tested
cat > /tmp/slow_mock_fw.py <<'PY'
import time, rclpy
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionServer
from rclpy.node import Node

class S(Node):
    def __init__(self):
        super().__init__("slow_mock_fw")
        self._s = ActionServer(self, FollowWaypoints, "follow_waypoints", self._e)
    def _e(self, gh):
        self.get_logger().info("goal accepted — hold 8s")
        time.sleep(8)
        gh.succeed()
        return FollowWaypoints.Result()
rclpy.init(); n=S(); rclpy.spin(n)
PY

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &
python3 /tmp/slow_mock_fw.py &
python3 "${WS}/test/fake_odom_pub.py" &
sleep 1
ros2 run workspace_nav mission_bridge --ros-args -p map_yaml_path:="${MAP}" &
ros2 run workspace_nav nav_status_aggregator &
sleep 6

echo "=== send (2 wp) ==="
ros2 service call /mission_bridge/send_waypoints m_common/srv/SendWaypoints \
  "{waypoints: [{header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 20.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}], mission_id: 'pause_test', command_id: ''}" 2>&1 | grep -E "success|message"
sleep 2
ros2 topic echo /mission_bridge/state --once 2>/dev/null | grep data

echo "=== pause ==="
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: true}" 2>&1 | grep -E "success|message"
sleep 1
ros2 topic echo /mission_bridge/state --once 2>/dev/null | grep data

echo "=== resume ==="
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: false}" 2>&1 | grep -E "success|message"
sleep 1
ros2 topic echo /mission_bridge/state --once 2>/dev/null | grep data

pkill -f "mission_bridge|nav_status|slow_mock|fake_odom|static_transform" 2>/dev/null
