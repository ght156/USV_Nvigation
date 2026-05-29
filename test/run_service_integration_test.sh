#!/usr/bin/env bash
# Integration test: upper-layer services against mission_bridge + aggregator.
set -eo pipefail

WS=/home/ght/wuxihik_navigation
MAP_YAML="${WS}/src/YILDIZ-USV/workspace_nav/config/map.yaml"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

cleanup() {
  jobs -p | xargs -r kill 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "==> static TF map -> base_link"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &
sleep 1

echo "==> mock FollowWaypoints action server"
python3 "${WS}/test/mock_follow_waypoints_server.py" &
sleep 1

echo "==> fake odometry /odometry/filtered"
python3 "${WS}/test/fake_odom_pub.py" &
sleep 0.5

echo "==> mission_bridge + nav_status_aggregator"
ros2 run workspace_nav mission_bridge --ros-args \
  -p map_yaml_path:="${MAP_YAML}" \
  -p odom_topic:=/odometry/filtered &
MB_PID=$!
ros2 run workspace_nav nav_status_aggregator --ros-args \
  -p odom_topic:=/odometry/filtered &
sleep 3

echo "==> ros2 service list (mission_bridge)"
ros2 service list | grep mission_bridge || true

echo "==> full service test suite"
python3 "${WS}/test/test_nav_services.py" --case all
EXIT=$?

exit "${EXIT}"
