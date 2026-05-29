#!/usr/bin/env bash
set +e
set +u
source /opt/ros/humble/setup.bash
source /home/ght/wuxihik_navigation/install/setup.bash

WS=/home/ght/wuxihik_navigation
MAP="${WS}/src/YILDIZ-USV/workspace_nav/config/map.yaml"
LOG=/tmp/nav_svc_test.log
: > "${LOG}"

log() { echo "$@" | tee -a "${LOG}"; }

pkill -f "mission_bridge|nav_status_aggregator|mock_follow_waypoints|fake_odom_pub|static_transform_publisher_Wxf" 2>/dev/null
sleep 2

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link >>"${LOG}" 2>&1 &
python3 "${WS}/test/mock_follow_waypoints_server.py" >>"${LOG}" 2>&1 &
python3 "${WS}/test/fake_odom_pub.py" >>"${LOG}" 2>&1 &
sleep 1
ros2 run workspace_nav mission_bridge --ros-args -p map_yaml_path:="${MAP}" -p odom_topic:=/odometry/filtered >>"${LOG}" 2>&1 &
ros2 run workspace_nav nav_status_aggregator --ros-args -p odom_topic:=/odometry/filtered >>"${LOG}" 2>&1 &
sleep 8

log "=== SERVICES ==="
ros2 service list 2>>"${LOG}" | grep mission_bridge | tee -a "${LOG}"

log "=== WAIT IDLE ==="
for i in $(seq 1 20); do
  S=$(timeout 1 ros2 topic echo /mission_bridge/state --once 2>/dev/null | grep "data:" | sed 's/data: //;s/"//g' | tr -d ' ')
  log "try $i state=$S"
  [ "$S" = "IDLE" ] && break
  sleep 0.5
done

log "=== send_waypoints ==="
ros2 service call /mission_bridge/send_waypoints m_common/srv/SendWaypoints \
  "{waypoints: [{header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 15.0, y: 15.0, z: 0.0}, orientation: {w: 1.0}}}], mission_id: 't1', command_id: 'c1'}" 2>&1 | tee -a "${LOG}"
sleep 2
timeout 2 ros2 topic echo /mission_bridge/state --once 2>&1 | tee -a "${LOG}"

log "=== pause ==="
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: true}" 2>&1 | tee -a "${LOG}"
sleep 1
timeout 2 ros2 topic echo /mission_bridge/state --once 2>&1 | tee -a "${LOG}"

log "=== resume ==="
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: false}" 2>&1 | tee -a "${LOG}"
sleep 1
timeout 2 ros2 topic echo /mission_bridge/state --once 2>&1 | tee -a "${LOG}"

log "=== emergency ==="
ros2 service call /mission_bridge/emergency_stop m_common/srv/EmergencyStop "{}" 2>&1 | tee -a "${LOG}"
sleep 1
timeout 2 ros2 topic echo /mission_bridge/state --once 2>&1 | tee -a "${LOG}"

log "=== cancel ==="
ros2 service call /mission_bridge/cancel_mission m_common/srv/CancelMission "{}" 2>&1 | tee -a "${LOG}"
sleep 1
timeout 2 ros2 topic echo /mission_bridge/state --once 2>&1 | tee -a "${LOG}"

log "=== nav_status once ==="
timeout 3 ros2 topic echo /nav_status --once 2>&1 | head -3 | tee -a "${LOG}"

log "=== DONE ==="
pkill -f "mission_bridge|nav_status_aggregator|mock_follow|fake_odom|static_transform" 2>/dev/null
