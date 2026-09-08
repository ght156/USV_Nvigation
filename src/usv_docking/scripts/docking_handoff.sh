#!/usr/bin/env bash
# Nav2 ↔ usv_docking 交接脚本（仿真 / 实船通用）
set -euo pipefail

ACTION="${1:-start}"

deactivate_nav2_controller() {
  echo "[docking_handoff] deactivate controller_server"
  ros2 lifecycle set /controller_server deactivate
}

activate_nav2_controller() {
  echo "[docking_handoff] activate controller_server"
  ros2 lifecycle set /controller_server activate
}

start_docking() {
  deactivate_nav2_controller
  echo "[docking_handoff] publish /dock/start"
  ros2 topic pub /dock/start std_msgs/msg/Bool "{data: true}" --once
}

cancel_docking() {
  echo "[docking_handoff] publish /dock/cancel"
  ros2 topic pub /dock/cancel std_msgs/msg/Empty "{}" --once
  activate_nav2_controller
}

case "${ACTION}" in
  start)
    start_docking
    ;;
  cancel)
    cancel_docking
    ;;
  deactivate)
    deactivate_nav2_controller
    ;;
  activate)
    activate_nav2_controller
    ;;
  *)
    echo "Usage: $0 {start|cancel|deactivate|activate}"
    exit 1
    ;;
esac
