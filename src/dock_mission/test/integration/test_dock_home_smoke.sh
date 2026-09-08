#!/usr/bin/env bash
# Optional smoke test: arm dock mission via /dock/home and mock send_waypoints.
# Requires dock_mission_node running separately (or via launch file).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "${WS_ROOT}/install/setup.bash"
  set -u
fi

MOCK_SEND_WAYPOINTS="${SCRIPT_DIR}/mock_send_waypoints.py"
if [ ! -f "${MOCK_SEND_WAYPOINTS}" ]; then
  echo "ERROR: missing ${MOCK_SEND_WAYPOINTS}" >&2
  exit 1
fi

echo "[smoke] Starting mock send_waypoints..."
python3 "${MOCK_SEND_WAYPOINTS}" &
MOCK_PID=$!
trap 'kill ${MOCK_PID} 2>/dev/null || true' EXIT

sleep 1

echo "[smoke] Publishing /dock/home=true..."
ros2 topic pub --once /dock/home std_msgs/msg/Bool "{data: true}"

echo "[smoke] Waiting for /dock/mission_status..."
timeout 10 ros2 topic echo /dock/mission_status std_msgs/msg/String --once || {
  echo "WARN: no mission status within timeout (is dock_mission_node running?)" >&2
  exit 0
}

echo "[smoke] Done."
