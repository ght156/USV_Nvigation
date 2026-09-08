#!/usr/bin/env bash
# 精确归港（usv_docking）独立数据流测试：/dock/start 直触发，录制 /cmd_vel_nav。
set +u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u
export ROS_DOMAIN_ID=43
ros2 daemon stop > /dev/null 2>&1 || true

OUT=/tmp/dock_test/out2
rm -rf "$OUT" && mkdir -p "$OUT"
pids=()
cleanup() {
  kill "${pids[@]}" 2>/dev/null
  sleep 1
  pkill -f "docking_fs[m]" 2>/dev/null
  pkill -f "docking_pose_estimato[r]" 2>/dev/null
  pkill -f "docking_motion_controlle[r]" 2>/dev/null
  pkill -f "docking_safet[y]" 2>/dev/null
  pkill -f "fake_en[v]" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

python3 "${SCRIPT_DIR}/fake_env.py" > "$OUT/fake_env.log" 2>&1 & pids+=($!)
ros2 launch usv_docking docking.launch.py test_only:=false \
  > "$OUT/usv_docking.log" 2>&1 & pids+=($!)
sleep 6

echo "== 上层直触发 /dock/start（仅精靠泊）=="
ros2 topic pub --once /dock/start std_msgs/msg/Bool "{data: true}" > /dev/null 2>&1

echo "== 在 t=25~33s（BACK_IN 段）录制 /cmd_vel_nav =="
sleep 17
timeout 8 ros2 topic echo /cmd_vel_nav > "$OUT/cmd_vel_nav_backin.log" 2>&1 & pids+=($!)
sleep 10

echo "== 采样 FSM 状态与 /dock/status =="
timeout 4 ros2 topic echo --once /docking/state > "$OUT/v2_state.log" 2>&1
timeout 4 ros2 topic echo --once /docking/pose_source > "$OUT/pose_source.log" 2>&1
timeout 4 ros2 topic echo --once /docking/dock_pose > "$OUT/dock_pose.log" 2>&1
echo "== DONE =="
