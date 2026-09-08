#!/usr/bin/env bash
# 归港数据流测试编排：假环境 + mock mission_bridge + usv_docking + dock_mission，
# 从上层接口 /gcs_dock/command 下发 one_click_dock，录制关键话题（输出日志在 /tmp/dock_test/out/）。
set +u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u
export ROS_DOMAIN_ID=42
ros2 daemon stop > /dev/null 2>&1 || true   # 清掉可能残留的跨 domain/已死 daemon

OUT=/tmp/dock_test/out
rm -rf "$OUT" && mkdir -p "$OUT"

pids=()
kill_nodes() {
  # launch 子节点不会因父进程退出而退出，必须按模式补杀（[x] 字符类防自匹配）
  pkill -f "docking_fs[m]" 2>/dev/null
  pkill -f "docking_pose_estimato[r]" 2>/dev/null
  pkill -f "docking_motion_controlle[r]" 2>/dev/null
  pkill -f "docking_safet[y]" 2>/dev/null
  pkill -f "dock_mission_nod[e]" 2>/dev/null
  pkill -f "dock_entry_validato[r]" 2>/dev/null
  pkill -f "fake_en[v]" 2>/dev/null
  pkill -f "mock_mission_bridg[e]" 2>/dev/null
  pkill -f "mock_event_serve[r]" 2>/dev/null
}
cleanup() { kill "${pids[@]}" 2>/dev/null; sleep 1; kill_nodes; wait 2>/dev/null; }
trap cleanup EXIT

kill_nodes   # 防上次运行残留
sleep 1

echo "== 启动 fake_env / mock_mission_bridge =="
python3 "${SCRIPT_DIR}/fake_env.py" > "$OUT/fake_env.log" 2>&1 & pids+=($!)
python3 "${SCRIPT_DIR}/mock_mission_bridge.py" > "$OUT/mock_bridge.log" 2>&1 & pids+=($!)
python3 "${SCRIPT_DIR}/mock_event_server.py" > "$OUT/mock_event_server.log" 2>&1 & pids+=($!)
sleep 2

echo "== 启动 usv_docking（test_only:=false → /cmd_vel_nav）=="
ros2 launch usv_docking docking.launch.py test_only:=false \
  > "$OUT/usv_docking.log" 2>&1 & pids+=($!)

echo "== 启动 dock_mission（profile=real）=="
ros2 launch dock_mission dock_mission.launch.py \
  > "$OUT/dock_mission.log" 2>&1 & pids+=($!)
sleep 5

echo "== 录制关键话题 =="
timeout 110 ros2 topic echo /dock_task/event_log > "$OUT/dock_task_event.log" 2>&1 & pids+=($!)
timeout 110 ros2 topic echo /docking/state > "$OUT/v2_state.log" 2>&1 & pids+=($!)
timeout 110 ros2 topic echo /dock/status > "$OUT/dock_status.log" 2>&1 & pids+=($!)
timeout 110 ros2 topic echo /docking/target_mode > "$OUT/v2_target_mode.log" 2>&1 & pids+=($!)

sleep 2
echo "== 上层下发 one_click_dock（/gcs_dock/command JSON）=="
ros2 topic pub --once /gcs_dock/command std_msgs/msg/String \
  "{data: '{\"action\": \"one_click_dock\", \"mission_id\": \"dock_staging\", \"command_id\": \"e2e-001\"}'}" \
  > /dev/null 2>&1

echo "== 运行 100s 观察全链路 =="
sleep 100

echo "== 采样 /cmd_vel_nav 与 /dock_task/status =="
timeout 5 ros2 topic echo --once /dock_task/status > "$OUT/dock_task_status_last.log" 2>&1
timeout 5 ros2 topic echo --once /cmd_vel_nav > "$OUT/cmd_vel_nav_last.log" 2>&1

echo "== DONE =="
