#!/usr/bin/env bash
# =============================================================================
# start_usv_nav.sh —— 兴飞船 导航 / 定位 / 归港 / 速度桥 / 激光雷达 一键启动
#
# 1) 全部跑在 tmux 后台会话里：SSH 断连、终端关掉都不影响节点继续运行
# 2) start / stop / status / attach / restart 五个动作
#    - stop 会先杀 tmux 会话，再按进程名兜底 TERM→KILL，确保节点真的退出
#      （只匹配本栈的节点名，不动船上别人的进程）
#
# 只启动本工作区 ~/usv_nav_ws 自己的包；激光雷达驱动是公共产物，
# 从 ~/usv-test/install/arm 只读引用（不改动、不复制）。
#
# 前置条件（脚本不负责，需已在跑）：
#   - MAVROS：/mavros/gps_input/local、/mavros/setpoint_velocity/cmd_vel_unstamped
#
# 用法：
#   ./start_usv_nav.sh start      # 依次启动 lidar → localize → nav → bridge → dock → docking
#   ./start_usv_nav.sh lidar      # 单独补起激光雷达（默认 start 不起雷达）
#   ./start_usv_nav.sh attach     # 进入 tmux 查看各窗口日志（Ctrl-b d 退出但不影响运行）
#   ./start_usv_nav.sh status     # 看会话窗口 + 关键进程是否活着
#   ./start_usv_nav.sh stop       # 关会话并杀死本栈所有节点
#   ./start_usv_nav.sh restart
#
# 环境变量：
#   ROS_DOMAIN_ID=5                    与船上其余节点一致
#   RMW：固定 rmw_fastrtps_cpp（FastDDS），外部 RMW_IMPLEMENTATION 会被忽略
#   USV_NAV_SESSION=usv_nav            tmux 会话名
#   START_LIDAR=0                      激光雷达默认不起；需要时 ./start_usv_nav.sh lidar
#   MID360_LAUNCH=msg_MID360_launch.py 激光雷达 launch
#   ODOM_TOPIC=/mavros/gps_input/local 定位源（切 SLAM 时改这里，导航/归港一起变）
#   START_TF=0                         公共包 usv_transforms 的 TF 默认不启动
#                                      （整船 launch 已在跑时不要再开，否则重复发 TF）
#   MAP_TOOL=1                         rosbridge(9090) + /usv_map_pose 船位桥（地图工具实时联动）
#   ROSBRIDGE_PORT=9090                地图工具网页填 ws://<NX_IP>:<该端口>
# =============================================================================
set -uo pipefail

SESSION="${USV_NAV_SESSION:-usv_nav}"
ACTION="${1:-start}"
# 全栈统一 FastDDS（与 MAVROS / 决策侧一致），不再受外部 RMW_IMPLEMENTATION 影响
RMW="rmw_fastrtps_cpp"
if [ -n "${RMW_IMPLEMENTATION:-}" ] && [ "${RMW_IMPLEMENTATION}" != "rmw_fastrtps_cpp" ]; then
  printf '\033[1;33m[start_usv_nav]\033[0m 忽略外部 RMW_IMPLEMENTATION=%s，强制使用 rmw_fastrtps_cpp\n' "$RMW_IMPLEMENTATION"
fi
DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
START_TF="${START_TF:-0}"
MID360_LAUNCH="${MID360_LAUNCH:-msg_MID360s_launch.py}"   # 兴飞船用带 s 的 MID360s
ODOM_TOPIC="${ODOM_TOPIC:-/mavros/gps_input/local}"
MAP_TOOL="${MAP_TOOL:-1}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
START_LIDAR="${START_LIDAR:-0}"

ROS_SETUP=/opt/ros/humble/setup.bash
WS="$HOME/usv_nav_ws"
WS_INSTALL="$WS/install/arm"
WS_SETUP="$WS_INSTALL/setup.bash"
SHARED_INSTALL="$HOME/usv-test/install/arm"      # 别人的公共产物：只读引用
LIVOX_PREFIX="$SHARED_INSTALL/livox_ros_driver2"

# —— 兴飞船参数（全部显式指定，不吃 launch 里的 chengfeng 默认值）——
NAV2_PARAMS="$WS_INSTALL/workspace_nav/share/workspace_nav/config/nav2_params_xingfei.yaml"
# 地图统一从部署根 app 目录读（船端 ~/usv-test/app/map/map.yaml）
NAV_MAP="${USV_HOME:-$HOME/usv-test}/app/map/map.yaml"
MISSION_PARAMS="$WS_INSTALL/workspace_nav/share/workspace_nav/config/mission_stack.xingfei.yaml"
LOC_PARAMS="$WS_INSTALL/usv_localization/share/usv_localization/config/usv_map_odom_tf.yaml"
DOCKING_PARAMS="$WS_INSTALL/usv_docking/share/usv_docking/config/docking.yaml"

log() { printf '\033[1;36m[start_usv_nav]\033[0m %s\n' "$*"; }

# 本栈自己的节点进程名（只用于 stop 兜底，别加宽泛模式）
NODE_PATTERNS=(
  "livox_ros_driver2_node"
  "usv_map_odom_tf_node"
  "component_container_isolated"
  "bt_navigator"
  "controller_server"
  "planner_server"
  "smoother_server"
  "behavior_server"
  "waypoint_follower"
  "velocity_smoother"
  "map_server"
  "lifecycle_manager"
  "nav2_map_server"
  "nav_status_aggregator"
  "mission_bridge"
  "zone_manager"
  "zone_monitor"
  "nav2_goal_checker"
  "ardupilot_velocity_bridge"
  "dock_mission_node"
  "docking_pose_estimator"
  "docking_fsm"
  "docking_motion_controller"
  "docking_safety"
  "rosbridge_websocket"
  "usv_map_pose_bridge"
)

require_tmux() {
  command -v tmux >/dev/null 2>&1 || { log "缺少 tmux：sudo apt-get install -y tmux"; exit 1; }
}

check_files() {
  local missing=0 f
  for f in "$ROS_SETUP" "$WS_SETUP" "$NAV2_PARAMS" "$NAV_MAP" \
           "$MISSION_PARAMS" "$LOC_PARAMS" "$DOCKING_PARAMS"; do
    [ -e "$f" ] || { log "缺少: $f"; missing=1; }
  done
  if [ "$START_LIDAR" = "1" ]; then
    [ -d "$LIVOX_PREFIX" ] || {
      log "缺少激光雷达驱动: $LIVOX_PREFIX"; missing=1; }
  fi
  return $missing
}

check_mavros() {
  # 只提示，不阻塞：mavros 可能晚于本脚本启动
  if ! timeout 8 bash -c "source $ROS_SETUP >/dev/null 2>&1; source $WS_SETUP >/dev/null 2>&1; \
        ros2 topic list 2>/dev/null | grep -q '^$ODOM_TOPIC$'"; then
    log "警告: 没看到 $ODOM_TOPIC（MAVROS 未起？导航/归港会等它）"
  fi
}

ENV_COMMON="export ROS_DOMAIN_ID=$DOMAIN_ID; export RMW_IMPLEMENTATION=$RMW; \
source $ROS_SETUP; source $WS_SETUP"
ENV_MAVROS_MSGS="export LD_LIBRARY_PATH=$SHARED_INSTALL/mavros_msgs/lib:\$LD_LIBRARY_PATH"
ENV_LIVOX="export AMENT_PREFIX_PATH=$LIVOX_PREFIX:\$AMENT_PREFIX_PATH; \
export LD_LIBRARY_PATH=$LIVOX_PREFIX/lib:\$LD_LIBRARY_PATH; \
export PYTHONPATH=$LIVOX_PREFIX/local/lib/python3.10/dist-packages:\$PYTHONPATH"

win() {  # win <窗口名> <命令串>
  local name="$1"; shift
  # 节点退出后保留窗口，方便 attach 看现场
  tmux new-window -t "$SESSION" -n "$name" "$*; echo; echo '--- [${name}] 已退出，按 Ctrl-b w 切窗口 ---'; exec bash"
  log "窗口 [$name] 已启动"
}

start() {
  require_tmux
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    log "会话 $SESSION 已存在：用 '$0 attach' 查看，或 '$0 stop' 停掉后重启"
    return 1
  fi
  check_files || { log "关键文件缺失，已中止"; return 1; }
  check_mavros

  tmux new-session -d -s "$SESSION" -n main
  tmux set-option -t "$SESSION" remain-on-exit on
  tmux set-option -t "$SESSION" destroy-unattached off
  log "已创建 tmux 会话 $SESSION（脱离 SSH 也不会退出），ROS_DOMAIN_ID=$DOMAIN_ID RMW=$RMW"

  # 1) 激光雷达：默认不起（START_LIDAR=1 或 '$0 lidar' 单独补起）
  if [ "$START_LIDAR" = "1" ]; then
    win lidar "$ENV_COMMON; $ENV_LIVOX; ros2 launch livox_ros_driver2 $MID360_LAUNCH"
    sleep 4
  else
    log "激光雷达未启动（需要时: $0 lidar）"
  fi
  # 2) 定位：map→odom（需要自定义 mavros_msgs/GetMapOrigin 的库）
  # publish_robot_tf 保持 false（aef33b8 起定位不再发 URDF TF，base_link→传感器
  # 由整船 launch 的 usv_transforms 发）；map_config_yaml 统一用部署根 app/map，
  # 换图只改 $USV_HOME/app/map/{map.yaml,map.pgm}（锚点不同会偏几十米，勿混用旧图）
  win localize "$ENV_COMMON; $ENV_MAVROS_MSGS; ros2 launch usv_localization usv_map_odom_tf.launch.py params_file:=$LOC_PARAMS map_config_yaml:=$NAV_MAP publish_robot_tf:=false"
  sleep 3
  # 3) 导航：兴飞参数 + mission_bridge
  win nav "$ENV_COMMON; ros2 launch workspace_nav nav2_boat.launch.py use_sim_time:=false use_rviz:=false enable_mission_bridge:=true params_file:=$NAV2_PARAMS map:=$NAV_MAP mission_params_file:=$MISSION_PARAMS odom_topic:=$ODOM_TOPIC"
  # 4) 速度桥：兴飞标定版
  win bridge "$ENV_COMMON; $ENV_MAVROS_MSGS; ros2 launch usv_ardupilot_velocity_bridge ardupilot_velocity_bridge_xingfei.launch.py"
  # 5) 归港编排（/gcs_dock/command、/dock/home、/dock/mission/*、/dock/in_dock）
  win dock "$ENV_COMMON; $ENV_MAVROS_MSGS; ros2 launch dock_mission dock_mission.launch.py profile:=real use_sim_time:=false odom_topic:=$ODOM_TOPIC"
  # 6) 归港精靠泊四节点
  win docking "$ENV_COMMON; ros2 launch usv_docking docking.launch.py use_sim_time:=false test_only:=false params_file:=$DOCKING_PARAMS odom_topic:=$ODOM_TOPIC"

  # 7) 地图工具实时联动：rosbridge WebSocket + /usv_map_pose 船位
  #    网页(when)填 ws://<NX_IP>:$ROSBRIDGE_PORT；/usv_map_pose 来自 TF map→base_link
  if [ "$MAP_TOOL" = "1" ]; then
    if [ -f "$WS/scripts/usv_map_pose_bridge.py" ]; then
      win mappose "$ENV_COMMON; python3 $WS/scripts/usv_map_pose_bridge.py"
    else
      log "警告: 缺少 $WS/scripts/usv_map_pose_bridge.py，跳过船位桥"
    fi
    win rosbridge "$ENV_COMMON; ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=$ROSBRIDGE_PORT address:=0.0.0.0"
  fi

  if [ "$START_TF" = "1" ]; then
    win tf "$ENV_COMMON; ros2 launch usv_transforms publish_tf.launch.py"
  fi

  log "已全部拉起。查看日志: $0 attach   |   停止: $0 stop   |   状态: $0 status"
}

lidar() {
  require_tmux
  [ -d "$LIVOX_PREFIX" ] || { log "缺少激光雷达驱动: $LIVOX_PREFIX"; return 1; }
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    log "会话 $SESSION 不存在，先 '$0 start'（或用 START_LIDAR=1 $0 start）"
    return 1
  fi
  if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx lidar; then
    log "lidar 窗口已存在：$0 attach 查看"
    return 0
  fi
  win lidar "$ENV_COMMON; $ENV_LIVOX; ros2 launch livox_ros_driver2 $MID360_LAUNCH"
}

status() {
  require_tmux
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    log "会话 $SESSION 未运行"
  else
    log "会话 $SESSION 运行中，窗口："
    tmux list-windows -t "$SESSION"
  fi
  echo
  log "关键节点进程："
  local alive=0 p
  for p in "${NODE_PATTERNS[@]}"; do
    if pgrep -af "$p" >/dev/null 2>&1; then
      printf '  \033[32m●\033[0m %s\n' "$p"
      alive=$((alive + 1))
    fi
  done
  [ "$alive" -eq 0 ] && log "（一个都没在跑）"
}

stop() {
  require_tmux
  log "停止 tmux 会话 $SESSION …"
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  sleep 3

  local left p
  for p in "${NODE_PATTERNS[@]}"; do
    pkill -TERM -f "$p" 2>/dev/null || true
  done
  sleep 3
  for p in "${NODE_PATTERNS[@]}"; do
    pkill -KILL -f "$p" 2>/dev/null || true
  done

  left=0
  for p in "${NODE_PATTERNS[@]}"; do
    if pgrep -f "$p" >/dev/null 2>&1; then
      log "仍在运行: $p"
      left=$((left + 1))
    fi
  done
  if [ "$left" -eq 0 ]; then
    log "已停止，本栈节点全部退出"
  else
    log "有 $left 类进程未退出，请 '$0 status' 复查"
  fi
}

case "$ACTION" in
  start)   start ;;
  lidar)   lidar ;;
  stop)    stop ;;
  status)  status ;;
  attach)
    require_tmux
    tmux attach -t "$SESSION" 2>/dev/null || log "没有会话 $SESSION，先 '$0 start'"
    ;;
  restart) stop; sleep 2; start ;;
  *)
    echo "用法: $0 {start|lidar|stop|status|attach|restart}"
    exit 1 ;;
esac
