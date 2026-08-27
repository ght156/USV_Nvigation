#!/usr/bin/env bash
# --------------------------------------------------------------------------------------- #
# start_nx_stack.sh — 在 NX（tegra-ubuntu, jetson）按顺序一键启动整条导航链路
#
# 启动顺序（硬依赖：先感知/定位，再导航/桥，最后巡逻）：
#   1) rtk      : bw_gi320_driver                              (~/usv install/arm)
#   2) mavros   : mavros apm.launch fcu_url:=/dev/ttyTHS1:921600 (~/mavros_ws)
#   3) localize : usv_localization usv_map_odom_tf.launch.py    (map→odom, ~/usv install/arm)
#   4) nav      : workspace_nav nav2_real_mavros.launch.py      (use_rviz:=false, NX 无显示)
#   5) bridge   : usv_ardupilot_velocity_bridge ardupilot_velocity_bridge.launch.py
#   —— 等 Nav2 就绪（bt_navigator 出现）后——
#   6) patrol   : scripts/rviz_waypoint_patrol.py（自动加载已保存点；需 attach 后手动 go）
#
# 每个组件一个 tmux 窗口，断连 SSH 仍保持运行。
#
# 用法：
#   ./start_nx_stack.sh start      # 按上述顺序启动
#   ./start_nx_stack.sh attach     # 进入 tmux 查看/操作
#   ./start_nx_stack.sh stop       # 停止整个会话（含所有节点）
#   ./start_nx_stack.sh status     # 查看会话状态
#
# 环境变量：
#   ROS_DOMAIN_ID    默认 5（与主机一致）
#   NX_STACK_SESSION 默认 nx_stack
#   AUTO_GO=1        巡逻脚本起来后自动发 /rviz_patrol/start（慎用，会直接发车）
#
#ssh nx
  # 1) 停掉整栈（含旧巡逻）
  #~/usv_nav_ws/scripts/start_nx_stack.sh stop

  # 2) 用新脚本起栈（不自动起巡逻）
 # ~/usv_nav_ws/scripts/start_nx_stack.sh start

  # 3) 等约 1 分钟 Nav2 就绪后，在【独立终端】启动巡逻（带重发）
 # ssh nx
 # source /opt/ros/humble/setup.bash
 # source ~/usv_nav_ws/install/setup.bash
  #python3 ~/usv_nav_ws/scripts/rviz_waypoint_patrol.py
  # 然后在该终端输入：go
 --------------------------------------------------------------------------------------- #

set -u

SESSION="${NX_STACK_SESSION:-nx_stack}"
ACTION="${1:-start}"
DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
export ROS_DOMAIN_ID

ROSETUP="/opt/ros/humble/setup.bash"
NAV_SETUP="$HOME/usv_nav_ws/install/setup.bash"
MAVROS_SETUP="$HOME/mavros_ws/install/setup.bash"
USV_SETUP="$HOME/usv/install/arm/setup.bash"
SCRIPTS_DIR="$HOME/usv_nav_ws/scripts"

log() { echo -e "\033[1;36m[start_nx_stack]\033[0m $*"; }

ensure_session() {
  command -v tmux >/dev/null 2>&1 || { log "缺少 tmux，请先: sudo apt-get install -y tmux"; exit 1; }
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    log "会话 $SESSION 已存在。用 '$0 attach' 查看，或 '$0 stop' 后再 start。"
    return 1
  fi
  tmux new-session -d -s "$SESSION" -n main
  # 进程退出的窗口不自动关闭，保留日志便于排查
  tmux set-option -t "$SESSION" remain-on-exit on
  log "已创建 tmux 会话: $SESSION (ROS_DOMAIN_ID=$DOMAIN_ID)"
  return 0
}

cleanup_stale() {
  log "清理残留节点（确保上次的实例已退出，避免占串口/端口）…"
  for p in \
    "mavros_nod[e]" \
    "ros2 launc[h]" \
    "bw_gi320_driv[e]r" \
    "usv_map_odom_t[f]" \
    "component_container_isolate[d]" \
    "ardupilot_velocity_bridg[e]" \
    "rviz_waypoint_patro[l]" \
    "nav2_real_mavro[s]" \
    "nav_status_aggregato[r]" \
    "zone_manag[e]r" "zone_monito[r]" ; do
    pkill -TERM -f "$p" 2>/dev/null
  done
  sleep 3
  log "清理完成。"
}

win() {
  # win <名称> <命令>
  local name="$1"; shift
  tmux new-window -t "$SESSION" -n "$name" "export ROS_DOMAIN_ID=$DOMAIN_ID; $*"
  log "窗口 [$name] 已启动"
}

rospkg_ready() {
  # 探测某个 Nav2 action/goal 服务是否可用（用于等导航就绪）
  timeout 6 bash -lc "source $ROSETUP >/dev/null 2>&1 && source $NAV_SETUP >/dev/null 2>&1 && ros2 node list 2>/dev/null | grep -qE 'bt_navigator|controller_server'"
}

wait_nav_ready() {
  log "等待 Nav2 就绪（bt_navigator / controller_server）…"
  # 先预热 ros2 daemon（首次 node list 较慢，避免被 timeout 误杀）
  timeout 20 bash -lc "source $ROSETUP >/dev/null 2>&1 && source $NAV_SETUP >/dev/null 2>&1 && ros2 node list >/dev/null 2>&1" || true
  for i in $(seq 1 30); do
    if rospkg_ready; then
      log "Nav2 就绪（第 ${i} 次探测）。"
      return 0
    fi
    sleep 2
  done
  log "警告: 约 60s 内未探测到 Nav2 就绪，仍继续启动巡逻；请 attach 到 [nav] 窗口查看报错。"
  return 0
}

start() {
  ensure_session || return
  cleanup_stale

  # —— 第 1 步：先感知/定位（RTK + MAVROS + map→odom）——
  win rtk "source $ROSETUP && source $USV_SETUP && ros2 launch bw_gi320_driver bw_gi320_driver.launch.py"
  sleep 3
  win mavros "source $ROSETUP && source $MAVROS_SETUP && ros2 launch mavros apm.launch fcu_url:=/dev/ttyTHS1:921600"
  sleep 3
  win localize "source $ROSETUP && source $USV_SETUP && ros2 launch usv_localization usv_map_odom_tf.launch.py"
  sleep 3

  # —— 第 2 步：导航（关 rviz）+ ArduPilot 速度桥 ——
  win nav "source $ROSETUP && source $NAV_SETUP && ros2 launch workspace_nav nav2_real_mavros.launch.py use_sim_time:=false use_rviz:=false"
  win bridge "source $ROSETUP && source $MAVROS_SETUP && source $NAV_SETUP && ros2 launch usv_ardupilot_velocity_bridge ardupilot_velocity_bridge.launch.py"

  # —— 第 3 步：不自动起巡逻。等导航就绪提示，巡逻由你在独立终端单独启动 ——
  log "导航栈已启动（rtk/mavros/localize/nav/bridge）。"
  log "等 Nav2 就绪后(约 1 分钟)，在【独立终端】启动巡逻："
  log "  ssh nx && source /opt/ros/humble/setup.bash && source ~/usv_nav_ws/install/setup.bash && python3 ~/usv_nav_ws/scripts/rviz_waypoint_patrol.py"
  log "查看栈日志: tmux attach -t $SESSION"
}

case "$ACTION" in
  start) start ;;
  attach)
    tmux attach -t "$SESSION" 2>/dev/null || log "没有会话 $SESSION，先 ./start_nx_stack.sh start"
    ;;
  stop)
    tmux kill-session -t "$SESSION" 2>/dev/null && log "已停止会话 $SESSION（所有窗口中节点被终止）" || log "没有运行中的会话 $SESSION"
    ;;
  status)
    if tmux has-session -t "$SESSION" 2>/dev/null; then
      log "会话 $SESSION 运行中，窗口："
      tmux list-windows -t "$SESSION"
    else
      log "会话 $SESSION 未运行"
    fi
    ;;
  *)
    echo "用法: $0 {start|attach|stop|status}"
    exit 1
    ;;
esac
