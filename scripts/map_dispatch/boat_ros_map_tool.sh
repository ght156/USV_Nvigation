#!/usr/bin/env bash
# ros_map_tool 船端（澄丰 nx）一键起服务：rosbridge(9090) + 页面静态 HTTP(8001)
#
# 部署：scripts/map_dispatch/deploy_to_nx.sh 会把它放到船端 ~/ros_map_tool/ros_map_tool.sh
# 船上用法：
#   ~/ros_map_tool/ros_map_tool.sh start     # 起服务（rosbridge 已在跑就复用）
#   ~/ros_map_tool/ros_map_tool.sh status    # 看端口/进程/日志尾巴
#   ~/ros_map_tool/ros_map_tool.sh url       # 只打印该开哪个网址
#   ~/ros_map_tool/ros_map_tool.sh stop      # 只停本脚本起的进程
#
# 说明：
# * 页面里已改成「默认连 页面所在主机:9090」，所以局域网里用 http://<NX_IP>:8001/ 打开即可；
# * 地面站与 NX 不同网段（本机 192.168.1.x / 船 192.168.2.x）时走 ssh 隧道：
#     ssh -N -L 8001:127.0.0.1:8001 -L 9090:127.0.0.1:9090 nx
#   再开 http://127.0.0.1:8001/  （两个端口都要转，页面要连 9090 的 rosbridge）
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$DIR"
PAGE="$DIR/ros_map_tool_v3_nav2_live.html"
HTTP_PORT="${MAP_TOOL_HTTP_PORT:-8001}"
WS_PORT="${MAP_TOOL_WS_PORT:-9090}"
LOG_DIR="$DIR/logs"
LOG_HTTP="$LOG_DIR/http.log"
LOG_WS="$LOG_DIR/rosbridge.log"
PID_HTTP="$LOG_DIR/http.pid"
PID_WS="$LOG_DIR/rosbridge.pid"          # 只有本脚本起的 rosbridge 才写

ROS_SETUP=/opt/ros/humble/setup.bash
USV_SETUPS=("$HOME/usv_nav_ws/install/setup.bash" "$HOME/usv/install/arm/setup.bash")

log() { echo "[ros_map_tool] $*"; }

port_listening() { ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$1\$"; }

lan_ip() {
  hostname -I 2>/dev/null | awk '{print $1}'
}

print_urls() {
  local ip; ip="$(lan_ip)"
  echo
  log "页面地址："
  echo "    本机:   http://127.0.0.1:$HTTP_PORT/"
  [ -n "$ip" ] && echo "    局域网: http://$ip:$HTTP_PORT/    （rosbridge 用 ws://$ip:$WS_PORT）"
  echo "    隧道:   ssh -N -L $HTTP_PORT:127.0.0.1:$HTTP_PORT -L $WS_PORT:127.0.0.1:$WS_PORT nx"
  echo "            → 再开 http://127.0.0.1:$HTTP_PORT/"
  echo
  log "底图（页面「加载 map.yaml + map.pgm」选这两个）：$DIR/map/map.yaml  $DIR/map/map.pgm"
}

do_start() {
  mkdir -p "$LOG_DIR"
  if [ ! -f "$PAGE" ]; then
    log "缺少页面 $PAGE —— 先在地面站跑 scripts/map_dispatch/deploy_to_nx.sh"
    exit 1
  fi

  # 1) rosbridge：已在跑就复用，绝不重复起（重复起会抢 9090 导致两边都收不到）
  if port_listening "$WS_PORT"; then
    log "rosbridge 已在 $WS_PORT 端口运行，直接复用（stop 不会停它）"
  else
    log "启动 rosbridge :$WS_PORT ..."
    setsid bash -c "source '$ROS_SETUP' 2>/dev/null; \
      for s in \"\${HOME}/usv_nav_ws/install/setup.bash\" \"\${HOME}/usv/install/arm/setup.bash\"; do [ -f \"\$s\" ] && source \"\$s\" && break; done; \
      export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-5} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
      exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=$WS_PORT" \
      >"$LOG_WS" 2>&1 &
    echo $! > "$PID_WS"
    sleep 4
  fi

  # 2) 页面静态服务
  if port_listening "$HTTP_PORT"; then
    log "$HTTP_PORT 端口已被占用，复用现有服务"
  else
    log "启动页面 HTTP 服务 :$HTTP_PORT （根目录 $WEB_DIR）"
    setsid python3 -m http.server "$HTTP_PORT" --bind 0.0.0.0 --directory "$WEB_DIR" \
      >"$LOG_HTTP" 2>&1 &
    echo $! > "$PID_HTTP"
    sleep 1
  fi

  # 3) 复核
  port_listening "$WS_PORT" && log "rosbridge :$WS_PORT ✅" || log "rosbridge :$WS_PORT ❌（看 $LOG_WS）"
  port_listening "$HTTP_PORT" && log "页面 :$HTTP_PORT ✅" || log "页面 :$HTTP_PORT ❌（看 $LOG_HTTP）"
  print_urls
}

do_stop() {
  if [ -f "$PID_HTTP" ]; then
    kill "$(cat "$PID_HTTP")" 2>/dev/null && log "已停页面服务 (pid $(cat "$PID_HTTP"))"
    rm -f "$PID_HTTP"
  fi
  if [ -f "$PID_WS" ]; then
    kill "$(cat "$PID_WS")" 2>/dev/null && log "已停 rosbridge (pid $(cat "$PID_WS"))"
    rm -f "$PID_WS"
  else
    log "rosbridge 是外部起的（如 webpath.sh），本脚本不动它"
  fi
}

do_status() {
  echo "web 目录: $WEB_DIR"
  echo "页面    : $PAGE  $([ -f "$PAGE" ] && md5sum "$PAGE" | awk '{print $1}')"
  echo "端口    : ws=$WS_PORT http=$HTTP_PORT"
  port_listening "$WS_PORT" && echo "  ws    : LISTEN" || echo "  ws    : -"
  port_listening "$HTTP_PORT" && echo "  http  : LISTEN" || echo "  http  : -"
  echo "pid 文件: http=$([ -f "$PID_HTTP" ] && cat "$PID_HTTP" || echo -) ws=$([ -f "$PID_WS" ] && cat "$PID_WS" || echo -)"
  for f in "$LOG_WS" "$LOG_HTTP"; do
    [ -f "$f" ] && { echo "--- $f (tail) ---"; tail -3 "$f"; }
  done
  echo "--- 底图 ---"; ls -l "$DIR/map" 2>/dev/null | head
}

case "${1:-start}" in
  start)  do_start ;;
  stop)   do_stop ;;
  status) do_status ;;
  url)    print_urls ;;
  *)      sed -n '2,18p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac
