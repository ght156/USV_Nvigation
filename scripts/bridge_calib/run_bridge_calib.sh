#!/usr/bin/env bash
# 速度桥水上标定编排（船端本地运行；setsid 脱离终端，ssh/4G 断开不影响）
#
# 用法:
#   ~/bridge_calib/run_bridge_calib.sh start    # 全流程:起驱动→录bag→各阶段标定→收尾
#   ~/bridge_calib/run_bridge_calib.sh stop     # 安全中止(停序列→停桥→停bag)
#   ~/bridge_calib/run_bridge_calib.sh status   # 查看进度
#
# 流程（全程自动，约 15 分钟）:
#   baseline → rot_nohold → line → env_verify → (桥换参数) env_probe → (桥换参数) rot_hold
# 安全:序列脚本自带 GUIDED 监控/范围保护;遥控器随时接管即中止。
export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="eth1"/></Interfaces></General></Domain></CycloneDDS>'
set +u
source /opt/ros/humble/setup.bash
source "$HOME/usv-main/install/arm/setup.bash"
set -u

CALIB_DIR="$HOME/bridge_calib"
RUN_LOG="$CALIB_DIR/run.log"
RUN_PID="$CALIB_DIR/.run.pid"
BAG_ROOT="$HOME/usv_bags"

# XML 内含双引号，嵌套 bash -c 会被剥掉导致 CycloneDDS 解析失败；写成 env 文件再 source
ENV_FILE="$CALIB_DIR/.cyclone_env"
mkdir -p "$CALIB_DIR"
printf "export CYCLONEDDS_URI='%s'\n" "$CYCLONEDDS_URI" > "$ENV_FILE"

TOPICS=(
  /cmd_vel_nav
  /mavros/setpoint_velocity/cmd_vel_unstamped
  /mavros/local_position/odom
  /mavros/local_position/velocity_body
  /mavros/global_position/global
  /mavros/global_position/raw/fix
  /mavros/state
  /mavros/rc/in
  /mavros/rc/out
  /mavros/imu/data
  /mavros/vfr_hud
)

log() { echo "[$(date +%H:%M:%S)] $*"; }

is_alive() { [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null; }

ensure_proc() { # name | check-pattern | start-cmd...
  local name="$1" pat="$2"; shift 2
  if pgrep -f "$pat" >/dev/null 2>&1; then
    log "SKIP $name 已在运行"
  else
    setsid nohup bash -c "export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source $ENV_FILE; source /opt/ros/humble/setup.bash; source $HOME/usv-main/install/arm/setup.bash; exec $*" \
      > "$CALIB_DIR/${name}.log" 2>&1 < /dev/null &
    log "OK   $name 已启动 (pid $!)"
  fi
}

wait_fcu() { # 等飞控连接 + odom 就绪,最多约 90s
  log "等待飞控连接与定位就绪..."
  for _ in $(seq 1 18); do
    if timeout 4 ros2 topic echo /mavros/local_position/odom --once >/dev/null 2>&1; then
      log "飞控与定位就绪"
      return 0
    fi
    sleep 1
  done
  log "!! 90s 内定位未就绪（GPS/RTK 正常?），中止"
  return 1
}

start_bag() {
  BAG_DIR="$BAG_ROOT/bridge_calib_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$BAG_ROOT"
  setsid nohup ros2 bag record -s sqlite3 -o "$BAG_DIR" "${TOPICS[@]}" \
    > "$CALIB_DIR/bag.log" 2>&1 < /dev/null &
  BAG_PID=$!
  echo "$BAG_PID" > "$CALIB_DIR/.bag.pid"
  echo "$BAG_DIR" > "$CALIB_DIR/.bag.dir"
  log "开始录制: $BAG_DIR (pid $BAG_PID)"
}

stop_bag() {
  [[ -f "$CALIB_DIR/.bag.pid" ]] || return 0
  local pid; pid="$(cat "$CALIB_DIR/.bag.pid")"
  kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
  rm -f "$CALIB_DIR/.bag.pid"
  log "录制停止: $(cat "$CALIB_DIR/.bag.dir" 2>/dev/null)"
}

start_bridge() { # 额外 launch 参数...
  setsid nohup ros2 launch usv_ardupilot_velocity_bridge ardupilot_velocity_bridge_xingfei.launch.py \
    max_linear_accel:=0.3 max_linear_decel:=0.3 max_angular_accel:=0.5 "$@" \
    > "$CALIB_DIR/bridge.log" 2>&1 < /dev/null &
  BRIDGE_PID=$!
  echo "$BRIDGE_PID" > "$CALIB_DIR/.bridge.pid"
  sleep 5
  log "速度桥已启动: $*"
}

stop_bridge() {
  [[ -f "$CALIB_DIR/.bridge.pid" ]] || return 0
  local pid; pid="$(cat "$CALIB_DIR/.bridge.pid")"
  kill -INT "$pid" 2>/dev/null
  sleep 2
  pkill -INT -f 'usv_ardupilot_velocity_bridge/lib' 2>/dev/null
  sleep 1
  rm -f "$CALIB_DIR/.bridge.pid"
  log "速度桥已停止"
}

run_phase() { # phase 名;返回码非 0 则中止全流程
  local phase="$1"
  log "==== 阶段开始: $phase ===="
  python3 "$CALIB_DIR/calib_seq.py" "$phase"
  local rc=$?
  log "==== 阶段结束: $phase rc=$rc ===="
  return "$rc"
}

main_flow() {
  cd "$CALIB_DIR"
  ensure_proc gi320 'bw_gi320_driver.launch.py' \
    "ros2 launch bw_gi320_driver bw_gi320_driver.launch.py"
  ensure_proc rs485 'usv_rs485_driver.launch.py' \
    "ros2 launch usv_rs485_driver usv_rs485_driver.launch.py"
  ensure_proc mavros 'mavros apm.launch' \
    "ros2 launch mavros apm.launch fcu_url:=udp://:15001@192.168.2.14:15001"
  sleep 8
  wait_fcu || { log "!! 前置条件不满足，退出"; exit 3; }

  start_bag
  sleep 2

  start_bridge enable_in_place_hold:=false enable_velocity_envelope:=true
  run_phase rot_nohold || exit 2
  run_phase line || exit 2
  run_phase env_verify || exit 2
  stop_bridge

  start_bridge enable_in_place_hold:=false enable_velocity_envelope:=false
  run_phase env_probe || exit 2
  stop_bridge

  start_bridge enable_in_place_hold:=true enable_velocity_envelope:=true
  run_phase rot_hold || exit 2
  python3 "$CALIB_DIR/calib_seq.py" baseline || exit 2
  stop_bridge

  stop_bag
  log "全流程完成。bag: $(cat "$CALIB_DIR/.bag.dir" 2>/dev/null)"
}

# 只跑剩余阶段（rot_nohold/line 已有数据时）：env_verify → env_probe → rot_hold → baseline
remaining_flow() {
  cd "$CALIB_DIR"
  ensure_proc gi320 'bw_gi320_driver.launch.py' \
    "ros2 launch bw_gi320_driver bw_gi320_driver.launch.py"
  ensure_proc rs485 'usv_rs485_driver.launch.py' \
    "ros2 launch usv_rs485_driver usv_rs485_driver.launch.py"
  ensure_proc mavros 'mavros apm.launch' \
    "ros2 launch mavros apm.launch fcu_url:=udp://:15001@192.168.2.14:15001"
  sleep 8
  wait_fcu || { log "!! 前置条件不满足，退出"; exit 3; }

  start_bag
  sleep 2

  start_bridge enable_in_place_hold:=false enable_velocity_envelope:=true
  run_phase env_verify || exit 2
  stop_bridge

  start_bridge enable_in_place_hold:=false enable_velocity_envelope:=false
  run_phase env_probe || exit 2
  stop_bridge

  start_bridge enable_in_place_hold:=true enable_velocity_envelope:=true
  run_phase rot_hold || exit 2
  python3 "$CALIB_DIR/calib_seq.py" baseline || exit 2
  stop_bridge

  stop_bag
  log "剩余阶段完成。bag: $(cat "$CALIB_DIR/.bag.dir" 2>/dev/null)"
}

case "${1:-}" in
  start)
    if [[ -f "$RUN_PID" ]] && kill -0 "$(cat "$RUN_PID")" 2>/dev/null; then
      echo "已在运行 (pid $(cat "$RUN_PID"))，先 stop"; exit 1
    fi
    mkdir -p "$CALIB_DIR"
    setsid nohup bash "$0" __run__ >> "$RUN_LOG" 2>&1 < /dev/null &
    echo $! > "$RUN_PID"
    echo "标定已后台启动 (pid $(cat "$RUN_PID"))，日志: $RUN_LOG"
    ;;
  start-remaining)
    if [[ -f "$RUN_PID" ]] && kill -0 "$(cat "$RUN_PID")" 2>/dev/null; then
      echo "已在运行 (pid $(cat "$RUN_PID"))，先 stop"; exit 1
    fi
    mkdir -p "$CALIB_DIR"
    setsid nohup bash "$0" __run_remaining__ >> "$RUN_LOG" 2>&1 < /dev/null &
    echo $! > "$RUN_PID"
    echo "剩余阶段已后台启动 (pid $(cat "$RUN_PID"))，日志: $RUN_LOG"
    ;;
  __run__)
    trap 'stop_bridge; stop_bag; log "流程退出(可能被中止)"' EXIT
    main_flow
    ;;
  __run_remaining__)
    trap 'stop_bridge; stop_bag; log "流程退出(可能被中止)"' EXIT
    remaining_flow
    ;;
  stop)
    pkill -INT -f 'calib_seq.py' 2>/dev/null
    sleep 2
    [[ -f "$RUN_PID" ]] && kill -INT "$(cat "$RUN_PID")" 2>/dev/null
    echo "已发中止信号（桥与 bag 由编排进程收尾），日志: $RUN_LOG"
    ;;
  status)
    if [[ -f "$RUN_PID" ]] && kill -0 "$(cat "$RUN_PID")" 2>/dev/null; then
      echo "运行中 (pid $(cat "$RUN_PID"))"
    else
      echo "未在运行"
    fi
    tail -n 8 "$RUN_LOG" 2>/dev/null
    ;;
  *)
    echo "用法: $0 start|stop|status"; exit 1 ;;
esac
