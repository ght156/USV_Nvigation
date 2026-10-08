#!/usr/bin/env bash
# --------------------------------------------------------------------------------------- #
# record_dock_data.sh — 主机侧归港调试数据录制（参考 record_field_data.sh）
#
# 用法：
#   ./record_dock_data.sh check           # 检查话题是否在线 + QoS，不录制
#   ./record_dock_data.sh start [标签]    # 开始录制，如 ./record_dock_data.sh start dock_rc1
#   ./record_dock_data.sh status          # 是否在录 / 存到哪 / 磁盘剩余
#   ./record_dock_data.sh stop            # 停止并打印各话题消息数（为 0 的会标红提醒）
#
# 话题清单参考 dock_tune_gui.py 的 BAG_TOPIC_GROUPS，自定义消息
# m_common/msg/DetectionObjList 需要 source USV_NAV 工作空间（脚本自动 source）。
# 产物：$BAG_DIR/dock_<时间>[_标签]/（*.db3 + metadata.yaml + record.log）
# --------------------------------------------------------------------------------------- #

set -u

TAG="${2:-}"
ROSETUP="/opt/ros/humble/setup.bash"
# 主机/船双机通用：自动选本机工作空间（主机 USV_NAV / 船端 usv_nav_ws）
for ws in /home/ght/USV_NAV/install/setup.bash /home/jetson/usv_nav_ws/install/setup.bash; do
  [[ -f "$ws" ]] && WSSETUP="$ws" && break
done

# ======================================================================= #
# 配置区
# ======================================================================= #
BAG_DIR="$HOME/usv_bags"           # 主机=/home/ght/usv_bags，船=/home/jetson/usv_bags（分析时 scp 回主机）
DOMAIN_ID=5
# ======================================================================= #

PID_FILE="$BAG_DIR/.recording.pid"
INFO_FILE="$BAG_DIR/.recording.info"

# —— 话题清单（在线才录得到的会自动发现类型；不在线的录成 0 条，stop 时会提醒）——
TOPICS=(
  # 二维码定位（核心：静止/进出坞全过程的 tag 测量）
  "/apriltag_node/detections"
  # 归港核心
  "/docking/dock_pose" "/docking/tag_visible" "/docking/pose_source"
  "/docking/measurement_age" "/docking/state" "/docking/target_mode"
  "/docking/safety_stop" "/docking/abort_request" "/docking/cmd_vel_test"
  "/cmd_vel_nav"
  # 任务/指令
  "/dock/status" "/dock/start" "/dock/cancel" "/dock/undock"
  "/dock/entry_status" "/dock/mission_status" "/dock/speed_authority"
  "/dock_task/status" "/dock_task/event_log"
  "/gcs_dock/command"
  # 定位
  "/mavros/gps_input/local" "/mavros/local_position/odom" "/odometry/filtered"
  # IMU（角速度真值，分析转向响应延迟/过冲用，2026-10-07）
  "/mavros/imu/data"
  # 转向链路延迟测量（2026-10-07）：把 cmd_vel_nav → 桥输出 → 飞控 PWM → IMU
  # 四段两两互相关，可把 ~2s 的转向滞后拆成「桥 / 链路+飞控 / 船体」三段
  "/mavros/setpoint_velocity/cmd_vel_unstamped"   # 速度桥的输出（发给飞控）
  "/mavros/rc/out"                                # 飞控执行器 PWM 输出
  # 全局经纬度（停船位 RTK 经纬度归档用）+ map->odom 锁定值（含协方差）
  "/mavros/global_position/global" "/usv_localization/map_odom"
  # TF
  "/tf" "/tf_static"
  # 相机压缩图（回看 tag 检测现场画面，排查误检/漏检）
  "/camera_back_video/color/image_raw/compressed"
)

log()  { echo -e "\033[1;36m[record]\033[0m $*"; }
warn() { echo -e "\033[1;33m[record]\033[0m $*"; }
err()  { echo -e "\033[1;31m[record]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[record]\033[0m $*"; }

ensure_ros() {
  export ROS_DOMAIN_ID="$DOMAIN_ID"
  set +u   # ROS setup.bash 引用了未定义变量，与 set -u 冲突
  [[ -f "$ROSETUP" ]] && source "$ROSETUP"
  [[ -f "$WSSETUP" ]] && source "$WSSETUP"   # m_common 自定义消息类型
  set -u
  command -v ros2 >/dev/null 2>&1 || { err "找不到 ros2"; exit 1; }
}

is_running() {
  [[ -f "$PID_FILE" ]] || return 1
  local pid
  pid="$(cat "$PID_FILE" 2>/dev/null)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  if [[ -r "/proc/$pid/cmdline" ]]; then
    tr '\0' ' ' < "/proc/$pid/cmdline" | grep -q "bag record" || return 1
  fi
  return 0
}

disk_avail_gb() {
  local d="$1"
  while [[ ! -d "$d" && "$d" != "/" ]]; do d="$(dirname "$d")"; done
  df -BG --output=avail "$d" 2>/dev/null | tail -1 | tr -dc '0-9'
}

fmt_dur() {
  local s=$1 h m
  h=$((s / 3600)); m=$(((s % 3600) / 60)); s=$((s % 60))
  ((h > 0)) && printf '%dh%02dm%02ds' "$h" "$m" "$s" || printf '%dm%02ds' "$m" "$s"
}

cmd_check() {
  ensure_ros
  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID，检查话题…"
  local missing=0 t info pcount ptype
  for t in "${TOPICS[@]}"; do
    info="$(ros2 topic info "$t" 2>&1)"
    if [[ "$info" == *"Unknown topic"* || "$info" == *"not found"* ]]; then
      warn "  △ $t —— 不存在（发布端没起则录不上，可先录后补）"
      missing=1
      continue
    fi
    ptype="$(grep -m1 '^Type:' <<<"$info" | awk '{print $2}')"
    pcount="$(grep -m1 'Publisher count:' <<<"$info" | awk '{print $3}')"
    if [[ "${pcount:-0}" -eq 0 ]]; then
      warn "  △ $t  类型=$ptype  发布者=0"
      missing=1
    else
      ok "  ✓ $t  类型=$ptype  发布者=$pcount"
    fi
  done
  log "磁盘可用：$(disk_avail_gb "$BAG_DIR" || echo '?') GB（$BAG_DIR）"
  [[ $missing -eq 0 ]] && ok "全部话题在线，可以 start" || warn "有话题暂不在线（见上），不影响 start，stop 时会复查消息数"
}

cmd_start() {
  ensure_ros
  if is_running; then
    err "已有录制在跑（pid $(cat "$PID_FILE")），先执行 $0 stop"
    exit 1
  fi
  rm -f "$PID_FILE" "$INFO_FILE"
  mkdir -p "$BAG_DIR"

  local name bag ts
  ts="$(date +%Y%m%d_%H%M%S)"
  name="dock_${ts}"
  [[ -n "$TAG" ]] && name="${name}_${TAG}"
  bag="$BAG_DIR/$name"
  [[ -e "$bag" ]] && { err "输出目录已存在：$bag"; exit 1; }

  local cmd
  cmd=(ros2 bag record -s sqlite3 -o "$bag")
  local t
  for t in "${TOPICS[@]}"; do cmd+=("$t"); done

  log "开始录制：$bag"
  log "话题数：${#TOPICS[@]} | DOMAIN_ID=$ROS_DOMAIN_ID"

  local logfile="$BAG_DIR/${name}.record.log"
  setsid nohup "${cmd[@]}" >> "$logfile" 2>&1 < /dev/null &
  local pid=$!
  echo "$pid" > "$PID_FILE"
  echo "$bag|$(date +%s)|${TOPICS[*]}" > "$INFO_FILE"

  sleep 2
  if ! kill -0 "$pid" 2>/dev/null; then
    err "录制进程启动即退出，日志尾部："
    tail -n 15 "$logfile"
    rm -f "$PID_FILE" "$INFO_FILE"
    exit 1
  fi
  ok "录制中（pid $pid）。停止：$0 stop   查看状态：$0 status"
  warn "提示：开始遥控前先让船静止十几秒录静止 tag；全程录完后执行 stop，会自动核对各话题消息数。"
}

cmd_status() {
  if is_running; then
    local pid bag start_epoch now
    pid="$(cat "$PID_FILE")"
    IFS='|' read -r bag start_epoch _ < "$INFO_FILE" 2>/dev/null || bag="?"
    now="$(date +%s)"
    log "正在录制（pid $pid）：$bag"
    [[ "$bag" != "?" ]] && {
      log "已录 $(fmt_dur $((now - start_epoch)))，文件："
      ls -lh "$bag" 2>/dev/null | awk '/\.db3|\.mcap/ {print "    "$9"  ("$5")"}'
    }
    log "磁盘可用：$(disk_avail_gb "$BAG_DIR" || echo '?') GB"
  else
    log "当前没有录制任务。"
    local last
    last="$(ls -1dt "$BAG_DIR"/dock_* 2>/dev/null | head -1)"
    [[ -n "$last" ]] && log "最近一次录制：$last（$(du -sh "$last" 2>/dev/null | awk '{print $1}')）"
  fi
}

cmd_stop() {
  if ! is_running; then
    warn "没有正在进行的录制。"
    rm -f "$PID_FILE" "$INFO_FILE"
    exit 0
  fi
  local pid bag start_epoch
  pid="$(cat "$PID_FILE")"
  IFS='|' read -r bag start_epoch _ < "$INFO_FILE"

  log "正在停止录制（pid $pid）… SIGINT 优雅收尾"
  kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
  local i
  for i in $(seq 1 40); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.5
  done
  if kill -0 "$pid" 2>/dev/null; then
    err "进程未退出（pid $pid）。为保护数据未强杀，请手动：kill -TERM $pid"
    exit 1
  fi

  rm -f "$PID_FILE" "$INFO_FILE"
  local dur=0
  [[ -n "${start_epoch:-}" ]] && dur=$(( $(date +%s) - start_epoch ))
  ok "已停止。时长 $(fmt_dur "$dur")，目录：$bag"

  if [[ -f "$bag/metadata.yaml" ]]; then
    python3 - "$bag/metadata.yaml" "${TOPICS[@]}" <<'PY' || warn "（无法解析 metadata.yaml）"
import sys, subprocess
try:
    import yaml
except ImportError:
    print("  （无 pyyaml，跳过统计；用 ros2 bag info <目录> 查看）")
    sys.exit(0)
meta = yaml.safe_load(open(sys.argv[1]))["rosbag2_bagfile_information"]
want = list(sys.argv[2:])
rows = {r["topic_metadata"]["name"]: (r["topic_metadata"]["type"], r.get("message_count", 0))
        for r in (meta.get("topics_with_message_count") or [])}
print("  各话题消息数：")
for t in want:
    if t not in rows:
        print(f"    {t:<46} {'-':<40} {'从未出现':>8}  ← 发布端全程没起来")
        continue
    ptype, n = rows[t]
    flag = "  ← 0 条，没录到！" if n == 0 else ""
    print(f"    {t:<46} {ptype:<40} {n:>10}{flag}")
for t, (ptype, n) in rows.items():
    if t not in want:
        print(f"    {t:<46} {ptype:<40} {n:>10}（额外录到的）")
files = meta.get("relative_file_paths") or []
print(f"  分片数：{len(files)}，总大小：", end="", flush=True)
subprocess.run(["du", "-sh", sys.argv[1].rsplit("/", 1)[0]])
PY
  else
    warn "未找到 metadata.yaml —— 试试: ros2 bag info \"$bag\""
  fi
}

usage() {
  cat <<'EOF'
主机侧归港调试数据录制（话题清单见脚本顶部 TOPICS）
  ./record_dock_data.sh check          # 检查话题在线情况
  ./record_dock_data.sh start [标签]   # 开始录制（后台，断终端不停）
  ./record_dock_data.sh status         # 录制状态
  ./record_dock_data.sh stop           # 优雅停止 + 各话题消息数自检
EOF
}

case "${1:-}" in
  check)  cmd_check ;;
  start)  cmd_start ;;
  status) cmd_status ;;
  stop)   cmd_stop ;;
  *)      usage; exit 1 ;;
esac
