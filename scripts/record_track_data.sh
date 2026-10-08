#!/usr/bin/env bash
# --------------------------------------------------------------------------------------- #
# record_track_data.sh — 船端轨迹对比数据录制（拷到 NX 上离线运行）
#
# 用途：平台测试期间录制"上报源 vs 导航源 vs 规划"三路轨迹，事后拷回主机分析
#   平台画轨迹用的数据源：/gi320/gngga（GNGGA 原始 GNSS 解，WGS84）
#   导航实际用的定位源  ：/gi320/inspvaa（INS 组合导航融合解）
#   导航轨迹/规划       ：/mavros/local_position/odom、/plan、/local_plan、TF
#
# 用法（在船上）：
#   ./record_track_data.sh check           # 检查话题是否在线，不录制
#   ./record_track_data.sh start [标签]    # 开始录制（tmux 离线运行，断 SSH 不停）
#   ./record_track_data.sh status          # 是否在录 / 存到哪 / 磁盘剩余
#   ./record_track_data.sh stop            # 停止并打印各话题消息数（0 条的标红）
#
# 拷回主机分析：
#   scp -r 'nx:~/usv_bags/track_*' ~/usv_bags/
#   python3 export_track_csv.py ~/usv_bags/track_<时间>   # 导出 CSV 对比轨迹
#
# 说明：
#   - 优先用 tmux 会话离线运行；无 tmux 时退化为 setsid+nohup，断终端同样不停。
#   - bag 按 300s 自动分片（-d 300），长时间测试不会产出单个巨大文件。
#   - 自定义消息 bw_gi320_driver/* 需要 source 装有该包的工作空间（脚本自动探测）。
# --------------------------------------------------------------------------------------- #

set -u

TAG="${2:-}"
ROSETUP="/opt/ros/humble/setup.bash"
# 依次 source 存在的工作空间（后面的覆盖/叠加前面的；自定义消息类型在里面）
WS_CANDIDATES=(
  /home/jetson/usv_nav_ws/install/setup.bash    # 船端导航工作区（含 bw_gi320_driver）
  /home/jetson/usv/install/setup.bash           # 船端主仓库工作区（如存在）
  /home/ght/USV_NAV/install/setup.bash          # 主机侧（本机调试时）
)

# ======================================================================= #
# 配置区
# ======================================================================= #
BAG_DIR="$HOME/usv_bags"             # 船=/home/jetson/usv_bags
DOMAIN_ID=5                          # 船端 ROS_DOMAIN_ID
RMW="rmw_cyclonedds_cpp"             # 船端 RMW
TMUX_SESSION="usv_track_rec"         # tmux 会话名
SPLIT_SEC=300                        # bag 分片时长（秒）
MIN_FREE_GB=2                        # 磁盘低于此值拒绝启动
# ======================================================================= #

PID_FILE="$BAG_DIR/.track_recording.pid"
INFO_FILE="$BAG_DIR/.track_recording.info"

# —— 话题清单 ——
TOPICS=(
  # 定位三路（核心对比对象）
  "/gi320/gngga"                     # 平台 OSD 上报的数据源（原始 GNSS 解）
  "/gi320/inspvaa"                   # 导航使用的定位源（INS 融合解）
  "/mavros/local_position/odom"      # Nav2 实际使用的里程计
  "/mavros/global_position/global"   # 参考：飞控解释后的全局位置（可能 NO_FIX）
  # 规划轨迹
  "/plan"                            # Nav2 全局规划路径（map 系）
  "/local_plan"                      # 控制器局部路径
  # 指令链（原地旋转/倒退问题用）
  "/cmd_vel_nav"
  "/mavros/setpoint_velocity/cmd_vel_unstamped"
  "/mavros/state"
  # TF（map->odom->base_link，复盘坐标对齐）
  "/tf" "/tf_static"
)

log()  { echo -e "\033[1;36m[record]\033[0m $*"; }
warn() { echo -e "\033[1;33m[record]\033[0m $*"; }
err()  { echo -e "\033[1;31m[record]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[record]\033[0m $*"; }

ensure_ros() {
  export ROS_DOMAIN_ID="$DOMAIN_ID"
  export RMW_IMPLEMENTATION="$RMW"
  set +u   # ROS setup.bash 引用了未定义变量，与 set -u 冲突
  [[ -f "$ROSETUP" ]] && source "$ROSETUP"
  local ws
  for ws in "${WS_CANDIDATES[@]}"; do
    [[ -f "$ws" ]] && source "$ws"
  done
  set -u
  command -v ros2 >/dev/null 2>&1 || { err "找不到 ros2"; exit 1; }
}

has_tmux() { command -v tmux >/dev/null 2>&1; }

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

# —— tmux/后台 会话内部实际执行的录制（勿手动调）——
cmd_run() {
  local bag="$1" logfile="$2"
  ensure_ros
  echo "$$" > "$PID_FILE"
  exec ros2 bag record -s sqlite3 -d "$SPLIT_SEC" -o "$bag" "${TOPICS[@]}" >> "$logfile" 2>&1
}

cmd_check() {
  ensure_ros
  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION，检查话题…（ros2 CLI 偶发卡死，已加 10s 超时）"
  local missing=0 t info pcount ptype
  for t in "${TOPICS[@]}"; do
    info="$(timeout 10 ros2 topic info "$t" 2>&1)"
    if [[ $? -eq 124 ]]; then
      warn "  △ $t —— ros2 topic info 超时（CLI 卡死老毛病），可先 start，stop 时核对消息数"
      missing=1
      continue
    fi
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
  # 自定义消息类型必须能解析，否则录不进
  if timeout 10 ros2 interface show bw_gi320_driver/msg/Gi320Gngga >/dev/null 2>&1; then
    ok "  ✓ bw_gi320_driver 消息类型可解析"
  else
    err "  ✗ bw_gi320_driver 消息类型无法解析——/gi320/* 录不上！检查工作空间 source（WS_CANDIDATES）"
    missing=1
  fi
  log "tmux：$(has_tmux && echo 可用 || echo '未安装（将用 setsid+nohup 退化模式）')"
  log "磁盘可用：$(disk_avail_gb "$BAG_DIR" || echo '?') GB（$BAG_DIR）"
  [[ $missing -eq 0 ]] && ok "全部话题在线，可以 start" || warn "有话题暂不在线（见上），不影响 start，stop 时会复查消息数"
}

cmd_start() {
  if is_running; then
    err "已有录制在跑（pid $(cat "$PID_FILE")），先执行 $0 stop"
    exit 1
  fi
  mkdir -p "$BAG_DIR"
  rm -f "$PID_FILE" "$INFO_FILE"

  local free
  free="$(disk_avail_gb "$BAG_DIR" || echo 999)"
  if [[ "$free" -lt "$MIN_FREE_GB" ]]; then
    err "磁盘剩余 ${free}GB < ${MIN_FREE_GB}GB，拒绝启动。先清理旧 bag：ls -lt $BAG_DIR"
    exit 1
  fi

  local name bag ts logfile
  ts="$(date +%Y%m%d_%H%M%S)"
  name="track_${ts}"
  [[ -n "$TAG" ]] && name="${name}_${TAG}"
  bag="$BAG_DIR/$name"
  logfile="$BAG_DIR/${name}.record.log"
  [[ -e "$bag" ]] && { err "输出目录已存在：$bag"; exit 1; }

  log "开始录制：$bag"
  log "话题数：${#TOPICS[@]} | DOMAIN_ID=$DOMAIN_ID | 分片 ${SPLIT_SEC}s"

  if has_tmux; then
    tmux kill-session -t "$TMUX_SESSION" 2>/dev/null
    tmux new-session -d -s "$TMUX_SESSION" "'$0' _run '$bag' '$logfile'"
  else
    setsid nohup "$0" _run "$bag" "$logfile" >> "$logfile" 2>&1 < /dev/null &
  fi
  echo "$bag|$(date +%s)|${TOPICS[*]}" > "$INFO_FILE"

  sleep 3
  if ! is_running; then
    err "录制进程未能起来，日志尾部："
    tail -n 15 "$logfile" 2>/dev/null
    rm -f "$PID_FILE" "$INFO_FILE"
    exit 1
  fi
  ok "录制中（pid $(cat "$PID_FILE")）。停止：$0 stop   查看状态：$0 status"
  if has_tmux; then
    log "tmux 会话：$TMUX_SESSION（tmux attach -t $TMUX_SESSION 可看实时输出，Ctrl+B 再按 D 脱离）"
  fi
  log "断 SSH 不影响录制；测完执行 stop 后把 $bag 拷回主机分析。"
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
    last="$(ls -1dt "$BAG_DIR"/track_* 2>/dev/null | grep -v '\.log$' | head -1)"
    [[ -n "$last" ]] && log "最近一次录制：$last（$(du -sh "$last" 2>/dev/null | awk '{print $1}')）"
  fi
}

cmd_stop() {
  if ! is_running; then
    warn "没有正在进行的录制。"
    rm -f "$PID_FILE" "$INFO_FILE"
    has_tmux && tmux kill-session -t "$TMUX_SESSION" 2>/dev/null
    exit 0
  fi
  local pid bag start_epoch
  pid="$(cat "$PID_FILE")"
  IFS='|' read -r bag start_epoch _ < "$INFO_FILE"

  log "正在停止录制（pid $pid）… SIGINT 优雅收尾"
  if has_tmux && tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    tmux send-keys -t "$TMUX_SESSION" C-c
  else
    kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
  fi
  local i
  for i in $(seq 1 40); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.5
  done
  if kill -0 "$pid" 2>/dev/null; then
    err "进程未退出（pid $pid）。为保护数据未强杀，请手动：kill -TERM $pid"
    exit 1
  fi
  has_tmux && tmux kill-session -t "$TMUX_SESSION" 2>/dev/null

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
  echo
  log "拷回主机：scp -r '$bag' <主机>:~/usv_bags/   然后 python3 export_track_csv.py <目录>"
}

usage() {
  cat <<'EOF'
船端轨迹对比数据录制（话题清单见脚本顶部 TOPICS）
  ./record_track_data.sh check          # 检查话题在线情况 + 消息类型可解析
  ./record_track_data.sh start [标签]   # 开始录制（tmux 离线，断 SSH 不停）
  ./record_track_data.sh status         # 录制状态
  ./record_track_data.sh stop           # 优雅停止 + 各话题消息数自检
拷回分析：scp -r 'nx:~/usv_bags/track_*' ~/usv_bags/
         python3 export_track_csv.py ~/usv_bags/track_<时间>
EOF
}

case "${1:-}" in
  _run)    shift; cmd_run "$@" ;;
  check)   cmd_check ;;
  start)   cmd_start ;;
  status)  cmd_status ;;
  stop)    cmd_stop ;;
  *)       usage; exit 1 ;;
esac
