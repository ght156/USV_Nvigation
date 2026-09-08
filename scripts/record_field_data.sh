#!/usr/bin/env bash
# --------------------------------------------------------------------------------------- #
# record_field_data.sh — 现场数据录制（AGX / NX 通用）：点云 + IMU + RTK
#
# 录制话题（要改话题名/增删，直接改下面 TOPICS 数组即可）：
#   /livox/lidar   sensor_msgs/msg/PointCloud2   Mid360 点云
#   /livox/imu     sensor_msgs/msg/Imu           Mid360 内置 IMU（若用外部 IMU 改成实际话题）
#   /rtk/odom      nav_msgs/msg/Odometry         RTK 暂定标准里程计类型；话题名定了以后改这里
#
# 说明：
#   - 三个话题都是标准消息类型，本脚本只依赖 /opt/ros，不需要 source 工作空间。
#   - QoS：点云/IMU 发布端一般是 best_effort，rosbag2 默认用 reliable 订阅会一条都收不到，
#     所以 start 时自动生成 QoS override 文件（对三话题 best_effort 订阅，兼容 reliable 发布端）。
#   - Humble 的 ros2 bag record 不支持 topic:type 写法，消息类型由发布端自动发现。
#   - 后台 setsid 运行，SSH 断连不影响；stop 发 SIGINT 优雅收尾（正常写完 metadata.yaml），
#     不要 kill -9，强杀会导致最后一个分片没有 metadata。
#
# 用法：
#   ./record_field_data.sh check           # 只检查话题是否在线 + QoS，不录制（上场前先跑一遍）
#   ./record_field_data.sh start [标签]    # 开始录制，如 ./record_field_data.sh start dock_run1
#   ./record_field_data.sh status          # 是否在录 / 存到哪 / 多大 / 磁盘剩余
#   ./record_field_data.sh stop            # 停止并打印各话题消息数（为 0 的会标红提醒）
#
# 配置：不用环境变量，直接改下方「配置区」里的 BAG_DIR / SPLIT_SEC / COMPRESS /
#       EXTRA_TOPICS / DOMAIN_ID 几个变量（脚本顶部，一眼能看到）。
#
# 产物：$BAG_DIR/usv_<时间>[_标签]/（*.db3 分片 + metadata.yaml + record.log + qos.yaml）
# 拷回：scp -r nx@<ip>:~/usv_bags/<目录名> .
# --------------------------------------------------------------------------------------- #

set -u

TAG="${2:-}"
ROSETUP="/opt/ros/humble/setup.bash"

# ======================================================================= #
# 配置区：上场前直接改这里（改完保存即可，不用环境变量）
# ======================================================================= #
BAG_DIR="$HOME/usv_bags"   # 录制根目录（AGX 的 SD/eMMC 空间小的话指到 NVMe/优盘）
SPLIT_SEC=1800              # 单个 bag 分片时长（秒），0=不分片。分片方便拷回和回放定位
COMPRESS=0                 # 置 1 开 zstd 压缩（体积约省一半，但吃 AGX CPU，点云大时慎用）
EXTRA_TOPICS=""            # 追加话题，如 "/tf /tf_static"（默认 QoS，发布端若为 best_effort 则收不到）
DOMAIN_ID=5                # ROS_DOMAIN_ID，与车端保持一致
# ======================================================================= #

PID_FILE="$BAG_DIR/.recording.pid"
INFO_FILE="$BAG_DIR/.recording.info"   # 格式：bag目录|起始epoch|话题列表

# —— 话题清单（name 不带类型；类型由发布端决定，bag 里如实记录）——
TOPICS=(
  "/livox/lidar"
  "/livox/imu"
  "/rtk/odom"
)

log()  { echo -e "\033[1;36m[record]\033[0m $*"; }
warn() { echo -e "\033[1;33m[record]\033[0m $*"; }
err()  { echo -e "\033[1;31m[record]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[record]\033[0m $*"; }

ensure_ros() {
  export ROS_DOMAIN_ID="$DOMAIN_ID"
  if ! command -v ros2 >/dev/null 2>&1; then
    [[ -f "$ROSETUP" ]] && source "$ROSETUP"
  fi
  command -v ros2 >/dev/null 2>&1 || { err "找不到 ros2，且 $ROSETUP 不存在"; exit 1; }
}

is_running() {
  [[ -f "$PID_FILE" ]] || return 1
  local pid
  pid="$(cat "$PID_FILE" 2>/dev/null)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  # 防止 pid 被复用：确认还真是 bag record 进程
  if [[ -r "/proc/$pid/cmdline" ]]; then
    tr '\0' ' ' < "/proc/$pid/cmdline" | grep -q "bag record" || return 1
  fi
  return 0
}

disk_avail_gb() {  # 目录不存在时向上找到第一个存在的祖先目录再查
  local d="$1"
  while [[ ! -d "$d" && "$d" != "/" ]]; do d="$(dirname "$d")"; done
  df -BG --output=avail "$d" 2>/dev/null | tail -1 | tr -dc '0-9'
}

cmd_check() {
  ensure_ros
  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID，检查话题（需发布端已启动）…"
  local missing=0 t info pcount ptype
  for t in "${TOPICS[@]}"; do
    info="$(ros2 topic info "$t" 2>&1)"
    if [[ "$info" == *"Unknown topic"* || "$info" == *"not found"* ]]; then
      err "  ✗ $t  —— 不存在（发布端没起？话题名对不上？）"
      missing=1
      continue
    fi
    ptype="$(grep -m1 '^Type:' <<<"$info" | awk '{print $2}')"
    pcount="$(grep -m1 'Publisher count:' <<<"$info" | awk '{print $3}')"
    if [[ "${pcount:-0}" -eq 0 ]]; then
      warn "  △ $t  类型=$ptype  发布者=0（暂无发布端，录上也是空）"
      missing=1
    else
      ok "  ✓ $t  类型=$ptype  发布者=$pcount"
    fi
  done
  # QoS 是否 best_effort（reliable 发布端没问题，best_effort 发布端必须 best_effort 订阅）
  for t in "${TOPICS[@]}"; do
    ros2 topic info "$t" 2>/dev/null | grep -q "Unknown topic" && continue
    if ros2 topic info -v "$t" 2>/dev/null | grep -q "Reliability: BEST_EFFORT"; then
      log "  i $t 发布端为 BEST_EFFORT（脚本会用 best_effort 订阅，兼容）"
    fi
  done
  local av; av="$(disk_avail_gb "$BAG_DIR" || true)"
  log "磁盘可用：${av:-?} GB（$BAG_DIR 所在分区；Mid360 满速约 23 GB/h，按录像时长估算够不够）"
  [[ $missing -eq 0 ]] && ok "全部话题在线，可以 start" || { warn "有话题缺失，确认后再 start"; return 1; }
}

write_qos_yaml() {  # $1 = 目标文件，其余参数 = 话题名
  local target="$1"; shift
  {
    echo "# 由 record_field_data.sh 自动生成：对录制话题统一 best_effort 订阅。"
    echo "# best_effort 订阅可同时兼容 reliable / best_effort 发布端（反过来则一条都收不到）。"
    local t
    for t in "$@"; do
      cat <<EOF
$t:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
  deadline:
    sec: 0
    nsec: 0
  lifespan:
    sec: 0
    nsec: 0
  liveliness: automatic
  liveliness_lease_duration:
    sec: 0
    nsec: 0
  avoid_ros_namespace_conventions: false
EOF
    done
  } > "$target"
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
  name="usv_${ts}"
  [[ -n "$TAG" ]] && name="${name}_${TAG}"
  bag="$BAG_DIR/$name"
  [[ -e "$bag" ]] && { err "输出目录已存在：$bag"; exit 1; }
  # 注意：不要预先创建 $bag —— ros2 bag record -o 要求目录不存在，由它自己创建

  local av; av="$(disk_avail_gb "$BAG_DIR" || true)"
  if [[ -n "${av:-}" && "$av" -lt 50 ]]; then
    warn "磁盘仅剩 ${av} GB —— Mid360 满速约 23 GB/h，录几小时会不够，检查 BAG_DIR 是否在大盘上！"
  fi

  local qos="$BAG_DIR/qos_override.yaml"
  write_qos_yaml "$qos" "${TOPICS[@]}"

  # 拼装录制命令
  local cmd
  cmd=(ros2 bag record -s sqlite3 -o "$bag" --qos-profile-overrides-path "$qos")
  [[ "$SPLIT_SEC" -gt 0 ]] && cmd+=(-d "$SPLIT_SEC")
  if [[ "$COMPRESS" == "1" ]]; then
    cmd+=(--compression-mode file --compression-format zstd)
  fi
  local t
  for t in "${TOPICS[@]}"; do cmd+=("$t"); done
  if [[ -n "${EXTRA_TOPICS:-}" ]]; then
    for t in $EXTRA_TOPICS; do cmd+=("$t"); done
  fi

  log "开始录制：$bag"
  log "话题：${TOPICS[*]} ${EXTRA_TOPICS:-} | 分片=${SPLIT_SEC}s | 压缩=$COMPRESS | DOMAIN_ID=$ROS_DOMAIN_ID"

  # setsid 脱离终端（SSH 断开不停录）；$! 即录制进程 pid（同时是进程组长）
  # 日志先放根目录（bag 目录由 recorder 自建），stop 时再归档进 bag
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
  # 此时 recorder 已建好 $bag 目录，把 qos 文件归档进去
  cp -f "$qos" "$bag/qos.yaml" 2>/dev/null || true
  ok "录制中（pid $pid）。停止：$0 stop   查看状态：$0 status"
  warn "开场提示：start 后等传感器出数再走动作，stop 结束时会打印各话题消息数（0 条=没录到）。"
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
      ls -lh "$bag" 2>/dev/null | awk '/\.db3|\.zstd|\.mcap/ {print "    "$9"  ("$5")"}'
    }
    log "磁盘可用：$(disk_avail_gb "$BAG_DIR" || echo '?') GB"
    log "日志尾部："
    tail -n 3 "$bag.record.log" 2>/dev/null | sed 's/^/    /'
  else
    log "当前没有录制任务。"
    local last
    last="$(ls -1dt "$BAG_DIR"/usv_* 2>/dev/null | head -1)"
    [[ -n "$last" ]] && log "最近一次录制：$last（$(du -sh "$last" 2>/dev/null | awk '{print $1}')）"
  fi
  return 0
}

fmt_dur() {  # 秒 → 1h02m03s
  local s=$1 h m
  h=$((s / 3600)); m=$(((s % 3600) / 60)); s=$((s % 60))
  ((h > 0)) && printf '%dh%02dm%02ds' "$h" "$m" "$s" || printf '%dm%02ds' "$m" "$s"
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

  log "正在停止录制（pid $pid）… 发 SIGINT 优雅收尾，写完 metadata.yaml 才算停干净"
  kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null

  local i
  for i in $(seq 1 30); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.5
  done
  if kill -0 "$pid" 2>/dev/null; then
    warn "3 秒内未退出，再等 5 秒…"
    kill -INT "$pid" 2>/dev/null
    for i in $(seq 1 10); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.5
    done
  fi
  if kill -0 "$pid" 2>/dev/null; then
    err "进程仍未退出（pid $pid）。为保护数据未强杀，请手动确认：kill -TERM $pid"
    exit 1
  fi

  rm -f "$PID_FILE" "$INFO_FILE"
  local dur=0
  [[ -n "${start_epoch:-}" ]] && dur=$(( $(date +%s) - start_epoch ))
  ok "已停止。时长 $(fmt_dur "$dur")，目录：$bag"

  # —— 收尾自检：从 metadata.yaml 统计各话题消息数，0 条 / 从未出现的当场揪出来 ——
  if [[ -f "$bag/metadata.yaml" ]]; then
    python3 - "$bag/metadata.yaml" "${TOPICS[@]}" <<'PY' || warn "（无法解析 metadata.yaml，可手动打开确认）"
import sys, subprocess
try:
    import yaml
except ImportError:
    print("  （AGX 上没有 pyyaml，跳过统计；用 ros2 bag info <目录> 查看）")
    sys.exit(0)
meta = yaml.safe_load(open(sys.argv[1]))["rosbag2_bagfile_information"]
want = list(sys.argv[2:])
rows = {r["topic_metadata"]["name"]: (r["topic_metadata"]["type"], r.get("message_count", 0))
        for r in (meta.get("topics_with_message_count") or [])}
print("  各话题消息数：")
for t in want:
    if t not in rows:
        print(f"    {t:<28} {'-':<35} {'从未出现':>8}  ← 发布端全程没起来！")
        continue
    ptype, n = rows[t]
    flag = "  ← 0 条，没录到！" if n == 0 else ""
    print(f"    {t:<28} {ptype:<35} {n:>10}{flag}")
for t, (ptype, n) in rows.items():
    if t not in want:
        print(f"    {t:<28} {ptype:<35} {n:>10}（额外录到的）")
files = meta.get("relative_file_paths") or []
print(f"  分片数：{len(files)}，总大小：", end="", flush=True)
subprocess.run(["du", "-sh", sys.argv[1].rsplit("/", 1)[0]])
PY
  else
    warn "未找到 metadata.yaml —— 可能是上次异常退出，试试: ros2 bag info \"$bag\""
  fi
  log "拷回开发机：scp -r nx@<AGX_IP>:\"$bag\" ."
}

usage() {
  cat <<'EOF'
现场数据录制：点云 + IMU + RTK
  录制话题（改脚本顶部 TOPICS 数组）：
    /livox/lidar  sensor_msgs/msg/PointCloud2
    /livox/imu    sensor_msgs/msg/Imu
    /rtk/odom     nav_msgs/msg/Odometry   ← RTK 暂定标准里程计，话题名定了改 TOPICS

用法:
  ./record_field_data.sh check          # 上场前检查话题是否在线（不录制）
  ./record_field_data.sh start [标签]   # 开始录制（后台运行，断 SSH 不停）
  ./record_field_data.sh status         # 是否在录 / 存到哪 / 磁盘剩余
  ./record_field_data.sh stop           # 优雅停止，打印各话题消息数（0 条=没录到）

配置（在脚本顶部「配置区」直接改，不用环境变量）:
  BAG_DIR="$HOME/usv_bags"  存储根目录（建议指到 NVMe/SSD）
  SPLIT_SEC=1800            单分片时长(秒)，0=不分片
  COMPRESS=0                置 1 开 zstd 压缩（吃 CPU，慎用）
  EXTRA_TOPICS=""           追加话题，如 "/tf"
  DOMAIN_ID=5               ROS_DOMAIN_ID，与车端一致

示例:
  ./record_field_data.sh start dock_run1
  ./record_field_data.sh stop
  scp -r nx@<AGX_IP>:~/usv_bags/usv_20260831_101500_dock_run1 .
EOF
}

case "${1:-}" in
  check)  cmd_check ;;
  start)  cmd_start ;;
  status) cmd_status ;;
  stop)   cmd_stop ;;
  *)      usage; exit 1 ;;
esac
