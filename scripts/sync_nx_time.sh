#!/usr/bin/env bash
# --------------------------------------------------------------------------------------- #
# sync_nx_time.sh — 把船端 NX 的系统时钟对齐到本机（主机）
#
# 背景：NX 无 RTC 电池，断电时时钟冻结，开机从关机时刻续走（误差=断电时长）；
#       且船上网络的 NTP(UDP 123) 被网关拦截，timesyncd 从未成功对时。
#       ROS 跨机 TF/消息过滤对时钟敏感，误差超过秒级导航就会报 extrapolation 错误。
#
# 前提（一次性配置，需在 NX 上手动执行一次，输 sudo 密码）：
#   ssh nx 'echo "jetson ALL=(ALL) NOPASSWD: /usr/bin/date, /usr/bin/timedatectl" | sudo tee /etc/sudoers.d/nx_timesync && sudo chmod 440 /etc/sudoers.d/nx_timesync'
#
# 用法：
#   ./scripts/sync_nx_time.sh            # 对齐到本机时间并校验
#   ./scripts/sync_nx_time.sh check      # 只查看两端钟差，不修改
#
# 长期方案（比本脚本更省心）：主机装 chrony 当 NTP 服务器（allow 192.168.2.0/24），
# NX 的 /etc/systemd/timesyncd.conf 改成 NTP=192.168.2.100 —— 一劳永逸，断外网也不怕。
# --------------------------------------------------------------------------------------- #
set -u

NX_HOST="${NX_HOST:-nx}"
MODE="${1:-sync}"

# 测 ssh 往返时延（毫秒），用于补偿下发时刻的网络延迟
measure_rtt_ms() {
  local t0 t1
  t0=$(date +%s%N)
  ssh -o ConnectTimeout=5 "$NX_HOST" true 2>/dev/null || return 1
  t1=$(date +%s%N)
  echo $(( (t1 - t0) / 1000000 ))
}

# 读两端钟差（船 - 主机，毫秒）
read_offset_ms() {
  local t0 t1 remote_epoch remote_frac
  t0=$(date +%s%N)
  read -r remote_epoch remote_frac < <(ssh -o ConnectTimeout=5 "$NX_HOST" 'date +"%s %N"') || return 1
  t1=$(date +%s%N)
  local local_mid=$(( (t0 + t1) / 2 ))
  local remote_ns=$(( 10#$remote_epoch * 1000000000 + 10#$remote_frac ))
  echo $(( (remote_ns - local_mid) / 1000000 ))
}

if ! ssh -o ConnectTimeout=5 "$NX_HOST" true 2>/dev/null; then
  echo "[sync_nx_time] 连不上 $NX_HOST，检查网络/ssh 配置"; exit 1
fi

OFFSET_BEFORE=$(read_offset_ms)
echo "[sync_nx_time] 对齐前钟差（船-主机）: ${OFFSET_BEFORE} ms"

if [ "$MODE" = "check" ]; then
  exit 0
fi

# 免密 sudo 探测（探测命令本身必须在 NOPASSWD 白名单里）
if ! ssh "$NX_HOST" 'sudo -n /usr/bin/date >/dev/null 2>&1'; then
  echo "[sync_nx_time] NX 上 sudo 需要密码，无法自动对时。"
  echo "请先在 NX 上执行一次性配置（输一次密码）："
  echo "  ssh nx 'echo \"jetson ALL=(ALL) NOPASSWD: /usr/bin/date, /usr/bin/timedatectl\" | sudo tee /etc/sudoers.d/nx_timesync && sudo chmod 440 /etc/sudoers.d/nx_timesync'"
  exit 2
fi

# 最多迭代 5 次收敛：把上一轮实测残差反馈进下一轮补偿（ssh+sudo 的执行延迟是主要误差源）
CORR_MS=0
for i in 1 2 3 4 5; do
  RTT=$(measure_rtt_ms)
  SET_MS=$(( $(date +%s%3N) + RTT / 2 + CORR_MS ))
  ssh "$NX_HOST" "sudo -n /usr/bin/date -s @$(( SET_MS / 1000 )).$(printf '%03d' $(( SET_MS % 1000 ))) >/dev/null" || { echo "[sync_nx_time] date -s 失败"; exit 3; }
  sleep 1
  OFFSET=$(read_offset_ms)
  echo "[sync_nx_time] 第 ${i} 次对齐后残差: ${OFFSET} ms"
  if [ "${OFFSET#-}" -lt 200 ]; then
    echo "[sync_nx_time] 完成，钟差已 < 0.2s（ROS 跨机无压力）。"
    exit 0
  fi
  CORR_MS=$(( CORR_MS - OFFSET ))
done

echo "[sync_nx_time] 警告：5 次迭代后残差仍有 ${OFFSET} ms，请检查 NX 是否有其他对时服务在抢时钟。"
exit 4
