#!/usr/bin/env bash
# 停止残留的 Gazebo Garden / gz sim（开新仿真前执行）
set -euo pipefail

stop_pattern() {
  local pattern="$1"
  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    echo "[stop_simulation] pkill: ${pattern}"
    pkill -f "${pattern}" 2>/dev/null || true
  fi
}

echo "[stop_simulation] 优先在终端1 Ctrl+C 停 launch；本脚本清理残留 gz 进程"

stop_pattern "gz sim"
stop_pattern "ign gazebo"
stop_pattern "ruby.*gz sim"

sleep 1

# 仍存活则强杀
if pgrep -f "gz sim" >/dev/null 2>&1; then
  echo "[stop_simulation] SIGKILL 残留 gz sim"
  pkill -9 -f "gz sim" 2>/dev/null || true
fi

if pgrep -f "gz sim" >/dev/null 2>&1; then
  echo "[stop_simulation] 警告：仍有 gz 相关进程："
  pgrep -af "gz sim" || true
  exit 1
fi

echo "[stop_simulation] 完成，可重新 ros2 launch workspace_gz simulation.launch.py"
