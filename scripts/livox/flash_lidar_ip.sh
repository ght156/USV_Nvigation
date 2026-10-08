#!/usr/bin/env bash
# 一键改雷达 IP（在船端 NX 上运行；需要 sudo 和 docker）。
#
#   bash flash_lidar_ip.sh <网口> [目标IP] [临时hostIP]
#
# 例：雷达当前 IP 在 1 网段（192.168.1.162）、期望的 host 是 192.168.1.10，
#     把它改到 192.168.2.191（NX 自身保持 192.168.2.101 不动）：
#       bash flash_lidar_ip.sh eth1 192.168.2.191 192.168.1.10
#
# 说明：
#  - 只在指定网口上临时加一个地址，跑完自动删掉，不动 NX 的 192.168.2.x 配置。
#  - 工具在 set-ip 之后会显式再发一次重启命令：雷达不会自己重启（2026-09-18 踩过的坑）。
#  - 现场只接一台雷达；发现多台时工具会拒绝执行。
set -euo pipefail

IFACE="${1:?用法: $0 <网口> [目标IP] [临时hostIP]}"
TARGET="${2:-192.168.2.191}"
HOSTIP="${3:-192.168.1.10}"
WS="${WS:-$HOME/ws_livox}"
IMAGE="${IMAGE:-usv-runtime:jp6-cu126-humble}"
TOOL_DIR="$WS/src/livox_ros_driver2/tools"

if [[ ! -x "$TOOL_DIR/set_lidar_ip_arm" ]]; then
  echo "缺少 ARM64 工具 $TOOL_DIR/set_lidar_ip_arm；编译方法见 scripts/livox/README.md" >&2
  exit 1
fi

echo "[1/3] 在 $IFACE 上临时配置 $HOSTIP/24"
sudo ip addr add "$HOSTIP/24" dev "$IFACE" 2>/dev/null || true
sudo ip link set "$IFACE" up
ip -br addr show "$IFACE"

echo "[2/3] 改雷达 IP -> $TARGET"
docker run --rm --network host -v "$WS:/ws" -w "$TOOL_DIR" \
  "$IMAGE" bash -lc "./set_lidar_ip_arm '$TARGET' --host-ip '$HOSTIP' --timeout 25"

echo "[3/3] 清理临时地址，等雷达重启（约 10~15s）"
sudo ip addr del "$HOSTIP/24" dev "$IFACE" 2>/dev/null || true
ip -br addr show "$IFACE"
for i in $(seq 1 12); do
  sleep 5
  if ping -c1 -W1 "$TARGET" >/dev/null 2>&1; then
    echo "OK：$TARGET 已上线（约 $((i * 5))s）"
    exit 0
  fi
  echo "  等待雷达重启… $((i * 5))s"
done
echo "超时：$TARGET 无响应；检查线缆/供电，或重跑本脚本" >&2
exit 1
