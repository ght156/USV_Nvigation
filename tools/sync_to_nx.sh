#!/bin/bash
# sync_to_nx.sh —— 增量同步项目到 NX，可选编译，不受 IP 变动影响
#
# 用法:
#   ./sync_to_nx.sh                # 仅同步
#   ./sync_to_nx.sh --build        # 同步 + 编译
#   ./sync_to_nx.sh --discover     # 扫描局域网找 NX
#
# 环境变量（可选覆盖）:
#   NX_HOST  - NX 地址，支持 IP 或 .local 主机名（默认 nvidia-desktop.local）
#             手机热点场景: export NX_HOST=192.168.43.x
#             网线直连场景: export NX_HOST=192.168.1.2

set -e

NX_USER="${NX_USER:-nvidia}"
NX_HOST="${NX_HOST:-nvidia-desktop.local}"
NX_WS="/home/${NX_USER}/USV_NAV"
BUILD=false
DISCOVER=false

for arg; do
  case "$arg" in
    --build) BUILD=true ;;
    --discover) DISCOVER=true ;;
    *) echo "用法: $0 [--build] [--discover]"; exit 1 ;;
  esac
done

if $DISCOVER; then
  echo "=== 扫描局域网寻找 NX（avahi/mDNS + 常见 IP）==="
  echo ""
  # mDNS
  for name in nvidia-desktop tegra-ubuntu nx nvidia-jetson; do
    ip=$(getent hosts "${name}.local" 2>/dev/null | awk '{print $1}')
    if [ -n "$ip" ]; then
      echo "✓ mDNS 发现: ${name}.local → $ip"
    fi
  done
  # 常见 IP 段
  for subnet in 192.168.1 192.168.2 192.168.43 192.168.55 10.42.0; do
    ip="${subnet}.2"
    if ping -c1 -W1 "$ip" >/dev/null 2>&1; then
      echo "✓ ping 在线: $ip"
    fi
  done
  echo ""
  echo "使用时 export NX_HOST=<上面任一地址>"
  exit 0
fi

echo "=== 同步到 ${NX_USER}@${NX_HOST}:${NX_WS} ==="

rsync -avh \
  --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='.git/' --exclude='__pycache__/' --exclude='*.pyc' \
  ~/USV_NAV/src/ ~/USV_NAV/map/ ~/USV_NAV/tools/ ~/USV_NAV/docs/ \
  ${NX_USER}@${NX_HOST}:${NX_WS}/

echo "=== 同步完成 ==="

if $BUILD; then
  echo "=== NX 上编译（ARM，预留 3-5 分钟）==="
  ssh ${NX_USER}@${NX_HOST} \
    "source /opt/ros/humble/setup.bash && cd ${NX_WS} && colcon build --symlink-install --executor sequential"
  echo "=== 编译完成 ==="
fi
