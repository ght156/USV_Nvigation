#!/usr/bin/env bash
# 把「选中航点下发到船端」补丁部署到 ros_map_tool 目录。
#
#   bash scripts/map_dispatch/deploy_to_map_tool.sh                 # 默认部署到 /home/ght/ros_map_tool-main
#   bash scripts/map_dispatch/deploy_to_map_tool.sh <工具目录>       # 或指定目录
#   bash scripts/map_dispatch/deploy_to_map_tool.sh --boat          # 顺带更新船端副本 ~/usv-main/tools/ros_map_tool-main
#
# 覆盖 HTML 前会自动备份成 <名字>.bak-<时间戳>。
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOAT=0
if [[ "${1:-}" == "--boat" ]]; then
    BOAT=1
    shift
fi
DEST="${1:-/home/ght/ros_map_tool-main}"
STAMP="$(date +%Y%m%d-%H%M%S)"
HTML="ros_map_tool_v3_nav2_live.html"

if [[ ! -f "$SRC/$HTML" ]]; then
    echo "缺少 $SRC/$HTML" >&2
    exit 1
fi
if [[ ! -d "$DEST" ]]; then
    echo "工具目录不存在: $DEST" >&2
    exit 1
fi

echo "== 部署到 $DEST =="
if [[ -f "$DEST/$HTML" ]] && ! cmp -s "$DEST/$HTML" "$SRC/$HTML"; then
    cp -p "$DEST/$HTML" "$DEST/$HTML.bak-$STAMP"
    echo "已备份原页面 → $HTML.bak-$STAMP"
fi
install -m 644 "$SRC/$HTML" "$DEST/$HTML"
install -m 755 "$SRC/waypoint_dispatch_bridge.py" "$DEST/waypoint_dispatch_bridge.py"
install -m 755 "$SRC/remote_nav_cli.py" "$DEST/remote_nav_cli.py"
echo "已部署: $HTML / waypoint_dispatch_bridge.py / remote_nav_cli.py"

if [[ "$BOAT" == "1" ]]; then
    echo "== 同步到船端 ~/usv-main/tools/ros_map_tool-main =="
    scp -q "$SRC/$HTML" "$SRC/waypoint_dispatch_bridge.py" "$SRC/remote_nav_cli.py" \
        hkzl:~/usv-main/tools/ros_map_tool-main/
    echo "船端副本已更新"
fi

echo
echo "下一步："
echo "  1) 本机启动下发桥： python3 $SRC/waypoint_dispatch_bridge.py"
echo "  2) 浏览器打开：     http://127.0.0.1:8770/    （或双击 $DEST/$HTML）"
echo "  3) 页面切「🚩 航点编辑器」→ 打点 → 📤 下发选中航点"
