#!/usr/bin/env bash
# 把 ros_map_tool（页面 + 船端启动脚本 + 船端下发客户端 + 船上当前底图）部署到澄丰 NX。
#
#   bash scripts/map_dispatch/deploy_to_nx.sh                 # 部署到 nx:~/ros_map_tool/
#   bash scripts/map_dispatch/deploy_to_nx.sh --host nx --dest '~/ros_map_tool'
#   bash scripts/map_dispatch/deploy_to_nx.sh --html /home/ght/ros_map_tool-main/ros_map_tool_v3_nav2_live.html
#
# 与 deploy_to_map_tool.sh（兴飞 hkzl）的区别：兴飞 NX 没装 rosbridge_server，走 ssh 下发；
# 澄丰 NX 自己跑着 rosbridge（webpath.sh 那一套），所以这里把页面也放到船上，
# 局域网里直接 http://<NX_IP>:8001/ 打开就能连本船 Nav2。
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST="nx"
DEST='~/ros_map_tool'
HTML=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --host) HOST="$2"; shift 2 ;;
        --dest) DEST="$2"; shift 2 ;;
        --html) HTML="$2"; shift 2 ;;
        *) echo "未知参数: $1" >&2; exit 2 ;;
    esac
done

# 页面来源：命令行 > 本机常驻工具目录（最新） > 仓库副本
PAGE_SRC="$HTML"
if [[ -z "$PAGE_SRC" ]]; then
    for c in "/home/ght/ros_map_tool-main/ros_map_tool_v3_nav2_live.html" "$SRC/ros_map_tool_v3_nav2_live.html"; do
        [[ -f "$c" ]] && { PAGE_SRC="$c"; break; }
    done
fi
[[ -f "$PAGE_SRC" ]] || { echo "找不到页面文件（用 --html 指定）" >&2; exit 1; }

echo "== 页面来源: $PAGE_SRC"
echo "== 目标: $HOST:$DEST"

LOCAL_PAGE_MD5="$(md5sum "$PAGE_SRC" | awk '{print $1}')"

ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" "mkdir -p $DEST/map $DEST/logs"

# 覆盖前备份旧页面
ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" "
    cd $DEST || exit 1
    if [ -f ros_map_tool_v3_nav2_live.html ] && ! [ \"\$(md5sum ros_map_tool_v3_nav2_live.html | awk '{print \$1}')\" = '$LOCAL_PAGE_MD5' ]; then
        cp -p ros_map_tool_v3_nav2_live.html ros_map_tool_v3_nav2_live.html.bak-\$(date +%Y%m%d-%H%M%S)
        echo '已备份旧页面'
    fi"

scp -q "$PAGE_SRC" "$HOST:$DEST/ros_map_tool_v3_nav2_live.html"
scp -q "$SRC/remote_nav_cli.py" "$HOST:$DEST/remote_nav_cli.py"
scp -q "$SRC/boat_ros_map_tool.sh" "$HOST:$DEST/ros_map_tool.sh"

# 船端收尾：可执行位 + 把船上当前底图（map.yaml/map.pgm）放到 ~/ros_map_tool/map/
ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" "
    set -u
    cd $DEST
    chmod +x ros_map_tool.sh remote_nav_cli.py
    MAP_DIR=\$(find \"\$HOME/usv_nav_ws/src\" -maxdepth 5 -type d -path '*workspace_nav/map' 2>/dev/null | head -1)
    CFG_DIR=\$(find \"\$HOME/usv_nav_ws/src\" -maxdepth 5 -type d -path '*workspace_nav/config' 2>/dev/null | head -1)
    if [ -n \"\$MAP_DIR\" ] && [ -n \"\$CFG_DIR\" ]; then
        cp -f \"\$CFG_DIR/map.yaml\" map/ 2>/dev/null && echo '底图 map.yaml 已就位'
        cp -f \"\$MAP_DIR/map.pgm\" map/ 2>/dev/null && echo '底图 map.pgm 已就位'
        [ -f \"\$CFG_DIR/map_back.yaml\" ] && cp -f \"\$CFG_DIR/map_back.yaml\" map/ 2>/dev/null || true
    else
        echo '⚠ 没找到船端 workspace_nav/map —— 底图请手工拷到 $DEST/map/'
    fi
    echo '--- 船端文件 ---'
    ls -l $DEST | head -20
    echo '--- 底图 ---'
    ls -l $DEST/map 2>/dev/null | head
    echo -n '页面 md5: '; md5sum ros_map_tool_v3_nav2_live.html | awk '{print \$1}'
"

echo "== 本地页面 md5: $LOCAL_PAGE_MD5"
echo
echo "下一步（船上或经 ssh）："
echo "  1) 启动服务:  ssh $HOST '$DEST/ros_map_tool.sh start'"
echo "  2) 浏览器打开: 局域网 http://<NX_IP>:8001/  ；跨网段用隧道"
echo "     ssh -N -L 8001:127.0.0.1:8001 -L 9090:127.0.0.1:9090 $HOST   → http://127.0.0.1:8001/"
echo "  3) 页面「加载 map.yaml + map.pgm」选 $DEST/map/ 下那份；点「🔌 连接 ROS」看实时 Nav2 数据"
