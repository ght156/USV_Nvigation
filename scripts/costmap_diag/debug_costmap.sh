#!/usr/bin/env bash
# debug_costmap.sh — 避障调试独立代价图一键启停（本机执行，驱动船端 hkzl）
#
# 干什么：在船端起一个【与主导航并存】的调试代价图（livox_cloud_filter 遮蔽 +
#         独立 nav2_costmap_2d，话题 /debug_costmap/*），不动主栈任何进程。
#         网页工具 ros_map_tool_v3_nav2_live.html 把"代价图话题"输入框改成
#         /debug_costmap/costmap 即可看图（footprint 框 /debug_costmap/published_footprint）。
#
# 用法：
#   scripts/costmap_diag/debug_costmap.sh start    # 船端 tmux 后台启动
#   scripts/costmap_diag/debug_costmap.sh status   # 看进程 + 话题有没有在发
#   scripts/costmap_diag/debug_costmap.sh stop     # 停掉（只杀调试会话）
#
# 前提：主栈在跑（定位 TF、MAVROS），否则代价图拿不到位姿不发帧。
# 调参：改船端 .../share/workspace_nav/config/debug_costmap_params.yaml 后
#       stop+start 生效；mark_threshold / max_obstacle_height 支持在线 set。
set -uo pipefail

HOST="${HOST:-hkzl}"
SESSION="dbg_costmap"
INSTALL="${INSTALL:-/home/hkzl/usv-test/install/arm}"

remote() { timeout 25 ssh "$HOST" "$@"; }

case "${1:-}" in
  start)
    remote "tmux has-session -t $SESSION 2>/dev/null && { echo '已在运行（先 stop 再 start）'; exit 1; }
            tmux new-session -d -s $SESSION \
              'source /opt/ros/humble/setup.bash; source $INSTALL/local_setup.bash; \
               export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
               ros2 launch workspace_nav debug_costmap.launch.py 2>&1 | tee /tmp/debug_costmap.log'
            sleep 6
            tmux capture-pane -p -t $SESSION | tail -5"
    ;;
  status)
    remote "tmux has-session -t $SESSION 2>/dev/null && echo '会话: 在跑' || echo '会话: 没起'
            source /opt/ros/humble/setup.bash >/dev/null 2>&1
            export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            timeout 6 ros2 topic hz /debug_costmap/costmap 2>&1 | head -2
            pgrep -af 'livox_cloud_filter|nav2_costmap_2d' | grep -v grep | head -4"
    ;;
  stop)
    remote "tmux kill-session -t $SESSION 2>/dev/null && echo '已停' || echo '本来就没起'"
    ;;
  log)
    remote "tail -30 /tmp/debug_costmap.log"
    ;;
  *)
    echo "用法: $0 start|stop|status|log   (HOST=hkzl 可覆盖)"; exit 1 ;;
esac
