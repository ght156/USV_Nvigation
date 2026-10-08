#!/usr/bin/env bash
# 船上跑压缩转发：source ROS + 工作区后启动节点。
# 默认参数按 cleaning_boat.launch.py 的后端相机写死，可直接改参数再跑。

set -e

source /opt/ros/humble/setup.bash
[ -f "$HOME/jzw_ws/install/setup.bash" ] && source "$HOME/jzw_ws/install/setup.bash"

# 船上感知栈跑在 domain 5（见 cleaning_boat.launch 启动环境），须同域才能看到相机流
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/compressed_republish.py" "$@"
