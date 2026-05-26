#!/bin/bash
# -------------------------------------------------------------------
# 增量推送项目到 NX (usv-nx.local) 并编译
# 用法: ./push_to_nx.sh
#
# 排除: map/ build/ install/ log/ docs/ .git/ .claude/ __pycache__/
#        *.tar.gz *.zip *.pyc test.txt workspace_gz/
#
# 编译: 仅 workspace_ros + workspace_nav（跳过 Gazebo 仿真包）
# -------------------------------------------------------------------
set -e

NX_HOST="nvidia@usv-nx.local"
NX_PATH="/home/nvidia/usv_sim"
LOCAL_PATH="/home/ght/wuxihik_navigation"

echo "=== [1/3] 增量同步源码到 NX ==="
rsync -avz --delete \
  --exclude='map/' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  --exclude='docs/' \
  --exclude='.git/' \
  --exclude='.claude/' \
  --exclude='__pycache__/' \
  --exclude='*.tar.gz' \
  --exclude='*.zip' \
  --exclude='*.pyc' \
  --exclude='test.txt' \
  --exclude='.pytest_cache/' \
  "$LOCAL_PATH/" \
  "$NX_HOST:$NX_PATH/"

echo ""
echo "=== [2/3] 在 NX 上编译 (workspace_ros + workspace_nav) ==="
ssh "$NX_HOST" "
  source /opt/ros/humble/setup.bash && \
  cd $NX_PATH && \
  colcon build --symlink-install --packages-select workspace_ros workspace_nav
"

echo ""
echo "=== [3/3] 完成 ==="
echo "在 NX 上使用前执行:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $NX_PATH/install/setup.bash"
