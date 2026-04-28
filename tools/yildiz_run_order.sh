#!/usr/bin/env bash
# YILDIZ 联调典型顺序（多终端分别运行）
cat <<'EOF'
按顺序在独立终端中执行（同一 ROS_DOMAIN_ID，且先 source 环境）:

  source /opt/ros/humble/setup.bash
  source ~/yildiz_ws/install/setup.bash

1) 仿真
   ros2 launch workspace_gz simulation.launch.py

2) 融合定位
   ros2 launch workspace_ros localization.launch.py

3) Nav2
   ros2 launch workspace_nav nav2.launch.py

4) 地面站（另开终端，需 Node + 本仓库 venv + ROS）
   cd ~/GROUND-CONTROL-STATION
   source /opt/ros/humble/setup.bash
   source venv/bin/activate
   npm run start:all
   在界面中规划航点/选择目标色；保存后可在本机执行:
   bash ~/yildiz_ws/tools/sync_gcs_json_to_yildiz.sh
   然后: cd ~/yildiz_ws && colcon build --packages-select workspace_nav --merge-install

5) 话题桥接
   ros2 run workspace_ros converter

6) 等 GCS 在 /color_code 上发布 green|red|black 后
   ros2 run workspace_ros target_buoy

7) 航点转换与状态（需有效 GPS/航点数据）
   ros2 run workspace_nav waypoint_transform
   ros2 run workspace_nav waypoint_with_state
EOF
