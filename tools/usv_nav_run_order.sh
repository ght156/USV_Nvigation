#!/usr/bin/env bash
# USV_NAV 联调典型顺序（多终端分别运行）
cat <<'EOF'
按顺序在独立终端中执行（同一 ROS_DOMAIN_ID，且先 source 环境）:

  source /opt/ros/humble/setup.bash
  source ~/usv_nav_ws/install/setup.bash

1) 融合定位
   ros2 launch workspace_ros localization.launch.py

2) Nav2 实船
   ros2 launch workspace_nav nav2_real_mavros.launch.py

3) 地面站（另开终端，需 Node + 本仓库 venv + ROS）
   cd ~/GROUND-CONTROL-STATION
   source /opt/ros/humble/setup.bash
   source venv/bin/activate
   npm run start:all
   在界面中规划航点/选择目标色；保存后可在本机执行:
   bash ~/usv_nav_ws/tools/sync_gcs_json_to_usv_nav.sh
   然后: cd ~/usv_nav_ws && colcon build --packages-select workspace_nav --merge-install

4) 话题桥接
   ros2 run workspace_ros converter

5) 等 GCS 在 /color_code 上发布 green|red|black 后
   ros2 run workspace_ros target_buoy

6) 航点转换与状态（需有效 GPS/航点数据）
   ros2 run workspace_nav waypoint_transform
   ros2 run workspace_nav waypoint_with_state
EOF
