#!/usr/bin/env bash
# 无头仿真环境一键启动（gz server 模式，无 GUI）。
# 用途：自动化联调 usv_docking V2；gz 偶发"干净退出"后直接重跑本脚本即可。
# 启动组件：gz headless + localization(EKF) + apriltag + converter
# 之后手动：ros2 launch usv_docking docking_v2.launch.py use_sim_time:=true test_only:=false
cd /home/ght/wuxihik_navigation
source install/setup.bash
export ROS_LOG_DIR=/home/ght/wuxihik_navigation/log/ros_smoke

nohup ros2 launch workspace_gz simulation_headless.launch.py > /tmp/gz_headless.log 2>&1 &
echo "gz headless 启动中(20s)..."
sleep 20

nohup ros2 launch workspace_ros localization.launch.py use_sim_time:=true > /tmp/loc.log 2>&1 &
sleep 6
nohup ros2 launch apriltag_localization apriltag_localization.launch.py rviz:=false > /tmp/apriltag.log 2>&1 &
sleep 6
nohup ros2 run workspace_ros converter > /tmp/converter.log 2>&1 &
sleep 8

echo "== 健康检查 =="
timeout 4 ros2 topic hz /clock 2>&1 | head -1
timeout 4 ros2 topic hz /odometry/filtered 2>&1 | head -1
timeout 4 ros2 topic hz /apriltag_node/detections 2>&1 | head -1
echo "完成。传送船位示例："
echo "  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: \"roboboat\", position: {x: 4.0, y: -1.5, z: 0.0}, orientation: {x: 0, y: 0, z: 1, w: 0}'"
