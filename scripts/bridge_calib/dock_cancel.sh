#!/usr/bin/env bash
# 归港两层中止：dock_mission 任务层 + docking FSM 层（单发一条命令全停）
# 用法: ~/bridge_calib/dock_cancel.sh
set -u
export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/logs/xingfei_stack/.cyclone_env 2>/dev/null || true
source /opt/ros/humble/setup.bash
source ~/usv-main/install/arm/setup.bash

# 任务层（服务）
timeout 10 ros2 service call /dock/mission/cancel std_srvs/srv/Trigger
# FSM 层（话题，Empty 类型；注意不是 Bool——类型不匹配会静默丢弃，2026-10-05 踩过）
timeout 10 ros2 topic pub --once /dock/cancel std_msgs/msg/Empty "{}"
