# Bug / 调试记录

现场问题统一记在这里，**一次问题一个文件**：`YYYY-MM-DD_<一句话标题>.md`。

固定结构（方便以后检索和交接）：

1. **基本信息**：日期、船/工作空间、会话时间段、数据来源（日志绝对路径）
2. **现象**：肉眼看到什么、什么时候出现、频次
3. **证据**：日志原文片段 + 时间线（不要只写结论）
4. **根因**：能确证的写"已确证"，推测的写"待验证"
5. **已做修改**：文件路径 + 关键改动
6. **待验证 / 下一步**：现场怎么复现、要录哪些数据

## 记录索引

| 日期 | 记录 | 状态 |
|---|---|---|
| 2026-10-07 | [Nav2 导航"一步不动 / 原地左右摆"：行为树缺 goal_checker + RPP 原地旋转阈值](2026-10-07_Nav2导航不动-行为树缺goal_checker与RPP原地旋转阈值.md) | 两个根因**已确证**；已用"目标带 BT + 在线改 RPP 三参数"绕过并跑通（去 35 m / 回 29 m）；**目标点/goal_checker 的正式方案与导航参数留给现场自调** |
| 2026-10-07 | [归港 REACQUIRE 盲驾把船带走 + 锚点 1m 乱跳 + 入口放宽与廊道碰撞](2026-10-07_归港REACQUIRE盲驾把船带走与锚点1m乱跳.md) | 盲驾路径与安全边界缺失**已确证**；单双码系统差待验证；本轮**未改动，先记录** |
| 2026-09-30 | [RS485 驱动 DO 定时关闭误伤背光与无线充电](2026-09-30_RS485驱动DO定时关闭误伤背光与无线充电.md) | 已确证；船端已修复重建（待同事合入），待夜间完整归港验证 |
| 2026-09-30 | [兴飞后视 AprilTag 与船坞中心不对称排查](2026-09-30_兴飞后视AprilTag与船坞中心不对称排查.md) | 链路自洽无 bug；已按现场要求重标定 tag 偏移（dock 中心对齐 base_link），待重启感知验证 + 实船归港验证 |
| 2026-09-23 | [SLAM 定位朝向镜像错误（gnss_unify_node yaw_enu 符号 bug）](2026-09-23_SLAM定位朝向错误-yaw_enu符号bug.md) | 已确证；算法组已修复发版，现场验证通过；导航侧 odom_topic launch 化 + 文件重命名 |
| 2026-09-21 | [RPP 误报碰撞（局部窗口过小，前瞻出图当致命障碍）](2026-09-21_RPP误报碰撞-局部代价地图窗口过小.md) | 已确证；参数已改（lookahead 16→13 / max 8→6.5），待现场验证 |
| 2026-09-17 | [航点卡顿与恢复行为失效](2026-09-17_航点卡顿与恢复行为失效.md) | 已定位；配置已改，待现场验证 |
| 2026-09-17 | [里程计/TF 与雷达时基相差 0.85 s](2026-09-17_TF与雷达时基偏差.md) | 已定位；**未修**，待定时间源修法 |

## 现场排查常用命令

船端（`ssh nx`，用户 `jetson`）：

```bash
# 本次会话的所有日志（ROS 运行日志按进程落在这里）
ls -lt ~/.ros/log/ | head -30

# 控制器 / 行为树 / 代价地图（同一个 component container 里）
grep -nE "Failed to make progress|detected collision|Reached the goal" \
  ~/.ros/log/component_container_isolated_*.log

# mission_bridge（逐点下发与结果）
grep -nE "Sending waypoint|reached successfully|MISSED" ~/.ros/log/python3_*.log

# 恢复行为是否真的执行了
grep -nE "Running spin|Running backup|Pose Goes Off Grid|Collision Ahead - Exiting" \
  ~/.ros/log/component_container_isolated_*.log

# 速度桥看门狗
grep -n "Velocity command timeout" ~/.ros/log/ardupilot_velocity_bridge_*.log
```

时间戳是 Unix 秒，可直接 `date -d @<ts>` 换算；`1789633163` = 2026-09-17 16:19:23。
