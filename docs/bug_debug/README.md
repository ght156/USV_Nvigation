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
| 2026-10-21 | [兴飞新 NX 全栈联调：预泊点覆盖与归港压紧超时](2026-10-21_兴飞新NX全栈联调-预泊点覆盖与归港压紧超时.md) | 地图工具 DDS、状态机旧预泊点覆盖、平台单点覆盖、mission_bridge 崩溃后 service unavailable、`FINAL_PUSH_TIMEOUT` 均已记录；归港已改为默认只用船端 bay2 预泊点；mission_bridge 崩溃修复和 DI0/压紧时长仍待处理 |
| 2026-10-19 | [兴飞后视相机缺失与出港未确认成功绕过](2026-10-19_兴飞后视相机缺失与出港未确认成功绕过.md) | 后视 USB `0bda:3035` 缺失、FRONT/REAR 相机故障**已确证**；`undock_unconfirmed_as_success=true` 临时放行两次出港（`UNDOCK_SUCCEEDED` ACK）；换相机/标定后**必须改回 false** |
| 2026-10-19 | [巡航到点容差 2 m 与 map→odom TF 卡滞导致任务 RUNNING](2026-10-19_巡航到点容差2m与map-odom_TF卡滞导致任务RUNNING.md) | 到点容差已在线+落盘改 2.0；mission_bridge 误读澄丰 YAML 已修；5/5 航点因 map→odom 时间戳滞后 0.37 s 被 controller abort，waypoint_follower 重试 + progress checker 被 reset，任务停在 RUNNING；**未取消未重启，待明天修 TF 时间戳/容差后重启**；控制频率保持 20 Hz（不降频），CPU 根因排查中 |
| 2026-10-16 | [澄丰局部代价图三问：不跟手 / 标记全丢 / 岸边左右反](2026-10-16_澄丰局部代价图-不跟手_标记丢失_倒装补偿三问.md) | ①「不跟手」= 网页把代价图节流成 1 Hz，**已修**（250 ms，TF 100 ms）；②「标记全丢」= 试关驱动外参旋转的副作用，**已回滚**（保持 `extrinsic_enable=false`）；③「左右反」现场确认当前组合正确，三处 180° 一处都别删；附固化参数表与 bag `~/usv_bags/nav_check_1016_1823` |
| 2026-10-15 | [出港背光全掩码写清夹爪位，船拖着坞跑](2026-10-15_出港背光全掩码写清夹爪位致拖坞.md) | 根因**已确证**（set_do 全掩码写 0x0002 清掉夹爪 bit0）；出港背光**已回滚**，`/dock/in_dock` 在坞查询保留；出港点灯（0x0003 写法）待嵌软确认线圈通电与 60s 全清后再议 |
| 2026-10-12 | [服务类调用全部超时：客户端 RMW 与船端节点不一致（FastDDS ↔ CycloneDDS）](2026-10-12_ros2服务类调用全超时-客户端RMW与船端节点不一致.md) | 根因**已确证**（非交互 ssh 不读 `~/.bashrc` → 客户端落 FastDDS，话题通、服务全超时）；节点/配置**未动**，规范=服务类操作前显式 export RMW；建议加 `~/rosenv.sh` 待定 |
| 2026-10-12 | [双 RS485 实例 + launch 被 Ctrl-C 打断：取证方法与残留坑](2026-10-12_双RS485实例与launch被Ctrl-C打断取证.md) | 已确证（终端前台 Ctrl-C + 手动实例不被 pid 文件判活/不被 stop 清）；已清成单实例重启；脚本 `stop` 补 pkill 兜底待定 |
| 2026-10-08 | [压紧段 6s 窗口不够：明明已进坞仍判 FINAL_PUSH_TIMEOUT 倒车驶出](2026-10-08_压紧段6s不够导致进坞后倒出.md) | 根因**已确证**（DI1 到 5.5s 才闭合、DI0 更晚）；已把 `final_push_time_sec` 6.0→15.0（含在线下发），**待现场验证**；DI0 含义/掩码待嵌软确认 |
| 2026-10-08 | [RS485 驱动回归：ControlDO 不点亮背光板 + 60s 全通道清零](2026-10-08_RS485驱动ControlDO漏点背光板与60s全清回归.md) | 已确证（今日 14:49 换驱动版本引入）；**未改动代码**，提单嵌软按 §6 修 |
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
# ⚠️ 服务类操作（param set/get、service call、action）前先让客户端 RMW 与节点一致，
#    否则服务全部超时（ros2 CLI 会静默挂住、rclpy 报 NO_RESULT）：
source ~/logs/xingfei_stack/.cyclone_env
export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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
