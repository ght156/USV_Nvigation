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
| 2026-11-20 | [归港 C++ 资源对比与出港执行器饥饿修复](2026-11-20_归港C++资源对比与出港执行器饥饿修复.md) | 五节点合计 CPU `120.50%→21.40%`、RSS `348.9→199.1 MiB`；出港失败已确证为 `wait_vision` 服务阻塞唯一 spin 线程导致订阅缓存饥饿，新增真实节点回归测试并改为独立线程 `executor.spin()`；24 项测试通过，实船一次出港和一次归港成功；长期稳定性与修复后资源复测待补 |
| 2026-10-29 | [船不动：局部代价图 current 卡死 + map_server 启动竞态](2026-10-29_船不动-局部代价图current卡死与map_server启动竞态.md) | ①船不动**已确证**：清局部代价图 → `InflationLayer::reset()`（Humble 版只置 `current_=false`、不置 `need_reinflation_`，main 分支已修）→ 本地图无启用的观测层时 `LayeredCostmap::updateMap()` 边界退化提前 return → `updateCosts()` 永不执行 → `current_` 卡 false → controller 卡在 `while(!isCurrent())` 100 Hz 空转、0 个 `/cmd_vel_nav`、BT 每秒新目标把 pending 槽顶掉 → 每秒一条 `Aborting handle`。判据：`Received a goal`=1 / `Passing new path`=0 / 线程 200 次切换每秒。单变量恢复实验（`inflation_radius` 5.0→5.01→5.0）后 cmd_vel 20–21 Hz、`Reached the goal!`、0 新增 abort。已落地 A（关 `clear_costmap_on_mission_start`）/ B（清图后自动重膨胀）/ C（输出静默 5 s 自愈看门狗，订阅 `/follow_path/_action/status` 判真实执行态），全部在自己节点，未改 nav2；②重启后地图不出**已确证**为 map_server 启动竞态（`load_node` 服务响应超时 → 同批 `lifecycle_manager_localization` 被静默丢弃，33 次启动里 1 次），已拆分装载 + 3 s 延迟 + `map_server_guard` 兜底；③**DDS/UDP 专项实测**：全局 RcvbufErrors 占入站 13.5%（218189/1618204），按进程 nav2 容器 67094 > mavros 36709；按窗口算速率——失败任务 mavros 1735/nav2 1263 每秒 vs **成功任务 804/603 每秒**（差异仅约 2 倍而非有/无），且空闲也在丢（86/55），故**否定**其为根因；MQTT raw 走 generic 订阅不反序列化且 nav-only 栈里未运行；④**未结案**：10:01 下发后 mission_bridge 接住但 FollowWaypoints 未到 nav2、status_detail 也未到 aggregator（注入消息可证 aggregator 正常），疑 DDS 端点匹配失效，同期有 overlay 测试服务制造 4 个重名节点；平台侧旧任务记录未清亦导致无法再下发；⑤**下午 11:18 另一次失败判明为"过载导致的服务超时"**：BT 恢复动作（清全局代价图）、planner goal ack、`get_state` 全部超时 → FollowPath 被取消、速度断 18 s，而 nav2 在 11:18:48 已自行恢复出 20 Hz，却被 mission_bridge 的 30 s 看门狗判死；局部代价图全程 4.8–5.0 Hz（非 §3.6 的 current 卡死）；load 12–20、mavros 占 80% CPU；另核实 livox 点云被丢 ≈1 次/分钟但 voxel_layer 关闭故无影响、TF 历史窗口正常 ~9 s、雷达实为 9.2 Hz |
| 2026-10-28 | [避障参数 A/B 与实船 MQTT/DDS 排查](2026-10-28_避障参数AB与实船MQTT-DDS排查.md) | 全天记录：高度 -0.30→-0.25→-0.20 每 5 cm 约 -12~14% lethal；盲区 1.4→1.6 有实质收益（清扫杆盒 33→5.3 点/帧），2.0→2.5 无收益（环内仅 ~5 点/帧）；坞边高度无效。实船"导航失败"=上层动作取消 + BATTERY 5001/BOAT_CONTROLLER 3006 阻断，`follow_path ... halt!` 为取消连带日志。仓库仅改 local 观测源为 `/livox/lidar`（未提交）；生产避障仍 `enabled:false` |
| 2026-10-28 | [MQTT 后视相机 raw 话题与 DDS 资源排查](2026-10-28_MQTT后视相机raw话题与DDS资源排查.md) | raw 6.22 MB×6 Hz≈38.7 MB/s vs compressed 3.05 MB/s；RMW=FastDDS；bridge 二进制无 OpenCV/cv_bridge/jpeg，开销为 DDS 反序列化+拷贝；control loop missed raw 12 vs compressed 5；干净栈 A/B raw 3/3、compressed 3/3 均正常完成，raw 增加压力但不阻断导航，此前失败在电池/飞控阻断窗口 |
| 2026-10-27 | [兴飞导航与归港联调：日志、TF、MQTT、围栏卡死修复与完整跑通](2026-10-27_兴飞导航与归港联调-日志TF_MQTT与围栏卡死修复验证.md) | `CTRL_STUCK` 根因为 map→odom 短时延迟（bag 中位 0.425 s / P95 2.238 s）；mission_bridge 的 subscription 销毁竞争**已修**；heading 阈值改 30°；MQTT 后视健康码 YAML 缩进**已修**；Decision `set_geofence` 无超时导致出坞后停船，对方加入 3 s×最多 4 次、耗尽 2032 后**完整 11 点任务+归港跑通**；本次未采样 TF/dock_pose，二维码精度不下结论 |
| 2026-10-26 | [兴飞导航未就绪、控制器卡死、CTRL_STUCK 误报与二维码通道未关](2026-10-26_兴飞导航未就绪_控制器卡死_CTRL_STUCK误报与二维码通道未关.md) | ①mavros `gps_input` 启动锁 `map_origin=(0,0,0)` → `map→odom` 野值，重启恢复，建议插件加经纬度校验；②mission_bridge `/tf` 未匹配 mavros `odom→base_link`（重启恢复，疑启动期时钟跳变）；③"出泊假成功"复核=实际移动 3.1 m，in_dock 旧锚点 12 帧重定位导致首查误判；④`follow_path` 目标卡 EXECUTING，107 个新目标全 ABORT（需重启）；⑤船漂入静态图陆地 → 2007；⑥新 CTRL_STUCK 看门狗误报**已修**（任务开始/恢复重置 + 线速度命令才算进展）；⑦归港取消/失败未关二维码通道**已修**；⑧`always_send_full_costmap`/局部发布频率已调 |
| 2026-10-27 | [避障过滤节点丢无效点 + min_obstacle_height 两档 A/B](2026-10-27_避障过滤节点丢无效点与最低高度AB.md) | `livox_cloud_filter` 新增 `drop_invalid_points`（默认丢 `|p|<1mm`，实测每帧 20k→2.3k、无效点 87~88%），证据图无影响；`mark_threshold` 确认保持 1；回放脚本新增 `--live-filter` / `--min-obstacle-height` / `--start-offset`。lake2 同段配对（79 对）：`-0.25` 比 `-0.30` lethal 中位少 40 格（93.7% 采样更低），幅度仅 2~3%，比 lake1 的 ~25% 小；生产参数未改 |
| 2026-10-26 | [避障代价图水面杂波：过滤节点、mark_threshold 与实船补录](2026-10-26_避障代价图水面杂波-过滤节点与阈值分析.md) | 已确证：`min_obstacle_height=-0.30` 在真实水线（约 -0.2~-0.27）以下；`mark_threshold` 数 z 体素层，z_res≤0.075 时 mark=2 与 mark=1 完全等价；强度在水线处不可分；过滤节点只遮 `x≥2.70`，近场还有一圈低强度点，每帧约 90% 为零距离无效点。已录 `lake2` 包（381 s / 4.0 GiB）并导出；`drop_invalid` 与 `--live-filter` 已于 10-27 完成，近场第二个小盒待确认 |
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
