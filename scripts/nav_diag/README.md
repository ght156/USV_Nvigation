# 船端导航 / 代价图在线诊断台（只读）

一个入口包住全部导航诊断：**只订阅、只读、不发指令**（不动船）。船端不用装东西——
每次由本机把船端脚本经 ssh stdin 送过去执行，跑完即退。

```bash
cd /home/ght/USV_NAV
python3 scripts/nav_diag/nav_diag.py <模式> [参数] [--host hkzl]
```

底层复用 `scripts/map_dispatch/waypoint_dispatch_bridge.py` 的 ssh/remote 机制与
`remote_nav_cli.py`（船端侧脚本）；航点下发（页面）也在 `scripts/map_dispatch/`。

## 模式速查

| 模式 | 用途 | 典型判读 |
|---|---|---|
| `check` | 环境/服务/`/nav_status` 自检 | `send_waypoints`/`set_pause`/`cancel` 是否在线、是否 IDLE |
| `status` | 读一次 `/nav_status` | `task_state`、`localization_overall`、`gps_fix` |
| `watch 60` | 流式盯导航 | `cmd v=0 w=±0.15` = RPP 原地转对准；`d_yaw` 收敛而 `d_goal` 不动 = 收角度；一直 `v≈0` 且 `d_goal` 不降 = 进了恢复行为 |
| `diag 10` | 雷达落图自检 | TF 值、点云俯视形状（上=船头、左=左舷）、各过滤级存活数、代价图形状 |
| `compare 5` | **点云 vs 局部代价图逐格对比** | `expected_but_NOT_marked` 大 = 点云有、代价图没标；`per_frame_ge2_cells` 小 = 被 `mark_threshold` 卡住 |
| `voxelmap 4` | 解码 `VoxelGrid`（层内部网格） | `#` = 已标（≥2 个 z 层）、`1` = 只有 1 个 z 层被 `mark_threshold` 挡掉 |
| `tfcheck 6` | 查 TF 重复发布者/值冲突 | `冲突的边` 非空 = 同一段 TF 两个发布者在打架 |
| `count [话题]` | 数一张 OccupancyGrid 的格值分布 | 清代价图前后对比"清掉了没有" |
| `params [节点]` | 列/读运行中参数 | 可直接照抄去 `ros2 param set` |

## 已经踩过的坑（都实测过，别再重复排查）

1. **清代价图的服务类型**是 `nav2_msgs/srv/ClearEntireCostmap`（不是 `std_srvs/Empty`）：
   写错会一直 `waiting for service to become available...`。
   ```bash
   ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
   ```
   同理 `clear_around_*` 用 `nav2_msgs/srv/ClearCostmapAroundRobot`、`clear_except_*` 用
   `nav2_msgs/srv/ClearCostmapExceptRegion`。
2. **`mark_threshold` 不是"点数"，是"该格被标过的 z 体素数"，且是严格大于**
   （`nav2_voxel_grid::markVoxelInMap()` 取该列 16 位 z 掩码的置位数，`popcount > 阈值` 才写
   `LETHAL_OBSTACLE`）。所以 0=单层即标、1=需≥2层、2=需≥3层（2026-10-10 更正，不是 ≥）。
   同一格里的点在同一个 10 cm 高度层时只有 1 层 → 阈值 1 就标不上，但 voxel 位跨帧累积，
   平面目标靠船体微晃几帧内能攒出第 2 层。实测：阈值 2 → 致命格 281；阈值 1 → 1408。
3. `voxel_layer.min_obstacle_height` 在 voxel 路径里**不生效**（实测把它改 -0.35，致命格数完全不变）；
   下界由**观测源**的 `min_obstacle_height` 决定。
4. `obstacle_max_range` 要覆盖滚动窗口：24×24 m 窗口半对角 ≈17 m → 取 18；
   全局图（非滚动窗口）想预判远岸就取 30。
5. 局部代价图**没有静态层**，障碍 100% 来自 `/livox/lidar`；清完代价图后雷达 10 Hz 会立刻重新标回来
   （⚠️ 2026-10-10 更正：`marking false` 在线设置**无效**（观测源参数启动时定死，param set 返回成功但
   行为不变），清完雷达 0.2s 内就重新标满；要真清只能停雷达或重启栈。详见
   `docs/bug_debug/2026-10-10_代价图与点云不吻合-观测源参数在线无效与栈重启生效.md`）。
6. `/local_costmap/voxel_grid` 是 `nav2_msgs/VoxelGrid`（不是点云）；`voxel_marked_cloud` 本版 nav2 不发布。
7. **"船贴岸但代价图里看不到岸"的排查顺序**（2026-10-09 实测）：
   ① 先确认看的是哪张图——**全局代价图 `voxel_layer.enabled=false`，上面没有任何雷达障碍**，
   只有静态图黑边（静态图岸线偏水域，看着就像"没显示"）；② 不是高度带（0.6→1.5 只多 37 格）；
   ③ 不是 TF 冲突（`tfcheck` 单值）；④ 贴边时真正吃掉岸的是 **`obstacle_min_range: 1.4`
   （以雷达光心为球心）+ `footprint_clearing_enabled: true`（船体 footprint+padding 内的标记
   每周期被清）**；⑤ 底噪：83% 的回波是 0~0.2 m 贴窗回波，先擦窗口（`--diag` 的"雷达 1.4m 内"
   应从 1.6 万降到几百）。
