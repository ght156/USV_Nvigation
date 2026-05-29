# 仿真 → 实船（USV_NAV）Mission / 上层对接同步方案

> **状态**：Phase 1 已完成（2026-05-29）| mock 测试 PASS 见 `wuxihik_navigation/test/MISSION_MIGRATION_TEST.md`
> **源（已冻结接口）**：`/home/ght/wuxihik_navigation`（`YILDIZ-USV/workspace_nav` + `src/m_common`）  
> **目标**：`/home/ght/USV_NAV`（`USV_NAV/workspace_nav` + `workspace_ros`）  
> **契约文档**：`docs/nav_task_interface.md` v4.1（GCS Topic + 上层 Service + `/nav_status`/`/task_event`）

---

## 1. 现状对比

### 1.1 仿真侧（已完成）

| 组件 | 路径 | 作用 |
|------|------|------|
| `mission_bridge` | `workspace_nav/mission_bridge.py` | GCS Topic + 上层 Service → FollowWaypoints；写 `waypoints.json` / `target_buoy.json` |
| `nav_status_aggregator` | `workspace_nav/nav_status_aggregator.py` | 发布 `/nav_status`、`/task_event` |
| `gps_map_conversion` | `workspace_nav/gps_map_conversion.py` | 经纬度↔map 共用解析/写盘 |
| `m_common` | `src/m_common/` | Service 定义（`SendWaypoints` 等） |
| `mission_bridge.launch.py` | `workspace_nav/launch/` | 两节点 + 实船参数示例 YAML |
| 接口文档 | `docs/nav_task_interface.md` | 对外契约 |

### 1.2 实船侧（当前）

| 组件 | 路径 | 作用 |
|------|------|------|
| `waypoint_transform` | `workspace_nav/waypoint_transform.py` | `/waypoint` → `waypoints.json`（**内联**坐标转换，未用 `gps_map_conversion`） |
| `waypoint_with_state` | `workspace_nav/waypoint_with_state.py` | 监视 JSON → `FollowWaypoints` |
| `target_buoy` | `workspace_ros/scripts/target_buoy.py` | `/color_code` → `target_buoy.json` |
| Nav2 | `nav2_real_mavros.launch.py` | 实船 Nav2 + RViz |
| Bringup | `real_boat_bringup.launch.py` | MAVROS odom TF、速度桥（**不含** mission 节点） |

**缺口**：无 `mission_bridge`、无 `nav_status_aggregator`、无 `m_common`、无 `/nav_status` 反馈、无上层 Service。

### 1.3 功能重叠（迁移后只跑一套）

```
当前实船三节点链：
  /waypoint → waypoint_transform → waypoints.json → waypoint_with_state → Nav2
  /color_code → target_buoy → target_buoy.json

迁移后（与仿真一致）：
  /waypoint ─┐
  /color_code ┼→ mission_bridge → waypoints.json + target_buoy.json + FollowWaypoints
  Service ×4 ─┘
  nav_status_aggregator ← status_detail / odom / GPS / planner → /nav_status, /task_event
```

**原则**：保留旧节点源码与 `console_scripts` 入口（便于回退/单测），**默认 launch 改跑 mission 栈**；文档标明「勿与 mission_bridge 并行」。

---

## 2. 同步范围

### 2.1 可直接复制（整文件）

| 源 | 目标 | 说明 |
|----|------|------|
| `wuxihik_navigation/src/m_common/` | `USV_NAV/src/m_common/` | 新包，`colcon build` 需编进工作区 |
| `.../gps_map_conversion.py` | `USV_NAV/.../workspace_nav/gps_map_conversion.py` | 新文件 |
| `.../mission_bridge.py` | 同路径 | 已含 `USV_NAV` 路径探测，一般可直接用 |
| `.../nav_status_aggregator.py` | 同路径 | 同上 |
| `.../launch/mission_bridge.launch.py` | 同路径 | launch 默认 `debounce`，实船参数靠 YAML 覆盖 |
| `.../config/mission_stack.example.yaml` | 同路径 | **复制后改实船路径**（见 §3） |
| `docs/nav_task_interface.md` | `USV_NAV/docs/nav_task_interface.md` | 契约原文 |
| `test/`（可选） | `USV_NAV/test/` | Service 联调脚本；实船部署可不拷 |

### 2.2 合并修改（不覆盖实船已有逻辑）

| 文件 | 改动 |
|------|------|
| `workspace_nav/setup.py` | 仅 **追加** `mission_bridge`、`nav_status_aggregator` 两个 `console_scripts` |
| `workspace_nav/package.xml` | 仅 **追加** `m_common`、`nav_msgs`、`nav2_msgs`、`action_msgs` 依赖（与仿真对齐） |
| `workspace_nav/workspace_nav/waypoint_transform.py` | **Phase 2 可选**：改为调用 `gps_map_conversion`（与仿真一致）；**Phase 1 可不动**，因默认不再启此节点 |

### 2.3 实船专用 launch / 配置（新建或小改）

| 文件 | 动作 |
|------|------|
| `config/mission_stack.real_boat.yaml` | **新建**（由 example 改实船默认值，见 §3.1） |
| `launch/mission_bridge.launch.py` | 复制后 **仅改 launch 默认值**：`use_sim_time=false`、`map_yaml_path`→实船 map、`odom_topic`→MAVROS |
| `launch/nav2_real_mavros.launch.py` | **可选** 末尾 `IncludeLaunchDescription(mission_bridge.launch.py)`；或独立终端启动（见 §3.2） |
| `workspace_ros/launch/real_boat_bringup.launch.py` | **不改**（保持 TF/速度桥职责单一） |
| `docs/项目运行与联调.md` | 更新「地面站与任务节点」章节 |

### 2.4 明确不修改（实船已有功能）

- `real_boat_bringup.launch.py` / `gnss_odom_map_tf` / MAVROS TF 链
- `nav2_params_real_mavros.yaml`（Nav2 参数，除非缺 `waypoint_follower` 插件——当前已有）
- `nav2_cmd_vel_to_mavros`、PX4/OFFBOARD 逻辑
- `kamikaze.py`、`target_buoy.py`（**保留源码**，默认文档改为不并行启动）
- `waypoint_with_state.py`（保留，作 legacy）
- 实船地图 `map_real_boat_hk.yaml`、`static_transform_real_boat.yaml` 等

---

## 3. 实船参数映射

### 3.1 `mission_stack.real_boat.yaml`（建议内容）

```yaml
mission_bridge:
  ros__parameters:
    map_yaml_path: "<install/share/workspace_nav/config/map_real_boat_hk.yaml>"  # 或与 Nav2 map:= 一致
    odom_topic: "/mavros/local_position/odom"
    global_frame: "map"
    robot_frame: "base_link"
    map_datum_ref_key: "ref_gnss_10"          # 与 gnss_odom_map_tf / 制图一致
    waypoint_command_mode: "debounce"
    allow_replace_running_mission: false
    mission_cancel_topic: "/gcs_mission/cancel"

nav_status_aggregator:
  ros__parameters:
    gps_topic: "/mavros/global_position/raw/fix"
    publish_rate: 2.0
    vehicle_id: "usv_real_001"
    odom_timeout: 2.0
    gps_timeout: 5.0
    stuck_progress_timeout: 12.0
```

> `odom_topic` 由 `mission_bridge.launch.py` 内联参数绑定到 aggregator，YAML 里 aggregator **不必重复** `odom_topic`（与仿真 launch 行为一致）。

### 3.2 推荐启动顺序（迁移后）

与现有「三终端」兼容，**新增终端 4**：

| 终端 | 命令 | 变更 |
|:---:|------|------|
| 1 | MAVROS | 不变 |
| 2 | `real_boat_bringup.launch.py` | 不变 |
| 3 | `nav2_real_mavros.launch.py` | 不变 |
| **4** | `ros2 launch workspace_nav nav2_real_mavros.launch.py`（默认含 mission；`map:=` 与 bringup 同源即可）或独立 `mission_bridge.launch.py` + `mission_stack_params_file:=...` | **已接入** |

**不再默认启动**（除非调试 legacy）：

```bash
# 旧流程 — 与 mission_bridge 互斥
ros2 run workspace_nav waypoint_transform
ros2 run workspace_nav waypoint_with_state
ros2 run workspace_ros target_buoy
```

### 3.3 Launch 集成策略（二选一，建议 A）

| 方案 | 做法 | 优点 | 缺点 |
|------|------|------|------|
| **A. 独立 launch** | 终端 4 手动起 `mission_bridge.launch.py` | 不动现有 bringup/Nav2；故障隔离 | 多开一个终端 |
| **B. Include 进 Nav2 launch** | `nav2_real_mavros.launch.py` 末尾 include mission | 一键起全栈 | 改实船 Nav2 launch；Nav2 挂掉时 mission 也重启 |

**建议 Phase 1 用方案 A**；联调稳定后再考虑 B。

---

## 4. 实施步骤（确认后执行）

### Phase 1 — 最小可联调（约 1 次 PR）

1. 复制 `m_common`、`gps_map_conversion.py`、`mission_bridge.py`、`nav_status_aggregator.py`、`mission_bridge.launch.py`
2. 新建 `mission_stack.real_boat.yaml`，改实船话题/地图路径
3. 改 `setup.py` / `package.xml`（仅追加）
4. `colcon build --packages-select m_common workspace_nav`
5. 更新 `docs/项目运行与联调.md` + 拷贝 `nav_task_interface.md`
6. **不删** legacy 三节点

### Phase 2 — 验证清单

| # | 项 | 通过标准 |
|---|-----|----------|
| 1 | GCS Dispatch | `/nav_status.task.state` → `RUNNING` |
| 2 | GCS Cancel | → `IDLE` |
| 3 | `/color_code` | `target_buoy.json` 更新（停 `target_buoy` 节点后仍正常） |
| 4 | 上层 Service | `ros2 service call .../send_waypoints` success + `TASK_STARTED` |
| 5 | 急停/恢复 | `emergency_stop` → `EMERGENCY`；`cancel_mission` → `IDLE` |
| 6 | 与 Nav2 共存 | `/cmd_vel_nav` 仍进 MAVROS；`kamikaze` 仍勿并行 |
| 7 | 地图一致 | `map_yaml_path` = Nav2 `map:=` = `gnss_odom_map_tf` |

### Phase 3 — 可选清理（非必须）

- `waypoint_transform` 重构为使用 `gps_map_conversion`（减少双份坐标逻辑）
- `nav2_real_mavros.launch.py` include mission
- 实船 `test/` 脚本
- 废弃文档中三节点「默认启动」描述

---

## 5. 风险与对策

| 风险 | 对策 |
|------|------|
| mission_bridge 与三 legacy 节点同时跑，双发 FollowWaypoints | 文档 + launch 注释；启动脚本只起 mission 栈 |
| `m_common` 未编入工作区导致 Service 找不到 | `src/m_common` 与 `workspace_nav` 同 workspace `colcon build` |
| GPS/odom 话题与仿真不同 | 全部走 `mission_stack.real_boat.yaml`，不改 Python 默认值 |
| `use_sim_time` 误为 true | launch 实船强制 `false` |
| GCS 仍连仿真域 | 实船/NX 设 `ROS_DOMAIN_ID` 与 GCS 一致（见实船联网手册） |

---

## 6. 文件级 Diff 预览（Phase 1）

```
USV_NAV/
├── src/m_common/                          [NEW 整目录复制]
├── src/USV_NAV/workspace_nav/
│   ├── workspace_nav/
│   │   ├── gps_map_conversion.py          [NEW]
│   │   ├── mission_bridge.py              [NEW]
│   │   └── nav_status_aggregator.py       [NEW]
│   ├── launch/
│   │   └── mission_bridge.launch.py       [NEW]
│   ├── config/
│   │   └── mission_stack.real_boat.yaml   [NEW 由 example 改]
│   ├── setup.py                           [MERGE +2 entry_points]
│   └── package.xml                        [MERGE +deps]
└── docs/
    ├── nav_task_interface.md              [NEW 复制]
    └── 项目运行与联调.md                   [MERGE §地面站]
```

**预计不改**：`workspace_ros/*`、`nav2_params_real_mavros.yaml`、`real_boat_bringup.launch.py`、legacy 三节点 `.py` 文件内容。

---

## 7. 确认项（请你拍板）

- [ ] Launch 集成选 **方案 A（独立终端）** 还是 **方案 B（并入 Nav2 launch）**？
- [ ] 实船默认地图用 **`map_real_boat_hk.yaml`** 还是当前 **`map.yaml`**？
- [ ] Phase 1 是否一并复制 **`test/`** 到实船仓库？
- [ ] legacy 三节点是否在文档中标记 **deprecated**，还是仅写「勿并行」？

确认后即可按 Phase 1 动手；GCS 与地面站侧 **无需再改接口**。
