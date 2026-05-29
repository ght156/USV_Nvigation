# mission_bridge 与上层 / 地面站对接（实船）

> 实船 **USV_NAV** 自 2026-05 起与仿真仓 `wuxihik_navigation` 共用同一套 mission 接口。  
> **接口契约全文**：仿真仓 `docs/nav_task_interface.md` v4.1（本地可复制一份到 `USV_NAV/docs/` 备查）。

---

## 1. 架构（默认）

```text
竞赛上层 ── Service ×4 ──┐
地面站 GCS ── Topic ─────┼──► mission_bridge ──► FollowWaypoints ──► Nav2
                         │
nav_status_aggregator ◄──┘ status_detail / odom / GPS / planner
         │
         ├── /nav_status
         └── /task_event
```

**Legacy（勿并行）**：`waypoint_transform` + `waypoint_with_state` + `target_buoy`。

---

## 2. 启动

与 Nav2 一体（推荐）：

```bash
ros2 launch workspace_nav nav2_real_mavros.launch.py use_sim_time:=false
```

- **`map_yaml_path`**：launch 内自动与 Nav2 **`map:=`** 一致；换图只改 `map:=`。
- **`mission_params_file`**：默认 `config/mission_stack.real_boat.yaml`（子 launch 参数名 `mission_stack_params_file`），**不是** Nav2 的 `params_file`。
- **`mission_odom_topic`**：默认 `/mavros/local_position/odom`（就绪检查、到点容差、`/nav_status`）。

仅 mission 栈（Nav2 已在其它终端）：

```bash
ros2 launch workspace_nav mission_bridge.launch.py use_sim_time:=false
```

Nav2 使用非默认地图时再传：`map_yaml_path:=<与 Nav2 map:= 相同的路径>`。

---

## 3. 实船话题默认值

| 项 | 值 |
|----|-----|
| odom | `/mavros/local_position/odom` |
| GPS（aggregator） | `/mavros/global_position/raw/fix` |
| GCS 航线 | `/waypoint`（WGS84 JSON） |
| GCS 取消 | `/gcs_mission/cancel` |
| 目标色 | `/color_code` |
| 状态反馈 | `/nav_status`、`/task_event` |

---

## 4. 从仿真迁移改了什么

| 操作 | 内容 |
|------|------|
| **复制** | `m_common`、`mission_bridge.py`、`nav_status_aggregator.py`、`gps_map_conversion.py` |
| **新建** | `mission_stack.real_boat.yaml`、`launch/mission_bridge.launch.py`（实船默认） |
| **小改** | `nav2_real_mavros.launch.py`（`enable_mission_bridge`）、`setup.py`、`package.xml` |
| **未改** | bringup、Nav2 参数、地图文件、legacy 节点 |

---

## 5. Mock 迁移测试

在**开发机**用仿真仓脚本验证（不启 PX4）：

```bash
cd /path/to/wuxihik_navigation
bash test/run_usv_migrate_smoke_test.sh
# USV_WS 默认 /home/ght/USV_NAV
```

| 文档 | 内容 |
|------|------|
| 仿真仓 `test/MISSION_MIGRATION_TEST.md` | 用例、环境、**2026-05-29 PASS 7/0** 结果 |
| 仿真仓 `docs/USV_NAV_MISSION_SYNC_PLAN.md` | 迁移规划（Phase 1 已完成） |

Mock **通过 ≠ 实船全栈通过**；全栈仍需 MAVROS + bringup + Nav2 +（可选）GCS。

---

## 6. 实船联调检查

- [ ] 终端 3 已起且 **未**并行 legacy 三节点  
- [ ] `ros2 topic echo /nav_status --once` 有 JSON  
- [ ] GCS Dispatch → `task.state=RUNNING`  
- [ ] GCS Cancel → `IDLE`  
- [ ] `map:=` 与 bringup `map_config_yaml` 同源  

---

## 7. 地面站

独立仓库 **`GROUND-CONTROL-STATION-dev`**；对接说明见其 `docs/NAV_STACK_INTEGRATION.md`。  
实船与 GCS 须 **同一 `ROS_DOMAIN_ID`**。
