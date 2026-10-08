# mission_bridge 与上层 / 地面站对接（实船）

> 实船 **USV_NAV** 自 2026-05 起与仿真仓 `wuxihik_navigation` 共用同一套 mission 接口。  
> **接口契约全文**：[`docs/nav_task_interface.md`](../../../docs/nav_task_interface.md) v4.1

---

## 1. 架构（默认）

```text
竞赛上层 ── Service ×3 ──┐
Decision ── Action ×1 ───┤
地面站 GCS ── Topic ─────┼──► mission_bridge ──► FollowWaypoints ──► Nav2
                         │
nav_status_aggregator ◄──┘ status_detail / odom / GPS / planner
         │
         ├── /nav_status
         └── /task_event
```

上层接口明细：Service `send_waypoints` / `set_pause` / `cancel_mission`，Action `/mission_bridge/navigate`（NavigateTask）。

**Legacy（勿并行）**：`waypoint_transform` + `waypoint_with_state` + `target_buoy`。

---

## 2. 启动

与 Nav2 一体（推荐）：

```bash
ros2 launch workspace_nav nav2_boat.launch.py use_sim_time:=false enable_mission_bridge:=true
```

- **`enable_mission_bridge`**：默认 `false`，不显式给 `true` 不会启动 mission 栈。
- **`map_yaml_path`**：launch 内自动与 Nav2 **`map:=`** 一致；换图只改 `map:=`。
- **`mission_params_file`**：默认 `config/mission_stack.chengfeng.yaml`（子 launch 参数名 `mission_stack_params_file`），**不是** Nav2 的 `params_file`。
- **`odom_topic`**：默认 `/mavros/gps_input/local`（统一控制 Nav2 与 mission 栈的 odom 话题；ArduPilot gps_input 链路；就绪检查、到点容差、`/nav_status`）。

仅 mission 栈（Nav2 已在其它终端）：

```bash
ros2 launch workspace_nav mission_bridge.launch.py use_sim_time:=false
```

Nav2 使用非默认地图时再传：`map_yaml_path:=<与 Nav2 map:= 相同的路径>`。

---

## 3. 实船话题默认值

| 项 | 值 |
|----|-----|
| odom | `/mavros/gps_input/local` |
| GPS（aggregator） | `/mavros/gps_input/raw/fix` |
| GCS 航线 | `/waypoint`（WGS84 JSON，**debounce** 约 0.45s 后自动开始） |
| GCS 取消 | `/gcs_mission/cancel` |
| 目标色 | `/color_code` |
| 状态反馈 | `/nav_status`、`/task_event` |

---

## 4. mission_bridge 参数清单（2026-09 逐条核对代码后）

全部参数在 `mission_bridge.py::__init__` 声明。除下列注明项外均真实生效；
实船值以 `config/mission_stack.chengfeng.yaml` + launch 内联覆盖为准。

| 参数 | 默认 | 生效情况 |
|------|------|----------|
| `waypoint_topic` | `/waypoint` | 生效，GCS 航线话题 |
| `color_topic` | `/color_code` | 生效，目标色话题 |
| `follow_waypoints_action` | `follow_waypoints` | 生效，Nav2 action 名 |
| `global_frame` / `robot_frame` | `map` / `base_link` | 生效，TF 就绪检查与位姿帧 |
| `map_yaml_path` | 空 | 生效；空时由 launch 注入，代码兜底 `share/config/map.yaml` |
| `waypoints_json_path` / `target_buoy_json_path` | 空 | 生效；空时走 `TARGET_JSON_PATH` 环境变量或包内默认路径 |
| `datum_source` | `map_yaml` | **实质固定**：其它值节点直接 fatal 退出（首版仅支持 map_yaml） |
| `map_datum_ref_key` | `ref_gnss_10` | 生效，map YAML 里的 datum 锚点键 |
| `projection` | `enu` | 生效；`utm` 仅警告不禁止 |
| `odom_topic` | `/mavros/gps_input/local` | 生效，到点容差距离计算 |
| `waypoint_tolerance_m` | `0.0` | `<=0` = 自动读 Nav2 参数文件的 `xy_goal_tolerance`（当前 2.0 m），与 Nav2 到达判定同源；显式正数则优先。仅用于"船已在容差内则跳过该点"，真正的到点判定在 Nav2 |
| `tf_check_period_sec` | `1.0` | 生效，WAITING_SYSTEM 轮询周期 |
| `debug_mode` | `false` | 生效，打印原始 waypoint/color 载荷 |
| `allow_replace_running_mission` | `false` | 生效，**仅约束 GCS topic 通道**；Service `send_waypoints` 恒可抢占，NavigateTask action 恒不抢占（需先 Cancel） |
| `allow_repeat_identical_route` | `false` | 生效，完成后拦截同 hash 被动重发 |
| `target_buoy_force_rewrite` | `false` | 生效，同色强制重写 target_buoy.json |
| `waypoint_commit_delay_sec` | `0.45` | 生效，GCS /waypoint 防抖窗口（下限 0.05s） |
| `mission_cancel_topic` | `/gcs_mission/cancel` | 生效；置空串则不订阅取消话题 |
| `suppress_passive_waypoints_after_cancel` | `true` | 生效，Cancel 后丢弃无 `explicit_replan` 的被动 /waypoint |
| `target_buoy_min_write_period_sec` | `0.0` | 生效，target_buoy.json 写盘节流（0=不节流） |
| `nav_zones_topic` | `/nav_zones/current` | 生效，电子围栏预校验；launch 未暴露同名 launch 参数，只能 YAML 覆盖 |

**已移除**：`discard_watchdog_sec`（"抢占丢弃 goal 看门狗"）——声明后从未被任何逻辑读取，
属于死参数，2026-09 从节点与 launch 中一并删除。

**2026-09 顺带修复的逻辑问题**：

- 被动/重复 /waypoint 消息以前会在被丢弃前覆盖 `_current_mission_id/_current_command_id`，
  导致 `/mission_bridge/status_detail` 的 `task_id` 被清空；现改为任务确定执行时才更新 id。
- `map_yaml_path` 代码兜底路径指向不存在的 `map_hk.yaml`，已改为实际存在的 `map.yaml`。

---

## 5. 从仿真迁移改了什么

| 操作 | 内容 |
|------|------|
| **复制** | `m_common`、`mission_bridge.py`、`nav_status_aggregator.py`、`gps_map_conversion.py` |
| **新建** | `mission_stack.real_boat.yaml`、`launch/mission_bridge.launch.py`（实船默认） |
| **小改** | `nav2_real_mavros.launch.py`（`enable_mission_bridge`）、`setup.py`、`package.xml` |
| **未改** | bringup、Nav2 参数、地图文件、legacy 节点 |

---

## 6. Mock 迁移测试

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

## 7. 实船联调检查

- [ ] 终端 3 已起且 **未**并行 legacy 三节点  
- [ ] `ros2 topic echo /nav_status --once` 有 JSON  
- [ ] GCS Dispatch → `task.state=RUNNING`  
- [ ] GCS Cancel → `IDLE`  
- [ ] `map:=` 与 bringup `map_config_yaml` 同源  

---

## 8. 地面站

独立仓库 **`GROUND-CONTROL-STATION-dev`**；对接说明见其 `docs/NAV_STACK_INTEGRATION.md`。  
实船与 GCS 须 **同一 `ROS_DOMAIN_ID`**。
