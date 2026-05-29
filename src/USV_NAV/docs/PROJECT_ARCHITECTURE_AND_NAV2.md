# 实船架构与 Nav2 数据流

本文描述 **USV_NAV 实船** 包分工与话题/TF 链。**Gazebo 仿真** 见独立仓库 `wuxihik_navigation`。

**联调命令**：[`docs/项目运行与联调.md`](../../../docs/项目运行与联调.md)  
**按文件改参**：[`实船配置修改清单.md`](./实船配置修改清单.md)

---

## 1. 包分工

| 包 | 职责 |
|----|------|
| **`workspace_ros`** | MAVROS 启动与 TF 覆盖、`gnss_odom_map_tf`、`static_transform_real_boat`、`nav2_cmd_vel_to_mavros`、感知/任务脚本 |
| **`workspace_nav`** | `nav2_real_mavros.launch.py`、`nav2_params_real_mavros.yaml`、地图 yaml、**`mission_bridge` / `nav_status_aggregator`**、legacy `waypoint_transform` |

---

## 2. 实船启动顺序

1. `mavros_px4_usv.launch.py` — 飞控桥，`odom→base_link` TF  
2. `real_boat_bringup.launch.py` — `gnss_odom_map_tf` + 传感器静态 TF +（可选）速度桥  
3. `nav2_real_mavros.launch.py` — Nav2 + RViz +（默认）**mission_bridge + nav_status_aggregator**  

---

## 3. 数据流总览

```text
                    ┌─────────────────────────────────────┐
                    │  map_server (map_real_boat_hk.yaml)   │
                    └─────────────────┬───────────────────┘
                                      │ map 系规划
┌──────────────┐   GNSS + local odom  ▼   ┌──────────────────┐
│ gnss_odom_   │ ───────────────────────► │ map → odom       │
│ map_tf       │                          └────────┬─────────┘
└──────────────┘                                   │
                                                   ▼
┌──────────────┐   /mavros/local_position/odom   ┌──────────────┐
│ PX4 + MAVROS │ ───────────────────────────────►│ odom→base_link│
└──────────────┘                                  └──────┬───────┘
                                                         │
     /livox/lidar ──► costmap ◄── Nav2 ◄── /cmd_vel_nav ─┤
                                                         │
                              nav2_cmd_vel_to_mavros ────┘
                                         │
                                         ▼
                              /mavros/setpoint_raw/local → PX4
```

---

## 4. TF 链（模式 B，默认）

| 变换 | 发布者 |
|------|--------|
| `map` → `odom` | `gnss_odom_map_tf`（读 `map_config_yaml` + `map_origin_ref_key`） |
| `odom` → `base_link` | MAVROS `local_position`（`mavros_px4_overrides_usv.yaml`） |
| `base_link` → 传感器 | `static_transform_real_boat.yaml` |

仅起 MAVROS、不起 bringup：**无 `map→odom`**。

---

## 5. 速度话题

| 话题 | 说明 |
|------|------|
| `/cmd_vel_nav` | **`controller_server` 发布**（Nav2 bringup remap） |
| `/cmd_vel` | **`velocity_smoother` 发布**（订阅 `/cmd_vel_nav`）；桥可选 **`cmd_vel_src:=/cmd_vel`** |
| `/mavros/setpoint_raw/local` | `nav2_cmd_vel_to_mavros`（默认订 `/cmd_vel_nav`）→ PX4 OFFBOARD |

调试命令见 [`docs/项目运行与联调.md`](../../../docs/项目运行与联调.md)「调试命令速查」。

**不要**在实船运行 `converter`（Gazebo 推力，仅 wuxihik 仿真用）。

---

## 6. Nav2 要点（`nav2_params_real_mavros.yaml`）

- **里程计**：`/mavros/local_position/odom`  
- **全局帧 / 局部帧**：`map` / `odom`，`robot_base_frame: base_link`  
- **避障**：`VoxelLayer` + **`/livox/lidar`**（核对点云 `frame_id` 与 TF）  
- **控制器**：Regulated Pure Pursuit；参数与 `nav2_cmd_vel_to_mavros` 限幅同量级  

---

## 7. 地图与 datum

- 默认栅格：**`config/map_real_boat_hk.yaml`** + **`map/hk_map.pgm`**  
- 四角 **`ref_gnss_*`**：`[longitude, latitude]`（度）  
- **`gnss_odom_map_tf`**、**`waypoint_transform`**、**PX4 HOME** 须共用同一角点约定  

详见 [`docs/地图与GNSS-Nav2对齐说明.md`](../../../docs/地图与GNSS-Nav2对齐说明.md)。

---

## 8. 航点、地面站与上层（mission_bridge，默认）

```text
GCS /waypoint (WGS84 JSON) ──► mission_bridge ──► FollowWaypoints
GCS /color_code ────────────► mission_bridge ──► target_buoy.json
上层 Service send_waypoints 等 ──► 同一 mission_bridge
nav_status_aggregator ──► /nav_status、/task_event
```

- 默认 **`waypoint_command_mode=debounce`**（GCS 发一次 `/waypoint` 约 0.45s 后自动开走）  
- **`datum_source:=map_yaml`**，与 `ref_gnss*` 同源  
- Legacy：`waypoint_transform` → JSON → `waypoint_with_state`（**仅** `enable_mission_bridge:=false` 时使用）

详见 [`mission_bridge与上层对接.md`](./mission_bridge与上层对接.md)。

---

## 9. 可选 / 未维护

| 项 | 状态 |
|----|------|
| `localization.launch.py` + EKF + navsat | **本仓未提供**；`real_boat_bringup` 若设 `robot_localization` 会引用缺失文件 |
| `nav2.launch.py` / `nav2_params.yaml` | **仿真用**，在 wuxihik |
| `kamikaze` | 与 Nav2 同发 `/cmd_vel_nav`，勿并行 |

---

## 10. 改参索引

| 目标 | 文件 |
|------|------|
| 避障话题/高度 | `nav2_params_real_mavros.yaml` |
| 控制/到点容差 | 同上 `controller_server` |
| 速度桥限幅 | `nav2_cmd_vel_to_mavros.py` / launch |
| 地图锚点 | map yaml、`real_boat_bringup` 的 `map_config_yaml` |
| MAVROS TF | `mavros_px4_overrides_usv.yaml` |

完整表：[`实船配置修改清单.md`](./实船配置修改清单.md)。
