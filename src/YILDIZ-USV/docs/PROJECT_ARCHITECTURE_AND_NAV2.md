# Gazebo 仿真与 Nav2（YILDIZ-USV / wuxihik_navigation）

本仓库 **以 Gazebo + EKF + Nav2 仿真为主**。**实船 MAWROS / PX4 / NX 联调** 在独立仓库 **[USV_NAV](https://github.com/ght156/USV_Navigation)**（路径 `USV_NAV/`）维护，避免双份栈同步成本。

**仿真联调入口**（本仓库根目录）：[`docs/项目运行与联调.md`](../../docs/项目运行与联调.md)

---

## 1. 包分工

| 包 | 职责 |
|----|------|
| **`workspace_gz`** | Gazebo 世界、模型、`ros_gz_bridge`、推进器与传感器话题 |
| **`workspace_ros`** | `localization.launch.py`（EKF + navsat + 静态 TF + 协方差转发）、`converter`（仿真推进器）、感知/任务脚本 |
| **`workspace_nav`** | `nav2.launch.py`、`nav2_params.yaml`、默认 **`config/map.yaml`**、航点节点 |

---

## 2. 推荐启动顺序（仿真）

1. **`workspace_gz` `simulation.launch.py`** — Gazebo + 时钟 + 桥接（IMU、GPS、激光、推力等）  
2. **`workspace_ros` `localization.launch.py`** — `use_sim_time:=true`，融合得到 **`/odometry/filtered`**  
3. **`workspace_nav` `nav2.launch.py`** — Nav2 + RViz（默认 `map:=` 包内 `map.yaml`）  
4. **（可选）** `ros2 run workspace_ros converter` — Nav2 `/cmd_vel_nav` → Gazebo 推力（与实船无关）

---

## 3. 数据流总览

```text
Gazebo ──► IMU/GPS/odom ──► covariance repub ──► EKF + navsat ──► /odometry/filtered
                                                      │
map_server (map.yaml) ◄── Nav2 ◄────────────────────┘  (map frame)
     │
     └── local_costmap ◄── LaserScan /roboboat/sensors/lidar/scan
```

若要把 Nav2 接到 **`/mavros/local_position/odom`** 或其它实机话题配置，请到 **USV_NAV** 使用对应 Nav2 yaml 与 launch。

---

## 4. TF 与里程计（仿真默认）

| 项 | 约定 |
|----|------|
| 融合里程计 | **`/odometry/filtered`**（`ekf.yaml`） |
| 静态 TF（传感器） | `static_transform.yaml`（与 URDF `*_link` → `roboboat/base_link/sensor_*` 对齐） |
| Nav2 `odom_topic` | **`/odometry/filtered`**（见 `nav2_params.yaml`） |

---

## 5. 航点与地面站

- GCS 发 **`/waypoint`** （见 `gps_map_conversion.parse_waypoint_payload`：list 或字典包裹 `waypoints`）与可选 **`/color_code`**：
  - **`mission_bridge`**（`enable_mission_bridge:=true` 或单独 `mission_bridge.launch.py`）：常驻、去重哈希、写 **`target_buoy.json`**、逐点 **FollowWaypoints**（`/odometry/filtered`）。
  - **`waypoint_transform`**（分立节点）：只写 **`waypoints.json`**，适合不关 Nav2、仅回放文件场景。  

---

## 6. 地图与 datum

- 默认栅格：**`workspace_nav/config/map.yaml`** 及对应 PGM  
- **`ref_gnss_*`**、**`navsat.yaml` datum**、**`waypoint_transform`** 须一致  

详见 [`docs/地图与GNSS-Nav2对齐说明.md`](../../docs/地图与GNSS-Nav2对齐说明.md)。

---

## 7. 改参索引（仿真）

| 目标 | 文件 |
|------|------|
| Nav2 全局/局部代价图、控制器 | `workspace_nav/config/nav2_params.yaml` |
| EKF / GPS 基准 | `workspace_ros/config/ekf.yaml`、`navsat.yaml` |
| 传感器帧 | `workspace_ros/config/static_transform.yaml`、URDF xacro |
| 船体动力学/推进 | `workspace_gz` xacro 与 Gazebo 插件参数 |

更全索引：`workspace_nav/config/boat_parameters_index.yaml`。

---

## 8. 实船

实船启动、MAVROS、`gnss_odom_map_tf`、Livox、`nav2_*_real*` 等：**见 USV_NAV 仓库文档**，本仓库不再附带这些 launch/配置。
