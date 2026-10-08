# USV_NAV

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-blue.svg)](https://releases.ubuntu.com/22.04/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](./LICENSE.txt)

**实船**自主导航：ROS 2 Humble、Navigation2、MAVROS/PX4、Livox 避障、航点与地面站联调。

> **本仓库不做 Gazebo 仿真。** 仿真栈在独立副本 [`wuxihik_navigation`](https://github.com/ght156/USV_Nvigation)（包名 `YILDIZ-USV`）中维护。

## 项目结构

```
USV_NAV/
├── docs/                    # 联调、NX 部署、地图/GNSS 说明
├── map/                     # 栅格图源（如 HK 园区）
└── src/
    ├── apriltag_localization/   # AprilTag 船坞定位（感知）
    ├── m_common/                # 公共消息/Service/工具库
    └── USV_NAV/
        ├── workspace_nav/       # Nav2、地图 yaml、mission 栈
        └── workspace_ros/       # MAVROS 桥接、gnss_odom_map_tf、速度桥
```

## 依赖与编译

见下文 **Step 1–5**；系统包需含 `ros-humble-mavros`、`ros-humble-nav2-bringup`、`ros-humble-robot-localization`（仅当将来补 EKF launch 时）。

```bash
source /opt/ros/humble/setup.bash
cd ~/USV_NAV   # 或你的工作区根目录
colcon build --merge-install
source install/setup.bash
```

## 实船快速启动（三终端）

每个终端先：

```bash
source /opt/ros/humble/setup.bash
source ~/USV_NAV/install/setup.bash   # 按实际路径
```

| 终端 | 命令 |
|:---:|------|
| **1** | `ros2 launch workspace_ros mavros_px4_usv.launch.py fcu_url:=serial:///dev/ttyACM0:57600` |
| **2** | `ros2 launch workspace_ros real_boat_bringup.launch.py use_sim_time:=false enable_nav2_cmd_vel_to_mavros:=true` |
| **3** | `ros2 launch workspace_nav nav2_boat.launch.py enable_mission_bridge:=true`（需要 mission 栈时） |

换海图时，终端 2 的 **`map_config_yaml:=`** 与终端 3 的 **`map:=`** 必须为**同一路径**；mission 的 `map_yaml_path` **随终端 3 的 `map:=` 自动一致**，无需再配。

mission 栈（mission_bridge + nav_status_aggregator + zone_manager/zone_monitor）**默认不启动**，需要 GCS/上层对接时加 `enable_mission_bridge:=true`。mission 参数默认 `mission_stack.chengfeng.yaml`，一般不必手传。详见 [`docs/项目运行与联调.md`](../docs/项目运行与联调.md)。

**控制链**：Nav2 `controller` → `/cmd_vel_nav` →（可选 `velocity_smoother` → `/cmd_vel`）→ `nav2_cmd_vel_to_mavros`（默认 `/cmd_vel_nav`）→ PX4 OFFBOARD。

详细步骤、TF、HOME、换图、**`/cmd_vel`↔`/cmd_vel_nav` 调试命令**：[`docs/项目运行与联调.md`](../docs/项目运行与联调.md)。

## 换船 / 切换定位源（改哪些文件）

### A. 换船（以新船兴飞 xingfei 为例）

**1) Nav2 参数**：`src/USV_NAV/workspace_nav/config/nav2_params_xingfei.yaml`
（已建好，复制自 `nav2_params_chengfeng.yaml`）。启动时换文件路径即可切换：

```bash
ros2 launch workspace_nav nav2_boat.launch.py \
  params_file:=$(ros2 pkg prefix workspace_nav)/share/workspace_nav/config/nav2_params_xingfei.yaml \
  map:=$(ros2 pkg prefix workspace_nav)/share/workspace_nav/config/map_xingfei.yaml
```

> `params_file` 要给绝对路径（`$(ros2 pkg prefix workspace_nav)/share/...` 或直接写全路径）。
> 船上用 `--symlink-install` 时，**新增** yaml 需要重跑一次 `colcon build --packages-select workspace_nav`
> （已存在文件改内容则免编译）。

**这份文件里兴飞要改的项**（其余保持与基线一致）：

| 位置 | 现基线值 | 说明 |
|---|---|---|
| `local_costmap` / `global_costmap` 的 `footprint`（各一处） | `[[2.72, 0.95], [2.72, -0.95], [-0.4, -0.95], [-0.4, 0.95]]` | 船体尺寸（前 2.72 / 后 0.4 / 半宽 0.95） |
| `inflation_radius` | local `5.0` / global `4.0` | 与 footprint 匹配 |
| RPP `min_lookahead_dist` / `max_lookahead_dist` / `lookahead_time` | `5.0` / `8.0` / `16.0` | 与兴飞最小转弯半径、最高速匹配 |
| `desired_linear_vel`、`regulated_linear_scaling_min_speed` | `0.5` / `0.1` | 现场按实测运动能力（原船实测转向封顶 ~0.13 rad/s） |
| `velocity_smoother` 的 `max_velocity` / `max_accel` | `[0.5, 0.0, 0.5]` | 与 `ardupilot_velocity_bridge` 的 `max_linear_x` / `max_angular_z` 同量级 |

**2) 地图（三处同源）**：Nav2 `map:=` ↔ bringup `map_config_yaml:=` ↔ SLAM（lightning）`map_dir:=`
都必须指向同一份 `map_xingfei.yaml`。

**3) 传感器安装外参**（不在 Nav2 参数里）：`src/usv_transforms/urdf/cleaning_boat.xacro`
→ 复制为 `cleaning_boat_xingfei.xacro`，改 `livox_xyz` / `livox_rpy`（倒装与否）/ `imu_xyz` /
`gps_xyz` / `camera_back_video_*`，启动时用 `robot_description_file:=` 指定。

**4) 归港栈**：

| 项 | 做法 |
|---|---|
| 速度/坞尺寸/泊位几何 | 复制 `usv_docking/config/docking.yaml` → `docking_xingfei.yaml`，`ros2 launch usv_docking docking.launch.py params_file:=...` |
| 泊位经纬度 | 复制 `dock_mission/config/dock_database.yaml` → `dock_database_xingfei.yaml`；用 `ros2 launch dock_mission dock_mission.launch.py profile:=xingfei`（overlay `dock_mission_xingfei.yaml` 里把 `dock_database_path` 指过去） |

### B. 切换定位源 MAVROS ⇄ SLAM

**1) odom 话题**：导航、归港各有 launch 参数 `odom_topic`，一处传值全链路生效：

```bash
ros2 launch workspace_nav nav2_boat.launch.py odom_topic:=/slam/localization ...  # Nav2 三处 + mission 栈
ros2 launch usv_docking docking.launch.py odom_topic:=/slam/localization          # 归港栈
```

- **不传（默认空）**= 尊重各 yaml 里的值；基线 `nav2_params_chengfeng.yaml` / `mission_stack.chengfeng.yaml` / `docking.yaml` 默认都是 `/mavros/gps_input/local`（MAVROS 模式）。
- **非空时**：Nav2 侧改写 `bt_navigator` / `controller_server` / `velocity_smoother` 三处 `odom_topic`；mission 侧覆盖 `mission_bridge` / `nav_status_aggregator` / `zone_monitor`；归港侧覆盖 `docking_fsm.odom_speed_topic`。
- 要固定换源，也可以直接改 yaml 里的 `odom_topic`（各文件内注释标了位置）。

**2) TF 两段只能有一个发布者**（现场执行，不是改文件）：

- `map→odom`：改由 SLAM 发 → 起 bringup 时加 **`use_gnss_map_odom_tf:=false`**（或不起 `usv_localization`/`gnss_odom_map_tf` 那一窗口）
- `odom→base_link`：改由 `slam_nav_bridge` 发 → **`ros2 param set /mavros/local_position tf.send false`**

**3) 频率判据**：`nav_status_aggregator` 的 odom 频率下限已参数化为 **`min_odom_hz`**（默认 8.0，
`mission_stack.chengfeng.yaml` 里可调）。实测 MAVROS ≈10Hz、SLAM ≈20-29Hz，默认 8Hz 两者都够用；
真断流由 `odom_timeout`(2s) → LOST 兜底。

**4) 不用动的**：归港的位姿估计走 TF（`odom→base_link` + `base_link→dock_frame`），
两种定位源帧名相同；`ardupilot_velocity_bridge` 只吃 `/cmd_vel_nav`，与定位源无关。

## 地面站 / 任务（mission_bridge）

终端 3 加 `enable_mission_bridge:=true` 时启动 `mission_bridge` + `nav_status_aggregator` + `zone_manager`/`zone_monitor`（`/waypoint`、`/nav_status`）。**勿与**下列 legacy 节点并行：

```bash
# 仅 enable_mission_bridge:=false 时使用
ros2 run workspace_nav waypoint_transform
ros2 run workspace_nav waypoint_with_state
ros2 run workspace_ros target_buoy
```

### mission 栈做什么

- **mission_bridge**：接收航线（GCS `/waypoint` 话题防抖、上层 3 个 Service、Decision 的 `NavigateTask` action），经纬度→map 坐标转换，写 `waypoints.json`，逐点调 Nav2 `FollowWaypoints`；状态机 `WAITING_SYSTEM/IDLE/RUNNING/PAUSED/COMPLETED/FAILED`，电子围栏预校验，发布 `/mission_bridge/state` 与 `/mission_bridge/status_detail`。
- **nav_status_aggregator**（纯观察，不干预导航）：汇总任务/定位/规划/控制状态 → `/nav_status`（2Hz NavStatus）+ `/task_event`（JSON 事件：TASK_STARTED/COMPLETED/FAILED/CANCELLED/PAUSED/RESUMED、ALARM_RAISED/CLEARED）。
- **zone_manager / zone_monitor**：作业区/禁止区/硬边界电子围栏（KeepoutFilter 掩码 + 越界监控）。

### 定位降级 / 告警行为（nav_status_aggregator）

判定 `localization_overall` 三档：**LOST**（TF 断 / odom 超 `odom_timeout`=2s / 位置协方差 >10）、**DEGRADED**（协方差 >1 / GPS fix 差 / odom <20Hz / GPS 超 `gps_timeout`=5s）、**GOOD**。

- 进入 DEGRADED → `/task_event` 报 `ALARM_RAISED LOC_DEGRADED`（WARN）；
- 进入 LOST → `ALARM_RAISED LOC_LOST`（ERROR），`last_error=LOC_LOST`；恢复 GOOD 时报 `ALARM_CLEARED`。
- RUNNING 中位移不足判 `CTRL_STUCK`：**阈值默认自动同步 Nav2 `controller_server` 的 progress_checker**（`required_movement_radius`/`movement_time_allowance`，当前 2.0m/30.0s，每 10s 轮询，改 Nav2 参数即生效）；Nav2 不可达时回退本地 `stuck_movement_radius_m`/`stuck_progress_timeout`。planner ABORTED → `PLAN_FAILED`（事件驱动，无阈值）。
- **aggregator 只上报、不停车**；真正影响执行的是 mission_bridge：RUNNING 中 TF/FollowWaypoints 不可用会主动中止任务回 WAITING_SYSTEM（NavigateTask 返回 `RESULT_LOCALIZATION_LOST`）。
- 已知判据问题：GPS fix 用 `status.status < 3` 判 DEGRADED，而 mavros 按标准填 0/-1，**当前会恒判 DEGRADED**（见 `mission_stack.chengfeng.yaml` 注释，待改判据或换 fix_type 源）。

mission_bridge 全部参数的逐条生效核对见 [`docs/mission_bridge与上层对接.md`](docs/mission_bridge与上层对接.md) §4。

## 文档索引

| 文档 | 用途 |
|------|------|
| [`docs/项目运行与联调.md`](../docs/项目运行与联调.md) | **主入口**：启动顺序、速度桥、换图、ROS_DOMAIN_ID |
| [`docs/Nav2参数详解与调参指南.md`](../docs/Nav2参数详解与调参指南.md) | **Nav2 实船调参**：`nav2_params_chengfeng.yaml` |
| [`docs/实船调试.md`](docs/实船调试.md) | MAVROS、TF 复盘、HOME、避障、echo 调试 |
| [`docs/实船配置修改清单.md`](docs/实船配置修改清单.md) | 按文件改参数 |
| [`docs/PROJECT_ARCHITECTURE_AND_NAV2.md`](docs/PROJECT_ARCHITECTURE_AND_NAV2.md) | 实船数据流与 Nav2 要点 |
| [`docs/mission_bridge与上层对接.md`](docs/mission_bridge与上层对接.md) | mission 栈接口、启动、**参数逐条生效核对** |
| [`docs/导航与归港异常告警.md`](../docs/导航与归港异常告警.md) | `/nav_status` 告警码与降级行为 |
| [`docs/地图与GNSS-Nav2对齐说明.md`](../docs/地图与GNSS-Nav2对齐说明.md) | 栅格、datum、`ref_gnss*` |
| [`docs/USV 地面站NX 联网部署手册.md`](../docs/USV%20地面站NX%20联网部署手册.md) | NX 部署 |

## 安装步骤（新环境）

### Step 1 — ROS 2 Humble

[官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Step 2 — 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-tf2 \
  ros-humble-tf2-ros
```

### Step 3 — 克隆

```bash
mkdir -p ~/usv_nav_ws/src && cd ~/usv_nav_ws/src
git clone https://github.com/ght156/USV_Nvigation.git USV_NAV
```

### Step 4 — Python 依赖

```bash
cd USV_NAV && pip install -r requirements.txt
```

### Step 5 — 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/usv_nav_ws && colcon build --merge-install
source install/setup.bash
```

## CONTRIBUTING

见 [CONTRIBUTING.md](CONTRIBUTING.md)。
