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
└── src/USV_NAV/
    ├── workspace_nav/       # Nav2、地图 yaml、航点节点
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
| **3** | `ros2 launch workspace_nav nav2_real_mavros.launch.py` |

换海图时，终端 2 的 **`map_config_yaml:=`** 与终端 3 的 **`map:=`** 必须为**同一路径**（默认 `map_real_boat_hk.yaml`）。

**控制链**：Nav2 `controller` → `/cmd_vel_nav` →（可选 `velocity_smoother` → `/cmd_vel`）→ `nav2_cmd_vel_to_mavros`（默认 `/cmd_vel_nav`）→ PX4 OFFBOARD。

详细步骤、TF、HOME、换图、**`/cmd_vel`↔`/cmd_vel_nav` 调试命令**：[`docs/项目运行与联调.md`](../docs/项目运行与联调.md)。

## 可选节点（地面站 / 任务）

```bash
ros2 run workspace_nav waypoint_transform
ros2 run workspace_nav waypoint_with_state
ros2 run workspace_ros target_buoy    # 需 GCS 提供目标信息
```

## 文档索引

| 文档 | 用途 |
|------|------|
| [`docs/项目运行与联调.md`](../docs/项目运行与联调.md) | **主入口**：启动顺序、速度桥、换图、ROS_DOMAIN_ID |
| [`docs/实船调试.md`](docs/实船调试.md) | MAVROS、TF 复盘、HOME、避障、echo 调试 |
| [`docs/实船配置修改清单.md`](docs/实船配置修改清单.md) | 按文件改参数 |
| [`docs/PROJECT_ARCHITECTURE_AND_NAV2.md`](docs/PROJECT_ARCHITECTURE_AND_NAV2.md) | 实船数据流与 Nav2 要点 |
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
