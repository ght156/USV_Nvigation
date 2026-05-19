# YILDIZ-USV（wuxihik_navigation）— 仿真栈

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-blue.svg)](https://releases.ubuntu.com/22.04/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

**Gazebo Garden** 仿真 USV：**Nav2**、**robot_localization（EKF）**、推力 `converter`。**实船 MAWROS / PX4 / NX** 在 **`USV_NAV`** 仓库维护（本仓已去掉实船独占 launch 与脚本，减少双份维护）。

## 仓库布局

```
src/YILDIZ-USV/
├── workspace_gz/      # Gazebo 世界、模型、桥接
├── workspace_ros/     # localization.launch.py、converter、感知脚本
└── workspace_nav/     # nav2.launch.py、地图、航点
```

## 依赖与编译

```bash
source /opt/ros/humble/setup.bash
cd <你的 colcon 工作区根目录>
colcon build --merge-install
source install/setup.bash
```

典型系统包：`ros-humble-nav2-*`、`ros-humble-robot-localization`、`ros-gz`（与 Gazebo Harmonic/Garden 对齐的安装名以官方为准）。

## 仿真快速启动（三终端）

每终端先：`source /opt/ros/humble/setup.bash` 与 `source install/setup.bash`。

| 终端 | 命令 |
|:---:|------|
| **1** | `ros2 launch workspace_gz simulation.launch.py` |
| **2** | `ros2 launch workspace_ros localization.launch.py use_sim_time:=true` |
| **3** | `ros2 launch workspace_nav nav2.launch.py use_sim_time:=true` |

可选第四终端：`ros2 run workspace_ros converter`（`/cmd_vel_nav` → 仿真推进器）。

默认地图：`workspace_nav/config/map.yaml`。换图时对 `nav2.launch.py` 传同一 `map:=` 路径。

## 文档

| 文档 | 说明 |
|------|------|
| [`../../docs/项目运行与联调.md`](../../docs/项目运行与联调.md) | **仿真**主入口 |
| [`docs/PROJECT_ARCHITECTURE_AND_NAV2.md`](docs/PROJECT_ARCHITECTURE_AND_NAV2.md) | 包分工与数据流 |
| [`../../docs/地图与GNSS-Nav2对齐说明.md`](../../docs/地图与GNSS-Nav2对齐说明.md) | 栅格、`ref_gnss*`、datum |

## 实船

见 **`USV_NAV`** 仓库 README 与 `docs/项目运行与联调.md`（实船版）。
