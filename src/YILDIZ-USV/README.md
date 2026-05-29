# YILDIZ-USV（wuxihik_navigation）— 仿真栈

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-blue.svg)](https://releases.ubuntu.com/22.04/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

**Gazebo Garden** 仿真 USV：**Nav2**、**robot_localization（EKF）**、推力 `converter`。**实船 MAVROS / PX4 / NX** 在 **`USV_NAV`** 仓库维护（本仓已去掉实船独占 launch 与脚本，减少双份维护）。

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
| **3** | `ros2 launch workspace_nav nav2.launch.py use_sim_time:=true`（地面站联调：**`enable_mission_bridge:=true`**） |

可选第四终端：`ros2 run workspace_ros converter`（`/cmd_vel_nav` → 仿真推进器）。

默认地图：`workspace_nav/config/map_hk.yaml`。换图时只对 `nav2.launch.py` 传 **`map:=`**；`enable_mission_bridge:=true` 时 mission 地图自动同源。须同步 `navsat.yaml` 的 `datum`（见 [`docs/项目运行与联调.md`](../../docs/项目运行与联调.md)）。

**mission 栈（可选）**：默认不加载 `mission_bridge_params_file` 即可；需改 debounce/GCS 话题时再传 YAML。勿与 Nav2 的 `params_file` 混用。

## 任务下发流程

地面站点击 **Run Mission** 后，`waypoint_publisher.py` **事件式 burst 发布**（0.2 s 间隔 × 3 次，带 `mission_id` + `explicit_replan`）然后自动退出。船端 `mission_bridge.py` 通过 `debounce` 模式（0.5 s 窗口）接收并去重，写 `waypoints.json` 后逐点调用 Nav2 FollowWaypoints。

取消任务通过地面站 **Cancel Nav** → `cancel_publisher.py`（一发即退 rclpy 节点）→ `mission_bridge` 清空缓冲并 preempt 当前 goal。

## 文档

| 文档 | 说明 |
|------|------|
| [`../../docs/项目运行与联调.md`](../../docs/项目运行与联调.md) | **仿真**主入口 |
| [`docs/PROJECT_ARCHITECTURE_AND_NAV2.md`](docs/PROJECT_ARCHITECTURE_AND_NAV2.md) | 包分工与数据流 |
| [`../../docs/工作进度汇报.md`](../../docs/工作进度汇报.md) | **绩效/工作报告**、仿真↔实船分工 |

## 实船

见 **`USV_NAV`** 仓库 README 与 `docs/项目运行与联调.md`（实船版）。
