# wuxihik_navigation（YILDIZ-USV）

ROS 2 Humble：**Gazebo USV 仿真**（Nav2 + EKF + `converter`）。

- **本仓库**：仿真与算法验证。代码在 **`src/YILDIZ-USV/`**；感知 **`src/apriltag_localization/`**；归港编排 **`src/dock_mission/`**；精靠泊 **`src/usv_docking/`**。  
- **实船**（MAVROS / PX4 / NX / 专用地图与 bringup）：**[USV_NAV](https://github.com/ght156/USV_Navigation)**。

## 仿真入口

- [`docs/项目运行与联调.md`](docs/项目运行与联调.md)（含 **归港入泊 7~8 终端全栈**、**一键归港 `/dock/home`**）  
- [`docs/nav_task_interface.md`](docs/nav_task_interface.md) **§13**（**归港上层 Service/Topic 对接**）  
- [`src/dock_mission/README.md`](src/dock_mission/README.md) · [`src/usv_docking/README.md`](src/usv_docking/README.md) · [`src/apriltag_localization/README.md`](src/apriltag_localization/README.md)  
- [`docs/Nav2参数详解与调参指南.md`](docs/Nav2参数详解与调参指南.md)（RPP / lookahead 调参）  
- **阶段成果 / 写报告用**：[`docs/工作进度汇报.md`](docs/工作进度汇报.md)（含 **仿真 vs 实船** 一页纸）  
- 架构说明：[`src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md`](src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md)

## 编译

```bash
source /opt/ros/humble/setup.bash
cd <工作区根目录>
colcon build --merge-install
source install/setup.bash
```
