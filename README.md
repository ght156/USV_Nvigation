# wuxihik_navigation（YILDIZ-USV）

ROS 2 Humble：**Gazebo USV 仿真**（Nav2 + EKF + `converter`）。

- **本仓库**：仿真与算法验证。代码在 **`src/YILDIZ-USV/`**；从该目录阅读 [`README.md`](src/YILDIZ-USV/README.md)。  
- **实船**（MAVROS / PX4 / NX / 专用地图与 bringup）：**[USV_NAV](https://github.com/ght156/USV_Navigation)**。

## 仿真入口

- [`docs/项目运行与联调.md`](docs/项目运行与联调.md)  
- 架构说明：[`src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md`](src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md)

## 编译

```bash
source /opt/ros/humble/setup.bash
cd <工作区根目录>
colcon build --merge-install
source install/setup.bash
```
