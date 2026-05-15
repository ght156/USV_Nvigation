# USV_NAV

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-blue.svg "Ubuntu 22.04 LTS")](https://releases.ubuntu.com/22.04/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg "ROS 2 Humble")](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg "Apache License 2.0")](./LICENSE.txt)

实船自主导航系统，基于 ROS 2 Humble 和 Navigation2，集成 MAVROS/PX4、EKF 融合定位、YOLOv11 目标检测与航点规划。

## 项目结构

```
.
├── docs/                    # 项目文档
├── map/                     # 栅格地图（HK 园区、华庄河道、老虎谭水库等）
├── tools/                   # 运维脚本
└── src/USV_NAV/
    ├── workspace_nav/       # Nav2 导航与航点转换
    │   ├── config/          # Nav2 参数（real_mavros）、地图 yaml
    │   ├── json/            # 航点/目标 JSON
    │   ├── launch/          # nav2_real_mavros.launch.py
    │   ├── map/             # 栅格图像（hk_map.pgm）
    │   └── workspace_nav/   # waypoint_transform / waypoint_with_state
    └── workspace_ros/       # 定位、MAVROS 桥接、感知
        ├── config/          # ekf.yaml, navsat.yaml, static_transform.yaml
        ├── launch/          # localization, mavros_px4, bringup 等
        ├── scripts/         # 节点脚本
        └── YOLOv11/         # YOLOv11 模型
```

## 依赖

### Step 1 — 安装 ROS 2 Humble

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Step 2 — 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-tf2 \
  ros-humble-tf2-ros
```

### Step 3 — 创建工作空间并克隆

```bash
mkdir -p ~/usv_nav_ws/src
cd ~/usv_nav_ws/src
git clone https://github.com/USV-NAV/USV_NAV.git
```

### Step 4 — 安装 Python 依赖

```bash
cd USV_NAV
pip install -r requirements.txt
```

### Step 5 — 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/usv_nav_ws
colcon build --merge-install
source ~/usv_nav_ws/install/setup.bash
```

## 快速启动（实船）

确保已 source 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/usv_nav_ws/install/setup.bash
```

### 1. 融合定位

```bash
ros2 launch workspace_ros localization.launch.py
```

### 2. Navigation2（实船 MAVROS）

```bash
ros2 launch workspace_nav nav2_real_mavros.launch.py
```

### 3. 话题桥接

```bash
ros2 run workspace_ros converter
```

### 4. 目标检测（GCS 发布 color_code 后）

```bash
ros2 run workspace_ros target_buoy
```

### 5. 航点转换与状态

```bash
ros2 run workspace_nav waypoint_transform
ros2 run workspace_nav waypoint_with_state
```

## 联调顺序

见 `tools/usv_nav_run_order.sh`

## CONTRIBUTING

贡献指南见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## REFERENCES

[Toward Maritime Robotic Simulation in Gazebo](https://wiki.nps.edu/display/BB/Publications?preview=/1173263776/1173263778/PID6131719.pdf)
