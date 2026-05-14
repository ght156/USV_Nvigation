你这个场景其实很标准：**笔记本主机 = 地面站（GCS）**，**船上 NX = 载体端（Onboard ROS2 + PX4 + Livox）**。
两者通过同一个局域网（路由器 / 交换机 / 网线直连）通信完全可行，而且这是最推荐的方式。

关键不是“能不能用局域网”，而是：

**你要控制 ROS2 的发现范围（Domain）、IP 连通性、TF 命名统一。**

------

# 一、推荐架构（你后续长期使用）

```text
地面站笔记本（Ubuntu + ROS2 + RViz + Nav2监控）
        │
        │  Wi-Fi / 网线 / 路由器（同一局域网）
        │
船载 NX（ROS2 + PX4 + Livox + usv_bridge）
```

------

# 二、最重要原则

## 原则 1：同一局域网

例如：

```bash
地面站：192.168.1.10
NX：   192.168.1.20
```

互相测试：

```bash
ping 192.168.1.20
ping 192.168.1.10
```

------

## 原则 2：ROS_DOMAIN_ID 必须一致

例如统一：

```bash
export ROS_DOMAIN_ID=20
```

这样双方 ROS2 节点互相可见。

------

## 原则 3：调试时可隔离，实船时关闭隔离

### 调试（本机不想被别人干扰）

```bash
export ROS_LOCALHOST_ONLY=1
```

### 实船（必须跨设备通信）

```bash
export ROS_LOCALHOST_ONLY=0
# 或直接 unset ROS_LOCALHOST_ONLY
```

------

# 三、建议你文档里直接写的标准配置

------

# 【地面站模式】

```bash
# ===== ROS2 Ground Station =====
source /opt/ros/humble/setup.bash
source ~/wuxihik_navigation/install/setup.bash

# 与船端统一 Domain
export ROS_DOMAIN_ID=20

# 必须关闭 localhost，否则看不到 NX
export ROS_LOCALHOST_ONLY=0

# 推荐 DDS 实现统一
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

------

# 【NX 船端模式】

```bash
# ===== ROS2 NX Boat =====
source /opt/ros/humble/setup.bash
source ~/boat_ws/install/setup.bash

export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

------

# 四、常用命令（建议直接放文档）

------

## 1. 查看网络是否通

```bash
ping 船IP
ping 地面站IP
```

### 原因：

先确认物理网络正常，否则 ROS2 永远发现不到。

------

## 2. 查看 ROS2 节点

```bash
ros2 node list
```

### 原因：

确认是否发现船端节点，例如：

```bash
/usv1/usv_bridge
/usv1/livox_lidar_publisher
```

------

## 3. 查看话题

```bash
ros2 topic list
```

### 原因：

确认传感器/控制数据是否可见。

------

## 4. 查看 TF

```bash
ros2 run tf2_tools view_frames
```

### 原因：

检查：

```text
map -> odom -> base_link -> livox_frame
```

是否完整。

------

## 5. 查看 PX4/MAVROS状态

```bash
ros2 topic echo /usv1/mavros/state
```

### 原因：

确认飞控/船控是否连接。

------

## 6. 查看点云

```bash
ros2 topic echo /livox/lidar --once
```

### 原因：

确认 Livox 正常。

------

# 五、最常见问题 + 原因

------

## 问题 1：看不到船端节点

### 原因：

```bash
ROS_DOMAIN_ID 不一致
```

------

## 问题 2：只能看到自己节点

### 原因：

```bash
ROS_LOCALHOST_ONLY=1
```

------

## 问题 3：TF 树断裂

### 原因：

```bash
base_link
usv1/base_link
```

命名不统一。

------

## 问题 4：节点很多但不是自己的

### 原因：

局域网里别人也在 ROS2 默认 Domain=0。

### 解决：

```bash
export ROS_DOMAIN_ID=20
```

------

# 六、推荐启动顺序（实船）

------

## NX 上：

```bash
ros2 launch px4_bridge ...
ros2 launch livox_ros_driver ...
ros2 launch usv_manager ...
```

------

## 地面站上：

```bash
rviz2
ros2 launch nav2 ...
ros2 topic list
```

------

# 七、最推荐的长期规范（非常重要）

## 不要再用默认：

```bash
ROS_DOMAIN_ID=0
```

## 建议固定：

```bash
你自己的船：20
备用船：21
仿真：88
```

例如：

```bash
真实船A：20
真实船B：21
办公室调试：88
```

------

# 八、一句话总结（适合写文档顶部）

```text
调试阶段：
ROS_LOCALHOST_ONLY=1
ROS_DOMAIN_ID=88

实船阶段：
ROS_LOCALHOST_ONLY=0
ROS_DOMAIN_ID=20

原则：
同局域网 + 同Domain + 同TF命名
```

------

# 九、你后续最省事的方式

建议你做两个脚本：

## debug_local.sh

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=88
```

## real_boat.sh

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=20
```

启动前：

```bash
source debug_local.sh
```

或：

```bash
source real_boat.sh
```

------

你这个架构本质上就是：

**NX负责感知与底层控制，笔记本负责地面站监控、规划、调试。**

这套方式非常适合你现在的 USV + Livox + PX4 + Nav2。
后面你如果要，我还可以继续帮你整理成：

**《USV 地面站/NX 联网部署手册》完整版**（包含 IP、ROS2、TF、MAVROS、Livox、Nav2 全流程）。