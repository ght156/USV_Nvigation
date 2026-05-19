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
source ~/USV_NAV/install/setup.bash   # 实船主仓；仿真副本为 ~/wuxihik_navigation

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
---

# 十、完整部署流程：从接线到编译运行

以下为**开发机（x86 笔记本）→ NX（ARM）首次完整部署**步骤。日常更新用 [§十一 增量同步脚本](#十一增量同步脚本)。

## 10.1 物理接线

**USB 直连** 和 **以太网** 两种方式可并存或择一（当前使用以太网）。

### 方式 A：USB 直连（主机 ↔ NX OTG 口）

```
  ┌──────────────────────────────────────┐
  │           船载 NX（ARM）               │
  │                                        │
  │  USB-A ─── PX4 TELEM2 (飞控,ttyACM0)   │
  │  USB-C/OTG ───┬───────────────────┐    │
  │  网口 ─── Livox MID-360（激光雷达） │    │
  │  电源 ─── 船载电池/稳压              │    │
  └───────────────┘                      │    │
                  │                      │    │
      USB 3.0 数据线（USB-C ↔ USB-A）    │    │
                  │                      │    │
  ┌───────────────┴──────────────────────┘    │
  │  地面站笔记本（x86 开发机）                  │
  └──────────────────────────────────────────┘
```

| 连接 | 说明 |
|------|------|
| NX USB-C（OTG/Device 口）↔ 笔记本 USB | **USB 虚拟以太网**，NX 默认 IP 端 `192.168.55.1`，主机自动获取 `192.168.55.100`（Jetson 默认网桥） |
| NX USB-A ↔ PX4 TELEM2 | MAVROS 串口（`/dev/ttyACM0`），波特率 57600 |
| NX 网口 ↔ Livox | 激光雷达（可选独立网段） |
| NX 电源 | Xavier NX: 19V DC；Orin NX 开发套件: USB-C PD |

### 方式 B：以太网（经路由器/交换机）

```
  NX 网口 ──── 路由器/交换机 ──── 笔记本（Wi-Fi 或 LAN）
```

需手动配固定 IP（见 10.2-B）。

## 10.2 网络配置

### 10.2-A USB 直连（Jetson 默认网桥，推荐首次使用）

Jetson 在 Device Mode 下默认会启动一个虚拟网桥（`l4tbr0`），IP 固定为 **`192.168.55.1`**，并通过 DHCP 给主机分配地址（通常 `192.168.55.100`）。

**笔记本侧：验证 USB 网口已识别**

```bash
# 笔记本上：查看新增的 USB 网络接口（通常 enp0s20f0u* 或 enx*）
ip link show | grep -E "enp|enx" --after=1
# 或
dmesg | tail -20 | grep -i "usb\|rndis\|cdc"

# 确认已获取 IP
ip addr show | grep "192.168.55"
# 预期输出: inet 192.168.55.100/24  ...
```

**如果主机没拿到 IP，手动设置：**

```bash
# 笔记本上（接口名用实际显示的替换）
sudo ip addr add 192.168.55.100/24 dev <接口名>
sudo ip link set dev <接口名> up
```

**验证互通：**

```bash
# 笔记本上
ping 192.168.55.1

# NX 上（通过串口或键盘显示器登录）
ping 192.168.55.100
```

### 10.2-B 以太网（网线直连，当前调试用）

NX 网口固定 IP `192.168.1.2`，笔记本同网段 `192.168.1.x`。

```bash
# NX 上设置固定 IP
sudo nmcli connection modify "Wired connection 1" \
  ipv4.addresses 192.168.1.2/24 \
  ipv4.method manual

sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "Wired connection 1"
```

验证：

```bash
# 笔记本
ping 192.168.1.2
ssh nvidia@192.168.1.2
```

### 10.2-C WiFi / 手机热点（下水实测用）—— 首次配置流程

**核心思路**：不知道 NX WiFi IP → 先用网线连 `192.168.1.2` → 远程给 NX 配好 WiFi → 拔网线 → NX 和笔记本都连手机热点。

#### 步骤 1：网线连接 NX，确认可达

```bash
# 笔记本上
ping 192.168.1.2
ssh nvidia@192.168.1.2
```

#### 步骤 2：在 NX 上配置自动连手机热点（通过 SSH 远程执行）

```bash
# 笔记本上 SSH 到 NX，执行以下命令：
ssh nvidia@192.168.1.2

# --- 以下在 NX 上执行 ---

# 查看 WiFi 网口名（通常是 wlan0）
nmcli device status | grep wifi

# 创建热点连接（替换 SSID 和密码）
sudo nmcli connection add \
  type wifi \
  con-name "boat-hotspot" \
  ifname wlan0 \
  ssid "你的手机热点名"

sudo nmcli connection modify "boat-hotspot" \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "你的热点密码"

# 设自动连接 + 高优先级
sudo nmcli connection modify "boat-hotspot" connection.autoconnect yes
sudo nmcli connection modify "boat-hotspot" connection.autoconnect-priority 10

# 立即连接看效果
sudo nmcli connection up "boat-hotspot"

# 查看获取到的 IP（记下来）
hostname -I
# 输出类似: 192.168.43.100 192.168.1.2
#            ↑ 热点分配的     ↑ 网线固定IP

exit
```

#### 步骤 3：笔记本连同一热点，拔网线验证

```bash
# 笔记本连上同一个手机热点后：
ping nvidia-desktop.local     # mDNS，推荐
# 或手动 ping NX 刚才获取的热点 IP
ping 192.168.43.100

ssh nvidia@nvidia-desktop.local   # 应该能通
```

#### 步骤 4：更新笔记本环境变量

```bash
# 把 NX_HOST 切到 mDNS 主机名（网线和热点都通用）
echo 'export NX_HOST=nvidia-desktop.local' >> ~/.bashrc
source ~/.bashrc

# 之后同步命令不变
cd ~/USV_NAV/tools
./sync_to_nx.sh --build
```

#### 步骤 5：之后每次下水

```
  手机开热点
    ├── NX 自动连（已配 auto-connect）
    └── 笔记本手动连
         ↓
  ping nvidia-desktop.local  ← 验证
  ./sync_to_nx.sh --build    ← 同步+编译
```

**手机热点常见子网**：

| 手机 | 子网 |
|------|------|
| Android 热点 | `192.168.43.x`、`192.168.42.x` |
| iPhone 热点 | `172.20.10.x` |
| 随身路由 | `192.168.1.x`、`192.168.8.x` |

> **为什么用 `nvidia-desktop.local` 而不是 IP？** 手机热点每次分配的 IP 可能不同，mDNS 主机名不受 IP 变动影响，网线和热点都能用。

## 10.3 笔记本上复制项目到 NX

### 方式 A：rsync（推荐，支持增量更新）

```bash
# 开发机上执行（替换 NX_USER；USB 直连 IP 默认 192.168.55.1，以太网改为实际 IP）
NX_USER="your_username"
NX_IP="192.168.1.2"

# 首次全量复制（排除 build/install/log 等编译产物）
rsync -avh --progress \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  --exclude='.git/' \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  ~/USV_NAV/ \
  ${NX_USER}@${NX_IP}:/home/${NX_USER}/USV_NAV/
```

### 方式 B：U 盘拷贝（无网络）

```bash
# 笔记本上
tar czf usv_nav.tar.gz \
  --exclude='build' --exclude='install' --exclude='log' --exclude='.git' \
  -C ~/ USV_NAV/

# 拷贝 tar.gz 到 U 盘 → 插入 NX → 解压
tar xzf /media/$USER/<U盘>/usv_nav.tar.gz -C ~/
```

### 方式 C：scp（简单小量）

```bash
scp -r ~/USV_NAV/src ${NX_USER}@${NX_IP}:/home/${NX_USER}/USV_NAV/
scp -r ~/USV_NAV/map ${NX_USER}@${NX_IP}:/home/${NX_USER}/USV_NAV/
scp -r ~/USV_NAV/tools ${NX_USER}@${NX_IP}:/home/${NX_USER}/USV_NAV/
```

## 10.4 NX 上安装依赖并编译

### SSH 登录 NX

```bash
ssh ${NX_USER}@192.168.55.1   # USB 直连默认 IP；以太网改为实际 IP
```

### 检查 ROS2 环境

```bash
# NX 上应已安装 ROS2 Humble（ARM 版）
ls /opt/ros/humble/
source /opt/ros/humble/setup.bash

# 检查 Python 依赖
python3 -c "import yaml, utm, numpy"

# 缺失的安装（按需）
pip3 install --user pyyaml utm numpy transforms3d
```

### 安装项目依赖的 ROS 包

```bash
# 本项目的关键依赖（NX 上未预装的需手动 apt install）
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-tf2-tools \
  ros-humble-teleop-twist-keyboard \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-geographic-msgs
```

### 编译项目

```bash
cd ~/USV_NAV
source /opt/ros/humble/setup.bash

# 编译（ARM 首次较慢，预留 5-10 分钟）
colcon build --symlink-install --executor sequential

# 编译成功标志
source install/setup.bash
ros2 pkg list | grep workspace
# 预期输出：
#   workspace_nav
#   workspace_ros
```

### 常见编译问题

| 现象 | 解决 |
|------|------|
| `ament_cmake` 找不到 | `sudo apt install ros-humble-ament-cmake` |
| `Python.h` 找不到 | `sudo apt install python3-dev` |
| `yaml.h` 找不到 | `sudo apt install libyaml-cpp-dev` |
| `tf2_geometry_msgs` 链接错误 | `sudo apt install ros-humble-tf2-geometry-msgs` |
| ARM 编译超时 | `colcon build --executor sequential --parallel-workers 2` |

## 10.5 NX 上验证

```bash
# NX 上
source /opt/ros/humble/setup.bash
source ~/USV_NAV/install/setup.bash

# 检查 launch 文件可发现
ros2 launch workspace_ros real_boat_bringup.launch.py --show-arguments
ros2 launch workspace_nav nav2_real_mavros.launch.py --show-arguments

# 检查关键依赖
ros2 pkg list | grep -E "nav2|mavros|tf2"
```

## 10.6 启动流程（NX + 笔记本）

### NX 终端 1：MAVROS

```bash
source /opt/ros/humble/setup.bash
source ~/USV_NAV/install/setup.bash
ros2 launch workspace_ros mavros_px4_usv.launch.py \
  fcu_url:=serial:///dev/ttyACM0:57600
```

### NX 终端 2：bringup

```bash
source /opt/ros/humble/setup.bash
source ~/USV_NAV/install/setup.bash
ros2 launch workspace_ros real_boat_bringup.launch.py \
  use_sim_time:=false \
  localization_backend:=mavros_odom \
  enable_nav2_cmd_vel_to_mavros:=true
```

### NX 终端 3：Nav2

```bash
source /opt/ros/humble/setup.bash
source ~/USV_NAV/install/setup.bash
ros2 launch workspace_nav nav2_real_mavros.launch.py
```

### 笔记本（地面站）：RViz 监控

```bash
source /opt/ros/humble/setup.bash
# 笔记本上不 source NX 的 setup.bash，仅需 ROS2 环境
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
rviz2
```

---

# 十一、增量同步与免密配置（脚本已内置在仓库）

项目 `tools/` 目录已包含两个脚本，无需手工创建。

## setup_nx_ssh.sh — 免密配置（只需运行一次）

```bash
cd ~/USV_NAV/tools
./setup_nx_ssh.sh
# 输入密码 nvidia，之后 ssh/rsync 不再需要密码
# 同时自动写入 /etc/hosts，使 nvidia-desktop.local 可解析
```

## sync_to_nx.sh — 日常增量同步

```bash
./sync_to_nx.sh              # 仅同步
./sync_to_nx.sh --build      # 同步 + NX 编译
./sync_to_nx.sh --discover   # 扫描局域网找 NX IP
```

**不同场景下的用法**：

```bash
# 网线直连（当前）
NX_HOST=192.168.1.2 ./sync_to_nx.sh --build

# 手机热点（下水后，IP 不定，用主机名）
NX_HOST=nvidia-desktop.local ./sync_to_nx.sh --build

# 如果不知道 NX 在哪，先扫描
./sync_to_nx.sh --discover
# 输出示例: ✓ mDNS 发现: nvidia-desktop.local → 192.168.43.100
# 然后: NX_HOST=nvidia-desktop.local ./sync_to_nx.sh --build
```

**环境变量**（可写入 `~/.bashrc` 设默认值）：

```bash
echo 'export NX_USER=nvidia' >> ~/.bashrc
echo 'export NX_HOST=nvidia-desktop.local' >> ~/.bashrc   # 热点用主机名
# 或网线时: echo 'export NX_HOST=192.168.1.2' >> ~/.bashrc
```

---

这个架构本质上就是：

**NX负责感知与底层控制，笔记本负责地面站监控、规划、调试。**