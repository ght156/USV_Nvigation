# Livox MID360 连接与 ROS2 驱动编译记录

## 1. 背景环境

本次使用设备与环境：

- 雷达：Livox MID360
- 系统：Ubuntu + ROS2 Humble
- 工作空间：`~/ws_livox`
- 有线网卡：`eno1`
- ROS 驱动：`livox_ros_driver2`
- 通信方式：以太网 UDP

---

## 2. 确认电脑网卡

首先查看网卡：

```bash
ip link
```

输出中可见：

```text
2: eno1: <BROADCAST,MULTICAST,UP,LOWER_UP>
```

说明 `eno1` 是当前连接 MID360 的有线网卡。

继续查看 IP：

```bash
ip addr show eno1
```

最初电脑 IP 是：

```text
inet 192.168.0.77/24
```

该网段与雷达不一致，因此无法直接通信。

---

## 3. 初步设置电脑 IP

曾尝试将电脑 IP 设置为常见 Livox 默认网段：

```bash
sudo ip addr del 192.168.0.77/24 dev eno1
sudo ip addr add 192.168.1.50/24 dev eno1
sudo ip link set eno1 up
```

测试：

```bash
ping 192.168.1.100
ping 192.168.1.1
```

结果均为：

```text
Destination Host Unreachable
```

扫描网段：

```bash
nmap -sn 192.168.1.0/24
```

只发现本机：

```text
192.168.1.50
```

说明 MID360 不在 `192.168.1.xxx` 网段。

---

## 4. 检查物理链路

执行：

```bash
ethtool eno1
```

关键输出：

```text
Speed: 1000Mb/s
Duplex: Full
Link detected: yes
```

继续查看内核日志：

```bash
sudo dmesg | grep eno1
```

输出：

```text
eno1: Link is Up - 1Gbps/Full - flow control rx/tx
```

结论：

- 网线正常
- 雷达供电正常
- 电脑与雷达之间已经建立千兆物理链路
- 问题不在物理层，而在 IP 配置

---

## 5. 使用 tcpdump 找到 MID360 真实 IP

执行抓包：

```bash
sudo tcpdump -i eno1
```

抓到如下信息：

```text
PTPv2 ...
ARP, Request who-has 192.168.2.101 tell 192.168.2.191
```

含义：

```text
IP 为 192.168.2.191 的设备正在寻找 192.168.2.101
```

因此判断：

- MID360 当前真实 IP：`192.168.2.191`
- MID360 期望连接的 host IP：`192.168.2.101`

---

## 6. 设置电脑为雷达期望的 host IP

清空原 IP：

```bash
sudo ip addr flush dev eno1
```

设置电脑 IP：

```bash
sudo ip addr add 192.168.2.101/24 dev eno1
sudo ip link set eno1 up
```

验证：

```bash
ip addr show eno1
```

测试雷达连通性：

```bash
ping 192.168.2.191
```

成功输出：

```text
64 bytes from 192.168.2.191: icmp_seq=1 ttl=255 time=0.485 ms
64 bytes from 192.168.2.191: icmp_seq=2 ttl=255 time=0.844 ms
64 bytes from 192.168.2.191: icmp_seq=3 ttl=255 time=1.58 ms
```

结论：

```text
MID360 网络已经连通。
可以在网络有线设置那里新增加一个配置。目前增加为mid 360
```

---

## 7. 修改 MID360 配置文件

配置文件路径一般为：

```bash
~/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

注意：实际启动日志中显示驱动读取的是 `install` 目录下的配置文件：

```text
/home/ght/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

因此不能只改 `src` 目录下的 JSON，否则启动时可能不会生效。

正确配置如下：

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.2.101",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.2.101",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.2.101",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.2.101",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.2.191",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

关键点：

- `host_net_info` 里填电脑 IP：`192.168.2.101`
- `lidar_configs` 里的 `ip` 填雷达 IP：`192.168.2.191`

曾经错误写法：

```json
"lidar_configs" : [
  {
    "ip" : "192.168.2.101"
  }
]
```

这是错误的，因为这里的 `ip` 指的是雷达自身 IP，不是电脑 IP。

---

## 8. 启动驱动时遇到 bind failed

启动命令：

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

曾出现错误：

```text
bind failed
Failed to init livox lidar sdk.
Init lds lidar fail!
```

含义：

```text
驱动绑定 UDP 端口失败。
```

常见原因：

- 上一次驱动进程没有关干净
- Livox 相关 UDP 端口被占用
- 配置文件中的 host IP 或端口异常

排查命令：

```bash
pkill -f livox_ros_driver2
pkill -f rviz2
```

查看端口占用：

```bash
sudo lsof -iUDP:56101
sudo lsof -iUDP:56201
sudo lsof -iUDP:56301
sudo lsof -iUDP:56401
sudo lsof -iUDP:56501
```

如有进程占用，杀掉对应 PID：

```bash
sudo kill -9 PID
```

---

## 9. 编译驱动时遇到的问题

### 9.1 直接 colcon build 报 livox_interfaces 相关错误

执行：

```bash
cd ~/ws_livox
colcon build --symlink-install
```

曾报错：

```text
LIVOX_INTERFACES_INCLUDE_DIRECTORIES
used as include directory ...
```

这是因为 `livox_ros_driver2` 工程结构和接口文件没有正确准备。

---

### 9.2 重新 clone 后直接 colcon build 报 package.xml 不存在

执行：

```bash
cd ~/ws_livox/src
rm -rf livox_ros_driver2
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/ws_livox
rm -rf build install log
colcon build --symlink-install
```

报错：

```text
CMake Error: File /home/ght/ws_livox/src/livox_ros_driver2/package.xml does not exist.
Packages installing interfaces must include
'<member_of_group>rosidl_interface_packages</member_of_group>' in their package.xml
```

原因：

```text
官方 livox_ros_driver2 仓库不能直接裸跑 colcon build，需要先执行官方 build.sh 脚本，为对应 ROS 版本准备 package.xml 等文件。
```

正确方式：

```bash
cd ~/ws_livox/src/livox_ros_driver2
./build.sh humble
```

如果没有执行权限：

```bash
chmod +x build.sh
./build.sh humble
```

---

## 10. Livox SDK2 版本不匹配问题

执行：

```bash
cd ~/ws_livox/src/livox_ros_driver2
./build.sh humble
```

曾报错：

```text
error: ‘kLivoxLidarTypeMid360s’ is not a member of ‘LivoxLidarDeviceType’
```

原因：

```text
当前 livox_ros_driver2 代码比本机安装的 Livox-SDK2 头文件更新，driver 中引用了 kLivoxLidarTypeMid360s，但本机 SDK2 中没有这个枚举。
```

推荐修复方式：更新本机 Livox-SDK2。

```bash
cd ~
rm -rf Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

然后重新编译驱动：

```bash
cd ~/ws_livox/src/livox_ros_driver2
./build.sh humble
```

如果仍然报同样错误，可临时修改源码兼容旧 SDK：

```bash
cd ~/ws_livox/src/livox_ros_driver2
grep -n "kLivoxLidarTypeMid360s" src/comm/pub_handler.cpp
gedit src/comm/pub_handler.cpp
```

将：

```cpp
} else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360||dev_type==LivoxLidarDeviceType::kLivoxLidarTypeMid360s) {
```

改为：

```cpp
} else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360) {
```

再编译：

```bash
cd ~/ws_livox/src/livox_ros_driver2
./build.sh humble
```

---

## 11. 关于 colcon build 和 build.sh 的结论

`livox_ros_driver2` 并不是完全不能用 `colcon build`，而是官方仓库需要先通过：

```bash
./build.sh humble
```

完成 ROS2 Humble 对应文件和编译流程准备。

更稳妥的方式是直接使用官方脚本：

```bash
cd ~/ws_livox/src/livox_ros_driver2
./build.sh humble
```

不建议一开始直接在工作空间根目录裸跑：

```bash
colcon build --symlink-install
```

因为容易遇到：

- `package.xml does not exist`
- `livox_interfaces` 路径错误
- SDK2 头文件与 driver 源码不匹配
- 安装目录配置文件未同步

---

## 12. 最终启动流程

每次使用前，可按下面流程执行。

### 12.1 设置电脑 IP

```bash
sudo ip addr flush dev eno1
sudo ip addr add 192.168.2.101/24 dev eno1
sudo ip link set eno1 up
```

### 12.2 测试雷达连通性

```bash
ping 192.168.2.191
```

### 12.3 加载 ROS2 工作空间

```bash
cd ~/ws_livox
source install/setup.bash
```

### 12.4 启动 MID360

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

或者只启动消息发布：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 12.5 查看话题

```bash
ros2 topic list
```

常见话题：

```text
/livox/lidar
/livox/imu
```

### 12.6 查看点云频率

```bash
ros2 topic hz /livox/lidar
```

---

## 13. 常用排查命令汇总

查看网卡：

```bash
ip link
ip addr show eno1
```

查看物理链路：

```bash
ethtool eno1
sudo dmesg | grep eno1
```

扫描网段：

```bash
nmap -sn 192.168.2.0/24
```

抓包查看雷达真实 IP：

```bash
sudo tcpdump -i eno1
```

查看端口占用：

```bash
sudo lsof -iUDP:56101
sudo lsof -iUDP:56201
sudo lsof -iUDP:56301
sudo lsof -iUDP:56401
sudo lsof -iUDP:56501
```

关闭残留进程：

```bash
pkill -f livox_ros_driver2
pkill -f rviz2
```

重新编译驱动：

```bash
cd ~/ws_livox/src/livox_ros_driver2
./build.sh humble
```

加载环境：

```bash
source ~/ws_livox/install/setup.bash
```

---

## 14. 本次最终正确参数

```text
电脑网卡：eno1
电脑 IP：192.168.2.101
MID360 IP：192.168.2.191
ROS2 版本：Humble
驱动：livox_ros_driver2
启动命令：ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

---

## 15. 经验总结

本次连接过程中的核心经验：

1. `ip link` 和 `ethtool` 用于确认物理链路是否正常。
2. `ping` 不通时，不能立刻判断雷达没启动，应使用 `tcpdump` 抓包。
3. `tcpdump` 中的 `ARP, Request who-has 192.168.2.101 tell 192.168.2.191` 是定位真实 IP 的关键。
4. `host_net_info` 填电脑 IP，`lidar_configs.ip` 填雷达 IP，二者不能写反。
5. `livox_ros_driver2` 推荐用官方 `./build.sh humble` 编译。
6. 如果出现 `kLivoxLidarTypeMid360s` 报错，优先更新 Livox-SDK2。
7. 启动时报 `bind failed`，优先检查端口占用和残留进程。

