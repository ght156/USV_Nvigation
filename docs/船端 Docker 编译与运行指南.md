# 船端 Docker 编译与运行指南（NX）

> 实测环境（2026-09-18，新 NX）：主机名 `MEC-MB002`、用户 `hkzl`、
> Ubuntu 22.04.5 LTS aarch64、船端 LAN 地址 `192.168.2.101`（eth1）。
> 镜像：`usv-runtime:jp6-cu126-humble`（约 20.9 GB，ROS 2 Humble + CUDA 12.6 + Livox SDK2）。
> 部署包：`~/USV/deploy/`（`deploy.sh`、`usv-runtime.tar.zst`、`usv-install-arm.tar.gz`）。

---

## 1. 先搞清分工，别把产物写进容器

| 东西 | 船端位置 | 容器内位置 | 说明 |
|------|----------|------------|------|
| Docker 镜像 | `docker images` | — | 只提供运行环境（ROS 2、CUDA、PCL、Livox SDK），不含我们的程序 |
| 交叉编译好的程序 | `~/USV/usv/install` | `/opt/usv/install` | `deploy.sh` 解出来的 install（含 `livox_ros_driver2`、`perception_launch` 等） |
| 常驻容器 | — | 容器名 `distracted_johnson` | `--network host`，挂载 `~/USV/usv → /opt/usv` |
| 自编源码 / 工具 | `~/ws_livox` | 编译时挂成 `/ws` | SDK 源码、驱动源码、ARM64 改 IP 工具 |

两条铁律：

- **不要用 `docker cp` 或容器内 `make install` 当交付**：那些只写在容器的临时层，容器一删就没了。
- **要持久就挂载**：`docker run --rm -v ~/ws_livox:/ws ...`，产物写在 `/ws/...` 下即落在船端磁盘。

---

## 2. 进入 docker 调试（最常用）

```bash
ssh hkzl@192.168.2.101
docker ps                                # 确认容器名（当前为 distracted_johnson）
docker exec -it distracted_johnson bash
```

容器里 `docker exec` **不会**带 entrypoint 设置好的 ROS 环境，所以每次都要自己 source：

```bash
source /opt/ros/humble/setup.bash
source /opt/usv/install/setup.bash

ros2 topic list
ros2 node list
ros2 topic hz /livox/lidar
usv-check            # 环境自检（GPU/CUDA/ROS/Livox SDK；容器内直接可执行）
```

只想跑一条命令、不想进交互 shell：

```bash
docker exec -it distracted_johnson bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/usv/install/setup.bash && ros2 topic list'
```

---

## 3. 在 docker 里编译

统一套路：`docker run --rm -v <船端源码目录>:<容器内挂载点> -w <工作目录> <镜像> bash -lc '<命令>'`。
因为 `--rm` + 挂载，编译产物落在船端磁盘，镜像本身不变。

### 3.1 编 Livox-SDK2（ARM64）

```bash
docker run --rm -v $HOME/ws_livox:/ws -w /ws/Livox-SDK2 usv-runtime:jp6-cu126-humble bash -lc '
  cmake -S . -B build_arm -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/ws/sdk_install &&
  cmake --build build_arm -j4 &&
  cmake --install build_arm'
```

产物：`~/ws_livox/sdk_install/{lib,include}`。

### 3.2 编“改雷达 IP”工具（ARM64）

```bash
docker run --rm -v $HOME/ws_livox:/ws -w /ws/src/livox_ros_driver2/tools \
  usv-runtime:jp6-cu126-humble bash -lc \
  'SDK_PREFIX=/ws/sdk_install OUT=set_lidar_ip_arm bash build_set_lidar_ip.sh'
```

产物：`~/ws_livox/src/livox_ros_driver2/tools/set_lidar_ip_arm`。
源码与脚本同步自本仓库 `scripts/livox/`。

### 3.3 编 livox_ros_driver2（一般不需要）

**日常调试不用编驱动**：`~/USV/usv/install/livox_ros_driver2` 已经是部署包里编好的成品，
容器里 `source /opt/usv/install/setup.bash` 后就能 `ros2 launch`。

确实要改源码重编时：

```bash
# 1) 同步源码（在笔记本上执行）
rsync -a --exclude .git /home/ght/ws_livox/src/livox_ros_driver2 hkzl@192.168.2.101:~/ws_livox/src/

# 2) 容器里编译（先 build.sh 生成 ROS2 版 package.xml / launch/，再 colcon）
docker run --rm -v $HOME/ws_livox:/ws -w /ws usv-runtime:jp6-cu126-humble bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /ws/src/livox_ros_driver2 && ./build.sh humble &&
  cd /ws && colcon build --symlink-install --packages-select livox_ros_driver2 \
    --cmake-args -DLIVOX_LIDAR_SDK_LIBRARY=/ws/sdk_install/lib/liblivox_lidar_sdk_shared.so \
                 -DLIVOX_LIDAR_SDK_INCLUDE_DIR=/ws/sdk_install/include'
```

已知坑：在新环境里 colcon 可能报
`LIVOX_INTERFACES_INCLUDE_DIRECTORIES ... set to NOTFOUND`（该 CMake 取 rosidl 目标属性的写法较脆）。
这时优先用官方 `./build.sh humble` 编，或者直接用部署包里那份预编译驱动——不影响跑雷达。

---

## 4. 运行激光雷达

**这台雷达是 MID-360S（dev_type 35），必须用 `msg_MID360s_launch.py`。**
用 `msg_MID360_launch.py`（`MID360_config.json`，type 8）时只会看到发现/搜索广播，
数据流建不起来，现象是起得来、`/livox/lidar` 不出数据。

```bash
# 前台跑（调试用）
docker exec -it distracted_johnson bash -lc '
  source /opt/ros/humble/setup.bash && source /opt/usv/install/setup.bash &&
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py'

# 后台跑
docker exec -d distracted_johnson bash -lc '
  source /opt/ros/humble/setup.bash; source /opt/usv/install/setup.bash;
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py > /tmp/livoxs.log 2>&1'

# 验证
docker exec -it distracted_johnson bash -lc '
  source /opt/ros/humble/setup.bash; source /opt/usv/install/setup.bash;
  ros2 topic list | grep livox; timeout 8 ros2 topic hz /livox/lidar'

# 停止
docker exec distracted_johnson pkill -f livox_ros_driver2_node
```

正常表现：`/livox/lidar` 约 **10 Hz**，`/livox/imu` 存在，日志里出现
`successfully set lidar attitude / enable Livox Lidar imu, ip: 192.168.2.191`。

网络参数（容器内）：
`/opt/usv/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json`
—— `host_net_info.host_ip = 192.168.2.101`（NX 自己），`lidar_configs[0].ip = 192.168.2.191`（雷达）。

> 主栈 `usv_test/launch/cleaning_boat_test.launch.py` 里 livox 那一行目前是**注释掉**的，
> 所以雷达驱动按现状要单独起。

---

## 5. 现场改雷达 IP

Livox 的 IP 存在雷达里。现场常见情况是“雷达还在 1 网段、NX 在 2 网段”，
这时先在 NX 上临时借一个雷达网段的地址去够到它，改完再删掉。

```bash
# 在船端 NX 上（脚本与 scripts/livox/flash_lidar_ip.sh 一致）
bash ~/ws_livox/flash_lidar_ip.sh eth1 192.168.2.191 192.168.1.10
```

正常输出与判据：

```
[discover] lidar found: sn=... dev_type=35 current_ip=192.168.1.162
SUCCESS: lidar ip set to 192.168.2.191; requesting reboot ...
reboot ack: status=0 ret_code=0
OK：192.168.2.191 已上线（约 15s）
```

**关键坑（2026-09-18）**：只发 set-ip 时雷达会回 ack `ret_code=0`，但 **IP 不变**。
官方样例是在 set-ip 回调里再调一次 `LivoxLidarRequestReboot`；本仓库
`scripts/livox/set_lidar_ip.cpp` 已包含这一步。旧版工具刷完不会重启，别被 `SUCCESS` 骗了。

不知道雷达当前 IP 时，先抓包看它的 ARP：雷达会持续发
`who-has <它期望的 host> tell <雷达自己的 IP>`，两个 IP 就都知道了（见 §7）。

---

## 6. 常见故障

| 现象 | 原因 / 处理 |
|------|-------------|
| `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED` | NX 换机或重装，指纹变了。确认是本机后 `ssh-keygen -R 192.168.2.101` |
| `ssh: No route to host` / ping 不通 | 用 `ethtool <网口>` 看 Link detected；用抓包看雷达是否在发 ARP |
| 手工 `ip addr add` 后 SSH 断一下 | 该网口受 NetworkManager 托管，手动改动会触发重配；持久改动用 `nmcli con mod`，临时用别的网口 |
| 驱动起来了但 `/livox/lidar` 没数据 | launch 用错（MID360 vs MID360S）；或 `host_net_info.host_ip` 不是本机地址；或雷达上报目标还指着旧 host |
| `bind failed` | 上次进程残留占端口：`docker exec <容器> pkill -f livox_ros_driver2_node` |
| 看不到雷达但能 ping 通 | 检查 `lidar_configs[0].ip` 与雷达实际 IP 是否一致 |

---

## 7. 船上没有 tcpdump 时怎么抓包

镜像/系统里没装 `tcpdump`，用 python 原始套接字（需 root）：

```bash
sudo python3 -c '
import socket,struct,time,collections
s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(3)); s.bind(("eth1",0)); s.settimeout(1)
end=time.time()+15; c=collections.Counter()
while time.time()<end:
    try: f=s.recv(2048)
    except socket.timeout: continue
    if len(f)<34: continue
    et=struct.unpack("!H",f[12:14])[0]; b=f[14:]
    if et==0x0806 and len(b)>=28:
        c[("ARP",socket.inet_ntoa(b[14:18]),socket.inet_ntoa(b[24:28]))]+=1
    elif et==0x0800:
        c[("IP",socket.inet_ntoa(b[12:16]),socket.inet_ntoa(b[16:20]),b[9])]+=1
for k,v in c.most_common(10): print(v,k)'
```

---

## 8. 路径与文件清单

船端：

- `~/USV/deploy/deploy.sh` —— 部署脚本（导入镜像 + 安装 install）
- `~/USV/usv/install` —— 程序（容器内 `/opt/usv/install`）
- `~/ws_livox/Livox-SDK2` —— SDK 源码；`~/ws_livox/sdk_install` —— 编好的 ARM64 SDK
- `~/ws_livox/src/livox_ros_driver2` —— 驱动源码 + `tools/set_lidar_ip*.cpp`
- `~/ws_livox/flash_lidar_ip.sh` —— 一键改雷达 IP

本仓库：

- `scripts/livox/set_lidar_ip.cpp`、`build_set_lidar_ip.sh`、`flash_lidar_ip.sh`、`README.md`
- `docs/船端 Docker 编译与运行指南.md`（本文）
- `docs/livox_mid_360_连接与驱动编译记录.md`（早前雷达连网/编译记录）
