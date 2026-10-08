# 雷达（Livox MID-360 / MID-360S）脚本

改雷达 IP 用的小工具与一键脚本。船上对应的副本在 `~/ws_livox/`（`tools/set_lidar_ip*.cpp`
随 `livox_ros_driver2` 源码一起同步，一键脚本是 `~/ws_livox/flash_lidar_ip.sh`）。

## 文件

| 文件 | 用途 |
|------|------|
| `set_lidar_ip.cpp` | 改雷达 IP 的独立程序（不依赖 ROS，只用 Livox SDK2） |
| `build_set_lidar_ip.sh` | 编译；支持 `SDK_PREFIX` / `OUT` 环境变量，本机 x86 与船端 ARM64 通用 |
| `flash_lidar_ip.sh` | 船端一键：临时加地址 → 改 IP → 清地址 → 等雷达重启 |

## 用法

本机编译（SDK 在 `/usr/local`）：

```bash
bash scripts/livox/build_set_lidar_ip.sh          # 产出 scripts/livox/set_lidar_ip
```

船端编译 ARM64（在 docker 里，源码和 SDK 都在 `~/ws_livox`）：

```bash
docker run --rm -v $HOME/ws_livox:/ws -w /ws/src/livox_ros_driver2/tools \
  usv-runtime:jp6-cu126-humble bash -lc \
  'SDK_PREFIX=/ws/sdk_install OUT=set_lidar_ip_arm bash build_set_lidar_ip.sh'
```

船端改雷达 IP：

```bash
bash ~/ws_livox/flash_lidar_ip.sh eth1 192.168.2.191 192.168.1.10
```

## 两个必须知道的点

1. **雷达不会自己重启。** 只发 set-ip 时雷达会回 ack `ret_code=0`，但 IP 不变；
   官方样例 `samples/livox_lidar_quick_start/main.cpp` 是在 set-ip 回调里再调一次
   `LivoxLidarRequestReboot`。本目录的源码已包含这一步。
2. **先让 NX 能到雷达所在的网段。** 雷达 IP 存在雷达里，比如现场雷达在
   `192.168.1.162` 且期望 host 是 `192.168.1.10`，就在雷达所在网口上临时加
   `192.168.1.10/24` 再刷，刷完删掉；NX 自己的 `192.168.2.101` 不要动。

改完 IP 后跑驱动的注意事项（用 MID-360S 的 launch）见
`docs/船端 Docker 编译与运行指南.md`。
