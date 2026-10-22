# 船坞 AprilTag dock_offset 现场自动标定

> 适用：兴飞船（MEC-MB002）更换后视相机、调整相机支架、或船坞二维码面板/坞体位置变化后，
> 重新标定 `detection_cfg.yml` 里 `tag_0` / `tag_1` 的 `dock_offset_*`。
>
> 本目录是**现场交付工具**：一条命令完成采样、反解、质量检查、备份和写配置。

## 1. 它做什么

`dock_offset_calib.py` 在船**正确停稳在坞内**时：

1. 读取 TF `base_link -> apriltag_<id>`（这是**不含** `dock_offset` 的原始 Tag 位姿）；
2. 反解每个 tag 的 `dock_offset_*`，使 `base_link -> dock_frame` 落到目标位姿（默认 `(0,0,0)`）；
3. 用保留的 30% 样本做验证，检查船是否停稳、结果是否可信；
4. 把结果写回 `detection_cfg.yml`，自动备份原文件；
5. `--verify` 模式在重启感知后复核残差。

**它不会向船发任何运动指令**；只读 TF，按需开关 AprilTag 检测通道/船坞背光，写 YAML。

### 与 apriltag_node 的约定

节点源码（`apriltag_node.cpp`）里：

```text
tag_tf  = camera2camera_link * apriltagPoseToTf2(pose) * camera_tag2ros_
dock_tf = tag_tf * dock_offset
```

所以 TF `apriltag_<id>` 就是 `dock_offset` 之前的 Tag 位姿。期望
`T_base_dock = T_target` 时：

```text
dock_offset = inv(T_base_tag) * T_target
```

`dock_offset_roll/pitch/yaw` 的约定是 `tf2::Quaternion::setRPY(roll,pitch,yaw)`，
即固定轴 XYZ；YAML 里单位是**度**。

### 和 2026-09-30 记录的关系

上次实船标定的完整过程、原始数据和已知值见：

- 本目录副本：[`docs/REFERENCE_2026-09-30.md`](docs/REFERENCE_2026-09-30.md)
- 仓库原件：`docs/bug_debug/2026-09-30_兴飞后视AprilTag与船坞中心不对称排查.md`

上次的值（当前船端配置里仍是这套，可作为基线）：

| 参数 | tag_0 | tag_1 |
|---|---|---|
| dock_offset_x | -0.666 | -0.688 |
| dock_offset_y | +0.724 | -0.540 |
| dock_offset_z | -0.649 | -0.649 |
| roll/pitch/yaw (°) | -0.19 / -0.25 / -0.26 | +0.21 / +0.34 / +0.15 |

> 上次标定把“当时停靠姿态”的偏差吸收进了 offset。换相机后如果支架/内参变了，
> 必须重新跑本工具，不要直接沿用旧值。

## 2. 目录内容

```text
scripts/dock_calib/
├── README.md                        # 本文
├── dock_offset_calib.py            # 主工具（现场用）
├── run_dock_offset_calib.sh        # 船端入口：source ROS + 兴飞 install 后运行
├── test_dock_offset_calib.py       # 纯数学/YAML 自测（本机无需 ROS）
├── docs/
│   └── REFERENCE_2026-09-30.md     # 上次标定记录副本
└── reference/
    └── compute_dock_offset_sim.py  # 旧仿真脚本（仅 Gazebo 用，不用于实船）
```

`reference/` 里的旧脚本是仿真 `detection_cfg_sim.yml` 的辅助脚本，
**不要拿它标实船**；实船用 `dock_offset_calib.py`。

## 3. 前置条件

- 船端 ROS 2 Humble，兴飞栈已启动（至少要有 `mavros`、TF、`usv_app` 里的
  `perception_launch` / `apriltag_localization`、`usv_transforms`）。
- 后视相机正常出图，`/apriltag_node` 能检测到 **tag_0 和 tag_1**。
- 船已**正确停稳在坞内**，`/dock/status` 为 `IDLE`（不能正在归港/出港）。
- 能读到 `base_link -> apriltag_0/1` 的 TF。
- 标定前先确认相机内参/外参已经正确（见 §7）。

## 4. 部署到船上

在主机仓库根目录：

```bash
scp -r scripts/dock_calib hkzl@192.168.2.101:~/usv-test/scripts/
ssh hkzl 'chmod +x ~/usv-test/scripts/dock_calib/run_dock_offset_calib.sh'
```

如果船上没有 `~/usv-test/scripts`，放到 `~/dock_calib` 也可以，脚本不依赖固定路径。

## 5. 标准现场流程

### 5.1 先看当前配置 / 自测

```bash
cd ~/usv-test/scripts/dock_calib

# 不用 ROS，确认脚本本身正常
python3 dock_offset_calib.py --self-test

# 打印当前 detection_cfg.yml 的 offset
python3 dock_offset_calib.py --print

# 或船端环境入口
./run_dock_offset_calib.sh --print
```

### 5.2 确认 TF 链路

```bash
./run_dock_offset_calib.sh --print   # 能读配置即可
# 另开终端，或直接：
ros2 run tf2_ros tf2_echo base_link camera_back_video_link
ros2 run tf2_ros tf2_echo base_link apriltag_0
ros2 run tf2_ros tf2_echo base_link apriltag_1
```

三个 TF 都能持续输出，才继续。

### 5.3 dry-run（推荐先跑，不写配置）

```bash
./run_dock_offset_calib.sh --dry-run
```

输出会包含：

```text
采样与拟合质量：
  tag  n    t_std(mm)  r_std(deg)  val_t(mm)  val_r(deg)
  0    60       2.1         0.08        3.0        0.10
  1    60       2.3         0.09        3.5        0.12

新 dock_offset（已按 node 的 setRPY 约定换算）：
  tag_0: x=... y=... z=...
  ...
```

- `t_std/r_std`：船是否停稳；默认超过 **20 mm / 1°** 拒绝写入；
- `val_t/val_r`：留出样本的验证残差；默认超过 **50 mm / 2°** 拒绝写入；
- 不合格时会明确打印原因，并且**不会写配置**。

### 5.4 写配置

确认 dry-run 结果后：

```bash
./run_dock_offset_calib.sh --write
```

脚本会：

1. 生成 `detection_cfg.yml.bak_YYYYmmdd_HHMMSS` 备份；
2. 只改 `dock_offset_*` 数值，保留原注释和格式；
3. 打印 12 条 old → new 变更。

### 5.5 重启感知/整栈

`usv_docking`、`apriltag_node` 只在启动时加载 offset。写完后必须重启：

```bash
# 兴飞新栈：整栈重启最稳妥（会短暂影响其他包）
~/usv-test/stop_cleaning_boat_xinfei.sh
~/usv-test/start_cleaning_boat_xinfei.sh
```

> 感知是 `usv_app` 的 `perception_launch` 的一部分；如果现场能单独重启
> `perception_launch`，也可以只重启它。注意兴飞启动脚本的 `--supervise`
> 在任一 launch 退出时会收掉整栈，所以单独 kill 感知节点通常等于整栈重启。

### 5.6 验证

```bash
./run_dock_offset_calib.sh --verify
# 或
ros2 run tf2_ros tf2_echo base_link dock_frame
```

期望：

- 每个 tag 的配置预测 `base_link -> dock_frame` 平移误差 < 50 mm、旋转 < 2°；
- `dock_frame` 的 TF 平移接近 `(0,0,0)`；
- 两 tag 的 dock 位姿重合（节点内部融合）。

### 5.7 回写源码，避免下次部署被旧值覆盖

船端运行的是 install 里的配置：

```text
~/usv-test/install/arm/apriltag_localization/config/detection_cfg.yml   ← 运行时实际加载
```

同时要更新源码/主机仓库，否则重新部署会覆盖新标定值：

```text
船端源码：~/usv-main/src/perception/apriltag_localization/config/detection_cfg.yml
主机源码：/home/ght/usv/src/perception/apriltag_localization/config/detection_cfg.yml
```

主机改完提交/推送流程按项目约定（当前由你手动 push）。

## 6. 常用参数

```bash
./run_dock_offset_calib.sh --help
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--config PATH` | `~/usv-test/install/arm/apriltag_localization/config/detection_cfg.yml` | 要读写的配置 |
| `--samples N` | 60 | 每个 tag 的目标样本数 |
| `--min-samples N` | 20 | 最少有效样本数 |
| `--rate HZ` | 10 | 采样频率 |
| `--timeout SEC` | 40 | 采样超时 |
| `--max-age SEC` | 0.5 | TF 最大龄期 |
| `--base-frame` | `base_link` | 参考系 |
| `--tag-frame-fmt` | `apriltag_{id}` | Tag TF 名 |
| `--tag-frame-prefix` | 空 | 多通道模式前缀，如 `back_uvc/` |
| `--dock-frame` | `dock_frame` | 融合坞系 TF 名 |
| `--target-x/y` | 0 / 0 | 目标坞系在 base_link 的 xy（m） |
| `--target-yaw-deg` | 0 | 目标坞系相对 base_link 的 yaw（度） |
| `--no-enable-channel` | 关 | 不自动开检测通道（手动已开时用） |
| `--channel` | `all` | 通道名 |
| `--backlight` | 关 | 采样时点亮背光 DO bit1（夜间/暗处用） |
| `--backlight-mask` | 2 | 背光 DO 掩码 |
| `--keep-channel/--keep-backlight` | 关 | 结束后保留开着的通道/背光 |
| `--max-std-m/--max-std-deg` | 0.02 / 1.0 | 停稳门限 |
| `--max-residual-m/--max-residual-deg` | 0.05 / 2.0 | 验证残差门限 |
| `--dry-run` | 默认 | 不写配置 |
| `--write` | 关 | 写配置 |
| `--force` | 关 | 跳过 dock 状态/质量门限（慎用） |
| `--verify` | 关 | 验证当前配置 |
| `--self-test` | 关 | 纯数学/YAML 自测，无需 ROS |
| `--verbose` | 关 | 输出等待细节 |

## 7. 换后视相机后的完整顺序

> 相机换了以后**不要直接标坞中心**，先把内参/外参弄对，否则 dock_offset 会把
> 相机误差一起吸收进去。

### 7.1 USB / 设备

```bash
lsusb | grep -i 0bda          # 当前期望 0bda:3035
ls -l /dev/video*
sudo dmesg -T | tail -30      # 应出现 Found UVC ...
```

- 如果新相机不是 `0bda:3035`，改
  `video_camera/config/params.yaml` 的 `usb_id`，必要时改分辨率/`pixel_format`。
- 当前船上前一台相机在 USB hub `1-2` 后面；换相机时顺便检查 hub 供电/线。

### 7.2 相机内参

工具在：

```text
主机：/home/ght/usv/tools/calibration
船端：~/usv-main/tools/calibration
```

船上当前**没有编译安装** `camera_calibration`。可以：

- 主机交叉编译：`make CROSS_REGISTRY= cross-jetson-pkgd-r PKGS="camera_calibration"`，
  然后只同步 `install/arm/camera_calibration`；或
- 用旧的/现成的标定程序，输出 ROS 兼容 `camera_info.yaml` 即可。

> ⚠️ **不要在船上直接 `make pkgd-r`**：当前 `build/arm/m_common/CMakeCache.txt`
> 仍指向 Docker 的 `/workspace/usv`，Makefile 的 `GUARD_CACHE` 会执行
> `rm -rf build/arm install/arm log/arm`，把所有人的包一起清掉。
> 单包请用主机交叉编译，或直接 `colcon build` + 全新 build 目录（见项目部署记录）。

标定参数与旧相机一致时用 1920x1080、MJPG。标定得到的新内参放到：

```text
~/usv-test/install/arm/video_camera/share/video_camera/config/camera_info/<新SN>.yaml
```

内参选择规则（`video_camera/config/params.yaml`）：

```yaml
camera_info_file: ""   # 空=按 app/metadata.json 的 sn 自动选 camera_info/<sn>.yaml
```

船端 `~/usv-test/app/metadata.json` 当前 `sn=HC2703163`，且
`camera_info/HC2703163.yaml` 是旧后视相机内参的副本（与 `26063001.yaml` 内容相同）。
换相机后二选一：

- 更新 `app/metadata.json` 的 `sn` 为新相机 SN；或
- 在 `params.yaml` 里显式写 `camera_info_file: "<绝对路径>/<新SN>.yaml"`。

旧值参考（`26063001.yaml`）：

```text
fx=920.051, fy=919.863, cx=994.569, cy=551.809
distortion=[0.0534694, -0.0507922, -0.00012208, -0.000619555, -0.0496482]
```

### 7.3 相机外参（安装位置）

```text
~/usv-test/src/usv_transforms/urdf/cleaning_boat_xingfei.xacro
  camera_back_video_xyz = 1.58 0.0 0.46
  camera_back_video_rpy = 0 -17° 180°
```

- 同位置、同支架更换：不用改；
- 换支架/角度：改这里并重新构建/部署 `usv_transforms`，再确认：
  ```bash
  ros2 run tf2_ros tf2_echo base_link camera_back_video_link
  ```

### 7.4 坞中心 dock_offset

内参/外参确认后，按 §5 运行本工具。若坞体/二维码面板也动过，先卷尺复核：

- 面板间距；
- 面板中心到坞中心距离/高度差；
- 二维码黑框外边尺寸（当前配置 `tag_size: 0.25`，family `tag36h11`，IDs `[1,0]`）。

## 8. 退出码与故障排查

| 退出码 | 含义 |
|---|---|
| 0 | 成功（dry-run/写入/验证通过） |
| 1 | 配置/采样/参数错误 |
| 2 | 质量门限未通过（或验证未通过），未写配置 |
| 130 | Ctrl+C |

常见问题：

| 现象 | 处理 |
|---|---|
| `tag_0/1 有效样本不足` | 检查相机出图、检测通道是否开启、Tag 是否可见、`tag_frame_fmt/prefix` 是否匹配 |
| `dock_mission 状态不是 IDLE` | 等任务结束；确认安全后 `--force` |
| TF 龄期太老 | 检查 `apriltag_node` 是否持续检测、相机是否掉帧、`use_sim_time` |
| 平移/旋转抖动超限 | 船没停稳；等稳或增加 `--samples`，不要 `--force` |
| 验证残差大 | 先重启感知；再检查内参/外参/面板安装；确认写的是 install 里的配置 |
| `dock_frame` 还是旧值 | 节点没重启；`--verify` 里“配置预测”和“运行节点”会分别显示 |
| 夜间检测不到 Tag | 加 `--backlight`（DO bit1），或检查船坞背光/曝光 |

## 9. 备份与回滚

每次 `--write` 都会生成：

```text
detection_cfg.yml.bak_YYYYmmdd_HHMMSS
```

回滚：

```bash
cp ~/usv-test/install/arm/apriltag_localization/config/detection_cfg.yml.bak_XXXX \
   ~/usv-test/install/arm/apriltag_localization/config/detection_cfg.yml
# 然后重启感知/整栈
```

## 10. 参考

- 2026-09-30 现场记录：`docs/REFERENCE_2026-09-30.md`（本目录副本）
- AprilTag 节点约定：`src/perception/apriltag_localization/README.md`、
  `src/perception/apriltag_localization/src/apriltag_node.cpp`
- 相机内参工具：`/home/ght/usv/tools/calibration/README.md`
- 归港调试手册：`docs/归港现场调试手册.md`
