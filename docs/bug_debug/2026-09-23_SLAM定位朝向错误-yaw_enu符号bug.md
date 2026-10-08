# 2026-09-23 SLAM 定位朝向镜像错误（gnss_unify_node yaw_enu 符号 bug）

## 一、基本信息

| 项 | 值 |
|---|---|
| 日期 | 2026-09-23 |
| 船端主机 | `tegra-ubuntu`（`ssh nx`，用户 `jetson`） |
| 定位栈 | lightning SLAM（`~/jzw_ws`）：`run_slam_online` + `run_rtk_pgo` + `slam_nav_bridge` |
| 导航栈 | `~/usv_nav_ws` Nav2（当时跑的是 `nav2_real_mavros.launch.py`，本次已改名 `nav2_boat.launch.py`） |
| 会话时间段 | 14:34 SLAM 栈/Nav2 启动 — 15:10 排查 |
| 现场状态 | 船靠泊位静止，RTK 33 星 HDOP 0.6，**浮点解 fix_type=5** |
| 排查方式 | 船端只读 rclpy 探针（TF 树、话题频率、参数 dump、栅格图投影复算），不发任何控制指令 |

## 二、现象

- RViz 里船的**朝向明显不对**（船头指向与实际不符），位置基本正确（船显示在泊位障碍物区，符合实际——船正停靠在坞边）。
- 用户现场确认："主要是朝向不对，位置在障碍物里面是对的"。

## 三、证据

### 3.1 TF 链完整、频率正常（排除链路断裂）

- `map→odom`（slam_nav_bridge）29 Hz、`odom→base_link` 39 Hz，`map→base_link` 可解算。
- 船位姿 map 系读数：`x=238.87, y=158.87, yaw=-120.7°`。

### 3.2 位置正确（排除地图/锚点为主因）

- 当前 RTK：lat=31.48757454, lon=120.37038862。
- 用导航侧 `map.yaml` 锚点 [120.36793468, 31.48618175] 投影 ENU → (232.95, 155.04)，与 TF 读数仅差 ~7 m（RTK 浮点解 + PGO 残差量级）。
- 栅格图复算：显示位置与真值都落在水边 1.6 m 内；而 SLAM 侧 `~/jzw_ws/app/map/map.yaml` 的锚点投影落在**陆地**上（见 5.2 附带发现）。
- 若地图本身旋转了 ~61°，离锚点 233 m 的船会偏出 200+ m、不可能正好在水边 → **地图是正北对齐的，朝向错不在地图**。

### 3.3 航向数值钉死（根因证据）

- GI320 INSPVAA 原始航向 `yaw=149.95°`（0=北，CW）。
- `/gnss/fix`（mapping_gnss/GnssFix，gnss_unify_node 输出）：`yaw=149.567°`，`yaw_enu=4.181236 rad`。
- **π/2 + 149.567°×π/180 = 4.181236 rad，小数点后 4 位完全吻合** → `yaw_enu` 是用 `π/2 + yaw` 算的；
  正确换算应为 `π/2 − yaw` = **−1.040 rad（−59.6°）**（约定见 `docs/归港现场调试手册.md`：`yaw_enu = (90 − yaw_gnss) mod 360`）。
- `map→odom` 的 yaw = −120.8° ≡ 239.6° = 错误的 yaw_enu 原值 → `run_rtk_pgo` 首帧把错误航向锚进了 map 系旋转。
- **偏差 = 2×罗盘航向**（镜像错误，当前航向下约 61°），随船朝向变化，无法用固定 `heading_bias_deg` 补偿。

## 四、根因（已确证）

`mapping_gnss` 包 `gnss_unify_node`：GI320 罗盘航向（0=北，CW，度）→ ENU（0=东，CCW，弧度）换算符号写反，
`yaw_enu = π/2 + yaw`，应为 `π/2 − yaw`。源码不在船上也不在岸端仓库（jzw_ws 只有二进制），由算法组修复发版。

**修复状态：算法组已改好并上传船端，现场确认朝向正确（2026-09-23 当天闭环）。**

## 五、附带发现（均已向相关方反馈，未处置）

1. **SLAM 侧地图配置过期**：`~/jzw_ws/app/map/map.yaml` 的 `ref_gnss_*` 四角间距（296×172 m）与栅格尺寸
   （359×291 m）自相矛盾，且与导航侧锚点差 **71 m**。接口文档要求 SLAM/导航共享同一份 map.yaml。
   当前 PGO 链路实测位置没受它影响，但属于隐患，建议以导航侧 yaml 为准替换。
2. **驱动重复启动**：`/livox/lidar` 有两个 livox 驱动在发同一雷达的点云（`usv_loc_stack.launch.py`
   默认 `start_driver:=true`，船上已有驱动时没设 false）。bw_gi320 也起了一个多余进程（串口占用失败在重试）。
3. **MAVROS 无 GNSS 数据**：`/mavros/global_position/global`、`raw/fix` 均无消息（APM 侧 GPS 链路断），
   与本次问题无关，待查。

## 六、已做修改（导航侧配套整改，岸端仓库）

朝向 bug 由算法组修；导航侧借这次把"接 SLAM 里程计"做成正式能力：

- **定位源 launch 化切换**：`nav2_boat.launch.py` / `mission_bridge.launch.py` / `docking.launch.py`
  新增统一参数 `odom_topic`（空 = 尊重 yaml，非空 = Nav2 三处 + mission 栈 + 归港栈一并覆盖）。
  用法：`ros2 launch workspace_nav nav2_boat.launch.py odom_topic:=/slam/localization`。
- **文件重命名**：`nav2_real_mavros.launch.py` → `nav2_boat.launch.py`；
  `nav2_params_real_mavros.yaml` → `nav2_params_chengfeng.yaml`；
  `nav2_params_real_xingfei.yaml` → `nav2_params_xingfei.yaml`；
  `mission_stack.real_boat.yaml` → `mission_stack.chengfeng.yaml`。
- `nav_status_aggregator` 的 odom 频率下限参数化：`min_odom_hz`（默认 8.0）。
- 详见 `src/USV_NAV/README.md` §B 与 `src/USV_NAV/docs/SLAM_INTEGRATION.md` §4。
- 注意：`.gitignore` 的 `config/` 规则会忽略新命名的 yaml，已 `git add -f` 强制纳入
  （这也是兴飞 yaml 此前一直未被跟踪的原因）。

## 七、验证

- 岸端冒烟：SLAM/默认两种模式的 Nav2 三处 + mission 三节点 + 归港 `odom_speed_topic` 参数实测正确；
  `min_odom_hz=8.0` 生效；`usv_docking` 78 测试通过（1 个失败为既有标定值/代码默认不一致，与本次无关）。
- 船端：算法组修复后实船朝向显示正确。
