# SLAM 定位对接规范（内部）

> 供导航组员内部使用。发给 SLAM 组员的精简版见 [`SLAM_对接接口（发给SLAM组员）.md`](SLAM_对接接口（发给SLAM组员）.md)。

---

## 1. 导航侧需要什么

导航栈消费的定位数据 = **1 个 Odometry 话题 + 1 条 TF 链 `map→odom→base_link`**。

### 1.1 话题：`/slam/localization`（`nav_msgs/Odometry`）

| 项目 | 要求 |
|------|------|
| 频率 | ≥ 20 Hz（低于此频率触发 DEGRADED 告警） |
| QoS | **RELIABLE**, depth ≥ 10（见下方说明） |
| `header.frame_id` | `"odom"` |
| `child_frame_id` | `"base_link"` |

**必须字段**（缺字段会导致定位判 LOST 或功能异常）：

| 字段 | 用途 |
|------|------|
| `pose.pose.position.x` | 当前位置 x，odom 系 (m) |
| `pose.pose.position.y` | 当前位置 y，odom 系 (m) |
| `pose.pose.orientation` | 航向四元数 → yaw |
| `twist.twist.linear.x` | 前向线速度 (m/s) |
| `twist.twist.angular.z` | 转向角速度 (rad/s) |
| `pose.covariance[0]` | x 方差 (m²)，> 10 → LOST，> 1 → DEGRADED |
| `pose.covariance[7]` | y 方差 (m²)，同上 |
| `pose.covariance[14]` | z 方差 (m²)，水面填小值即可 |
| `pose.covariance[21]` | roll 方差 (rad²) |
| `pose.covariance[28]` | pitch 方差 (rad²) |
| `pose.covariance[35]` | yaw 方差 (rad²) |
| `header.stamp` | 时间戳，超时 2s → LOST |

**可填 0 的字段**：`pose.pose.position.z`、`twist.twist.linear.y`、`twist.twist.linear.z`

> **QoS 说明（2026-09-15 更正）**：导航侧订阅端是 RELIABLE（Nav2 `bt_navigator`/`controller_server`/
> `velocity_smoother` 用 `rclcpp::SystemDefaultsQoS()`，`mission_bridge`/`nav_status_aggregator` 用 rclpy 默认），
> 所以 `/slam/localization` 必须用 **RELIABLE** 发布。RELIABLE 发布端兼容 BEST_EFFORT 订阅端，反之不兼容。
> 现网 `slam_nav_bridge` 输出即 RELIABLE（`rclcpp::QoS(10)`），与订阅端一致。

### 1.2 TF 广播

| 变换 | 父帧 | 子帧 | 说明 |
|------|------|------|------|
| `map → odom` | `map` | `odom` | SLAM 全局漂移修正；≥ 20 Hz |
| `odom → base_link` | `odom` | `base_link` | 局部里程计；Odometry 消息自带或单独广播 |

**时间戳约定**：两条 TF 都使用**定位源 odom 消息的时间戳**（与 `/slam/localization` 的 `header.stamp` 同源），
保证 `map→odom` 与 `odom→base_link` 处于同一时间基；无有效时间戳时回退 `now()`。
导航侧已**废弃**"准静态 `map→odom` 打 `now()+0.2s` 未来时间戳"的旧做法（`usv_map_odom_tf` 的
`tf_future_offset_sec` 已移除），SLAM 侧不需要任何预偏移。

**`/tf`、`/tf_static` 的 QoS 不用改**：tf2_ros 默认 RELIABLE（depth 100），两端天然一致；只有
`/slam/localization` 这个普通话题需要注意 QoS。

**为什么分两层？**
- `odom → base_link`：连续平滑的局部里程计（帧间准确，但会累积漂移）
- `map → odom`：SLAM 通过回环/特征匹配消除漂移后的全局修正（可能离散跳变）
- Nav2 local_costmap 用 `odom` 帧，global_costmap 用 `map` 帧，两层结构允许各自独立运作

**如果 SLAM 只输出 `map → base_link`**，需要改 Nav2 local_costmap 的 `global_frame` 为 `map`，不推荐。

---

## 2. 坐标系对齐方案

**核心问题**：SLAM 启动时自身原点 ≠ 导航 OccupancyGrid（pgm+YAML）的原点。船的 SLAM 坐标和船在导航地图上的位置必须对应。

### 方案 A：初始位姿对齐（推荐）

```
导航侧 → 发布 /initialpose (PoseWithCovarianceStamped, frame_id: map)
SLAM侧 → 收到后重置参考系，使当前位置 = 指定 map 坐标
```

**SLAM 不需要导航栅格地图**，只需知道启动瞬间自己在 map 系下的 (x, y, yaw)。

**初始位姿来源**：
- GNSS：当前 GNSS → ENU(map 锚点) → (x, y)；yaw 可从 GNSS 航向或手动指定
- `/initialpose` topic：由 `gnss_odom_map_tf` 节点在收到第一帧 GNSS 后计算并发布一次
- 手动参数：启动时通过 ros2 param 指定

### 方案 B：SLAM 订阅导航栅格地图（基于地图的定位）

```
导航侧 → /map (OccupancyGrid, 由 map_server 发布, QoS: RELIABLE + TRANSIENT_LOCAL)
SLAM侧 → 在栅格地图中做扫描匹配/重定位，输出 map 系位姿
```

**适用**：SLAM 有 LiDAR 且需要在导航地图中定位。

### 方案 C：统一 GNSS 锚点

SLAM 和导航共用 `map.yaml` 中的 GNSS 锚点（`ref_gnss_*`）：

```
map 系原点 = (锚点经度, 锚点纬度)
map 系坐标 = ENU(当前GNSS − 锚点GNSS)
```

SLAM 融合 GNSS 初始化即可输出 map 系位姿。

### 2.4 GNSS 数据来源（船端实际话题）

船端**没有 `/rtk/fix` 发布者**，GNSS 原始数据只有 GI320 驱动这两路：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/gi320/gngga` | `bw_gi320_driver/Gi320Gngga` | GGA：`fix_type` / HDOP / 卫星数 |
| `/gi320/inspvaa` | `bw_gi320_driver/Gi320Inspvaa` | INSPVAA：10 Hz 位置、姿态、航向、速度 |

SLAM 侧统一经 `mapping_gnss/gnss_unify_node` 取数，输出 `mapping_gnss/GnssFix` 到 `/gnss/fix`：

```bash
ros2 launch mapping_gnss gnss_unify.launch.py \
  gngga_topic:=/gi320/gngga inspvaa_topic:=/gi320/inspvaa output_topic:=/gnss/fix
```

lightning `system.rtk_topic` 与 mapping_bridge 各节点的 `gps_topic` / `heading_topic` 都必须指向
`/gnss/fix`；写 `/rtk/fix` 在船上收不到数据。已安装的 `run_lightning_odom.launch.py` 已 include
`gnss_unify` 并使用 `/gnss/fix`；`~/usv/src/mapping/lightning-lm` 源码副本是旧版（`/rtk/fix`、
未 include `gnss_unify`），从该副本重新编译前需先同步。

---

## 3. 导航 → SLAM 的输出

| 接口 | 类型 | 必须 | 用途 |
|------|------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | # | 方案 B：SLAM 在导航地图中定位 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | # | 方案 A：坐标系对齐 |
| `/tf_static` | `tf2_msgs/TFMessage` | # | 传感器静态变换（base_link→lidar/camera） |
| map 锚点参数 | ros2 param | # | 方案 C：ENU 投影参数 |

---

## 4. 当前 MAVROS 方案 → SLAM 方案的配置切换

从 MAVROS（ArduPilot + GI320）切换到 SLAM 定位，用 launch 参数 `odom_topic` 一处切换（2026-09-23 起）：

```bash
# 导航 + mission 栈（Nav2 的 bt_navigator / controller_server / velocity_smoother
# 与 mission_bridge / nav_status_aggregator / zone_monitor 同时被改写）
ros2 launch workspace_nav nav2_boat.launch.py odom_topic:=/slam/localization ...

# 归港栈（覆盖 docking_fsm.odom_speed_topic）
ros2 launch usv_docking docking.launch.py odom_topic:=/slam/localization
```

### 4.1 odom_topic 参数行为

| launch 参数 | 空（默认） | 非空（如 `/slam/localization`） |
|------|----------|--------|
| `nav2_boat.launch.py` 的 `odom_topic` | 尊重 `params_file`（`nav2_params_chengfeng.yaml` 默认 `/mavros/gps_input/local`） | RewrittenYaml 改写 Nav2 全部三处 `odom_topic`，并透传 mission 栈 |
| `mission_bridge.launch.py` 的 `odom_topic` | 尊重 `mission_stack.chengfeng.yaml` / 代码默认 | 覆盖 mission_bridge / nav_status_aggregator / zone_monitor |
| `docking.launch.py` 的 `odom_topic` | 尊重 `docking.yaml` | 覆盖 `docking_fsm.odom_speed_topic` |

**说明**

- MAVROS 模式的 odom 不是 `/mavros/local_position/odom`：实船（ArduPilot + GI320）用的是 mavros
  `gps_input` 插件的 `/mavros/gps_input/local`，`/mavros/local_position/odom` 是另一路（未参与导航）。
- 要固定换源，也可以直接改 yaml：`nav2_params_chengfeng.yaml` 三处（bt_navigator /
  controller_server / velocity_smoother）、`mission_stack.chengfeng.yaml`、`docking.yaml`
  里的 `odom_topic`（文件内注释标了位置）；legacy `waypoint_with_state` 不在实船链路，不用管。
- `nav_status_aggregator` 的 odom 频率判据是参数 `min_odom_hz`（默认 8.0），SLAM 掉频时可在
  `mission_stack.chengfeng.yaml` 调低。

### 4.2 不再需要的节点

切换到 SLAM 后，以下 MAVROS 相关定位节点可停用：

| 节点 | 原因 |
|------|------|
| `usv_map_odom_tf`（旧称 `gnss_odom_map_tf`） | `map→odom` TF 改由 SLAM 发布，两个发布者会抢同一条 TF |
| MAVROS `local_position` TF 发布 | `odom→base_link` TF 改由 SLAM 发布 |
| `nav_status_aggregator` 的 GPS 订阅 | 如 SLAM 不提供 NavSatFix，gps_fix 字段需调整或禁用 |

---

## 5. 验证清单

| # | 检查项 | 命令 / 方法 |
|---|--------|-------------|
| 1 | SLAM 话题正在发布 | `ros2 topic hz /slam/localization` |
| 2 | 字段完整（pose, twist, covariance 非零） | `ros2 topic echo /slam/localization --once` |
| 3 | TF `map→odom→base_link` 完整 | `ros2 run tf2_tools view_frames` |
| 4 | Odometry 频率 ≥ 20 Hz | `ros2 topic hz /slam/localization` |
| 5 | covariance 在阈值内（< 1.0 为 GOOD） | 检查 /nav_status 中 `localization.overall != LOST` |
| 6 | Nav2 能规划路径 | `ros2 service call /compute_path_to_pose ...` 或通过 mission_bridge 发单航点 |
| 7 | `/nav_status` 中 pose.x/y 与实际船位一致 | 在 map 系中对比 GNSS 换算值和 SLAM 输出 |

---

## 6. 协方差填写建议

如果 SLAM 不输出动态协方差，填静态值即可：

```
pose.covariance = [
  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,   # x方差=0.01
  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,   # y方差=0.01
  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,   # z方差=0.01
  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,   # roll方差=0.01
  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,   # pitch方差=0.01
  0.0,  0.0,  0.0,  0.0,  0.0,  0.01   # yaw方差=0.01
]
```

**关键**：全部静态值需 < 1.0，否则 nav_status_aggregator 判 DEGRADED。

---

## 7. Odometry 消息示例

```python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Point, Quaternion, Vector3
from builtin_interfaces.msg import Time

msg = Odometry()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "odom"
msg.child_frame_id = "base_link"

# pose
msg.pose.pose.position = Point(x=12.5, y=-3.2, z=0.0)
msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)  # yaw=90°
msg.pose.covariance[0] = 0.01   # x方差
msg.pose.covariance[7] = 0.01   # y方差
msg.pose.covariance[14] = 0.01  # z方差
msg.pose.covariance[21] = 0.01  # roll方差
msg.pose.covariance[28] = 0.01  # pitch方差
msg.pose.covariance[35] = 0.01  # yaw方差

# twist
msg.twist.twist.linear = Vector3(x=0.5, y=0.0, z=0.0)   # 前进 0.5 m/s
msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.05)  # 转向 0.05 rad/s
```
