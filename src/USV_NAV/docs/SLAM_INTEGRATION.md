# SLAM 定位对接规范（内部）

> 供导航组员内部使用。发给 SLAM 组员的精简版见 [`SLAM_对接接口（发给SLAM组员）.md`](SLAM_对接接口（发给SLAM组员）.md)。

---

## 1. 导航侧需要什么

导航栈消费的定位数据 = **1 个 Odometry 话题 + 1 条 TF 链 `map→odom→base_link`**。

### 1.1 话题：`/slam/localization`（`nav_msgs/Odometry`）

| 项目 | 要求 |
|------|------|
| 频率 | ≥ 20 Hz（低于此频率触发 DEGRADED 告警） |
| QoS | BEST_EFFORT, depth ≥ 10 |
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

### 1.2 TF 广播

| 变换 | 父帧 | 子帧 | 说明 |
|------|------|------|------|
| `map → odom` | `map` | `odom` | SLAM 全局漂移修正；≥ 20 Hz |
| `odom → base_link` | `odom` | `base_link` | 局部里程计；Odometry 消息自带或单独广播 |

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

从 MAVROS（PX4 EKF）切换到 SLAM 定位，需修改以下配置中的 odom_topic。

### 4.1 需修改的文件

| 文件 | 参数路径 | 当前值 | 改为 |
|------|----------|--------|------|
| `nav2_params_real_mavros.yaml` | `bt_navigator.odom_topic` | `/mavros/local_position/odom` | `/slam/localization` |
| `nav2_params_real_mavros.yaml` | `velocity_smoother.odom_topic` | `/mavros/local_position/odom` | `/slam/localization` |
| `nav2_mavros_odom_overlay.yaml` | `bt_navigator.odom_topic` | `/mavros/local_position/odom` | `/slam/localization` |
| `nav2_mavros_odom_overlay.yaml` | `velocity_smoother.odom_topic` | `/mavros/local_position/odom` | `/slam/localization` |
| `mission_stack.real_boat.yaml` | `mission_bridge.odom_topic` | `/mavros/local_position/odom` | `/slam/localization` |
| `mission_bridge.launch.py` | `odom_topic` 参数默认值 | `/mavros/local_position/odom` | `/slam/localization` |

### 4.2 不再需要的节点

切换到 SLAM 后，以下 MAVROS 相关定位节点可停用：

| 节点 | 原因 |
|------|------|
| `gnss_odom_map_tf` | `map→odom` TF 改由 SLAM 发布 |
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
