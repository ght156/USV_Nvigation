# SLAM 与导航对接接口

> **发给 SLAM 组员**。本文定义 SLAM 模块与导航栈之间的全部接口。

---

## 一、SRAM 需要发布的内容

### 1.1 话题：定位位姿

| 项目 | 要求 |
|------|------|
| **话题名** | `/slam/localization` |
| **消息类型** | `nav_msgs/Odometry` |
| **频率** | ≥ 20 Hz |
| **QoS** | **RELIABLE**, depth ≥ 10 |
| **header.frame_id** | `"odom"` |
| **child_frame_id** | `"base_link"` |

**必须填写的字段：**

```
pose.pose.position.x          # 当前位置 x (m)
pose.pose.position.y          # 当前位置 y (m)
pose.pose.position.z          # 填 0 即可
pose.pose.orientation         # 航向四元数 (x,y,z,w)

twist.twist.linear.x          # 前向速度 (m/s)
twist.twist.linear.y          # 填 0 即可
twist.twist.linear.z          # 填 0 即可
twist.twist.angular.z         # 转向角速度 (rad/s)

pose.covariance[0]            # x 方差 (m²)，需 < 1.0
pose.covariance[7]            # y 方差 (m²)，需 < 1.0
pose.covariance[14]           # z 方差 (m²)，填小值即可
pose.covariance[21]           # roll 方差 (rad²)，填小值即可
pose.covariance[28]           # pitch 方差 (rad²)，填小值即可
pose.covariance[35]           # yaw 方差 (rad²)，需 < 1.0

header.stamp                  # 时间戳（ROS 时间）
```

> **协方差说明**：如果 SLAM 不输出动态协方差，全部填 `0.01` 即可（必须 < 1.0，否则导航判定位异常）。

> **QoS 说明（2026-09-15 更正）**：导航侧订阅端（Nav2 `bt_navigator`/`controller_server`/`velocity_smoother`、
> `mission_bridge`、`nav_status_aggregator`）的 odom 订阅都是 **RELIABLE**，因此 `/slam/localization`
> 必须用 **RELIABLE**（depth ≥ 10）发布。RELIABLE 发布端可以兼容 BEST_EFFORT 订阅端，反之不兼容
> （BEST_EFFORT 发布的数据不会被 RELIABLE 订阅端接收）。本文早前写的 BEST_EFFORT 是错的，已作废。
> 参考：`slam_nav_bridge.cpp` 里输出即 `rclcpp::QoS(10)` + RELIABLE。

### 1.2 TF 变换

广播 TF 链：**`map → odom → base_link`**

| 变换 | 父帧 | 子帧 | 频率 | 说明 |
|------|------|------|------|------|
| `odom → base_link` | `odom` | `base_link` | ≥ 20 Hz | 局部里程计，帧间连续平滑 |
| `map → odom` | `map` | `odom` | ≥ 20 Hz | 全局漂移修正（回环/特征匹配消除漂移后的偏移量） |

- `odom → base_link` 就是 Odometry 消息表达的位姿（`frame_id: odom, child_frame_id: base_link`），可以直接从 Odometry 消息发布 TF 或单独广播。
- `map → odom` 是 SLAM 消除累积漂移后的修正量。如果没有回环检测，初始启动时发布 identity 变换即可。

**时间戳约定**：两条 TF 都使用**定位源 odom 消息的时间戳**（即 `/slam/localization` 的 `header.stamp`），
使 `map → odom` 与 `odom → base_link` 处于同一时间基；没有有效时间戳时回退 `now()`。
导航侧已**废弃**"给准静态 `map → odom` 打 `now()+0.2s` 未来时间戳"的旧做法（`tf_future_offset_sec`），
SLAM 侧不需要做任何预偏移。

**`/tf`、`/tf_static` 的 QoS 不用改**：由 `tf2_ros` 统一管理（RELIABLE，depth 100），发布端与订阅端天然一致；
需要额外注意 QoS 的只有 `/slam/localization` 本身。

**为什么是两层而不是直接 `map → base_link`？**

导航的 local_costmap（局部避障）工作在 `odom` 帧，global_costmap（全局规划）工作在 `map` 帧。两层解耦让局部规划不受全局修正跳变的影响。

---

## 二、导航侧提供给 SLAM 的数据

**导航侧与 SLAM 之间无需任何运行时通信**：SLAM 直接读取和导航同一份 `map.yaml` 自己提取锚点与元数据；
GNSS 由 SLAM 自行从船端 GNSS 话题获取（见 2.2）。

### 2.1 map.yaml 格式

```yaml
# 导航和 SLAM 共享同一份 map.yaml
image: hk_map.pgm
resolution: 1.0               # 米/像素
origin: [0.0, 0.0, 0.0]       # PGM 左下角在 map 系的位置

# 地图锚点 GNSS（现场实测）
ref_gnss_00: [120.36783085, 31.48922948]   # [经度, 纬度] — NW 角
ref_gnss_11: [120.37224289, 31.48551322]   # [经度, 纬度] — SE 角
```

- `ref_gnss_00` 对应 map 系 `(0, H)`，`ref_gnss_11` 对应 `(W, 0)`，W = 宽×分辨率，H = 高×分辨率
- **SLAM 自行解析此文件**，不需要导航侧传递任何参数

### 2.2 GNSS 输入（SLAM 侧订阅，船端实际话题）

船端 GNSS 原始数据来自 GI320 驱动，**没有 `/rtk/fix` 的发布者**（`/gnss/fix` 也需要由下述节点生成）：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/gi320/gngga` | `bw_gi320_driver/Gi320Gngga` | GGA：`fix_type` / HDOP / 卫星数 |
| `/gi320/inspvaa` | `bw_gi320_driver/Gi320Inspvaa` | INSPVAA：10 Hz 位置、姿态、航向、速度 |

SLAM 侧统一从 `mapping_gnss` 的 `gnss_unify_node` 取数，它把上面两路合成 `mapping_gnss/GnssFix` 后发到 `/gnss/fix`：

```bash
ros2 launch mapping_gnss gnss_unify.launch.py \
  gngga_topic:=/gi320/gngga inspvaa_topic:=/gi320/inspvaa output_topic:=/gnss/fix
```

因此 lightning / mapping_bridge 里所有 GNSS 话题参数（lightning `system.rtk_topic`、bridge 各节点的
`gps_topic` / `heading_topic`）都必须指向 **`/gnss/fix`**；写成 `/rtk/fix` 或别的名字在船上收不到数据
（船上对应话题不存在）。带旧 bag 的 NavSatFix 场景用 `navsat_topic:=` 传入，正常实船运行留空即可。

---

## 三、坐标系对齐方案（SLAM 自行初始化）

### 3.1 初始化流程

```
启动时（一次性）：

  1. SLAM 读取导航提供的 map 锚点参数 (ref_gnss_00, ref_gnss_11)
  2. SLAM 从自己的 GNSS 获取当前经纬度
  3. SLAM 自己做 ENU 投影，算出当前在 map 系下的 (x, y)
     map_x = east_offset(当前GNSS, 锚点GNSS)
     map_y = north_offset(当前GNSS, 锚点GNSS)
  4. yaw 从 GNSS 航向获取，或从船体静止朝向指定
  5. SLAM 以此位姿初始化内部坐标系

运行中（持续）：

  SLAM 输出 map→odom→base_link TF + /slam/localization
  导航直接消费，无需任何坐标转换
```

### 3.2 关键要求

- SLAM 的 `map` 系必须与导航 OccupancyGrid 的原点一致（PGM 左下角 = map 坐标 `origin: [x, y, z]`）
- **初始化（坐标对齐 / 首次锁定 `map→odom`）只在启动时做一次**，之后 SLAM 独立运行
- 但 GNSS 不是"用完就丢"：运行中 SLAM 仍需继续用它做低频漂移修正。当前 `rtk_lio_fuse` 会持续软融合
  RTK（`alpha_xy≈0.05`、创新门控 5 m、`z_mode=rtk`），GNSS 失锁超过 `max_gps_age_s`（1 s）才退回纯 LIO。
  因此"初始化一次"指的是坐标对齐只做一次，不等于"之后可以没有 GNSS"
- 如果启动时 GNSS 不可用，可回退到手动指定初始位姿

### 3.3 帧树

```
map ─────(SLAM发布)─────► odom ─────(SLAM发布)─────► base_link
 │                                                      │
 │  导航占用栅格地图坐标系                                 │  static transforms
 │  global_costmap 工作帧                                ├── livox_frame
 │  全局规划在此帧                                        ├── imu_link
 │                                                      ├── gps_link
 │                                                      └── camera_link
```

---

## 四、接口汇总

### SLAM → 导航

| 接口 | 类型 | 频率 | 用途 |
|------|------|------|------|
| `/slam/localization` | `nav_msgs/Odometry` | ≥20 Hz | 位姿、速度、协方差 |
| TF `map → odom` | `TransformStamped` | ≥20 Hz | 全局漂移修正 |
| TF `odom → base_link` | `TransformStamped` | ≥50 Hz | 局部位姿 |

### 导航 → SLAM

| 接口 | 形式 | 用途 |
|------|------|------|
| `map.yaml` | 共享配置文件 | SLAM 读取锚点 GNSS、分辨率、原点，自行计算初始位姿 |

> 导航不向 SLAM 发任何 topic 或 service。

---

## 五、消息示例

### Odometry（SLAM 发布，`/slam/localization`）

```python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3

msg = Odometry()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "odom"
msg.child_frame_id = "base_link"

msg.pose.pose.position = Point(x=12.5, y=-3.2, z=0.0)
msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)  # yaw=90°

for i in [0, 7, 14, 21, 28, 35]:
    msg.pose.covariance[i] = 0.01

msg.twist.twist.linear = Vector3(x=0.5, y=0.0, z=0.0)
msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.05)
```
