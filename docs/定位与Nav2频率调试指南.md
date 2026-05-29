# 定位与 Nav2 频率调试指南

## 症状

- RViz 中船体模型/方框**每次更新位置跳跃较大**（不连续）
- 船走 **S 线**（尤其小 lookahead 时明显，如 1.4-3.0 m）
- 速度和角速度数值正常，但轨迹振荡

## 根因判断

**定位输入不连续 → Nav2 控制器被迫跟随跳变的 `base_link` 位姿计算 carrot 点。**

不是 lookahead 的主因。大 lookahead（8-16m）可以"低通滤波"掉定位跳变造成的角度误差；小 lookahead（1.4-3m）太敏感，每次位置跳变 RPP 就重新算一个很近的 carrot，角速度来回修正，产生 S 线。

---

## 频率链路总览

### 当前定位链路（mavros_odom 模式，默认）

```
PX4 EKF → MAVROS /mavros/local_position/odom → odom→base_link TF
GNSS + odom → gnss_odom_map_tf → map→odom TF（首次锁定后 20Hz 重发缓存）
```

| 节点 | 发布内容 | 频率 |
|------|---------|------|
| MAVROS (`/mavros/local_position/odom`) | odom→base_link | **取决于 PX4 MAVLink 流率（关键瓶颈）** |
| `gnss_odom_map_tf` | map→odom | 首次锁定后 20Hz 重发（静态值） |
| EKF (`robot_localization`) | odom→base_link | 配置 30Hz，**当前未启用** |
| `navsat_transform` | UTM 坐标 | 配置 15Hz，**当前未启用** |

### Nav2 控制链路

| 节点 | 频率 | 说明 |
|------|------|------|
| `controller_server` (RPP) | 10 Hz | 控制器计算新速度指令 |
| `velocity_smoother` | 20 Hz | 平滑输出 |
| `nav2_cmd_vel_to_mavros` | 20 Hz | 速度桥到 PX4 |
| `local_costmap` | 5 Hz | 更新/发布 |
| `global_costmap` | 1 Hz | 更新/发布 |
| `planner_server` | 1 Hz | 全局路径重规划 |

### 实船建议频率

```
/mavros/local_position/odom : ≥ 20 Hz
/tf                          : 30-50 Hz
controller_server            : 5-10 Hz
local_costmap                : 5 Hz
planner                      : 0.5-1 Hz
```

---

## 调试命令

### 1. 查看 odom 实际发布频率

```bash
ros2 topic hz /mavros/local_position/odom
```

跑 15-20 秒看稳定后的平均值。这是 `odom→base_link` TF 的实际更新频率，是定位链路的**核心瓶颈**。

### 2. 查看 TF 发布频率

```bash
ros2 topic hz /tf
```

注意：`/tf` 包含所有静态+动态 TF，频率会被稀释。重点还是看 odom。

### 3. 查看 odom 时间戳是否连续（判断跳变）

```bash
ros2 topic echo /mavros/local_position/odom --field header.stamp
```

观察秒数/nanosec 是否均匀递增，有无明显跳变或重复。

### 4. 监控 TF 链路延迟

```bash
ros2 run tf2_ros tf2_monitor map base_link
ros2 run tf2_ros tf2_monitor odom base_link
```

查看 TF 链的发布者和实际延迟。

### 5. 生成 TF 树可视化

```bash
ros2 run tf2_tools view_frames
```

生成 `frames.pdf`，包含各 TF 发布者和平均频率。

### 6. 批量对比关键话题频率

```bash
ros2 topic hz /mavros/local_position/odom &
ros2 topic hz /mavros/global_position/global &
ros2 topic hz /tf &
sleep 15 && kill %1 %2 %3
```

### 7. 查看 Nav2 控制器实际计算频率

```bash
ros2 topic hz /cmd_vel_nav
```

### 8. 提高 MAVROS stream rate（如果 odom 频率太低）

```bash
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  "{stream_id: 0, message_rate: 20, on_off: true}"
```

### 9. RViz Fixed Frame 切换法定位跳变来源

| Fixed Frame | 现象 | 结论 |
|-------------|------|------|
| `odom` | 平滑 | 问题在 `map→odom` |
| `odom` | 也跳 | 问题在 `/mavros/local_position/odom` 或 `odom→base_link` |
| `map` | 跳、`odom` 平滑 | 问题在 `map→odom` TF 链路 |

---

## 参数建议

### RPP 控制器（当前配置 vs 建议）

| 参数 | 当前值 | 建议值 | 原因 |
|------|--------|--------|------|
| `desired_linear_vel` | 1.2 | 0.8-1.2 | 速度越高定位跳变影响越大 |
| `lookahead_time` | 5.0 | 4.0-5.0 | — |
| `min_lookahead_dist` | 8.0 | 6.0-8.0 | 大前瞻滤除定位跳变引起的角度误差 |
| `max_lookahead_dist` | 16.0 | 14.0-16.0 | — |
| `max_angular_accel` | 1.0 | **0.3-0.5** | 水面惯量大，减小抑制 overshoot |
| `transform_tolerance` | 0.2 | **0.3-0.5** | 放宽 TF 查询容差 |

### velocity_smoother 速度限幅建议

```yaml
max_velocity: [0.5, 0.0, 0.5]      # 保持
max_accel: [0.3, 0.0, 0.3]         # 从 0.5 降低，减少冲击
deadband_velocity: [0.05, 0.0, 0.05] # 适当放宽
```

---

## 根本解决方案

### 方案 A：调高 PX4 MAVLink 流率（推荐先试）

在 QGC 参数中搜索 `SR` 或 MAVLink stream rate，将 `LOCAL_POSITION_NED` 流率调到 **20-30Hz**。或者用上面的 `set_stream_rate` 服务调用。

### 方案 B：启用 robot_localization EKF（mA 不够时）

切换到 `robot_localization` 模式（`localization_backend:=robot_localization`），EKF 以 **30Hz** 发布 `odom→base_link`，提供插值和平滑，位置更新会明显流畅。代价是需要确保 IMU+GPS 话题正常。

```bash
ros2 launch workspace_ros real_boat_bringup.launch.py localization_backend:=robot_localization
```

### 方案 C：保持大 lookahead（权宜但有效）

如果定位频率暂时无法改善，保持当前 8-16m 大 lookahead，配合降低 `max_angular_accel`，可以有效抑制 S 线。

---

## 检查清单

- [ ] `ros2 topic hz /mavros/local_position/odom` ≥ 20 Hz？
- [ ] `ros2 topic hz /cmd_vel_nav` ≈ 10 Hz（与 controller_frequency 一致）？
- [ ] RViz Fixed Frame 切 `odom` 后是否平滑？
- [ ] `ros2 topic echo /mavros/local_position/odom --field header.stamp` 时间戳连续？
- [ ] `/tf` 频率 ≥ 30 Hz？
- [ ] `max_angular_accel` 已降到 0.3-0.5？
- [ ] lookahead 保持 ≥ 6m（不要退回 1.4-3m）？
