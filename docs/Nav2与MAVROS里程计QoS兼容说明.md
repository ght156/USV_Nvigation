# Nav2 与 MAVROS 里程计 `/mavros/gps_input/local` 的 QoS 兼容说明

> 日期：2026-09-04
> 结论：**保持 mavros 不动，把所有 odom 订阅端统一为 BEST_EFFORT**（因为高频里程计用 RELIABLE 会在 DDS 层因背压丢数据/卡顿）。

---

## 1. 现象

- Nav2 / mission_bridge 启动后反复出现：
  ```
  New publisher discovered on topic '/mavros/gps_input/local', offering incompatible QoS.
  No messages will be received from it. Last incompatible policy: RELIABILITY
  ```
- `mission_bridge` 卡在：
  ```
  Waiting for odometry before sending waypoint
  ```
- 但主导航仍能沿路径行驶（因为 TF 链正常）。

## 2. 根因

`/mavros/gps_input/local`（`nav_msgs/Odometry`，frame `odom`→`base_link`）上，发布方与订阅方的 **RELIABILITY 策略不一致**：

| 端 | 实现 | 一致性策略 |
|---|---|---|
| mavros `gps_input` 插件（发布 `~/local`） | `rclcpp::SensorDataQoS()` | **BEST_EFFORT**（KEEP_LAST 5，VOLATILE） |
| Nav2 `controller_server`（`nav_2d_utils::OdomSubscriber`） | `rclcpp::SystemDefaultsQoS()` | **RELIABLE**（KEEP_LAST 10，VOLATILE） |
| Nav2 `bt_navigator` / `velocity_smoother`（`nav2_util::OdomSmoother`） | `rclcpp::SystemDefaultsQoS()` | **RELIABLE** |
| 导航侧自定义节点（`mission_bridge`/`zone_monitor`/`nav_status_aggregator`/`waypoint_with_state`） | rclpy 默认 QoS | **RELIABLE** |
| `usv_map_odom_tf` | `rclcpp::SensorDataQoS()` | **BEST_EFFORT**（兼容，正常） |

ROS 2 规则：**BEST_EFFORT 的发布方不能喂给 RELIABLE 的订阅方** → 这两端不建立连接、不传数据。

为何导航仍能走：导航主链靠 **TF**（`map`→`odom`→`base_link`），而这段 TF 由 `usv_map_odom_tf`（BEST_EFFORT，兼容）+ mavros 提供，所以坐标/规划/跟踪不受影响。

## 3. 方案选择：为什么改「全部 BEST_EFFORT」

- **方案 A（改 mavros 发布端为 RELIABLE）**：一处解决所有节点；但高频传感器/里程计用 RELIABLE，若订阅方跟不上，DDS 层会因背压而产生**丢数据、延迟、卡顿**。一般不推荐用于高频流。
- **方案 B（保持 mavros 不动，全部订阅端 BEST_EFFORT，本文采用）**：里程计属高频传感器数据，best-effort 是 ROS 2 的标准做法，允许少量丢弃不影响导航。

代价：Nav2 的 `controller_server`/`bt_navigator`/`velocity_smoother` 的 odom 订阅是**源码写死** `SystemDefaultsQoS()`，参数配置改不了，所以必须改 Nav2 源码并重编。

## 4. 改动清单

### 4.1 Nav2 源码（`/home/ght/nav2_ws`）`SystemDefaultsQoS()` → `SensorDataQoS()`

| 文件 | 影响的节点 |
|---|---|
| `nav2_util/src/odometry_utils.cpp`（两处） | `OdomSmoother` → `bt_navigator` + `velocity_smoother` |
| `nav2_dwb_controller/nav_2d_utils/include/nav_2d_utils/odom_subscriber.hpp` | `OdomSubscriber` → `controller_server` |
| `nav2_behavior_tree/plugins/condition/is_stuck_condition.cpp` | BT 的 `is_stuck` 条件（其订阅话题为 `/odom`） |

### 4.2 导航侧 / 泊靠自定义节点（`/home/ght/USV_NAV`），odom 订阅加 `BEST_EFFORT` QoSProfile

- `src/USV_NAV/workspace_nav/workspace_nav/mission_bridge.py`
- `src/USV_NAV/workspace_nav/workspace_nav/zone_monitor.py`
- `src/USV_NAV/workspace_nav/workspace_nav/nav_status_aggregator.py`
- `src/USV_NAV/workspace_nav/workspace_nav/waypoint_with_state.py`
- `src/dock_mission/dock_mission/entry_validator_node.py`（dock 入泊校验，`odom_topic` 实船为 `/mavros/gps_input/local`）
- `src/usv_docking/usv_docking/docking_fsm.py`（仅 odom 订阅单独用 best_effort，其余 topic 的共享 `qos` 保持不变）
- `src/USV_NAV/workspace_nav/workspace_nav/waypoint_transform.py`（**原本已是** BEST_EFFORT，无需改动）

### 4.3 本次一并修复（非 QoS，记录备查）

- `src/m_common/CMakeLists.txt`：补回 `msg/GeoPolygon.msg`、`msg/NavZones.msg`、`srv/GetNavZones.srv`、`srv/SetNavZones.srv` 的 `rosidl_generate_interfaces()` 注册。
- 把 `GetNavZones.srv`、`SetNavZones.srv` 从 `msg/` 挪回 `srv/`（复制时放错了目录）。

## 5. 编译 / 部署

```bash
# Nav2（改了源码必须重编；且 Nav2 跑在 NX 上，需同步到 NX 对应工作区并生效）
cd /home/ght/nav2_ws
colcon build --packages-select nav2_util nav2_dwb_controller \
  nav2_behavior_tree nav2_bt_navigator nav2_velocity_smoother \
  nav2_controller --symlink-install
```

- 自定义节点：Python 节点，走 symlink-install 直接**重启**即可；否则重编 `workspace_nav`。
- `m_common`：接口注册改动需重编。

### 5.2 部署到 NX（运行栈所在）

Nav2 与导航栈实际跑在 NX（`/home/jetson/usv`）。以本机为源同步过去并生效：

1. 把改动同步到 NX 对应工作区（源码 + 文档）；
2. 在 NX 上重编 Nav2（同 §5.1 的 `colcon build --packages-select ...`，路径按 NX 实际 Nav2 工作区调整）；
3. 在 NX 上重编 `m_common`（接口注册改动），并视情况重编 `workspace_nav`、`dock_mission`、`usv_docking`（若未走 symlink-install）；
4. 重新 `source install/setup.bash`，重启相关节点 / bringup。

## 6. 影响

| 环节 | 改前 | 改后 |
|---|---|---|
| 定位 / TF（`map`→`odom`→`base_link`） | ✅ 正常 | ✅ 不变 |
| 规划 / 全局路径 / 代价地图 | ✅ 正常 | ✅ 不变 |
| 沿路径行驶 | ✅ 能走 | ✅ 不变 |
| `controller_server` odom 速度反馈 | 无（开环） | 有（闭环，RPP 可按实际速度自适应） |
| `bt_navigator` / `velocity_smoother` odom 速度 | 无 | 有 |
| `mission_bridge` 发航点 | 卡在 "Waiting for odometry" | 正常 |
| `zone_monitor` / `nav_status_aggregator` | 收不到 odom | 收到 |

## 7. 验证

```bash
# 看发布/订阅 QoS 是否全部 BEST_EFFORT、是否还有 incompatible
ros2 topic info /mavros/gps_input/local -v

# best_effort 订阅应能拿到实时 odom
ros2 topic echo /mavros/gps_input/local --qos-reliability best_effort --once

# mission_bridge 不再出现 "Waiting for odometry"
```

## 8. 遗留 / 注意

- `is_stuck_condition` 订阅的话题是字面量 `"odom"`（即 `/odom`），**不是** `/mavros/gps_input/local`。若 BT 里用到 is_stuck，需单独确认 `/odom` 是否有数据。
- Nav2 是源码改动，后续升级 Nav2 需重新打补丁。
- `nav_status_aggregator` 的 GPS 订阅 `/gps/fixed_cov` 仍为默认 QoS；若该发布端（GI320 GNSS 驱动）也是 best_effort，需同样处理（当前不在本次 odom 范围内）。
- `DockTaskCommand`：`/dock_task/command` service 已于 2026-07-28 从 `m_common` **故意删除**；`dock_mission_node.py` 用 `try/except ImportError` 优雅降级（不创建该 service），**并非缺失文件，无需补**。
