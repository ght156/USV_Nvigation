# usv_docking

差速 / 双推进器 USV 的 **GNSS 虚拟入口 + Tag 闭环** 精靠泊，以及 **odom 出泊**（v5.0）。

> **任务编排（一键归港/出泊、Nav 预泊、Entry 验收）** 见 [`../dock_mission/README.md`](../dock_mission/README.md)。  
> 本包只管：**预泊点之后 → GNSS 到虚拟入口 → 搜 Tag → 对准 → 倒船 → 停船**；**出泊 → odom 前进离坞**。

---

## 和两个 dock 包分别干什么？

| | **dock_mission** | **usv_docking**（本包） |
|---|------------------|-------------------------|
| **管多远** | 任意位置 → 预泊点（~4 m 外） | 预泊点 → 坞内（最后几米） |
| **用什么导航** | Nav2 + RTK/map | **GNSS 经纬度**（无 RTK 也可用 NavSat）+ AprilTag 视觉 + 里程计航向 |
| **谁启动它** | GCS / `/dock_task/command` | `/dock/start`（靠泊）或 `/dock/undock`（出泊） |
| **配置文件** | `dock_mission/config/dock_database.yaml` | `usv_docking/config/docking_controller_*.yaml` |
| **你要调什么** | 预泊坐标、Nav 容差、入口走廊 | Tag 对准、倒船速度、通道门槛、充电 |

```text
一键归港（推荐）                    手动调试精靠泊
─────────────────                  ─────────────────
/dock/home                         Nav2 手动到预泊点
    → dock_mission                     → deactivate Nav2
    → Nav2 到预泊点                    → docking_handoff.sh start
    → Entry 验收                           → 本包接管
    → /dock/start ──────────────►    本包：GNSS→虚拟入口 → VISION_SEARCH → BACK_IN → STOP
```

**设计原则**：本包 **不能** 从任意位置 Nav 到码头；预泊点必须由 Nav2（或 dock_mission）先送到。本包只做 Tag 闭环的最后几米。

---

## 流程概览

```text
/dock/start（且 Nav2 controller 已 deactivate）
  → PRECHECK
  → GNSS_HEADING_ALIGN → GNSS_BACK_TO_ENTRY → GNSS_ENTRY_SETTLE（虚拟入口，WGS84）
  → VISION_SEARCH_TAG → (无 Tag) SEARCH_SPIN
  → ALIGN_ENTRY → BACK_IN（真中心 depth→0）→ STOP
                                                      ↓
                                            WAIT_CHARGE → 充电稳定 → STOP
```

**出泊**（`undock_controller.py`，与靠泊状态机分离）：

```text
/dock/undock（上层 UNDOCK 经 dock_mission 转发）
  → DOCK_UNDOCK_OUT   # odom 平面累计前进 undock_distance_m
  → DOCK_UNDOCK_SETTLE
  → DOCK_UNDOCK_STOP  # undock_success=true
```

定位只用 **`/odometry/filtered`**（默认话题 `odom_topic`），不依赖 Tag/GNSS；可选锁定起始 yaw。取消靠泊/出泊均用 `/dock/cancel`。

**GNSS**：Gazebo `/roboboat/sensors/gps/navsat` → `workspace_ros` 转发为 **`/gps/fixed_cov`**（`NavSatFix`）。  
段 1：**位置** = GNSS 经纬度；**航向** = odom yaw；进入对齐时 **锁定 leg（起点→虚拟入口）与艉向**，`deyaw` 不再追 live bearing（见 [`docs/GNSS阶段一设计与复盘_20260706.md`](docs/GNSS阶段一设计与复盘_20260706.md)）。  
设 `gnss_approach_enabled: false` 可回退 v4（`WAIT_TAG → APPROACH_ENTRY`）。

传感器断线（Gazebo 关、apriltag 停）：**1.5~3 s 内零速 + ABORT**。

设计参考 [opennav_docking](https://github.com/open-navigation/opennav_docking) 的工程模式（状态机、重试、零速 fail-safe），**控制律为 USV 专用 corridor-gated P 控制**。

---

## 快速上手

### 方式 A：配合 dock_mission（一键归港，推荐）

```bash
# 前置：Gazebo + Nav2 + mission_bridge + apriltag 已启动

# 1. 本包
ros2 launch usv_docking docking_controller.launch.py profile:=sim use_sim_time:=true

# 2. 编排包
ros2 launch dock_mission dock_mission.launch.py profile:=sim use_sim_time:=true

# 3. 一键归港
ros2 topic pub --once /dock/home std_msgs/msg/Bool "{data: true}"
```

**出泊**（船已在坞内、`DOCK_STOP`）：

```bash
ros2 service call /dock_task/command m_common/srv/DockTaskCommand \
  "{command: 3, mission_id: '', command_id: '', require_camera: false}"
# 或直接：ros2 topic pub --once /dock/undock std_msgs/msg/Bool "{data: true}"
```

dock_mission 会在 Entry 通过后自动发 `/dock/start`。**注意**：usv_docking 要求 Nav2 `controller_server` 为 deactivate；若 handoff 后 Nav2 仍在发 cmd_vel，需检查 mission 互锁或手动 deactivate。

### 方式 B：手动交接（只测精靠泊）

```bash
ros2 launch usv_docking docking_controller.launch.py profile:=sim use_sim_time:=true

# 手动 Nav 到预泊点后：
bash src/usv_docking/scripts/docking_handoff.sh start    # deactivate Nav2 + /dock/start
bash src/usv_docking/scripts/docking_handoff.sh cancel   # 取消 + 恢复 Nav2
```

### 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/wuxihik_navigation
colcon build --packages-select usv_docking --symlink-install
source install/setup.bash
```

---

## 配置文件地图（改哪个文件？）

```text
usv_docking/config/
├── docking_controller_sim.yaml   ← 全部参数 ★ 主文件
├── dock_geometry_sim.yaml        ← 坞中心 GNSS + virtual_entry_standoff_m ★
└── dock_bays_sim.yaml            ← 泊位元数据参考（Tag ID、预泊 hint）

dock_mission/config/
└── dock_database.yaml            ← 预泊点坐标（Nav 阶段，不是本包算出来的）
```

**预泊点坐标不在本包 yaml 里改**，在 `dock_mission/config/dock_database.yaml` 的 `map_staging` / `gnss_staging`。

---

## 调参指南（按优先级）

参数很多，按**你遇到的问题**往下找，不要从头改一遍。

### ★ 第一优先级：必对项（不对就完全跑不起来）

| 参数 | 文件区块 | 说明 |
|------|----------|------|
| `camera_frame` | 坐标系 | 默认 `camera_rear_link`，须与 apriltag 一致 |
| `dock_pose_topic` | 话题接口 | 默认 `/apriltag_node/dock_pose` |
| `gnss_topic` | GNSS | 默认 `/gps/fixed_cov` |
| `dock_geometry_file` | 坞几何 | `dock_geometry_sim.yaml` |
| `heading_offset_rad` | 位姿符号 | 后向相机通常 ≈ π，使对准通道时 `heading_error≈0` |
| `invert_x/y/yaw` | 位姿符号 | 与 URDF / 标定一致，不对则倒船方向反 |
| `require_mission_idle` | Mission 互锁 | `true` 时 mission_bridge 须 IDLE 才开跑 |

位姿语义详解：[`docs/归港对准与位姿说明.md`](docs/归港对准与位姿说明.md)（段 2）  
GNSS 段 1 设计复盘：[`docs/GNSS阶段一设计与复盘_20260706.md`](docs/GNSS阶段一设计与复盘_20260706.md)

---

### ★ 第二优先级（v5）：GNSS 虚拟入口（「对齐后仍一直修方向」）

| 参数 | 默认(sim) | 何时改 |
|------|-----------|--------|
| `gnss_align_yaw_tol` | 0.08 rad | 对齐过严/过松 |
| `gnss_back_heading_correct_tol` | 0.12 rad | 停倒大修阈值；过大则少修，过小则频繁 `correct` |
| `gnss_back_steer_deadband` | 0.04 rad | 纯倒 vs walk_fix 分界 |
| `gnss_entry_yaw_tol` | 0.10 rad | 到点艉向（与 relax 无关，须 dist+yaw） |
| `dock_geometry_file` | `dock_geometry_sim.yaml` | 虚拟入口 GNSS；**world GPS datum 须一致** |

联调看 `/dock/status`：`gnss_leg_locked`、`gnss_deyaw`、`gnss_back_mode`、`gnss_leg_remaining_m`。

---

### ★ 第三优先级：预泊 / 搜 Tag（「到了预泊点但不动」）

| 参数 | 默认(sim) | 何时改 |
|------|-----------|--------|
| `wait_tag_timeout_sec` | 10 | 禁用自转时等 Tag 上限 |
| `enable_tag_search_spin` | true | 预泊点看不到 Tag 时是否自转搜 |
| `tag_search_spin_speed` | 0.25 | 自转太快易漏检 → 调低 |
| `tag_search_spin_angle_rad` | 2π | 须转满才判失败 |
| `tag_acquire_cycles` | 5 | 连续 N 帧 Tag 才离开 WAIT |
| `max_retries` | 2 | 搜 Tag 失败自动重试次数 |

**预泊点本身**在 dock_mission 的 `map_staging`：船尾朝码头、后向相机能看到 Tag（约离坞 4 m）。

---

### ★ 第四优先级：对准 / 通道门槛（「有 Tag 但不进 BACK_IN」）

| 参数 | 默认(sim) | 含义 |
|------|-----------|------|
| `prealign_yaw_limit` | 0.25 rad | PRE_ALIGN 完成阈值 |
| `corridor_enter_y_limit` | 0.35 m | 允许进 BACK_IN 的 \|y\| |
| `corridor_enter_yaw_limit` | 0.15 rad | 允许进 BACK_IN 的 \|heading\| |
| `outside_centering_timeout_sec` | 60 | 码头外弧线纠偏超时 |
| `outside_speed` | 0.10 m/s | 码头外线速度 |
| `skip_outside_if_heading_ready` | true | 航向已准可跳过 OUTSIDE |

U 型坞侧偏大：先调 dock_mission 重 Nav 预泊 + Entry 走廊；仍进不了 BACK_IN 再放宽 `corridor_enter_*`（谨慎）。

---

### ★ 第五优先级：倒船入坞（「进了 BACK_IN 但撞/偏/停不住」）

| 参数 | 默认(sim) | 含义 |
|------|-----------|------|
| `back_in_speed` | 0.15 m/s | 倒船线速度 |
| `back_in_backward_projection_m` | 0.25 m | 目标后移，减少末端抖动 |
| `kx` / `ky_back` / `kyaw_back` | 0.25/0.20/0.40 | 倒船 P 增益 |
| `back_in_y_limit` / `back_in_yaw_limit` | 0.55/0.25 | 倒船中超限 → ABORT |
| `tag_loss_in_corridor_sec` | 0.15 | BACK_IN 中 Tag 丢失立即 ABORT |

---

### ★ 第六优先级：停船 / 成功判定

**仿真**（`require_charging_confirm: false`）：

| 参数 | 默认 | 含义 |
|------|------|------|
| `stop_x_threshold` | 0.40 m | \|x_base\| 到位 |
| `stop_y_threshold` | 0.25 m | \|y_base\| 到位 |
| `yaw_tolerance` | 0.12 rad | \|heading_error\| 到位 |
| `settle_cycles` | 5 | 连续 N 帧满足 → DOCK_STOP |

**充电确认**（`require_charging_confirm: true` 时启用）：

| 参数 | 默认 | 含义 |
|------|------|------|
| `charge_confirm_hold_sec` | 4.0 s | charging=true **稳定**多久算成功 |
| `charge_confirm_timeout_sec` | 60 | 等充电超时 |
| `charge_pause_on_pose_settle` | true | 视觉到位但未充电 → 先停住等 |

---

### 一般不用动：安全 / 看门狗

| 参数 | 说明 |
|------|------|
| `odom_timeout_sec` | 里程计断流判定阈值 |
| `resume_on_sensor_recovery` | 断流时零速保态，恢复后继续（默认 true） |
| `sensor_hold_abort_sec` | 连续断流超过此秒数才 ABORT |
| `dock_pose_msg_timeout_sec` | apriltag 话题断流判定 |
| `max_docking_duration_sec` | 单次入泊总时长上限 120 s |
| `halt_on_sensor_loss` | 传感器异常零速 |

---

## 依赖

| 组件 | 说明 |
|------|------|
| `apriltag_localization` | `/apriltag_node/dock_pose` |
| TF | `camera_rear_link` → `base_link` |
| `mission_bridge` | `/mission_bridge/state`，默认要求 `IDLE` |
| Nav2 | 入泊前须 **deactivate** `controller_server` |
| `converter`（仿真） | `/cmd_vel_nav` → Gazebo 推力 |
| `dock_mission`（可选） | 自动 Nav 预泊 + handoff |

---

## 话题接口

| 方向 | 话题 | 类型 |
|------|------|------|
| 订 | `/apriltag_node/dock_pose` | `Float64MultiArray` |
| 订 | `/odometry/filtered` | `nav_msgs/Odometry` |
| 订 | `/dock/start` | `Bool` |
| 订 | `/dock/undock` | `Bool` |
| 订 | `/dock/cancel` | `Empty` |
| 订 | `/mission_bridge/state` | `String` |
| 订 | `/wireless_charging/is_charging` | `Bool` |
| 发 | `/cmd_vel_nav` | `Twist` |
| 发 | `/dock/status` | `String` (JSON) |

与 dock_mission 协作时还会用到：`/dock/home`（上层）、`/dock/mission_status`（编排状态）、`/dock/speed_authority`（若接仲裁器）。

---

## `/dock/status` JSON 字段（精简，约 22 项）

| 字段 | 说明 |
|------|------|
| `state` / `phase` / `docking_stage` | 状态机 |
| `success` / `needs_reapproach` / `abort_reason` / `error_code` | 靠泊结果（`dock_mission` 订阅 `success`） |
| `undock_success` / `undock_traveled_m` / `session_mode` | 出泊结果（`dock_mission` 订阅 `undock_success`） |
| `docking_time_sec` | 本次归港耗时 |
| `x_base`, `y_base`, `heading_error` | 段 2 控制位姿 |
| `dist_to_virtual_entry`, `gnss_deyaw`, `gnss_back_mode` | 段 1 GNSS |
| `gnss_leg_locked`, `gnss_leg_remaining_m` | 段 1 leg 锁定与剩余距离 |
| `approach_depth_m`, `entry_x_delta` | 深度 `\|x_base\|` 及与虚拟入口偏差 |
| `pose_valid`, `tag_fresh`, `tf_error` | 感知健康 |
| `cmd_linear_x`, `cmd_angular_z` | 当前输出 |
| `back_in_blind_active`, `back_in_holding_for_tag` | 段2 Tag 丢失 |
| `pose_settled` | 段2 到位判据 |
| `motion_sensors_ok`, `sensor_hold_*` | 传感器暂停 |

联调：`ros2 topic echo /dock/status`（长输出加 `--full-length`）。

### error_code 速查

| 码 | 典型 abort_reason |
|----|-------------------|
| 10–14 | Tag：`WAIT_TAG_TIMEOUT`, `TAG_SEARCH_*`, `TAG_LOST_IN_CORRIDOR` |
| 20–21 | 传感器：`ODOM_LOST`, `DOCK_POSE_STREAM_LOST` |
| 31–33 | 通道/超时：`CORRIDOR_VIOLATION_*`, `MAX_DOCKING_DURATION` |
| 34–36 | 充电：`CHARGE_CONFIRM_TIMEOUT`, `CHARGE_POSE_LOST`, `CHARGE_STATUS_LOST` |
| 37 | `APPROACH_ENTRY_TIMEOUT`（第一次到港超时） |
| 44–45 | 出泊：`UNDOCK_TIMEOUT`, `UNDOCK_ODOM_LOST` |

出泊主参（`docking_controller_*.yaml` 末尾）：`undock_distance_m`（默认 5 m）、`undock_speed`、`undock_heading_hold`。

---

## 状态机（v4 两次到港）

```text
DOCK_IDLE → PRECHECK → WAIT_TAG ──(无 Tag)──► SEARCH_SPIN
                │
                ▼
      DOCK_APPROACH_ENTRY    ★ Tag 闭环，depth → entry_standoff_m（虚拟入口中心）
                │
                ▼
           DOCK_BACK_IN       ★ Tag 闭环，depth → 0（真坞中心）
                │
                ▼
            DOCK_STOP / DOCK_WAIT_CHARGE
```

全程 **Tag→base_link** 同一套 P 控制，只改深度目标；odom 用于搜 Tag 转角、传感器看门狗，**出泊段则全程用 odom 计距**。

代码：`docking_controller.py`（靠泊 + ROS 薄层）、`undock_controller.py`（出泊状态机）、`undock.py`（距离/速度辅助）。

---

## 与 opennav_docking 的对应

| opennav 概念 | usv_docking 实现 |
|--------------|------------------|
| `DockRobot` phase | `/dock/status.phase` |
| `doInitialPerception` | `WAIT_TAG` / `SEARCH_SPIN` |
| `backward_projection` | `back_in_backward_projection_m` |
| `DockDatabase` / staging | **预泊在 dock_mission**；本包 `dock_bays_*.yaml` 仅参考 |
| `ChargingDock` | 充电话题 + `DOCK_WAIT_CHARGE` |
| 螺旋控制律 | **未用**；corridor-gated P 控制 |

---

## 约束

- 独立包，不改 Nav2 / `apriltag_localization` 源码。
- 入泊前须 deactivate Nav2 `controller_server`（handoff 脚本或 dock_mission 流程）。
- 本包 **不做** 长距离 Nav；预泊由 Nav2 / dock_mission 负责。
- `DOCK_*` 状态不写入 `mission_bridge.task.state`。

---

## 相关文档

| 文档 | 内容 |
|------|------|
| [`../dock_mission/README.md`](../dock_mission/README.md) | 一键归港、预泊点、Entry、Nav GoalChecker |
| [`../../docs/usv_docking任务规划.md`](../../docs/usv_docking任务规划.md) | 全栈架构 |
| [`../../docs/项目运行与联调.md`](../../docs/项目运行与联调.md) | 7 终端联调 |
| [`docs/GNSS阶段一设计与复盘_20260706.md`](docs/GNSS阶段一设计与复盘_20260706.md) | v5 GNSS leg lock、三档倒船、联调复盘 |
| [`docs/CHANGELOG_v5_gnss_approach.md`](docs/CHANGELOG_v5_gnss_approach.md) | v5 变更摘要 |
| [`docs/归港对准与位姿说明.md`](docs/归港对准与位姿说明.md) | x_base / heading_error 语义 |
