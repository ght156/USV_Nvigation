# usv_docking — 自动归港 / 倒船入泊任务规划

> **版本**：v5.0（GNSS 虚拟入口 + Mission 编排）| **状态**：v5 GNSS 段 1 仿真定稿（2026-07-06）；实船 RTK 标定与全栈联调待验  
> **工作区**：`wuxihik_navigation`（仿真）→ 同步 `USV_NAV`（实船）  
> **原则**：`usv_docking` = 最后几米精靠泊；**`dock_mission`** = Nav 编排 + 入口验收 + handoff；不改 Nav2 / apriltag 源码。

### 版本摘要

| 版本 | 要点 |
|------|------|
| v2.1~v2.4 | corridor-gated、Tag 闭环、搜 Tag 自转、充电确认 |
| v3.0 | **Mission FSM** + Entry Validator + dock_mission 编排 |
| **v5.0** | **段1 GNSS** 锁定 leg 逼近虚拟入口 → **段2 Tag** ALIGN/BACK_IN（[`GNSS阶段一设计与复盘_20260706.md`](../src/usv_docking/docs/GNSS阶段一设计与复盘_20260706.md)） |

### 核心实现要求（Cursor 必读）

```text
1. 禁止「看到 dock_pose 就直接倒车」。
2. 差速 / 双推进器 USV 不具备横向平移：v=0 时只能修 yaw，不能修 y。
3. 横向 + 航向纠偏必须在码头外（OUTSIDE_CENTERING）用低速前进/倒车弧线完成。
4. GATE_CHECK：|y| 与 |yaw| 均小于入口阈值，才允许 BACK_IN。
5. BACK_IN 以低速直倒为主，只允许小角度微调；y/yaw 超限 → ABORT，回预泊点重来。
6. dock_pose 经 TF 转到 base_link 后再控制；目标为船坞中心（含 dock_offset）。
```

---

## 1. 目标与真实码头约束

### 1.1 窄通道 + 差速 kinematics

```text
两侧浮台 / 岸壁 → 窄通道
双推进器差速船 → 无侧推、不能原地横移
```

| 错误假设 | 正确做法 |
|----------|----------|
| 通道内 `v=0` 同时修 y 和 yaw | **v=0 只能修 yaw**；修 y 必须靠 **v≠0 的弧线** |
| BACK_IN 中大幅纠偏 y | **BACK_IN 仅微调**；大幅纠偏在 **码头外** 完成 |
| 通道内超限继续斜倒 | **ABORT** → 重新 Nav2 到预泊点 |

### 1.2 任务链

```text
1. Nav2 → 预泊点（码头正前方 3~5 m，船尾朝码头，mission IDLE）
2. deactivate controller_server
3. apriltag → /apriltag_node/dock_pose
4. usv_docking：PRE_ALIGN → OUTSIDE_CENTERING → GATE_CHECK → BACK_IN → STOP
5. converter / 实船速度桥 → 推进器
```

---

## 2. 位姿语义与「能否判断对齐走廊」

### 2.1 AprilTag 输出含义

```text
Tag 检测 → × dock_offset_x/y/yaw → 船坞中心相对相机的位姿
Float64MultiArray [x, y, z, roll, pitch, yaw]  （相机系，m / rad）
```

**可以**用其判断相对码头中心线的偏差，**前提**：

1. `dock_offset_*` 与真实码头几何标定正确  
2. 输出代表 **船坞中心**，不是原始二维码中心  
3. **必须** `camera_frame → base_link` TF 后再控制  

相机装偏时，若跳过 TF，会系统性斜入库。

**读数注意**：`/apriltag_node/dock_pose` 在 **相机系**；船原地转 yaw 时 x/y 可大幅变化。控制与门槛判断一律用 TF 后的 **`x_base / y_base / yaw_base`**（`/dock/status`）。

### 2.2 控制用误差（base_link 系）

```text
x_base：沿船体前后轴相对船坞中心的距离（倒车时主要关注 |x|）
y_base：横向偏离船坞中心线（**必须在入通道前消小**）
yaw_base：船体与中心线夹角
```

实现：`pose_transform.py`（TF + yaml `invert_*`）+ `pose_filter.py`（EMA）。

| 参数 | 默认 |
|------|------|
| `camera_frame` | 仿真 **`camera_rear_link`**；实船 **`camera_left_link`**（与 apriltag `frame_id` 一致） |
| `robot_frame` | `base_link` |
| `allow_camera_frame_fallback` | `false` |

---

## 3. 包结构

```text
src/usv_docking/
├── README.md                    # 包说明（编译、话题、联调）
├── usv_docking/
│   ├── docking_controller.py
│   ├── pose_transform.py
│   └── pose_filter.py
├── config/
│   ├── docking_controller_sim.yaml
│   └── docking_controller_real.yaml
├── launch/
│   └── docking_controller.launch.py
└── scripts/
    └── docking_handoff.sh
```

---

## 4. 话题接口

| 方向 | 话题 | 类型 |
|------|------|------|
| 订 | `/apriltag_node/dock_pose` | `Float64MultiArray` |
| 订 | `/dock/start` | `Bool` |
| 订 | `/dock/cancel` | `Empty` |
| 订 | `/mission_bridge/state` | `String` |
| 发 | `/cmd_vel_nav` | `Twist` |
| 发 | `/dock/status` | `String` (JSON) |

JSON 建议字段：`state`, `x_base`, `y_base`, `yaw_base`, `gate_ready`, `gate_ready_count`, `gate_ready_cycles`, `tag_age_sec`, `pose_valid`, `tf_error`, `abort_reason`, `needs_reapproach`, `mission_state`, `cmd_linear_x`, `cmd_angular_z`。

---

## 5. 状态机（v2.2）

### 5.1 状态一览

| 状态 | 位置 | 作用 |
|------|------|------|
| `DOCK_IDLE` | — | 未接管 |
| `DOCK_PRECHECK` | — | mission 互锁 |
| `DOCK_WAIT_TAG` | 预泊点 | 静止等待 Tag（Nav2 容差大时 Tag 可能不在视野） |
| **`DOCK_SEARCH_SPIN`** | **预泊点** | **无 Tag 时原地慢速自转一圈搜 Tag，再判失败** |
| **`DOCK_PRE_ALIGN`** | **码头外** | **v=0，只对 yaw，船尾大致对准中心线** |
| **`DOCK_OUTSIDE_CENTERING`** | **码头外** | **低速前进/倒车弧线，同时修 y 和 yaw** |
| **`DOCK_GATE_CHECK`** | **入口** | **判定是否允许进通道** |
| **`DOCK_BACK_IN`** | **通道内** | **低速直倒 + 小角微调** |
| `DOCK_STOP` | 泊位内 | 到位，零速 |
| `DOCK_ABORT` | — | 失败；`needs_reapproach=true` 时要求回预泊点 |

### 5.2 状态图

```text
DOCK_IDLE → PRECHECK → WAIT_TAG（静止 wait_tag_stationary_sec）
                          │ 无 Tag
                          ▼
                    DOCK_SEARCH_SPIN      v=0, ω=tag_search_spin_speed，累计 2π
                          │ 找到 Tag / 转满一圈仍无
                          ▼
                    DOCK_PRE_ALIGN        v=0, ω=f(yaw)
                          │ |yaw| < prealign_yaw_limit
                          ▼
                 DOCK_OUTSIDE_CENTERING   弧线修 y+yaw（v 可正可负）
                          │ 周期性进入
                          ▼
                    DOCK_GATE_CHECK       检查入口阈值
                          │ pass
                          ▼
                    DOCK_BACK_IN          直倒 + 微调
                          │ settle
                          ▼
                    DOCK_STOP

GATE_CHECK fail → 回 OUTSIDE_CENTERING（仍在码头外）
BACK_IN 内 y/yaw 超限 / Tag 丢 → DOCK_ABORT（需 reapproach）
```

### 5.3 分状态规则

#### DOCK_PRE_ALIGN（码头外）

```text
v = 0
ω = clamp(kyaw_pre * yaw_base, ±max_yaw_rate)
目的：先把船尾方向大致对准码头中心线
转移：|yaw_base| < prealign_yaw_limit → OUTSIDE_CENTERING
```

#### DOCK_OUTSIDE_CENTERING（码头外 — **横向纠偏主阶段**）

差速船通过 **v 与 ω 组合** 走弧线，同时缩小 y 和 yaw：

```python
# 根据 y、yaw 符号选择倒车或前进（参数可固定 prefer_reverse:=true）
if prefer_outside_reverse:
    v = -outside_speed          # 默认小倒车，如 0.08~0.12 m/s
else:
    v = outside_speed if need_forward else -outside_speed

ω = clamp(ky * y_base + kyaw * yaw_base, ±max_yaw_rate)
```

```text
· 允许 |v| ≤ outside_speed（前进或倒车，yaml 配置）
· 禁止 |v| > outside_speed（码头外也不猛冲）
· 每 control_cycle 可进入 GATE_CHECK 子判断（或独立 GATE_CHECK 状态）
· 若长时间（outside_centering_timeout_sec）未过 gate → ABORT
```

#### DOCK_GATE_CHECK（入口判据）

**仅当同时满足** 才进入 `DOCK_BACK_IN`：

```text
|y_base|   < corridor_enter_y_limit
|yaw_base| < corridor_enter_yaw_limit
tag_valid && TF OK
```

不满足 → 回到 `DOCK_OUTSIDE_CENTERING`（**仍在码头外弧线修**）。

建议默认：

```yaml
corridor_enter_y_limit: 0.35
corridor_enter_yaw_limit: 0.15    # ~8.6°
```

#### DOCK_BACK_IN（通道内 — **只微调，不大幅纠偏**）

```text
前提：已通过 GATE_CHECK，视为已进入通道
策略：低速直倒为主，只允许小角度修正

v = -clamp(kx * |x_base|, min_back_in_speed, back_in_speed)
    # back_in_speed < max_reverse_speed，如 0.12~0.20

ω = clamp(ky_back * y_base + kyaw_back * yaw_base, ±back_in_max_yaw_rate)
    # back_in_max_yaw_rate < max_yaw_rate，如 0.15~0.20

禁止：
  · 为修 y 而大幅打角 + 大倒车（斜着冲进岸壁）
  · |y| > back_in_y_limit 或 |yaw| > back_in_yaw_limit 时继续硬修

超限处理：
  · 立刻 v=0, ω=0 → DOCK_ABORT
  · abort_reason = CORRIDOR_VIOLATION_IN_BACK_IN
  · needs_reapproach = true（操作员重新 Nav2 到预泊点）
```

**BACK_IN 内 Tag 丢失**：立即零速 → ABORT（`TAG_LOST_IN_CORRIDOR`），`needs_reapproach=true`。

#### DOCK_ABORT 与重新入泊

```text
needs_reapproach=true 时：
  1. 发布零速
  2. /dock/status 提示 REAPPROACH_REQUIRED
  3. 操作员 cancel → IDLE
  4. activate controller_server
  5. mission 重新导航到预泊点
  6. 再 /dock/start
```

---

## 6. 控制律汇总（base_link）

### 6.1 预处理

```python
x, y, yaw = transform_camera_to_base(raw_pose)
x, y, yaw = apply_invert(...)
x, y, yaw = ema_filter(...)
```

### 6.2 伪代码

```python
if state == DOCK_PRE_ALIGN:
    v, w = 0.0, clamp(kyaw_pre * yaw, ±max_yaw_rate)

elif state == DOCK_OUTSIDE_CENTERING:
    v = -outside_speed if prefer_outside_reverse else signed_outside_v(y, yaw)
    w = clamp(ky_out * y + kyaw_out * yaw, ±max_yaw_rate)

elif state == DOCK_GATE_CHECK:
    v, w = 0.0, 0.0   # 或保持 OUTSIDE 末速度；推荐刹停再判
    if gate_ok:
        state = DOCK_BACK_IN

elif state == DOCK_BACK_IN:
    if abs(y) > back_in_y_limit or abs(yaw) > back_in_yaw_limit:
        → ABORT (needs_reapproach)
    v = -clamp(kx * abs(x), min_back_in_speed, back_in_speed)
    w = clamp(ky_back * y + kyaw_back * yaw, ±back_in_max_yaw_rate)
```

### 6.3 Tag 超时

| 阶段 | 行为 |
|------|------|
| WAIT / PRE_ALIGN / OUTSIDE / GATE | `tag_loss_grace_sec` → hold；≥ `tag_timeout` → ABORT |
| **BACK_IN** | ≥ `tag_loss_in_corridor_sec`（~0.15s）→ **立即 ABORT + reapproach** |

### 6.4 到位（DOCK_STOP）

连续 `settle_cycles` 帧：

```text
|x_base| < stop_x_threshold
|y_base| < stop_y_threshold
|yaw_base| < yaw_tolerance
```

---

## 7. 默认参数（仿真 yaml 摘要）

```yaml
docking_controller:
  ros__parameters:
    dock_pose_topic: "/apriltag_node/dock_pose"
    cmd_vel_topic: "/cmd_vel_nav"
    start_topic: "/dock/start"
    cancel_topic: "/dock/cancel"
    status_topic: "/dock/status"
    mission_state_topic: "/mission_bridge/state"

    camera_frame: "camera_left_link"
    robot_frame: "base_link"
    tf_timeout_sec: 0.1
    allow_camera_frame_fallback: false

    control_rate: 20.0
    pose_filter_alpha: 0.35

    tag_loss_grace_sec: 0.15
    tag_timeout: 0.5
    tag_loss_in_corridor_sec: 0.15
    max_docking_duration_sec: 120.0
    outside_centering_timeout_sec: 60.0

    # PRE_ALIGN
    prealign_yaw_limit: 0.25
    kyaw_pre: 0.9

    # OUTSIDE_CENTERING（码头外弧线）
    outside_speed: 0.10
    prefer_outside_reverse: true
    ky_out: 0.40
    kyaw_out: 0.85

    # GATE（入通道门槛，必须严于 BACK_IN 微调限）
    corridor_enter_y_limit: 0.35
    corridor_enter_yaw_limit: 0.15

    # BACK_IN（通道内）
    back_in_speed: 0.15
    min_back_in_speed: 0.05
    kx: 0.25
    ky_back: 0.20          # 小于 outside，仅微调
    kyaw_back: 0.40
    back_in_max_yaw_rate: 0.18
    back_in_y_limit: 0.50
    back_in_yaw_limit: 0.22

    max_reverse_speed: 0.35
    max_yaw_rate: 0.35
    max_linear_accel: 0.12
    max_angular_accel: 0.35

    stop_x_threshold: 0.40
    stop_y_threshold: 0.25
    yaw_tolerance: 0.12
    settle_cycles: 5

    invert_x: false
    invert_y: false
    invert_yaw: false

    require_mission_idle: true
    allow_unknown_mission_state: false   # sim yaml 可设 true
    wait_tag_timeout_sec: 10.0
    gate_ready_cycles: 5
    abort_on_mission_emergency: true
    publish_zero_when_idle: true
```

实船：按 **船宽、通道宽** 收紧 `corridor_enter_*`、`back_in_*_limit`。

---

## 8. 预泊点（mission 侧，非本节点参数）

```text
· 码头正前方 3~5 m（pre_dock_standoff_m，写入 waypoints / 地图标定）
· 船头朝外、船尾朝码头
· yaw 与码头中心线一致
· Nav2 到达 → mission IDLE → 再 /dock/start
```

ABORT 且 `needs_reapproach=true` 时，**必须**重新执行该航点，不可在通道内硬倒。

---

## 9. Nav2 交接

```bash
ros2 lifecycle set /controller_server deactivate
ros2 launch usv_docking docking_controller.launch.py profile:=sim
ros2 topic pub /dock/start std_msgs/msg/Bool "{data: true}" --once
# 结束或 ABORT 后
ros2 lifecycle set /controller_server activate
```

---

## 10. 安全逻辑

1. 差速船：**不在通道内用「v=0 修 y」**（做不到）。  
2. 横向纠偏 **仅** 在 `OUTSIDE_CENTERING`。  
3. `GATE_CHECK` 不过 → 回 OUTSIDE，不进 BACK_IN。  
4. BACK_IN 超限 / Tag 丢 → ABORT + `needs_reapproach`。  
5. TF 失败 → 不控船。  
6. EMERGENCY / cancel / max_duration → ABORT。  
7. 全局限幅 + 斜率限制。

---

## 11. 测试计划

| # | 项 | 通过标准 |
|---|-----|----------|
| 1 | 大 yaw @ PRE_ALIGN | v=0，仅 ω |
| 2 | 大 y @ OUTSIDE | v≠0 弧线，y 减小 |
| 3 | v=0 时大 y | **y 不变**（验证无侧移） |
| 4 | GATE 未过 | 不进 BACK_IN |
| 5 | GATE 过后 BACK_IN | 直倒为主，|ω| ≤ back_in_max_yaw_rate |
| 6 | BACK_IN 注入大 y | ABORT + needs_reapproach |
| 7 | BACK_IN Tag 丢 | 立即 ABORT |
| 8 | 无 TF | 不输出非零速度 |
| 9 | 完整流程 | 预泊→…→STOP |
| 10 | ABORT 后 | 文档流程回预泊点可重来 |
| 11 | 无 Tag @ WAIT_TAG | 静止 `wait_tag_stationary_sec` 后 → **SEARCH_SPIN** |
| 11b | SEARCH_SPIN 转满 2π 仍无 Tag | ABORT `TAG_SEARCH_NO_TAG` + needs_reapproach |
| 12 | Tag 抖动 @ GATE | 须连续 `gate_ready_cycles` 帧才进 BACK_IN |
| 13 | 仿真/Gazebo 断线 | `ODOM_LOST` / `DOCK_POSE_STREAM_LOST` → 零速 ABORT |
| 14 | TAG_SEARCH 失败且 retries 未满 | 自动回 `WAIT_TAG` 再搜（opennav resetApproach） |

---

## 12. 借鉴 opennav_docking 的工程模式（v2.3，已实现）

| opennav | usv_docking | 说明 |
|---------|-------------|------|
| Action `phase` 反馈 | `/dock/status.phase` | `INITIAL_PERCEPTION` / `SEARCH_SPIN` / `CONTROLLING` / `DOCKED` / `FAILED` |
| `error_code` | `/dock/status.error_code` | 见 `usv_docking/dock_feedback.py` |
| `num_retries` + `resetApproach` | `max_retries` + `auto_retry_recoverable` | 搜 Tag 失败在预泊点自动重试 |
| `doInitialPerception` | `tag_acquire_cycles` | 连续 N 帧 Tag 才进控制 |
| `publishZeroVelocity` | 取消 / ABORT / 传感器看门狗 / 节点退出 | |
| `backward_projection` | `back_in_backward_projection_m` | 倒船深度目标后移 |
| `DockDatabase` | `config/dock_bays_sim.yaml` + Nav2 预泊航点 | 元数据参考，staging 由 mission 给定 |
| 螺旋控制律 | **未采用** | 保留 corridor-gated 状态机 |

---

## 13. 实施 Phase 1

- [x] `pose_transform.py` + `pose_filter.py`
- [x] 状态机：PRE_ALIGN / OUTSIDE_CENTERING / GATE_CHECK / BACK_IN
- [x] `needs_reapproach` 与 status JSON
- [x] sim yaml + launch + handoff 脚本
- [x] 预泊点写入联调说明（见 [`项目运行与联调.md`](./项目运行与联调.md) §归港入泊）
- [x] 搜 Tag 自转 `SEARCH_SPIN` + 里程计累计转角
- [x] 传感器看门狗 + opennav 风格 status（phase / error_code / retries）
- [x] 参数 yaml 注释 + `dock_bays_sim.yaml`

---

## 14. 实现约束（Cursor）

1. 包路径：`src/usv_docking/` only。  
2. **差速 kinematics**：通道外弧线修 y；通道内微调；超限 ABORT 回预泊。  
3. **TF 到 base_link** 后再控制。  
4. 状态名 `DOCK_*`，与 mission 分离。  
5. `/cmd_vel_nav` + deactivate Nav2 controller。  
6. 不改 apriltag / Nav2 源码。

---

## 15. 相关文档

| 文档 | 内容 |
|------|------|
| [README.md](../src/usv_docking/README.md) | **包 README**：编译、话题、联调命令 |
| [`AprilTag船坞定位接口.md`](../src/apriltag_localization/docs/AprilTag船坞定位接口.md) | dock_pose |
| [`仿真码头与AprilTag配置.md`](./仿真码头与AprilTag配置.md) | 仿真 dock / Tag 定稿 |
| [nav_task_interface.md](./nav_task_interface.md) | 预泊 mission |
| [项目运行与联调.md](./项目运行与联调.md) | 仿真启动 |
| [导航接口与参数速查.md](./导航接口与参数速查.md) | 归港话题速查 |
| [dock_mission README](../src/dock_mission/README.md) | Phase 2 编排包 |

---

## 16. Phase 2 — 归港全栈架构（Mission + Nav + Docking）

> **定位**：`usv_docking` 职责已足够（最后 0~2 m Tag 闭环 + 充电确认）。Phase 2 解决 **Nav 到点 ≠ 可泊**、**U 型侧偏**、**cmd_vel 控制权冲突**。

### 16.1 工业终版分层

```text
               ┌──────────────────┐
               │  dock_mission    │  Mission FSM、重试、上报、BACKOFF
               └────────┬─────────┘
                        │
          ┌─────────────▼─────────────┐
          │ Nav2 → staging 航点        │
          └─────────────┬─────────────┘
                        │ TASK_COMPLETED（仅触发验收）
          ┌─────────────▼─────────────┐
          │ Dock Entry Validator     │  dock_enu Gate
          └──────┬──────────────┬─────┘
                 NO             YES → usv_docking → speed_arbitrator
```

| 模块 | 包 | 职责 |
|------|-----|------|
| Mission FSM | `dock_mission` | 编排、双重 retry、handoff |
| Entry Validator | `dock_mission` | dock_enu 验收、Tag/map 交叉验证 |
| Speed Arbitrator | `dock_mission` | cmd_vel 互斥、零速 watchdog |
| 精靠泊 | `usv_docking` | PRE_ALIGN…BACK_IN…WAIT_CHARGE→STOP |

### 16.2 dock_enu（权威 Entry 帧）

```text
GNSS / RTK / map ──► T_map_dock ──► (ex, ey, eψ)
AprilTag ──► 与 map 交叉验证（冲突则拒接）
```

配置：`src/dock_mission/config/dock_database.yaml`。

### 16.3 Mission FSM

```text
DOCK_ARMED → NAV_TO_STAGING → WAIT_NAV → SETTLE(2~3s)
  → ENTRY_VALIDATE
       PROCEED → DOCK_HANDOFF → MONITOR_DOCK → SUCCEEDED
       REPLAN_STAGING → staging_retry++ (max 3)
       BACKOFF → 退出 maneuver → NAV_TO_STAGING
  → DOCK_FAILED
```

**Nav2 COMPLETED ≠ 可泊**；永不直接 `/dock/start`。

### 16.4 Entry Validator

- Service：`/dock/validate_entry`
- Topic：`/dock/entry_status`（2 Hz）

| dock_enu 条件 | 动作 |
|---------------|------|
| `x_min < ex < 0`，`\|ey\|≤y_max`，`\|eψ\|≤yaw_max` | PROCEED |
| `ex ≥ 0` 或过深 | BACKOFF |
| 横/航向超限 | REPLAN_STAGING |
| RTK 非 FIX | REJECT |
| Tag vs map 冲突 | FRAME_MISMATCH |

### 16.5 双重 retry

| 计数器 | 上限 | 触发 |
|--------|------|------|
| `staging_retry` | 3 | Nav/Entry/needs_reapproach |
| `dock_retry` | 2 | usv_docking `max_retries` |

### 16.6 Speed Arbitrator

| 模式 | Nav2 | docking | cmd_vel |
|------|------|---------|---------|
| NAVIGATION | active | off | Nav2 |
| STAGING_VERIFY | off | off | 零速 |
| DOCKING | off | on | docking |
| FAILED | off | cancel | 零速 |

切换前零速 ≥300 ms；watchdog 1 s → 零速。

### 16.7 成功条件栈（实船）

L1 零速稳定 → L2 pose settled（仿真）→ L3 charging 稳定 3~5 s → L4 mission 上报。

### 16.8 Nav2 容差

常规 1 m；归港 staging 0.5~0.8 m；**真正 Gate = Entry Validator**。

### 16.9 动态 Staging Replan

侧偏时将 staging 投影回通道轴（`staging_planner.py`）：`x=x_entry-standoff, y=0, yaw=π`。

### 16.10 Phase 2 清单

- [x] dock_enu + dock_database（`dock_mission/config/dock_database.yaml`）
- [x] entry_validator + `/dock/validate_entry` + `/dock/entry_status`
- [x] dock_mission_node FSM（`/dock/home`、Nav staging、handoff）
- [x] speed_arbitrator（骨架 + launch）
- [x] staging_planner BACKOFF/REPLAN
- [x] 仿真 map pose 预泊（`use_gnss_staging:=false`）
- [x] Nav2 双 GoalChecker（`docking_goal_checker` in `nav2_params.yaml`）
- [x] pytest 44 项（`src/dock_mission/test/`）
- [ ] 实船 `gnss_staging` RTK 标定 + `profile:=real` 联调
- [ ] speed_arbitrator 接入 Nav2 cmd_vel remap
- [ ] Entry Validator 从 yaml 加载走廊（当前部分硬编码）
- [ ] task_event：`DOCK_STAGING_FAILED` / `DOCK_MISSION_FAILED` 上报 GCS

### 16.11 Git 备份（不删工作区）

`git stash push -u` 会**撤掉工作区文件**，仅适合「临时切分支」；**只备份请用**：

```bash
REF=$(git stash create -m "WIP 描述 $(date +%F)")
git stash store -m "WIP 描述 $(date +%F)" "$REF"
```

恢复：`git stash apply stash@{N}`（工作区已有文件时优先用 apply，避免 pop 冲突）

**暂存说明**：使用 `stash create` + `stash store` **不会清空工作区**；勿用 `git stash push -u` 做仅备份（会撤掉未跟踪文件）。

---

## 17. Phase 2 实现约束

1. `usv_docking` 不扩展为 Anywhere→Dock。  
2. Entry 判定只认 **dock_enu**。  
3. handoff：零速 → deactivate Nav2 → `/dock/start` → Arbitrator→DOCKING。  
4. 归港时 mission 状态用专用 `DOCKING`（与 `require_mission_idle` 协调）。  
5. BT 包装放 Phase 2 后期。
