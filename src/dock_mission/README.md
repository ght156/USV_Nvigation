# dock_mission

**归港 Phase 2 — 任务编排包**：负责「一键归港」从 Nav2 预泊到 `usv_docking` 交接的全流程，**不负责**最后几米的 Tag 倒船控制。

> 精靠泊控制器见 [`../usv_docking/README.md`](../usv_docking/README.md)。  
> 全栈联调见 [`../../docs/项目运行与联调.md`](../../docs/项目运行与联调.md)。

---

## 两个 dock 包分别干什么？

| | **dock_mission**（本包） | **usv_docking**（精靠泊包） |
|---|--------------------------|-----------------------------|
| **职责** | 任务编排：预泊 Nav、入口验收、handoff、重试 | 最后几米：搜 Tag、对准、倒船入坞、充电确认 |
| **距离尺度** | 几十米 → 预泊点（约 4 m 外） | 预泊点 → 坞内（0~2 m） |
| **主要传感器** | RTK/里程计 + map/GNSS | AprilTag + 里程计（+ 无线充电） |
| **控制输出** | 调 Nav2（`send_waypoints`）+ 仲裁 cmd_vel | 直接发 `/cmd_vel_nav`（或经仲裁器） |
| **你什么时候改它** | 预泊点坐标、Nav 容差、入口走廊、一键归港接口 | Tag 对准、倒船速度、通道门槛、充电判定 |
| **典型启动** | `ros2 launch dock_mission dock_mission.launch.py` | `ros2 launch usv_docking docking_controller.launch.py` |

**一句话**：`dock_mission` = 「把船 Nav 到预泊点并检查能不能泊」；`usv_docking` = 「进了预泊点之后怎么倒进去」。

### 协作流程

```text
GCS / 上层
    │  /dock/home=true
    ▼
┌─────────────────────────────────────────────────────────────┐
│  dock_mission_node（本包）                                    │
│  1. 切换 Nav2 → docking_goal_checker（更紧的到点容差）        │
│  2. mission_bridge.send_waypoints(预泊点)                    │
│  3. SETTLE 等待船停稳                                         │
│  4. entry_validator 检查 dock_enu 走廊                        │
│  5. 发布 /dock/start → 交给 usv_docking                       │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│  usv_docking                                                  │
│  WAIT_TAG → PRE_ALIGN → BACK_IN → DOCK_STOP / 充电确认       │
└─────────────────────────────────────────────────────────────┘
```

**也可以不用本包**：手动 Nav 到预泊点后，deactivate Nav2，直接 `bash src/usv_docking/scripts/docking_handoff.sh start`。本包只是把这几步自动化并加重试。

---

## 本包包含哪些节点？

| 节点 | 作用 | 要不要单独调 |
|------|------|--------------|
| `dock_mission_node` | 归港 FSM、Nav2 GoalChecker 切换、发航点、handoff | **主要调这个** |
| `dock_entry_validator` | 预泊后检查船是否在 dock_enu 入口走廊内 | 调走廊几何 / Tag 要求 |
| `speed_arbitrator` | Nav 与 Dock 的 `cmd_vel` 互斥（可选接入） | 接 Nav2 remap 时才需要 |

一个 launch 会同时起这三个节点：

```bash
ros2 launch dock_mission dock_mission.launch.py profile:=sim use_sim_time:=true
```

---

## 快速上手（仿真）

### 前置条件

以下进程**必须先跑起来**：

| 组件 | 作用 |
|------|------|
| Gazebo + 船 | 仿真环境 |
| Nav2（`workspace_nav`） | 预泊导航 |
| `mission_bridge` | 接收 `send_waypoints` |
| `apriltag_localization` | 供 `usv_docking` 用 |
| `usv_docking` | 精靠泊（见另一包 README） |

### 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/wuxihik_navigation
colcon build --packages-select dock_mission usv_docking workspace_nav
source install/setup.bash
```

### 启动顺序（示例）

```bash
# 1. 仿真 + Nav2 + mission_bridge + apriltag（见 docs/项目运行与联调.md）

# 2. 精靠泊控制器
ros2 launch usv_docking docking_controller.launch.py profile:=sim use_sim_time:=true

# 3. 归港编排（本包）
ros2 launch dock_mission dock_mission.launch.py profile:=sim use_sim_time:=true
```

### 一键归港

```bash
# 方式 A：GCS / 上层话题（推荐）
ros2 topic pub --once /dock/home std_msgs/msg/Bool "{data: true}"

# 方式 B：Service
ros2 service call /dock/mission/start std_srvs/srv/Trigger {}

# 取消
ros2 service call /dock/mission/cancel std_srvs/srv/Trigger {}
```

### 看状态

```bash
ros2 topic echo /dock/mission_status   # JSON：state、staging_retry、nav2_profile
ros2 topic echo /dock/status           # usv_docking 精靠泊状态
ros2 topic echo /dock/entry_status     # 入口验收
```

---

## 快速上手（实船）

与仿真相同，但 launch 时改 profile：

```bash
ros2 launch dock_mission dock_mission.launch.py profile:=real use_sim_time:=false
ros2 launch usv_docking docking_controller.launch.py profile:=real use_sim_time:=false
```

**实船必做两件事**：

1. 用 RTK 在预泊点实测，更新 [`config/dock_database.yaml`](config/dock_database.yaml) 里的 `gnss_staging`
2. 确认 `map_hk.yaml` 的 `ref_gnss_10` 与现场 datum 一致

---

## 配置文件地图（改哪个文件？）

参数分散在多个文件，按**你要改什么**找文件，不要在一个 yaml 里乱搜。

```text
dock_mission/
├── config/
│   ├── dock_mission.yaml          ← 三个节点的 ROS 参数（主配置）
│   ├── dock_mission_sim.yaml      ← 仿真 overlay：use_gnss_staging=false
│   ├── dock_mission_real.yaml     ← 实船 overlay：use_gnss_staging=true
│   └── dock_database.yaml         ← 泊位几何 + 预泊点坐标（GNSS/map）★ 最常改
└── launch/dock_mission.launch.py  ← profile:=sim|real 选择 overlay

workspace_nav/config/
└── nav2_params.yaml               ← general / docking 两个 GoalChecker ★ Nav 容差

usv_docking/config/
├── docking_controller_sim.yaml    ← 精靠泊全部参数（另一包）
└── dock_bays_sim.yaml             ← 泊位元数据参考（Tag ID 等）
```

### 加载规则

`dock_mission.yaml`（基础） + `dock_mission_{profile}.yaml`（覆盖） → 合并后给各节点。

---

## 调参指南（按场景）

### 场景 1：预泊点位置不对（Nav 到的点偏了）

**改这里** → [`config/dock_database.yaml`](config/dock_database.yaml)

| 字段 | 仿真 | 实船 |
|------|------|------|
| `map_staging.x/y/yaw` | 直接 map 坐标 | 一般不改 |
| `gnss_staging.latitude/longitude/yaw_deg` | 占位 | **RTK 标定后填写** |
| `standoff_m` | 预泊离坞中心距离参考（4 m） | 同上 |

仿真默认：船 spawn `(0,0)` 朝 +x，预泊 `(3.5, 0, yaw=0)`。

本包节点参数：

| 参数 | 文件 | 说明 |
|------|------|------|
| `use_gnss_staging` | `dock_mission_sim/real.yaml` | `false`=用 map，`true`=用 GNSS |
| `bay_id` | `dock_mission.yaml` | 对应 `dock_database.yaml` 里的 bay 名 |

---

### 场景 2：Nav 说到了但 Entry 验收失败

Nav2 **COMPLETED ≠ 可泊**。Entry Validator 用 dock_enu 走廊再判一次。

**改这里** → [`config/dock_database.yaml`](config/dock_database.yaml) 的 `entry_corridor`：

| 字段 | 默认 | 含义 |
|------|------|------|
| `x_min` | -6.0 | 入口外最远（ex 更负 = 更远） |
| `x_max` | 0.0 | 入口线（ex≥0 判为已过线 → BACKOFF） |
| `y_max` | 1.0 | 横向 \|ey\| 上限 (m) |
| `yaw_max` | 0.15 | 航向误差上限 (rad) |

**或改 Nav 到点容差**（让 Nav 停得更准）→ `workspace_nav/config/nav2_params.yaml`：

```yaml
docking_goal_checker:
  xy_goal_tolerance: 0.6    # 预泊 xy 容差 (m)
  yaw_goal_tolerance: 0.15  # 预泊 yaw 容差 (rad)
```

本包会在归港时切换到这个 checker；也可在 `dock_mission.yaml` 里改 fallback 容差：

```yaml
docking_xy_goal_tolerance: 0.6
docking_yaw_goal_tolerance: 0.15
```

**SETTLE 时间**（Nav 完成后等船停稳）：

```yaml
settle_sec: 2.5   # dock_mission.yaml → dock_mission_node
```

---

### 场景 3：预泊 Nav 失败 / 需要重试

**改这里** → `dock_mission.yaml` → `dock_mission_node`：

| 参数 | 默认 | 说明 |
|------|------|------|
| `staging_retry_max` | 3 | Nav 预泊最大重试次数 |
| `dock_mission_id` | `dock_staging` | 须与 mission_bridge 任务 ID 一致 |
| `send_waypoints_service` | `/mission_bridge/send_waypoints` | 航点服务名 |

失败时本包会：cancel dock → 恢复 cruise GoalChecker → 再发 Nav。

---

### 场景 4：GCS 一键归港接口

**改这里** → `dock_mission.yaml`：

| 参数 | 默认 | 说明 |
|------|------|------|
| `dock_home_topic` | `/dock/home` | GCS 发 `Bool true` 的话题 |

Service 固定为 `/dock/mission/start`、`/dock/mission/cancel`，一般不用改。

---

### 场景 5：Entry Validator 要求看到 Tag 才 handoff

默认 **不要求** Tag（`require_tag_for_proceed: false`），只靠 RTK/odom + 走廊。

实船若要加强：

```yaml
# dock_mission.yaml → dock_entry_validator
require_tag_for_proceed: true
tag_mismatch_threshold_m: 0.8   # Tag 与 map 偏差过大则拒绝
```

---

### 场景 6：Nav 与 Dock 抢 cmd_vel

若 Nav2 输出 `/cmd_vel_nav_raw`，Dock 输出 `/cmd_vel_dock`，需要仲裁：

```yaml
# dock_mission.yaml → speed_arbitrator
cmd_vel_out_topic: "/cmd_vel_nav"      # 最终给 converter 的话题
cmd_vel_nav_topic: "/cmd_vel_nav_raw"  # Nav2 remap 到这里
cmd_vel_dock_topic: "/cmd_vel_dock"    # usv_docking 改发这里（需改 usv_docking 参数）
```

并在 Nav2 launch 里把 controller 输出 remap 到 `cmd_vel_nav_raw`。

**authority 规则**：`NAVIGATION` 用 Nav；`DOCKING` 用 Dock；`SETTLE/FAILED` 零速。

---

### 场景 7：精靠泊阶段（倒船、Tag、充电）

**不在本包调**，去 [`../usv_docking/README.md`](../usv_docking/README.md) 的调参章节。

本包只负责在 Entry 通过后发 `/dock/start`；之后全是 `usv_docking`。

---

## 话题 / 服务接口

### 输入（你或 GCS 发的）

| 名称 | 类型 | 说明 |
|------|------|------|
| `/dock/home` | `Bool` | `data=true` 开始归港（参数 `dock_home_topic` 可改） |
| `/dock/mission/start` | `Trigger` | 同上 |
| `/dock/mission/cancel` | `Trigger` | 取消归港 |
| `/task_event` | `String` | mission_bridge 任务事件（内部订阅） |
| `/dock/status` | `String` | usv_docking 状态（内部订阅，判成功/需重 Nav） |

### 输出（本包发的）

| 名称 | 类型 | 说明 |
|------|------|------|
| `/dock/mission_status` | `String` JSON | 编排状态 |
| `/dock/start` | `Bool` | handoff 给 usv_docking |
| `/dock/cancel` | `Empty` | 取消精靠泊 |
| `/dock/speed_authority` | `String` | 速度仲裁权（NAVIGATION / DOCKING / …） |
| `goal_checker_selector` | `String` | Nav2 GoalChecker 切换 |

### 服务

| 名称 | 类型 | 说明 |
|------|------|------|
| `/dock/validate_entry` | `Trigger` | entry_validator 提供，FSM 内部调用 |

---

## 状态机（dock_mission_node）

```text
IDLE
  │ /dock/home 或 /dock/mission/start
  ▼
ARMED → NAV_TO_STAGING（apply_docking + send_waypoints）
  │ TASK_COMPLETED
  ▼
SETTLE（settle_sec 秒）
  ▼
ENTRY_VALIDATE（/dock/validate_entry）
  │ 通过
  ▼
DOCK_HANDOFF（/dock/start=true，authority=DOCKING）
  ▼
MONITOR_DOCK（等 /dock/status success 或 needs_reapproach）
  ├─ success → SUCCEEDED，恢复 cruise GoalChecker
  └─ needs_reapproach → 重 Nav（staging_retry_max 次内）
```

`/dock/mission_status` JSON 示例：

```json
{
  "state": "NAV_TO_STAGING",
  "bay_id": "bay2",
  "staging_retry": 0,
  "staging_retry_max": 3,
  "nav2_profile": "docking",
  "use_gnss_staging": false
}
```

---

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 发 `/dock/home` 没反应 | `dock_mission_node` 未启动 | launch 本包 |
| `send_waypoints service unavailable` | mission_bridge 未跑 | 先起 mission_bridge |
| Nav 完成但 entry 失败 | 预泊点或走廊参数不对 | 调 `dock_database.yaml` entry_corridor / map_staging |
| `/dock/start` 后 usv_docking 不动 | Nav2 未 deactivate | 本包 handoff 前需 usv_docking 自己处理；或检查 mission 互锁 |
| 实船预泊点飘 | GNSS 未标定 | 更新 `gnss_staging`，确认 `use_gnss_staging:=true` |
| 精靠泊 Tag 搜不到 | 预泊姿态 / 相机 | 调 usv_docking，不是本包 |

---

## 测试

```bash
cd src/dock_mission
./test/run_tests.sh          # 44 项 pytest
```

详见 [`test/README.md`](test/README.md)。

---

## 相关文档

| 文档 | 内容 |
|------|------|
| [`../usv_docking/README.md`](../usv_docking/README.md) | 精靠泊：Tag、倒船、充电 |
| [`../../docs/usv_docking任务规划.md`](../../docs/usv_docking任务规划.md) | 架构与 Phase 2 设计 |
| [`../../docs/项目运行与联调.md`](../../docs/项目运行与联调.md) | 全栈 7 终端联调 |

---

## 上层对接（第三方 / GCS）

> **主文档**：[`../../docs/nav_task_interface.md`](../../docs/nav_task_interface.md) **§13**  
> **字段说明**：[`../../docs/msg/dock_task_status.msg`](../../docs/msg/dock_task_status.msg)、[`dock_task_event.msg`](../../docs/msg/dock_task_event.msg)

| 方向 | 接口 | 说明 |
|------|------|------|
| 上层 → 船 | `/dock_task/command` | `DockTaskCommand`：一键归港 / 仅精靠泊 / 出泊 / 取消 |
| GCS → 船 | `/gcs_dock/command` | JSON `action`；兼容 `/dock/home` |
| 船 → 上层 | `/dock_task/status` | 约 5Hz：`run_state`、`retry_count`、`needs_manual_takeover`、`dock_active` |
| 船 → 上层 | `/dock_task/event` | `DOCK_SUCCEEDED`、`DOCK_FAILED`、`MANUAL_TAKEOVER_REQUESTED` 等 |

- **商用单坞**，上层 JSON **不含** `bay_id`。
- **`dock_active==false`** 表示归港会话已关闭（成功后自动释放 usv_docking）。
- **勿将 `/dock/status`** 作为上层接口（船内 usv_docking 联调全量字段）。

```bash
ros2 service call /dock_task/command m_common/srv/DockTaskCommand \
  "{command: 1, mission_id: 'm001', command_id: '', require_camera: false}"
ros2 topic echo /dock_task/status
ros2 topic echo /dock_task/event
```

