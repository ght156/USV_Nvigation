# dock_mission

**归港 Phase 2 — 任务编排包**：负责「一键归港」从 Nav2 预泊到 `usv_docking` 交接的全流程，**不负责**最后几米的 Tag 倒船控制。

> 精靠泊控制器见 [`../usv_docking/README.md`](../usv_docking/README.md)。

---

## 实船仓（USV_NAV）迁移说明（2026-09-02）

本包从仿真仓 `wuxihik_navigation` 迁移到实船仓，与仿真版差异如下：

| 项 | 仿真仓 | 本仓（实船） |
|----|--------|--------------|
| `SendWaypoints` 航点类型 | `geometry_msgs/PoseStamped[]`（map 系） | `m_common/MissionWaypoint[]`（**WGS84 经纬度+航向**，mission_bridge 内部转 map） |
| `use_gnss_staging` 默认 | false（map_staging） | **true**（泊位库 `gnss_staging` 直发 WGS84） |
| `use_sim_time` 默认 | true | **false** |
| launch `profile` 默认 | sim | **real**（`dock_mission_real.yaml` overlay） |
| `speed_arbitrator` | 默认启动 | **默认不启动**（`enable_speed_arbitrator:=true` 才起；实船 Nav2/docking 都直发 `/cmd_vel_nav`，互斥由交接时序 + 速度桥 1s 看门狗保证） |
| `dock_entry_validator` 里程计 | `/odometry/filtered`（代码默认/仿真） | `/mavros/gps_input/local`（`dock_mission_real.yaml` overlay） |

实船使用前**必须**：

1. 用 RTK 标定泊位，覆盖 `config/dock_database.yaml`（现有 `bay2` 是仿真值）；
2. 确认 Nav2 预泊容差：本包默认向 `goal_checker_selector` topic 发切换并兜底改
   `general_goal_checker` 参数——实船 `nav2_params_real_mavros.yaml` 若未定义这两个
   checker，切换会静默无效（仅日志告警），需自行添加或调 `controller_server` 默认容差；
3. 视觉精定位依赖运行时 TF `base_link→dock_frame`（由 AprilTag 定位节点广播），
   本仓暂未包含 AprilTag 节点，需另行部署；
4. `dock_entry_validator` 的 bay 几何仍是硬编码占位值（`entry_validator_node.py` TODO），
   且把 odom 系位姿当 map 系用——实船标定时需改从 `dock_database.yaml` 加载。


## 两个 dock 包分别干什么？

| | **dock_mission**（本包） | **usv_docking**（精靠泊包） |
|---|--------------------------|-----------------------------|
| **职责** | 任务编排：预泊 Nav、入口验收、handoff、重试 | 最后几米：搜 Tag、对准、倒船入坞、充电确认 |
| **距离尺度** | 几十米 → 预泊点（约 4 m 外） | 预泊点 → 坞内（0~2 m） |
| **主要传感器** | RTK/里程计 + map/GNSS | AprilTag + 里程计（+ 无线充电） |
| **控制输出** | 调 Nav2（`send_waypoints`）+ 仲裁 cmd_vel | 直接发 `/cmd_vel_nav`（或经仲裁器） |
| **你什么时候改它** | 预泊点坐标、Nav 容差、入口走廊、一键归港接口 | Tag 对准、倒船速度、通道门槛、充电判定 |
| **典型启动** | `ros2 launch dock_mission dock_mission.launch.py` | `ros2 launch usv_docking docking.launch.py` |

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
│  [WAIT_DOCK_OPEN*] → ACQUIRE_TAG → APPROACH_ENTRY →          │
│  ALIGN_ENTRY → BACK_IN → FINAL_DOCK → DOCKED                 │
│  * 船坞夹爪交互（dock_claw_enabled=true 才启用，默认关闭）：    │
│    归港先请求船坞打开夹爪，夹爪抓住（WaitIO）才判 DOCKED；     │
│    出泊先请求松开夹爪（ControlDO）                             │
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

一个 launch 会同时起前两个节点（`speed_arbitrator` 需 `enable_speed_arbitrator:=true`）：

```bash
ros2 launch dock_mission dock_mission.launch.py   # 实船默认 profile:=real, use_sim_time:=false
```

---

## 快速上手（实船）

### 前置条件

以下进程**必须先跑起来**：

| 组件 | 作用 |
|------|------|
| 实船 bringup + Nav2（`workspace_nav`） | 预泊导航（见仓根 docs/ 与 scripts/start_nx_stack.sh） |
| `mission_bridge` | 接收 `send_waypoints` |
| AprilTag 定位节点 | 广播 `base_link→dock_frame` TF（本仓暂未包含，需另行部署） |
| `usv_docking` | 精靠泊（见另一包 README） |

### 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/USV_NAV
colcon build --merge-install --packages-select dock_mission usv_docking
source install/setup.bash
```

### 启动顺序（示例）

```bash
# 1. 实船 bringup + Nav2 + mission_bridge（见仓根 docs/ 与 scripts/start_nx_stack.sh）

# 2. 精靠泊控制器
ros2 launch usv_docking docking.launch.py

# 3. 归港编排（本包）
ros2 launch dock_mission dock_mission.launch.py
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

## 配置文件地图（改哪个文件？）

参数分散在多个文件，按**你要改什么**找文件，不要在一个 yaml 里乱搜。

```text
dock_mission/
├── config/
│   ├── dock_mission.yaml          ← 三个节点的 ROS 参数（主配置）
│   ├── dock_mission_real.yaml     ← 实船 overlay（默认 profile）
│   └── dock_database.yaml         ← 泊位几何 + 预泊点坐标（GNSS/map）★ 实船须 RTK 重标定
└── launch/dock_mission.launch.py  ← profile:=real 选择 overlay（默认实船）

workspace_nav/config/
└── nav2_params_real_mavros.yaml   ← GoalChecker 容差 ★ Nav 容差

usv_docking/config/
└── docking.yaml                   ← 精靠泊全部参数（另一包）
```

### 加载规则

`dock_mission.yaml`（基础） + `dock_mission_{profile}.yaml`（覆盖） → 合并后给各节点。

---

## 调参指南（按场景）

### 场景 1：预泊点位置不对（Nav 到的点偏了）

**改这里** → [`config/dock_database.yaml`](config/dock_database.yaml)

| 字段 | 说明 |
|------|------|
| `map_staging.x/y/yaw` | 直接 map 坐标 |
| `gnss_staging.latitude/longitude/yaw_deg` | GNSS 预泊（需 use_gnss_staging=true） |
| `standoff_m` | 预泊离坞中心距离参考（4 m） |

实船默认 `use_gnss_staging=true`：直接使用泊位库 `gnss_staging`（WGS84 经纬度+航向），须先用 RTK 实测标定。

本包节点参数：

| 参数 | 文件 | 说明 |
|------|------|------|
| `use_gnss_staging` | `dock_mission.yaml`（实船默认 true） | `true`=用泊位库 GNSS 预泊点（WGS84 直发） |
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

若要加强校验：

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
| 精靠泊 Tag 搜不到 | 预泊姿态 / 相机 | 调 usv_docking，不是本包 |

---

## 测试

```bash
cd src/dock_mission
./test/run_tests.sh          # 48 项 pytest
```

详见 [`test/README.md`](test/README.md)。

---

## 相关文档

| 文档 | 内容 |
|------|------|
| [`../usv_docking/README.md`](../usv_docking/README.md) | 精靠泊：Tag、倒船、充电 |
| [`../usv_docking/docs/架构与上层接口.md`](../usv_docking/docs/架构与上层接口.md) | 架构与接口契约 |
| [`../../../docs/导航与归港异常告警.md`](../../../docs/导航与归港异常告警.md) | 归港告警与特殊情况码 |

---

## 上层对接（第三方 / GCS）

| 方向 | 接口 | 说明 |
|------|------|------|
| GCS → 船 | `/gcs_dock/command` | JSON `action`：一键归港 / 仅精靠泊 / 出泊 / 取消；可选 `dock_lat`/`dock_lon`/`dock_yaw` 指定**预泊点**（WGS84 经纬度 + **度**航向，`dock_yaw` 超 `(−360,360)` 或 `65536.0` = 不指定朝向），下发后 Nav 到该点再精确归港，缺省用泊位库 `gnss_staging`；`dock_mission` 内部把度转弧度后发给 mission_bridge |
| 上层 → 船 | `/dock/home` | `Bool`：一键归港 |
| 上层 → 船 | `/dock/mission/start` | `Trigger` service：一键归港 |
| 船 → 上层 | `/dock_task/status` | 约 5Hz：`run_state`、`retry_count`、`needs_manual_takeover`、`dock_active` |
| 船 → 上层 | `/dock_task/event`（service） | `DOCK_SUCCEEDED`、`DOCK_FAILED`、`MANUAL_TAKEOVER_REQUESTED` 等，嵌软回 ACK（`m_common/srv/DockTaskEvent`） |
| 船 → 本地 | `/dock_task/event_log`（可选） | 同 JSON 观察流（日志/联调；参数 `dock_task_event_log_topic`，置空关闭） |

> 2026-09-01：`/dock_task/command` service 已移除（`DockTaskCommand.srv` 2026-07-28
> 从 m_common 删除），上层触发请改用上述 topic / Trigger。

- **商用单坞**，上层 JSON **不含** `bay_id`。
- **`dock_active==false`** 表示归港会话已关闭（成功后自动释放 usv_docking）。
- **勿将 `/dock/status`** 作为上层接口（船内 usv_docking 联调全量字段）。

```bash
ros2 topic pub --once /dock/home std_msgs/msg/Bool "{data: true}"
ros2 service call /dock/mission/start std_srvs/srv/Trigger "{}"
ros2 topic echo /dock_task/status
ros2 topic echo /dock_task/event_log
```
