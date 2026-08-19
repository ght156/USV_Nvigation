# decision ↔ 导航节点 接口对接说明

> 面向导航节点(nav)开发的对接清单。
> decision 只下发“去哪 / 跑哪条航线 / 停 / 急停”,由导航节点负责排序、局部避障与实际控船。
>
> **本文覆盖「交给导航节点」的任务:单点导航、返航、归港、多点航线任务(`mission_upload`/`mission_execute`)。**
> 多点航线由 decision **整包转发**给 nav,decision 不拆航点、不管算法,只等 nav 回任务进展。
> 手动/半自动/悬停/巡航由 sub_decision 承担,不在本文范围。
> 关联云端协议:`src/mqtt/src/mqtt_bridgev2/无人船上云接口文档_合并版V0.5.md`(§4.2.1 `set_mode`、§4.2.2 航线任务、附录 E.1 模式枚举)。

---

## 0. 职责边界:哪些任务交给导航节点

导航节点在 **GUIDED** 模式下,通过持续下发 setpoint(`/mavros/setpoint_velocity/cmd_vel`)做带**局部避障**的自主航行。以下云端任务全部委托给它:

| 云端下发 | 逻辑语义 | 导航节点动作 |
|---|---|---|
| `set_mode` mode=15 / `nav_to_waypoint` | 单点导航 | 去指定航点 |
| `set_mode` mode=11 | 返航 | 去 Home 位姿 |
| `set_mode` mode=12 | 智能返航 | **decision 归一为 11**,与返航同一套实现,只有一种返航 |
| `set_mode` mode=8 | 归港 | 去船坞位姿 |
| `mission_upload` / `mission_execute` | 多点航线任务 | **decision 整包转发整条航线**,nav 负责排序 / 避障 / 执行航点动作 / 回报进展 |

**明确不交给导航节点**(由 sub_decision → mavros,原生模式):

| 云端下发 | 归属 |
|---|---|
| `set_mode` mode=0/3/5/17、`manual_control` | sub_decision,原生模式 / 手动 |

> `set_mode` 是**分发器**:decision 按 `mode` 值路由。8/11/12/15 与航线任务(`mission_upload`/`mission_execute`)**拦截**交导航节点,**不透传给 mavros**;mode=0/3/5/17 与 `manual_control` 原样交 sub_decision。
> 多点航线由 decision **原样整包转发**(execution_id + speed + finish_action + 整条 waypoints,含航点 actions),不在 decision 侧拆点或编排。

---

## 1. mavros 单写者与所有权令牌

任一时刻 mavros 的**运动控制只能有一个所有者**。decision 持一个令牌,在 sub_decision 与导航节点间二选一:

- **令牌在导航节点**(正在跑单点/返航/归港):导航节点**独占** mavros 全部写入 —— setpoint、`set_mode`→GUIDED、**以及开锁/解锁(arming)**。decision 与 sub_decision **只读状态,不得向 mavros 下发任何控制**。
- **令牌在 sub_decision**:导航节点只读,不发 setpoint。

### 1.1 开锁/解锁权限(重要)

**凡是交给导航节点的模式,arm/disarm 权限全部归导航节点。** 即:

- 令牌在导航节点期间,sub_decision 的 arming 通道**关闭**;
- 导航节点在任务生命周期内**自管 arm/disarm**(任务开始自 arm,终止自 disarm);
- 此期间强制停船只走 `/nav/emergency_stop`(见 §5),这是导航节点持令牌期间唯一的强停通道。

### 1.2 令牌交接

```
接管:decision 收 8/11/12/15
  → 停 sub_decision 的 mavros 写与 arming
  → 令牌 = 导航节点 → 导航节点 arm + GUIDED + /nav/way_point(目标)
交回:导航节点回报 arrived / unreachable / stuck / 急停
  → 令牌 = decision → NavHold 兜底,等下一条指令
打断:云端 manual_control 或 set_mode=0(手动优先级最高)
  → decision 先让导航节点 hold/交回令牌,再把令牌给 sub_decision
```

---

## 2. OSD control_mode 上报飞控原始模式(不做逻辑覆盖)

**约定:`usv_osd.heartbeat` 上报船的原始真实状态,`control_mode` 直接来自飞控当前模式**(mqtt_bridge 用 `control_mode_from_mavros_mode(state->mode)` 映射)。返航/归港/单点/多点物理上都是 **GUIDED(15)**,OSD 就如实显示 GUIDED —— 这是给用户看的、最准确的船况;**不能用 decision 的"逻辑意图"去覆盖它**(否则船若中途掉进 HOLD/failsafe,OSD 还显示"返航"就是骗人)。

任务/返回的**逻辑意图**由别的通道表达,而非 OSD 的 control_mode:
- 多点航线:`mission_state.phase`(RUNNING/PAUSED/COMPLETED…,见 §4.1.2.5)。
- 返航/归港:如需在前端显示"正在返航/归港",应走独立字段/状态,不改 `control_mode`(待定,本期不做)。

---

## 3. 接口总览(共 12 个:8 service + 4 topic)

### A. 导航节点作为 service **server**(decision 调用)

**所有下发 service 都遵守「立即受理」语义**:`result` 仅表示受理(0/非0),真正到达/进展一律走 topic 异步回报。

| 话题名 | 类型 | 语义 | 对应云端 |
|---|---|---|---|
| `/nav/way_point` | `m_common/srv/NavGotoPoint` | 单点导航,收单点目标 | `set_mode=15` / `nav_to_waypoint` |
| `/nav/return_home` | `m_common/srv/NavReturnHome` | 返航,去 Home 位姿(由 decision 传入) | `set_mode=11`(12 归一到 11) |
| `/nav/return_dock` | `m_common/srv/NavReturnDock` | 归港(**船坞位姿由 nav 自存自读**,decision 只发触发) | `set_mode=8` |
| `/nav/mission_upload` | `m_common/srv/NavMissionUpload` | **仅上传航线**(装载不执行),整条 waypoints | `mission_upload` |
| `/nav/mission_execute` | `m_common/srv/NavMissionExecute` | **上传并立即执行航线**,整条 waypoints | `mission_execute` |
| `/nav/mission_control` | `m_common/srv/NavMissionControl` | 航线任务暂停/恢复/取消(`action`) | `mission_pause`/`resume`/`cancel` |
| `/nav/gcs_mission/cancel` | `m_common/srv/NavHold` | 停在当前位置(航段还在,可被下一个目标续上) | — |
| `/nav/emergency_stop` | `m_common/srv/NavEmergencyStop` | 立即急停,request 带 `reason` | — |

> 单点/返航/归港三个"去某处" service 用 `seq` 配对回报;航线任务两个 service 用 `execution_id` 配对回报(见 §6)。

### B. 导航节点作为 topic **publisher**(decision 订阅)

**两套回报按粒度分开**——单目标任务(单点/返航/归港)与任务级(多点航线)各一套:

| 话题名 | 类型 | 语义 | 用于 |
|---|---|---|---|
| `/nav/nav_status` | `m_common/msg/NavState` | **周期**:internal_state / current_seq / 当前目标经纬度 | 单点/返航/归港 |
| `/nav/task_event` | `m_common/msg/NavEvent` | **事件**:event_type / seq / detail | 单点/返航/归港 |
| `/nav/mission_status` | `m_common/msg/NavStatus` | **周期**:任务快照(task_state / current_waypoint / total / progress) | 多点航线任务 |
| `/nav/mission_event` | `m_common/msg/NavTaskEvent` | **事件**:TASK_STARTED / COMPLETED / FAILED / PAUSED … | 多点航线任务 |

> - 单点/返航/归港是**单目标**:用 `NavState`/`NavEvent`,靠 `seq` 配对(单点=云端航点;返航=Home;归港=船坞位姿)。
> - 多点航线是**任务级**:用 `NavStatus`/`NavTaskEvent`,靠 `task_id`(= `execution_id`)+ 航点序号回报。decision 把它们映射成云端 `mission_state`(V0.5 §4.1.2.5)。

---

## 4. 数据流

```
decision                         nav 节点
   │  /nav/way_point (lat/lon/alt/speed/seq)   ─────►  收下,立即回 result=0(受理)
   │  ◄───── result/msg(仅受理,不是到达)
   │                                                  开始导航 + 避障 + 控船
   │  ◄───── /nav/nav_status (周期: navigating ...)
   │  ◄───── /nav/task_event (arrived / unreachable / stuck, 带回同一个 seq)
   │
   │  /nav/gcs_mission/cancel  ──►  停在原地
   │  /nav/emergency_stop      ──►  立即停一切
```

decision 侧:发完 way_point 即返回不阻塞,进入“导航中”状态,每个 tick 查 `/nav/task_event`;
收到 `arrived` 判成功、`unreachable`/`stuck` 判失败,然后才允许下一个目的地。

**多点航线任务(整包转发)**:

```
decision                              nav 节点
   │  /nav/mission_execute (execution_id, speed, finish_action, waypoints[])  ─────►
   │      (mission_upload 则只装载不启动)                             收下,立即回 result=0(受理)
   │  ◄───── result/msg(仅受理,不是完成)
   │                                             nav 自己排序 + 逐点避障 + 执行航点 actions
   │  ◄───── /nav/mission_status (周期: RUNNING, current_waypoint=k/total, progress%)
   │  ◄───── /nav/mission_event  (TASK_STARTED → … → TASK_COMPLETED / TASK_FAILED, 带 task_id)
   │
   │  /nav/gcs_mission/cancel  ──►  暂停/停(配合云端 mission_pause)
   │  /nav/emergency_stop      ──►  立即停一切
```

decision 侧:发完 mission 即返回不阻塞;把 `/nav/mission_status` + `/nav/mission_event` 映射成云端 `mission_state`(phase / current_seq / progress / failure_message)。**decision 不拆航点、不逐段控制**。

---

## 5. 该读哪些文件

**接口名字(防止话题名打错):**
- `src/decision/src/actuator/nav_actuator.cpp` 第 9–11 行 —— decision 调用的 3 个 service 名
- `src/decision/src/module_data/nav_data.cpp` 第 12–15 行 —— decision 订阅的 2 个 topic 名

**消息字段定义(导航节点的包 `depend` 上 `m_common`,在这些文件里修改,勿另定义同名类型):**
- `src/m_common/srv/NavGotoPoint.srv`
- `src/m_common/srv/NavReturnHome.srv`
- `src/m_common/srv/NavReturnDock.srv`
- `src/m_common/srv/NavMissionUpload.srv`
- `src/m_common/srv/NavMissionExecute.srv`
- `src/m_common/srv/NavMissionControl.srv`
- `src/m_common/srv/NavHold.srv`
- `src/m_common/srv/NavEmergencyStop.srv`
- `src/m_common/msg/NavState.msg`(单目标周期)
- `src/m_common/msg/NavEvent.msg`(单目标事件)
- `src/m_common/msg/NavStatus.msg`(任务级周期)
- `src/m_common/msg/NavTaskEvent.msg`(任务级事件)

---

## 6. 消息字段

### NavGotoPoint.srv(`/nav/way_point`)
```
# request
float64 lat
float64 lon
float32 alt
float32 speed
uint32  seq        # decision 分配的关联号,须原样回带到 NavState/NavEvent
---
# response(立即返回=受理回执,≠到达!船是否真到达由 /nav/task_event 的 arrived 事件按 seq 回报)
int32   result     # 0=accepted, 非0=rejected
string  msg
```

### NavReturnHome.srv(`/nav/return_home`)
```
# request(目标 Home 位姿由 decision 从飞控 HOME_POSITION 取好后传入)
float64 lat
float64 lon
float32 alt
float32 speed
uint32  seq        # 原样回带到 NavState/NavEvent
---
# response(立即返回=受理回执,≠到达!船是否真到达由 /nav/task_event 的 arrived 事件按 seq 回报)
int32   result     # 0=accepted, 非0=rejected
string  msg
```

### NavReturnDock.srv(`/nav/return_dock`)
```
# request:船坞位姿由**导航节点自己预存储并读取**,decision 不传位姿,只发触发 + seq。
float32 speed      # 归港速度;0 = 交给 nav 用默认
uint32  seq        # 原样回带到 NavState/NavEvent
---
# response(立即返回=受理回执,≠到达!船是否真到达由 /nav/task_event 的 arrived 事件按 seq 回报)
int32   result     # 0=accepted, 非0=rejected
string  msg
```
> **归港目标位姿在导航节点侧预存储**,nav 收到本触发后读自己的船坞位姿并在 GUIDED 下带避障返回。decision 不再提供船坞坐标。

### NavHold.srv(`/nav/gcs_mission/cancel`)
```
# request: 空
---
int32  result
string msg
```

### NavEmergencyStop.srv(`/nav/emergency_stop`)
```
# request
string reason
---
int32  result
string msg
```

### NavMissionUpload.srv(`/nav/mission_upload`)/ NavMissionExecute.srv(`/nav/mission_execute`)

两者请求/响应字段**完全相同**,区别仅在语义:`upload`=仅装载不执行,`execute`=装载并立即执行。
```
# request(decision 把云端整条航线原样转发)
string  execution_id                 # 任务 ID,原样回带到 NavStatus.task_id / NavTaskEvent.task_id
float32 speed                        # 航线默认速度
uint8   finish_action                # 结束动作,见 V0.5 附录 E.4(0=返回上次/1=悬停/2=返航/3=重复/4=进坞)
m_common/Waypoint[] waypoints        # 整条航线;每个 Waypoint = { lat, lon, alt, WaypointAction[] actions }
---
# response(立即返回=受理回执,≠完成!任务进展由 /nav/mission_status + /nav/mission_event 回报)
int32   result     # 0=accepted, 非0=rejected
string  msg
```
> `Waypoint.actions[]` 里含 `HOVER`(悬停)/ `WATER_SAMPLE`(采水)等航点动作(V0.5 附录 E.3)。nav 负责在对应航点触发这些动作 —— 其中**采水要回到载荷通道发 `payload_control`(MAV_CMD_WATERPUMP)**,这条跨节点链路见 §8 开放点。**采水本期不启用,可先忽略 `WATER_SAMPLE`。**

### NavMissionControl.srv(`/nav/mission_control`)
```
# request:作用于当前正在执行的航线任务
string action     # "pause" / "resume" / "cancel"
---
# response(立即受理;状态变化仍由 /nav/mission_status + /nav/mission_event 回报)
int32   result    # 0=accepted, 非0=rejected
string  msg
```
> decision 收到云端 `mission_pause`/`mission_resume`/`mission_cancel` 时,原样把 `action` 转给本接口。nav 需支持在当前位置暂停并保留剩余航线,`resume` 时从暂停点续跑。

### NavState.msg(`/nav/nav_status`)——周期状态,回答“船现在在哪、在干嘛”

```
builtin_interfaces/Time stamp
string  internal_state
uint32  current_seq
float64 current_target_lat
float64 current_target_lon
```

**发布规则**:**周期发**,建议 **1~2 Hz**;QoS 默认 Reliable。无论有无任务都持续发(idle 也发),decision 靠它的 `stamp` 做**失联检测**(超时未更新 = nav 掉线)。

**字段逐条**:

| 字段 | 类型 | 由谁填 | 说明 |
|---|---|---|---|
| `stamp` | Time | nav | 本帧发布时刻,填 `now()`。decision 用它判老化/失联。 |
| `internal_state` | string | nav | nav 当前内部状态,取值见下表(字符串,大小写一致)。 |
| `current_seq` | uint32 | nav | **原样回带** decision 下发目标时给的 `seq`;无任务时填最后一次或 0。用于把这帧状态归到“哪个目标”。 |
| `current_target_lat` | float64 | nav | 当前正在去的目标纬度(WGS84 度);idle 时可填 0。 |
| `current_target_lon` | float64 | nav | 当前正在去的目标经度(WGS84 度);idle 时可填 0。 |

`internal_state` 取值:

| 值 | 含义 | 何时 |
|---|---|---|
| `idle` | 无任务待命 | 没有在跑任何目标 |
| `navigating` | 正常朝当前目标航行 | 受理目标后正常行进 |
| `avoiding` | 局部避障绕行中(终点仍是当前目标) | 遇障绕行时 |
| `arrived` | 已到达当前目标 | 到点瞬间(与下面的 `arrived` 事件同步) |
| `holding` | 停在原地 | 收到 `/nav/gcs_mission/cancel` 后 |
| `failed` | 当前目标失败 | 判定不可达/卡死后 |

> `NavState` 是**连续过程量**(用于显示进度、判失联),**不是到达判据**;到达判据是下面的 `NavEvent`。

### NavEvent.msg(`/nav/task_event`)——事件,回答“到了没 / 失败没”

```
builtin_interfaces/Time stamp
string  event_type
uint32  seq
string  detail
```

**发布规则**:**事件驱动**,状态**跳变时各发一次**(不是周期);QoS 默认 Reliable、建议 depth≥50 防丢。每条必带 `seq`。

**字段逐条**:

| 字段 | 类型 | 由谁填 | 说明 |
|---|---|---|---|
| `stamp` | Time | nav | 事件发生时刻。 |
| `event_type` | string | nav | 事件类型,取值见下表。 |
| `seq` | uint32 | nav | **原样回带**本事件对应目标的 `seq`。decision **只认 seq 匹配当前目标**的事件,防止上一目标的迟到事件误判。 |
| `detail` | string | nav | 自由文本补充(如失败原因、障碍描述);可空。 |

`event_type` 取值与 decision 处理:

| 值 | 含义 | decision 如何处理 | 是否必发 |
|---|---|---|---|
| `arrived` | **船真的到达当前目标点** | 判**成功**,推进到下一步 | ★**必发**(否则 decision 永远不知道到没到) |
| `unreachable` | 规划不出可行路径 / 目标不可达 | 判**失败** | ★失败时必发 `unreachable`/`stuck` 之一 |
| `stuck` | 长时间无进展、卡死 | 判**失败** | ★失败时必发 `unreachable`/`stuck` 之一 |
| `obstacle_detected` | 探测到障碍、进入避障 | 仅记日志 | 可选 |
| `replanned` | 触发重规划 | 仅记日志 | 可选 |
| `nav_resumed` | 避障/暂停后恢复正常航行 | 仅记日志 | 可选 |

> **导航节点最少要做到**:① 周期发 `NavState`(至少带 `internal_state` + `current_seq`);② 到达时发一条 `arrived`(带匹配 `seq`);③ 失败时发一条 `unreachable` 或 `stuck`(带匹配 `seq`)。其余事件可后补。三类任务(单点/返航/归港)**共用这同一套回报**,nav 侧不用为返航/归港单独加回报,decision 靠自己下发时记的 `seq` 就能区分是哪次任务。

### NavStatus.msg(`/nav/mission_status`)——多点航线任务的**周期快照**

`NavStatus` 字段较多(含 planner/controller/定位诊断),**任务进展只需下列子集为必填**,其余诊断字段可选、暂可留空/默认:

| 字段 | 类型 | 必填 | 说明 → 映射云端 `mission_state` |
|---|---|:--:|---|
| `header.stamp` | Time | ✅ | 发布时刻,用于失联检测 |
| `vehicle_id` | string | ⚪ | 车辆 ID |
| `task_state` | string | ✅ | `WAITING_SYSTEM/IDLE/RUNNING/PAUSED/COMPLETED/FAILED/EMERGENCY` → `mission_state.phase` |
| `task_id` | string | ✅ | **原样回带下发的 `execution_id`**;decision 靠它认领是哪条任务 |
| `current_waypoint` | int32 | ✅ | 已完成/当前航点数 → `mission_state.current_seq` |
| `total_waypoints` | int32 | ✅ | 航点总数 → `mission_state.waypoint_total` |
| `progress_percent` | float32 | ✅ | 0~100 → `mission_state.progress` |
| `elapsed_sec` | float32 | ⚪ | 已运行秒数 |
| `distance_to_goal_m` / `eta_sec` | float32 | ⚪ | 到目标距离 / 预计剩余(-1=未知) |
| `last_error` | string | ⚪ | 错误码,无错误留空 |
| `nav_phase` | string | ⚪ | `IDLE/TRACKING/STUCK/RECOVERY/PAUSED/EMERGENCY`,辅助诊断 |
| 其余 planner/controller/localization/pose/flags/alerts 字段 | — | ⚪ | 诊断用,任务进展不依赖,可后补 |

**发布规则**:周期 **~2 Hz**,QoS Reliable(建议 `TRANSIENT_LOCAL` 便于晚订阅者拿到最后一帧)。

### NavTaskEvent.msg(`/nav/mission_event`)——多点航线任务的**事件**

| 字段 | 类型 | 必填 | 说明 |
|---|---|:--:|---|
| `header.stamp` | Time | ✅ | 事件时刻 |
| `task_id` | string | ✅ | 原样回带 `execution_id` |
| `command_id` | string | ⚪ | 指令 ID(如有) |
| `event` | string | ✅ | 见下表 |
| 其余 detail 字段 | — | ⚪ | 按 `event` 选填(如 `total_waypoints`/`error_code`/`reason`/`nav2_error_code` 等) |

`event` 取值与 decision 处理:

| 值 | 含义 | decision → 云端 `mission_state.phase` | 是否必发 |
|---|---|---|:--:|
| `TASK_STARTED` | 任务开始执行 | `RUNNING` | ★必发 |
| `TASK_COMPLETED` | **整条航线跑完** | `COMPLETED`(任务会话结束) | ★必发 |
| `TASK_FAILED` | 任务失败(带 `error_code`/`reason`/`nav2_error_code`) | `FAILED`(结束) | ★失败时必发 |
| `TASK_CANCELLED` | 被取消 | `CANCELLED`(结束) | 取消时必发 |
| `TASK_PAUSED` / `TASK_RESUMED` | 暂停 / 恢复(带 `waypoint_index`) | `PAUSED` / `RUNNING` | 配合云端 mission_pause/resume |
| `EMERGENCY_STOP` | 急停触发 | `FAILED` + failure_message | 急停时发 |
| `ALARM_RAISED` / `ALARM_CLEARED` | 任务级告警(带 `alarm_*`) | 走告警通道,不改 phase | 可选 |

> **多点任务的“真完成”判据 = `/nav/mission_event` 的 `TASK_COMPLETED`**(对应云端 `mission_state.phase=COMPLETED`,任务会话 `bid` 到此结束,见 V0.5 §2.3.1)。`NavStatus` 只给连续进度,不作完成判据。
> **最少要做到**:① 周期发 `NavStatus`(带 `task_state`/`task_id`/`current_waypoint`/`total_waypoints`/`progress_percent`);② 发 `TASK_STARTED` / `TASK_COMPLETED`;③ 失败发 `TASK_FAILED`。

---

## 7. 需要遵守的语义约定

1. **`/nav/way_point` 立即返回**:`result` 只表示受理(0=接受,非0=拒绝),**绝不能等船开到目标才回**。否则 decision 的状态机会被该次 service 调用挂住。

2. **seq 必须原样回带**:decision 在 `NavGotoPoint.Request.seq` 给一个关联号(从 1 起自增),导航节点要把它原封不动填进 `NavState.current_seq` 与 `NavEvent.seq`。decision 靠它判断“这个 arrived 是不是当前目标的”,防止上一目标的迟到事件被误判为新目标到达。

3. **decision 真正消费的 `event_type`**(字符串,大小写一致):
   - `arrived` → 判为**到达成功**
   - `unreachable` / `stuck` → 判为**失败**
   - 其它(`obstacle_detected` / `replanned` / `nav_resumed`)→ decision 目前只记日志,不影响状态流

4. **`internal_state` 取值**:`idle / navigating / avoiding / arrived / holding / failed`。decision 用它辅助到达判定 + 老化检测失联(长时间不更新 = nav 失联)。

5. **QoS**:decision 端订阅用默认(Reliable, depth 10)。导航节点两个 publisher 用**默认 QoS(Reliable)**即可兼容;**不要用纯 BestEffort**,否则 decision 可能收不到。

6. **同一时刻只有一个目标**:decision 保证到达/失败前不会再发新的 `/nav/way_point`(中途新目标会被 decision 侧直接拒绝)。导航节点按“单目标、串行”实现即可。

7. **arming 自管**:导航节点持令牌期间自行 arm/disarm(见 §1.1);任务开始前确认 arm 成功再进 GUIDED,任务终止后 disarm 或按约定交回。sub_decision 在此期间不会与之抢 arming。

---

## 8. 实现现状 / TODO

### 8.1 decision 侧(已完成,已编译通过)

- **set_mode 分发器**([`MqttData::handleSetMode`](src/decision/src/module_data/mqtt_data.cpp)):`RTL`/`SMART_RTL`→返航、`RETURN_DOCK`→归港(入队 `ReturnRequest`);`MANUAL/SEMI_AUTO/HOLD/CRUISE`→boat 原生模式;`AUTO`/`GUIDED` 走专用通道,此处拒绝。
- **返航 / 归港**:新状态 `ReturnNavigatingState`(`Idle --return_received--> ReturnNavigating --done--> Idle`),调 `returnHome`/`returnDock`,消费 `/nav/task_event` 判终态。
- **多点航线整包转发**:`MissionReadyState` 调 `nav uploadMission`;`MissionExecutingState` 重写为 `executeMission` 整包下发 + 消费 `/nav/mission_status`/`/nav/mission_event`,`TASK_COMPLETED`→succeed、`TASK_FAILED`/`EMERGENCY_STOP`→abort,pause/resume/cancel 透传 `missionControl`。
- **NavData** 已订阅 `/nav/mission_status` + `/nav/mission_event` 并映射云端 `mission_state`。

### 8.2 导航节点侧(待实现)

- **8 个 service server 未实现**:`/nav/way_point`、`/nav/return_home`、`/nav/return_dock`、`/nav/mission_upload`、`/nav/mission_execute`、`/nav/mission_control`、`/nav/gcs_mission/cancel`、`/nav/emergency_stop`。
- **2 个任务级 publisher 未实现**:`/nav/mission_status`(`NavStatus`)、`/nav/mission_event`(`NavTaskEvent`)。
- **arming 自管**:任务开始自 arm、终止自 disarm,持令牌期间接管全部开锁/解锁。

### 8.3 遗留开放点

1. ~~归港船坞位姿来源~~ **已定并落地**:船坞位姿由**导航节点自己预存储读取**,decision 不传;`NavReturnDock` 只带 `speed`+`seq`,`ReturnNavigatingState` 归港分支直接 `returnDock(seq, speed)`。
2. ~~逻辑模式覆盖 OSD~~ **已撤销(见 §2)**:OSD `control_mode` 保持飞控原始模式,不做逻辑覆盖。返航/归港的"逻辑意图"若要给前端看,另走独立字段,本期不做。
3. **令牌 / arming 门控未写**:boat 与 nav 的 mavros 单写者互斥、交给 nav 时关闭 sub_decision arming 通道(见 §1)尚未实现。
4. **采水动作跨节点(本期不启用)**:`WATER_SAMPLE` 要走载荷通道 `payload_control`(WATERPUMP),归属未定;当前多点整包给 nav,`HOVER` 由 nav 自处理,采水暂忽略。
