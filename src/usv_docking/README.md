# usv_docking

差速 / 双推进器 USV 的 **GNSS 虚拟入口 + Tag 闭环** 精靠泊，以及 **odom 出泊**（v5.0）。

> **V2 重构版已并联上线（2026-07）**：四节点管线架构，全程倒船（船尾入坞，无线充电对接）。
> 旧 `docking_controller` 保留不动；新系统独立节点名/话题名（`_v2`、`/docking_v2/*`），见下方 [V2 架构](#v2-重构四节点管线2026-07)。

---

> **任务编排（一键归港/出泊、Nav 预泊、Entry 验收）** 见 [`../dock_mission/README.md`](../dock_mission/README.md)。  
> 本包只管：**预泊点之后 → GNSS 到虚拟入口 → 搜 Tag → 对准 → 倒船 → 停船**；**出泊 → odom 前进离坞**。

---

## V2 重构：四节点管线（2026-07）

**核心改动**：全程倒船（船尾朝坞，无线充电对接）；Tag 经 TF 输出（不再用 Float64MultiArray 话题）；odom 锚定/推算解耦为独立估计器；状态机/控制器/安全监督分离。

```text
AprilTag TF (camera_rear→dock_frame) + odom→base_link TF
  → docking_pose_estimator_v2     # TF 锚定 odom→dock_est、EMA+跳变拒绝、丢 Tag odom 推算
      │ /docking_v2/dock_pose (PoseStamped: base_link 在 dock_est 系)
      │ /docking_v2/tag_visible | pose_source | measurement_age
  → docking_fsm_v2                # 状态机 + /dock/status 兼容层
      │ /docking_v2/state | /docking_v2/target_mode
  → docking_motion_controller_v2  # 各阶段控制律 -> cmd_vel
      │ /cmd_vel_nav（test_only:=false）或 /docking_v2/cmd_vel_test
  → converter → 推进器
docking_safety_v2                 # 独立监督：6 项检查
      │ /docking_v2/safety_stop（控制器立即零速）| /docking_v2/abort_request（FSM 裁决）
```

**状态流**：
`IDLE → ACQUIRE_TAG → APPROACH_ENTRY → ALIGN_ENTRY → BACK_IN → FINAL_DOCK → DOCKED`
异常：入口外丢 Tag → `REACQUIRE_TAG`（闭环搜索）；坞内失败 → `ABORT_EXIT`（前进驶出后上报）；出泊：`UNDOCK_EXIT → UNDOCK_SETTLE`。

**关键约定**：
- `dock_est` 系：原点在坞中心，**+x 从入口指向坞内**（坞外 x<0）；对准 = 船艏向 ±π（`e_yaw = wrap(yaw − π)`，船尾朝坞）。
- 倒船控制律（差速运动学推导）：`ω = −kyaw·(e_yaw + ky·e_y)`；前进驶出：`ω = kyaw·(ky·e_y − e_yaw)`。APPROACH 也是船尾朝目标倒退，**全程保住后相机 Tag 视线**。
- `/dock/status` 契约（dock_mission 唯一消费）：`success`（DOCKED）/ `needs_reapproach`（入泊失败，FAILED 时强制 true 防挂死）/ `undock_success` / `state`（FAILED 映射 `"DOCK_ABORT"`，原名在 `v2_state`）/ `abort_reason`。
- 坞内丢 Tag 分级：0~0.5s 停车 → 0.5~2s 小角度搜索（±8°）→ >2s ABORT_EXIT；FINAL_DOCK 不搜索只等待 2s。

**📖 架构、状态机、上层接口（service/话题触发、/dock/status 契约、紧急状态）详见 [`docs/V2架构与上层接口.md`](docs/V2架构与上层接口.md)。**

全部参数集中在 `config/docking_v2.yaml`（四个节点分节，注释含符号推导与历次实测缺陷记录）。

### V2 分步复现与调试

```bash
# ── 步骤 0：编译（改了代码/yaml 都要执行；yaml 装在 share 里）──
cd ~/wuxihik_navigation
colcon build --packages-select usv_docking && source install/setup.bash

# ── 步骤 1：起仿真环境（二选一）──
# 1a. GUI 仿真：按 docs/项目运行与联调.md 三终端手动起（Gazebo GUI + 定位 + apriltag + converter）
# 1b. 无头仿真（推荐自动化调试用）：
bash scripts/headless_sim_up.sh        # gz server + EKF + apriltag + converter 一键拉起

# ── 步骤 2：起 V2 四节点（先观察模式验证数据链，再真实接管）──
export ROS_LOG_DIR=~/wuxihik_navigation/log/ros_smoke   # 沙箱环境防日志目录权限问题
ros2 launch usv_docking docking_v2.launch.py use_sim_time:=true test_only:=true   # 观察：指令发 /docking_v2/cmd_vel_test
ros2 launch usv_docking docking_v2.launch.py use_sim_time:=true test_only:=false  # 真实：接管 /cmd_vel_nav

# ── 步骤 3：验证数据链（10 秒冒烟，全部应有输出）──
ros2 topic echo --once /docking_v2/pose_source     # VISION（船尾相机对着坞时）
ros2 topic echo --once /docking_v2/dock_pose       # x 坞外为负，y 横向偏差
ros2 run tf2_ros tf2_echo odom dock_est            # 锚点 TF（坞在 odom 系位置）

# ── 步骤 4：触发归港 / 出泊 / 取消（直接发话题，绕过 dock_mission）──
ros2 topic pub --once /dock/start  std_msgs/msg/Bool  '{data: true}'    # 开始靠泊（注意是 Bool！）
ros2 topic pub --once /dock/cancel std_msgs/msg/Empty '{}'              # 取消 → IDLE 零速
ros2 topic pub --once /dock/undock std_msgs/msg/Bool  '{data: true}'    # 出泊（DOCKED 后）
# 经 dock_mission（GCS 接口）则是 service：
# ros2 service call /dock_task/command m_common/srv/DockTaskCommand "{command: 1, mission_id: '', command_id: '', require_camera: false}"

# ── 步骤 5：运行监控（各开一个 watch 终端）──
ros2 topic echo /docking_v2/state                  # FSM 状态流
ros2 topic echo /docking_v2/target_mode            # 控制器模式
ros2 topic echo /dock/status                       # 上层契约 JSON（成功看 success:true）
ros2 topic echo /docking_v2/pose_source            # VISION/ODOM_PREDICTION/INVALID 切换
ros2 topic echo /docking_v2/abort_request          # 安全撤离请求（正常为空串）

# ── 步骤 6：结束清理（pkill 模式用 [v] 字符类防自杀）──
pkill -9 -f "docking_pose_estimator_[v]2"; pkill -9 -f "docking_fs[m]_v2"
pkill -9 -f "docking_motion_controller_[v]2"; pkill -9 -f "docking_safet[y]_v2"
pkill -9 -f "docking_[v]2.launch"
pgrep -c -f "docking_pose_estimator_[v]2"          # 确认 0 才算清干净
```

**调试速查**：
- 船不动/速度不对 → 先查有无 **teleop_twist_keyboard** 残留（它会抢 cmd_vel_nav）：`pgrep -af teleop`；
  再查是否多套 V2 并存：`ros2 node list | grep v2` 每个节点应只有一个。
- 一直 REACQUIRE 打转 → 看 `/docking_v2/pose_source` 是否频繁 INVALID；RViz 里 `odom→dock_est` 锚点是否还在。
- 想看船在坞系实时位置：`ros2 topic echo /docking_v2/dock_pose`（x 向 0 收敛=倒入中，y=横偏）。
- 单元测试（30 项，无需仿真）：

```bash
source install/setup.bash
python3 -m pytest src/usv_docking/test/ -q
```

**仿真验证记录**（Gazebo）：
- 2026-07-28：出生点 −5.5m 全自动闭环归港至坞心（全链路状态转移 ✓）；倒船/横偏/驶出三组控制符号 ✓；自主出泊控制 ✓。
- 2026-07-29：BACK_IN 冻 tag → 分级停车/搜索 → ABORT_EXIT **纯 odom 推算自主驶出** −2.2→−4.4 → IDLE + `needs_reapproach` ✓；APPROACH 冻 tag → REACQUIRE 搜索 → 解冻恢复路由 ✓；第二次全链路至 DOCKED ✓。
- 2026-07-29（偏轴线入场，出生 x≈−9.5m、艏向背坞）：ACQUIRE 旋转搜索捕获 ✓ → APPROACH 倒退入场 ✓；FINAL_DOCK 卡死（见缺陷④⑤）→ 自触发 CORRIDOR_VIOLATION → ABORT_EXIT 驶出 ✓；撤离中 `/dock/cancel` 正确接管 → IDLE ✓。
- 2026-07-29（无头仿真自建，脚本 `scripts/headless_sim_up.sh`）：①偏轴线全链路（传送至坞系 −9.5m/横偏 1.5m/艏向背坞）：ACQUIRE 旋转搜索捕获 → 倒退 8.7m 收横偏 → ALIGN → BACK_IN → FINAL_DOCK 全程零抖动零中止 ✓；②预备点外短回路（−2.8m/横偏 0.3m）：**全链路至 DOCKED**，终点 x≈0.03、y≈−0.05、success=true ✓。
- 2026-07-29（GUI 仿真，含越点重启动场景）：船越过预备点卡在坞边（x=−2.06>staging_x=−2.5）→ 双向 APPROACH **前进倒出**修 y → ALIGN → BACK_IN → FINAL_DOCK → **DOCKED，终点 y≈0.015、success=true** ✓。当日修复三缺陷：⑥APPROACH 单一倒船律在"船在预备点内侧"时要求船尾调头 180°，stern_bearing 落 ±π 回绕奇点致 ±0.35 转向 bang-bang 震荡 → 双向化（内侧前进倒出，船尾相机始终朝坞）；⑦前进/弧线段纯 P 横向律无阻尼，大 e_y 蟹行角达 10° 全速横移、y 冲过 0 荡秋千（0.49→−0.20）→ `approach_crab_deg=8°` 限幅后单调平缓收敛；⑧y 容差治理：真船坞宽≈船宽，**y 必须在坞外修到位**（`align_y_tol` 收回 0.15=approach_y_tol，不达标回 APPROACH"向前挪动→弧线修 y"），BACK_IN 门控仅兜漂移（真船须按单侧间隙−余量收紧 gate1/gate2_y）。
- 2026-07-29 晚（GUI 仿真，用户重置环境后大偏轴 (−6.75, −2.98) 入场）：共识播种滑窗拒绝 0.64m 单双码离散 → 8 帧共识锚定 → APPROACH → ALIGN 锚点 EMA 精化逼出真 y=0.28 → **y 卡死逃逸**（6.1s）→ APPROACH **锥形降速**修 y（0.29→−0.10 单调无过冲）→ BACK_IN → **DOCKED (x=0.098, y=−0.042)，success=true 真成功** ✓。当日再修四缺陷：⑨估计器首帧即锚点 + 跳变拒绝自我强化偏差（单码远距播种偏 0.7m、真值被持续拒绝，旧码曾在坞外 2.66m 处假 DOCKED）→ 共识播种（8 帧中位数 + 离散度滑窗）+ 拒绝簇 EMA 吸附解锁；⑩集帧零容忍清零（视野边缘闪烁致捕获 6 分钟、HOLD↔SEARCH 角速度忽高忽低）→ acquire/reacquire_miss_tolerance=3；⑪APPROACH 全速修 y 过冲 → 锥形降速（slow_y=0.5 起降，min_speed=0.08 蠕行）；⑫ALIGN 卡死带 (y_tol 0.15, y_abort 0.35] 干等到 45s 超时 → align_y_stuck_sec=6.0 确定性回 APPROACH 修 y。另注意：仿真重启（时钟归零）后所有 use_sim_time 节点必须重启，否则定时器冻结成僵尸。
- 实测修复的设计缺陷：①估计器推算硬上限 3s→70s（否则撤离 3 秒即瘫）；②安全误差检查仅限坞内走廊 BACK_IN/FINAL_DOCK（APPROACH 远距离噪声、ALIGN 初始大艏偏都会误中止）；③搜索状态豁免安全 odom 时长检查（否则 REACQUIRE 活不过 3s）；④APPROACH→ALIGN 交接死区（approach_y_tol 0.5 > align_y_abort 0.35 → 两态高频抖动，approach_y_tol 收紧至 0.15 < align_y_tol 0.20）；⑤BACK_IN 横向收敛太慢（ky_back 0.2→0.8：收敛长度 5m→1.25m，否则 y 残差 >docked_y_tol 在 x 到点后形成 v=0 死锁）；⑥ALIGN 丢 Tag 零容忍与 REACQUIRE 看到 Tag 仍旋转导致两态互弹（加 5s 宽限 + 停车集帧 + 集帧 3→5）；⑦倒船律稳态 e_yaw=−ky·e_y 随横向残差必超 3° 判据形成终点死锁（新增终局消艏偏：x/y 达标后原地消 e_yaw）。
- 联调注意事项：①`/dock/start`/`/dock/undock` 是 **Bool** 不是 String；②同一话题多套节点并存会互相打架（kill 时 pkill 模式须用 `[v]` 类字符类避免匹配自身 shell）；③teleop_twist_keyboard（cmd_vel 重映射到 cmd_vel_nav）会与控制器抢速度，联调前确认已关。
- 备注：仿真机 gz 偶发崩溃（WaveVisual 析构段错误已用注释补丁规避；其余为外部退出，疑似 OOM），与算法无关。

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
| [`docs/V2架构与上层接口.md`](docs/V2架构与上层接口.md) | ★ V2 架构、状态机、上层触发/反馈契约、紧急状态、真船参数治理 |
| [`../dock_mission/README.md`](../dock_mission/README.md) | 一键归港、预泊点、Entry、Nav GoalChecker |
| [`../../docs/usv_docking任务规划.md`](../../docs/usv_docking任务规划.md) | 全栈架构 |
| [`../../docs/项目运行与联调.md`](../../docs/项目运行与联调.md) | 7 终端联调 |
| [`docs/GNSS阶段一设计与复盘_20260706.md`](docs/GNSS阶段一设计与复盘_20260706.md) | v5 GNSS leg lock、三档倒船、联调复盘 |
| [`docs/CHANGELOG_v5_gnss_approach.md`](docs/CHANGELOG_v5_gnss_approach.md) | v5 变更摘要 |
| [`docs/归港对准与位姿说明.md`](docs/归港对准与位姿说明.md) | x_base / heading_error 语义 |
