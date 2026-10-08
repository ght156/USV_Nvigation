# usv_docking

差速 / 双推进器 USV 的 **Tag 闭环** 精靠泊（全程倒船，船尾入坞，无线充电对接），以及 **odom 出泊**。

> 旧 `docking_controller`（v4/v5 单节点链）已于 **2026-08-31 移除**（代码/launch/配置/专属测试；
> 回退见对应 commit，git 历史可随时找回）。

---

> **任务编排（一键归港/出泊、Nav 预泊、Entry 验收）** 见 [`../dock_mission/README.md`](../dock_mission/README.md)。  
> 本包只管：**预泊点之后 → 搜 Tag → 对准 → 倒船入坞 → 停船**；**出泊 → 前进驶离**。

> **实船仓（USV_NAV）说明（2026-09-02）**：本包已从仿真仓迁移到实船仓。
> `docking.launch.py` 的 `use_sim_time` 默认改为 **false**（实船），并新增
> `params_file` 参数便于传实船标定后的整份 yaml。运行时依赖：
> TF `odom→base_link`（飞控/EKF）与 `base_link→dock_frame`（AprilTag 定位节点，
> **本仓暂未包含，需另行部署**）、Odometry 话题（`odom_speed_topic`，实船配
> `/mavros/gps_input/local`）、以及能消费 `/cmd_vel_nav` 的执行端
> （`usv_ardupilot_velocity_bridge`，自带 1s 看门狗兜底清零）。

---

## 四节点管线架构

**核心改动**（2026-07 重构）：全程倒船（船尾朝坞，无线充电对接）；Tag 经 TF 输出（不再用 Float64MultiArray 话题）；odom 锚定/推算解耦为独立估计器；状态机/控制器/安全监督分离。

```text
AprilTag TF (camera_rear→dock_frame) + odom→base_link TF
  → docking_pose_estimator     # TF 锚定 odom→dock_est、EMA+跳变拒绝、丢 Tag odom 推算
      │ /docking/dock_pose (PoseStamped: base_link 在 dock_est 系)
      │ /docking/tag_visible | pose_source | measurement_age
  → docking_fsm                # 状态机 + /dock/status 兼容层
      │ /docking/state | /docking/target_mode
  → docking_motion_controller  # 各阶段控制律 -> cmd_vel
      │ /cmd_vel_nav（test_only:=false）或 /docking/cmd_vel_test
  → converter → 推进器
docking_safety                 # 独立监督：6 项检查
      │ /docking/safety_stop（控制器立即零速）| /docking/abort_request（FSM 裁决）
```

**状态流**：
`IDLE → ACQUIRE_TAG → APPROACH_ENTRY → ALIGN_ENTRY → BACK_IN → FINAL_DOCK → DOCKED`
异常：入口外丢 Tag → `REACQUIRE_TAG`（闭环搜索）；坞内失败 → `ABORT_EXIT`（前进驶出后上报；位姿 INVALID 超时则诚实上报"驶出未确认"，不盲报可重试）；出泊：`UNDOCK_EXIT → UNDOCK_SETTLE`。
船坞夹爪交互（`dock_claw_enabled=true` 才启用，默认关闭）：归港前 `WAIT_DOCK_OPEN` 请求船坞打开夹爪（嵌软 `usv_rs485_driver` 的 ControlDO action），成功才进 ACQUIRE_TAG；BACK_IN/FINAL_DOCK 发 WaitIO 等夹爪抓住，**抓住即 DOCKED**（位置到位不再判成功）；倒船位移停滞超 `backin_stuck_timeout_sec` → ABORT_EXIT 驶出重试；出泊先 `WAIT_DOCK_RELEASE` 请求松开夹爪，失败/超时 → FAILED（禁止盲动）。

**关键约定**：
- `dock_est` 系：原点在坞中心，**+x 从入口指向坞内**（坞外 x<0）；对准 = 船艏向 ±π（`e_yaw = wrap(yaw − π)`，船尾朝坞）。
- 倒船控制律（差速运动学推导）：`ω = −kyaw·(e_yaw + ky·e_y)`；前进驶出：`ω = kyaw·(ky·e_y − e_yaw)`。APPROACH 也是船尾朝目标倒退，**全程保住后相机 Tag 视线**。
- `/dock/status` 契约（dock_mission 唯一消费）：`success`（DOCKED）/ `needs_reapproach`（入泊失败，FAILED 时强制 true 防挂死）/ `undock_success` / `state`（FAILED 映射 `"DOCK_ABORT"`，原名在 `v2_state`）/ `abort_reason`。
- 坞内丢 Tag 分级：0~0.5s 停车 → 0.5~2s 小角度搜索（±8°）→ >2s ABORT_EXIT；FINAL_DOCK 不搜索只等待 2s。

**📖 架构、状态机、上层接口（service/话题触发、/dock/status 契约、紧急状态）详见 [`docs/架构与上层接口.md`](docs/架构与上层接口.md)。**

全部参数集中在 `config/docking.yaml`（四个节点分节，注释含符号推导与历次实测缺陷记录）。

### 分步复现与调试

```bash
# ── 步骤 0：编译（改了代码/yaml 都要执行；yaml 装在 share 里）──
cd ~/USV_NAV
colcon build --merge-install --packages-select usv_docking && source install/setup.bash

# ── 步骤 1：起实船 bringup + Nav2（见仓根 docs/ 与 scripts/start_nx_stack.sh）──
#   另需 AprilTag 定位节点广播 base_link→dock_frame TF（本仓暂未包含）

# ── 步骤 2：起四节点（先观察模式验证数据链，再真实接管）──
ros2 launch usv_docking docking.launch.py                    # 观察：指令发 /docking/cmd_vel_test
ros2 launch usv_docking docking.launch.py test_only:=false   # 真实：接管 /cmd_vel_nav

# ── 步骤 3：验证数据链（10 秒冒烟，全部应有输出）──
ros2 topic echo --once /docking/pose_source     # VISION（船尾相机对着坞时）
ros2 topic echo --once /docking/dock_pose       # x 坞外为负，y 横向偏差
ros2 run tf2_ros tf2_echo odom dock_est            # 锚点 TF（坞在 odom 系位置）

# ── 步骤 4：触发归港 / 出泊 / 取消（直接发话题，绕过 dock_mission）──
ros2 topic pub --once /dock/start  std_msgs/msg/Bool  '{data: true}'    # 开始靠泊（注意是 Bool！）
ros2 topic pub --once /dock/cancel std_msgs/msg/Empty '{}'              # 取消 → IDLE 零速
ros2 topic pub --once /dock/undock std_msgs/msg/Bool  '{data: true}'    # 出泊（DOCKED 后）
# 经 dock_mission（GCS 接口）则是 Trigger service：
# ros2 service call /dock/mission/start std_srvs/srv/Trigger {}

# ── 步骤 5：运行监控（各开一个 watch 终端）──
ros2 topic echo /docking/state                  # FSM 状态流
ros2 topic echo /docking/target_mode            # 控制器模式
ros2 topic echo /dock/status                       # 上层契约 JSON（成功看 success:true）
ros2 topic echo /docking/pose_source            # VISION/ODOM_PREDICTION/INVALID 切换
ros2 topic echo /docking/abort_request          # 安全撤离请求（正常为空串）

# ── 步骤 6：结束清理（pkill 模式用字符类防自杀）──
pkill -9 -f "docking_pose_estimato[r]"; pkill -9 -f "docking_fs[m]"
pkill -9 -f "docking_motion_controlle[r]"; pkill -9 -f "docking_safet[y]"
pkill -9 -f "docking\.l[a]unch"
pgrep -c -f "docking_pose_estimato[r]"          # 确认 0 才算清干净
```

**调试速查**：
- 船不动/速度不对 → 先查有无 **teleop_twist_keyboard** 残留（它会抢 cmd_vel_nav）：`pgrep -af teleop`；
  再查是否多套并存：`ros2 node list | grep docking` 每个节点应只有一个。
- 一直 REACQUIRE 打转 → 看 `/docking/pose_source` 是否频繁 INVALID；RViz 里 `odom→dock_est` 锚点是否还在。
- 想看船在坞系实时位置：`ros2 topic echo /docking/dock_pose`（x 向 0 收敛=倒入中，y=横偏）。
- 单元测试（67 项，无需仿真）：

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
- 2026-08-05（GUI 仿真，偏轴位置起步：dock_est 初始 x≈−7.3、y≈−2.8、e_yaw≈−28°；16ms 步长 RTF≈0.5）：ACQUIRE→APPROACH 正常，**复现“靠近中轴线时横穿轴线、船 x 轴与坞轴偏差大”**。APPROACH 把 y 修到 −0.05 后即放行 ALIGN，但进 ALIGN 时 **e_yaw 仍 ≈+57°**（`_staging_reached` 只查 x/y，不查艏向）；ALIGN 原地转艏向时残余横向漂移使 y 冲过 0（−0.05→+0.35，过线瞬间 e_yaw≈53°），触发 `align_y_abort` 弹回 APPROACH；第二次 ALIGN 又以 e_yaw≈58° 进入，再次横穿轴线（+0.08→−0.14，过线瞬间 e_yaw≈−45°）；第二次转正后 BACK_IN→FINAL_DOCK→**DOCKED (x≈−0.02, y≈−0.04, e_yaw≈2.2°, success=true)**，总耗时约 116 仿真秒。**状态：现象已复现，待修复**；当日尝试“look-ahead 收艏向 + ALIGN 入口 e_yaw 门槛”（固定前伸与预备点停止逻辑冲突致 y 残留 −0.26m 缓慢微调、移动 pursuit 未验证即震荡），**已回退到快照 61e72a7**，试验详情与下次建议见 [归港冲线修复试验记录](../../docs/archive/归港冲线修复试验记录_20260805.md)。
- 实测修复的设计缺陷：①估计器推算硬上限 3s→70s（否则撤离 3 秒即瘫）；②安全误差检查仅限坞内走廊 BACK_IN/FINAL_DOCK（APPROACH 远距离噪声、ALIGN 初始大艏偏都会误中止）；③搜索状态豁免安全 odom 时长检查（否则 REACQUIRE 活不过 3s）；④APPROACH→ALIGN 交接死区（approach_y_tol 0.5 > align_y_abort 0.35 → 两态高频抖动，approach_y_tol 收紧至 0.15 < align_y_tol 0.20）；⑤BACK_IN 横向收敛太慢（ky_back 0.2→0.8：收敛长度 5m→1.25m，否则 y 残差 >docked_y_tol 在 x 到点后形成 v=0 死锁）；⑥ALIGN 丢 Tag 零容忍与 REACQUIRE 看到 Tag 仍旋转导致两态互弹（加 5s 宽限 + 停车集帧 + 集帧 3→5）；⑦倒船律稳态 e_yaw=−ky·e_y 随横向残差必超 3° 判据形成终点死锁（新增终局消艏偏：x/y 达标后原地消 e_yaw）。
- 联调注意事项：①`/dock/start`/`/dock/undock` 是 **Bool** 不是 String；②同一话题多套节点并存会互相打架（kill 时 pkill 模式须用 `[v]` 类字符类避免匹配自身 shell）；③2026-09-01 起控制器在 HOLD/DOCKED_HOLD 停稳 1s 后静默，teleop 可直接接管 cmd_vel_nav（运动态期间仍会互抢，手动接管前先 `/dock/cancel`）。
- 2026-09-11（实船联调：归港一直刷 `odom TF 不可用（age=3.07s）`，`pose_source` 恒 INVALID）：**根因不在 odom，而在估计器自己把 /tf 订阅饿死了**。`_lookup_2d` 用 `tf_timeout_sec=0.1` 阻塞查询，目标帧不在缓存里时（未检测到 Tag 的 `base_link→dock_frame`）每周期干等满 0.1s → 20Hz 定时器永远追不上 → 单线程 executor 被定时器回调占满 → `/tf` 订阅回调被饿死 → 估计器自己的 tf buffer 落后实时流 2.4~4.3s → 超过 `odom_tf_timeout_sec`(3.0) 误判 INVALID（启动头 10s 的 `age=infs` 同因）。独立纯订阅监听同一时刻看到 odom 是 10Hz、时间戳略超前（新鲜），坐实误报；照抄该模式（20Hz 定时器 + 0.1s 阻塞查询）可 1:1 复现。修复双层：①`tf_timeout_sec` 默认改 **0（非阻塞）**，定时器 20Hz 重试本就够（实测恢复 20.0Hz、告警消失）；②`main()` 改 `MultiThreadedExecutor`（tf2_ros 的 /tf 订阅建在 ReentrantCallbackGroup，定时器在默认互斥组）兜底，即使有人再配回阻塞也不会饿死订阅（实测 0.1s 阻塞下 dock_pose 掉到 10Hz 但 odom 不再误判）；③新增周期超时看门狗，回调超过 2 个周期即告警（该类缺陷下次可直接看见）。回归测试 `test_lookup_missing_frame_does_not_block`。**注意：本次会话里根本没有 AprilTag 定位节点（节点列表无 tag/april 节点，/tf、/tf_static 均无 `base_link→dock_frame`），修好本缺陷后仍须先部署该节点才能归港。**
- 备注：仿真机 gz 偶发崩溃（曾怀疑 WaveVisual 析构段错误并注释禁用，但禁用后出港崩溃仍复现，2026-09-01 已恢复该插件；其余为外部退出，疑似 OOM），与算法无关。
- 2026-08-22（无头仿真基线对比）：治本项——**ALIGN 弧线化**（v=0 原地转 → 小倒速 `align_arc_speed`=0.08 弧线消艏向，x≥`align_arc_x_limit`=−1.5 回退 v=0 防卡死滑入坞）。根因：差速双推进在船尾（x=−0.3），v=0 旋转绕推进器中点、base_link 扫弧 Δy≈R_eff·sinθ（sim R_eff≈0.3~0.42m）；实船 MAVROS→ArduPilot Guided 链路 R_eff 未知且不同，故选**行为层修复**（不依赖执行链参数，横漂≈|v|·sin(e_yaw)，入口 e_yaw≤10° 下≈1.4cm/s），对仿真/实船通用。基线（含 08-10 三修复）同偏轴场景 DOCKED (x=0.087, y=0.029, e_yaw=−1.78°，验证记录见 logs/dock_baseline_run_20260822.log)；弧线化修复单元 50 项全过，但同场景无头复测两次均卡在 **APPROACH→REACQUIRE**（预备点附近 e_yaw≈70° 原地修艏向时后相机丢失 Tag、REACQUIRE 30s 超时）——该阶段与本修复无关（仅 ALIGN 分支改动，未进入 ALIGN），属既有 flaky 项，旧码基线恰巧 1.4s 重捕获成功，新码两次未捕获；非回归，须另立专项（见下条）。
- 2026-08-31（代码评估 + 无头仿真端到端复测，详见仿真仓 docs/归港修复评估与仿真验证_20260831.md，日志 logs/dock_eval_run_20260831.log）：评估修复五项——⑬safety 推算超时按状态分档（ALIGN 新参数 `align_odom_prediction_timeout`=7.0s > FSM 宽限 5s，此前 3.5s 抢跑使 ALIGN 的 REACQUIRE 路径永远走不到）；⑭ABORT_EXIT 位姿 INVALID 超时不再假报可重试，改报 `ABORT_EXIT_UNCONFIRMED`+`needs_reapproach=false`+`needs_manual_takeover=true`，safety 新增 `EXIT_POSE_LOST` 纯告警（5s）；⑮`_enter()` 补复位 `_approach_tag_loss`/`_staging_hold`（残留计时致 REACQUIRE 返回后复弹）；⑯FINAL_DOCK 入口 y 门控 `final_dock_entry_y_tol`=0.15 + 控制器 x 到点 y 超差时蠕行续修（原干等 60s）；⑰FSM/控制器 16 项代码默认值与 yaml 对齐 + `test_param_defaults.py` 防漂移。单测 61/61。**无头复测**（同 08-22 偏轴位姿）：ACQUIRE 搜索 32.6s → APPROACH 收 y（−3.96→0.13）→ e_yaw≈73° 丢 Tag 触发 REACQUIRE **受控旋转 2.2s 重捕**（8ed8267 修复场景首次闭环验证，旧码此处 30s 超时死）→ ALIGN（入口 e_yaw=6.1°）→ BACK_IN → FINAL_DOCK → **DOCKED (x=0.095, y=0.026, e_yaw=−1.91°, success=true)**，全程 99.6 仿真秒，无冲线/无 abort/无互弹。遗留：cmd_vel 下游看门狗上实船前补（`test_only:=false` 前提）；`ABORT_EXIT_UNCONFIRMED` 场景 dock_mission 停 MONITOR_DOCK 待人工 `/dock/cancel`。

- 2026-10-07（实船兴飞，两次现场会话；完整复盘见 `docs/bug_debug/2026-10-07_归港REACQUIRE盲驾把船带走与锚点1m乱跳.md`）：
  ①**锚点远场重定位跳变 ~1m 引发"越走越远"**——远场观测把 `odom→dock` 锚点搬走 0.77~1.21m
  （dyaw 仅 0.7~8.7° < `reloc_max_yaw_jump_deg`(10°) 门限 → 守卫放行），dock 系随之报出 5~7m **假横偏**；
  ②**先上轴目标随船后退（根因）**——`staging = min(x, approach_axis_far_x) - approach_axis_pullout`
  = `min(x,-4.5)-2.0` 永远比当前位置外 2m，永远追不到 → 以 0.45m/s 一路把船带到 **x≈-13m**
  （超出后视相机 9~10m 检测极限）→ 丢 Tag。新增 `approach_axis_max_out_x`（默认 **-8.0**）封底
  + 回归测试 `test_axis_mode_pullout_capped`；
  ③**虚拟坞位盲驾**——`REACQUIRE + ODOM_PREDICTION → MODE_APPROACH` 在无 Tag 下驾驶，实测单段位移
  3.4~4.5m、累计把船带走。新增 `reacquire_drive_max_dist_m`（默认 **5.0**）距离门：`|dock_x|>5m`
  禁止驾驶、退回 HOLD + 回归测试 `test_reacquire_virtual_dock_gated_by_distance`；
  ④**入口/廊道门槛收紧**（廊道单侧实测仅 5cm）——`approach_y_tol`/`align_y_tol` 0.25→**0.15**、
  `align_yaw_tol_deg` 10→**7**、`back_in_speed_normal/slow` 0.40/0.20→**0.25/0.15**
  （0.40 配 ~2s 转向滞后会"窜着进入"；实测入段横偏 0.236m 时艏偏被廊道壁推到 −15°、0.07m 时仅 −5.9°）；
  `back_in_gate1/2_yaw_deg` **维持 5/15**（现场要求：既然进去了就别再被赶出来）；
  ⑤**两天失败统计**（bag `/dock/status` 统计）：10-06 触发 18 / 成功 2（**11%**，主因
  `ACQUIRE_TIMEOUT`×8 + 夹爪 `DOCK_OPEN_FAILED`×6）；10-07 触发 12 / 成功 5（**42%**，
  主因 `REACQUIRE_TIMEOUT`×5 + 夹爪×5）；
  ⑥**澄清 Nav2 兜底**——`dock_only` **永不**触发 Nav2 预泊（失败原地重试、船停哪算哪），
  只有 `one_click`（`/dock/mission/start`、`/dock/home`、`{"action":"one_click_dock"}`）才在
  `needs_reapproach=true` 时经 Nav2 回预泊点（≤`staging_retry_max`=5 次）；
  ⑦**记录更正**——`/apriltag_node/detections` **本来就在**录包清单里（run8 bag 实测 Count 7287）；
  该话题实测 **tag0 被重复检出**（`(0,0)`/`(0,0,1)` 占检出帧约 60%），是否带偏坞位姿待验证；
  ⑧**部署路径教训**——ament_python 模块在
  `install/arm/usv_docking/lib/python3.10/site-packages/usv_docking/`（**不是** `lib/usv_docking/`，
  后者只有可执行入口脚本）；拷错位置时 `ros2 param get` 会显示新参数 "not set"。

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
| [`docs/架构与上层接口.md`](docs/架构与上层接口.md) | ★ 架构、状态机、上层触发/反馈契约、紧急状态、真船参数治理 |
| [`../dock_mission/README.md`](../dock_mission/README.md) | 一键归港、预泊点、Entry、Nav GoalChecker |
| [`../../../docs/归港任务对接文档.md`](../../../docs/归港任务对接文档.md) | 嵌软状态机对接契约（dock_mission 触发/状态/事件） |
| [`../../../docs/导航与归港异常告警.md`](../../../docs/导航与归港异常告警.md) | 告警清单与处置 |
