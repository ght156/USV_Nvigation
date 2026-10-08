# 2026-10-07 归港 REACQUIRE 盲驾把船带走 + 锚点 1m 乱跳 + 入口放宽与廊道碰撞

## 1. 基本信息

- 日期：2026-10-07（下午场，约 15:03–15:11 失败循环）
- 船：兴飞（`ssh hkzl`，MEC-MB002），ROS_DOMAIN_ID=5 / cyclonedds（eth1）
- 栈：`~/usv-main/install/arm`，四节点 `ros2 launch usv_docking docking.launch.py test_only:=false`
  （本轮栈于 11:5x 重启，含 2026-10-07 的"虚拟坞位回归"与"重定位守卫"两个新功能）
- 数据：
  - 全量 bag：`~/usv_bags/dock_20261007_145821_run6`（1.7 GB，含 IMU / 相机视频 / tf）
  - 导出时间序列（10 Hz）：主机 `/tmp/run6.csv`（分析脚本 `/tmp/an_run6.py`）
  - 关键帧：主机 `/tmp/frames_run3`、`/tmp/frames_run5`
  - 节点日志：`~/logs/xingfei_stack/docking.log`、`dock_mission.log`、`rs485.log`
- 参考对比：同日成功两轮 `dock_20261007_115224_run4_astuck`（已删）、`dock_20261007_120527_run5_deadlock`（已删），
  时间序列 `/tmp/run4.csv`、`/tmp/run5.csv`；前一日震荡轮 run3（bag 保留）

## 2. 现象（现场目视）

1. 触发归港时船位比较偏（横向/艏向都偏），归港**没有把船带回预备点**；
2. 归港过程中**船没有被停下**，反而越走越偏，**甚至往侧边跑远**，现场感觉危险；
3. 进坞段（廊道）**蹭碰比较多，是"挤进去"的**；
4. `dock_mission` 失败后只是重试，**没有触发 Nav2 回预泊点**。

## 3. 证据

### 3.1 失败循环时间线（`docking.log`，墙钟）

```
15:03:17  estimator: 连续5帧跳变，锚点重定位到 (-0.07, -0.77)
15:03:23  estimator: 拒绝锚点跳变 dp=1.21m dyaw=8.7°（1/5）
15:03:24  estimator: 连续5帧跳变，锚点重定位到 (-1.23, -0.01)
15:03:25  ACQUIRE_TAG → APPROACH_ENTRY
15:03:28  APPROACH_ENTRY → REACQUIRE_TAG        ← 进 APPROACH 仅 3.3s 就丢 Tag
15:03:38  REACQUIRE 推算驾驶累计超 reacquire_drive_max_sec(10s) 预算 → HOLD
15:03:58  REACQUIRE_TIMEOUT → IDLE
15:03:59  自动重试 → WAIT_DOCK_OPEN → DOCK_OPEN_FAILED（夹爪，今天第 3 次）
15:04:04  重试成功 → ACQUIRE_TAG
15:04:18  estimator: 拒绝锚点跳变 dp=1.09m dyaw=0.7°（1/5）→ 重定位到 (-0.52, -0.63)
15:04:19  ACQUIRE_TAG → APPROACH_ENTRY
15:04:29  → REACQUIRE_TAG（10.9s）
15:04:40  预算用完 → HOLD ； 15:05:00 REACQUIRE_TIMEOUT → IDLE
（此后又重复 2 轮，直到 15:10:59 最后一轮 REACQUIRE 结束）
```

### 3.2 REACQUIRE 段：**全程盲开（tag 可见率 0%），且在往前开**

`/tmp/run6.csv` 分段统计（cmd 取自 `/cmd_vel_nav`，即归港控制器输出）：

| REACQUIRE 段 | 时长 | pose_source | tag 可见 | \|v\|max | 移动时间占比 | 该段位移 |
|---|---|---|---|---|---|---|
| 段1 | 30.0s | ODOM_PREDICTION 100% | **0%** | 0.45 | 36% | x −6.28→−9.75（**−3.47m**） |
| 段2 | 30.0s | ODOM_PREDICTION 100% | **0%** | 0.45 | 80% | x −6.59→−11.13（**−4.53m**） |
| 段3 | 30.0s | ODOM_PREDICTION 100% | **0%** | 0.45 | 82% | x −4.33→−6.97，y −4.74→−7.38 |
| 段4 | 29.9s | ODOM_PREDICTION 99% | 3% | 0.45 | 90% | y −6.54→+5.50（**横摆 5.8m**） |

段2 逐秒细节（每 1s 抽样）：

```
 t(s)   cmd_v   cmd_w   src              x        y
  0.0  +0.000  -0.120   ODOM_PREDICTION  -6.59   -5.55   ← 原地转
  6.7  +0.330  -0.120   ODOM_PREDICTION  -6.33   -6.70   ← 开始前进
  7.8  +0.450  -0.120   ODOM_PREDICTION  -6.50   -6.95   ← 全速前进（背离坞）
 10.0  +0.450  -0.037   ODOM_PREDICTION  -7.50   -7.39
 11.1  +0.150  +0.000   ODOM_PREDICTION  -8.00   -7.45   ← 10s 预算用完，开始收
 12.2  +0.015  +0.000   ODOM_PREDICTION  -8.39   -7.47   ← 指令≈0
 19.3  +0.015  +0.000   ODOM_PREDICTION  -9.01   -7.76   ← 但船继续漂（水流 0.1~0.3m/s）
 30.0  +0.015  +0.000   ODOM_PREDICTION -11.13   -5.63
```

### 3.3 dock_mission 不触发 Nav2 回预泊

```
mission DOCK_FAILED → DOCK_ARMED dock_only requested
mission DOCK_ARMED → DOCK_HANDOFF dock_only skip nav      ← 明确跳过 Nav2 预泊
mission DOCK_HANDOFF → MONITOR_DOCK handoff usv_docking
dock session released: staging retry: DOCK_OPEN_FAILED...
dock_mission_node: staging retry 1/5: ...
```

`dock_only` 流程设计上就 `skip nav`，失败只做 staging retry（重跑归港 FSM），**没有任何环节把船送回预泊点**。

### 3.4 结束时船的真实状态

```
state=IDLE  pose_source=ODOM_PREDICTION  tag_visible=false  tag_age=22s
dock_x=-7.83  dock_y=+1.24  e_yaw=+151.94°      ← 艏向偏了 152°（~掉头），离坞 7.8m
```

后视相机看不到 tag，因为船已经基本掉头。

## 4. 根因

### 4.1 已确证：REACQUIRE 盲驾 + "先上轴"分支把船往远处开

链条（每一步都有上面的数据支撑）：

1. 丢 tag → FSM 进 `REACQUIRE_TAG`；此时 estimator 锚点还活着，`pose_source=ODOM_PREDICTION`；
2. **2026-10-07 新增的"虚拟坞位回归"**使 `REACQUIRE + PREDICTION → MODE_APPROACH`
   （旧版本此状态只发 HOLD，船不动）→ **在完全看不到 tag 的情况下开始驾驶**；
3. 控制器进入 **"先上轴"模式**：判据 `|e_y| > approach_axis_y_far(0.8)`，而此刻报出的
   `|e_y|` 高达 **5.5~7.9 m**（是锚点错误导致的假横偏）→ 目标被换成
   `staging = min(x, -4.5) − approach_axis_pullout(2.0)`，于是 `dx < 0` →
   走内侧**前进分支** → `v = +0.45 m/s`（**朝远离坞的方向**）；
4. 该分支本意是"横偏 1~2m 时先向外拉出再倒入"，前提是横偏真实；喂给它 7m 假横偏就变成"把船往外送"；
5. `reacquire_drive_max_sec(10s)` 预算耗尽后退 HOLD，但**船不会停**——水流本底 0.1~0.3 m/s，
   19s 又漂 3m；
6. 失败 → dock_mission 自动重试 → **进入 IDLE 会清零会话级盲驾预算**（`_update_flags_on_enter`）
   → 新一轮又给满 10s → 循环累计，船越走越远。

### 4.2 已确证：锚点在 1 m 量级乱跳，而重定位守卫只守 yaw 不守位置

`odom→dock` 锚点在 odom 系里**应为常量**，实际反复搬家：

```
dp=1.21m dyaw=8.7°   → 守卫放行（dyaw < reloc_max_yaw_jump_deg=10°）→ 重定位
dp=1.09m dyaw=0.7°   → 守卫放行 → 重定位
dp=0.37m dyaw=25.2°  → 守卫拦下（这条是守卫正确工作的例子）
```

量级（0.7~1.2 m）与 estimator 自己注释里的"**单双码系统差可达 0.7 m**"吻合，**疑似**单码/双码解算系统差
（待验证，见 §6）。锚点错 → dock 坐标系错 → `e_y/e_yaw` 全错 → 控制器追假目标 →
把 tag 甩出视野 → 进 REACQUIRE → 回到 §4.1。

（注：`reloc_max_yaw_jump_deg` 只判 yaw，是本项目 2026-10-07 评审时就指出过的漏洞，本轮现场实证。）

### 4.3 安全边界缺失（已确证）

- safety 的横向误差中止只在 `ERROR_CHECK_STATES = (BACK_IN, FINAL_DOCK)` 生效，
  **APPROACH / REACQUIRE 期间不检查横向误差**；
- `SEARCH_STATES = (ACQUIRE_TAG, REACQUIRE_TAG)` 豁免 odom 推算超时检查；
- 因此 REACQUIRE 盲目驾驶期间**没有任何基于误差/超时的保护**，唯一边界是
  `reacquire_timeout_sec(30s)` + `reacquire_drive_max_sec(10s)`，而**重试会重置预算**。
- 万幸：盲驾方向是"背离坞"，不会撞坞；但会把船带到开阔水域/其他障碍物附近。

## 5. 关联问题：入口（预备点）对齐放宽 → 廊道碰撞

### 5.1 昨天/今天放宽的"入口对齐"参数

2026-10-06（外八坞入口放宽）与 2026-10-07（提交 ALIGN）共改了 5 个：

| 参数 | 原值 | 现值 | 改时 | 作用点 |
|---|---|---|---|---|
| `approach_y_tol` | 0.15 | **0.25** | 10-06 | APPROACH → ALIGN 的 \|e_y\| 门槛 |
| `approach_yaw_tol_deg` | 10 | 15（10-06）→ **25**（10-07） | | APPROACH → ALIGN 的 \|e_yaw\| 门槛 |
| `align_y_tol` | 0.15 | **0.25** | 10-06 | **ALIGN → BACK_IN（即入廊道）的 \|e_y\| 门槛** |
| `align_yaw_tol_deg` | 5 | **10** | 10-06 | **ALIGN → BACK_IN 的 \|e_yaw\| 门槛** |
| `align_y_abort` | 0.35 | **0.45** | 10-06 | ALIGN 退回 APPROACH 的门槛 |

### 5.2 与廊道门控对比：入口比廊道本身还松

| 廊道侧参数（未放宽） | 值 |
|---|---|
| `back_in_gate1_y` / `back_in_gate1_yaw_deg` | **0.20** / **5.0°**（全速门控） |
| `back_in_gate2_y` / `back_in_gate2_yaw_deg` | 0.35 / 15.0°（超出即判走廊违规 → ABORT_EXIT） |
| `max_lateral_error` / `max_yaw_error_deg`（safety） | 0.50 / 15.0° |
| `entry_window_min_x` / `entry_window_max_x` | −3.5 / −1.0 |

`docs/架构与上层接口.md` 自己写着规则：

> **坞宽≈船宽**：y 必须在坞外修到位（`align_y_tol`），BACK_IN 门控（`back_in_gate1_y/gate2_y`）
> 仅兜漂移——按 **单侧间隙 − 安全余量** 收紧。

**现状矛盾**：`align_y_tol(0.25) > back_in_gate1_y(0.20)`、`align_yaw_tol_deg(10°) > gate1_yaw(5°)`，
即**船被允许以比廊道全速门控更差的状态进入廊道**。坞宽≈船宽时，0.25 m 横偏 + 10° 艏偏
基本就意味着船体已经压上廊道壁 —— 与现场观察"**蹭碰比较多，是挤进去的**"一致。

这是 10-06 放宽入口门槛的**副作用**：放宽是为了治入口磨蹭（APPROACH 退化区原地打转、昨天实测 86s / 120s 超时），
代价转移到廊道段。**两个方向都需要，但放错了位置**——应该"远处松、进廊道前紧"，见 §6 建议。

### 5.3 实测间隙 ±5 cm → 现有门控比物理间隙松 5~10 倍

现场实测（2026-10-07）：**船与船坞左右各剩 5 cm**（船宽 1.15 m ⇒ 廊道净宽 ≈1.25 m）。

几何约束：船体（宽 W、长 L）要以偏角 θ、横偏 y 进入净间隙 c=0.05 m 的廊道，须满足

```
|y| + (L/2)·sin|θ| ≤ c            （船首/尾扫掠不出廊道壁）
```

按 L≈1.6 m 估算：

| 条件 | θ=1° | θ=2° | θ=3° | θ=5° |
|---|---|---|---|---|
| 允许的 \|y\| | 3.6 cm | 2.2 cm | 0.8 cm | —（已超差） |

也就是说**入廊道时 `|y|` 应在 2~3 cm、`|e_yaw|` 应在 2° 量级**。与现有配置对比：

| 参数 | 现值 | 物理含义允许值 | 松紧倍数 |
|---|---|---|---|
| `align_y_tol`（入廊道 \|y\|） | 0.25 m | ≈0.03 m | **×8** |
| `align_yaw_tol_deg`（入廊道 \|θ\|） | 10° | ≈2° | **×5** |
| `back_in_gate1_y` / `gate1_yaw_deg` | 0.20 m / 5° | ≈0.03 m / 2° | ×7 / ×2.5 |
| `back_in_gate2_y` / `gate2_yaw_deg` | 0.35 m / 15° | —（远大于间隙） | ×12 / ×7 |
| `max_lateral_error`（safety 中止） | 0.50 m | ≈0.06~0.08 m | **×7~8** |

**结论（已确证）**：按 ±5 cm 的物理间隙，现有的入口/廊道门控**整体松一个数量级**，
所以"每次都是蹭着挤进去"是必然结果，不是偶发。现场观察的"进去了但还是歪"同样对应：
入廊道时允许 10° 艏偏，而物理上 >2° 就已经压壁。

### 5.4 "入口往坞里倒时突然出现大速度然后降下去"的机理（已确证）

run6 第一段 `BACK_IN` 实测（`/cmd_vel_nav`）：

```
t=0.0s  cmd_v=-0.080   x=-2.13  y=-0.07  eyaw=+1.3   ← 刚进 BACK_IN
t=0.5s  cmd_v=-0.230                                ← 快速上升
t=0.9s  cmd_v=-0.350
t=1.4s  cmd_v=-0.400                                ← 峰值 0.40（=back_in_speed_normal）
t=2.3s  cmd_v=-0.370
t=3.2s  cmd_v=-0.400
t=4.1s  cmd_v=-0.336                                ← 开始线性衰减
t=5.0s  cmd_v=-0.272
t=6.4s  cmd_v=-0.214                                （x=-0.88，dist=0.88 ⇒ 0.25×0.88=0.22 ✓）
```

两个原因叠加：

1. **速度上限是"阶跃"的**（controller `_control_loop`）：
   `cap = 0.40 (back_in_speed_normal) if |e_y|<gate1_y(0.20) and |e_yaw|<gate1_yaw(5°) else 0.20 (back_in_speed_slow)`。
   一进 BACK_IN 就因为"对准达标"直接拿到 0.40 的上限；
2. **纵向律是 `v = -min(cap, kx_final·dist)`**（`kx_final`=0.25）：入口处
   `dist≈2.1 m ⇒ 0.25×2.1=0.53 > cap`，所以被 cap 顶住 = **恒 0.40**；
   等到 `dist < 0.40/0.25 = 1.6 m` 之后才变成 `0.25·dist` 的**线性降速**。
   从 0 升到 0.40 用 1.4 s，是 `max_linear_accel(0.30 m/s²)` 的斜坡限出来的。

**风险**：0.40 m/s 对 ±5 cm 的廊道太快了——按实测 ~2 s 的转向响应滞后（§7.4），
船在这 2 s 里已经走了 0.8 m，而这段时间艏向是"开环"在漂。run6 第一段 BACK_IN 里
`e_yaw` 从 +1.3° 一路漂到 **−5.4°**（同时 `cmd_w` 多次打满 0.12）就是这个过程。
这也解释了"进去了但还是歪"。

## 6. 已做修改

### 6.1 事故本身：未改动

§4 的 REACQUIRE 盲驾路径**未做任何改动**（用户要求先记录、先不改）。
仅导出数据、清理了 2 个已分析完的监测 bag（`run4_astuck`、`run5_deadlock`）。

### 6.2 廊道/入口参数：2026-10-07 下午按现场反馈**适度收紧**（已生效）

现场反馈：**船与坞左右各约 5 cm**，但**机械本身有限位**会兜横向；`align_y_tol=0.25` 实测**能进去**，
故不做激进收紧；`max_lateral_error` 保持不动。**只动"入口"三项，廊道内的门控容忍一律不动**
（现场补充：*"途中容忍不用改，既然进去了，就不要再出去了"*）。

最终改动（3 项，已生效）：

| 参数 | 节点 | 原值 | 现值 | 理由 |
|---|---|---|---|---|
| `align_yaw_tol_deg` | fsm | 10.0 | **7.0** | **入廊道前**收小艏偏（现场要求 6~8） |
| `back_in_speed_normal` | controller | 0.40 | **0.25** | 治"入口突然窜速"（§5.4） |
| `back_in_speed_slow` | controller | 0.20 | **0.15** | 同上，缩小两级差 |

**刻意不动**（横向有机械限位兜底 / 已实测能进 / 进坞后不再驱赶）：

| 参数 | 值 | 理由 |
|---|---|---|
| `align_y_tol` | 0.25 | 机械限位兜横向；0.25 实测能进 |
| `back_in_gate1_y` / `back_in_gate1_yaw_deg` | 0.20 / 5.0 | 廊道内门控，不动（曾改 4°，已还原） |
| `back_in_gate2_y` / `back_in_gate2_yaw_deg` | 0.35 / 15.0 | 廊道违规→ABORT_EXIT；曾改 10°，**现场要求还原**：进去了就不要再被赶出来 |
| `max_lateral_error`（safety） | 0.50 | 用户明确不动 |
| `reacquire_drive_max_sec`（虚拟坞位） | 10.0 | **未关**，保持原样（§4 的问题仅记录未改） |

改动落点：
- 源码+yaml：`src/usv_docking/config/docking.yaml`、`docking_fsm.py`、`docking_motion_controller.py`
  （代码默认值同步改，`test_param_defaults` / `test_param_consistency` 103 项全过）；
- 船端：`~/usv-main/install/arm/usv_docking/share/usv_docking/config/docking.yaml` 已同步覆盖，
  并用 `ros2 param set` **在线生效**（FSM/控制器每周期读参数，无需重启，回读已确认）。

### 6.3 纠偏外边界 + 虚拟坞位距离门（2026-10-07 晚，已部署生效）

现场补充两条关键物理约束：**后视相机对二维码的检测极限 9~10 m**；**河道内没有那么大空间**。
由此定位并修掉两个"把船带走"的直接成因（均为代码改动，已重启生效）：

| 参数（新增） | 节点 | 值 | 作用 |
|---|---|---|---|
| `approach_axis_max_out_x` | controller | **−8.0** | 上轴/纠偏拉出的**硬边界**。原式 `min(x,-4.5)-2.0` 的目标**随 x 一起后退** → 永远差 2m 追不到 → 实测把船一路带到 x≈−13m、超出相机 9~10m 检测极限。改为 `max(min(x,-4.5)-2.0, −8.0)` 封底：界内正常拉出、到 −8 停、界外不会再往外开 |
| `reacquire_drive_max_dist_m` | fsm | **5.0** | **虚拟坞位驾驶的距离门**。现场判断："远距离锚点本来就必跳（重定位 dp 0.77~1.21m），拿它当真值盲开等于朝幻影开"。故 `\|dock_x\| > 5m` 时不允许 REACQUIRE 驾驶，退回 HOLD（旧行为）；进到门内才允许 |

回归测试：`test_axis_mode_pullout_capped`、`test_reacquire_virtual_dock_gated_by_distance`（全套 105 项通过）。

**部署踩坑记录（下次别再犯）**：ament_python 的模块在
`install/arm/usv_docking/lib/python3.10/site-packages/usv_docking/`，
**不是** `install/arm/usv_docking/lib/usv_docking/`（那里只有可执行入口脚本）。
第一次拷错位置 → `ros2 param get` 显示新参数 "not set" → 改用正确路径后重启才生效。

**待办（锚点治本方向，现场已给出思路）**：虚拟坞位/重定位应"**只在近距离 或 双码检测时才认**"
（现只做了距离门）。要做双码门，需让 estimator 订阅 `/apriltag_node/detections` 统计 tag 数并记录
锚点来源；顺带把该话题补进录包清单（本次 run6/run8 都缺，导致单双码假设无法验证）。

### 6.4 两天实测统计（2026-10-07 晚补，数据来源：船上 dock bag 的 `/dock/status`）

从 `~/usv_bags/dock_2026*` 的 bag 里统计"触发次数 / 成功 / 失败原因"
（脚本 `/tmp/stat_bags.py`；部分 bag 已删或打不开，所以是**下限**）：

**10-06（两个可读 bag，共 47 分钟）**

```
触发 18 次，成功(DOCKED) 2 次      成功率 11%
失败 16：ACQUIRE_TIMEOUT ×8（50%）  ← 找不到 tag（起始摆位/朝向）
         DOCK_OPEN_FAILED ×6（38%） ← 夹爪执行机构
         GLOBAL_TIMEOUT ×1，REACQUIRE_TIMEOUT ×1
```

**10-07（四个可读 bag，共 30 分钟）**

```
触发 12 次，成功(DOCKED) 5 次      成功率 42%
失败 7：REACQUIRE_TIMEOUT ×5
        DOCK_OPEN_FAILED ×2 / DOCK_RELEASE_FAILED ×3   ← 夹爪
        APPROACH_TIMEOUT ×1
```

**归类**：感知/对准类（ACQUIRE/REACQUIRE/APPROACH 超时）14 次；夹爪执行机构类 9 次。
现场结论：夹爪类归硬件+嵌软（节后交办）；"太远找不到 tag"归起始摆位，不属归港控制问题
（`one_click` 模式本可由 Nav2 预泊兜底，见 §7.5）。

### 6.5 更正：`/apriltag_node/detections` 其实**有**录进 bag（2026-10-07 晚）

之前（§7.2 第 7 条）判断"录包清单缺该话题"是**错的**：
`~/record_dock_data.sh` 第 38 行本来就含 `/apriltag_node/detections`；
run8 bag 里实测 **Count: 7287**（`ros2 bag info`）。（run6 那次确实没有，属个别会话，原因待查。）

顺带用 run8 的该话题做了第一次"单双码"核查（脚本 `/tmp/cc2.py`，窗口 1791360800~900）：

```
每帧 AprilTag 检测数分布: {0 个: 679 帧, 2 个: 181 帧, 3 个: 140 帧}
id 组合分布: () ×679 | (0,0,1) ×140 | (0,1) ×124 | (0,0) ×57
```

两个发现：

1. **tag 0 会被重复检出**（出现 `(0,0)` 与 `(0,0,1)` 共 197 帧 ≈ 全部检出帧的 60%），
   疑似同一码板被重复上报；而融合节点拿到的重复观测会不会把坞位姿带偏，**待验证**；
2. 真的"只剩单码"（如仅 `(0,1)` 缺 0 或仅 0）在数据里占比不高——所以
   "重定位 dp≈1m 的跳变 = 单码系统差"这个假设**尚未被证实**，需要专门对齐
   "跳变时刻 ↔ 检测数量"再看（本次 run8 的节点日志已被重启覆盖，只能下次带日志一起抓）。

### 6.6 安全兜底 + 锚点可信度门（2026-10-07 晚，已实现）

针对 §4 的两条根因各下一道防线：

**① 锚点可信度门（estimator）** —— 用 TF 判断"当前几个码可见"（实测 TF 里有
`apriltag_0/1`、`dock_frame_tag_0/1`、融合 `dock_frame`，各 8.5Hz，**无需引入 m_common 依赖**）：

| 参数 | 默认 | 行为 |
|---|---|---|
| `reloc_max_dist_m` | **6.0** | 船-坞距离 > 6m 时**拒绝重定位**（远场坞位姿不可信） |
| `reloc_require_dual_tag` | **true** | **明确只看到一个码**（n==1）时拒绝重定位；n==0（查不到码，观测本身无效）不据此拦，避免影响离线单测/无 apriltag 场景 |

门加在"真正改写锚点之前"，守卫逃逸路径与 yaw 合规路径都受管。

**② safety 三条"位姿不可信还在驾驶"兜底 —— 当天已加了，当天也撤掉了**（保留此段作为教训）：

曾实现（独立于 FSM 的最后防线，触发即发 abort_request）：

| 兜底 | 参数（默认） | 触发 |
|---|---|---|
| A 无视觉驾驶超时 | `blind_drive_max_sec`(8.0)/`blind_drive_min_speed`(0.05) | `pose_source≠VISION` 且 `\|cmd_v\|>0.05` 连续超 8s（撤离/出泊豁免） |
| B 横向误差合理性 | `sanity_max_lateral_error`(3.0)/`sanity_lateral_hold_sec`(3.0) | `\|e_y\|>3m` 持续 3s |
| C 越界还在往外开 | `out_of_range_x`(-8.5)/`out_of_range_hold_sec`(2.0) | `dock_x ≤ -8.5` 且 `cmd_v>0` 持续 2s |

**❌ 上线即出事并当日撤销（2026-10-07 18:21–18:24 现场）**：

```
IDLE → WAIT_DOCK_OPEN → ACQUIRE_TAG → [0.025 s] 安全退出请求: LATERAL_INSANE → IDLE   ← 连打 4 轮
```

**根因（我的实现 bug）**：三条检查的"持续计时"只在**非静默状态**里更新/清零
（代码写成 `if state not in QUIET_STATES: <检查> else: 清零`，但实际只写了 if 分支），
所以计时器在 **IDLE 期间不清零**：上一次尝试里一旦置过计时（当时 `|e_y|` 确实超限），
船在 IDLE 停留期间该计时**继续累加**，下一次归港一进 ACQUIRE_TAG（非静默）就立刻满足
“持续 > 3 s” ⇒ **秒级误中止，归港完全跑不起来**。

**教训**：给状态机/监督器加"持续 N 秒"类判据时，**必须显式定义"什么事件清零计时"**，
并且要覆盖**所有**不满足条件的路径（含静默状态、状态切换、任务开始）；
这类 bug 的表现是"刚进入活动状态就触发"，与"信号真的异常"极容易混淆。

**处置**：三条检查、参数、订阅、测试、文档**全部撤除**（`docking_safety` 回到原 6 项检查；
`ros2 param get` 显示 `sanity_max_lateral_error` 等已 `Parameter not set`）。
与 §5 一脉相承：**"位姿不可信"的根治点是锚点（①），不是加一层可能误判的驾驶禁令**。

回归测试：estimator 3 项（距离拦截/单码拦截/双码放行）保留 → 全套 **108 项通过**。

### 7.5 `dock_only` vs `one_click`：Nav2 预泊兜底只在后者生效（2026-10-07 澄清）

`dock_mission_node._on_staging_failed()`：

```python
if self._mode == "dock_only":
    self._transition(MissionState.DOCK_HANDOFF, reason)    # 原地重试，跳过 Nav2
else:
    self._transition(MissionState.NAV_TO_STAGING, reason)  # Nav2 回预泊点
```

触发 Nav2 回预泊点需**同时**满足：① 模式是 `one_click`；② 归港失败且 `/dock/status.needs_reapproach==true`；
③ `_staging_retry < staging_retry_max(5)`。命中后经 `mission_bridge` 的 `send_waypoints`
把预泊点（泊位库 `gnss_staging` 的 WGS84，或 GCS 传入的 `dock_point` 覆盖）交给 Nav2，
到达 → `SETTLE` → `DOCK_HANDOFF` → 再交归港 FSM。

各入口对应模式：`/dock/mission/start`、`/dock/home`、`{"action":"one_click_dock"}` → one_click；
**`{"action":"dock_only"}` → dock_only（永远跳过 Nav2）**。2026-10-07 全天测试用的都是 `dock_only`
（手册里的示例命令），所以"失败后船偏了没人把它带回预泊点"是**预期行为**，不是 bug。

> 现场决定：Nav2 侧尚未调参，暂不使用 `one_click`；失败后由人工把船开回预泊区。
> 另注：`nav2` 与 `mission_bridge` 此前**一直没起**（`start_xingfei_stack.sh` 里虽有，
> 但 10-07 上午进程里没有），16:27 整栈重启后才都在线。

## 7. 待验证 / 下一步

### 7.1 立即止血（可选，一条命令在线生效）

```bash
ros2 param set /docking_fsm reacquire_drive_max_sec 0.0
```

`REACQUIRE` 退回"原地 HOLD"（旧行为），**盲驾消失**；代价是失去"丢 tag 后主动开回重捕"的能力。
FSM 每周期读参数，改完立即生效，无需重启。

### 7.2 治本（按优先级）

1. **重定位守卫加位置门限**：`dp > ~0.5m 且 dyaw < 10°` 也不重定位（正是单码系统差指纹）；
2. **REACQUIRE 盲驾不用完整 APPROACH 律**：至少禁用"先上轴"分支，
   限制成"小角度对准 + 小倒速"；并加 `|e_y| > 2m` 时判定位姿不可信 → 停车/退人工；
3. **入口/廊道门槛分两步**（同时治入口磨蹭与廊道碰撞）：
   - ALIGN 入口保持较松（如 `align_y_tol 0.25`、`align_yaw_tol_deg 10`）→ 远处不磨蹭；
   - 在"**放行 BACK_IN 之前**"再按廊道能力收紧（例如 `align_arc` 把 `|y|` 修到 ≤0.15、
     `|e_yaw|` ≤5°，即回到廊道 gate1 之内）。需要先实测**单侧间隙**，按"间隙 − 安全余量"定值；
4. **按实测 ±5 cm 收紧廊道门控**（2026-10-07 现场数据，见 §5.3）：建议量级
   `align_y_tol ≈0.05`、`align_yaw_tol_deg ≈3`（入廊道前最后一道门）、
   `back_in_gate1_y/gate1_yaw_deg ≈0.05/2°`、`gate2 ≈0.08/4°`、
   `max_lateral_error ≈0.08`（safety 必须在压壁前中止）。**这些值需先与现场确认船宽/船长**；
5. **把 BACK_IN 速度上限从"阶跃"改成"连续随对准度衰减"**（§5.4）：
   例如 `cap = back_in_speed_slow · f(|e_y|, |e_yaw|)`，并**整体下调**
   `back_in_speed_normal`（0.40 对 ±5 cm 廊道明显偏快，建议 ≤0.15），
   消除入口那个"突然加速到 0.40 再线性降"的行为；
6. **safety 在 REACQUIRE/APPROACH 也监控横向误差**，并给盲驾加独立兜底（如累计盲驾位移上限）；
7. **录包清单**：`/apriltag_node/detections` **本次没录进 bag**（bag 24 个话题里没有它，虽然话题在线），
  导致无法验证"单双码"假设。建议补进录制清单并优先录它。

### 7.3 现场复现要点

- 让船在**偏离坞轴 1~2m、艏偏 >30°** 的位置触发归港（即 2026-10-07 上午那次"预备线带横偏"工况），
  观察是否出现锚点 1m 级重定位；
- 同时录 `/apriltag_node/detections`，对齐"锚点跳变时刻"与"检测数量变化"，验证单码/双码系统差；
- 廊道碰撞：在 BACK_IN 段抽相机帧 + 记录 `dock_y`，量化"入廊道时 \|y\|"与"是否发生接触"。

### 7.4 遗留（非本轮）

- 夹爪 `DOCK_OPEN_FAILED / DOCK_RELEASE_FAILED: failed to close DO0 and DO1` 今天出现 **4 次**
  （11:54×2、14:59×1、15:03×1），间歇性，疑似执行机构/限位层面问题，另立专项；
- 转向响应滞后 ~2 s（IMU 实测：`corr(cmd_w, imu_wz)` 即时仅 +0.42，延迟互相关 lag≈2.1s 处 +0.80），
  是 APPROACH 段反复摆头的可能根因之一。
