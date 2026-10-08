# 2026-10-07 Nav2 导航"一步不动 / 原地左右摆"：行为树缺 goal_checker + RPP 原地旋转阈值

## 1. 基本信息

- 日期：2026-10-07 傍晚（约 17:30–18:10）
- 船：兴飞（`ssh hkzl`，MEC-MB002），ROS_DOMAIN_ID=5 / cyclonedds
- 栈：`~/start_xingfei_stack.sh`（导航侧 `nav2_boat.launch.py params_file:=…/nav2_params_xingfei.yaml`、`mission_bridge`）
- 目标点：`map(166,144)`（去，49 m）、`map(128,128)`（回，29 m），由 `NavigateToPose` action 下发
- 数据：`~/usv_bags/dock_20261007_175112_navtest`（585 MB）、`…_175758_navtest2`（2.1 GB）；
  `~/logs/xingfei_stack/nav2.log`；主机分析脚本 `scripts/dock_test/{send_nav_goal,an_step}.py`（发送/分析同款）
- 前置：栈已重启（11 项全起），`planner_server`/`controller_server` 均 active，船 GUIDED+armed

## 2. 现象

1. 第一次发目标：**被接受**（`✓ 目标被接受，开始导航`），但 `distance_remaining` 一直是 49~50 m
   不下降，最后 `status=6`（ABORTED）。**船一步没动**。
2. 修掉第一个问题后（见 §4.1）：船**开始原地左右旋转、始终不前进**。

## 3. 证据

### 3.1 现象 1 的根因：FollowPath 每次都立刻中止

`~/logs/xingfei_stack/nav2.log`（原文）：

```
[ERROR] [controller_server]: FollowPath called with goal_checker name  in parameter
        'current_goal_checker', which does not exist.
        Available goal checkers are: general_goal_checker docking_goal_checker .
[WARN]  [controller_server]: [follow_path] [ActionServer] Aborting handle.
[ERROR] [bt_navigator]: Goal failed
```

排查链（逐条实测）：

| 检查 | 结果 |
|---|---|
| `ros2 param get /controller_server current_goal_checker` | **Parameter not set**（该参数未声明，也设不上：`cannot be set because it was not declared`） |
| `ros2 param get /bt_navigator default_nav_to_pose_bt_xml` | Nav2 **默认** `navigate_to_pose_w_replanning_and_recovery.xml` |
| 该默认 BT 里有没有 `GoalCheckerSelector` / `goal_checker_id` | **0 处**：`<FollowPath path="{path}" controller_id="FollowPath"/>` |
| Nav2 标准 `GoalCheckerSelector` **action** 是否在跑 | **否**（`ros2 action list` 无 goal_checker_selector） |
| `/goal_checker_selector` 是什么 | **`std_msgs/String` 话题，发布者是 `dock_mission_node`** —— 名字与 Nav2 的 action 同名但类型不同，**不是** Nav2 的 selector |

⇒ 默认 BT 不给 `goal_checker_id`，控制器回落到未声明的 `current_goal_checker`，名字为空 ⇒
**每次 FollowPath 立即 abort**，船不动。

### 3.2 现象 2 的根因：RPP 一直处于"原地旋转对齐"模式

`navtest` bag 里 `/cmd_vel_nav` 统计（215 s）：

```
v: min +0.000  max +0.000      |v|>0.05 占比 0%      ← 线速度恒为 0
w: -0.250 ~ +0.250，反向 8 次，周期 ≈17 s           ← 只左右摆
   t= 0~5s w=+0.25 ｜ 6~11s 衰减过零 ｜ 13~15s w=-0.25 ｜ 16~22s 再衰减过零 …
```

对应参数（`nav2_params_xingfei.yaml`）：

```yaml
use_rotate_to_heading: true
rotate_to_heading_min_angle: 0.0     # ← 元凶：0 = 只要航向有偏差(哪怕 0.1°)就停下原地对齐
rotate_to_heading_angular_vel: 0.25
```

机理：`shouldRotateToPath()` 判据是 `|angle_to_carrot| > rotate_to_heading_min_angle`。
`min_angle = 0` ⇒ 永远满足 ⇒ `rotateToHeading()` 把 `linear_vel` 置 0 只转。
而船体有 **1.36 s 转向滞后**（见 `docs/船体动力学指标.md`）⇒ 转过头 → 误差反号 → 反向转 ⇒
**误差永远回不到 0，永远出不了对齐模式**（周期 ≈ 转头时间 + 滞后）。

## 4. 已做修改（2026-10-07 在线生效）

### 4.1 绕过"缺 goal_checker"（临时，**未固化**）

复制 Nav2 默认 BT，给 FollowPath 补上 checker，并在**下发目标时**通过
`NavigateToPose.Goal.behavior_tree` 指定（不改配置、不重启）：

```xml
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="general_goal_checker"/>
```

### 4.2 RPP 原地旋转三参数（在线 `ros2 param set`，**未写入参数文件**）

| 参数 | 文件原值 | 在线改为 |
|---|---|---|
| `FollowPath.rotate_to_heading_min_angle` | 0.0 | **0.12**（6.9°） |
| `FollowPath.rotate_to_heading_angular_vel` | 0.25 | **0.12** |
| `FollowPath.max_angular_accel` | 2.5 | **0.1** |

> ⚠️ **后两个的依据不牢**：来源是 `归港现场调试手册` §5 里 2026-07-29 的"角速度封顶 ~0.13 rad/s"，
> 而 nav2 参数注释记的是 2026-10-05 水测 **~0.25 rad/s**，两者矛盾且**今天没有重测上限**
> （今天的阶跃试验只发 ±0.12，测不到上限）。现场决定：**导航参数由现场自己调**，本文只记录改动。

### 4.3 结果：导航恢复

```
去： (120.0,127.0) → (154.1,140.7)   目标 (166,144)，走了 ~35 m 后停下
回： (154.1,140.7) → (125.7,128.0)   目标 (128,128)，29 m 到达（终点距目标 2.3 m）
```

## 5. 机理补充（源码核对结论，供后续调参）

源码：`nav2_regulated_pure_pursuit_controller.cpp`（Humble 1.1.20）。

**① `rotate_to_heading_min_angle` 是"全程每周期都判"，不是"只在开头对齐"**

```cpp
if (shouldRotateToGoalHeading(carrot))      // ① 接近终点(dist<goal_dist_tol_)→转到规划终点朝向
    rotateToHeading(..., 终点朝向);
else if (shouldRotateToPath(carrot, ang))   // ② ★每周期判：|atan2(carrot)| > min_angle → 停下原地转
    rotateToHeading(..., ang);
else
    angular_vel = linear_vel * curvature;   // 正常跟路径（此时会走弧线）
```

⇒ **值小 = 一路上频繁"停下转正再走"（曲线/被水流带偏都会触发）；值大 = 边走边转（走弧线）**。
想做"只开头对齐一次"，只能：把 min_angle 设大（如 1.2~1.57）、关 `use_rotate_to_heading`、
或先单独发 `Spin`/`RotateToPose` 再导航。

**② `max_angular_accel` 同时管"起步"和"收尾"，两者方向相反**

```cpp
angular_vel = sign * rotate_to_heading_angular_vel_;                  // 目标转速
angular_vel = clamp(angular_vel, 实测ω - a·dt, 实测ω + a·dt);          // ① 相对实测值的爬坡限幅
double max_vel_to_stop = sqrt(2 * a * fabs(angle_to_path));           // ② 防过冲减速
if (fabs(angular_vel) > max_vel_to_stop) angular_vel = sign * max_vel_to_stop;
```

- **a 大**：起步立刻到 ω_max（不落静摩擦死区 ✓），但 `sqrt(2aΔθ)` 大 ⇒ 减速几乎不生效 ⇒ **转过头**；
- **a 小**：减速生效（残余角误差 ≈ `死区²/(2a)`，a=0.1 时 ≈2.9° ✓），但起步爬坡慢，
  **前 ~1s 指令落在 0.1 rad/s 死区里 → 船要等一秒才转**（兴飞实测死区在 0.1 以下）。

⇒ ω_max 越接近死区 0.1，a 的可用区间越窄。若 10-05 的 0.25 rad/s 上限属实，
把 `rotate_to_heading_angular_vel` 放到 ~0.2，两头都好办。

## 6. 顺带核对的结论（今天一起查的，避免以后重复排查）

| 疑问 | 结论 |
|---|---|
| 定位与导航是否用了**不同的图** | **不是**。`src/.../config/map.yaml` 与 `install/arm/.../share/.../config/map.yaml` **md5 相同**，pgm 也相同（`ec660bb3…`） |
| 定位用的基准 | `usv_map_odom_tf.yaml`：`map_origin_ref_key: ref_gnss_10` = map.yaml 里 [120.36793468, 31.48618175] ✓ |
| 飞控 `/mavros/gps_input/get_map_origin` 返回的是什么 | **是 odom/本地系的原点**（命名有歧义），返回 (31.4872566, 120.3692831, yaw 78.5°)，与 map.yaml 基准差 ~175 m —— 正常，它就是 odom 锚点，定位节点用它 + `ref_gnss_10` 算 map→odom |
| 导航图覆盖范围 | 可航区仅 133 k 像素 ≈ **5300 m²**（那张 pgm 里黑色=不可航）。船当时在**可航区西边缘**（代价 9，2 m 内已有代价 65 的格子）→ 这解释了"为什么选点很受限" |
| 船的实际 footprint | **≈2.7 m**（`nav2_params` 注释："船头在 base_link 前方约 2.8 m"）—— 比早先按 1.6 m 的估算大，涉及间隙/角度换算的文档（`docs/船体动力学指标.md`）需按 2.7 m 修正 |

## 7. 待办 / 下一步

1. **决定 goal_checker 走哪条路**（结构性问题，必须先定，否则调参无用）：
   ① 写自定义 BT（`<GoalCheckerSelector>` 或直接给 `FollowPath` 加 `goal_checker_id`）并配
   `default_nav_to_pose_bt_xml`；② 让 Nav2 的 selector action 真正起来（现在 `/goal_checker_selector`
   这个名字被 dock_mission 的 String 话题占用）；③ 给控制器一个默认可用的 checker。
2. **导航参数由现场自调**：`rotate_to_heading_min_angle`（建议不要 0，否则一步不走）、
   `rotate_to_heading_angular_vel`、`max_angular_accel`（关系见 §5）。当前在线值
   `0.12 / 0.12 / 0.1` 重启后会回到文件值 `0.0 / 0.25 / 2.5`。
3. **补测船的真实角速度上限**：阶梯递增（0.10→0.15→0.20→0.25→0.30→0.35 rad/s 各保持 3~5 s），
   用 IMU 实测定值，解决"0.13 vs 0.25"这处矛盾（同一方法见 `docs/船体动力学指标.md`）。
4. 参考行为树的 `Spin`/`Backup` 也报过超时（`Exceeded time allowance … Exiting Spin/DriveOnHeading`）——
   船转得慢，恢复行为的 time allowance 也要按实船放宽。
