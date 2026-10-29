# 2026-10-29 船不动：局部代价图 `current_` 卡死（Nav2 Humble InflationLayer reset 缺陷）+ map_server 启动竞态

## 1. 基本信息

| 项 | 值 |
|---|---|
| 日期 | 2026-10-29 |
| 船 / 主机 | 兴飞船（MEC-MB002，Jetson NX），`ssh hkzl` → 192.168.2.101 |
| 工作区 | `~/usv-test`（脚本 `~/usv-test/start_usv_nav.sh`，tmux 会话 `usv_nav`） |
| 环境 | ROS 2 Humble / FastDDS（`rmw_fastrtps_cpp`）/ `ROS_DOMAIN_ID=5` |
| 会话时间段 | 07:50 – 09:35 |
| 数据来源 | `~/.ros/log/2026-10-29-08-01-28-167635-MEC-MB002-145618/`（含 07:51 的 DEBUG 日志目录 `2026-10-29-07-51-56-448457-MEC-MB002-139761`）、探针 `~/usv_diag_runs/20261029_075209_raw`、`20261029_080258_raw`、`stall_20261029_084008`、`stall_20261029_090219` |
| 结论状态 | **已确证**（单变量恢复实验闭环）；map_server 竞态**已确证并已加兜底** |

---

## 2. 现象

1. 地图工具下发任务后，船**一步不动**，`/cmd_vel_nav` 一个包都没有。
2. 等 58 秒后 `nav_status` 报 `CTRL_STUCK`，再过 30 秒任务被取消。
3. Nav2 日志每秒刷一条 `[controller_server]: [follow_path] [ActionServer] Aborting handle.`，但**没有任何 ERROR**、没有 `Failed to make progress`。
4. 同一个进程里，任务**要么全好、要么全坏**：8:04 和 8:42 两次任务全卡死；重启后的任务正常。
5. 重启导航栈后，地图工具**一直不出地图**（另一个独立问题，见 §4.2）。

---

## 3. 排查路径与证据

### 3.1 先排除网络 / UDP（负结果，别再走这条路）

| 假设 | 核查 | 结论 |
|---|---|---|
| 内核 UDP 缓冲太小 | 空闲 8 s 采样：全局 `RcvbufErrors` 增量 **0**，`ss -uanpm` 每个 socket `skmem d=0`；`/dev/shm` 只用 18 MB / 7.7 GB | **否定** |
| MQTT 再订阅一份 raw 图像挤爆 DDS | `mqtt_bridgev2` **没在跑**（nav-only 栈）；`/camera_back_video/color/image_raw` 只有 1 发布 1 订阅 | **否定** |
| CPU 过载直接导致 | `RcvbufErrors` 只在 load 12–24 时出现，是"进程来不及 recv"的**结果**；同样负载下 07:57 的任务能正常出 17.9 Hz 速度 | 降级为背景因素 |

### 3.2 关键日志：151 次 abort 的归属

08:01 那次运行（`component_container_isolated_145835_1793260891820.log`）：

```
Received follow waypoint request with 1 waypoints      2 次
Begin navigating from ... to ...                        2 次
Received a goal, begin computing control effort         1 次   ← 控制器只进过 1 次循环
Passing new path to controller                          0 次   ← 循环体里的 INFO 一次都没打
Aborting handle                                         151 次
Failed to make progress                                 0 次
Current goal was not completed successfully             0 次
Action server failed while executing                    0 次
```

- 151 次 abort 从 `1793261051.773` 到 `1793261237.937`，间隔**恒定 1.07 秒**。
- 结尾：`[ERROR] ... Failed to get result for follow_path in node halt!`

### 3.3 源码定位：`Aborting handle` 到底谁打的

`/opt/ros/humble/include/nav2_util/simple_action_server.hpp`：

```cpp
// accept_pending_goal()：新 Goal 顶掉旧 Goal
if (is_active(current_handle_) && current_handle_ != pending_handle_) {
  debug_msg("Cancelling the previous goal");
  current_handle_->abort(empty_result());      // ← 只有 DEBUG，不是 WARN
}

// terminate()：WARN 只在这里
if (is_active(handle)) {
  if (handle->is_canceling()) { info_msg("Client requested to cancel..."); handle->canceled(result); }
  else { warn_msg("Aborting handle."); handle->abort(result); }
}
```

`terminate()` 的调用点：`terminate_current()` / `terminate_all()` / `work()` 的
"Current goal was not completed successfully" / `handle_accepted()` 里 **pending 槽被占用**。

日志计数把前三个都排除了（0 次），所以 151 条 WARN 只能来自
**pending 槽被下一个新目标顶掉** —— 前提是控制器从来不去消费 pending 目标。

DEBUG 那次（07:51）直接拍到这段：

```
[1793260683.875] ComputePathToPose RUNNING -> SUCCESS
[1793260683.876] RateController RUNNING -> SUCCESS
[1793260683.876] rcl_action: Sending action goal request      ← 新一轮 FollowPath
[controller_server]: Receiving a new goal
[controller_server]: An older goal is active, moving the new goal to a pending slot.
```

7 秒内 5 次；同期 `/plan` 稳定 1 Hz（BT 每秒重规划是**正常**的）。

### 3.4 控制器卡在哪：`controller_server.cpp` 的唯一静默死等

```cpp
while (rclcpp::ok()) {
  if (action_server_->is_cancel_requested()) { ... }
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) { r.sleep(); }   // ← 无超时、无日志
  updateGlobalPath();                                  // ← preempt 只在这里 accept
  computeAndPublishVelocity();
  if (isGoalReached()) break;
  loop_rate.sleep();
}
```

### 3.5 探针实证（`usv_stall_probe.py`）

08:42:32 下发任务（`maptool_20261029084232`），08:44:07 被取消：

| 信号 | 观测 |
|---|---|
| `/cmd_vel_nav` | 全程 **0**（对照：09:03 恢复后 21 Hz） |
| `/local_costmap/published_footprint` | **稳定 5 Hz** —— 代价图更新线程活得好好的 |
| 新线程 tid **162631** | 08:42:39 出现、08:44:07 消失；**197–204 次上下文切换/秒，持续 87 秒**，全程 S 态 |
| 对照线程 tid 162630 | 稳定 **20 次/秒** = `waypoint_follower` 的 `loop_rate: 20`，与日志里 20 Hz 的 `Processing waypoint 0...` 对得上 |

200 次/秒 = 100 Hz × 2，正是 `rclcpp::Rate r(100)` 的空转节奏；该线程就是
`SimpleActionServer` 用 `std::async` 起的 `computeControl()`。

> 注意：该线程 `wchan` 采样一直是 `futex_wait_queue_me` 而不是 `hrtimer_nanosleep`，
> 属于"高可信旁证"，还不是调用栈级证明。船上 `ptrace_scope=1`，非 root 附加不到容器。

### 3.6 根因链（源码级）

```
清局部代价图（mission_bridge 任务开始 / BT Recovery）
  → Costmap2DROS::resetLayers() → 每个插件 reset()
    → InflationLayer::reset() { matchSize(); current_ = false; }      // Humble 版
      （上游 main 分支已改成 { matchSize(); setCurrent(false); need_reinflation_ = true; }）
  → 本地图只有 voxel_layer(enabled=false) + inflation_layer，没有任何插件产生更新边界
    → LayeredCostmap::updateMap() 里
         if (xn < x0 || yn < y0) { return; }        // 边界退化，提前返回
       → InflationLayer::updateCosts() 永不执行 → current_ 永远回不到 true
  → controller_server 卡在 while (!costmap_ros_->isCurrent()) { r.sleep(); }（100 Hz 空转）
  → 不输出 /cmd_vel_nav；BT 每秒重规划的新 FollowPath 目标堆进 pending 槽互相顶掉
    → 每秒一条 WARN "Aborting handle"
  → 60 s 后 nav_status CTRL_STUCK → mission_bridge 30 s 后取消任务
```

**为什么"时好时坏"**：栈启动后、第一次清图之前是好的；清过一次就废了，直到重启。
- 07:51 那个进程：任务（07:57:52）之前**没有**清图记录 → 正常（`/cmd_vel_nav` 17.9 Hz）
- 08:01 那个进程：08:02:45 清过一次局部图 → 之后 08:04 / 08:42 两次任务全卡死

### 3.7 单变量恢复实验（闭环）

09:02 只动一个参数：

```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius   # 5.0
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 5.01
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 5.0
```

参数回调会置 `need_reinflation_ = true` → 下一轮 `updateBounds()` 给出整图边界 →
`updateCosts()` 执行 → `current_ = true`。

| 指标 | 恢复前（08:42） | 恢复后（09:03） |
|---|---|---|
| `/cmd_vel_nav` | 0（87 秒 0 个包） | **20–21 Hz** |
| `Passing new path to controller` | 0 | 16 次（每次重规划一条） |
| 新增 `Aborting handle` | 85 次 | **0 次** |
| `nav_phase` / `controller_status` | 60 s 后 STUCK | TRACKING / OK |
| 实际位置 | 不动 | 40 秒移动约 10 m |
| 任务结局 | 被取消 | **`Reached the goal!`（09:04:34）** |

两次任务之间唯一变化就是这个参数，因果闭环。

---

## 4. 第二个问题：map_server 启动竞态（重启后地图工具不出图）

### 4.1 现象与证据

09:13 重启后地图工具一直没有地图。核查：

```
ros2 topic info /map            → Publisher count: 0（3 个订阅者在等）
ros2 lifecycle get /map_server  → unconfigured [1]
ros2 node list | grep lifecycle → 只有 /lifecycle_manager_navigation，缺 /lifecycle_manager_localization
global_costmap 日志             → 每 5 秒一条 "Robot is out of bounds of the costmap!"
```

容器日志：

```
[INFO] [1793265230.658] nav2_container: Instantiate class: ...MapServer
[INFO] [1793265231.146] map_server: Creating
[WARN] [1793265231.262] nav2_container.rclcpp: failed to send response to
       /nav2_container/_container/load_node (timeout)
```

`nav2_boat.launch.py` 原本用**一次** `LoadComposableNodes` 装两个组件
（`map_server` + `lifecycle_manager_localization`）。第一个的 `load_node` 服务响应超时后，
**第二个组件被静默丢弃** → 地图服务器起来了但没人 configure/activate → `/map` 没有发布者 →
`global_costmap` 静态层为空 → 全局规划不可用。

扫描最近 33 次启动的容器日志：**32 次正常，1 次缺失**（约 3%）。

**影响**：不只是"地图不显示"，而是初始化后导航直接不可用，现象容易被误判成"船不动"。

### 4.2 临时恢复（两行命令）

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

页面需刷新（`/map` 是 transient_local 锁存话题）。

---

## 5. 已做修改

### 5.1 船不动（A + B + C，全部在我们自己的节点，不改 nav2）

| 文件 | 改动 |
|---|---|
| `workspace_nav/costmap_reinflate.py`（**新增**） | 共用恢复模块：`get inflation_radius → set +0.01 → set 回原值`，全异步回调，带超时与 `last_result` |
| `config/mission_stack.xingfei.yaml` | **A**：`clear_costmap_on_mission_start: false`（当前无观测层，清图无意义；注释写明恢复条件） |
| `workspace_nav/mission_bridge.py` | **B**：新参数 `reinflate_after_costmap_clear`（默认 true）；`_clear_local_costmap()` 清图成功回调里自动触发一次重膨胀 |
| `workspace_nav/nav_status_aggregator.py` | **C**：订阅 `/follow_path/_action/status`，用 `STATUS_EXECUTING`（真实 Nav2 状态，不靠 TRACKING 推断）判定控制器在跑；`_update_auto_recovery()` 每任务最多一次自愈 |

**C 的触发条件**：

```
任务 RUNNING
且 FollowPath 目标处于 STATUS_EXECUTING
且 now - max(FollowPath 开始, 最后一条 cmd_vel) > 5 s（auto_recover_silence_sec）
且 本任务未触发过；恢复器不忙
```

约束：不碰 `CTRL_STUCK` 的计时与告警（原有 60 s + 30 s 流程不变）；恢复后 10 s 仍无速度 →
记一条 ERROR 并停止，不反复尝试；成功 → INFO + 清除 `CTRL_OUTPUT_SILENT` 告警。
总开关 `auto_recover_costmap_current`。

### 5.2 map_server 竞态

| 文件 | 改动 |
|---|---|
| `launch/nav2_boat.launch.py` | 拆成两次 `LoadComposableNodes`（`load_map_server` / `load_lifecycle_manager_map`），两件套用 `TimerAction(period=3.0)` 延迟装载；新增 `map_server_guard` 进程（`IfCondition(autostart)`） |
| `workspace_nav/map_server_guard.py`（**新增**） | 启动 20 s 后查 `/map_server` 状态：active 就退出；连续两次检测到 unconfigured/inactive 才动手 configure→activate，最多 3 次，失败打 ERROR + 退出码 1 |

**注意**：两套启动方式共用 `nav2_boat.launch.py` —— 开机自启的
`self_cleaning_boat_xingfei.launch.py:86` include 的就是它，所以两套一起生效。

### 5.3 部署位置

| 位置 | 说明 |
|---|---|
| 船端 `~/usv-test/install/arm/workspace_nav/{lib/python3.10/site-packages,share/workspace_nav}/{workspace_nav,launch,share}` | 已覆盖；备份 `*.bak.20261029_0910`（Python）、`.bak.20261029_0931`（launch） |
| `/home/ght/usv/src/USV_NAV/workspace_nav/` | 已同步（注意 usv 仓的 `mission_bridge.py` 比船端旧，已按"取较新一方"覆盖） |
| 本地导航仓 | 已更新，**未提交** |

---

## 6. 工具与使用方式

### 6.1 卡死现场探针 `usv_stall_probe.py`

船端 `~/usv_dds_diag/usv_stall_probe.py`（本仓 `scripts/usv_dds_diag/`）。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 -u ~/usv_dds_diag/usv_stall_probe.py --out ~/usv_diag_runs/stall_$(date +%H%M%S) --max-sec 900
```

每秒一行 JSON，两类记录：

| kind | 内容 |
|---|---|
| `topics` | `local_costmap_update_hz`（`/local_costmap/published_footprint`，正常 ~5 Hz）、`cmd_vel_nav_hz` |
| `threads` | nav2 容器每个线程 `[tid, state, wchan, 自愿上下文切换/秒]` |

判读：

| 现象 | 含义 |
|---|---|
| 某线程持续 ~200 次/秒、S 态 | 在 `while (!isCurrent()) r.sleep()` 里 100 Hz 空转 |
| `local_costmap_update_hz` 掉到 0 | 代价图更新线程死了（更严重，另一种故障） |
| 代价图 5 Hz 正常但 `cmd_vel_nav=0` | 就是本文的 `current_` 卡死 |

### 6.2 现场救活（不用重启，可逆）

```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 5.01
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 5.0
```

恢复原值即可，行为等价；每次全图重膨胀 240×240 只需毫秒级。

### 6.3 确认恢复

```bash
ros2 topic hz /cmd_vel_nav                          # 应 ~20 Hz
grep -c "Passing new path to controller" <容器日志>  # 约 1 次/秒
grep -c "Aborting handle" <容器日志>                 # 不再增长
ros2 topic echo --once /nav_status                  # task RUNNING / phase TRACKING / ctrl OK
```

### 6.4 地图不出时的三步检查

```bash
ros2 topic info /map                             # Publisher count 应为 1
ros2 lifecycle get /map_server                   # 应为 active [3]
ros2 node list | grep lifecycle_manager          # 应有 lifecycle_manager_localization
```

---

## 7. 待验证 / 下一步

1. **重启后回归**（新代码生效需要重启导航栈）：
   - 普通任务：日志里 `costmap_reinflate` 应**一条都没有**（自愈不误触发）；
   - mission_bridge 清图：应出现「局部代价图已清空」→「已强制重膨胀…current 状态恢复」；
   - BT Recovery / 地图工具清图：5 秒后应出现「FollowPath 执行中但 /cmd_vel_nav 静默…触发重膨胀」；
   - 连续两个任务：第二个任务不重复自愈。
2. **`map_server_guard` 的 `unconfigured` 路径**只做了逻辑核对，实测走的是 `inactive → activate`
   （手动 deactivate 触发）。今天的真实场景是 `unconfigured → configure → activate`，
   逻辑同一分支，但建议下次重启时留意 20 s 后是否有兜底日志。
3. **雷达常开后**：`voxel_layer.enabled=true` 时观测层会产生边界，该缺陷会自然消失；
   届时可把 `clear_costmap_on_mission_start` 改回 true（前提是 `reinflate_after_costmap_clear` 已验证）。
4. **仍未解释的小尾巴**：07:57 那次任务为什么没有触发 mission_bridge 的「任务开始清图」
   （mission_bridge 日志只打到 tmux，没有落盘，查不到）。不影响结论。
5. **栈级证明缺口**：`ptrace_scope=1` 导致非 root 无法 gdb 附加容器；若以后需要调用栈级证据，
   需用 sudo 在卡死瞬间 `gdb -p <pid> -batch -ex "thread apply all bt"`。
6. 本次没有再碰 UDP 缓冲 / BT Action 超时 —— 单变量实验已证明它们不是这条故障链上的变量。
