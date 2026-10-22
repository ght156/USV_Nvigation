# USV 归港第二轮源码审查报告（更新版对比）

> 对比范围：`dock_mission.zip` ↔ `dock_mission(1).zip`、`usv_docking.zip` ↔ `usv_docking(1).zip`。仅针对两包，不包括 Nav2、mission_bridge、ArduPilot 速度桥及实船 rosbag。未修改任何源码、参数或船端配置。行号以本次新版源码为准。

## 一、更新概览

两个包共 8 个非缓存文件变化：

- `dock_mission`：`config/dock_mission.yaml`、`dock_mission/dock_mission_node.py`、`dock_mission/dock_task_api.py`、`test/test_dock_mission_fsm.py`。
- `usv_docking`：`config/docking.yaml`、`usv_docking/docking_fsm.py`、`usv_docking/docking_motion_controller.py`、`test/test_controller_signs.py`。

已经落实的改进：

1. `dock_mission_node.py:713-774`：SETTLE 新增里程计速度接收、新鲜度门、持续停稳判断和超时处理。
2. `docking_motion_controller.py:283-342`：拒绝非有限位置，新增 pose/source 消息新鲜度检测。
3. `docking_fsm.py:1309-1339`：开爪和松爪 Action 结果超时分别配置。
4. `dock_mission_node.py:64-66,352-387`：移除已经没有实际执行意义的 `require_camera` 指令入口，只保留 status 中的兼容字段；状态上报 `camera_ready=True` 不应作为真实相机就绪信号。
5. 补充停稳与断流的测试用例，但**未覆盖控制输出与重试目标下发的关键闭环**。

## 二、需要优先处理的问题

### P0-1 自动重新预泊不会实际发起下一次 Nav2 目标（控制流可确认）

- 文件：`dock_mission/dock_mission/dock_mission_node.py:635-654,667-709,511-538`。
- `_on_staging_failed()` 在非 `dock_only` 模式下，只执行 `_transition(MissionState.NAV_TO_STAGING, reason)`；`_tick()` 的 `NAV_TO_STAGING` 分支只有已在飞发送请求的超时处理，没有再次调用 `_start_nav_to_staging()`。
- 新版 `test/test_dock_mission_fsm.py:164-176` 与 `:238-255` 只断言状态回到 `NAV_TO_STAGING`、重试计数增加/取消请求次数，并没有断言 `send_staging_async()` 再次被调用。
- 结果：预泊失败、SETTLE 超时或精靠泊要求重新预泊时，可能挂在 NAV_TO_STAGING，不会重新导航。
- **最小修复**：在 NAV_TO_STAGING tick 内，当 `_pending_nav=False` 且 `_staging_send_inflight=False` 时调一次 `_start_nav_to_staging()`；同时保留失败上限和发送超时保护。改动后应考虑异步回调迟到的问题，给每次发送加 `attempt_id` 过滤。补单测断言第一次失败后的下一 tick 实际调用 `send_staging_async`，以及重试耗尽不再调用。

### P0-2 位姿失效只有速度目标清零，没有立即指令清零

- 文件：`usv_docking/usv_docking/docking_motion_controller.py:599-608,646-681`。
- 新增 `_pose_valid()` 后，`not _pose_valid()` 的运动态分支只是 `pass`：目标设为零，但最终 `_v_cmd/_w_cmd` 仍经加速度斜坡收敛。只有 `_safety_stop=True` 分支直接把指令清零。
- 软件指令示例：`v=0.45m/s`、`max_linear_accel=0.30m/s²`，从非零指令减至零需约 1.5s；这是指令限幅推导，不是船体实际刹停耗时。
- **最小修复**：为输入失效建立单一 `must_stop_now` 判断；在非保持态且输入不可用时立即将 `_v_cmd=_w_cmd=0`，发布零速度，绕过软斜坡，并记录原因。ROS 命令清零不能代替推进器实际刹车/漂移安全验证。
- 单测要预先把 `_v_cmd` 设为非零后触发失效；当前测试在失效前主动把 `_v_cmd/_w_cmd` 设为 0，因此未验证斜坡仍下发非零指令的问题（`usv_docking/test/test_controller_signs.py` 新增 `test_stale_pose_msg_holds_motion`）。

### P0-3 FSM 目标模式断流时，控制器没有本地看门狗

- 文件：`usv_docking/usv_docking/docking_motion_controller.py:226-236,248-259,279-282,580-645`。
- `_mode_cb()` 只更新字符串 `_mode`，没有更新时间戳。若 `docking_fsm` 停止运行，但估计器仍在持续发布有效位置、safety 节点也异常或其信号停留在 false，控制器会持续按最后一个 APPROACH/BACK_IN/FINAL_DOCK 模式给速度。
- **最小修复**：将 FSM 模式接收时间并入控制器的唯一输入有效性校验；运动态模式 > 指定时长（可按 10Hz 状态机频率设数百毫秒级，需实测余量）没有更新则立即零速。不要另建独立的复杂安全模块。
- 可进一步审查 `/docking/safety_stop` 的失联语义；不要把上次收到的 false 无限期当作安全授权。

### P1-4 新增 SETTLE 的“持续停稳”仍有漏检窗口

- 文件：`dock_mission/dock_mission/dock_mission_node.py:713-774`。
- `_settle_still_hold += dt` 依据定时器循环累计，数据只需“接收时间距离现在不超过 1 秒”，而 `settle_still_hold_sec=0.5s`。因此**即便在 0.5 秒内没有新速度帧**，单帧小速度也可能连续经过 3 次 tick 被算作持续停稳。
- 只检查线速度 `hypot(vx,vy)`，未检查 `angular.z`；船原地偏航时也可能满足放行条件。
- **最小修复**：速度样本必须连续更新（并限制最大样本间隔）；引入现有 Odometry 的 `angular.z` 停稳判断，增加一个明确的角速度阈值即可，不必新建状态。不宜只依赖接收时间，传感器时间戳/时钟一致性也应核实。

### P1-5 FINAL_DOCK 压紧时间并不总是实际经过时间

- 文件：`usv_docking/usv_docking/docking_fsm.py:1279-1296`。
- `_final_push_start` 保存了时间，但 `_final_push_elapsed += dt` 与 `FINAL_PUSH_TIMEOUT` 判断仅在满足 x/y 小窗口时执行。离开窗口时累计计时暂停，且离开窗口期间不会判断压紧最长时长，退回 `final_dock_timeout_sec=60s` 的粗超时。
- **最小修复**：首次进入压紧窗口时锁存时刻，后续用 `now - _final_push_start` 的单调时间计算最大压紧时长；是否允许继续倒退仍由控制器几何限制决定。要有退出窗口的明确安全行为。

### P1-6 FSM 缺乏与控制器一致的消息有效性检查

- 文件：`usv_docking/usv_docking/docking_fsm.py:466-485,1064-1080,1414-1434`。
- FSM 的 `_dock_pose_cb`、`_odom_cb` 没有数值有效性/速度源龄期检查。控制器虽然丢掉 NaN/Inf，但 FSM 仍可缓存该值。`_staging_reached` 和 `_route_after_reacquire` 只检查速度值非 None，不检查是否来自新鲜速度帧。
- **最小修复**：FSM 输入回调也拒绝非有限值，对速度做本地超时检查；在到达/转状态条件内复用 `_odom_speed_ok()`。此处属于判断一致性，而不是再加一套控制器。

### P1-7 任务事件匹配过宽，重试的旧事件可能污染新任务

- 文件：`dock_mission/dock_mission/dock_mission_node.py:557-574`。
- `if mid and mid != self._mission_id and "dock" not in mid: return` 的逻辑允许**任何包含字符串 `dock` 的不同任务 ID**通过过滤；还允许不带 ID 的 legacy 文本。重试仍使用同一个 `_mission_id`，只把 `command_id` 写为 `retryN`。
- **建议**：和 mission_bridge 明确事件字段后，对 JSON 事件进行严格 `mission_id + attempt/command_id` 关联；兼容 legacy 无 ID 事件要限定独立兼容模式，避免出现非本次任务事件提前触发 SETTLE 或错误重试。

### P1-8 运行时切换里程计只覆盖了 docking_fsm

- 文件：`usv_docking/launch/docking.launch.py:33-41` 与 `dock_mission/launch/dock_mission.launch.py:69-75`。
- `odom_topic` launch 参数只改 `docking_fsm.odom_speed_topic`；新版 `dock_mission` 也订阅速度，但另一个 launch 没有同名 `odom_topic` 参数。切换 SLAM 或其他定位源时，FSM 与 SETTLE 可能读不同里程计；没有 MAVROS 默认话题时甚至会 15s 后不断重试。
- **建议**：部署脚本统一向两端传同一个已校验的 Odometry 话题，或者把两者放进部署 profile。并核对 ROS 实际 topic 类型、QoS compatible、twist 坐标系与速度单位。

### P1-9 实船输出默认值与 launch 文件顶部说明冲突

- 文件：`usv_docking/launch/docking.launch.py:10-11,85-89`；`usv_docking/config/docking.yaml:293-296`。
- 文件顶部仍说默认 `test_only=true`、只发送测试话题，但 launch 实际参数默认是 `false`，而 YAML 也指定 `false`：直接启动将发真实 `/cmd_vel_nav`。
- **建议**：至少同步文档、终端日志和启动命令说明。调试环境建议显式 `test_only:=true`，实船接管须显式核查控制权限；不要仅凭 README 以为是测试输出。

## 三、减法优化建议（暂时不动这些安全模块）

- 保留 `APPROACH_ENTRY / ALIGN_ENTRY / BACK_IN / FINAL_DOCK` 四个职责明确的状态；先不引入多边形船坞包络或 MPC。
- 给一个现有控制器输入检查函数整合 `pose_valid + mode_fresh + safety_stop`，确保所有运动模式都由同一个“能否运动”的入口把关；原状态机只负责何时到点、失败、撤离及夹爪判定。
- 把重复的线速度与角速度测量/新鲜度小函数复用逻辑思路（可以分别实现在两个 ROS 节点，不必新增通信层），减少分散数值常量。
- `docking_fsm.py` 中 `_approach_stuck=0.0` 在初始化里出现两次（约第 405–407 行），属于安全可清理的重复赋值。
- `docked_yaw_tol_deg`、`docked_hold_sec`、`reacquire_search_yaw_deg` 被当前项目测试标记为“为旧 GUI 兼容保留的死参数”：确认没有外部 GUI 使用后才删；避免只为减少行数就破坏接口。
- 控制器、状态机共享的 `staging_x`、`final_target_x`、门槛值由 `test_param_consistency.py` 校验，**这套测试应该保留**，不推荐为去重把 ROS 节点硬性耦合成一个文件。

## 四、必须补的测试

1. Nav2 `TASK_FAILED` 后下一 tick 确实二次调用 `send_staging_async`；SETTLE_TIMEOUT 也能重发；到达 `staging_retry_max` 停止重发。
2. 旧请求延迟成功/失败回调，不应污染新一轮 attempt；其他含 `dock` 字样的任务事件不得被误接受。
3. 位姿在控制器速度非零时断流/NaN，**第一帧**指令必须归零；原单测先把速度清零，无法覆盖此缺陷。
4. FSM 模式流断掉而位姿依然新鲜，控制器仍应零速。
5. SETTLE 只有一帧零速后停止更新，不能凑足连续停稳时间；角速度未稳定也不能通过。
6. FINAL_PUSH 已开始后位置短暂漂出窗口，15s 实际超时仍生效。
7. launch 切换 `odom_topic` 后，mission 与 FSM 使用相同已存在的 Odometry 话题。

## 五、验证记录与结论

- `python -m compileall -q dock_mission/dock_mission usv_docking/usv_docking`：**通过**（仅验证 Python 语法）。
- `pytest -q usv_docking/test/test_dead_params.py usv_docking/test/test_param_consistency.py`：**6 passed**，属于无需 ROS 的参数静态检查。
- 需要 ROS 的 `test_param_defaults.py` 等：无法在当前环境收集，`ModuleNotFoundError: No module named 'rclpy'`。不能据此声称实船逻辑已测试通过。
- 尚未进行实船、rosbag 回放或推进器运动学验证。

**建议先修 P0-1 / P0-2 / P0-3，再修 P1-4 / P1-5；等闭环测试完成后才考虑继续调参。** 最好维持四节点结构和现有物理状态划分，不用再增加几何包络模型。窄坞防碰撞最终仍依赖可靠的相机标定、近距定位及实际停车能力，不可将纯软件零速理解为立即停止船体运动。
