# USV 归港源码审查（第一轮，基于用户上传的两个 ZIP）

## 范围与局限
本次读取 `dock_mission.zip`、`usv_docking.zip` 中的 Python、YAML 源文件；非整个 USV_NX 工作区，缺 Nav2/mission_bridge/速度桥和飞控实现、实船 rosbag。未对运行中船只作任何修改。当前环境无 ROS2 `rclpy`，pytest 因环境依赖未能收集测试。

## 已由源码确认的设计缺口

1. `dock_mission/dock_mission/dock_mission_node.py` `_tick_settle`：`MissionState.SETTLE` 仅累计 `dt` 达到 `settle_sec`（配置默认 2.5 秒），立即转 `DOCK_HANDOFF`；不检查实际速度、角速度、位置有效性。`_tick_handoff` 随后切换 authority、发布 `/dock/start`。说明“停稳”等待只是延时，并非停稳实测确认。
2. `usv_docking/usv_docking/docking_motion_controller.py`：`_pose_valid()` 只检查 `pose_source != SRC_INVALID` 与 `_pose_x is not None`，未检查 pose 消息的本地接收新鲜度；运动输出虽然依赖估计器的 source，但若上游数据流卡住且 source 未刷新，可能继续依据旧缓存控制。需要在控制器本地加消息超时门，尤其对网络/进程异常。
3. `usv_docking/usv_docking/docking_fsm.py`：`_trans_approach_entry` 允许推算源在 `approach_tag_loss_grace_sec` 内延续；`_trans_align_entry` 允许 `align_tag_loss_grace_sec` 内延续；`_trans_reacquire_tag` 还维护推算驾驶预算。安全意图合理，但有多套时间门，应核实优先级和每段运动模式，避免控制器/FSM对预测可否行驶理解不一致。
4. `docking_fsm.py` `_trans_back_in` 和 `_trans_final_dock` 使用横偏/航向走廊判断；`docking_motion_controller.py` `_compute_back_in` 再次以 `back_in_gate2_y`/`back_in_gate2_yaw_deg` 限制倒船。安全约束是必要的，不宜删除，建议集中相同常量并做一致性测试。
5. `docking_fsm.py` ` _trans_wait_dock_io`：开爪和松爪两个等待状态使用同一个 `dock_open_timeout_sec` 参数。若设备开/松动作时间不同，难以独立调参；非必然 bug，仅是语义耦合。
6. `dock_mission` 在接受任务时，注释明确 `require_camera` 只透传事件，而 `_camera_ready` 恒为 True；调用方如果把 `require_camera=true` 当作真正的相机就绪门，会产生错误信心。

## 最小化重构顺序（不增加几何包络）

P0：先核对部署下的完整速度控制链，对 `/cmd_vel_nav` 的发布者做记录；确认真实仲裁器和桥的看门狗行为。禁止仅凭此仓库推断控制源必然冲突。

P0：`SETTLE` 增加“实测停稳才交接”的可选开关和带超时的保守处理；应使用具有新鲜度验证的实际速度与角速度来源，并明确缺测时不启动 Docking。不要仅仅把 2.5 秒加长。

P0：给控制器加 pose/source 接收超时与非法数值检查；断流时零速。注意退出/撤离阶段必须结合可观测性与人工接管策略，不可强行运动。

P1：保留 `APPROACH_ENTRY`、`ALIGN_ENTRY`、`BACK_IN`、`FINAL_DOCK` 等物理职责不同的状态，不宜仅为减少状态数而删状态。统一判定函数和配置阈值校验，避免重复阈值相悖；对 APPROACH↔ALIGN、APPROACH↔REACQUIRE 设计状态转换测试。

P1：对每轮实船数据记录状态、控制 mode、预测龄期、目标姿态、实际速度、cmd_vel、失败原因；先区分定位波动与动力学迟滞，再调 Kp/漏斗参数。

## 必测用例
- Nav2到预泊点，但实际速度不为零：不得启动 docking。
- Nav2到达但 odom 缺失/超时：不得开始倒船。
- 控制器位姿话题冻结、source 长时间保持 VISION：必须安全停机。
- APPROACH/ALIGN 视觉闪断：状态转换有界，停止条件一致。
- BACK_IN/FINAL_DOCK 走廊超限：控制器停止危险运动、FSM给出合理中止。
- 夹爪失败、退出位置无法确认：不得误报已驶出或成功。

## 测试环境
尝试运行两个包的 `pytest -q`：`ModuleNotFoundError: No module named 'rclpy'`，测试未运行；不能视为通过或失败。

## 结论
先修交接和上游消息断流保护，减少重试/姿态/时序条件的交叉影响；不要新建船坞和船体实时多边形包络。对五厘米级侧向间隙，仍需实体尺寸、可靠的近场感知/定位精度与低速实船验证，否则无法保证防碰撞。
