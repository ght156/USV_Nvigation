# dock_test — 归港栈数据流测试套件（无硬件）

不依赖真船/Nav2/AprilTag 的归港栈数据流回归：用假 TF/里程计 + mock mission_bridge 驱动完整链路。

## 文件

| 文件 | 作用 |
|------|------|
| `run_e2e.sh` | **一键编排归港全链路**：fake_env + mock mission_bridge + usv_docking + dock_mission 全拉起来，从 `/gcs_dock/command` 下发 one_click_dock，录事件/状态/cmd_vel |
| `run_dock_only.sh` | **仅精靠泊**：fake_env + usv_docking，`/dock/start` 直触发，录 BACK_IN 段 `/cmd_vel_nav` |
| `fake_env.py` | 假环境：广播 `odom→base_link`、`base_link→dock_frame`（脚本化轨迹，模拟船从坞外倒入坞内），及两路零速度 Odometry |
| `mock_mission_bridge.py` | mock 实船 WGS84 `SendWaypoints` 服务 + 延时回 `/task_event TASK_COMPLETED` |
| `mock_event_server.py` | mock 上层状态机：实现 `/dock_task/event` service 并回 ACK（嵌软未实现时的联调替代） |

## 用法

```bash
cd /home/ght/USV_NAV
colcon build --packages-select dock_mission usv_docking && source install/setup.bash
bash scripts/dock_test/run_e2e.sh            # 一键编排归港全链路（约 2 分钟，日志在 /tmp/dock_test/out/）
bash scripts/dock_test/run_dock_only.sh      # 仅精靠泊（约 40s）

预期结果：
- run_e2e：/dock_task/event_log 依次出现 DOCK_TASK_ACCEPTED → DOCK_STAGING_STARTED → DOCK_HANDOFF → DOCK_SUCCEEDED，
  mock_event_server 逐条回 ACK；/dock_task/event service 由 dock_mission 以 client 调用
- run_dock_only：BACK_IN 段 /cmd_vel_nav 有负向线速度（倒船）输出
```

注意：使用独立 ROS_DOMAIN_ID=42/43，与实船图（DOMAIN 5）隔离。

---

## 实船现场工具（2026-09-12 归港实测沉淀）

以下脚本在**船端（nx）**运行（scp 到 `/tmp` 或 `~` 直接用；船上先 `export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，source ROS + `~/usv_nav_ws/install/setup.bash`）。船端一键启动：`~/start_all.sh`（全栈）/ `~/start_dock_stack.sh`（归港栈）/ `~/record_dock_data.sh`（录制，~/usv_bags）。

| 文件 | 干什么用 | 用法 |
|------|---------|------|
| `motion_probe.py` | **运动能力开环测试**：阶梯发 cmd_vel（前进/倒退/转向各档 + 零速漂移本底），同步录 CSV | `python3 motion_probe.py /tmp/mp.csv`（约 90s，自动归零收尾） |
| `motion_bag_analyze.py` | 运动测试分析：cmd_vel 切段 × TF 真值位移，出达成率/方向/漂移表 | `python3 motion_bag_analyze.py <bag目录>` |
| `dock_cycle.py` | **出港→自动再归港全循环**：undock → 等视觉位姿进触发门 → 自动 dock_only → 盯到 DOCKED；ey 超限自动取消 | `python3 dock_cycle.py` |
| `dock_watch.py` | 归港监视+人肉安全兜底：触发 dock_only 后 4Hz 盯状态，\|ey\|>1.0 持续 1s 自动取消（APPROACH 段 safety 不覆盖） | `python3 dock_watch.py` |
| `dock_snap.py` | 归港状态快照：state/pose_source/tag_visible/dock_pose/safety 一次看全（绕开船上 ros2 CLI 卡死） | `python3 dock_snap.py` |
| `odom_jump_check.py` | 检查 odom->base_link TF 跳变（定位原点重置排查） | `python3 odom_jump_check.py [起始epoch] [结束epoch]` |
| `crash_review.py` | 事故复盘模板：状态切换 + 2s 间隔对照表（估计位姿/cmd_vel/原始视觉/GPS速度） | 改文件顶部 bag 路径和时间窗 |
| `set_params.py` / `set_undock.py` | 在线改参样例（rclpy SetParameters，船上 ros2 CLI 卡死时用） | 改文件里的参数名/值 |
| `cmd_vel_pub.py` | 持续发 cmd_vel N 秒后自动归零（手动挪船用） | `python3 cmd_vel_pub.py <vx> <wz> <秒>` |
| `drive_to_staging.py` | 按锚点闭环把船开到坞轴线上（备用，一般人工遥控摆位） | `python3 drive_to_staging.py [target_x=-7]` |

**触发门经验值**（有流 0.1~0.3m/s 环境）：起手 ex∈(-5,-1.5)、|ey|<0.6、eyaw<10° 成功率高；6.5m+偏轴 1.3m 起手撞过走廊。
