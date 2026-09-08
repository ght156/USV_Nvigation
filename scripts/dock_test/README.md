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
