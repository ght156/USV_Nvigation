# dock_mission 测试

## 单元 / 组件测试（pytest）

```bash
# 推荐：一键脚本（自动 source ROS + workspace）
./test/run_tests.sh

# 或手动
source /opt/ros/humble/setup.bash
cd /home/ght/USV_NAV && colcon build --merge-install --packages-select dock_mission
source install/setup.bash
cd src/dock_mission && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v --ignore=test/integration
```

当前 **48 项**测试（全部通过；3 个仿真仓遗留的过时断言已于 2026-09-02 对齐现行行为），覆盖：

| 模块 | 文件 | 要点 |
|------|------|------|
| 泊位加载 | `test_bay_loader.py` | YAML 解析、缺 key、默认值 |
| GNSS 预泊 | `test_gnss_staging.py` | map/GNSS 互转、`make_staging_pose`、`make_staging_waypoint`（实船 WGS84 契约） |
| dock_enu | `test_dock_enu.py` | 坐标变换、roundtrip |
| 入口验收 | `test_entry_validator.py` | RTK / 走廊 / tag / 拒绝分支 |
| 任务事件 | `test_task_event.py` | JSON 与 legacy 字符串解析 |
| Nav2 切换 | `test_nav2_goal_checker.py` | docking/cruise GoalChecker |
| 归港 FSM | `test_dock_mission_fsm.py` | `/dock/home` → Nav staging → handoff |

## 可选 ROS2 冒烟（integration）

需**另开终端**先启动 `dock_mission_node`（或 launch），再运行：

```bash
# 终端 1（实船仓默认 profile=real / use_sim_time=false；本冒烟用默认值即可）
ros2 launch dock_mission dock_mission.launch.py

# 终端 2
./test/integration/test_dock_home_smoke.sh
```

脚本会启动 mock `send_waypoints`（实船 WGS84 `MissionWaypoint[]` 契约），发布 `/dock/home=true`，并等待 `/dock/mission_status`。
