# 导航 Service 接口 — 本地测试说明

> 用于验证**竞赛上层**使用的四个 Service 与反馈 Topic 是否正常。  
> 地面站 GCS 走 Topic（`/waypoint` 等），**不在此测试范围内**。

---

## 1. 前置条件

```bash
source /opt/ros/humble/setup.bash
cd /home/ght/wuxihik_navigation
colcon build --packages-select m_common workspace_nav
source install/setup.bash
```

---

## 2. 一键 CLI 测试（推荐）

启动最小依赖栈（静态 TF + 模拟 FollowWaypoints + 假 odom + mission_bridge + aggregator），依次调用四个 Service：

```bash
bash test/run_cli_service_test.sh
```

查看完整日志：

```bash
cat /tmp/nav_svc_test.log
```

**预期**：

| 步骤 | 预期 |
|------|------|
| `send_waypoints` | `success=True`，状态 → `RUNNING` |
| `set_pause` pause | 任务进行中 `success=True` → `PAUSED` |
| `set_pause` resume | `success=True` → `RUNNING` |
| `emergency_stop` | `success=True` → `EMERGENCY` |
| `cancel_mission` | `success=True` → `IDLE` |

> 若 `send_waypoints` 后立即 `pause` 失败，可能是 mock 航点太快已 `COMPLETED`；用 `test_pause_resume.py` 单独测暂停。

---

## 3. Python 测试脚本

| 脚本 | 用途 |
|------|------|
| `test_nav_services.py` | 完整用例（需先起栈，见下） |
| `quick_test.py` | 快速冒烟（WAITING_SYSTEM / EMERGENCY / cancel） |
| `test_pause_resume.py` | 专注 pause / resume |
| `run_service_integration_test.sh` | 起栈 + `test_nav_services.py` |

**手动起栈后跑完整测试**：

```bash
# 终端 1 — 依赖
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
python3 test/mock_follow_waypoints_server.py
python3 test/fake_odom_pub.py

# 终端 2 — 导航
ros2 run workspace_nav mission_bridge --ros-args \
  -p map_yaml_path:=$(pwd)/src/YILDIZ-USV/workspace_nav/config/map.yaml
ros2 run workspace_nav nav_status_aggregator

# 终端 3 — 测试
python3 test/test_nav_services.py
python3 test/quick_test.py
python3 test/test_pause_resume.py
```

---

## 4. 单条 ros2 命令示例

```bash
ros2 service call /mission_bridge/send_waypoints m_common/srv/SendWaypoints \
  "{waypoints: [{header: {frame_id: map}, pose: {position: {x: 10.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}], mission_id: 'm001', command_id: 'c1'}"

ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: true}"
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: false}"
ros2 service call /mission_bridge/emergency_stop m_common/srv/EmergencyStop "{}"
ros2 service call /mission_bridge/cancel_mission m_common/srv/CancelMission "{}"

# 反馈（上层必订 nav_status 用 TRANSIENT_LOCAL + RELIABLE）
ros2 topic echo /nav_status --once
ros2 topic echo /task_event --once
```

---

## 5. 实船 / 仿真联调

CLI 测试使用 **mock action**，不能代替实船。实船需：

- Nav2 全栈 + `follow_waypoints` action
- TF `map`→`base_link`、里程计、GPS（aggregator 用）
- `ros2 launch workspace_nav mission_bridge.launch.py`（或船端 bringup）

上层 Service 接口与 mock 测试**相同**。

---

## 6. 辅助文件

| 文件 | 说明 |
|------|------|
| `mock_follow_waypoints_server.py` | 最小 FollowWaypoints action server |
| `fake_odom_pub.py` | 发布 `/odometry/filtered` |
| `run_cli_service_test.sh` | 一键 CLI 集成测试 |
| `run_service_integration_test.sh` | 起栈 + Python 全量测试 |

---

## 7. 发给上层厂商的文件

见 [`docs/UPPER_LAYER_DELIVERY.md`](../docs/UPPER_LAYER_DELIVERY.md)（对接包清单，**不要**把本 `test/` 目录发给上层）。
