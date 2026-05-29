# 实船 Mission 迁移 — 测试方案与结果

> **用途**：验证从 `wuxihik_navigation` 同步到 `USV_NAV` 的 `mission_bridge` / `nav_status_aggregator` 在**实船参数与地图**下是否行为正确。  
> **性质**：本地 mock 测试（不替代实船 MAVROS + Nav2 全栈联调）。  
> **接口契约**：见 `docs/nav_task_interface.md` v4.1。

---

## 1. 迁移内容摘要

| 类型 | 实船路径 | 说明 |
|------|----------|------|
| 整包复制 | `src/m_common/` | Service 定义 |
| 整文件复制 | `workspace_nav/gps_map_conversion.py` | 坐标/JSON 解析 |
| 整文件复制 | `workspace_nav/mission_bridge.py` | GCS Topic + 上层 Service |
| 整文件复制 | `workspace_nav/nav_status_aggregator.py` | `/nav_status`、`/task_event` |
| 复制 + 改默认值 | `launch/mission_bridge.launch.py` | 实船 `use_sim_time`、MAVROS 话题、`map.yaml` |
| 新建 | `config/mission_stack.real_boat.yaml` | odom/GPS/cancel/debounce |
| 小改 | `launch/nav2_real_mavros.launch.py` | `enable_mission_bridge:=true`，与 Nav2 共用 `map:=` |
| 小改 | `setup.py` / `package.xml` | entry_point 与依赖 |

**未改**：`real_boat_bringup`、Nav2 参数 yaml、地图 pgm/yaml 内容、legacy 三节点源码。

---

## 2. 测试环境（Mock）

不启动 PX4 / MAVROS / 真 Nav2，仅模拟 mission 栈所需最小依赖：

```text
static_transform_publisher   map → base_link
mock_follow_waypoints_server follow_waypoints (action)
假 odom 发布器               /mavros/local_position/odom
mission_bridge.launch.py     实船 map.yaml + mission_stack.real_boat.yaml
nav_status_aggregator        同上 odom 话题
```

**被测工作区**：`USV_WS`（默认 `/home/ght/USV_NAV`）的 `install`，脚本位于仿真仓 `wuxihik_navigation/test/`。

---

## 3. 测试用例

| # | 路径 | 操作 | 预期 |
|---|------|------|------|
| 1 | 上层 Service | `send_waypoints` 2 点 | `success=true`，`/nav_status.task.state=RUNNING` |
| 2 | GCS Topic | `/waypoint` JSON + `explicit_replan:true` | `task_id=gcs_migrate`，debounce 后换线 |
| 3 | GCS Topic | `/gcs_mission/cancel` Empty | `task.state=IDLE` |
| 4 | 上层 Service | `send_waypoints` → `emergency_stop` | `EMERGENCY` |
| 5 | 上层 Service | `cancel_mission` | 清急停 → `IDLE` |

**不在本脚本范围**（见 `test/README.md` 其它脚本）：

- 仿真仓 `YILDIZ-USV` 的 Service 全量用例（`test_nav_services.py`）
- 地面站 HTTP → ROS 全链路
- 实船 OFFBOARD、Livox costmap、真 FollowWaypoints 规划

---

## 4. 如何运行

### 前置

```bash
# 实船工作区已编译
cd /home/ght/USV_NAV
source /opt/ros/humble/setup.bash
colcon build --packages-select m_common workspace_nav
source install/setup.bash
```

### 一键（推荐）

在仿真工作区：

```bash
cd /home/ght/wuxihik_navigation
bash test/run_usv_migrate_smoke_test.sh
```

指定实船路径：

```bash
USV_WS=/path/to/USV_NAV bash test/run_usv_migrate_smoke_test.sh
```

### 手动

```bash
source /opt/ros/humble/setup.bash
source /home/ght/USV_NAV/install/setup.bash
export USV_WS=/home/ght/USV_NAV
python3 /home/ght/wuxihik_navigation/test/usv_migrate_smoke_test.py
```

---

## 5. 最近测试结果

| 项目 | 值 |
|------|-----|
| 日期 | 2026-05-29 |
| 环境 | Ubuntu + ROS 2 Humble，本机 mock |
| 实船地图 | `USV_NAV/.../config/map.yaml`（未改文件） |
| 实船参数 | `mission_stack.real_boat.yaml` |
| 结果 | **PASS 7 / FAIL 0** |

```
========== USV_NAV mission 迁移模拟测试 ==========
  [PASS] send_waypoints service 就绪
  [PASS] send_waypoints success (Mission started: 2 waypoints, hash=...)
  [PASS] nav_status RUNNING (got RUNNING)
  [PASS] GCS /waypoint 换线 task_id (got gcs_migrate)
  [PASS] GCS cancel → IDLE (got IDLE)
  [PASS] emergency_stop → EMERGENCY (got EMERGENCY)
  [PASS] cancel_mission 清急停 → IDLE (got IDLE)
========== 结果: PASS=7 FAIL=0 ==========
```

---

## 6. 实船全栈联调（Mock 通过后）

| 步骤 | 命令 |
|:---:|------|
| 1 | MAVROS |
| 2 | `real_boat_bringup.launch.py` |
| 3 | `nav2_real_mavros.launch.py use_sim_time:=false`（默认含 mission 栈） |
| 4 | GCS Dispatch / Cancel，或 `ros2 service call ... send_waypoints` |

检查：

- `ros2 topic echo /nav_status --once` → `task.state`
- **勿**与 `waypoint_transform` + `waypoint_with_state` + `target_buoy` 并行

---

## 7. 相关文件

| 文件 | 说明 |
|------|------|
| `test/usv_migrate_smoke_test.py` | Python 冒烟脚本 |
| `test/run_usv_migrate_smoke_test.sh` | 一键入口 |
| `test/mock_follow_waypoints_server.py` | Mock Nav2 action |
| `docs/USV_NAV_MISSION_SYNC_PLAN.md` | 迁移规划（Phase 1 已完成） |
| `USV_NAV/docs/项目运行与联调.md` | 实船运行入口（已更新 mission 节） |

---

## 8. 已知限制

1. Mock **不验证** 经纬度→map 转换精度（仅走通 GCS JSON 解析与状态机）。
2. 无 GPS 发布时 aggregator 可能报 `LOC_DEGRADED`/`LOST` 告警，**不影响** mission 状态机用例。
3. 须用 **`mission_bridge.launch.py`** 启动；裸 `ros2 run mission_bridge` 不会加载 cancel 话题等 launch 默认值。
