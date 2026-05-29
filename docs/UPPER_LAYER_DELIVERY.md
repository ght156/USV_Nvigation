# 发给竞赛上层的对接包清单

> 地面站 GCS（`/waypoint` Topic 等）**不要**发给上层；上层只走 **Service 下发 + Topic 反馈**。

---

## 必发（最小包，共 10 个文件）

### 1. 主文档（1）

| 文件 | 说明 |
|------|------|
| [`nav_task_interface.md`](nav_task_interface.md) | **主对接文档**：Service 名、行为、状态机、JSON 字段、示例 |

### 2. 可编译 Service 定义（4）— 来自 `src/m_common/srv/`

上层 `find_package(m_common)` / `ros2 interface show` **必须用这一组**：

| 文件 | ROS 类型 | Service 名 |
|------|----------|------------|
| `SendWaypoints.srv` | `m_common/srv/SendWaypoints` | `/mission_bridge/send_waypoints` |
| `SetPause.srv` | `m_common/srv/SetPause` | `/mission_bridge/set_pause` |
| `EmergencyStop.srv` | `m_common/srv/EmergencyStop` | `/mission_bridge/emergency_stop` |
| `CancelMission.srv` | `m_common/srv/CancelMission` | `/mission_bridge/cancel_mission` |

**打包路径**：`wuxihik_navigation/src/m_common/srv/` 下上述 4 个文件。

> `m_common` 里还有 `RTSPStreamSwitch.srv`、`ReportVideoWarning.srv` 等**感知/视频**接口，**与导航无关，不要发**。

### 3. Service 说明 + 索引（5）

| 文件 | 说明 |
|------|------|
| [`srv/README.md`](srv/README.md) | 索引与架构说明 |
| [`srv/nav_send_waypoints.srv`](srv/nav_send_waypoints.srv) | 与 `SendWaypoints.srv` 同义（文档版） |
| [`srv/nav_set_pause.srv`](srv/nav_set_pause.srv) | 与 `SetPause.srv` 同义 |
| [`srv/nav_emergency_stop.srv`](srv/nav_emergency_stop.srv) | 与 `EmergencyStop.srv` 同义 |
| [`srv/nav_cancel_mission.srv`](srv/nav_cancel_mission.srv) | 与 `CancelMission.srv` 同义 |

`docs/srv/nav_*.srv` 与 `m_common/srv/*.srv` **字段一致**，区别仅在于：

- **`m_common/srv/`** → 参与编译，生成 C++/Python 类型  
- **`docs/srv/nav_*.srv`** → 随文档阅读，文件名带 `nav_` 前缀便于识别  

**二选一即可编译**；建议 **两组都发**，避免上层只拿到 doc 版却无法编译。

---

## 强烈建议发（4）— `docs/msg/` JSON 字段说明

| 文件 | 用途 | 是否 ROS 可编译类型 |
|------|------|---------------------|
| `nav_status_snapshot.msg` | `/nav_status` 的 JSON 结构 | **否**（说明文档） |
| `nav_task_event.msg` | `/task_event` 的 JSON 结构 | **否** |
| `nav_waypoint_gps.msg` | 地面站经纬度格式（上层 Service **可忽略**） | **否** |
| `nav_task_result.msg` | Topic 拒绝码参考（上层 Service **可忽略**） | **否** |

**为什么要发**：上层订 `/nav_status`、`/task_event` 时需知道 JSON 里有哪些字段；**但 ROS 订阅类型仍是 `std_msgs/String`**，不是这些 `.msg` 名。

**最小 msg 包**：只发 `nav_status_snapshot.msg` + `nav_task_event.msg` 即可。

---

## 不要发

| 内容 | 原因 |
|------|------|
| `test/` 目录 | 内部联调脚本 |
| `GROUND-CONTROL-STATION-dev` | 地面站，非上层接口 |
| `m_common` 中视频/感知 srv、msg | 与导航无关 |
| 船端 launch、map yaml、Nav2 参数 | 船端集成，见 `mission_stack.example.yaml` |

---

## 推荐打包命令

在仓库根目录执行，生成 `upper_layer_nav_interface.tar.gz`：

```bash
cd /path/to/wuxihik_navigation

tar czvf upper_layer_nav_interface.tar.gz \
  docs/nav_task_interface.md \
  docs/UPPER_LAYER_DELIVERY.md \
  docs/srv/README.md \
  docs/srv/nav_send_waypoints.srv \
  docs/srv/nav_set_pause.srv \
  docs/srv/nav_emergency_stop.srv \
  docs/srv/nav_cancel_mission.srv \
  docs/msg/nav_status_snapshot.msg \
  docs/msg/nav_task_event.msg \
  docs/msg/nav_waypoint_gps.msg \
  docs/msg/nav_task_result.msg \
  src/m_common/srv/SendWaypoints.srv \
  src/m_common/srv/SetPause.srv \
  src/m_common/srv/EmergencyStop.srv \
  src/m_common/srv/CancelMission.srv
```

共 **14 个文件**（1 主文档 + 1 本清单 + 5 srv 说明 + 4 msg 说明 + 4 可编译 srv）。

---

## 上层还需要什么（不在压缩包里）

1. **运行中的船端**：已启动 `mission_bridge` + `nav_status_aggregator`，且 Nav2/TF/odom 就绪。  
2. **`m_common` 安装包或源码**：若上层要在自己的 workspace 里编译客户端，需整个 `m_common` 包（至少含上述 4 个 srv + `package.xml` + `CMakeLists.txt`）。  
   - 仅发 4 个 `.srv` 文件不够编译，需完整 ament 包；**更简单做法**：船东提供已 `colcon build` 的 `install/setup.bash` 环境。  
3. **QoS**：订阅 `/nav_status` 必须用 **RELIABLE + TRANSIENT_LOCAL**（见主文档 §4.1）。

---

## 一句话

| 发给上层 | 要不要 |
|----------|--------|
| `nav_task_interface.md` | **必发** |
| `src/m_common/srv/` 四个导航 srv | **必发**（或可编译的完整 `m_common` 包） |
| `docs/srv/` 四个 `nav_*.srv` + README | **建议发**（与 m_common 对照阅读） |
| `docs/msg/` 四个文件 | **建议发**（解析 JSON 用；**至少** status + event 两个） |
| `test/` | **不要发** |
