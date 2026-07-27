# TF 归港控制器说明

本文说明 `tf_docking_node` 的架构、状态机、控制律与参数。
该节点与旧 `docking_controller` **并存互斥**，launch 时二选一。

---

## 1. 与旧控制器的区别

| | 旧 `docking_controller` | 新 `tf_docking_node` |
|---|---|---|
| 位姿来源 | 订阅 `/apriltag_node/dock_pose` (Float64MultiArray) | TF2 查询 `dock_frame → base_link` |
| 坐标变换 | 手动构造 PoseStamped + `do_transform_pose_stamped` | TF2 自动走 URDF 链 |
| 控制坐标系 | base_link（dock 在船体系下的位置） | **dock_frame**（船在坞系下的位置） |
| 状态机 | 含 GNSS 接近、PRE_ALIGN、GATE_CHECK 等 | 精简：假设 Nav2 已到预泊点 |
| 文件 | `docking_controller.py` (2100+ 行) | `tf_docking_node.py` (~700 行) |

**接口完全兼容**：`/dock/start`、`/dock/status`（JSON）、`/cmd_vel_nav`，dock_mission 无需修改。

---

## 2. 数据流

```
apriltag_localization (C++)
    广播 TF: camera_link → dock_frame
        │
        ▼  TF tree (URDF: base_link → camera_link)
tf_pose_provider.py
    lookup_transform("dock_frame", "base_link")
    → PoseData(x, y, yaw, age, quality)
        │
        ▼  EMA 滤波 + yaw 跳变剔除
tf_docking_node.py
    状态机 → (v, w) → /cmd_vel_nav
```

### 坐标系含义

`lookup_transform("dock_frame", "base_link")` 返回 **船在船坞坐标系下的位姿**：

| 量 | 含义 |
|---|---|
| `x` | 船沿坞轴方向的位置（入坞方向为正） |
| `y` | 船偏离中轴线的横向距离（左正） |
| `yaw` | 船艏在坞系下的朝向 |

倒泊对准时 `yaw ≈ π`（船头朝外），`heading_error = wrap(yaw - entry_heading)`。

### TF 质量等级

| quality | 含义 | 控制器行为 |
|---|---|---|
| `GOOD` | 正常 | 更新位姿，正常控制 |
| `STALE` | TF 时间戳过旧 | 视为丢失，停船等待 |
| `JUMP` | 相邻帧跳变过大 | 视为丢失 |
| `OUT_OF_RANGE` | 距离不合理 | 视为丢失 |
| `INVALID` | TF 查询失败 | 视为丢失 |

---

## 3. 状态机

```
IDLE ──[/dock/start]──→ ACQUIRE_TAG
                            │
                    连续5帧GOOD │ 超时
                            ▼      ▼
                       RECORD_POSE  SEARCH_TAG ←──┐
                            │       旋转搜索      │
                    偏差过大 │  交替CW/CCW        │
                      ▼     │       │ 找到tag     │
                   ABORT    ▼       └─────────────┘
              (reapproach) APPROACH_ALIGN
                           恒速倒车+转向
                                │
                          depth < 5m
                                ▼
                             ALIGN
                           精调 y/yaw
                          (低速蠕动)
                                │
                           连续5帧达标
                                ▼
                           BACK_IN ←──────┐
                           沿中轴倒车      │
                           速度∝剩余距离   │
                                │         │
                     走廊超限    │  修正成功 │
                       (10帧)   ▼         │
                          CORRECTING ─────┘
                          原地修正(0.03m/s舵效)
                                │
                           修正超时×3
                                ▼
                             ABORT

BACK_IN 近距离丢 tag（depth < 1.5m）：
    → 盲倒模式：0.05m/s 直退，不搜索
    → 超时15s → 视为 DOCKED

任意状态丢 tag（非盲倒区）：
    → 停船等5s → 未恢复 → SEARCH_TAG（记住恢复状态）
```

---

## 4. 控制律

所有倒车阶段共用转向律（dock_frame 下）：

```
w = ky · y − kyaw · heading_error
```

- `y > 0`（船偏左）→ `w > 0`（CCW，倒车时船尾右移，向中线靠拢）
- `heading_error > 0`（船艏偏左）→ `w < 0`（CW，修正航向）

| 状态 | 纵向速度 v | 转向 w |
|---|---|---|
| APPROACH_ALIGN | `-approach_speed`（恒速） | `ky·y − kyaw·err` |
| ALIGN | 航向误差 < 15° 时 `-0.08`，否则 `0` | `ky·y − kyaw·err` |
| BACK_IN | `-clamp(kx·remaining, min, max)` | `ky·y − kyaw·err` |
| CORRECTING | `-0.03`（保持舵效） | `ky·y − kyaw·err` |
| 盲倒 | `-0.05`（直退） | `0` |

---

## 5. 安全机制

| 场景 | 处理 |
|---|---|
| 预泊点看不到 tag | SEARCH_TAG 旋转搜索，交替 CW/CCW，最多重试 2 次 |
| 搜索后仍无 tag | ABORT + `needs_reapproach=true`，dock_mission 重新导航 |
| 初始位置偏差过大 | ABORT + `needs_reapproach=true` |
| 接近/对准中丢 tag | 停船等 5s → 搜索 → 找到后回 RECORD_POSE 重新校验 |
| **倒车近距离丢 tag** | **盲倒**：depth < 1.5m 时不搜索，0.05m/s 直退 |
| 倒车偏离走廊 | CORRECTING 原地修正（最多 2 次）→ 超过则 ABORT |
| 各状态超时 | ABORT |
| 总会话超时 | 240s 强制 ABORT |

---

## 6. 启动方式

```bash
# 新 TF 归港（与旧节点互斥）
ros2 launch usv_docking tf_docking.launch.py

# 旧归港（保持不变）
ros2 launch usv_docking docking_controller.launch.py
```

---

## 7. 参数速查

完整参数及注释见 `config/tf_docking_sim.yaml`。关键参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `entry_heading_rad` | π | 对准目标航向，倒泊=π |
| `approach_speed` | 0.30 | 恒速接近 (m/s) |
| `approach_entry_x_m` | 5.0 | 切入精调的距离 (m) |
| `align_y_tol_m` | 0.10 | 对准横向阈值 (m) |
| `align_yaw_tol_rad` | 0.08 | 对准航向阈值 (rad) |
| `back_in_blind_depth_m` | 1.5 | 盲倒触发距离 (m) |
| `back_in_blind_speed` | 0.05 | 盲倒速度 (m/s) |
| `back_in_corridor_y_m` | 0.30 | 走廊宽度 (m) |
| `stop_x_m` / `stop_y_m` / `stop_yaw_rad` | 0.30/0.15/0.08 | 停靠判定 |

---

## 8. 相关文档

| 文档 | 内容 |
|---|---|
| [`归港对准与位姿说明.md`](归港对准与位姿说明.md) | 旧控制器 y/yaw 含义、双码、联调 |
| [`../../apriltag_localization/docs/AprilTag船坞定位接口.md`](../../apriltag_localization/docs/AprilTag船坞定位接口.md) | AprilTag TF 广播接口 |
| [`../../apriltag_localization/docs/实船Tag安装与船坞坐标系标定.md`](../../apriltag_localization/docs/实船Tag安装与船坞坐标系标定.md) | dock_frame 定义与标定 |
