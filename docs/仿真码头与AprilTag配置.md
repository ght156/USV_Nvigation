# 仿真码头与 AprilTag 配置

本文记录 **Gazebo `dock_2022` + `apriltag_localization` + `usv_docking`** 的当前定稿几何、参数与联调步骤，便于改 SDF / 换 Tag 后继续开发。

**相关文档**

| 文档 | 内容 |
|------|------|
| [`项目运行与联调.md`](./项目运行与联调.md) | 仿真栈启动、归港交接命令 |
| [`usv_docking任务规划.md`](./usv_docking任务规划.md) | 状态机与安全逻辑 |
| [`../src/apriltag_localization/docs/AprilTag船坞定位接口.md`](../src/apriltag_localization/docs/AprilTag船坞定位接口.md) | `dock_pose` 接口 |
| [`../src/usv_docking/README.md`](../src/usv_docking/README.md) | 归港控制器 |

---

## 1. 数据流

```text
Gazebo 后向相机 ──► apriltag_localization ──► /apriltag_node/dock_pose（camera_rear_link 系，bay 中心）
                                                      │
                                                      ▼ TF: camera_rear_link → base_link
                                               usv_docking 状态机 ──► /cmd_vel_nav ──► converter ──► Gazebo
```

**原则**：不改 Nav2 / `apriltag_localization` 源码；改 world / 模型 / yaml 即可。

---

## 2. 关键文件

| 路径 | 作用 |
|------|------|
| `src/YILDIZ-USV/workspace_gz/worlds/world.sdf` | 码头 world pose、VRX 评分插件 |
| `src/YILDIZ-USV/workspace_gz/description/roboboat/roboboat.xacro` | 船体 URDF；**前向 `camera_link` + 后向 `camera_rear_link`** |
| `src/YILDIZ-USV/workspace_gz/models/dock_2022/model.sdf` | 三泊位 + placard1/2 + 二维码平面 |
| `src/YILDIZ-USV/workspace_gz/models/placard_2022/model.sdf` | 仅立柱（展示板由 dock 上 tag 平面担任） |
| `src/YILDIZ-USV/workspace_gz/models/placard_2022/materials/textures/*.png` | Gazebo 贴图（**Aruco 图仅视觉**） |
| `src/apriltag_localization/config/detection_cfg_sim.yml` | 仿真 AprilTag 参数 |
| `src/apriltag_localization/config/detection_cfg.yml` | 实船 ZED 参数 |
| `src/usv_docking/config/docking_controller_sim.yaml` | 仿真归港控制器 |

改 **world / 模型** 后须 `colcon build --packages-select workspace_gz` 并重启 Gazebo（见 [`项目运行与联调.md`](./项目运行与联调.md)「停旧进程」）。

---

## 3. 码头 world 布局（2026-06 定稿）

| 项 | 值 |
|----|-----|
| 模型 | VRX **`dock_2022`**（三泊位） |
| world `<pose>` | **`(-4.0, 9.0, 0, yaw=π)`** |
| 船 spawn | `(0, 0, yaw=0)`，船头 **+x** |
| 归港目标 | **bay2**（`correct_dock=True`），对应 **tag id 43 / placard2** |
| 入口朝向 | 码头 yaw=π，开口朝 **-x**（对准船尾） |
| 距 spawn | 入口约 **4 m**（可调 world pose 的 x：-3.5 ~ -5.0） |

**为何 y=9**：`dock_2022` 模型原点在角点；y=9 使 **bay2 中心** 落在船后方通道上。

**bay 中心（模型固定坐标，勿手填）**：取自 `dock_2022/model.sdf` PerformerDetector

| 泊位 | 模型内 pose (x, y, z) |
|------|------------------------|
| bay1 | (1.5, 3, 0.25) |
| bay2 | (1.5, 9, 0.25) |
| bay3 | (1.5, 15, 0.25) |

`world.sdf` 里 bay3 评分配置可删，**Gazebo 仍显示第三泊位几何**（在 base 模型内）。

---

## 4. Placard / 二维码（SDF 定稿）

placard1（tag **0**）与 placard2（tag **43**）几何相同：

| 元素 | pose / 尺寸 |
|------|-------------|
| `link_symbols` | `(0, 0.07, 0)` |
| `tag_visual` | `(0, 0, 0.02, roll=π/2, yaw=π)` |
| `<plane><size>` | **0.5 × 0.5 m**（与 `detection_cfg_sim.yml` 的 `tag_size: 0.5` 一致） |

贴图 URI（须 `colcon build workspace_gz` 后生效）：

```xml
file://placard_2022/materials/textures/AprilTag-tag36h11-ID0.png   <!-- placard1 -->
file://placard_2022/materials/textures/AprilTag-tag36h11-ID43.png  <!-- placard2 -->
```

### tag36h11 贴图

- **`apriltag_localization` 只识别 `tag36h11`**（`apriltag_family_name`）。
- 当前 SDF 已使用 **tag36h11 id 0 / 43** PNG；**Aruco DICT_4X4_1000** 图仅历史参考，不能识别。
- 换贴图时在 [tagsgen.top](https://tagsgen.top/) 生成与 **plane 边长一致** 的 PNG，并同步 **`tag_size`**。
- 无有效 tag 时：`/apriltag_node/dock_pose` 发布 `data: []`，归港卡在 `WAIT_TAG`。

---

## 5. `dock_offset` 参数

**坐标系**：只按 **dock 模型 SDF 坐标** 标定（浮块 y=6/12，bay 中心 y=9）；**不要**用 odom 里船 y≈0 填 yaml。world 换算见 [`AprilTag船坞定位接口.md`](../src/apriltag_localization/docs/AprilTag船坞定位接口.md) §坐标系说明。

**当前定稿**（单 bay 双码，均指向 **P = (1.5, 9, 0.25)**；须含 `camera_tag2ros_` 链，见接口文档）：

| Tag | `dock_offset_x` | `dock_offset_y` | `dock_offset_z` | roll | pitch | yaw |
|-----|-----------------|-----------------|-----------------|------|-------|-----|
| id 0 | **-4.52** | **-3.0** | **2.52** | **-180°** | 0° | 0° |
| id 43 | **-4.52** | **+3.0** | **2.52** | **-180°** | 0° | 0° |

旧参数（±3 / -1.52 / -4.52 / roll±90°）会导致 **仅 tag0 与仅 tag43 时 x 差 ~7 m**、双码平均居中但错误。

### 修改 SDF 后如何同步

1. 改 `tag_visual` pose 或 `<size>` → 更新 **`tag_size`**。
2. 运行 `python3 src/apriltag_localization/scripts/compute_dock_offset_sim.py` 重算 **`dock_offset_*`**。
3. `colcon build --packages-select apriltag_localization` 并重启节点。
4. 分别只挡一个码测 `/apriltag_node/dock_pose` 的 x，两码应接近；双码约为同值。

---

## 6. 相机与 TF（仿真）

倒船归港使用 **后向相机**；前向相机保留（感知 / 调试），AprilTag 与 `usv_docking` **不订阅前向话题**。

| 组件 | 前向 | 后向（归港） |
|------|------|----------------|
| URDF link | `camera_link` @ `(0.25, 0, 0.35)` | **`camera_rear_link`** @ `(-0.25, 0, 0.35, yaw=π)` |
| Gazebo 传感器 | `sensor_camera` | `sensor_camera_rear` |
| 图像 | `/roboboat/sensors/camera/image` | **`/roboboat/sensors/camera_rear/image`** |
| 内参 | `/roboboat/sensors/camera/camera_info` | **`/roboboat/sensors/camera_rear/camera_info`** |
| apriltag `frame_id` | — | **`camera_rear_link`**（`detection_cfg_sim.yml`） |
| usv_docking `camera_frame` | — | **`camera_rear_link`**（`docking_controller_sim.yaml`） |

实船 profile 仍用 ZED 的 **`camera_left_link`**。

验证 TF：

```bash
ros2 run tf2_ros tf2_echo camera_rear_link base_link
ros2 topic hz /roboboat/sensors/camera_rear/image
```

### 坐标系读数提示

- **`/apriltag_node/dock_pose` 的 x/y** 在 **相机系**；船原地转 yaw 时 **xy 可跳米级**（正常，码头在世界系固定、相机系在转）。
- **控制看 `/dock/status` 的 `x_base / y_base / yaw_base`**（经 TF 到 `base_link`）；原地转船时 x/y 应相对稳定，主要变 **yaw_base**。
- 入通道前须 **PRE_ALIGN + OUTSIDE_CENTERING** 把 **|y_base|、|yaw_base|** 压到门槛内（见 `usv_docking` 的 `GATE_CHECK`）。

---

## 7. 联调终端（归港全栈）

每个新终端：`source install/setup.bash`，Gazebo 运行时 **`use_sim_time:=true`**。

| 终端 | 命令 |
|:----:|------|
| 1 | `ros2 launch workspace_gz simulation.launch.py` |
| 2 | `ros2 launch workspace_ros localization.launch.py use_sim_time:=true` |
| 3 | `ros2 launch workspace_nav nav2.launch.py use_sim_time:=true enable_mission_bridge:=true` |
| 4 | `ros2 run workspace_ros converter` |
| 5 | `ros2 launch apriltag_localization apriltag_localization.launch.py`（默认 `profile:=sim`） |
| 6 | Nav2 到预泊点且 mission **IDLE** 后：`ros2 launch usv_docking docking_controller.launch.py profile:=sim use_sim_time:=true` |
| 7 | `bash src/usv_docking/scripts/docking_handoff.sh start` |

**编译**（改包后）：

```bash
colcon build --packages-select workspace_gz apriltag_localization usv_docking
```

**监控**：

```bash
ros2 topic echo /apriltag_node/dock_pose
ros2 topic echo /dock/status
ros2 topic echo /vrx/task/info    # VRX 入泊评分（可选）
```

---

## 8. 常见问题

| 现象 | 处理 |
|------|------|
| 贴图粉/缺图 | `colcon build workspace_gz`；确认 URI 为 `file://placard_2022/materials/textures/...` |
| 无 `dock_pose` | 查 **后向** 话题与 tag36h11 贴图；船尾是否对准 placard |
| `tf_error` in `/dock/status` | 仿真须 **`camera_rear_link`**；查 `tf2_echo camera_rear_link base_link` |
| 原地转船 dock_pose xy 乱跳 | **正常**（相机系）；看 `/dock/status` 的 `x_base/y_base` |
| 改 world 不生效 | 先 `stop_simulation.sh`，再重启终端 1 |
| 退出时 `Unknown message type [9]` / WaveVisual segfault | VRX 波浪插件已知问题；**运行中可忽略**；用脚本停仿真 |
| Nav2 与 docking 抢 `/cmd_vel_nav` | 入泊前 `docking_handoff.sh start`（deactivate controller） |

---

## 9. 待办 / 风险

- [x] 仿真船 **后向相机** + tag36h11 识别联调（`camera_rear_link`）
- [ ] 全栈归港：`GATE_CHECK` → `BACK_IN` → VRX 评分验收
- [ ] 实船 placard 若与模型不一致，单独标定 `detection_cfg.yml` 的 `dock_offset`
- [ ] 通道宽度 / 船宽标定后收紧 `usv_docking` 的 `corridor_enter_*`、`back_in_*_limit`
