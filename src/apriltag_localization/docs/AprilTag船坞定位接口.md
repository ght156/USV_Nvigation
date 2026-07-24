# AprilTag 船坞定位接口

## 配置文件

| profile | 路径 |
|---------|------|
| `sim`（launch 默认） | `config/detection_cfg_sim.yml` |
| `real` | `config/detection_cfg.yml` |

安装后：`install/apriltag_localization/config/`。

---

## 坐标系说明（必读）

仿真里会同时看到 **三套坐标**，参数只认其中一套：

| 坐标系 | 谁在用 | 典型数值示例 |
|--------|--------|--------------|
| **dock 模型系** | SDF、`dock_offset`、PerformerDetector | 浮块 y=**6 / 12**，bay 中心 P 的 y=**9** |
| **world / odom / map** | Nav2、RViz、浮标、船位显示 | 船在 bay 中心时 y≈**0**（因 `world.sdf` 里码头 `<pose y=9 yaw=π>`） |
| **camera_link / camera_rear_link / base_link** | `/apriltag_node/dock_pose`、`usv_docking` | 由节点 + TF 自动计算（仿真归港用 **`camera_rear_link`**） |

**`detection_cfg_sim.yml` 里所有 `dock_offset_*` 只按 dock 模型系标定，不要填 odom 里的 y=0。**

码头 `<static>true</static>` 只表示不随 physics 移动，**不会**导致坐标系不一致。

### world 与 model 的换算（当前定稿）

`world.sdf` 中：

```xml
<include>
  <uri>model://dock_2022</uri>
  <pose>-4.0 9.0 0 0 0 3.141592653589793</pose>
</include>
```

模型点 `(x_m, y_m)` → world：

```text
world_x = -x_m - 4
world_y = -y_m + 9
```

| dock 模型系 | world 系（约） |
|-------------|----------------|
| 左浮块 y=6 | y = +3 |
| **bay 中心 P (1.5, 9, 0.25)** | **(-5.5, 0, 0.25)** |
| 右浮块 y=12 | y = -3 |

船 spawn 在 world `(0,0)`，开到通道正中时 **odom y≈0 是正确的**，与 SDF 里 y=6/12 **不矛盾**。

---

## 单 bay 双码布局（当前仿真）

```text
dock 模型系（俯视图，+x 为通道深度方向）

  y=12  ████  placard2 / tag43
           |
           |  Δy = 3 m
  y=9   ······ P  bay 中心 (1.5, 9, 0.25)
           |
           |  Δy = 3 m
  y=6   ████  placard1 / tag0
```

- **一个 bay**，左右各 **一个** tag36h11（id **0** 与 **43**）。
- 两个 `dock_offset` 均指向 **同一个 P**。
- `tag_ids: [0, 43]`：两码同时可见时对位姿 **取平均**（降噪）；仅见一码时也能输出。

---

## dock_offset 含义

检测得到 Tag 位姿后，在 **Tag 坐标系**下再乘 `dock_offset`，得到 **bay 中心 P** 在 **`frame_id` 相机系**（仿真为 `camera_rear_link`）下的位姿。

```text
Tag 检测 → × dock_offset（Tag 系）→ bay 中心 P → 发布 /apriltag_node/dock_pose
```

| 字段 | 单位 | 含义 |
|------|------|------|
| `dock_offset_x/y/z` | m | Tag 中心 → P 的平移（**Tag 系**表达） |
| `dock_offset_roll/pitch/yaw` | deg | 同上旋转（节点内部转 rad） |

左右 tag 的 `dock_offset_x` **符号相反**（±3.0），`y/z` 与姿态角相同——这是 Tag 安装镜像导致的，**不是**写错。

### 当前定稿数值（经 SDF + 节点坐标链重算）

节点内变换：`T = camera2camera_link × T_det × camera_tag2ros_ × dock_offset`。  
**`dock_offset` 写在 `camera_tag2ros_` 之后**，不能直接把 dock 模型系里的 Δx/Δy 填进 yaml（旧值 ±3 / -1.52 / -4.52 / roll±90° 会导致 tag0 与 tag43 算出不同 bay 中心，单码 x≈13 vs 6、双码平均≈9）。

bay 中心 **P = (1.5, 9, 0.25)**，placard1/2 链式 pose 见 `dock_2022/model.sdf`：

| Tag | `dock_offset_x` | `dock_offset_y` | `dock_offset_z` | roll | pitch | yaw |
|-----|---------------|-----------------|-----------------|------|-------|-----|
| id **0**（y=6） | **-4.52** | **-3.0** | **2.52** | **-180°** | 0° | 0° |
| id **43**（y=12） | **-4.52** | **+3.0** | **2.52** | **-180°** | 0° | 0° |

左右 tag 仅 **`dock_offset_y` 符号相反**；改 SDF 后须用 [`scripts/compute_dock_offset_sim.py`](../scripts/compute_dock_offset_sim.py) 重算。

改 SDF 里 placard / tag pose 或 P 后，须 **整链重算**（见下文「重算步骤」）。

---

## 参数样例（`detection_cfg_sim.yml`）

```yaml
apriltag_node:
  ros__parameters:
    camera_info_topic: "/roboboat/sensors/camera_rear/camera_info"
    image_topic: "/roboboat/sensors/camera_rear/image"
    detection_result_topic: "/apriltag_node/dock_pose"
    frame_id: camera_rear_link
    dock_frame_id: dock_frame        # TF 广播的 dock 坐标系名称（默认 dock_frame）
    apriltag_family_name: tag36h11
    tag_size: 0.5                    # 与 model.sdf plane 边长一致
    tag_ids: [0, 43]

    tag_0:
      dock_offset_x: -4.52
      dock_offset_y: -3.0
      dock_offset_z: 2.52
      dock_offset_roll: -180.0
      dock_offset_pitch: 0.0
      dock_offset_yaw: 0.0

    tag_43:
      dock_offset_x: -4.52
      dock_offset_y: 3.0
      dock_offset_z: 2.52
      dock_offset_roll: -180.0
      dock_offset_pitch: 0.0
      dock_offset_yaw: 0.0
```

---

## 使用流程

1. 贴图须为 **tag36h11**（[tagsgen.top](https://tagsgen.top/)），与 `tag_size`、SDF plane 边长一致。
2. 改 SDF 后按「重算步骤」更新 `tag_size` 与各 `dock_offset_*`。
3. Gazebo + bridge 发布相机话题。
4. 启动节点：

```bash
ros2 launch apriltag_localization apriltag_localization.launch.py profile:=sim use_sim_time:=true
```

---

## 输出

### 话题输出

| 项 | 值 |
|----|-----|
| 话题 | `/apriltag_node/dock_pose` |
| 类型 | `std_msgs/msg/Float64MultiArray` |
| 内容 | **bay 中心 P** 在 **`frame_id`**（仿真 **`camera_rear_link`**）下 `(x, y, z, roll, pitch, yaw)` |
| 单位 | xyz：**m**；rpy：**rad** |

无 tag 时发布 `data: []`（节点在跑，只是未检测到）。

### TF 广播

节点在每次有效检测后同时广播一条 TF 变换：

| 项 | 值 |
|----|-----|
| parent frame | `frame_id` 配置值（仿真 `camera_rear_link`，实船 `camera_left_link`） |
| child frame | `dock_frame_id` 配置值（默认 `dock_frame`） |
| 内容 | bay 中心在相机系下的位姿（与 Float64MultiArray 相同，不求逆） |
| 时间戳 | 与图像帧一致 |

**用途：** 下游节点（如 `usv_docking`）可通过 TF 直接查询 `dock_frame → base_link`，获得船体在船坞坐标系下的位姿，无需手动变换：

```bash
# 查看船在 dock 坐标系下的位姿
ros2 run tf2_ros tf2_echo dock_frame base_link
```

TF2 自动利用 `base_link → camera_rear_link`（固定外参）+ `camera_rear_link → dock_frame`（本节点广播）完成反算。

> **注意：** 无检测时不广播 TF，`dock_frame` 在 TF 树中会超时消失。下游节点需处理 TF 查询失败的情况。

---

## 标定验证

1. 在 Gazebo 将船开到 **通道几何中心**（world 约 **x=-5.5, y=0**）。
2. 启动 `usv_docking` 后看 `/dock/status`：`x_base`, `y_base`, `yaw_base` 应接近 **0**。
3. 若系统性偏差 → 微调 `dock_offset` 或检查 `tag_size` / SDF pose，**不要**用 odom 坐标直接填 yaml。

---

## 重算 dock_offset 步骤

1. 在 **dock 模型系** 定 bay 中心 **P**（当前 bay2：**(1.5, 9, 0.25)**）。
2. 由 SDF 链式 pose 得各 Tag 在 dock 系下的 `T_dock_tag`。
3. **不要**直接把 dock 系 `P−Tag` 填 yaml；须按节点乘法顺序求  
   `dock_offset = inv(camera_tag2ros_) × inv(T_det) × inv(camera2camera_link) × T_cam_P`  
   （理想检测下 `T_det = inv(camera2camera_link) × T_cam_tag`）。  
4. 或运行：`python3 src/apriltag_localization/scripts/compute_dock_offset_sim.py`
5. `colcon build --packages-select apriltag_localization` 并重启节点。

---

## 下游：归港入泊（usv_docking）

| 文档 | 说明 |
|------|------|
| [`../usv_docking/README.md`](../../usv_docking/README.md) | 编译、话题、联调 |
| [`../../docs/usv_docking任务规划.md`](../../docs/usv_docking任务规划.md) | 状态机 |
| [`../../docs/仿真码头与AprilTag配置.md`](../../docs/仿真码头与AprilTag配置.md) | world 布局与联调终端 |

`/apriltag_node/dock_pose` 经 TF（仿真 **`camera_rear_link` → `base_link`**）后由 `usv_docking` 控制；仿真 `frame_id` 须与 URDF 中**归港相机**帧一致（当前 **`camera_rear_link`**）。

节点同时广播 `camera_rear_link → dock_frame` TF，下游也可直接查询 `dock_frame → base_link` 获得船在船坞系下的位姿（x/y/yaw），用于中轴线对准判断。详见 [实船Tag安装与船坞坐标系标定](实船Tag安装与船坞坐标系标定.md)。
