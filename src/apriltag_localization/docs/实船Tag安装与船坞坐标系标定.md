# 实船 Tag 安装与船坞坐标系标定

> 本文面向现场安装与调试人员，说明 AprilTag 二维码在实船船坞上的安装方法、船坞坐标系约定、以及 `dock_offset` 参数的标定流程。

---

## 1. 系统概述

船尾安装后向相机（`camera_rear_link`），倒泊时相机朝向船坞。船坞后墙安装 AprilTag 二维码，`apriltag_localization` 节点检测 Tag 后：

1. 发布 `/apriltag_node/dock_pose`（Float64MultiArray）：泊位中心在相机系下的位姿
2. 广播 TF `camera_rear_link → dock_frame`：使 dock 坐标系进入 TF 树

归港控制节点通过查询 `dock_frame → base_link` 获得**船在船坞坐标系下的位姿**，用 `y`（横向偏差）和 `yaw`（航向偏差）判断是否对准廊道中轴线。

```
船（倒泊方向 ←）                    船坞
┌──────────┐                    ┌─────────────────┐
│          │  ← 后向相机        │                 │
│  船体    │  ─────────────►   │  [Tag0] [Tag43] │ ← 后墙
│ base_link│   相机视线         │                 │
│          │                    │   泊位中心 P     │
└──────────┘                    │   (dock_frame)  │
                                │                 │
                                └──── 入口 ───────┘
```

---

## 2. 船坞坐标系约定

```
dock_frame（右手坐标系）:
  原点：泊位中心 P（船最终停靠位置）
  X 轴：沿廊道，指向坞内（后墙方向）——即倒泊方向
  Y 轴：横向，面向 X 方向时的左侧
  Z 轴：向上
```

归港控制中的含义：

| 分量 | 含义 | 对准条件 |
|------|------|----------|
| `x_dock` | 船到泊位中心的纵向距离（沿廊道） | 倒泊过程中逐渐减小 |
| `y_dock` | 船偏离廊道中轴线的横向距离 | `|y_dock| < 0.20 m` |
| `yaw_dock` | 船首向与廊道轴线的夹角 | `|yaw_dock| < 5°` |

> **注意：** 船对准廊道准备倒泊时，船头朝外（远离船坞），`base_link` X 与 `dock_frame` X 反向，因此 `yaw_dock ≈ ±π`。归港控制器中已设 `heading_offset = π` 来补偿此偏移。

---

## 3. Tag 安装要求

### 3.1 安装位置

```
船坞后墙（正视图，面向入口方向看）

         ┌─────────────────────────────────┐
         │            后墙/横梁             │
         │                                 │
         │   ┌─────┐         ┌─────┐      │
         │   │Tag 0│         │Tag43│      │  ← Tag 安装高度：相机视野内
         │   └─────┘         └─────┘      │
         │       ←── 关于中轴线对称 ──→     │
         │              │                  │
         │              │ 廊道中轴线        │
         └──────────────┼──────────────────┘
                        │
                    入口方向（船从此倒入）
```

**要求：**

| 项目 | 要求 |
|------|------|
| 安装面 | 船坞**后墙**或末端横梁，面朝入口方向（朝船） |
| 对称性 | 两个 Tag 关于廊道中轴线**左右对称** |
| 高度 | Tag 中心在相机视野范围内（建议与相机等高或略高） |
| 姿态 | Tag 面尽量**竖直**，与廊道轴线**垂直** |
| 间距 | 两 Tag 间距建议 ≥ 2 m（基线越长，航向估计越准） |
| 环境 | 避免强光直射、水面反光；Tag 表面平整无褶皱 |

### 3.2 Tag 制作

| 项目 | 规格 |
|------|------|
| Tag 族 | **tag36h11**（与配置一致） |
| ID | 与配置中 `tag_ids` 一致（当前 `[0, 43]`） |
| 尺寸 | 与配置中 `tag_size` 严格一致（实船当前 2.0 m 边长） |
| 材质 | 防水、防紫外线（推荐铝板喷印或户外级 PVC） |
| 生成工具 | [tagsgen.top](https://tagsgen.top/) 或 `apriltag-generation` 工具 |
| 白边 | Tag 四周保留 ≥ 1 个单元宽度的白色静区 |

> **`tag_size` 是位姿估算的唯一尺度参考。** 打印/制作尺寸与配置不一致会导致距离成比例偏差。

### 3.3 安装检查清单

- [ ] Tag 族为 tag36h11，ID 正确
- [ ] 实际边长与 `tag_size` 配置一致（用卷尺核实）
- [ ] Tag 面竖直，与廊道轴线垂直（用水平尺/量角器）
- [ ] 两 Tag 关于中轴线对称（测量间距和到中线距离）
- [ ] Tag 面朝向入口方向（船倒入时相机能看到正面）
- [ ] 无遮挡、无强光反射
- [ ] 记录 Tag 中心到泊位中心 P 的距离（用于标定 dock_offset）

---

## 4. dock_offset 参数标定

### 4.1 参数含义

`dock_offset` 定义 **Tag 中心 → 泊位中心 P** 的刚体变换，在 Tag 的 ROS 坐标系下表达：

```yaml
tag_0:
  dock_offset_x: <纵向>    # Tag → P 的 X 分量（沿廊道方向）
  dock_offset_y: <横向>    # Tag → P 的 Y 分量（垂直廊道）
  dock_offset_z: <高度>    # Tag → P 的 Z 分量（竖直方向）
  dock_offset_roll: <deg>  # 绕 X 旋转（度）
  dock_offset_pitch: <deg> # 绕 Y 旋转（度）
  dock_offset_yaw: <deg>   # 绕 Z 旋转（度）
```

节点内部变换链：

```
最终位姿 = camera2camera_link × Tag检测位姿 × camera_tag2ros_ × dock_offset
```

`dock_offset` 的旋转部分决定了 `dock_frame` 的坐标轴方向。当前配置 `roll: -180°` 使 dock_frame 的 Z 轴朝上、X 轴指向坞内。

### 4.2 标定步骤

**第一步：确定泊位中心 P**

泊位中心是船最终停靠时船体中心应对准的位置。在船坞地面上标记此点。

**第二步：测量 Tag 到 P 的距离**

用卷尺/测距仪测量每个 Tag 中心到 P 点的三维距离：

```
对 Tag 0:
  纵向距离（沿廊道）: d_x = ___ m
  横向距离（垂直廊道）: d_y = ___ m  （P 在 Tag 左侧为正）
  高度差: d_z = ___ m               （P 在 Tag 上方为正）

对 Tag 43: 同上
```

**第三步：确定旋转参数**

旋转参数使 dock_frame 满足第 2 节的约定（X 入坞、Y 横向、Z 向上）。

对于 Tag 面竖直、正对入口的标准安装：

```yaml
dock_offset_roll: -180.0    # 使 dock Z 朝上
dock_offset_pitch: 0.0
dock_offset_yaw: 0.0
```

> 如果 Tag 安装面不垂直于廊道（有偏转角），需在 `yaw` 中补偿该角度。

**第四步：填入配置文件**

编辑 `config/detection_cfg.yml`（实船）：

```yaml
apriltag_node:
  ros__parameters:
    camera_info_topic: "/zed/zed_node/rgb/color/rect/camera_info"
    image_topic: "/zed/zed_node/rgb/color/rect/image"
    detection_result_topic: "/apriltag_node/dock_pose"
    frame_id: camera_left_link          # 实船相机坐标系名称
    dock_frame_id: dock_frame           # dock 坐标系名称
    apriltag_family_name: tag36h11
    tag_size: 2.0                       # Tag 实际打印边长（米），必须精确
    tag_ids: [0, 43]

    tag_0:
      dock_offset_x: <实测纵向>         # 例: -4.52
      dock_offset_y: <实测横向>         # 例: -3.0（P 在 Tag 右侧则为负）
      dock_offset_z: <实测高度差>       # 例: 2.52
      dock_offset_roll: -180.0
      dock_offset_pitch: 0.0
      dock_offset_yaw: 0.0

    tag_43:
      dock_offset_x: <实测纵向>         # 与 tag_0 相同（同面墙）
      dock_offset_y: <实测横向>         # 符号与 tag_0 相反（对称）
      dock_offset_z: <实测高度差>       # 与 tag_0 相同
      dock_offset_roll: -180.0
      dock_offset_pitch: 0.0
      dock_offset_yaw: 0.0
```

**关键规则：**
- 两 Tag 在同一面墙上时：`dock_offset_x` 和 `dock_offset_z` 相同，`dock_offset_y` **符号相反**
- 角度单位为**度**（节点内部自动转弧度）
- `tag_size` 必须与实际打印边长精确一致

**第五步：编译部署**

```bash
colcon build --packages-select apriltag_localization
```

### 4.3 仿真中的 dock_offset（参考）

仿真配置 `detection_cfg_sim.yml` 当前值（基于 `dock_2022` 模型 SDF 计算）：

| Tag | x | y | z | roll | pitch | yaw |
|-----|------|------|------|------|-------|-----|
| 0 | -2.52 | 3.0 | 2.52 | -180° | 0° | 0° |
| 43 | -2.52 | -3.0 | 2.52 | -180° | 0° | 0° |

仿真中可用 `scripts/compute_dock_offset_sim.py` 自动计算。实船必须手动测量。

---

## 5. 验证

### 5.1 基本验证

启动节点后，确认以下日志出现：

```
TF broadcaster: camera_left_link -> dock_frame
=========================AprilTagLocalization Init Successfully!!=========================
```

### 5.2 TF 验证

将船停在廊道中轴线上、正对船坞：

```bash
# 查看船在 dock 坐标系下的位姿
ros2 run tf2_ros tf2_echo dock_frame base_link
```

期望结果：

| 分量 | 期望值 | 含义 |
|------|--------|------|
| y | ≈ 0 | 船在中轴线上 |
| yaw | ≈ ±π (±180°) | 船头朝外（正常，因船倒泊） |
| x | > 0 | 船在泊位前方（尚未入坞） |

### 5.3 横向偏差验证

将船向左/右平移 0.5 m，观察 `y` 变化：
- 船向左移 → `y` 应增大（或减小，取决于 Y 轴方向）
- 确认方向与预期一致，否则检查 `dock_offset_y` 符号

### 5.4 航向偏差验证

将船转一个小角度（如 10°），观察 `yaw` 变化：
- 确认变化方向与预期一致
- 偏差为 0 时 yaw 应接近 ±π

### 5.5 常见问题

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 无 TF 输出 | 相机话题未发布 / Tag 未检测到 | 检查 `ros2 topic hz` 和相机画面 |
| y 方向反 | `dock_offset_y` 符号错误 | 交换两 Tag 的 y 符号 |
| yaw 偏差固定偏移 | Tag 面未正对入口 / yaw 未补偿 | 在 `dock_offset_yaw` 中补偿安装偏角 |
| 距离成比例偏差 | `tag_size` 与实际不符 | 用卷尺核实 Tag 边长 |
| dock_frame Z 朝下 | `dock_offset_roll` 错误 | 确认 roll = -180° |
| 两 Tag 算出不同 P | dock_offset 不对称 | 检查两 Tag 的 y 是否符号相反 |

---

## 6. 配置文件完整参考

```yaml
# =============================================================================
# 实船配置 — config/detection_cfg.yml
# =============================================================================
apriltag_node:
  ros__parameters:
    # --- 话题配置 ---
    camera_info_topic: "/zed/zed_node/rgb/color/rect/camera_info"  # 相机内参话题
    image_topic: "/zed/zed_node/rgb/color/rect/image"              # 图像话题
    detection_result_topic: "/apriltag_node/dock_pose"             # 定位结果发布话题
    frame_id: camera_left_link       # 输出位姿的参考坐标系（须与 URDF 一致）
    dock_frame_id: dock_frame        # TF 广播的 dock 坐标系名称

    # --- Tag 配置 ---
    apriltag_family_name: tag36h11   # Tag 族（须与打印的 Tag 一致）
    tag_size: 2.0                    # Tag 物理边长（米），必须与实物精确一致
    tag_ids: [0, 43]                 # 需检测的 Tag ID 列表

    # --- Tag 0 的 dock_offset（Tag 中心 → 泊位中心 P）---
    tag_0:
      dock_offset_x: -4.52           # 纵向距离（沿廊道，米）
      dock_offset_y: -3.0            # 横向距离（垂直廊道，米）
      dock_offset_z: 2.52            # 高度差（米）
      dock_offset_roll: -180.0       # 绕 X 旋转（度），-180 使 dock Z 朝上
      dock_offset_pitch: 0.0         # 绕 Y 旋转（度）
      dock_offset_yaw: 0.0           # 绕 Z 旋转（度），补偿 Tag 安装偏角

    # --- Tag 43 的 dock_offset ---
    tag_43:
      dock_offset_x: -4.52           # 与 tag_0 相同（同一面墙）
      dock_offset_y: 3.0             # 符号与 tag_0 相反（对称安装）
      dock_offset_z: 2.52            # 与 tag_0 相同
      dock_offset_roll: -180.0
      dock_offset_pitch: 0.0
      dock_offset_yaw: 0.0
```

---

## 7. 相关文档

| 文档 | 内容 |
|------|------|
| [AprilTag船坞定位接口.md](AprilTag船坞定位接口.md) | 接口快速参考、坐标系说明、使用流程 |
| [AprilTag定位实现与参数说明.md](AprilTag定位实现与参数说明.md) | 节点实现细节、检测算法、全部参数 |
| [dual_tag_baseline_mode.md](dual_tag_baseline_mode.md) | 双 Tag 基线增强模式 |
| [../../usv_docking/README.md](../../usv_docking/README.md) | 下游归港控制器 |
