# AprilTag 定位实现与参数说明

## 1. 架构概览

`apriltag_localization` 是一个 ROS2 C++ 节点，订阅相机图像和 `CameraInfo`，通过内嵌的 AprilTag C 库检测图像中的 AprilTag 二维码，估算每个 Tag 的 6-DOF 位姿，经坐标系转换和 dock_offset 偏移后，输出船坞（bay）中心在相机坐标系下的位姿。

```
Camera Image ──► imageCallback() ──► main_loop() ──► detect()
                                                         │
CameraInfo ────► cameraInfoCallback() ────────────────────┘
                                                         │
                                          ┌──────────────┘
                                          ▼
                              apriltag_detector_detect()   (C 库：四边形检测 + 解码)
                                          │
                                          ▼
                              estimate_tag_pose()          (C 库：单应性 → 正交迭代 → 去歧义)
                                          │
                                          ▼
                              坐标系转换 + dock_offset      (camera2camera_link × tag_pose × camera_tag2ros_ × dock_offset)
                                          │
                                          ▼
                              多 Tag 位姿平均               (Markley SVD 方法)
                                          │
                                          ▼
                              发布 Float64MultiArray        (/apriltag_node/dock_pose)
```

核心依赖：

| 依赖 | 用途 |
|------|------|
| `apriltag` (内嵌 C 库) | Tag 检测与位姿估算 |
| `cv_bridge` + OpenCV | 图像格式转换 |
| `Eigen3` | 矩阵运算（旋转平均、SVD） |
| `tf2` | 坐标系变换 |
| `yaml-cpp` | 配置文件解析 |
| `m_common` (mlogger) | 日志输出 |

---

## 2. 节点生命周期

### 2.1 构造与初始化

1. **initConfig()** — 从 ROS2 参数服务器加载所有配置项（话题名、Tag 族、Tag 尺寸、dock_offset 等），详见 [第 3 节](#3-参数配置)
2. **initModel()** — 根据 `apriltag_family_name` 创建对应的 Tag 族对象和检测器：
   - 调用 `tag36h11_create()` 等函数创建 `apriltag_family_t`
   - 调用 `apriltag_detector_create()` 创建检测器
   - 将 Tag 族注册到检测器，线程数固定为 2
3. **intiNode()** — 创建 ROS2 订阅/发布，启动 `main_loop_` 线程

### 2.2 运行时

- **cameraInfoCallback()**：仅接收首条 `CameraInfo` 消息，提取 `fx, fy, cx, cy` 和畸变参数。后续消息忽略（假设内参不变）。
- **imageCallback()**：收到图像后通过 `cv_bridge::toCvShare` 零拷贝共享指针，通知 `main_loop_` 线程。
- **main_loop()**：以 2 秒超时等待条件变量，收到图像后调用 `detect()`。

---

## 3. 检测与位姿估算流水线

`detect()` 是核心处理函数，完整流程如下：

### 3.1 图像预处理

```cpp
cv_bridge_shared_->image.clone()    // 深拷贝 BGR 图像（线程安全）
cv::cvtColor(image_mat, image_gray, cv::COLOR_BGR2GRAY)  // 转灰度
```

构造 `image_u8_t` 结构体直接包装 OpenCV 的 `data` 指针（零拷贝），传入 C 库。

### 3.2 AprilTag 检测（C 库）

调用 `apriltag_detector_detect(td_ptr_, &img)`，内部流程：

1. **四边形检测** (`apriltag_quad_thresh.c`)：
   - 计算图像梯度 (gx, gy)，基于梯度幅值阈值分割
   - 使用并查集进行连通分量标记
   - 对连通区域拟合线段，组合成候选四边形
   - 验证四边形（角度检查、边缘拟合误差）

2. **Tag 解码** (`apriltag.c`)：
   - 通过单应性矩阵将四边形内的像素映射到标准 Tag 坐标
   - 按位单元采样亮度（使用灰度模型回归确定局部的黑/白阈值）
   - 解码二进制有效载荷，与 Tag 族编码比对
   - 检查 Hamming 距离（容许一定数量的位错误）

3. 返回 `zarray_t*`（检测结果数组），每个元素为 `apriltag_detection_t`（id、hamming 距离、角点坐标、单应性矩阵等）。

### 3.3 位姿估算（C 库）

对每个检测到的 Tag 调用 `estimate_tag_pose(&info, &pose)`，内部采用三方法组合：

1. **基于单应性矩阵的初始估计** (`homography_to_pose()`)：
   - 从检测阶段得到的单应性矩阵 H 和相机内参，恢复 4×4 位姿矩阵
   - 平移缩放至 `tag_size / 2`
   - 应用固定方向修正矩阵

2. **正交迭代细化** (Lu 2000 方法)：
   - 以单应性估计为初始值，进行最多 50 次迭代
   - 每次迭代：计算投影算子 → 更新平移 → 使用 SVD 更新旋转（强制 det(R) = +1）
   - 返回对象空间误差 `err1`

3. **位姿歧义消除** (Schweighofer & Pinz 2006 方法)：
   - 单应性方法可能收敛到局部极小值
   - 构造四阶多项式寻找替代极小值，使用 Newton-bisection 混合法求解
   - 返回替代位姿的误差 `err2`
   - 选择 `err1` 和 `err2` 中较小的作为最终位姿

返回的 `apriltag_pose_t` 包含旋转矩阵 R (3×3) 和平移向量 t (3×1)，均相对于相机坐标系。

### 3.4 误差过滤

```cpp
if (err > 1e-3)  // 对象空间误差阈值
    continue;     // 跳过该 Tag
```

此阈值严格，可有效过滤误检和低质量位姿估计。

### 3.5 坐标系转换链

单个 Tag 位姿经过以下变换得到最终船坞位姿：

```
final_pose = camera2camera_link × tag_pose × camera_tag2ros_ × dock_offset[tag_id]
```

其中各变换的含义：

| 变换 | 类型 | 含义 |
|------|------|------|
| `tag_pose` | AprilTag 检测位姿 | Tag 在相机坐标系中的位姿（AprilTag 库输出） |
| `camera2camera_link` | 固定常量 | 将 AprilTag 相机系映射到 ROS `camera_link` 系 |
| `camera_tag2ros_` | 固定常量 | 将 AprilTag 的 Tag 局部坐标系旋转到 ROS 坐标系 |
| `dock_offset[tag_id]` | YAML 配置 | 从 Tag 中心指向对应泊位 (bay) 中心的偏移 |

**camera2camera_link** 的旋转矩阵：

```
[ 0,  0,  1]
[-1,  0,  0]
[ 0, -1,  0]
```

**camera_tag2ros_** 的旋转矩阵 (Ry(-90°) × Rx(90°))：

```
相当于将 AprilTag 的 Tag 坐标系（Z 轴垂直 Tag 平面朝外）旋转到 ROS 标准坐标系。
```

### 3.6 多 Tag 位姿平均

当检测到多个 Tag 时，使用 **Markley SVD 方法** 进行旋转平均：

1. **平移**：算术平均（所有 Tag 对应的 dock 位姿平移量取均值）
2. **旋转**：Markley SVD 方法
   - 将各四元数转换为 `(w, x, y, z)` 向量
   - 处理四元数双覆盖（确保与参考四元数的点积为正）
   - 构造矩阵 `M = Σ(v_i × v_i^T)`
   - 对 M 做特征分解，最大特征值对应的特征向量即为平均四元数
   - 归一化得到最终平均旋转

相比简单的四元数平均，Markley SVD 方法在数值上更稳定。

### 3.7 结果发布

- 有检测结果：发布 `Float64MultiArray`，data 字段为 `[x, y, z, roll, pitch, yaw]`，长度 m，角度 rad
- 无检测结果：发布空数组（`data` 为空），供下游判断定位是否有效
- 如 >= 2 个 Tag，额外日志输出每个 Tag 的独立位姿

---

## 4. 参数配置

### 4.1 Launch 参数

```bash
ros2 launch apriltag_localization apriltag_localization.launch.py profile:=sim
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `profile` | string | `sim` | `sim` → 加载 `detection_cfg_sim.yml`；`real` → 加载 `detection_cfg.yml` |

### 4.2 YAML 配置参数

配置文件位于 `config/`，安装至 `install/apriltag_localization/config/`。

#### 4.2.1 必需参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `camera_info_topic` | string | `CameraInfo` 话题名，用于获取相机内参 (fx, fy, cx, cy) |
| `image_topic` | string | 图像话题名，编码须为 `bgr8` (或可被 `cv_bridge` 转换) |
| `detection_result_topic` | string | 定位结果发布话题，类型 `Float64MultiArray` |
| `frame_id` | string | 结果帧 ID，须与 TF 树中相机帧一致 |
| `apriltag_family_name` | string | Tag 族名称，支持见 [4.3](#43-支持的-tag-族) |
| `tag_size` | double | Tag 物理边长，单位 **米**，须与实际打印/贴图尺寸严格一致 |
| `tag_ids` | int[] | 期望检测的 Tag ID 列表 |

#### 4.2.2 dock_offset 参数（每个 tag_id 一组）

格式为 `tag_{id}.dock_offset_*`：

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `tag_{id}.dock_offset_x` | double | m | Tag 坐标系 X 方向偏移 |
| `tag_{id}.dock_offset_y` | double | m | Tag 坐标系 Y 方向偏移 |
| `tag_{id}.dock_offset_z` | double | m | Tag 坐标系 Z 方向偏移 |
| `tag_{id}.dock_offset_roll` | double | **度** | 绕 X 轴旋转（节点内部自动转弧度） |
| `tag_{id}.dock_offset_pitch` | double | **度** | 绕 Y 轴旋转 |
| `tag_{id}.dock_offset_yaw` | double | **度** | 绕 Z 轴旋转 |

**注意**：
- dock_offset 在 **Tag 坐标系** 下定义，表示从 Tag 中心指向对应泊位 (bay) 中心的变换
- 角度参数在 YAML 中填**度**，节点内部通过 `degreesToRadians()` 转为弧度
- 如某 tag_id 的任意一个 dock_offset 参数缺失，该 id 会被跳过（打印 WARN 日志），但不会阻止节点启动

#### 4.2.3 完整配置示例

```yaml
apriltag_node:
  ros__parameters:
    # --- 订阅/发布话题 ---
    camera_info_topic: "/roboboat/sensors/camera/camera_info"
    image_topic: "/roboboat/sensors/camera/image"
    detection_result_topic: "/apriltag_node/dock_pose"
    frame_id: camera_link

    # --- Tag 配置 ---
    apriltag_family_name: tag36h11
    tag_size: 0.5
    tag_ids: [0, 43]

    # --- Tag ID 0：placard1 → bay1 ---
    tag_0:
      dock_offset_x: -3.0
      dock_offset_y: -1.52
      dock_offset_z: -4.52
      dock_offset_roll: -90.0
      dock_offset_pitch: -90.0
      dock_offset_yaw: 0.0

    # --- Tag ID 43：placard2 → bay2 ---
    tag_43:
      dock_offset_x: -3.0
      dock_offset_y: -1.52
      dock_offset_z: -4.52
      dock_offset_roll: -90.0
      dock_offset_pitch: -90.0
      dock_offset_yaw: 0.0
```

### 4.3 支持的 Tag 族

| 族名称 | 位数 | Hamming 距离 | 说明 |
|--------|------|-------------|------|
| `tag36h11` | 36 | 11 | **默认推荐**，本项目使用 |
| `tag25h9` | 25 | 9 | 较小，适合近距离 |
| `tag16h5` | 16 | 5 | 最小，适合极近距离 |
| `tagCircle21h7` | 21 | 7 | 圆形 Tag |
| `tagCircle49h12` | 49 | 12 | 圆形 Tag，误检率最低 |
| `tagCustom48h12` | 48 | 12 | 自定义 |
| `tagStandard41h12` | 41 | 12 | 标准 |
| `tagStandard52h13` | 52 | 13 | 标准，误检率最低 |

选择建议：Hamming 距离越大，误检率越低；Tag 位数越多，可区分 ID 越多。本项目使用 `tag36h11` 是平衡选择。

### 4.4 检测器内部参数（固定值，未暴露为 ROS 参数）

| 参数 | 值 | 说明 |
|------|-----|------|
| `nthreads` | 2 | 检测器工作线程数 |
| `quad_decimate` | 1.0 (默认) | 图像降采样因子，>1 加快检测但降低对小 Tag 的灵敏度 |
| `quad_sigma` | 0.0 (默认) | 高斯模糊 σ，>0 可平滑噪声 |
| `refine_edges` | 1 (默认) | 是否细化四边形边缘 |
| `decode_sharpening` | 0.25 (默认) | 解码时锐化强度 |
| 对象空间误差阈值 | 1e-3 | `estimate_tag_pose()` 返回误差超过此值则丢弃该检测 |
| 检测循环超时 | 2000 ms | `main_loop()` 等待新图像的超时 |

如需修改检测器参数（quad_decimate、quad_sigma 等），需修改 `apriltag_node.cpp` 中 `initModel()` 函数对 `td_ptr_` 成员的赋值。

---

## 5. 输出

| 属性 | 值 |
|------|-----|
| 话题 | `/apriltag_node/dock_pose` (可配) |
| 消息类型 | `std_msgs/msg/Float64MultiArray` |
| data 内容 | `[x, y, z, roll, pitch, yaw]` |
| 平移单位 | 米 (m) |
| 角度单位 | 弧度 (rad) |
| 坐标系 | 相机坐标系 (`camera_link` / `camera_left_link`) |
| 语义 | 船坞泊位 (bay) 中心在相机坐标系下的位姿 |
| 无检测时 | 发布空数组 (data.size() = 0) |

---

## 6. 坐标系说明

### 6.1 涉及的坐标系

```
世界系 (world)
  └── 相机系 (camera_rear_link / camera_left_link)    ← 输出位姿所在坐标系（仿真归港用后向）
        └── Tag 系 (AprilTag 局部坐标系)          ← 检测结果
              └── Bay 系 (泊位中心)               ← 经 dock_offset 变换后
```

### 6.2 仿真与实船的差异

| 项目 | 仿真 (sim) | 实船 (real) |
|------|-----------|------------|
| 相机话题前缀 | `/roboboat/sensors/camera_rear/`（归港） | `/zed/zed_node/rgb/color/rect/` |
| frame_id | **`camera_rear_link`** | `camera_left_link` |
| tag_size | 0.5 m | 2.0 m |
| 配置文件 | `detection_cfg_sim.yml` | `detection_cfg.yml` |

### 6.3 frame_id 与下游对接

下游 `usv_docking` 通过 TF 将 `/apriltag_node/dock_pose` 从 `frame_id` 变换到 `base_link`，因此 `frame_id` 须与 URDF/TF 树中的相机帧名称严格一致。

---

## 7. dock_offset 计算方法

dock_offset 是**在 Tag 坐标系下**从 Tag 中心到 bay 中心的偏移。仿真中基于 VRX `dock_2022` 模型：

```
bay1 中心  = (1.50, 3.00, 0.25)   # 取自 dock_2022/model.sdf 的 PerformerDetector pose
bay2 中心  = (1.50, 9.00, 0.25)
Tag 0 位置 = placard1 的 link_symbols pose  (在 world 中)
Tag 43 位置 = placard2 的 link_symbols pose

dock_offset = Tag 坐标系下的 (bay_center - tag_position)
```

Tag 位姿/尺寸或 bay 中心变化时，需重新计算 dock_offset 并同步更新 YAML。

---

## 8. 常见问题

### 8.1 Tag 无法识别

1. 确认贴图/打印的 Tag 是 **AprilTag**（非 Aruco/QR），且族名为 `tag36h11`
2. 确认 `tag_size` 与实际物理尺寸一致（Gazebo 中与 `<plane><size>` 匹配）
3. 确认图像清晰、Tag 在视野内且未被遮挡
4. 可用 [tagsgen.top](https://tagsgen.top/) 生成正确的 tag36h11 PNG

### 8.2 位姿误差大

1. `tag_size` 不准确——这是位姿估算中唯一的尺度参考，必须精确
2. `CameraInfo` 内参不准确——确保 `fx, fy, cx, cy` 来自正确的标定结果
3. Tag 在图像中过小——增大 Tag 尺寸或缩短距离

### 8.3 无输出 / 发布空数组

1. 检查 `dock_pose_ext_map_` 中是否包含检测到的 Tag ID（日志会打印 `TAG ID xx is not exist in config map`）
2. 检查对象空间误差是否超过 1e-3 阈值
3. 确认 `camera_info_topic` 和 `image_topic` 都在正常发布

### 8.4 修改配置后不生效

配置文件的安装路径在 `install/apriltag_localization/config/`，修改源码 `config/` 后需重新 `colcon build`。

---

## 9. 相关文档

| 文档 | 内容 |
|------|------|
| [AprilTag船坞定位接口.md](AprilTag船坞定位接口.md) | 接口快速参考与使用流程 |
| [../../docs/仿真码头与AprilTag配置.md](../../docs/仿真码头与AprilTag配置.md) | dock_2022 几何、Tag 贴图与 TF 联调 |
| [../../usv_docking/README.md](../../usv_docking/README.md) | 下游归港入泊消费者 |
| [../../docs/usv_docking任务规划.md](../../docs/usv_docking任务规划.md) | 归港状态机与安全逻辑 |
