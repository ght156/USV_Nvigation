# 2026-09-30 兴飞后视 AprilTag 与船坞中心不对称排查

## 1. 基本信息

- 日期：2026-09-30
- 船：兴飞（`ssh hkzl`，MEC-MB002），ROS_DOMAIN_ID=5 / cyclonedds
- 工作区：`~/jzw_ws/install`（感知+TF 部署）、`~/usv-main`（开发主工程）
- 运行节点：`perception_launch cleaning_boat_xingfei.launch.py`（后视 UVC + apriltag_localization）、`usv_transforms publish_tf.launch.py`、`usv_rs485_driver`
- 数据：`/apriltag_node/detections`、TF（`apriltag_0/1`、`dock_frame`）、后视图像一帧（1920×1080）

## 2. 现象

后视感知起起来后，RViz/图像里二维码位置与船坞中心看起来不对称、不在中间，
怀疑相机装偏或配置错误。

## 3. 证据

处理链路自洽性检查（全部通过）：

| 检查项 | 实测 | 配置 | 结论 |
| --- | --- | --- | --- |
| 两个 tag 各自推算的船坞中心 | 相差仅 2.9 cm、姿态差 ~1° | — | 融合自洽 |
| 两 tag 实测间距 | 1.261 m | 1.268 m（±0.634×2） | 吻合（差 7 mm） |
| 图像中两面板的像素位置 | 与 TF 几何互相印证（fx=920） | — | 内参无问题 |
| 运行节点实际加载的偏移参数 | tag_0 y=+0.634、tag_1 y=-0.634、z=-0.52（`ros2 param dump`） | 同左 | 加载的就是这份配置 |

实测几何（相机系 `camera_back_video_link`）：

```text
tag0 原始位姿:  (2.177, -0.735, -0.456)
tag1 原始位姿:  (2.233, +0.525, -0.485)
dock_frame(融合): (2.055, -0.113, -0.966)
```

换算到 base_link（相机外参 xyz=(1.58, 0, 0.46)、rpy=(0, -17°, 180°)，TF 实测一致）：

```text
dock_frame ≈ (-0.67, +0.11, +0.14)
```

即船坞中线偏船中线 **+y 约 11 cm**，船-坞相对角约 **2°**。
后视图像（/tmp 抓帧）中两块二维码面板中点在图像中心右侧约 80 px ≈ 2.7°，
与 TF 几何完全对上 —— 偏的是真实物理几何，不是解算错误。

## 4. 根因

待验证。处理链路（内参、tag 尺寸、双 tag 融合、TF 外参加载）均无 bug，
偏差只可能来自物理安装层面，三选一：

1. **二维码面板在坞上装得不对称**：配置假设"两面板中点 = 船坞中心"
   （tag_0/tag_1 的 `dock_offset_y` = ±0.634），若实际安装不对称则 dock_frame 偏；
2. **后视相机 yaw 安装角偏 ~2.7°**：xacro 写的是精确 180°，实际支架可能有偏；
3. **船当时就没停正**：船-坞横向偏 11 cm、相对角 2° 也可能就是真实位姿。

## 5. 已做修改

按现场要求（江志伟），以"船当前已正确停入船坞"为基准，把 dock 中心对齐 base_link
的修正量吸收进 tag 偏移（不改相机外参）：

- 船端 `~/jzw_ws/install/apriltag_localization/config/detection_cfg.yml`
  （原件备份为 `detection_cfg.yml.bak_20260930`）
- 本地 `src/apriltag_localization/config/detection_cfg.yml`（同步）

新值（由同一时刻两 tag 原始位姿反解，回代校验 dock_frame 在 base_link = (0,0,0)）：

| 参数 | tag_0 | tag_1 | 旧值（对称） |
| --- | --- | --- | --- |
| dock_offset_x | -0.666 | -0.688 | 0.0 |
| dock_offset_y | +0.724 | -0.540 | ±0.634 |
| dock_offset_z | -0.649 | -0.649 | -0.52 |
| dock_offset_r/p/y (°) | -0.19 / -0.25 / -0.26 | +0.21 / +0.34 / +0.15 | 0 |

注意：该标定把**当前停靠状态**当作"对中基准"。若标定时船实际没停正
（横向/前后/朝向有偏），偏差会固化进偏移量，影响后续归港终点精度；
后续若实测发现归港停位系统性偏移，需重新标定或改查相机外参。

## 6. 待验证 / 下一步

1. 重启感知 launch（参数只在启动时加载）后验证：

```bash
source /opt/ros/humble/setup.bash && source ~/jzw_ws/install/setup.bash
ros2 run tf2_ros tf2_echo base_link dock_frame     # 平移应 ≈ (0, 0, 0)
ros2 topic echo --once /apriltag_node/detections   # 两 tag 推算的 dock 位姿应重合
```

2. 实船归港一次，确认 FINAL_DOCK 停位（base_link 到 x≈0）后船在坞里是对中的；
   若系统性偏前/后/侧，按偏差方向微调 dock_offset_x/y（两 tag 同向调）。
3.（可选）卷尺复核：面板间距、面板到坞中心距离、面板中心到坞中心高度差、
   打印二维码黑框外边尺寸（验证 `tag_size: 0.25`）。
