# 修改说明

本次修改采用“默认保留旧逻辑、参数开启增强逻辑”的兼容方式。

## 修改文件

1. `include/apriltag_node.h`
   - 新增 TagObservation、DockPerception 数据结构。
   - 新增双 Tag 融合、单 Tag 降级、入口位姿和诊断发布接口。
   - 新增增强模式参数、历史可靠 yaw 状态和安全退出标志。

2. `src/apriltag_node.cpp`
   - `use_baseline_yaw=false` 时保留原多 Tag dock 位姿平均逻辑。
   - 开启后固定使用 `baseline_tag_id_a -> baseline_tag_id_b`，不再由距离决定基线方向。
   - 配置仍按原语义 `T_tag_dock_center` 读取；理论基线通过逆变换得到 Tag 在 dock 坐标系的位置。
   - 双 Tag 中心一致时使用基线 yaw；单 Tag 或双 Tag 冲突时使用近 Tag 位置，并可保持历史基线 yaw。
   - 新增虚拟入口位姿和 `/dock/perception` 17 元素数组。
   - `max_pose_error`、`max_hamming` 参数化。
   - 修复未知 Tag 未跳过、AprilTag pose 矩阵未释放、检测线程析构不安全问题。

3. `config/detection_cfg.yml`
4. `config/detection_cfg_sim.yml`
   - 新增增强模式参数，默认关闭，因而升级后不会自动改变旧控制链输出。

5. `docs/dual_tag_baseline_mode.md`
   - 新增模式、降级规则和数组协议说明。

6. `README.md`
   - 补充旧模式兼容、新模式启用和新话题说明。

7. `CMakeLists.txt`、`package.xml`
   - 显式补齐 `sensor_msgs`、`std_msgs`、`geometry_msgs`、`tf2` 等依赖。

## 启用方式

在对应配置中改为：

```yaml
use_baseline_yaw: true
```

并按实际船坞坐标系填写：

```yaml
dock_entry_offset_x: 0.0
dock_entry_offset_y: 0.0
dock_entry_offset_z: 0.0
```

首次实船测试建议同时记录 `/apriltag_node/dock_pose` 与 `/dock/perception`，确认 `yaw_source=1` 时的方向和船坞中心位置正确后，再让归港控制器使用动态入口。
