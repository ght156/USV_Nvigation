# apriltag_localization

AprilTag 船坞定位：订阅**后向**相机图像（仿真），发布 **bay 中心** 在 `camera_rear_link` 下的位姿，供 **`usv_docking`** 消费。

## 配置文件

| profile | 文件 | 用途 |
|---------|------|------|
| **sim**（launch 默认） | `config/detection_cfg_sim.yml` | Gazebo **后向**相机；单 bay 双码 id 0/43 |
| **real** | `config/detection_cfg.yml` | ZED RGB / `camera_left_link` |

**坐标系**：`dock_offset_*` 只按 **dock 模型 SDF 坐标** 标定（经 `camera_tag2ros_` 链），不用 odom/map 船位。详见 [`docs/AprilTag船坞定位接口.md`](docs/AprilTag船坞定位接口.md)。

## 编译

```bash
cd ~/wuxihik_navigation
colcon build --packages-select apriltag_localization
source install/setup.bash
```

## 启动（仿真）

须在 Gazebo + converter 栈之后、**usv_docking 之前**：

```bash
ros2 launch apriltag_localization apriltag_localization.launch.py profile:=sim use_sim_time:=true
```

## 输出

| 话题 | 类型 | 说明 |
|------|------|------|
| `/apriltag_node/dock_pose` | `Float64MultiArray` | bay 中心在 **`frame_id` 相机系** 下 `[x,y,z,roll,pitch,yaw]`（rad） |

双码（0/43）同屏时取平均；单码亦可输出。节点**持续发布**（无检测时发空数组），供 `usv_docking` 传感器看门狗判断话题是否存活。

## 与 usv_docking 联调

- `usv_docking` 的 `camera_frame` / `dock_pose_topic` 须与本包 `frame_id` / `detection_result_topic` 一致。
- 仿真默认：`camera_rear_link` + `profile:=sim`；实船：`camera_left_link` + `profile:=real`。
- 状态监控：`ros2 topic echo /dock/status` 查看 `phase`、`error_code`、`motion_sensors_ok`。

## 标定

- 改 SDF / tag pose / `tag_size` 后运行：  
  `python3 src/apriltag_localization/scripts/compute_dock_offset_sim.py`
- 联调阶段可 **分 tag 单码** 微调 `dock_offset_y`（见 [`docs/仿真码头与AprilTag配置.md`](../../docs/仿真码头与AprilTag配置.md)）

## 相关文档

| 文档 | 内容 |
|------|------|
| [`docs/AprilTag船坞定位接口.md`](docs/AprilTag船坞定位接口.md) | dock_offset 计算与接口 |
| [`docs/仿真码头与AprilTag配置.md`](../../docs/仿真码头与AprilTag配置.md) | world 布局、tag36h11 贴图 |
| [`../usv_docking/README.md`](../usv_docking/README.md) | 归港消费者与全栈命令 |
| [`docs/项目运行与联调.md`](../../docs/项目运行与联调.md) §归港入泊 | 7 终端联调 |
