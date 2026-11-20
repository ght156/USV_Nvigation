# dock_mission 测试

## 完整测试

```bash
source /opt/ros/humble/setup.bash
cd /home/ght/USV_NAV
colcon build --symlink-install --packages-select dock_mission
source install/setup.bash
colcon test --packages-select dock_mission --event-handlers console_direct+
```

测试由两层组成：

| 文件 | 覆盖 |
|------|------|
| `test_dock_mission_core.cpp` | 泊位 YAML、GNSS 反解、任务事件、GCS 指令、上层 JSON、在坞判定、低频停稳门 |
| `test_cpp_param_defaults.py` | C++ 声明与 YAML 配置面一致 |
| `test_cpp_migration.py` | C++ ELF、构建依赖、ROS 时钟与运行时去 Python 化 |
| `test_cpp_ros_integration.py` | 真实 Trigger/Service/Topic：预泊请求、任务完成、停稳交接、成功 ACK、取消和在坞查询 |

集成测试在独立 DDS domain 中启动被测 C++ 节点，模拟周边 `mission_bridge`、Nav2 参数服务、事件 ACK 与 AprilTag 通道服务，不需要实船或仿真。

## 手动冒烟

需另开终端启动 `dock_mission_node`，再运行：

```bash
ros2 launch dock_mission dock_mission.launch.py
./test/integration/test_dock_home_smoke.sh
```
