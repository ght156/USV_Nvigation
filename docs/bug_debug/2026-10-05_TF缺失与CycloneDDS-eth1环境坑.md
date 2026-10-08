# 2026-10-05 "看得见用不了"：TF 缺失致归港 ACQUIRE 空转 + CycloneDDS eth1 环境坑

> 现象 1：归港一直在 ACQUIRE_TAG 搜索自转、60s 超时，但 apriltag 检出率 100%、相机正常。
> 现象 2：ssh 执行的 `ros2` CLI / 新起节点经常收不到话题、服务不可用，时好时坏。
> 两个都是环境/配置问题，不是算法 bug。

## 问题 1：缺 URDF 静态 TF（根因）

**链式故障**:

1. `start_xingfei_stack.sh` 的 localization 项写的 `publish_robot_tf:=false` → robot_state_publisher 从未启动；
2. URDF 静态 TF（`base_link→camera_back_video_link`）缺失；
3. apriltag 节点发的 TF 是 `camera_back_video_link→dock_frame`（detections 话题 100% 正常）;
4. 归港位姿估计器查 `base_link→dock_frame` 需要经 `base_link→camera_back_video_link` 桥接 → **解算失败，`pose_source` 永远 INVALID**;
5. ACQUIRE 拿不到有效位姿 → 开环搜索自转 → 60s `ACQUIRE_TIMEOUT`。

**迷惑点**：检测数据一直正常（`objects` 非空），现场容易误判为"相机/光照问题"。当天傍晚太阳逆光确实有影响（检出为空），但主因是 TF。

**排查动作**（以后"看得见用不了"先查这个）:

```bash
ros2 run tf2_ros tf2_echo base_link camera_back_video_link   # URDF 静态段
ros2 run tf2_ros tf2_echo base_link dock_frame               # 完整链
```

**修复**：栈脚本 localization 项改 `publish_robot_tf:=true`（其 launch 内含 robot_state_publisher + 兴飞 xacro)，重启 localization 后 `pose_source: VISION`。

## 问题 2：CycloneDDS 限定 eth1,ssh 非交互环境缺 CYCLONEDDS_URI

- 船上 `~/.bashrc` 把 CycloneDDS 限定到 **eth1**（船内网，飞控 192.168.2.14 所在网段）;
- **ssh 非交互执行不读 .bashrc**，只设 `ROS_DOMAIN_ID/RMW_IMPLEMENTATION` 时，新节点走默认网卡选择，与 eth1 上的节点互相发现不了/部分通——表现为"ros2 topic echo 偶发空、服务等待超时、桥收不到 /mavros/state";
- **修复**：栈脚本内生成并 source `~/logs/xingfei_stack/.cyclone_env`（内容与 .bashrc 一致）。ssh 手动操作先 `source ~/logs/xingfei_stack/.cyclone_env`。

## 相关提醒

- eth1(192.168.2.101）是船内网，**这根线不能拔**：飞控和 CycloneDDS 都在上面；eth0 是岸上调线，随便拔。
- ros2 CLI 在船上偶发卡死（已知），话题诊断优先用 rclpy 小脚本（`/tmp/check_bridge2.py` 可作模板）。
