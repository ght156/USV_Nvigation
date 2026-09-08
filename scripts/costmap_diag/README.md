# costmap_diag — Nav2 避障/costmap 诊断工具

2026-09-02 实船避障调试时写的采样脚本。在船上直接跑（`ssh nx` 后）或在能直连船端图（ROS_DOMAIN_ID=5、CycloneDDS）的机器上跑；脚本只读，不改任何参数。

## 文件

| 文件 | 作用 |
|------|------|
| `watch_avoid.py` | 一次性快照：costmap lethal/inflated 格数、最近障碍距离、`/cmd_vel_nav` 当前值、nav_to_pose/follow_waypoints 是否有活动目标 |
| `ghost_watch.py [秒数]` | 连续监视（默认 40s）：逐条 costmap 消息打印 lethal 数/最近障碍，看假障碍是否间歇性出现 |
| `ghost_diag.py` | 水面假障碍定位：原始点云变换到 odom 后过当前滤波链，统计幸存点 z/r 分布 + costmap lethal 的距离环/扇区散布（扇区多=水面噪声，扇区少=真实目标） |
| `align_diag.py` | 对齐诊断：local costmap 障碍转 map 系后与 global costmap 静态障碍比对（距离+偏移向量），低河岸点云 z 分带，GNSS fix 状态 |
| `marked_probe.py` | 采样 `/local_costmap/voxel_marked_cloud`（若发布）统计真正打标点的 z/r 分布 |

## 用法（船上）

```bash
source /opt/ros/humble/setup.bash && source ~/usv_nav_ws/install/setup.bash
export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 scripts/costmap_diag/watch_avoid.py
python3 scripts/costmap_diag/ghost_watch.py 40
python3 scripts/costmap_diag/ghost_diag.py
python3 scripts/costmap_diag/align_diag.py
```

注意：`align_diag.py`/`ghost_diag.py` 里的滤波链阈值（`R_MIN/R_MAX/Z_OBS_MIN/...`）是脚本内常量，
改了 `nav2_params_real_mavros.yaml` 的避障参数后要同步改脚本顶部常量。
