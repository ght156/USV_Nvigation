# 双 Tag 基线增强模式

## 兼容策略

`use_baseline_yaw: false` 为默认值。此时 `/apriltag_node/dock_pose` 仍使用原算法：每个有效 Tag 经原坐标链换算为 dock 中心位姿，再对全部结果求平均；无检测时发布空数组。

设为 `true` 后，旧话题的类型与六元素格式保持不变，但位姿改为增强融合结果；同时发布 `/dock/perception`。

## 固定基线方向

基线始终按 `baseline_tag_id_a -> baseline_tag_id_b` 计算，默认 `0 -> 43`，不会因左右 Tag 距离变化而交换方向。配置中的 `dock_offset_*` 沿用原语义 `T_tag_dock_center`，理论 Tag 基线通过其逆变换 `T_dock_center_tag` 计算。

## 降级规则

双 Tag 中心一致且基线有效时，位置由双 Tag 融合，yaw 由基线计算，并保存为最近可靠 yaw。只看到一个 Tag 或双 Tag 一致性失败时，位置使用最近 Tag；若 `hold_baseline_yaw_on_single=true` 且历史基线存在，则继续保持历史 yaw，否则使用当前单 Tag yaw。

`yaw_source`：0=当前单 Tag；1=当前双 Tag 基线；2=历史双 Tag yaw 保持。

## `/dock/perception` 数组格式

`Float64MultiArray` 共 17 个元素：

```
[0..5]   center: x,y,z,roll,pitch,yaw
[6..11]  entry:  x,y,z,roll,pitch,yaw
[12]     tag_count
[13]     yaw_source
[14]     dual_consistent (0/1)
[15]     valid (0/1)
[16]     dual center difference, metres
```

虚拟入口由 `T_camera_dock_center * T_dock_center_entry` 得到，对应 `dock_entry_offset_*` 参数。
