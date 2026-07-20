# usv_docking v4 快照（GNSS v5 修改前）

**备份时间**：2026-07-06  
**备份目录**：`src/usv_docking/.backup_v4_pre_gnss_20260706/`

## 版本标识

| 项 | 值 |
|---|---|
| 控制器版本 | **v4.0** — Tag 两阶段（虚拟入口深度 → 真中心） |
| 包版本 | `0.1.0` |
| 备份原因 | 实现 v5：预泊点后 GNSS dock_enu 逼近虚拟入口 + Phase1.5 Tag 搜获/对齐 |

## v4 状态机（备份时）

```text
PRECHECK → WAIT_TAG → SEARCH_SPIN?
  → APPROACH_ENTRY（Tag 倒车至 entry_standoff_m）
  → ALIGN_ENTRY → BACK_IN → STOP
```

## 备份文件清单

| 文件 | 说明 |
|---|---|
| `usv_docking/docking_controller.py` | 主状态机（62 KB） |
| `usv_docking/dock_feedback.py` | phase / error code |
| `config/docking_controller_sim.yaml` | 仿真参数 |
| `config/docking_controller_real.yaml` | 实船参数 |
| `README.md` | 包说明 |
| `dock_database.yaml` | 泊位几何（dock_mission 共享） |

## 恢复方法

```bash
BACKUP=src/usv_docking/.backup_v4_pre_gnss_20260706
cp "$BACKUP/usv_docking/docking_controller.py" src/usv_docking/usv_docking/
cp "$BACKUP/usv_docking/dock_feedback.py" src/usv_docking/usv_docking/
cp "$BACKUP/config/"*.yaml src/usv_docking/config/
cp "$BACKUP/README.md" src/usv_docking/
```

## v5 计划变更摘要

1. 新增 GNSS 阶段：`GNSS_HEADING_ALIGN` → `GNSS_BACK_TO_ENTRY` → `GNSS_ENTRY_SETTLE`
2. 新增 Phase1.5：`VISION_SEARCH_TAG` → `SEARCH_SPIN` → `ALIGN_ENTRY`
3. 删除主路径：`WAIT_TAG` / `APPROACH_ENTRY`（`gnss_approach_enabled:=false` 可回退 v4）
4. 新增模块：`gnss_approach.py`、`bay_config.py`
5. 几何目标：`dock_database.yaml` → `entry_point_dock_enu`
