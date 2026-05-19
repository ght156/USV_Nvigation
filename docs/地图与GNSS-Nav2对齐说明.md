# 地图、GNSS 与 Nav2 对齐说明（实船）

归纳 **栅格海图、`ref_gnss*`、PX4 HOME、`gnss_odom_map_tf`、航点** 的关系。日常启动见 [`项目运行与联调.md`](./项目运行与联调.md)。

> 历史设计笔记（navsat+EKF 仿真方案）已归档删除；实船默认 **不用 EKF**，用 **map YAML 固定角点 + `gnss_odom_map_tf`** 装订 `map→odom`。

---

## 1. 栅格地图

- **默认**：`workspace_nav/config/map_real_boat_hk.yaml` → `map/hk_map.pgm`（无锡 HK 园区）。  
- **白板图**（几乎全白）：全局规划约束弱，**主要靠 local_costmap + Livox** 避障。  
- **带岸线/禁区**：须在 PGM 标占据区，并核对 **`origin` / `ref_gnss*`** 与现场一致。

---

## 2. `map` 与 `odom` 不会自动对齐

- **`/mavros/local_position/odom`** 原点一般为 **PX4 HOME**，不会随换 Nav2 海图自动变。  
- **`gnss_odom_map_tf`**（默认开启）用 **GNSS + 局域 odom** 计算 **`map→odom`**，锚点来自 **`map_config_yaml`** 的 **`map_origin_ref_key`**（默认 `ref_gnss_10`）。  
- 关 **`use_gnss_map_odom_tf:=false`** 时退化为静态 `map→odom`，更依赖 **HOME 与锚点一致**。

---

## 3. `map.yaml` 字段

```yaml
image: ../map/hk_map.pgm
resolution: 1.0
origin: [0.0, 0.0, 0.0]    # 栅格参考角在 map 系位姿
ref_gnss_10: [lon, lat]      # 与 map 原点 (0,0) 对应的角点（默认 datum 键）
```

- **`ref_gnss_*`**：每条为 **`[longitude, latitude]`**（度）。  
- **`origin`**：Nav2 将栅格角点放在 map 系的位姿；**`origin=[0,0,0]`** 时常见为角点与 `map`(0,0) 重合。  
- 船在 map 上的位置来自 **TF**（`map→odom→base_link`），不是自动等于某角点。

---

## 4. 实船对齐 checklist

| 链路 | 配置 |
|------|------|
| Nav2 载入海图 | `nav2_real_mavros.launch.py` → **`map:=`** |
| `map→odom` | bringup → **`map_config_yaml:=`**（与上相同） |
| 航点经纬→map x,y | `waypoint_transform` → **`map_yaml_path`** / 默认同包内 yaml |
| 局域原点 | QGC 或 **`/mavros/cmd/set_home`** 对齐 **`ref_gnss*`** |

**核心**：上述项共用 **同一份 YAML、同一 `ref_gnss*` 角点**。

---

## 5. `navsat.yaml`（可选）

本仓 **默认不运行** `navsat_transform`。若将来补 EKF 链，`config/navsat.yaml` 的 **`datum`** 须与所选 **`ref_gnss*`** 的 lat/lon 一致。

---

## 6. 相关文档

| 文档 | 内容 |
|------|------|
| [`工作进度汇报.md`](./工作进度汇报.md) | 航点/map 坐标改版记录 |
| [`实船调试.md`](../src/USV_NAV/docs/实船调试.md) | HOME、`gnss_odom_map_tf` 算法要点 |
| [`PROJECT_ARCHITECTURE_AND_NAV2.md`](../src/USV_NAV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md) | 数据流图 |
