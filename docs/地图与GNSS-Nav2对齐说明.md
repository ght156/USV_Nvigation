# 地图、GNSS 与 Nav2 对齐说明

归纳 **栅格海图、`ref_gnss*`、datum、EKF/航点** 的关系。日常仿真启动见 [`项目运行与联调.md`](./项目运行与联调.md)。

**实船**（PX4 HOME、`gnss_odom_map_tf`、MAVROS 里程计）在 **`USV_NAV`** 仓库说明；本文件以 **本仓仿真默认** 为主。

---

## 1. 栅格地图

- **仿真当前默认海图**：`src/YILDIZ-USV/workspace_nav/config/map_hk.yaml` 及 yaml 内 `image:` 指向的 PGM。  
- **白板图**：全局规划约束弱，**主要靠 local_costmap + LaserScan**（`/roboboat/sensors/lidar/scan`）避障。  
- **带岸线/禁区**：须在 PGM 标占据区，并核对 **`origin` / `ref_gnss*`** 与 `navsat.yaml` **datum** 一致。

---

## 2. 仿真：`map` 与 `odom`

- **`robot_localization` EKF** 输出 **`/odometry/filtered`**，提供 **`odom` → `base_link`**（见 `ekf.yaml`）。  
- **`navsat_transform_node`**（由 `localization.launch.py` 启动）在 **固定 datum** 下协助全球坐标与图的一致性；**`navsat.yaml` 的 datum** 应与地图 **`ref_gnss*`**（或所选角点）一致。  
- 换图后请同时核对 **地图 yaml**、`navsat.yaml`、航点节点的 **`map_yaml_path`**。

---

## 3. `map.yaml` 字段示例

```yaml
image: ../map/your_map.pgm
resolution: 1.0
origin: [0.0, 0.0, 0.0]    # 栅格参考角在 map 系位姿
ref_gnss_10: [lon, lat]     # 与 map 约定角点对应的 [经度, 纬度]（度）
```

- **`ref_gnss_*`**：每条为 **`[longitude, latitude]`**（度）。  
- **`origin`**：Nav2 `map_server` 将栅格角点放在 map 系的位姿。  
- 船在地图上的位置来自 **TF**：`map` → `odom` → `base_link`。

### `map_shanxi` 分辨率精度说明

`map_shanxi.yaml` 的 `resolution: 7.708785784797631`（15 位小数），不是任意值，是从 **四角 GNSS 坐标与 PGM 像素尺寸** 反算得出的：

```
ref_gnss_00 (左上): [119.76308268, 27.75586125]
ref_gnss_11 (右下): [120.0832403,  27.54541513]

经度跨: 0.32015762° × 98654 m/° ≈ 31588 m
纬度跨: 0.21044612° × 111320 m/° ≈ 23427 m

resolution = 地理跨度(m) / PGM 像素数
```

保留高精度是为了 **GNSS ↔ pixel ↔ map 坐标双向转换不累积误差**。换地图时如果 resolution 是手工取整的（如 1.0），需确认四角 ref_gnss 与 PGM 尺寸自洽，否则远端目标点会逐渐偏移。

---

## 4. 仿真对齐 checklist

| 链路 | 配置 |
|------|------|
| Nav2 载入海图 | `nav2.launch.py` → **`map:=`** |
| GNSS ↔ 地图角点 | `navsat.yaml` datum 与 **`ref_gnss*`** |
| 航点经纬→map x,y | `waypoint_transform`：`map_yaml_path` / 默认包内 `map_hk.yaml` |
| 里程计 | Nav2 **`odom_topic: /odometry/filtered`** |

---

## 5. 实船（USV_NAV）

实船上 **`/mavros/local_position/odom`** 原点通常随 **PX4 HOME**，需用 **`gnss_odom_map_tf`** 与地图锚点装订 **`map→odom`**。详见 **USV_NAV** 文档，不在此重复。

---

## 6. 相关文档

| 文档 | 内容 |
|------|------|
| [`工作进度汇报.md`](./工作进度汇报.md) | 历史记录（若有） |
| [`map与gnss的对齐.md`](./map与gnss的对齐.md) | 详细几何与调试笔记 |
| [`../src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md`](../src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md) | 仿真数据流 |
