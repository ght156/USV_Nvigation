# 地图、GNSS 与 Nav2 / 仿真对齐说明

本文为**复盘与改版**用：归纳栅格地图、仿真世界锚点、经纬度航点与 Nav2 `map` 系之间的关系，避免误解「只改一处即可全局生效」。

**相关**：日常启动步骤见 [`项目运行与联调.md`](./项目运行与联调.md)；功能包内更细的架构说明见 [`../src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md`](../src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md)；进度台账见 [`工作进度汇报.md`](./工作进度汇报.md)。

---

## 1. 栅格地图：白板 vs 岸线 / 水库

- **当前 Nav2** 在 `global_costmap` 里使用 **`static_layer`**，会加载 `workspace_nav/config/map.yaml` 所指的 PGM。
- **整图可通行（如纯白或近似均匀）** 时，全局代价几乎处处为「自由」，全局规划主要在「空图」上算路径；**避障更依赖 `local_costmap` 的激光**（如 `/roboboat/sensors/lidar/scan`）。`nav2_params.yaml` 中全局规划器对 unknown 的处理（如 `allow_unknown`）需与制图方式一致。
- **水库、岸线、禁航区** 等场景：若希望全局路径**不要穿陆岸或禁区**，应用**水陆分界清晰**的 PGM（或海图/测绘成果栅格化），把岸线标为**占据**或配合 **inflation** 留出安全带宽。
- **结论**：白板可用于快速联调；要与地理或任务语义一致，需要**带几何约束**的栅格，并与下文 **origin / yaw / GNSS** 一并核对。

---

## 2. Nav2 地图加载 与 仿真中船的位置：是否「自动对齐」？

**不会自动对齐**，需要配置与定位栈共同保证「同一平面、同一北向」。

### 2.1 仿真侧

- 船由 `workspace_gz/launch/simulation.launch.py` 在 Gazebo 中以 **`x=y=z=0`** 生成（`ros_gz_sim create`）。

### 2.2 TF：`map` 与 `odom`

- `workspace_ros/launch/localization.launch.py` 中发布 **`map` → `odom` 静态变换**，平移与旋转均为 **0**（两帧在 TF 上**重合**）。
- 机器人位姿主要来自 **`robot_localization`（EKF + `navsat_transform_node`）** 等对传感器融合的输出（如 `/odometry/filtered`），经 TF 树与 **`base_link`** 关联。

### 2.3 栅格在 `map` 中的位姿（`origin` 与 `map` 的 (0,0)）

- **`map` 是一个平面坐标系**，可延伸到栅格之外，其原点 **`map`(0,0)** 是坐标轴交点，**不必**与地图图像重合。
- `map.yaml` 的 **`origin: [x, y, yaw]`**（`yaw` 单位以所用 `map_server`/ROS 2 版本文档为准，常见为弧度）表示 **栅格 cell (0,0) 的参考角点**（惯例为**左下角**）在 **`map` 系**下的位姿：该角落在 **`(x, y)`**，整体再绕 z 旋转 **`yaw`**。
- **常见误解**：以为 `map_server`「把 `origin` 加载到 `map` 的 (0,0)」。**不正确。** 只有当你把 **`origin` 写成 `[0.0, 0.0, …]`** 时，上述**栅格角**才与 **`map`(0,0)** 重合；否则角点在 **`(x,y)`**，与 **`map`(0,0)** 不同。
- **ENU 对齐**：若 **`map` 系**与**制图所用平面**均按 **ENU（x 东、y 北）**、栅格为**上北**，且 **PGM 行序与「北向上」一致**，则 **`yaw` 常为 0**（仍建议在 RViz 复核）。此时几何上可认为**地图栅格与 `map` 系朝向一致**。
- **船在 `map` 中的位置**仍来自定位 TF（**`map`→`base_link`**），不是自动等于某个角点，除非你通过定位/初始化把船放在该坐标。

### 2.4 使「船出生点」与「地图上某一点的 GNSS」一致（仿真）

- 船在仿真中于 **Gazebo world (0,0,0)** 生成；**`world.sdf` 中 `<spherical_coordinates>`** 的经纬度定义 **world 原点**对应的 **WGS84 参考点**。
- **做法（与你的定稿理解一致）**：将 **`latitude_deg` / `longitude_deg`** 设为**地图上某一特征点**在真实（或任务定义）中的 **GNSS**；该点又已通过 **`map.yaml` 的 `origin`、分辨率**等与栅格绑定，则在几何约定一致时，**船在出生点对应的仿真 GNSS**，与**地图上该点的地理位置**一致。
- 若希望船出生在**另一些**经纬度，应改 **world 锚点**到那一点，或改出生位姿（需在 launch / SDF 中改 spawn，而不仅改地图）。

### 2.5 复盘检查

- 在 **RViz2** 中同时看 **`/map`**、机器人 footprint、（若有）仿真模型位置，确认：**水道/岸线相对船体**与任务预期一致。
- 更换 PGM 或调整 **`origin`/`yaw`** 或 **`world.sdf` 锚点** 后，务必重做上述检查。

---

## 3. GPS 航点规划在本项目中的数据流

这不是「在 RViz 里点栅格取点」，而是 **经纬度 → 平面 `x,y` → JSON → Nav2 多点跟随**（与 **地面站 GROUND CONTROL STATION** 的任务链路一致：UI 保存 `backend/data/waypoints.json` 后，通过 API 启动的 **`waypoint_publisher`** 向 **`/waypoint`** 周期发布含 `latitude`/`longitude` 的 JSON，供下游消费）。

1. **`waypoint_transform`**（`workspace_nav`）  
   - **默认**从 **`workspace_nav/config/map.yaml`** 读取 **`ref_gnss_10` / `ref_gnss`** 作为 **datum**（与 **`navsat_transform` 固定 `datum`** 一致），将地面站发来的经纬度转为 **UTM 米制偏移**，写入 **`waypoints.json`**。  
   - 兼容旧仿真：节点参数 **`datum_source:=first_gps`** 时仍可用 **首帧 `/gps/filtered`** 作为 datum。  
2. **`waypoint_with_state`**  
   - 读取 **`waypoints.json`** 中的 **`x`、`y`**，发布 **`frame_id: map`** 的 **`PoseStamped`**，调用 Nav2 **`FollowWaypoints`**。

---

## 4. 是否与「真实世界」对齐？

- **需要任务级地理一致时**：制图原点、**北向**、datum 与 **WGS84/UTM** 带号等必须与**真实库区/赛场**及 **GNSS 数据源**一致（或明确采用纯仿真约定）。
- **仅仿真自洽**：可只保证「仿真 GNSS + 航点 + 当前 PGM」在**同一套平面约定**下自洽，不必对标真实水库；但一旦对接真机或真图，必须收紧对齐条件。

---

## 5. 修改 `world.sdf` 中的经纬度：在链路上的含义

**文件**：`src/YILDIZ-USV/workspace_gz/worlds/world.sdf` 中 **`<spherical_coordinates>`**（`latitude_deg`、`longitude_deg`、`elevation`、`heading_deg` 等）。

- 该块将 **Gazebo 世界系** 与 **WGS84 地球模型** 绑定；船在 **world `(0,0,0)`** 附近生成时，**仿真 GNSS** 读数应对应**该参考点附近的经纬度**（具体以传感器与 ENU 约定为准）。
- **改动锚点后需重启仿真**；地面站下发的**绝对经纬度**若仍指向旧区域，会与**新锚点**下的相对 `x,y` 不一致，需同步任务数据。

---

## 6. 「地图与 GNSS 绑定」应如何理解？

更准确的说法是两条链路必须在**同一套 `map`/`odom` 平面与北向**下可接：

| 链路 | 作用 |
|------|------|
| **GNSS / 航点** | datum + UTM 差分 → **`waypoints.json` 的 `x,y`**，在 **`map`** 中用 |
| **地图** | `map.yaml` 把 **PGM 栅格**放到 **`map`** 中；**`static_layer`** 给全局代价 |

**`map.yaml` 不会因修改 `world.sdf` 而自动更新。**  
白板图时，全局代价对位置不敏感，容易**误以为**「只改 world 就万事大吉」。一旦使用**带地理意义的岸线 PGM** 或 **map.yaml 中的参考经纬字段**（见下节），必须与本仿真锚点、任务区一致。

---

## 7. 「只改 `world.sdf` 经纬度，其它都不动」在什么情况下可行？

- **较适用**：仿真、**全局图近似全可通行**、主要验证 **相对 datum 的经纬度航点链**；定位与 Nav2 仍正常。
- **不适用或不够**：使用**与真实岸线/禁区对齐**的栅格、依赖 **`map.yaml` 中 `ref_gnss*` 等角点与真实经纬的对应**、或要求 **RViz 地图与仿真景象严格重合** —— 此时需同步调整 **PGM、`map.yaml` 的 `origin`/`yaw`、参考经纬**及任务航点。

---

## 8. `map` 方向与真实地理北向如何对齐？

### 8.1 常见平面约定（REP-105）

常采用 **ENU**：**`map` 的 x 朝东、y 朝北**（具体以全队约定与 `robot_localization` 输出为准）。

### 8.2 `map.yaml` 中的 **yaw**（务必读 8.4）

- **`origin` 第三项**在 **`map_server` 发布的 `/map` 与 RViz 的 Map 显示**中是有效的：用于把 **PGM 的像素行/列**摆到与 **`map` 的 ENU** 一致。
- 底图若为「上北」，且与 ENU 一致，常从 **yaw = 0** 配合正确 **x,y 平移** 开始标定；若底图北向与 ENU 差固定角，**从 RViz 看**可以把该角补进 **`yaw`**。
- **但与 Nav2 `global_costmap` 的 `static_layer` 联用时，`yaw` 往往不能指望**（见 **8.4**）；纠方向应优先 **旋 PGM + `yaw:=0`**。

### 8.3 `navsat_transform` 与航向

- 当前仓库 `workspace_ros/config/navsat.yaml` 中 **`use_odometry_yaw: true`**：航向更多依赖 **里程计/IMU 融合**，而非单独依赖 GNSS 航迹。  
- **真北 / 船首向**要可信，需保证 **IMU（及若有磁力计）标定与初值**合理；地图 **yaw** 再与这套 **ENU** 对齐。

### 8.4 **重要限制**：`static_layer` 对 `origin` **朝向（yaw）** 的处理

- **现象**：只在 `map.yaml` 里加 **`origin` 的旋转** 时，**RViz 里 `/map` 可能对**，但 **`global_costmap` / 全局规划用的障碍图**仍像**未旋转的原栅格**，**与 Map 显示拧着**，甚至 **无解路径**。  
- **原因**：Nav2 的 **`static_layer` 长期不按占用栅格的 `origin` 朝向做旋转**（与 RV1 Navigation 同类问题）；`/map` 消息虽带完整 `origin` 姿态，**代价图融合侧不按 yaw 同步**。官方 issue 例如 [navigation2#1511](https://github.com/ros-navigation/navigation2/issues/1511)（**wontfix**，建议用制图侧解决）。  
- **推荐做法**：在 **GIMP / QGIS / 脚本** 中把 **PGM 旋到与 `map` ENU 轴一致**，`map.yaml` 里 **`origin` 第三项置 `0`**，仅保留 **`x,y` 平移**；则 **`/map`、global costmap、规划**一致。  
- **`waypoint_transform`** 仍只做 **UTM 相对 datum 的 `x,y`**，**不读** `map.yaml`；地图与 ENU 一致后，航点与栅格才容易自洽。

---

## 9. 关键文件索引（便于改版时全局搜索）

| 内容 | 路径（相对仓库根） |
|------|-------------------|
| 仿真世界锚点 | `src/YILDIZ-USV/workspace_gz/worlds/world.sdf` |
| 仿真启动 / 船生成 | `src/YILDIZ-USV/workspace_gz/launch/simulation.launch.py` |
| 地图 YAML / PGM | `src/YILDIZ-USV/workspace_nav/config/map.yaml`、`map/map.pgm`（或 `map.yaml` 内 `image` 所指路径） |
| Nav2 参数 | `src/YILDIZ-USV/workspace_nav/config/nav2_params.yaml` |
| `map`↔`odom`、EKF、navsat | `src/YILDIZ-USV/workspace_ros/launch/localization.launch.py`、`config/ekf.yaml`、`config/navsat.yaml` |
| 经纬度 → `waypoints.json` | `src/YILDIZ-USV/workspace_nav/workspace_nav/waypoint_transform.py` |
| `waypoints.json` → Nav2 | `src/YILDIZ-USV/workspace_nav/workspace_nav/waypoint_with_state.py` |

> **说明**：若 `map.yaml` 中含 **`ref_gnss*`** 等非 `map_server` 标准字段，可能为工程内**制图参考/文档**；改版 world 或任务区时，应人工确认这些参考是否仍有效，避免岸线/角点与地球位置脱节。

---

## 10. 改版复盘 Checklist（建议逐项打勾）

- [ ] **PGM 朝向与 Nav2**：若用 **`static_layer`**，优先 **旋图 + `origin` yaw=0**，避免只改 yaml 旋转导致 **RViz `/map` 与 global costmap 拧转**（见 **§8.4、§14**）。  
- [ ] 改 `world.sdf` 锚点后，已重启仿真并核对 **GNSS 话题**是否与预期经纬度一致。  
- [ ] **`map.yaml` 的 `origin`（尽量 `yaw=0` 纠旋转）** 与 PGM、ENU 一致；RViz **`/map`** 与 **`global_costmap/costmap`** 几何对照无整体拧转。  
- [ ] **`ref_gnss*`**（若使用）与当前仿真/实场一致。  
- [ ] 地面站或测试用例中的 **经纬度航点** 与当前锚点、datum 逻辑一致。  
- [ ] 岸线/禁航 **PGM** 与 `static_layer`、全局规划参数（如 `allow_unknown`）匹配。  
- [ ] 文档与版本控制中记录 **本次锚点、地图版本、UTM 带** 等元数据，便于回滚对比。

---

## 11. 卫星图导出「上北」：最少标定信息（备忘）

仅「分辨率 + 一个参考 GNSS」**不够**，除非同时约定**该经纬度对应栅格上哪一个点**（例如 cell (0,0) 的角、图中心像素等）。否则无法唯一写出 **`origin`**。

- **至少需要**：**一对对应** `(lat, lon) ↔` 该点在 **`map`/栅格上的位置**（米制 `x,y` 或行列 + 角点约定）+ **`resolution`** + **图宽高**（或能从 PGM 读出）；**上北 + ENU + 行序**核对后 **`yaw` 常为 0**。
- **`ref_gnss*`** 等字段若出现在 `map.yaml` 中，多为**人读/工具用的参考**；`map_server` **不会**仅因写了这些字段就自动完成绑定，除非自建节点读取并发布 TF/位姿。

---

## 12. 四角 GNSS 控制点与丢定位后重定位（扩展思路）

- 在地图上标定 **多对** `(map_x, map_y) ↔ (lat, lon)`（四角为非退化配置），可用**仿射/相似变换**把任意 GNSS 位姿变到 **`map` 平面**，从而在**重捕 GNSS** 后仍可知船在图上的位置，**减少对「上电初始姿态」的依赖**。
- **朝向**：四角配对会**隐含**地图栅格相对地理系的旋转；**不**等于取消艏向问题——**单点 GNSS** 仍通常需要 **IMU/双天线** 等定艏向。
- 与本仓库当前 **`map`≡`odom` + EKF** 栈的**具体接线**需另做节点或滤波重初始化设计；本节仅作**方案备忘**。

---

## 13. 定稿结论摘要（便于快速复读）

1. **栅格角点**由 **`map.yaml` 的 `origin`** 放在 **`map` 系的 `(x,y)` 并带 `yaw`**，**不是**默认放在 **`map`(0,0)**。  
2. **`map` 与底图均为 ENU、上北且行序正确**时，朝向与 `map` 系可对齐；**与 Nav2 规划一致时**优先 **旋 PGM、`yaw=0`**，**不要**只依赖 **`origin.yaw`**（见 **8.4**）。  
3. **仿真船**：把 **`world.sdf` 球坐标经纬度**设为**地图上选定点的 GNSS**，可使 **world (0,0) 出生点**的仿真 GNSS 与该地图点**一致**（在传感器与 ENU 约定一致时）。  
4. **绑定** = 几何与控制链一致；**改 world 不会自动改 `map.yaml`**，二者需按任务一起设计。

---

## 14. RViz `/map` 与 global costmap、滚动窗口（复盘）

### 14.1 为啥 Nav2 插件里的「格图」和加载的 Map 不完全重合？

- **话题往往不同**：RViz **Map** 多为 **`/map`**（`map_server`）；Navigation 2 / 全局规划相关显示多为 **`/global_costmap/costmap`** 等，来自 **Nav2 costmap**，不是把 `/map` 再画一遍。
- **分辨率/范围**：本仓库 **global costmap** 与 **`map.yaml` 分辨率**可不一致；且 **`rolling_window: true`** 时只有 **固定宽高（格）× resolution** 的一片「视窗」，**不等于整张 PGM**。
- **膨胀**：costmap 含 **inflation**，岸线会比 `/map` **更「胖」**。
- **若再加上 8.4 的 yaw 未进 static_layer**：会出现 **连旋转都不一致** 的情况。

### 14.2 Global costmap 的「滚动窗口」是什么？

- **含义**：在 **`map` 系**里维持 **固定物理尺寸** 的一块代价栅格（见 `nav2_params.yaml` 中 **width/height/resolution、rolling_window**），窗格在 **`map` 里平移**，从 **`/map`** 的 **static_layer** 填入障碍；**不是**把整张贴图一次性铺满内存。
- **目标在「窗外」**：通常 costmap 会 **更新边界/平移窗口** 以覆盖 **机器人与目标** 一带，**多数情况仍能规划**；若 **目标在 `/map` 静态图覆盖之外**、或 **unknown / 对齐错误**，仍可能 **失败或路径怪**。  
- **大水域大地图**：滚动窗是 **用有限格子干大图的常见折中**，**适合**「大地图 + 2D 栅格 Nav2」类场景（含水库/海面栅格化），**不是**水域专用设计；障碍稀疏时也要保证 **窗尺度与航程、参数** 合理。

### 14.3 `map_updates` 一类话题

- **`map_server` 主要发整图 `/map`**；带 **`…/costmap_updates`** 的常常是 **costmap 的增量（`OccupancyGridUpdate`）**，**不是**静态 `/map` 的替代品；RViz 看岸线对齐应以 **`/map`** 与 **`global_costmap/costmap`** 在 **`map` 系**下对照为准。

---

*文稿依据讨论稿整理；若代码或参数文件后续有变，请以仓库内实际配置为准并更新本节。*
