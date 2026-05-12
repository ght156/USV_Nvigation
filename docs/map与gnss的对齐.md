按照你说的第三个方案，是不是我只要把robotlocalization里面不使用第一次gnss做相对位姿计算，而是使用固定gnss做计算就可以了，那么其实只要改navsat_transform的参考gnss为地图的左下角参考点就行了，你看一下项目代码有关定位的部分。这个项目的其他部分好像是使用了这个节点来计算目标点，那这样的话，目标点计算的逻辑应该也得改吧https://github.com/YILDIZ-USV/YILDIZ-USV/tree/main去看这个代码



对，你这个理解**大方向是对的**：
如果你走第三个方案，核心就是 **不要让 `navsat_transform_node` 用第一帧 GNSS 当相对原点，而是让它使用地图固定 GNSS 参考点作为 datum**。但我看了这个项目的定位和目标点代码后，结论要再补一句：

**只改 `navsat.yaml` 不够，目标点转换逻辑也必须一起改。否则船的位置用“地图左下角 datum”，目标点却还在用“船第一次 GPS datum”，两套坐标系会错开。**

> **仓库维护备忘（Nav2 避障，可与本文 GNSS/地图对齐一并阅读）**：当栅格**近白板、全局占据约束弱**时，**动态避障主要依赖 `local_costmap`**。**仿真**（`workspace_nav/config/nav2_params.yaml`）：**`ObstacleLayer`** + **`LaserScan`**（约定 **`/roboboat/sensors/lidar/scan`**）。**实船 MAVROS**（`workspace_nav/config/nav2_params_real_mavros.yaml`）：**`VoxelLayer`** + **`sensor_msgs/PointCloud2`**（默认 **`/livox/lidar`**）；体内 3D 体素仍**投影为 2D costmap**，**`InflationLayer` 未改**。现场须核对 **点云 `header.frame_id`→`base_link`（及 map/odom 链）**、**时间戳与 `use_sim_time`**，并标定 **`origin_z`、`z_voxels`、`min/max_obstacle_height`、量程** 等。更细的说明见 [`地图与GNSS-Nav2对齐说明.md`](./地图与GNSS-Nav2对齐说明.md) §1、[`../src/YILDIZ-USV/docs/实船调试.md`](../src/YILDIZ-USV/docs/实船调试.md)「避障传感器（模式 B）」。

------

这个项目现在的定位链路确实是你说的那样。README 里启动顺序是先启动仿真、再启动 localization、再启动 Nav2，后面再跑 `converter`、`waypoint_transform` 和 `waypoint_with_state`。([GitHub](https://github.com/YILDIZ-USV/YILDIZ-USV/tree/main))
你上传的 `localization.launch.py` 里也能看到，它启动了 `navsat_transform_node`、`ekf_node`，然后又额外发布了一个静态 `map -> odom`，而且是全 0：`map` 和 `odom` 被强行重合。

你现在的问题根源就在这里：

```text
map -> odom = 固定 0
odom -> base_link = ekf 根据 /odometry/gps 算
/odometry/gps = navsat_transform 根据 GNSS 算
```

但当前 `navsat.yaml` 只有：

```yaml
navsat_transform_node:
  ros__parameters:
    use_odometry_yaw: true
    frequency: 15.0
    delay: 1.0
```

仓库里的原始文件也是这样，没有 `wait_for_datum`，没有 `datum`。([GitHub](https://raw.githubusercontent.com/YILDIZ-USV/YILDIZ-USV/main/workspace_ros/config/navsat.yaml))
所以 `navsat_transform_node` 现在本质上还是在用运行时状态建立 GPS 到世界坐标的关系，而不是用你的地图左下角 GNSS 作为固定地图基准。

------

## 你说的“只改 navsat_transform 的参考 GNSS 为地图左下角”是否成立？

**在一个前提下成立：你的地图坐标系本身就是以左下角为 `(0,0)`，并且 x/y 方向和 ENU 或 UTM 东/北方向一致。**

也就是说，如果你希望：

```text
ref_gnss_10 / ref_gnss / 左下角  ->  map 坐标 (0, 0)
向右                              ->  map +x
向上                              ->  map +y
```

那么你可以把 `navsat.yaml` 改成类似这样：

```yaml
navsat_transform_node:
  ros__parameters:
    use_sim_time: true

    frequency: 15.0
    delay: 1.0

    # 关键：不要用第一帧 GPS 自动当 datum
    wait_for_datum: true

    # 地图左下角 GNSS
    # [latitude, longitude, heading]
    # heading 是 ENU 下的朝向角，0 表示 x 轴朝东
    datum: [LEFT_BOTTOM_LAT, LEFT_BOTTOM_LON, 0.0]

    zero_altitude: true
    publish_filtered_gps: true
    broadcast_utm_transform: true
```

`robot_localization` 官方文档里说得很明确：`wait_for_datum: true` 时，`navsat_transform_node` 会等待 `datum` 参数或 `set_datum` 服务，而不是直接使用运行时第一帧；`datum` 的第三个量是 heading。([ROS](https://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html))
同时它会发布 `/odometry/gps`，这个话题就是把 GPS 坐标转换到机器人世界坐标系后的 Odometry，可以再融合进 EKF。([ROS](https://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html))

但这里还有两个坑。

------

## 第一个坑：当前 `ekf.yaml` 还是 `world_frame: odom`

你当前的 `ekf.yaml` 是：

```yaml
ekf_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    imu0: /imu/fixed_cov
    odom0: /odometry/gps
```

仓库原始文件也是这个结构。([GitHub](https://raw.githubusercontent.com/YILDIZ-USV/YILDIZ-USV/main/workspace_ros/config/ekf.yaml))

这意味着 EKF 现在发布的是：

```text
odom -> base_link
```

而不是：

```text
map -> odom
```

项目又在 launch 里额外发布了静态：

```text
map -> odom = 0
```

所以现在其实是把 `map` 和 `odom` 当成同一个坐标系用了。这个在仿真里能跑，但从严格 Nav2 / robot_localization 设计看，并不是最稳的实船结构。

官方文档建议：如果融合的是 GPS 这种全局绝对位置，`world_frame` 应该设成 `map`；同时要有别的来源发布连续的 `odom -> base_link`，比如另一个只融合 IMU/速度/里程计的本地 EKF。([ROS](https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html))
Nav2 的 robot_localization 教程也说明，EKF 可以发布 `odom => base_link`，`navsat_transform_node` 用于把 GPS 转成机器人 world frame。([Nav2](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html))

所以有两个落地层级。

### 最小改法，适合你先验证

保留现在单 EKF 和静态 `map -> odom = 0`，只做这几件事：

```yaml
navsat_transform_node:
  ros__parameters:
    use_odometry_yaw: true
    frequency: 15.0
    delay: 1.0

    wait_for_datum: true
    datum: [ref_gnss_10_lat, ref_gnss_10_lon, map_yaw]
    zero_altitude: true
    publish_filtered_gps: true
    broadcast_utm_transform: true
```

这样等价于：

```text
map == odom
ref_gnss_10 对应 map/odom 原点
船的 GNSS 转成 odom 坐标
因为 map->odom=0，所以船也显示在 map 正确位置
```

这个方法改动小，容易先验证。

但它有一个问题：你把 GPS 全局跳变融合进了 `odom -> base_link`，`odom` 可能会跟着 GPS 恢复时跳。仿真和低速船可以先用，实船长期建议升级。

### 更规范的实船改法

用两个 EKF：

```text
ekf_local:
  输入 IMU + 船体速度/里程计
  world_frame: odom
  输出 odom -> base_link

navsat_transform:
  输入 /gps/fix + /imu + /odometry/local
  固定 datum = 地图左下角 GNSS
  输出 /odometry/gps

ekf_global:
  输入 /odometry/local + /odometry/gps
  world_frame: map
  输出 map -> odom
```

然后删掉 launch 里面这个静态 TF：

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_odom_tf',
    ...
    '--frame-id', 'map',
    '--child-frame-id', 'odom'
)
```

因为一旦 `ekf_global` 发布 `map -> odom`，这里再静态发布一个同名 TF，就会冲突。

------

## 第二个坑：目标点转换逻辑必须改

你怀疑得对，项目里目标点计算逻辑确实也要改。

我看了 `workspace_nav/scripts/waypoint_transform.py`，它现在是这样做的：订阅 `/gps/filtered`，收到第一帧后把它当作 `datum_lat/datum_lon`，然后把地面站发来的航点经纬度转成 UTM，再减去这个 datum 的 UTM。代码里就是：

```python
self.gps_sub = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_callback, qos_profile=qos)
...
if self.datum_lat is not None and self.datum_lon is not None:
    return
...
datum_lat = float(msg.latitude)
datum_lon = float(msg.longitude)
easting, northing, _, _ = utm.from_latlon(datum_lat, datum_lon)
self.datum_lat = datum_lat
self.datum_lon = datum_lon
self.datum_easting = easting
self.datum_northing = northing
```

然后航点坐标：

```python
x = easting - self.datum_easting
y = northing - self.datum_northing
```

也就是说：**航点坐标现在是相对于“第一次 `/gps/filtered`”的，不是相对于地图左下角的。** 仓库里的代码说明也写了这个节点会使用“acquired GPS datum”把 waypoint 转成本地 UTM-relative 坐标。([GitHub](https://raw.githubusercontent.com/YILDIZ-USV/YILDIZ-USV/main/workspace_nav/scripts/waypoint_transform.py))

所以你如果只改 `navsat_transform_node`，但不改 `waypoint_transform.py`，可能会出现这种情况：

```text
船的位置：相对于 ref_gnss_10 左下角
目标点：相对于第一次 /gps/filtered，通常接近船启动点
```

这样 Nav2 收到的目标点就会错位。

`waypoint_with_state.py` 后面会读取 `waypoints.json` 里的 `x/y`，然后创建 `frame_id='map'` 的 `PoseStamped` 发给 Nav2 FollowWaypoints。([GitHub](https://raw.githubusercontent.com/YILDIZ-USV/YILDIZ-USV/main/workspace_nav/scripts/waypoint_with_state.py))
所以 `waypoint_transform.py` 输出的 `x/y` 必须已经是 **map 坐标系下的 x/y**，不能再是相对船第一次 GPS 的 x/y。

------

## 你应该把 `waypoint_transform.py` 改成使用地图 datum

现在它是：

```text
目标经纬度 - 第一次 /gps/filtered 经纬度
```

应该改成：

```text
目标经纬度 - 地图左下角 ref_gnss_10 经纬度
```

最小修改思路：

```python
# 不要订阅 /gps/filtered 来获取 datum
# 直接从参数或 map.yaml 读取 ref_gnss_10

self.declare_parameter('datum_lat', 0.0)
self.declare_parameter('datum_lon', 0.0)

self.datum_lat = self.get_parameter('datum_lat').value
self.datum_lon = self.get_parameter('datum_lon').value
self.datum_easting, self.datum_northing, _, _ = utm.from_latlon(
    self.datum_lat,
    self.datum_lon
)
```

然后保留原来的航点转换：

```python
easting, northing, _, _ = utm.from_latlon(lat, lon)
x = easting - self.datum_easting
y = northing - self.datum_northing
```

这样 `waypoints.json` 里存出来的 `x/y` 就是：

```text
相对于地图左下角 ref_gnss_10 的 map 坐标
```

这才和 `navsat_transform_node` 的固定 datum 对齐。

------

## 但如果地图不是正北朝上，还要加旋转

如果你的天地图三维地形/二维 Nav2 栅格地图是正北朝上，并且：

```text
map +x = 东
map +y = 北
```

那么只用 UTM 差值就够了。

但如果你的地图图片有旋转，比如地图右边不是正东，而是偏了一个角度，那么仅仅设置左下角 datum 还不够。你还需要根据四个角点算 `map_yaw`：

```text
ref_gnss_10 左下
ref_gnss_11 右下
```

把这两个点转成 UTM：

```text
dx = E11 - E10
dy = N11 - N10
map_yaw = atan2(dy, dx)
```

然后经纬度转 map 坐标时做旋转：

```text
dE = E - E10
dN = N - N10

x_map =  cos(map_yaw) * dE + sin(map_yaw) * dN
y_map = -sin(map_yaw) * dE + cos(map_yaw) * dN
```

注意这里的意思是：把 UTM 东北坐标旋转到你的地图 x/y 坐标系里。

如果你只用 `navsat_transform_node` 的 `datum: [lat, lon, heading]`，它可以解决一部分“世界坐标朝向”的问题，但你的 `waypoint_transform.py` 也必须用同样的旋转逻辑，否则船的位置和目标点仍然不是同一套坐标。

------

## 我建议你按这个顺序改

第一步，先统一 datum。

`navsat.yaml`：

```yaml
navsat_transform_node:
  ros__parameters:
    use_odometry_yaw: true
    frequency: 15.0
    delay: 1.0

    wait_for_datum: true
    datum: [ref_gnss_10_lat, ref_gnss_10_lon, 0.0]

    zero_altitude: true
    publish_filtered_gps: true
    broadcast_utm_transform: true
```

第二步，改 `waypoint_transform.py`，不要再从 `/gps/filtered` 获取 datum，而是从参数或 `map.yaml` 读取同一个 `ref_gnss_10`。

第三步，确认 `waypoint_with_state.py` 不用大改。它本来就是把 `x/y` 作为 `map` 坐标发给 Nav2，这个逻辑是对的；真正错的是前面的 `x/y` 是怎么来的。

第四步，检查 `map.yaml` 的 `origin`。如果你要让 `ref_gnss_10` 对应地图左下角，建议：

```yaml
origin: [0.0, 0.0, 0.0]
```

地图自身不要再靠猜 origin 偏移。

第五步，验证四个点：

```text
ref_gnss_10 -> x≈0, y≈0
ref_gnss_11 -> x≈地图宽度, y≈0
ref_gnss_00 -> x≈0, y≈地图高度
ref_gnss_01 -> x≈地图宽度, y≈地图高度
```

如果这四个点对不上，就不能只靠一个 `datum`，需要四角点仿射变换。

------

## 最终判断

所以你的问题可以直接回答为：

**是的，你要让 robot_localization 不再用第一帧 GNSS 做相对位姿，而是使用地图左下角 GNSS 作为固定 datum。**

但在这个项目里，必须同时改两处：

```text
1. workspace_ros/config/navsat.yaml
   让 navsat_transform_node 使用固定 ref_gnss_10 datum

2. workspace_nav/scripts/waypoint_transform.py
   让目标点转换也使用同一个 ref_gnss_10 datum
```

否则就是：

```text
船的位置坐标系 = 地图左下角 datum
目标点坐标系 = 船第一次 GPS datum
```

这两个不一致，Nav2 导航目标肯定会偏。

更长期、实船更稳的版本，是把现在单 EKF 改成“local EKF + navsat_transform + global EKF”，并移除当前 launch 里的静态 `map -> odom`。当前这个静态 TF 在你做真正 GNSS 全局重定位时会变成限制。



还有一个小问题，是我个人的问题，为啥开源的原项目不用参考gnss呢，只用第一帧gnss,是因为他们只是做仿真测试吗，他们开源项目使用的是空白栅格图，直接把地图origin设置为-50，-50，让船在地图中心出现，是不是因为他们绑定了map和odom,所以船才只会在map中心出现啊。我如果要改为实船落地，里面的很多东西我都要改对吧



对，你这个判断基本是对的。原项目不用固定参考 GNSS，大概率不是因为他们不知道这个问题，而是因为它的目标更偏向**仿真流程验证**，不是“真实地理地图 + 实船定位”的完整落地方案。

它现在的设计逻辑更像这样：

```text
Gazebo 仿真世界原点附近生成船
        ↓
第一帧 GNSS 被当成本地坐标原点
        ↓
navsat_transform 把后续 GNSS 转成本地 x/y
        ↓
map 和 odom 又被静态绑定在一起
        ↓
Nav2 在一个空白栅格地图里跑通导航流程
```

所以它不需要知道“这个地图左下角真实经纬度是多少”。因为对它来说，地图只是一个足够大的局部平面，船在哪里启动，哪里就近似是局部导航坐标的起点。

------

你说的这个点也对：原项目用空白栅格图，把 `origin` 设成类似：

```yaml
origin: [-50.0, -50.0, 0.0]
```

这个目的通常就是让一张 100m × 100m 左右的空白地图覆盖：

```text
x: -50 ~ +50
y: -50 ~ +50
```

如果船的初始定位在：

```text
odom/map: (0, 0)
```

那它自然会显示在地图中心附近。

这不是地理配准意义上的 origin，而是仿真里为了让机器人别出现在地图边缘，人为把空白地图中心对准 `(0,0)`。

------

更关键的是，你说的“他们是不是绑定了 map 和 odom，所以船才只会在 map 中心出现”，这个理解非常接近真实原因。

你上传的 `localization.launch.py` 里有这个静态 TF：

```python
map -> odom = 0
```

也就是：

```text
map 坐标系和 odom 坐标系完全重合
```

而 `ekf.yaml` 里 `world_frame` 又是 `odom`，所以 EKF 主要是在算：

```text
odom -> base_link
```

这样一来，整套系统等价于：

```text
map == odom
```

船如果通过第一帧 GNSS 或仿真初始状态被放在 `odom` 的 `(0,0)` 附近，那么它在 `map` 里也就是 `(0,0)` 附近。再配合空白地图 `origin: [-50, -50, 0]`，船就看起来正好在地图中心。

所以这不是“真实地图定位已经解决了”，而是仿真环境里坐标被人为简化了。

------

你如果要改成实船落地，确实很多东西要重新理顺，但不是全部推倒重来。原项目有价值的部分可以保留，比如 Gazebo 仿真、Nav2 bringup、waypoint action、基础 sensor republisher、robot_localization 框架。真正需要改的是“坐标体系”和“定位基准”。

你要从这个仿真逻辑：

```text
第一帧 GNSS = 本地原点
map == odom
空白地图覆盖 -50~50
目标点也相对第一帧 GNSS 计算
```

改成实船逻辑：

```text
地图左下角 GNSS = 固定地理参考点
map 是真实地图坐标系
船的 GNSS 被转换到 map 坐标
目标点 GNSS 也被转换到同一个 map 坐标
map -> odom 用于全局校正
odom -> base_link 保持局部连续
```

核心变化其实就这几个。

------

第一，`map.yaml` 不能再只是“空白图 + origin -50 -50”。
如果你用天地图/真实场地地图，那么 `origin` 应该表达地图图像和 `map` 坐标的关系。你可以先规定：

```text
地图左下角 = map (0, 0)
ref_gnss_10 = 地图左下角 GNSS
```

然后：

```yaml
origin: [0.0, 0.0, 0.0]
```

这样地图本身不乱动。

------

第二，`navsat_transform_node` 不能再依赖第一帧 GNSS。
原项目这样做对仿真没问题，因为船从哪里启动，哪里就是局部原点。但实船不行，因为你每次下水、重启、丢星恢复，第一帧 GNSS 都可能不同。

你应该改成：

```yaml
wait_for_datum: true
datum: [地图左下角纬度, 地图左下角经度, 地图朝向角]
```

这样每次运行时，同一个 GNSS 点都会落到同一个 map/odom 坐标位置。

------

第三，目标点转换必须跟着改。
原项目的目标点转换逻辑如果是：

```text
目标点 GNSS - 第一次船的 GNSS
```

那么这只适合仿真或临时测试。实船应该改成：

```text
目标点 GNSS - 地图参考 GNSS
```

也就是船的位置和目标点都使用同一个参考：

```text
ref_gnss_10 / 地图左下角 GNSS
```

否则就会出现：

```text
船的位置坐标系：地图左下角
目标点坐标系：船启动点
```

这两个一旦不一致，Nav2 规划的目标点就会偏。

------

第四，`map -> odom` 最好不要长期静态绑定。
仿真里这样写很方便：

```text
map -> odom = 0
```

但实船里，更规范的是：

```text
odom -> base_link：短时间连续，不能跳
map -> odom：根据 GNSS / 重定位做全局校正
```

这样 GNSS 丢失时，船还能靠 IMU/速度/里程计在 `odom` 下连续运动；GNSS 恢复后，再通过 `map -> odom` 把整体位置拉回地图正确位置。

如果你暂时为了快速验证，可以先保留 `map == odom`，但长期实船建议改成双 EKF：

```text
local EKF:
  IMU + 速度/里程计
  world_frame = odom
  发布 odom -> base_link

navsat_transform:
  固定 datum = 地图左下角 GNSS
  输出 /odometry/gps

global EKF:
  local odom + /odometry/gps
  world_frame = map
  发布 map -> odom
```

然后删掉原来 launch 里的静态 `map -> odom`。

------

所以你可以这样理解原项目：

它不是一个完整的“实船地理配准导航系统”，而是一个“USV + Gazebo + Nav2 + GPS 转局部坐标”的仿真验证框架。它把很多真实问题简化掉了：

```text
没有真实地图地理配准
没有固定 datum
没有严格区分 map 和 odom
没有处理 GNSS 丢失恢复后的全局重定位
目标点也是相对启动点计算
```

这些在仿真里都能跑，因为仿真世界是干净的、地图是空白的、船从固定位置启动、GNSS 不会有真实环境里的各种异常。

但你现在换成天地图三维地形、准备长期测试甚至实船，那这些简化就会暴露出来。

------

你的改造方向应该是：

```text
1. 地图固定，不再靠猜 origin
2. 使用 ref_gnss_10 / 四角 GNSS 做地图地理配准
3. navsat_transform 使用固定 datum，不用第一帧 GNSS
4. 船 GNSS 转 map 坐标
5. 目标点 GNSS 也转同一个 map 坐标
6. 后续把静态 map->odom 改成由 global EKF 或重定位节点动态维护
```

所以答案是：**对，你要实船落地，定位和坐标相关的部分基本都要改对。**
但不是全部重写，而是把原项目里“仿真局部坐标”的假设，替换成“真实地理地图坐标”的假设。最先该改的就是 `navsat.yaml`、`waypoint_transform.py`、`map.yaml` 的 georeference 逻辑，以及后面 `map->odom` 的发布方式。

---

## 实施记录（本仓库已落地的最小闭环）

**日期：2026-04-30**

| 项 | 状态 | 说明 |
|----|------|------|
| `workspace_ros/config/navsat.yaml` | 已改 | `wait_for_datum: true`，`datum: [lat, lon, heading_rad]` 与 `map.yaml` 的 `ref_gnss_10` 一致；`heading=0` 表示 ENU 下 x 朝东。 |
| `workspace_nav/workspace_nav/waypoint_transform.py` | 已改 | 默认 `datum_source:=map_yaml`；读 **`map.yaml` 的 `origin`**，将 **ENU/UTM 平面偏移**变换为 **Nav2 `map` 世界坐标**；**`projection`**：`enu`（默认，贴近 navsat 小范围平面）或 `utm`；**`map_datum_ref_key`** 指定与 **栅格 (0,0)** 对应的 `ref_gnss_*`（默认 `ref_gnss_10`）；输出 JSON 含 **`map_frame_meta`**。`first_gps` 时**不**套用 map origin（与 map 不一定一致）。 |
| `workspace_nav/config/map.yaml` | 已注 | `ref_gnss_*` 为 `[lon, lat]`，须与 `navsat.yaml` 的 `datum` 同步。 |
| 地图相对正北的旋转 | 未自动 | 若栅格 x 不朝东，需统一改 `datum[2]` 与航点投影旋转。 |
| 双 EKF、动态 `map→odom` | 未实施 | 仍为静态 `map→odom` 恒等。 |

**脚本路径**：`workspace_nav/workspace_nav/waypoint_transform.py`。

### 补充：`map` 帧航点（与「仅 UTM 差分」的区别）

**现象复盘**：地面站/Cesium 上经纬度正确，但 **`waypoints.json` 的 x,y** 与 RViz `map` 帧不符（如符号反号、象限错、与 navsat+EKF 船位不一致）。

**根因**：Nav2 目标在 **`frame_id: map`**，必须同 **`map_server`/`map.yaml` 定义的世界系**。仅做「航点与 datum 的 UTM 东/北差」**未**叠加 **`origin: [ox, oy, yaw]`**，或未确认 **datum 角点 = PGM (0,0) 角点**，会与定位栈脱节。

**已落地逻辑**（摘要）：

1. 以 `map_datum_ref_key` 从 `map.yaml` 取 **(lat₀, lon₀)**，与 **`navsat.yaml` datum** 一致。  
2. 对每个航点求相对 **ENU 米**（默认）或 **UTM 差分**（`projection:=utm`）。  
3. 若 `datum_source=map_yaml`：`map_x, map_y = origin + R(yaw)·(east, north)`（与 costmap/map_server 常用平面关系一致）。  
4. 写出 **`map_frame_meta`**（`origin_*`、`applied_origin_transform`、`projection` 等）便于对照 RViz。

**地面站 UI 罗盘 90° 偏置**（简要）：ROS **`map`/odom ENU** 的 **`yaw`** 为 **0 朝东**；航空罗经为 **0 朝北顺时针**。显示时需 **`π/2 - yaw`**（再归一化）。该修正落在独立 **GCS** 仓库（前端 Redux + `/odometry/filtered` 的 `yaw` 写入 `data_store`），与本仓库航点数学独立。

---

## 实船补充：MAVROS 用 `CommandHome` 对齐 PX4 HOME（模式 B）

真机仅用 **`/mavros/local_position/odom`** 且 **`map→odom` 恒等**时，局域里程原点随 **PX4 HOME**；需与当前 **`nav2`** 载入的 **`map.yaml` / `map_real_boat_hk.yaml`** 中约定的 **`ref_gnss*`** 角点一致。除 **QGroundControl「设 HOME」** 外，可在 ROS 2 调用服务 **`/mavros/cmd/set_home`**（**`mavros_msgs/srv/CommandHome`**）：**指定经纬度高程时必须 `current_gps: false`**（**`current_gps: true`** 为用**当前船位**，会忽略你填写的经纬度）。可复制命令、自检 **`/mavros/home_position/home`**、以及与 **`home_position/set` 话题**的区别，均以 **[`src/YILDIZ-USV/docs/实船调试.md`](../src/YILDIZ-USV/docs/实船调试.md)**「用 ROS 2 服务设置 PX4 HOME」为准。
