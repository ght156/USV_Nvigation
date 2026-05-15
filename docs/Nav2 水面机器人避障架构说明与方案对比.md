# Nav2 水面机器人避障架构说明与方案对比

## 一、当前项目避障架构（现状）

你当前采用的是一种：

> **“全局规划 + 局部感知 + 路径跟踪式避障”**

的经典工程化架构。

整体结构：

```text
global_costmap (map)
    static_layer
    + voxel_layer(远距预判)
    + inflation

        ↓

planner_server
    生成全局路径

        ↓

local_costmap (odom)
    voxel_layer(实时障碍)
    + inflation

        ↓

controller_server
    Regulated Pure Pursuit (RPP)

        ↓

collision detection
+ cost scaling
+ velocity regulation
```

------

# 二、两层 Costmap 的职责分工

## 1. global_costmap

运行坐标系：

```text
map
```

通常为固定窗口或整图。

主要作用：

```text
用于全局路径规划
```

即：

- 绕开静态障碍
- 基于海图规划
- 提前规避已知区域
- 给 planner_server 提供代价地图

你的结构：

```text
static_layer
+ voxel_layer(远距感知)
+ inflation
```

其中：

### static_layer

来自：

- 海图
- 栅格地图
- 已知障碍

这是全局规划核心。

------

### global voxel_layer

作用：

```text
远距离动态障碍预判
```

例如：

- 40m 外船只
- 临时浮标
- 远距离雷达点云

planner 可以提前绕行。

但它不是必须项。

------

### inflation_layer

作用：

```text
给障碍物外围增加代价缓冲
```

避免贴边航行。

你现在：

```text
inflation_radius: 1.8m
```

比较适合船。

------

# 2. local_costmap

运行坐标系：

```text
odom
```

采用滚动窗口。

例如：

```text
20m × 15m
```

主要作用：

```text
实时局部避障
```

即：

- 实时障碍检测
- 临近碰撞规避
- 给 controller_server 提供代价

------

你的结构：

```text
voxel_layer
+ inflation_layer
```

其中：

## local voxel_layer

这是：

# 真正的实时避障核心

作用：

```text
把实时点云/雷达障碍
投影到局部代价地图
```

RPP 才能：

- 减速
- 偏航
- 绕障
- 防撞

如果关闭 local voxel：

```text
RPP 将“看不见”动态障碍
```

------

## local inflation

作用：

```text
给船体保留安全距离
```

防止：

- 擦碰
- 贴边
- 激进绕障

------

# 三、RPP（Regulated Pure Pursuit）在避障中的作用

很多人误以为：

```text
RPP 只是路径跟踪器
```

实际上：

现代 Nav2 的 RPP 已经具备：

# “轻量级局部避障能力”

包括：

- collision checking
- curvature regulation
- cost-aware velocity scaling
- time-to-collision checking

------

## 1. collision_detection

用于：

```text
预测当前轨迹是否会撞障碍
```

如果前方障碍进入碰撞区域：

```text
减速 / 停船
```

------

## 2. cost_scaling

根据 costmap 代价：

```text
自动降低速度
```

即：

```text
离障碍越近
速度越低
```

这是非常重要的工程能力。

------

# 四、你当前架构的本质定位

你的系统本质上属于：

# “路径跟踪式反应避障”

即：

```text
已有主路径
+
局部实时修正
```

它不是：

```text
全自由轨迹优化器
```

------

这类系统特点：

| 特点         | 表现     |
| ------------ | -------- |
| 稳定性       | 很高     |
| 参数复杂度   | 低       |
| 工程可靠性   | 强       |
| 适合大空间   | 非常适合 |
| 动态穿行能力 | 中等     |
| 极限机动性   | 一般     |

------

# 五、DWA / TEB 与当前方案对比

------

# 1. DWA（Dynamic Window Approach）

核心思想：

```text
实时采样大量速度组合
```

尝试：

- 左转
- 右转
- 加速
- 减速

然后选：

```text
当前最优轨迹
```

------

## 优点

### 灵活

适合：

- 狭窄空间
- 小机器人
- 室内 AGV

------

### 可主动绕障

即使原路径被挡：

```text
也能临时寻找替代轨迹
```

------

## 缺点

### 参数复杂

典型问题：

- 振荡
- 左右抖动
- 原地转圈
- 速度不稳定

------

### 对动力学依赖强

船艇：

- 惯性大
- 转向慢
- 无法急停

DWA 很容易：

```text
规划出理论可行
但船实际做不到的轨迹
```

------

# 2. TEB（Timed Elastic Band）

TEB 本质：

# “时空轨迹优化器”

它优化：

```text
轨迹 + 时间
```

会同时考虑：

- 动力学
- 障碍
- 时间
- 曲率
- 速度

------

## 优点

### 非常聪明

适合：

- 人群穿行
- 动态环境
- 窄空间导航

------

### 轨迹平滑

比 DWA 更自然。

------

## 缺点

### 参数地狱

非常难调。

------

### 对模型要求高

需要：

- 准确 footprint
- 准确速度模型
- 准确运动学

------

### 工程稳定性一般

很多项目：

最终会从 TEB 回退。

原因：

```text
“太聪明，但不够稳”
```

------

# 六、为什么 RPP 更适合船

船与 AGV 最大区别：

| AGV        | 船           |
| ---------- | ------------ |
| 可急停     | 惯性大       |
| 可原地转向 | 转向半径大   |
| 低速       | 中高速       |
| 室内       | 户外开放水域 |

------

因此：

船更适合：

```text
平滑路径
+
稳定跟踪
+
渐进避障
```

而不是：

```text
高频局部轨迹搜索
```

------

RPP 的优势：

| 能力       | 表现     |
| ---------- | -------- |
| 稳定       | 很强     |
| 平滑       | 很好     |
| 参数量     | 少       |
| 工程可靠性 | 高       |
| 水面适配性 | 非常适合 |

------

# 七、VoxelLayer 与 STVL 对比

------

# 1. VoxelLayer

特点：

```text
只有空间维度
```

它知道：

```text
这里有障碍
```

但不知道：

```text
障碍是否已经离开
```

------

## 优点

- 官方维护稳定
- 生态成熟
- 配置简单
- 工程可靠

------

## 缺点

容易出现：

```text
ghost 残留
```

特别是：

- 点云抖动
- clearing 不充分
- TF 漂移
- 动态环境

------

# 2. STVL（Spatio-Temporal Voxel Layer）

核心：

# 给 voxel 增加时间维度

即：

```text
障碍多久没再观测到？
```

超过时间：

```text
自动衰减删除
```

------

## 优点

### 非常适合动态环境

包括：

- 行人
- 车辆
- 动态船只
- 水面反射环境

------

### Ghost 少

因为：

```text
旧障碍会自动过期
```

------

## 缺点

### 生态不如 voxel 稳定

包括：

- 文档较少
- 社区案例少
- 版本兼容性一般

------

# 八、当前项目推荐路线

------

# 推荐结构（第一阶段）

```text
global_costmap
    static_layer
    inflation_layer

local_costmap
    voxel_layer
    inflation_layer

controller
    RPP
```

即：

```text
global 不开 voxel
local 保留 voxel
```

这是当前最稳方案。

------

# 第二阶段（高级优化）

如果后续出现：

- ghost 严重
- 动态障碍增多
- clearing 难调
- 码头复杂环境

再升级：

```text
local voxel
→ STVL
```

即可。

------

# 九、Ghost 残留处理建议

| 场景         | 推荐方案                |
| ------------ | ----------------------- |
| 水面正常航行 | global voxel 可开       |
| 码头/岸测    | global voxel 关闭       |
| 动态障碍多   | local 使用 STVL         |
| 纯导航测试   | local/global voxel 全关 |

------

# 十、最终结论

你的方案：

```text
RPP + local voxel + global planner
```

并不是“低级避障”。

相反：

它属于：

# “工业稳定型导航方案”

特别适合：

- 船
- 户外
- GNSS
- 大空间
- 稀疏障碍环境

------

相比：

```text
DWA / TEB
```

你当前方案：

- 更稳定
- 更容易调试
- 更符合船动力学
- 更适合长期工程运行

------

真正应该优先优化的不是：

```text
换 DWA / TEB
```

而是：

# 更高质量的 Costmap

包括：

- 点云质量
- clearing
- footprint
- inflation
- obstacle persistence
- 全局路径平滑
- behavior tree recovery

这些才是真正决定工程表现的关键。