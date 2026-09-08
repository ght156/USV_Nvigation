# usv_ardupilot_velocity_bridge

用于将 Nav2 输出的速度指令桥接到 ArduPilot / MAVROS，并结合实船测试得到的 **`vx-ω` 二维速度可行域**，对不符合船体实际运动能力的速度指令进行约束。

## 1. 数据链路

```text
Nav2 / RPP
    |
    | geometry_msgs/msg/Twist
    v
/cmd_vel_nav
    |
    v
ardupilot_velocity_bridge
    |
    |-- 指令超时保护
    |-- 角速度绝对限幅
    |-- 左/右转非对称 vx-ω 可行域查询
    |-- 超出可行域时优先降低 vx
    |-- 线加速度 / 减速度限制
    |-- 角加速度限制
    v
/mavros/setpoint_velocity/cmd_vel_unstamped
    |
    | geometry_msgs/msg/Twist
    v
MAVROS
    |
    v
ArduPilot
    |
    v
左右推进器
```

本节点只负责速度指令桥接和船体可行域约束。

**不负责：**

- 自动切换 ArduPilot 到 GUIDED 模式；
- 自动解锁 ARM；
- 直接输出左右电机 PWM。

---

## 2. 为什么需要 `vx-ω` 速度包络

实船测试表明，前进线速度 `vx` 和角速度 `ω` 并不是相互独立的。

在较低或中等前进速度下，船可以产生较大的转向角速度；但随着前进速度增加，可实现的最大角速度明显下降。

因此，简单采用：

```text
vx <= vx_max
|omega| <= omega_max
```

这种相互独立的矩形限幅，并不能准确描述实船的运动能力。

当前 bridge 使用实船数据建立：

```text
vx_max = f(omega)
```

形式的二维速度包络。

同时，由于实测中左转和右转能力并不完全对称，因此正、负 `omega` 分别使用不同的包络曲线。

---

## 3. 超出可行域时如何处理

基本原则：

> **尽量保留 Nav2 / RPP 的转向需求，优先降低前进速度。**

例如 RPP 输出：

```text
vx_cmd = 0.90 m/s
omega_cmd = 0.25 rad/s
```

实船包络显示：

```text
vx_max(0.25) ≈ 0.65 m/s
```

若：

```text
envelope_safety_factor = 0.90
```

则实际允许：

```text
vx_limit ≈ 0.65 × 0.90
         ≈ 0.585 m/s
```

bridge 最终输出约为：

```text
vx_out    = 0.585 m/s
omega_out = 0.25 rad/s
```

这样做的原因是：当 RPP 已经要求较大角速度时，说明当前路径需要明显转弯。

如果仍保持高速前进、反而降低 `omega`，会使船继续向前冲而转不过来，通常会进一步增大路径跟踪误差。

---

## 4. PWM / 推进器数据的作用

本次实船数据不仅包含：

```text
vx
omega
```

还包含 RC、SERVO / PWM 等推进器相关数据。

分析表明，可以定义：

```text
PWM_common = (SERVO1 + SERVO3) / 2
```

用于表征左右推进器的共同推力分量，它主要与前进运动相关。

同时定义：

```text
PWM_diff = (SERVO1 - SERVO3) / 2
```

用于表征左右推进器的差动分量，它与船体转向角速度具有很强的关联。

部分高角速度测试点中，还可以观察到某一路推进器逐渐接近 PWM 输出边界。

这说明实测中：

```text
前进速度越高
        ↓
可用于左右差动的推进器余量越小
        ↓
最大可实现角速度下降
```

因此，`vx-ω` 速度包络的收缩与推进器的差动推力能力及 PWM 饱和存在明显关系。

---

## 5. 当前第一版为什么不直接控制 PWM

当前版本中：

```text
RC / SERVO / PWM / vx / omega 实船数据
                    |
                    v
             离线分析与标定
                    |
                    v
              vx-omega 包络
                    |
                    v
        ardupilot_velocity_bridge
```

PWM 数据用于帮助判断和校准船体速度可行域。

但运行时 bridge 仍然向 MAVROS 发送：

```text
(vx, omega)
```

速度目标，而不是直接发送左右推进器 PWM。

这样可以继续让 ArduPilot 负责底层推进器控制，避免在尚未充分验证推进器模型的情况下绕过现有底层闭环。

---

## 6. 当前 bridge 的处理顺序

每收到一组新的 Nav2 速度命令后：

```text
(vx_cmd, omega_cmd)
        |
        v
1. 判断指令是否超时
        |
        v
2. 对 omega 做绝对能力限幅
        |
        v
3. 根据 omega 正负选择左/右转速度包络
        |
        v
4. 对包络进行线性插值
   得到 vx_max(omega)
        |
        v
5. 乘 envelope_safety_factor
        |
        v
6. vx_out = min(vx_cmd, vx_max)
        |
        v
7. 线加速度 / 减速度限制
        |
        v
8. 角加速度限制
        |
        v
9. 发布 Twist 到 MAVROS
```

如果 Nav2 指令超过设定时间没有更新，则 bridge 输出零速度。

---

## 7. MAVROS 转向正负方向

当前节点发布：

```text
/mavros/setpoint_velocity/cmd_vel_unstamped
```

消息类型：

```text
geometry_msgs/msg/Twist
```

当前 MAVROS 配置已经完成需要的 ENU -> NED 坐标转换，因此 bridge 内部**不要再次对 `angular.z` 取反**。

当前约定：

```text
angular.z > 0  -> 左转
angular.z < 0  -> 右转
```

除非后续修改 MAVROS 配置，否则不要额外增加转向符号转换。

---

## 8. 启动方式

直接运行：

```bash
ros2 run usv_ardupilot_velocity_bridge ardupilot_velocity_bridge --ros-args \
  -p input_cmd_topic:=/cmd_vel_nav \
  -p publish_rate_hz:=20.0
```

也可以通过 launch 启动：

```bash
ros2 launch usv_ardupilot_velocity_bridge ardupilot_velocity_bridge.launch.py
```

---

## 9. 主要参数

当前 bridge 主要涉及：

```text
state_topic
input_cmd_topic          # 旧版单话题参数，保留兼容
input_cmd_topics         # 输入话题列表（launch 用逗号分隔字符串传入）
input_cmd_qos            # 输入订阅 QoS：reliable / best_effort（默认 best_effort）
output_cmd_topic

publish_rate_hz
command_timeout_sec

max_linear_x
max_linear_y
max_linear_z
max_angular_z

enable_velocity_envelope
envelope_safety_factor

max_positive_yaw_rate
max_negative_yaw_rate

max_linear_accel
max_linear_decel
max_angular_accel
```

### 输入话题与 QoS

`input_cmd_topics` 支持配置多个输入话题，每个话题各建一个订阅，共同写入同一份当前指令状态（后到者覆盖先到者），任一来源停发后由超时保护统一归零。

`input_cmd_qos` 默认 `best_effort`。按 DDS 兼容规则（订阅要求 ≤ 发布提供），**best_effort 订阅可以同时匹配 reliable 和 best_effort 两种发布者**，因此不同节点以不同 QoS 发布速度指令都能被接收；若上游要求可靠传输，可改回 `reliable`（此时只匹配 reliable 发布者）。

```bash
ros2 run usv_ardupilot_velocity_bridge ardupilot_velocity_bridge --ros-args \
  -p "input_cmd_topics:=[\"/cmd_vel_nav\",\"/cmd_vel_auto\"]" \
  -p input_cmd_qos:=best_effort
```

实际参数名称以当前 `ardupilot_velocity_bridge.cpp` 和 `ardupilot_velocity_bridge.launch.py` 为准。

---

## 10. 第一轮实船测试建议

第一轮测试暂时**不要修改 RPP 源码，也不要加入在线 PWM 推进器分配模型**。

先单独验证：

> 根据实船 `vx-ω` 可行域主动降低转弯时的前进速度，能否明显改善 Nav2 / RPP 路径跟踪效果。

建议至少记录：

```bash
ros2 bag record \
  /cmd_vel_nav \
  /mavros/setpoint_velocity/cmd_vel_unstamped \
  /mavros/local_position/velocity_local \
  /mavros/local_position/odom \
  /mavros/state
```

如果当前 MAVROS / ArduPilot 已经发布 RC、SERVO 或 actuator 输出，也建议一起录制。

测试后重点比较：

```text
RPP 原始 vx、omega
        ↓
bridge 输出 vx、omega
        ↓
实船实际 vx、omega
        ↓
SERVO1 / SERVO3 PWM
```

重点观察：

- 直线航行时 bridge 是否基本不干预；
- 入弯时是否能够提前降低前进速度；
- 实际角速度是否更接近 RPP 的要求；
- 转弯半径是否明显减小；
- 横向路径误差是否下降；
- 是否还存在明显的左右转向差异；
- bridge 是否限速过于保守；
- 高曲率转弯时是否仍出现某一路 PWM 饱和。

---

## 11. 第二版：推进器分配能力模型

如果第一版测试后仍存在明显误差，并且误差与推进器 PWM 饱和高度相关，可以进一步建立：

```text
(vx, omega)
     |
     v
推进器能力模型
     |
     +--> PWM_L_pred
     |
     +--> PWM_R_pred
```

即拟合：

```text
PWM_L = f_L(vx, omega)
PWM_R = f_R(vx, omega)
```

运行时可以预测某个 RPP 速度指令需要的左右推进器 PWM。

如果预测结果超过推进器允许范围，例如：

```text
PWM_L < PWM_min

或

PWM_R > PWM_max
```

则说明该 `(vx, omega)` 指令从推进器能力上不可实现。

bridge 可以在保持 `omega` 需求的前提下继续降低 `vx`，直到：

```text
PWM_min <= PWM_L <= PWM_max
PWM_min <= PWM_R <= PWM_max
```

这样得到的可行域会比固定 `vx-omega` 查表具有更明确的推进器物理意义。

---

## 12. 推荐开发顺序

### Version 1：当前版本

```text
RPP
 |
 v
ardupilot_velocity_bridge
 |
 |-- 实船 vx-omega 包络
 |-- 左右转非对称限制
 |-- safety factor
 |-- 加速度限制
 |
 v
MAVROS
 |
 v
ArduPilot
```

目的：

> 先验证二维速度包络是否能够解决“高速前进时船转不动”的主要问题。

### Version 2：实船验证后

```text
RPP
 |
 v
(vx_cmd, omega_cmd)
 |
 v
推进器能力预测
 |
 +--> PWM_L_pred
 +--> PWM_R_pred
 |
 v
PWM 饱和判断
 |
 v
动态求解可实现 vx
 |
 v
MAVROS / ArduPilot
```

目的：

> 用推进器实际能力进一步替代或修正经验速度包络。

---

## 13. 当前建议

现阶段建议保持：

```text
Nav2 / RPP
     |
     v
ardupilot_velocity_bridge
     |
     |-- vx-omega 二维可行域
     |-- 左右转非对称约束
     |-- 转弯优先降 vx
     |-- 加速度约束
     |
     v
MAVROS
     |
     v
ArduPilot
     |
     v
推进器
```

暂时不要修改 RPP 源码。

也暂时不要让 bridge 直接控制左右推进器 PWM。

先完成一轮 Version 1 实船测试，并同时记录：

```text
RPP 原始命令
bridge 输出命令
实际 vx / omega
SERVO1 / SERVO3 PWM
```

再根据测试结果决定是否升级到 Version 2 的推进器分配能力模型。
