# bw_gi320_driver

## 1. 功能简介

`bw_gi320_driver` 是基于 ROS2 的 GI320 RTK 驱动功能包，用于接收 RTK 设备通过 UDP 输出的 NMEA 数据，并完成：

* UDP 数据接收
* NMEA 协议解析

  * GNGGA
  * INSPVAA
* RTK 定位数据发布
* RTK 经纬度里程累计计算
* 里程计清零服务

适用于无人船、移动机器人等需要 RTK 高精度定位的应用场景。


---

## 2. 数据流程

整体数据流程如下：

```
             UDP
              |
              |
          GI320 RTK
              |
              |
      +----------------+
      | UDP Receiver   |
      +----------------+
              |
              |
        NMEA Parser
              |
       +------+------+
       |             |
     GNGGA        INSPVAA
       |             |
       |             |
  GGA里程累计   INSPVAA里程累计
       |             |
       |             |
 Gi320Gngga.msg  Gi320Inspvaa.msg
       |             |
       +-------------+
              |
          ROS2 Topic
```

---


# 3. ROS2 Topic

## 3.1 GNGGA数据

Topic:

```
/gi320/gngga
```

消息类型：

```
bw_gi320_driver/msg/Gi320Gngga
```

包含：

* RTK定位信息
* UTC时间
* GGA累计里程

---

## 3.2 INSPVAA数据

Topic:

```
/gi320/inspvaa
```

消息类型：

```
bw_gi320_driver/msg/Gi320Inspvaa
```

包含：

* RTK位置
* 姿态角
* 速度
* INSPVAA累计里程

---

# 4. 里程计说明

驱动内部维护两个独立里程计：

```
gga_odom_

inspvaa_odom_
```

两者互不影响。

---

## 4.1 GGA里程

计算方式：

```
当前GPS坐标
        |
        |
Haversine距离计算
        |
        |
累计距离
```

特点：

* 基于定位点变化
* 依赖GGA输出频率
* 适合记录轨迹距离

---

## 4.2 INSPVAA里程

计算方式：

```
当前GPS坐标
        |
        |
Haversine距离计算
        |
        |
累计距离
```

特点：

* 独立统计
* 与GGA里程分开
* 可用于导航距离统计

---

# 5. ROS2 Service

## 5.1 清零GNGGA里程

服务：

```
/gi320/reset_gga_odom
```

类型：

```
bw_gi320_driver/srv/ResetGgaOdom
```

调用：

```bash
ros2 service call \
/gi320/reset_gga_odom \
bw_gi320_driver/srv/ResetGgaOdom {}
```

返回：

```yaml
success: true
message: "GGA odometer reset success"
```

---

## 5.2 清零INSPVAA里程

服务：

```
/gi320/reset_inspvaa_odom
```

类型：

```
bw_gi320_driver/srv/ResetInspvaaOdom
```

调用：

```bash
ros2 service call \
/gi320/reset_inspvaa_odom \
bw_gi320_driver/srv/ResetInspvaaOdom {}
```

---

# 6. 时间说明

发布消息中的：

```
builtin_interfaces/Time stamp
```

使用 RTK 输出的 UTC 时间。

数据流程：

```
RTK UTC时间

      |

转换Unix时间戳

      |

ROS Message stamp
```

不是 ROS 接收数据时间。

这样可以避免 UDP 网络延迟造成时间偏差。

---

