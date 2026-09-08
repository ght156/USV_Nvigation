# 后端相机压缩转发（防 host rviz 抢爆图传）

## 问题

`cleaning_boat.launch.py`（apriltag 容器）里的后端相机以
`/camera_back_video/color/image_raw`（1920×1080@10fps）发布裸图，约 **60MB/s**。
主机端 rviz 一订阅这条裸图，船↔地面无线链路立刻被抽空，图传(RTSP)延迟飙升到几千 ms。
Apriltag 输出（TF `dock_frame`、`/apriltag_node/detections`）只有几百字节，不是元凶；
纯粹是 **主机拉后端裸图** 在抢带宽。

## 方案

在船上本地把裸图降采样 + JPEG 压缩，再以 `CompressedImage` 发布到一个低带宽话题；
主机 rviz 改订这个压缩话题即可。Apriltag 继续在船内订裸图（不跨网），完全不受影响。

> 实测（船上域 5）：`/camera_back_video/color/image_raw/compressed` 约 **0.9 MB/s**
> （单帧 ~105KB，~9fps），对比裸图 1920×1080@10fps 的 **~62 MB/s**，下降约 **70 倍**。
> 若无线链路很窄，再加 `-p scale:=0.5`（960×540，约 0.25 MB/s）或 `-p max_fps:=5`。

## 船上启动

```bash
cd ~/jzw_ws/scripts/jzw_compressed_republish
bash run_compressed_republish.sh
```

脚本会自动 `source` ROS、并设 `ROS_DOMAIN_ID=5`（船上感知栈所在域）与
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。若栈换域，用 `ROS_DOMAIN_ID=<N>` 覆盖：

```bash
ROS_DOMAIN_ID=5 bash run_compressed_republish.sh
```

带自定义参数：

```bash
bash run_compressed_republish.sh \
  --ros-args \
  -p in_topic:=/camera_back_video/color/image_raw \
  -p out_topic:=/camera_back_video/color/image_raw/compressed \
  -p scale:=0.5 \
  -p jpeg_quality:=80 \
  -p max_fps:=8.0
```

## 主机端 rviz

把 Image / Camera 显示的 Topic 改成：

```
/camera_back_video/color/image_raw/compressed
```

rviz 用 compressed transport 解码即可显示。

**不要在主机端订阅 `/camera_back_video/color/image_raw`（裸图）**——那是 60MB/s 的元凶；
主机端一律用上面的压缩话题，带宽降到 1MB/s 级别，图传不再被抢。

**若主机缺 `compressed_image_transport` 插件**：节点加上
`-p publish_small_image:=true`，rviz 改订 `/camera_back_video/color/image_small`
（默认 jpeg 编码；要纯裸图更保险可加 `-p small_format:=raw`，但带宽更大）。

## 参数说明

| 参数 | 默认 | 说明 |
|------|------|------|
| `in_topic` | `/camera_back_video/color/image_raw` | 输入裸图 |
| `out_topic` | `/camera_back_video/color/image_raw/compressed` | JPEG 压缩输出 |
| `jpeg_quality` | 80 | JPEG 质量 1-100 |
| `scale` | 1.0 | 长宽缩放（0.5=半尺寸） |
| `target_width/height` | 0 | >0 则覆盖 scale，直接设输出分辨率 |
| `max_fps` | 0.0 | >0 限帧率，进一步降 CPU/带宽 |
| `publish_small_image` | false | 同时发一个给无压缩插件的 rviz 用 |
| `small_topic` | `/camera_back_video/color/image_small` | 上述降采样图话题 |
| `small_format` | `raw` | `raw`(bgr8，rviz 可直接显示)或 `jpeg` |

## 验证带宽

主机端对比（关/开 rviz）：

```bash
ros2 topic hz /camera_back_video/color/image_raw
ros2 topic bw /camera_back_video/color/image_raw
```

跑通后应看到裸图 `bw` 不再被 rviz 拉高；`/camera_back_video/color/image_raw/compressed`
的带宽远低于裸图。
