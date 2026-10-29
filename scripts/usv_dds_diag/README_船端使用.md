# USV DDS / 线程调度故障记录器

目的：在 raw / compressed 两种 MQTT 相机订阅配置下，连续记录 Linux 网络计数器、UDP socket 丢包、Fast DDS /dev/shm 文件规模、Nav2 与相关进程的逐线程运行队列等待时间和 CPU 运行时间、CPU/内存/IO pressure、ROS 2 日志关键事件。**只读，不改变任何 DDS、ROS 参数，不录制图像帧**。

## 环境

- 船端 Linux + Python 3（仅标准库，无 pip 安装）。
- 推荐在与 ROS 2 节点相同的 PID namespace / 宿主机运行；若 ROS 运行在容器中而采集脚本在另一容器，可能看不到进程。
- 最好用有权限读取 `/proc/<pid>/task/<tid>/schedstat` 和 ROS 日志的用户；如果没有，尝试 `sudo -E`，但必须指定正确的 `--ros-log-dir`。
- 建议运行前使用 `df -h` 检查可用空间；每秒一次的 JSON 采集每小时可能产生数十至数百 MB，取决于进程、线程和网络设备数。

## 运行

```bash
cd ~/usv_dds_diag
mkdir -p ~/usv_diag_runs
# 在 MQTT REAR_CAMERA=raw 配置下
python3 -u usv_dds_watch.py --mode raw --out ~/usv_diag_runs --minutes 180 \
  --interval 1 --ros-log-dir ~/.ros/log | tee ~/usv_diag_raw_console.log
```

完成 raw 后，将 MQTT 自己切到 compressed（脚本**不会**切换配置），以相同参数再采集：

```bash
python3 -u usv_dds_watch.py --mode compressed --out ~/usv_diag_runs --minutes 180 \
  --interval 1 --ros-log-dir ~/.ros/log | tee ~/usv_diag_compressed_console.log
```

可选：在 ROS 环境已经 source、ROS_DOMAIN_ID=5 和 RMW 设置正确的终端中加 `--topic /nav_status`，把该 topic 的 `ros2 topic echo` 输出保存到 `topic_echo.txt`。**仅用于辅助对时，不建议同时 echo 图像话题**。话题观察进程可能有少量额外开销，不必启用。

## 故障发生时

1. 记录平台上首次看到停船/故障的本地时间（尽量精确到秒）。
2. `CTRL_STUCK` 如出现在 ROS `.log` 中，脚本自动创建 `event_HHMMSS.json`，收集最近约 200 个采样概要，并复制部分 ROS 日志到 `event_logs/`。
3. 如果没有自动触发，在**另一个终端**执行：

```bash
pgrep -af usv_dds_watch.py
kill -USR1 <脚本PID>
```

或在当前运行目录中创建 `TRIGGER` 空文件：

```bash
touch ~/usv_diag_runs/<本轮目录>/TRIGGER
```

4. 建议故障发生后继续采集 1~3 分钟再 Ctrl-C，以便观察故障恢复或持续恶化。

## 查看摘要

```bash
python3 analyze_usv_watch.py ~/usv_diag_runs/<本轮目录>
```

完成两种模式采集后，将两个**完整运行目录**压缩：

```bash
tar -czf ~/usv_diag_runs.tar.gz -C ~ usv_diag_runs
```

上传压缩包，以及 raw/compressed 切换时间、导航任务开始/结束时间、故障时间和是否重启过船端主栈。

## 关键观测含义

- `Udp.RcvbufErrors`：内核 UDP 接收缓冲溢出计数；增长不等于已经证明 DDS 某个 topic 丢包。
- `udp_sockets[].drops`：各 UDP socket 累计丢包数和 inode，可辅助缩小范围；还需要 socket 所属进程映射才能完全归因。
- `processes[pid].threads.max_thread_wait_ns_delta`：采样期间等待 Linux 调度器运行的时间（一个线程的最大值）；不等于 DDS 消息端到端延迟。若 `/proc` schedstat 不可用或内核未启用，则不会有有效数据。
- `pressure.cpu`：CPU 资源竞争压力；平均 CPU 未达到 100% 时也可能存在单核竞争或短时调度延迟。
- `shm`：共享内存文件数量和逻辑文件长度，不是共享内存实际驻留占用量；不代表 DDS 的完整资源利用率。
- `fault_lines.jsonl`：扫描 ROS 日志发现的关键行，包括 `CTRL_STUCK`、控制环丢拍、3006/5001 等。

## 限制与注意

- 脚本并未直接测量 DDS writer -> reader 的消息排队时间、丢样次数或 Fast DDS 内部队列深度。要严格确认 DDS 堵塞，需要后续针对相关 topic 追加带时间戳/序列号的 publisher/subscriber tracing，或启用 Fast DDS Statistics / ros2_tracing；它们会改变运行时开销，应先用本脚本确定时间窗口。
- `--topic` 启动的 `ros2 topic echo` 可能因环境未 source 而失败，可查看 `topic_echo.txt`；不影响其他采集。
- 系统日志可能包含 IP 地址、设备标识、GPS 和路径信息；上传前可检查敏感内容。
- 事件自动检测依赖 ROS 日志里有匹配行；如果故障只出现在 MQTT 或平台上，手动触发即可。
