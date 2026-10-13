# 2026-10-12 双 RS485 实例 + launch 被 Ctrl-C 打断：取证方法与残留坑

## 1. 基本信息

| 项 | 内容 |
|---|---|
| 日期 | 2026-10-12 09:05–10:50（船端时间） |
| 船 / 工作空间 | 兴飞 MEC-MB002（`ssh hkzl`）/ `~/usv-main` |
| 两个现象 | ① rs485 驱动"无故"消失；② 之后船上同时跑着**两个** rs485 实例 |
| 数据来源 | `~/.ros/log/2026-10-12-09-05-19-725887-MEC-MB002-8931/launch.log`、`~/.ros/log/usv_rs485_driver_node_8934_1791767119819.log`、`ps -eo pid,ppid,pgid,lstart,tty,cmd` |
| 同类记录 | `2026-10-05_节点残留致modbus冲突与鬼节点.md` |

## 2. 现象

1. 09:40:22 正在跑的 rs485（launch 8931 / node 8934）消失，`/kvmsbms_state`、`/wireless_charger_state` 停发；
2. 10:05 按脚本方式重新拉起一个（launch 29893 / node 29918，pid 文件写 `~/logs/xingfei_stack/rs485.pid`）；
3. 10:22 船上又冒出第二个 rs485（launch 33826 / node 33829）——**两个实例同时轮询同一条半双工总线**。

## 3. 证据（怎么确定"是谁杀的"）

节点自己的文件日志只有一行（时间戳精确到 ns），launch.log 里直接写了打断者的身份：

```text
# ~/.ros/log/usv_rs485_driver_node_8934_1791767119819.log（全文）
[INFO] [1791769222.522807656] [rclcpp]: signal_handler(SIGINT/SIGTERM)

# ~/.ros/log/2026-10-12-09-05-19-725887-MEC-MB002-8931/launch.log（尾部）
1791769222.5232778 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
1791769227.5287197 [ERROR] [usv_rs485_driver_node-1]: process[usv_rs485_driver_node-1] failed to
             terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
1791769227.5344667 [ERROR] [usv_rs485_driver_node-1]: process has died
             [pid 8934, exit code -15, cmd '.../usv_rs485_driver_node ...']
```

- `1791769222` = **2026-10-12 09:40:22**（`date -d @1791769222`）；
- `user interrupted with ctrl-c` → 是**终端里按的 Ctrl-C**；同时可排除：
  OOM（`free` 有余量、无内存告警）、串口/IO 报错（日志里没有）、脚本 `stop`（当时没跑过 stop）、
  崩溃段错误（exit -15 = 收到 SIGTERM 后退出，不是 -11/异常码）；
- 定位到具体终端：该 launch 的父进程是 `pts/0` 上的交互 `-bash`，`w` 显示 `pts/0` 来自 `10.168.1.100`
  （调试笔记本）→ 就是那个终端窗口里按下的 Ctrl-C。

双实例证据：

```text
$ ps -eo pid,ppid,pgid,lstart,tty,cmd | grep "ros2 launch usv_rs485"
29893     1 29893 Mon Oct 12 10:05:06 2026  ?      ros2 launch usv_rs485_driver usv_rs485_driver.launch.py  # 脚本起的
33826 25923 33826 Mon Oct 12 10:22:15 2026  pts/0  ros2 launch usv_rs485_driver usv_rs485_driver.launch.py  # 终端里手动起的
```

## 4. 根因（已确证）

1. **被打断**：这个 rs485 是当天 09:05 在交互终端里**前台**手动起的（不走脚本），09:40 终端里按了 Ctrl-C →
   信号打到前台进程组 → launch + node 一起退（node 错过了 SIGINT，5 s 后升级 SIGTERM，exit -15）。
2. **双实例**：`start_xingfei_stack.sh start` 只按 `~/logs/xingfei_stack/<名>.pid` **判活**：
   手动起的实例不写这个 pid 文件 → 脚本既不会拦重复、也不认为它"已在跑"；
   而它的 `stop` 里也没有 rs485 的兜底 `pkill` 模式 → **手动实例在整栈 stop 后依然活着**。
   本次就是"脚本实例 + 手动实例"并存（抢 modbus 总线，属鬼节点同类问题）。

## 5. 已做修改

- 处置顺序（本次实做）：整栈 `~/start_xingfei_stack.sh stop` → 手动实例按进程组
  `kill -INT/-TERM/-KILL -33826` → 确认 `ros2 launch` 与 `usv_rs485_driver_node` 都为 0 →
  再 `start`，最终**单实例**（launch 38882）在跑，12 项全 UP（含 livox）。
- **未改脚本**（`stop` 的 pkill 模式、`start` 的判活逻辑都没动）。

## 6. 待验证 / 下一步

1. 手动起栈内节点前先 `pgrep -af "ros2 launch"` 查重，或干脆只用脚本起/停；
   若必须手动起，建议顺手把 pid 写进 `~/logs/xingfei_stack/<名>.pid`（本次没做）。
2. 可考虑给 `stop` 补 rs485 / livox 的兜底 `pkill` 模式（与既有 `docking_fsm` 等一致），待现场确认。
3. 通用取证套路（以后"节点莫名消失"照这个走）：
   **节点文件日志找 `signal_handler(SIGINT/SIGTERM)` → launch.log 找 `user interrupted with ctrl-c` /
   `escalating to` / `process has died` → `ps -o ppid,tty` + `w` 定位到具体终端**。
