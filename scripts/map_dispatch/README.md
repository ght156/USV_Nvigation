# 地图工具 → 船端航点下发（ros_map_tool 补丁）

把 `ros_map_tool` 航点编辑器里**选中的 map 坐标航点**，一键下发到船端跑起来。
下发的是 WGS84 经纬度（船端接口约定），转换用船端 `gps_map_conversion.py` 的同一套
ENU + `map.yaml` origin 公式，datum 取 `map.yaml` 的 `ref_gnss_10`。

## 为什么走 ssh 而不是 rosbridge

* 船端（兴飞 NX / MEC-MB002）**没装 `rosbridge_server`**，浏览器连不上 DDS；
* 地面站笔记本不需要装 ROS，浏览器 + ssh 就够了；
* 跨网段 / 4G 时 DDS 发现（多播）基本不通，而 ssh 只要 TCP 22 通即可。

链路：

```
浏览器（工具页「📤 下发到船端」）
   │ fetch POST http://127.0.0.1:8770/api/dispatch
   ▼
waypoint_dispatch_bridge.py（地面站本机）
   │ map(x,y,yaw) → WGS84(lat,lon,yaw°)，并用 map.yaml 锚点做包围盒安全检查
   │ ssh <host> → python3 - < remote_nav_cli.py（源码经 stdin 送入，船端不落盘）
   ▼
船端 mission_bridge /mission_bridge/send_waypoints（或 /waypoint 话题）→ Nav2 开跑
```

## 文件

| 文件 | 跑在哪 | 作用 |
| --- | --- | --- |
| `ros_map_tool_v3_nav2_live.html` | 浏览器 | 打过补丁的页面：航点勾选列表 + 下发面板（暂停/继续/取消/状态） |
| `waypoint_dispatch_bridge.py` | 地面站本机 | HTTP 桥（默认 `127.0.0.1:8770`），做坐标换算 + ssh 转发 |
| `remote_nav_cli.py` | 船端（一次性，经 stdin） | rclpy 客户端：`check/status/dispatch/cancel/pause`，输出 `@@RESULT@@ {json}` |
| `deploy_to_map_tool.sh` | 地面站本机 | 部署页面与脚本到 `/home/ght/ros_map_tool-main`（自动备份旧页面） |

除下发面板外，页面还做了一处小改动：原来 `three.js` 是**静态 import**（CDN 不通时整个
模块脚本不执行、所有按钮失灵），现在改成只在点「⚡ 生成地图」时**动态 import**。
没有外网的现场照样能加载 `yaml + pgm`、规划航点、下发到船端。

## 用法

```bash
# 1) 本机起桥（新终端里常驻；日志直接打屏）
python3 /home/ght/USV_NAV/scripts/map_dispatch/waypoint_dispatch_bridge.py

# 2) 打开页面：直接访问桥托管的页面（同源，最省事）
#    http://127.0.0.1:8770/
#    或者双击 /home/ght/ros_map_tool-main/ros_map_tool_v3_nav2_live.html

# 3) 页面操作
#    「🛠️ 地图编辑器」加载 map.yaml + map.pgm（船端同一张图）
#    「🚩 航点编辑器」左键点住拖出朝向 → 右侧列表勾选要下发的点
#    「🔌 检查船端」→ 看船端是否 IDLE、服务是否在；
#       2026-10-09 起会自动顺带从船端 map.yaml 读回 datum/锚点（不覆盖手工输入），
#       也可随时点「📡 从船端读 datum」手动读 —— 不加载 yaml 也能下发
#    「🧮 只换算（不下发）」→ 先核对经纬度对不对（不动船）
#    「📤 下发选中航点」→ 二次确认后下发，船开跑
```

命令行自检（不用浏览器）：

```bash
B=scripts/map_dispatch/waypoint_dispatch_bridge.py
python3 $B --check                  # 船端自检：环境 / 服务 / /nav_status（只读）
python3 $B --status                 # 读一次 /nav_status
python3 $B --datum                  # 读船端 map.yaml 的 ref_gnss 锚点/datum（只读）
python3 $B --dispatch-file p.json --dry-run   # 只做坐标换算，不发船端
python3 $B --cancel                 # 取消船端任务
python3 $B --pause 1                # 暂停（0 = 继续）
```

## 下发方式与朝向

| 方式 | 接口 | 朝向 | 启动 |
| --- | --- | --- | --- |
| 服务（默认） | `/mission_bridge/send_waypoints` | 用规划朝向（`yaw` 度） | 立即执行，`RUNNING` 时自动抢占 |
| 话题 | `/waypoint`（WGS84 JSON） | 不指定 → 船端用行进方向 | debounce ≈0.45s 自动开始 |

「朝向」下拉里选「不指定（行进方向）」时，服务方式也会下发 `yaw=65536` 哨兵值，
由船端按上一航点到本点的行进方向决定船头朝向。

## 安全设计

* **包围盒校验**：用 `map.yaml` 的 `ref_gnss_00/01/10/11` 算经纬度范围（±50 m），
  换算结果落在外面的航点直接拒绝下发 —— 防止「底图/datum/原点不匹配」把船指到别处。
  （`--dispatch-file` 时可用 `allow_out_of_bounds: true` 跳过，网页上不给这个开关。）
* **二次确认**：网页弹窗列出全部待下发航点的 map 坐标，确认后才 ssh。
* **船端仍会校验**：`mission_bridge` 自己做经纬度合法性 + 作业区/禁航区围栏校验，
  失败原因会原样回显到页面「下发状态」里。
* **不会静默动手**：只在你点「📤 下发选中航点 / ⏸ / ▶ / 🛑」时才碰船；「检查船端 /
  只换算 / 状态刷新」全是只读（订阅 `/nav_status`、查服务是否在线）。

## 前置条件

* 地面站：能 `ssh hkzl`（`~/.ssh/config` 里已有别名；桥默认就用 `hkzl`，网页上可改）。
* 船端：`nav2_boat.launch.py` + `mission_bridge.launch.py` 已起（`mission_bridge` 节点在跑、
  `/nav_status` 有数据），且状态为 `IDLE`；`WAITING_SYSTEM` 时 `send_waypoints` 会返回失败。
* 船端 `map.yaml` 与页面加载的底图必须是同一张（兴飞：`~/usv-main/src/USV_NAV/workspace_nav/map/map.pgm`，
  已核对与 `ros_map_tool-main/map.pgm` 逐字节一致 md5 `ec660bb3…`）。

## 排错

| 现象 | 原因 / 处理 |
| --- | --- |
| 页面报「连不上桥接服务」 | 桥没起（`python3 waypoint_dispatch_bridge.py`）；或改用 `http://127.0.0.1:8770/` 打开页面避开 CORS/私网限制 |
| `❗ datum 未设置` | 页面加载的 yaml 里没有 `ref_gnss_10`（例如 PCD 生成的新图）→ 点「📡 从船端读 datum」直接取船端 map.yaml，或手工填 datum 经纬度 |
| 「航点超出地图锚点范围」 | 底图/原点/datum 与船端不一致，先核对两边 `map.yaml` |
| 船端返回 `System not ready (WAITING_SYSTEM)` | TF 或 Nav2 action 未就绪（定位/导航栈没起全），先看 `--check` 输出 |
| 定位 `DEGRADED/LOST` | 船端没有 GPS fix（室内/无 RTK）时会出现；导航起来后会飘，先解决定位 |
| 检查船端要 5 s 左右 | 正常：每次都是一次 ssh + source 船端工作区 + rclpy 起进程 |
