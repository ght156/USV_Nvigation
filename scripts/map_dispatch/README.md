# 地图工具 → 船端航点下发（ros_map_tool 补丁）

把 `ros_map_tool` 航点编辑器里**选中的 map 坐标航点**，一键下发到船端跑起来。
下发的是 WGS84 经纬度（船端接口约定），转换用船端 `gps_map_conversion.py` 的同一套
ENU + `map.yaml` origin 公式，datum 取 `map.yaml` 的 `ref_gnss_10`。

## 为什么走 ssh 而不是 rosbridge

* 地面站笔记本**不需要装 ROS**：浏览器 + ssh 就够了（也便于复现：一条命令跑完就退）；
* 跨网段 / 4G 时 DDS 发现（多播）基本不通，而 ssh 只要 TCP 22 通即可。

补充（2026-10-16 更正）：兴飞船端**已经装了 `rosbridge_server`**（2026-10-09 起，
现为 `~/start_xingfei_stack.sh` 的第 12 项，随栈自启），页面也能用 `ws://<船IP>:9090`
看实时数据；但**下发链路仍然走 ssh**，与 rosbridge 无关。
另注意 rosbridge 全双工不限流，别订 costmap / 点云类话题（同《兴飞船调试文档》§6 的带宽教训）。

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

## ⚠️ 中间件（RMW）必须与船端一致 —— 2026-10-16 实测

船端 2026-10-15 起全船走 **FastDDS**（`rmw_fastrtps_cpp`），桥的默认值已跟着改；
澄丰等仍跑 CycloneDDS 的船用 `--rmw rmw_cyclonedds_cpp` 起桥（或请求体里带 `"rmw"`）。

不一致时的症状**很容易误判**：发现层和话题照旧正常（页面「🔌 检查船端」里服务全 ✅、
`/nav_status` 也读得到），但**服务请求-应答静默超时**，页面报
`下发失败：服务 /mission_bridge/send_waypoints 超时（8s）无响应`。
2026-10-16 同日实测 A/B：老默认值下 `/mission_bridge/get_geofence` 无响应；
改 `rmw_fastrtps_cpp` 立即正常返回。
因此 `check` 里加了 `service_roundtrip`（真发一次只读 `get_geofence`）：
**它才是"客户端能不能下发"的判据，服务列表绿不代表能下发。**

```bash
# 兴飞（默认，FastDDS）
python3 scripts/map_dispatch/waypoint_dispatch_bridge.py
# 澄丰（CycloneDDS）
python3 scripts/map_dispatch/waypoint_dispatch_bridge.py --ssh-host nx --rmw rmw_cyclonedds_cpp
```

另一个 FastDDS 特有的坑：`/nav_status` 上同时存在 `m_common/NavStatus` 与
`std_msgs/String` 两种类型的发布者，而**同一进程里同名话题只能注册一种类型**——
再建第二个订阅必失败（rcl 报 `invalid allocator`，并留下错误状态连累后续建实体）。
`remote_nav_cli.read_nav_status()` 原先是"两种都挂"，兴飞切 FastDDS 后 status/check
会整个挂掉；2026-10-16 改成**优先 typed、建不起来才退回 String，绝不同时挂**。

## 文件

| 文件 | 跑在哪 | 作用 |
| --- | --- | --- |
| `ros_map_tool_v3_nav2_live.html` | 浏览器 | 打过补丁的页面：航点勾选列表 + 下发面板（暂停/继续/取消/状态） |
| `waypoint_dispatch_bridge.py` | 地面站本机 | HTTP 桥（默认 `127.0.0.1:8770`），做坐标换算 + ssh 转发 |
| `remote_nav_cli.py` | 船端（一次性，经 stdin） | rclpy 客户端：`check/status/dispatch/cancel/pause`，输出 `@@RESULT@@ {json}`；`check` 里含一次**只读服务往返探测**（`get_geofence`），只有它 OK 才说明「能下发」 |
| `deploy_to_map_tool.sh` | 地面站本机 | 部署页面与脚本到 `/home/ght/ros_map_tool-main`（自动备份旧页面） |
| `deploy_to_nx.sh` | 地面站本机 | 部署到**澄丰 NX**（`nx:~/ros_map_tool/`）：页面 + 船端启动脚本 + 船端客户端 + 船上底图 |
| `boat_ros_map_tool.sh` | 船端（部署成 `~/ros_map_tool/ros_map_tool.sh`） | 船端一键起 `rosbridge(9090)` + 页面 HTTP(8001)，`start/stop/status/url` |

除下发面板外，页面还做了一处小改动：原来 `three.js` 是**静态 import**（CDN 不通时整个
模块脚本不执行、所有按钮失灵），现在改成只在点「⚡ 生成地图」时**动态 import**。
没有外网的现场照样能加载 `yaml + pgm`、规划航点、下发到船端。

## 用法

### 船端先起什么（兴飞）

```bash
ssh hkzl '~/start_xingfei_stack.sh start'    # 12 项，全部取自 ~/usv-main/install/arm
ssh hkzl '~/start_xingfei_stack.sh status'   # 逐项 UP/DOWN
```

这一套里已经包含**地图工具需要的全部**：`nav2`（`workspace_nav nav2_boat.launch.py`）、
`mission_bridge`（提供 `/mission_bridge/send_waypoints|set_pause|cancel_mission` 与
typed `/nav_status`）、`rosbridge`，以及定位/mavros/gi320/rs485/速度桥/归港/TF/感知。
2026-10-16 起 **livox（Mid360）不随栈自启**，要起得显式：
`WITH_LIVOX=1 ~/start_xingfei_stack.sh start`。

另一套是 usv-test 的「整船 launch + 补起 5 项」，**不要和上面混用**：

```bash
ssh hkzl '~/usv-test/start_cleaning_boat_xinfei.sh'          # 整船（含 usv_app 那一大堆）
ssh hkzl '~/usv-test/start_nav_dock_stack.sh start'          # 补 定位/导航/速度桥/归港/坞任务
ssh hkzl 'WITH_DOCK=0 ~/usv-test/start_nav_dock_stack.sh start'  # 只补 定位/导航/速度桥
```

（usv-test 那套要求整船栈先在跑；只跑补起脚本会让定位/导航空转。它的 `stop` 是
`~/usv-test/start_nav_dock_stack.sh stop`，整船停用 `~/usv-test/stop_cleaning_boat_unified.sh`。）

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

## 船端部署（澄丰 nx：页面放船上、连本船 rosbridge）

兴飞那套是「地面站起桥 + ssh 下发」，因为兴飞 NX 没装 `rosbridge_server`。**澄丰 NX 自带
rosbridge**（`webpath.sh` 那一套，`:9090`），所以澄丰的做法是把页面直接放到船上，
局域网里开 `http://<NX_IP>:8001/` 就能看本船 Nav2 的实时数据：

```bash
# 地面站：部署（页面 + 船端启动脚本 + 客户端 + 船上底图）
bash scripts/map_dispatch/deploy_to_nx.sh

# 船端：起服务（rosbridge 已在跑就复用，不会重复起）
ssh nx '~/ros_map_tool/ros_map_tool.sh start'

# 打开：局域网 http://<NX_IP>:8001/；跨网段用隧道（两个端口都要转）
ssh -N -L 8001:127.0.0.1:8001 -L 9090:127.0.0.1:9090 nx   # → http://127.0.0.1:8001/
```

页面从 HTTP 打开时会**自动把 rosbridge 指向「页面所在主机」的 9090**（用 `file://`
双击打开时仍是 `127.0.0.1`），所以隧道/局域网都不用再手改地址。

船端目录：

```
~/ros_map_tool/
  ros_map_tool_v3_nav2_live.html   页面本体
  ros_map_tool.sh                  start/stop/status/url
  remote_nav_cli.py                船端 rclpy 客户端（下发面板用）
  map/map.yaml, map/map.pgm        船上当前底图（页面「加载」时选这两个）
  logs/                            rosbridge.log / http.log
```

注意（2026-10-16）：澄丰船上跑的是 `workspace_nav nav2_boat.launch.py` + `rviz_waypoint_patrol.py`
那条链路，**没有 `mission_bridge`**（`/mission_bridge/send_waypoints` 等服务不存在），
所以页面的「📤 下发到船端」面板在澄丰上暂时用不了 —— 看实时数据/编辑地图/导出
`yaml+pgm` 都正常，下发要走船上的 `/waypoint` 话题（`~/usv_nav_ws/scripts/send_waypoint.py`）
或 RViz 巡逻脚本。

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
* 船端：栈已起（见上面「船端先起什么」），`mission_bridge` 在跑、`/nav_status` 有数据，
  且状态为 `IDLE`；`WAITING_SYSTEM` 时 `send_waypoints` 会返回失败。
  用 `--check` 的 `service_roundtrip` 确认「服务真能往返」，别只看服务列表。
* 船端 `map.yaml` 与页面加载的底图必须是同一张（2026-10-16 核对）：
  * **兴飞（usv-main 栈）**：`~/usv-main/install/arm/workspace_nav/share/workspace_nav/config/map.yaml`
    （`image: ../map/map.pgm`，resolution 0.2，`ref_gnss_10=[120.36793468, 31.48618175]`）；
    本机对应底图 `ros_map_tool-main/HK (1)/map.pgm`（与 `USV_NAV/map.pgm` 同源），
    md5 `368d598e69…`。
  * **澄丰（nx）**：`ros_map_tool-main/boat_nx_map/{map.yaml,map.pgm}`，md5 `ec660bb3…`。
  * 页面里 `yaml` 和 `pgm` **必须成对加载**，混用两张图会算错经纬度（桥侧包围盒也会拦）。

## 排错

| 现象 | 原因 / 处理 |
| --- | --- |
| 页面报「连不上桥接服务」 | 桥没起（`python3 waypoint_dispatch_bridge.py`）；或改用 `http://127.0.0.1:8770/` 打开页面避开 CORS/私网限制 |
| `❗ datum 未设置` | 页面加载的 yaml 里没有 `ref_gnss_10`（例如 PCD 生成的新图）→ 点「📡 从船端读 datum」直接取船端 map.yaml，或手工填 datum 经纬度 |
| 「航点超出地图锚点范围」 | 底图/原点/datum 与船端不一致，先核对两边 `map.yaml` |
| 船端返回 `System not ready (WAITING_SYSTEM)` | TF 或 Nav2 action 未就绪（定位/导航栈没起全），先看 `--check` 输出 |
| 下发报「服务 /mission_bridge/send_waypoints 超时（8s）无响应」 | 九成是**中间件不一致**（见上面「中间件(RMW)」小节）：船端 FastDDS、客户端 CycloneDDS。判据看 `--check` 的 `service_roundtrip`，服务列表全绿不算数 |
| `--check`/`--status` 报 `RCLError: Failed to create subscription: invalid allocator` | 同一进程里给同名话题挂了两种类型的订阅（`/nav_status` 同时有 `m_common/NavStatus` 与 `std_msgs/String`）。FastDDS 下第二个必失败；2026-10-16 已改成只挂 typed |
| 定位 `DEGRADED/LOST` | 船端没有 GPS fix（室内/无 RTK）时会出现；导航起来后会飘，先解决定位 |
| 检查船端要 7~10 s | 正常：一次 ssh + source 船端工作区 + rclpy 起进程，外加一次真服务往返 |
