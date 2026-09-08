# 任务进度周期上报,对应 §4.2.2.4 / state topic mission_state 由 decision 发布,mqtt_bridgev2 订阅后转 MQTT events
builtin_interfaces/Time stamp

# 任务归属者。mqtt_bridgev2 下发任务时填 productKey,decision 须原样回传;为空或含 / + # 时桥会直接丢弃该条 mission_state,不上报云端。
string user_id
string execution_id
string phase

# 当前正在前往的目标航点。仅决策层内部使用 —— 上云的 current_seq 语义是「最新已完成航点」(V0.7 §4.1.2.5),由 mqtt_bridgev2 取 reached_seq 填充。
int32 current_seq

# 最新已完成的实际航点序号。航点从 1 开始连续编号;一个都没走完时为 0。
int32 reached_seq
int32 waypoint_total
float32 progress

# 飞控/任务执行器任务状态和模式；取值口径与 MAV_MISSION_STATE/MISSION_CURRENT 对齐。
int32 mission_state
int32 mission_mode

# 失败原因；非 FAILED 状态为空。
string failure_message

# 特殊情况码（docs/特殊情况码.md §3）。0 = 正常/无特殊情况。决策层产的码见下方常量；导航失败类(2006/2007/2009/21xx 等)由导航栈给出后原样透传。CANCELLED 时必须带码区分原因，否则云端只能显示"已取消"而说不出为什么。
int32 task_code

# 每行末尾标注 docs/特殊情况码.md §3.1 规定的 phase,便于产码处直接对齐。
int32 TASK_SUCCESS = 0                    # 任务完成 / 无特殊情况            -> COMPLETED/RUNNING
int32 GEOFENCE_EXCLUSION_ENTERED = 2001   # 执行中驶入禁航区                 -> PAUSED
int32 NAVIGATION_NOT_READY = 2004         # 导航未就绪(action 建不起来)      -> FAILED
int32 MISSION_CANCELED = 2005             # 人员显式取消                     -> CANCELLED
int32 PLANNING_FAILED = 2006              # 规划重试耗尽后仍失败             -> FAILED
int32 CONTROLLER_OR_STUCK = 2007          # 导航控制失败/船体受困            -> FAILED
int32 LOCALIZATION_LOST = 2009            # 定位丢失                         -> PAUSED
int32 EMERGENCY_STOPPED = 2010            # 急停/安全硬停                    -> FAILED
int32 MISSION_TIMEOUT = 2011              # 任务层计时超时                   -> FAILED
int32 MISSION_SUPERSEDED = 2012           # 被新任务替代(不含虚拟遥控)       -> FAILED
int32 MISSION_RC_OVERRIDE = 2013          # 物理遥控器接管                   -> FAILED
int32 LOW_BATTERY_RETURN = 2014           # 因低电自动返航而结束             -> FAILED
int32 ULTRA_LOW_BATTERY_DRIFT = 2015      # 因超低电漂流而失败               -> FAILED
int32 MISSION_MANUAL_INTERRUPTED = 2016   # 人为暂停,任务仍保留              -> PAUSED
int32 MISSION_RESUMED_FROM_WAYPOINT = 2017  # 从指定航点恢复                 -> RUNNING

# 2018~2027 阻断码:由传感器健康监控产生(§3.2)。是否真的阻断任务见 §4 的策略注释。"一般不阻断"的那四个(2021/2022/2024/2025)仍然要上报,只是 phase 保持 RUNNING、任务照跑;故障恢复后 task_code 清回 0。
int32 LIDAR_BLOCKOUT = 2018               # 激光雷达故障阻断                 -> PAUSED
int32 RTK_BLOCKOUT = 2019                 # RTK 故障阻断                     -> PAUSED
int32 IMU_BLOCKOUT = 2020                 # IMU 故障阻断                     -> PAUSED
int32 FRONT_CAMERA_BLOCKOUT = 2021        # 前视相机故障(一般不阻断)         -> RUNNING
int32 REAR_CAMERA_BLOCKOUT = 2022         # 后视相机故障(一般不阻断)         -> RUNNING
int32 BOAT_CONTROLLER_BLOCKOUT = 2023     # 飞控故障阻断                     -> PAUSED
int32 WATER_SENSOR_BLOCKOUT = 2024        # 水质传感器故障(一般不阻断)       -> RUNNING
int32 GIMBAL_BLOCKOUT = 2025              # 云台相机故障(一般不阻断)         -> RUNNING
int32 BATTERY_BLOCKOUT = 2026             # 电池/BMS 故障阻断                -> FAILED
int32 WIRELESS_CHARGING_BLOCKOUT = 2027   # 无线充电故障阻断                 -> FAILED

# 2001/2002/2009 是「阻断但可恢复」:任务保留断点停在 PAUSED,不是终态。注意此时导航 Action 已经结束,恢复走的是从 current_seq+1 重新派发,不是 SetPause(false)。
int32 GEOFENCE_INCLUSION_EXITED = 2002    # 执行中驶出作业区                 -> PAUSED


# 2031:虚拟遥控抢占且抢占前未收到暂停时,原任务的终态码。
int32 VIRTUAL_MANUAL_OVERRIDE = 2031      # 虚拟遥控接管且未收到暂停导致原任务结束       -> FAILED                  
int32 MISSION_INTERNAL_ERROR = 2099       # 未匹配到具体信号的兜底           -> FAILED
int32 DOCK_NOT_FOUND = 2101               # 未识别到船坞/位姿不可用          -> FAILED
int32 DOCK_APPROACH_FAILED = 2102         # 入口校验/对准/入泊控制失败       -> FAILED
int32 DOCK_LOCK_FAILED = 2103             # 船坞锁定失败                     -> FAILED
int32 DOCK_STAGING_NAV_FAILED = 2104      # 去预泊点导航重试耗尽             -> FAILED
int32 DOCK_MANUAL_TAKEOVER_REQUIRED = 2105  # 归港重试耗尽,需人工接管        -> FAILED
int32 UNDOCK_FAILED = 2106                # 出泊失败                         -> FAILED
int32 DOCK_CHARGE_CONFIRM_TIMEOUT = 2107  # 入泊后充电确认超时               -> FAILED

# Unix 毫秒。进行中 end_time=0，MQTT 转换成 JSON null。
int64 start_time
int64 end_time

# 船体当前位置;当下没有有效定位时,填本次任务最后一次有效位置(WGS84)。V0.7 明确禁止用 0,0 表示"位置无效" —— has_position=false 才是无效的唯一表达, 桥据此把 lat/lon 上报成 JSON null 而不是 0。
bool has_position
float64 lat
float64 lon
