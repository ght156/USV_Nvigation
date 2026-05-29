# 导航状态快照（对应 /nav_status JSON）
# 发布频率 2Hz，QoS: RELIABLE + TRANSIENT_LOCAL

std_msgs/Header header             # stamp + frame_id
string vehicle_id                  # 车辆 ID

# --- 任务状态 ---
string task_state                  # WAITING_SYSTEM|IDLE|RUNNING|PAUSED|COMPLETED|FAILED|EMERGENCY
string task_id                     # 任务 ID，无任务时为空
string command_id                  # 指令 ID
string nav_phase                   # IDLE|TRACKING|STUCK|RECOVERY|PAUSED|EMERGENCY
int32 current_waypoint             # 已完成航点数
int32 total_waypoints              # 总航点数
float32 progress_percent           # 0.0~100.0
float32 elapsed_sec                # 已运行秒数
float32 distance_to_goal_m         # 到目标距离
float32 eta_sec                    # 预计剩余时间（-1 表示未知）
string last_error                  # 错误码，无错误时为空

# --- 规划器状态 ---
string planner_status              # OK|FAILED
float32 planner_last_plan_time_ms
string planner_last_error

# --- 控制器状态 ---
string controller_status           # OK|STUCK
float32 controller_tracking_error_m
string controller_last_error

# --- 定位状态 ---
string localization_overall        # GOOD|DEGRADED|LOST
float32 localization_position_cov_max   # 位置协方差最大值 (m²)
float32 localization_orientation_cov_max
int32 localization_gps_fix               # 0=无 3=3D fix
bool localization_tf_ok                  # map→base_link TF 可用
float32 localization_odom_hz             # 里程计实际频率

# --- 位姿 (map 坐标系) ---
float64 pose_x
float64 pose_y
float64 pose_yaw                  # rad
float64 pose_v                    # m/s
float64 pose_w                    # rad/s

# --- 标志位 ---
bool flags_manual_override
bool flags_emergency_stop
bool flags_recovery_active

# --- 告警布尔 ---
bool alerts_odom_stale
bool alerts_gps_stale
bool alerts_mission_bridge_alive
bool alerts_planner_error
bool alerts_controller_error
bool alerts_emergency_active
bool alerts_mission_paused
