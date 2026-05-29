# 导航任务事件（对应 /task_event JSON）
# 事件驱动，QoS: RELIABLE, depth=50

std_msgs/Header header             # stamp
string vehicle_id
string task_id
string command_id
string event                       # TASK_STARTED|TASK_COMPLETED|TASK_FAILED|TASK_CANCELLED|TASK_PAUSED|TASK_RESUMED|EMERGENCY_STOP|ALARM_RAISED|ALARM_CLEARED

# --- detail 载荷（按 event 类型选填）---
int32 total_waypoints              # TASK_STARTED
float32 elapsed_sec                # TASK_COMPLETED
string error_code                  # TASK_FAILED / EMERGENCY_STOP
int32 failed_waypoint_index        # TASK_FAILED
string reason                      # TASK_FAILED
int32 nav2_error_code              # TASK_FAILED（Nav2 FollowWaypoints 原生错误码）
string nav2_error_msg              # TASK_FAILED（Nav2 原生错误信息）
string source                      # TASK_CANCELLED
int32 waypoint_index               # TASK_PAUSED / TASK_RESUMED
string alarm_code                  # ALARM_RAISED / ALARM_CLEARED
string alarm_level                 # ALARM_RAISED: WARN|ERROR
string alarm_message               # ALARM_RAISED
string alarm_suggested_action      # ALARM_RAISED
