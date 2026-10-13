#!/usr/bin/env python3
"""docking_fsm — 完整归港状态机。

状态流：
    IDLE -> [WAIT_DOCK_OPEN*] -> ACQUIRE_TAG -> APPROACH_ENTRY -> ALIGN_ENTRY
        -> BACK_IN -> FINAL_DOCK -> DOCKED
    异常：REACQUIRE_TAG（入口外丢 Tag）/ ABORT_EXIT（坞内失败驶出）/ FAILED
    出泊：[WAIT_DOCK_RELEASE*] -> UNDOCK_EXIT -> UNDOCK_SETTLE（/dock/undock 触发）
    终局（2026-10-05 压紧段设计）：浮动坞位置会漂移，FINAL_DOCK 到达原"死位置"
      （final_target_x±docked_x_tol）不再停车判成功，控制器低速继续倒船（压紧段），
      成功判据为 WaitIO 夹爪抓住；压紧超时（final_push_time_sec）-> ABORT_EXIT 重试。
      （2026-10-08 删除充电信号判据：rs485 驱动改版后 /wireless_charger_state
      只在夹爪抓住后才发布，充电判据不可能先于抓住触发，判定由 WaitIO 单一口径）
    * 夹爪交互，仅 dock_claw_enabled=true 时启用（默认 true＝兴飞船；
      船坞无夹爪时置 false）。**2026-10-08 起归港/出港方向独立**
      （兴飞：归港不需要夹爪动作，出港必须主动抬爪）：
      - 归港 WAIT_DOCK_OPEN（ControlDO 打开夹爪）由 dock_claw_open_on_start
        控制，兴飞 false —— 船体靠机械结构自己把爪子撞开，直接进 ACQUIRE_TAG；
        坞爪需电机主动打开才能进船的船坞置 true。
      - BACK_IN/FINAL_DOCK 期间发 WaitIO action 等待夹爪抓住，抓住即 DOCKED；
        倒船位移长时间无进展（backin_stuck_timeout_sec）-> ABORT_EXIT 驶出由
        dock_mission 重试。
        **2026-10-09 夹爪半抓住保护**（同事现场确认：爪子扣住船体时驶出会
        拉拽船坞，危险）：WaitIO feedback 的 digital_input（bit0/bit1 = 两个
        夹爪）任一置位即记 _claw_partial_mask，此后坞内一切驶出路径
        （停滞/压紧/走廊/丢 Tag/安全请求/全局超时）一律改为就地停车 FAILED
        （needs_manual_takeover，不重试）；WaitIO 继续重发监视——纯轮询无
        危险，第二个夹爪抓住仍判 DOCKED。
      - 出泊 WAIT_DOCK_RELEASE（ControlDO 抬爪）由 dock_claw_release_on_undock
        控制，兴飞 true，成功才 UNDOCK_EXIT，失败 FAILED（爪子可能仍扣着，
        禁止盲动）。
      Action 接口为嵌软的 usv_rs485_driver/action/{ControlDO,WaitIO}
      （懒加载，接口缺失时报错降级）。
    * 船坞背光板（DO bit1，Find Tag 必需，backlight_enabled 默认 true）：
      /dock/start 经 SetDO service 点亮 bit1，**整个会话保持**并按
      backlight_keepalive_sec 周期保活；**失败/重试不关灯**（重试期间相机还要
      看 Tag），只有 DOCKED（归港成功）/ /dock/cancel / /dock/undock 才关闭。
      WAIT_DOCK_OPEN/RELEASE 与 BACK_IN/FINAL_DOCK 不干预——那几段 bit1 由
      驱动自己驱动（ControlDO 写 0x0001 / WaitIO 开关），FSM 再写会互抢。
      背景：驱动 ControlDO 成功后 60s 会 writeDO(0x0000) 全清 DO，且这版驱动
      kDOOnMask=0x0001 不含背光位，详见
      docs/bug_debug/2026-10-08_RS485驱动ControlDO漏点背光板与60s全清回归.md。

输入：
  /docking/dock_pose        (PoseStamped, base_link 在 dock_est 系位姿)
  /docking/tag_visible      (Bool)
  /docking/pose_source      (String: VISION/ODOM_PREDICTION/INVALID)
  /docking/measurement_age  (Float32)
  /docking/abort_request    (String, 来自 safety，空串=无请求)
  /dock/start                  (Bool, dock_mission 契约)
  /dock/cancel                 (Empty, dock_mission 契约)
  /dock/undock                 (Bool, dock_mission 契约)

输出：
  /docking/state            (String)
  /docking/target_mode      (String, 控制器阶段目标，见 MODE_*)
  /docking/reset_anchor     (Bool, 可选在进入 ACQUIRE_TAG 时重置锚点)
  /dock/status                 (String JSON, dock_mission 兼容契约：
                                 success/needs_reapproach/abort_reason/
                                 undock_success/state；FAILED 映射为
                                 state="DOCK_ABORT" 且 needs_reapproach=true，
                                 原始状态见 v2_state)

关键约定：
  - 误差在 dock_est 系表达：x<0 在坞外、x≈0 在 bay 中心；对准 = yaw≈±pi
    （e_yaw = wrap(yaw - pi)，船尾朝坞）；e_y = y（横向偏差）。
  - 入口外状态（ACQUIRE/APPROACH/ALIGN/REACQUIRE）失败 -> IDLE + needs_reapproach；
    坞内状态（BACK_IN/FINAL_DOCK）失败 -> ABORT_EXIT 驶出后再上报。
  - 内部不自动重试：ABORT_EXIT 完成回 IDLE，重试由 dock_mission 决定。
  - ABORT_EXIT / UNDOCK 完成判据：位姿有效时用 dock 系 x；位姿 INVALID 时
    控制器只能停车等待（不存在"odom 距离跟踪驶出"），超时兜底并诚实上报
    "驶出未确认"——ABORT_EXIT：回 IDLE 且 abort_reason=ABORT_EXIT_UNCONFIRMED、
    needs_reapproach=False、needs_manual_takeover=True（防上层在船仍卡坞内时
    盲重试）；UNDOCK：FAILED（UNDOCK_TIMEOUT）。
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, String

from usv_docking.docking_common import (
    DockState,
    MODE_ALIGN,
    MODE_APPROACH,
    MODE_BACK_IN,
    MODE_DOCKED_HOLD,
    MODE_EXIT_FORWARD,
    MODE_FINAL_DOCK,
    MODE_HOLD,
    MODE_SEARCH,
    MODE_SEARCH_LIMITED,
    MODE_UNDOCK_FORWARD,
    SRC_INVALID,
    SRC_PREDICTION,
    SRC_VISION,
    wrap_angle,
    yaw_from_quat,
)


# 坞内状态（失败须 ABORT_EXIT 驶出）与入口外状态（失败直接回 IDLE 上报）
CORRIDOR_STATES = (DockState.BACK_IN, DockState.FINAL_DOCK)
OUTSIDE_STATES = (
    DockState.ACQUIRE_TAG,
    DockState.APPROACH_ENTRY,
    DockState.ALIGN_ENTRY,
    DockState.REACQUIRE_TAG,
    DockState.WAIT_DOCK_OPEN,  # 船尚未动，按入口外处理
)

# 背光板（DO bit1）由 FSM 自己管（2026-10-08）：归港不再走 ControlDO
# （兴飞船坞的爪子靠船体机械撞开），驱动没有机会顺带点亮背光。
# 策略：/dock/start 点亮后**整个会话保持**（失败/重试不关，重试期间 Tag 还要
# 看），只有归港成功（DOCKED）、/dock/cancel、/dock/undock 才关。
# 下面这些状态里 bit1 由**驱动**自己驱动，FSM 不插手，否则两边互抢：
#   WAIT_DOCK_OPEN/WAIT_DOCK_RELEASE —— ControlDO 写 0x0001 会清 bit1；
#   BACK_IN/FINAL_DOCK —— WaitIO 进坞点亮、抓住后熄灭。
BACKLIGHT_DRIVER_STATES = (
    DockState.WAIT_DOCK_OPEN,
    DockState.WAIT_DOCK_RELEASE,
    DockState.BACK_IN,
    DockState.FINAL_DOCK,
)

# DockState / MODE_* / SRC_* / wrap_angle / yaw_from_quat 统一在
# usv_docking.docking_common 定义（此处 import 复用，取值不变）。


class DockingFsm(Node):
    """归港状态机（状态切换 + 契约输出，不发 cmd_vel）。"""

    def __init__(self):
        super().__init__("docking_fsm")

        # ── 输入话题 ──
        self.declare_parameter("dock_pose_topic", "/docking/dock_pose")
        self.declare_parameter("tag_visible_topic", "/docking/tag_visible")
        self.declare_parameter("pose_source_topic", "/docking/pose_source")
        self.declare_parameter("measurement_age_topic", "/docking/measurement_age")
        self.declare_parameter("abort_request_topic", "/docking/abort_request")
        self.declare_parameter("start_topic", "/dock/start")
        self.declare_parameter("cancel_topic", "/dock/cancel")
        self.declare_parameter("undock_topic", "/dock/undock")

        # ── 输出话题 ──
        self.declare_parameter("state_topic", "/docking/state")
        self.declare_parameter("target_mode_topic", "/docking/target_mode")
        self.declare_parameter("reset_anchor_topic", "/docking/reset_anchor")
        self.declare_parameter("status_topic", "/dock/status")

        # ── 运行 ──
        self.declare_parameter("state_rate", 10.0)
        self.declare_parameter("reset_anchor_on_start", False)

        # ── ACQUIRE_TAG ──
        self.declare_parameter("tag_acquire_frames", 5)
        self.declare_parameter("acquire_timeout_sec", 60.0)
        # 集帧闪烁容忍：视野边缘逐帧丢检是常态，零容忍会反复清零
        # 集帧计数并伴随 HOLD<->SEARCH 角速度忽高忽低（2026-07-29 实测捕获 6 分钟）
        self.declare_parameter("acquire_miss_tolerance", 3)

        # ── APPROACH_ENTRY：预备点与到点容差（dock 系）──
        # 下列默认值与 config/docking.yaml 的实船标定值逐项对齐（2026-10-07），
        # 由 test/test_param_defaults.py 强制——不带参数文件启动时行为才一致
        self.declare_parameter("staging_x", -2.0)
        self.declare_parameter("approach_x_tol", 0.4)
        # 必须 <= align_y_tol：否则 (align_y_abort, approach_y_tol) 成死区致
        # APPROACH<->ALIGN 慢性循环
        self.declare_parameter("approach_y_tol", 0.15)
        # APPROACH 放行 ALIGN 的艏向门槛：基线只查 x/y，大艏偏（实测 55°）进场后
        # ALIGN 原地旋转把 y 甩离轴线冲线（08-05/08-10 两次复现）
        # 2026-10-07：15→25 让大艏偏尽早交给 ALIGN（ALIGN 有 align_arc_speed
        # 边转边倒，不会像 APPROACH 退化区那样卡死打转）
        self.declare_parameter("approach_yaw_tol_deg", 25.0)
        # 预备点退化区打转逃逸（2026-10-07 现场 bag 实锤）：预备线附近带横偏时
        # 方位角奇异，v=0 原地打转横向修不掉。艏向偏差持续超门限累计此秒数
        # 仍收不进 -> ABORT 回 IDLE 上报 needs_reapproach（dock_mission 重试，
        # 比干等 approach_timeout_sec=120s 快得多）
        self.declare_parameter("approach_stuck_sec", 25.0)
        # 进 ALIGN 前实际速度须接近 0：FSM 切 ALIGN 即发 v=0，实测切换时船仍有
        # ~0.27m/s（命令已降到 0.13），旋转 + 残余线速度放大横向漂移
        self.declare_parameter("approach_stop_speed", 0.15)
        self.declare_parameter("approach_stop_hold_sec", 0.5)
        # 实船局部里程（ArduPilot/MAVROS）；仿真默认 /odometry/filtered，由 params_file 覆盖
        self.declare_parameter("odom_speed_topic", "/mavros/gps_input/local")
        # APPROACH 丢 Tag 宽限：近场视野边缘逐帧闪烁时 0.5s 就弹 REACQUIRE 会
        # 造成 APPROACH<->REACQUIRE 活锁（08-10 实测 21 轮零净进度）；
        # 宽限期内靠 odom 推算继续控制（ALIGN 已有 5s 宽限，此处移动中取 3s）
        self.declare_parameter("approach_tag_loss_grace_sec", 3.0)
        self.declare_parameter("approach_timeout_sec", 120.0)

        # ── ALIGN_ENTRY：进 BACK_IN 门槛 / 退回门槛 ──
        # 2026-10-06 外八坞入口放宽（y 0.15→0.25 / yaw 5→10 / abort 0.35→0.45）；
        # 2026-10-07 yaw 10→7（实测廊道单侧仅 5cm，入廊道艏偏要收小）。与 yaml 对齐
        self.declare_parameter("align_y_tol", 0.15)
        self.declare_parameter("align_yaw_tol_deg", 7.0)
        self.declare_parameter("align_hold_sec", 2.0)
        self.declare_parameter("align_y_abort", 0.45)
        self.declare_parameter("align_timeout_sec", 45.0)
        # 艏向已准但 y 滞留卡死带 (y_tol, y_abort] 的逃逸时限：
        # 超时确定性回 APPROACH 修 y（替代干等噪声/总超时）
        self.declare_parameter("align_y_stuck_sec", 6.0)
        # 允许进 BACK_IN 的纵向窗口（坞外）
        self.declare_parameter("entry_window_min_x", -3.5)
        self.declare_parameter("entry_window_max_x", -1.0)

        # ── BACK_IN ──
        self.declare_parameter("final_target_x", 0.0)  # 最终充电位（bay 中心 P）
        self.declare_parameter("final_dock_entry_dist", 0.8)
        # 进 FINAL_DOCK 的横向门控：|e_y| 未收敛到此值内不进终局——终局段 x 已近
        # 目标、纵向行程余量小，倒船横向律修不动大 y 会干等到 final_dock_timeout。
        # 与 docked_y_tol 关联取值：max(docked_y_tol, 0.15)
        self.declare_parameter("final_dock_entry_y_tol", 0.15)
        self.declare_parameter("back_in_timeout_sec", 90.0)
        self.declare_parameter("back_in_tag_loss_hold_sec", 0.5)
        self.declare_parameter("back_in_tag_loss_search_sec", 2.0)
        # 走廊违规（与控制器门控2一致；连续 violation_cycles 周期 -> ABORT_EXIT）
        self.declare_parameter("back_in_gate2_y", 0.35)
        self.declare_parameter("back_in_gate2_yaw_deg", 15.0)
        self.declare_parameter("violation_cycles", 10)

        # ── FINAL_DOCK ──
        self.declare_parameter("docked_x_tol", 0.15)
        self.declare_parameter("docked_y_tol", 0.10)
        # 以下两参数为压紧段设计（2026-10-05）前的"位置到位判 DOCKED"遗留，
        # FSM 不再使用；保留声明以兼容 scripts/dock_tune_gui.py 在线调参
        self.declare_parameter("docked_yaw_tol_deg", 3.0)
        self.declare_parameter("docked_hold_sec", 1.0)
        self.declare_parameter("final_dock_timeout_sec", 60.0)
        self.declare_parameter("final_dock_tag_loss_timeout_sec", 2.0)

        # ── FINAL_DOCK 压紧段（2026-10-05）──
        # 到死位置后控制器低速继续倒船（压紧段，见控制器 final_push_speed）：
        # 压紧段最长时长，超时未抓住 -> ABORT_EXIT（FINAL_PUSH_TIMEOUT）重试
        # 2026-10-08 晚：6.0 -> 15.0。当晚实测压紧段里 DI1(0x0002) 到 5.5s 才闭合、
        # DI0 更晚，而成功判据 WaitIO 掩码 0x0003 要求 DI0+DI1 同时置位 ->
        # 6s 窗口必然超时，表现为"明明已经进坞了却倒车驶出"（FINAL_PUSH_TIMEOUT）。
        # 待现场验证后定稿（见 docs/bug_debug/2026-10-08_压紧段6s不够导致进坞后倒出.md）。
        self.declare_parameter("final_push_time_sec", 15.0)
        # 2026-10-08：充电信号判据已删（rs485 驱动改版后 /wireless_charger_state
        # 只在夹爪抓住后才发布，充电判据不可能先触发；成功判定统一为 WaitIO 抓住）

        # ── ALIGN_ENTRY 丢 Tag 宽限 ──
        # 视野边缘检测逐帧闪烁（估计器 0.3s 无帧即切推算），零容忍会与 REACQUIRE
        # 高频互弹（2026-07-29 实测 40s 弹 6 次）；宽限期内靠 odom 推算继续对准
        self.declare_parameter("align_tag_loss_grace_sec", 5.0)

        # ── REACQUIRE_TAG ──
        self.declare_parameter("reacquire_frames", 5)
        self.declare_parameter("reacquire_miss_tolerance", 3)
        self.declare_parameter("reacquire_timeout_sec", 30.0)
        # 遗留参数（2026-10-07 虚拟坞位回归后 FSM 不再使用）：原"REACQUIRE
        # 艏偏>此值才自转寻找"逻辑已由推算驾驶（reacquire_drive_max_sec）
        # 取代；保留声明仅为兼容旧 yaml/launch 覆盖
        self.declare_parameter("reacquire_search_yaw_deg", 30.0)
        # 会话级虚拟坞位回归（2026-10-07）：REACQUIRE 且锚点活着（odom 推算中）
        # 时不再原地等/纯转圈，受控对准并接近本次会话锚定的虚拟坞位
        # （MODE_APPROACH，复用现有 APPROACH 控制律的先上轴/限速/迟滞保护；
        # 预备点附近该模式主要是原地转向对准，不平移）。本参数为同一归港会话
        # 内推算驾驶的累计预算（APPROACH↔REACQUIRE 乒乓不重置），用完后退回
        # HOLD 等闪烁恢复，由 reacquire_timeout_sec 兜底。safety 对 REACQUIRE
        # 本就豁免 odom 推算时长检查（SEARCH_STATES），不会被 3s 窗口掐掉
        self.declare_parameter("reacquire_drive_max_sec", 10.0)
        # 虚拟坞位驾驶的距离门（2026-10-07 现场）：远距离视觉的坞位姿本来就会跳
        # （实测远场重定位 dp 0.77~1.21m），拿它当"虚拟坞位"等于朝幻影开。
        # 只有船在坞外这个距离以内（此时锚点新鲜且可信，且丢失 Tag 多半是纠偏
        # 机动造成）才允许 REACQUIRE 期间驾驶；超出则退回 HOLD 等 Tag 恢复
        self.declare_parameter("reacquire_drive_max_dist_m", 5.0)

        # ── ABORT_EXIT ──
        self.declare_parameter("exit_complete_x", -4.0)
        self.declare_parameter("abort_exit_timeout_sec", 45.0)

        # ── UNDOCK ──
        # 出少点：0.45m/s 惯性还要滑 1.5~2m，-4 会冲出再归港触发门
        # （2026-09-12 实测）；与 yaml 对齐
        self.declare_parameter("undock_complete_x", -3.0)
        self.declare_parameter("undock_timeout_sec", 60.0)
        self.declare_parameter("undock_settle_sec", 1.0)

        # ── 全局 ──
        self.declare_parameter("max_docking_duration_sec", 150.0)

        # ── 船坞夹爪交互（usv_rs485_driver Action；默认关闭，船坞装夹爪后开启）──
        # 开启后：/dock/start 先 WAIT_DOCK_OPEN 请求打开夹爪（ControlDO），成功才
        # 开始归港；BACK_IN/FINAL_DOCK 发 WaitIO 等夹爪抓住，抓住即 DOCKED（不再
        # 以位置到位判成功）；倒船位移停滞超时 -> ABORT_EXIT 驶出重试；
        # /dock/undock 先 WAIT_DOCK_RELEASE 请求松开夹爪，成功才驶出。
        # 兴飞船坞带夹爪：默认开启（无夹爪船坞在 yaml 置 false）
        self.declare_parameter("dock_claw_enabled", True)
        # 2026-10-08 方向拆分：兴飞船坞归港不需要夹爪动作（船会通过机械结构
        # 自己把爪子撞开），只有出港必须主动抬爪。原来一个 dock_claw_enabled
        # 同时管两个方向，表达不了这个组合。
        # 归港是否先请求打开夹爪（WAIT_DOCK_OPEN）：兴飞 false；
        # 坞爪需电机主动打开才能进船的船坞（如澄丰）置 true。
        self.declare_parameter("dock_claw_open_on_start", False)
        # 出港是否先请求抬爪（WAIT_DOCK_RELEASE）：兴飞 true（必须主动抬）
        self.declare_parameter("dock_claw_release_on_undock", True)
        self.declare_parameter("control_do_action", "control_do")
        self.declare_parameter("wait_io_action", "wait_io")
        # ControlDO 等结果/等 server 的超时（电机动作约 3s，留足余量）
        self.declare_parameter("dock_open_timeout_sec", 15.0)
        # WaitIO goal 的 max_queries（0 = 用驱动 yaml 默认值 3000）
        self.declare_parameter("wait_io_max_queries", 0)
        # WaitIO 返回失败（非"抓住"）时的重发次数上限，超限 -> ABORT_EXIT
        self.declare_parameter("wait_io_retry_max", 3)
        # 坞内倒船位移停滞检测：x 进展 < backin_stuck_progress_m 持续
        # backin_stuck_timeout_sec -> ABORT_EXIT 驶出重试（顶住坞/夹爪未抓住）
        self.declare_parameter("backin_stuck_timeout_sec", 15.0)
        self.declare_parameter("backin_stuck_progress_m", 0.03)

        # ── 船坞背光板（DO bit1；傍晚/夜间找 Tag 必需，调试手册 §4.5）──
        # 归港会话进入找 Tag 阶段时点亮 + 周期保活（驱动 ControlDO 成功后
        # 60s 会 writeDO(0x0000) 全清，出港紧接归港时会把刚点亮的灯抹掉，
        # 故不能只点一次）；归港成功/失败/回 IDLE 关闭。
        # 无背光船坞或有自己管理方式的船坞置 false。
        self.declare_parameter("backlight_enabled", True)
        self.declare_parameter("backlight_mask", 2)  # DO 掩码：2=只 bit1 背光
        self.declare_parameter("backlight_keepalive_sec", 20.0)
        self.declare_parameter("set_do_service", "set_do")

        p = self.get_parameter
        qos = QoSProfile(depth=10)

        self._state_pub = self.create_publisher(String, p("state_topic").value, qos)
        self._target_mode_pub = self.create_publisher(
            String, p("target_mode_topic").value, qos
        )
        self._reset_anchor_pub = self.create_publisher(
            Bool, p("reset_anchor_topic").value, qos
        )
        self._status_pub = self.create_publisher(String, p("status_topic").value, qos)

        self.create_subscription(
            PoseStamped, p("dock_pose_topic").value, self._dock_pose_cb, qos
        )
        self.create_subscription(
            Bool, p("tag_visible_topic").value, self._tag_visible_cb, qos
        )
        self.create_subscription(
            String, p("pose_source_topic").value, self._pose_source_cb, qos
        )
        self.create_subscription(
            Float32, p("measurement_age_topic").value, self._measurement_age_cb, qos
        )
        # odom 订阅使用默认 QoS（RELIABLE，与 Nav2/系统默认一致）
        _odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry, p("odom_speed_topic").value, self._odom_cb, _odom_qos
        )
        self.create_subscription(
            String, p("abort_request_topic").value, self._abort_request_cb, qos
        )
        self.create_subscription(Bool, p("start_topic").value, self._start_cb, qos)
        self.create_subscription(Empty, p("cancel_topic").value, self._cancel_cb, qos)
        self.create_subscription(Bool, p("undock_topic").value, self._undock_cb, qos)

        # ── 运行状态 ──
        self._state = DockState.IDLE
        self._state_enter_time = None
        self._mission_start_time = None
        self._last_tick_time = None

        # 最新位姿（dock 系）
        self._dock_x = None
        self._dock_y = None
        self._dock_yaw = None
        self._tag_visible = False
        self._pose_source = SRC_INVALID
        self._measurement_age = float("inf")
        self._odom_speed = None

        # 计数器 / 计时器
        self._acquire_frames = 0
        self._acquire_miss = 0
        self._align_hold = 0.0
        self._align_tag_loss = 0.0
        self._align_y_stuck = 0.0
        # 压紧段状态：离开 BACK_IN/FINAL_DOCK（含 /dock/cancel）即复位
        self._final_push_start = None
        self._final_push_elapsed = 0.0
        self._violation_count = 0
        self._reacquire_frames = 0
        self._reacquire_miss = 0
        # REACQUIRE 推算驾驶预算：同一归港会话内累计（APPROACH↔REACQUIRE
        # 乒乓不重置，防每轮重获全额预算累计失控）；会话开始（_reset_flags）
        # 与结束（IDLE/FAILED/DOCKED）时清零，此处只是初始化
        self._reacquire_drive_elapsed = 0.0
        self._staging_hold = 0.0
        self._approach_tag_loss = 0.0
        # 预备点退化区打转计时（大艏偏持续累计），随状态切换复位
        self._approach_stuck = 0.0
        # 预备点退化区打转计时（大艏偏持续累计），随状态切换复位
        self._approach_stuck = 0.0

        # 上报标志（dock_mission 契约）
        self._success = False
        self._undock_success = False
        self._needs_reapproach = False
        self._needs_manual_takeover = False
        self._abort_reason = None

        # 船坞夹爪交互（usv_rs485_driver Action，懒加载；接口缺失时报错降级）
        self._control_do_client = None
        self._wait_io_client = None
        self._ControlDO = None
        self._WaitIO = None
        self._io_import_failed = False
        # 船坞背光板（SetDO service，懒加载）
        self._set_do_client = None
        self._SetDO = None
        self._set_do_import_failed = False
        self._backlight_last_send_time = None
        self._backlight_session_active = False
        # ControlDO 在途 goal 与结果（结果由 tick 消费，(success, message)）
        self._control_do_goal_handle = None
        self._control_do_goal_active = False
        self._control_do_result = None
        # WaitIO 在途 goal 与结果
        self._wait_io_goal_handle = None
        self._wait_io_active = False
        self._wait_io_need_retry = False
        self._wait_io_retries = 0
        self._claw_grabbed = False
        # WaitIO feedback 见过的夹爪 DI 位（bit0/bit1）；非零即"有爪子扣住船体"，
        # 此后禁止一切驶出重试（2026-10-09，见 _abort_corridor）
        self._claw_partial_mask = 0
        # 坞内倒船位移停滞检测
        self._stuck_best_x = None
        self._stuck_elapsed = 0.0

        # 状态 -> 转移 handler 分派表（IDLE/DOCKED/FAILED 无 handler，等待外部触发）
        self._trans_handlers = {
            DockState.ACQUIRE_TAG: self._trans_acquire_tag,
            DockState.APPROACH_ENTRY: self._trans_approach_entry,
            DockState.ALIGN_ENTRY: self._trans_align_entry,
            DockState.BACK_IN: self._trans_back_in,
            DockState.FINAL_DOCK: self._trans_final_dock,
            DockState.WAIT_DOCK_OPEN: self._trans_wait_dock_open,
            DockState.WAIT_DOCK_RELEASE: self._trans_wait_dock_release,
            DockState.REACQUIRE_TAG: self._trans_reacquire_tag,
            DockState.ABORT_EXIT: self._trans_abort_exit,
            DockState.UNDOCK_EXIT: self._trans_undock_exit,
            DockState.UNDOCK_SETTLE: self._trans_undock_settle,
        }

        rate = float(p("state_rate").value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info("docking_fsm 已启动，状态 IDLE，等待 /dock/start")

    # ══════════════ 输入回调 ══════════════
    def _dock_pose_cb(self, msg: PoseStamped):
        self._dock_x = msg.pose.position.x
        self._dock_y = msg.pose.position.y
        self._dock_yaw = yaw_from_quat(
            msg.pose.orientation.z, msg.pose.orientation.w
        )

    def _tag_visible_cb(self, msg: Bool):
        self._tag_visible = bool(msg.data)

    def _pose_source_cb(self, msg: String):
        self._pose_source = msg.data

    def _measurement_age_cb(self, msg: Float32):
        self._measurement_age = float(msg.data)

    def _odom_cb(self, msg: Odometry):
        self._odom_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y
        )

    def _abort_request_cb(self, msg: String):
        if not msg.data or self._state in (DockState.IDLE, DockState.FAILED):
            return
        reason = msg.data
        if reason.startswith("EXIT_POSE_LOST"):
            # safety 对撤离通道位姿 INVALID 的提示性告警：驶出由本状态机的
            # 超时兜底裁决，此处仅记录，不改变状态（避免中止盲驶尝试）
            self.get_logger().warn(
                f"安全提示: {reason}（当前 {self._state}，继续按超时兜底）",
                throttle_duration_sec=5.0,
            )
            return
        self.get_logger().warn(f"安全退出请求: {reason}（当前 {self._state}）")
        if reason.startswith(("ODOM", "TOPIC_TIMEOUT")):
            # odom 真异常 / 上游节点断流：不能盲驶，直接 FAILED 停车
            self._fail(f"SAFETY:{reason}")
        elif self._state in (
            DockState.UNDOCK_EXIT,
            DockState.UNDOCK_SETTLE,
            DockState.WAIT_DOCK_RELEASE,
        ):
            # 出泊中异常：无再撤离必要，直接 FAILED 上报
            self._fail(f"SAFETY:{reason}")
        elif self._state in CORRIDOR_STATES:
            # 经 _abort_corridor 走夹爪半抓住保护（爪子扣住时禁止驶出）
            self._abort_corridor(f"SAFETY:{reason}")
        elif self._state in OUTSIDE_STATES:
            self._abort_and_report(f"SAFETY:{reason}")

    def _start_cb(self, msg: Bool):
        # IDLE / FAILED 均可（重新）开始；DOCKED 须先 /dock/undock 或 /dock/cancel
        if not msg.data or self._state not in (DockState.IDLE, DockState.FAILED):
            return
        self.get_logger().info(f"收到 /dock/start：{self._state} -> ...")
        self._reset_flags()
        self._mission_start_time = self.get_clock().now()
        if self.get_parameter("reset_anchor_on_start").value:
            self._reset_anchor_pub.publish(Bool(data=True))
        # 船坞背光：整会话保持，失败重试不关（见 _backlight_session_begin）
        self._backlight_session_begin("归港开始")
        # 夹爪模式（仅需打开的那类船坞）：先请求船坞打开夹爪，成功才进 ACQUIRE_TAG。
        # 兴飞：爪子靠船体机械撞开，dock_claw_open_on_start=false，直接进 ACQUIRE_TAG，
        # 背光由 _backlight_on_enter() 在找 Tag 阶段点亮。
        self._enter(
            DockState.WAIT_DOCK_OPEN
            if self._claw_open_on_start()
            else DockState.ACQUIRE_TAG
        )

    def _cancel_cb(self, _msg: Empty):
        if self._state == DockState.IDLE:
            return
        self.get_logger().info(f"收到 /dock/cancel：{self._state} -> IDLE")
        self._backlight_session_end("收到 /dock/cancel")
        self._enter(DockState.IDLE)

    def _undock_cb(self, msg: Bool):
        if not msg.data or self._state not in (DockState.IDLE, DockState.DOCKED):
            return
        self.get_logger().info(f"收到 /dock/undock：{self._state} -> ...")
        self._reset_flags()
        self._mission_start_time = self.get_clock().now()
        # 出港不需要背光；且随后的 ControlDO 会清 bit1，先显式收掉会话
        self._backlight_session_end("出港")
        # 夹爪模式：先请求船坞抬爪，成功才驶出（失败则 FAILED 禁止盲动）。
        # 兴飞：出港必须主动抬爪（dock_claw_release_on_undock=true）。
        self._enter(
            DockState.WAIT_DOCK_RELEASE
            if self._claw_release_on_undock()
            else DockState.UNDOCK_EXIT
        )

    # ══════════════ 状态工具 ══════════════
    def _enter(self, state: str):
        self.get_logger().info(f"状态: {self._state} -> {state}")
        self._state = state
        self._state_enter_time = self.get_clock().now()
        self._reset_state_timers()
        self._manage_io_goals_on_enter(state)
        self._update_flags_on_enter(state)
        self._backlight_on_enter(state)

    def _reset_state_timers(self):
        """计时器/计数器组：每次状态进入全量清零。

        历史上漏重置出过 bug（见各注释），新增计时器/计数器必须加在这里。
        """
        self._acquire_frames = 0
        self._acquire_miss = 0
        self._align_hold = 0.0
        self._align_tag_loss = 0.0
        self._align_y_stuck = 0.0
        # 压紧段状态：离开 BACK_IN/FINAL_DOCK（含 /dock/cancel）即复位
        self._final_push_start = None
        self._final_push_elapsed = 0.0
        self._violation_count = 0
        self._reacquire_frames = 0
        self._reacquire_miss = 0
        # 注意：_reacquire_drive_elapsed 不在此复位——它是会话级预算
        #（APPROACH↔REACQUIRE 乒乓时每轮重置会累计失控），
        # 只在会话开始（_reset_flags）/结束（IDLE/FAILED/DOCKED）清零
        # APPROACH 阶段计时器也须复位：REACQUIRE 集帧路由回 APPROACH 后若残留
        # 旧推算计时，遇短暂新闪烁会立即复弹 REACQUIRE（08-10 活锁的弱化版）
        self._staging_hold = 0.0
        self._approach_tag_loss = 0.0
        # 预备点退化区打转计时（大艏偏持续累计），随状态切换复位
        self._approach_stuck = 0.0
        # 坞内倒船位移停滞检测计时随状态切换复位
        self._stuck_best_x = None
        self._stuck_elapsed = 0.0

    def _manage_io_goals_on_enter(self, state: str):
        """Action 生命周期组：夹爪交互 goal 随状态进出创建/取消。

        WaitIO 仅在坞内（BACK_IN/FINAL_DOCK）存活；ControlDO 仅在
        WAIT_DOCK_OPEN/WAIT_DOCK_RELEASE 存活，离开即取消。
        """
        if state not in CORRIDOR_STATES:
            self._cancel_wait_io()
        else:
            self._wait_io_retries = 0
        if state not in (DockState.WAIT_DOCK_OPEN, DockState.WAIT_DOCK_RELEASE):
            self._cancel_control_do()
        else:
            self._control_do_result = None

    def _update_flags_on_enter(self, state: str):
        """标志组：DOCKED 置成功；回 IDLE 清 success/undock_success。"""
        if state == DockState.DOCKED:
            self._success = True
        if state in (DockState.IDLE, DockState.FAILED, DockState.DOCKED):
            # 会话结束：清零 REACQUIRE 推算驾驶的会话级累计预算
            self._reacquire_drive_elapsed = 0.0
        if state == DockState.IDLE:
            # success / undock_success 分别在 DOCKED / UNDOCK_SETTLE 期间已被
            # dock_mission 消费，回 IDLE 必须清掉：残留 true 会让下一次任务进
            # MONITOR_* 时读到在途旧帧"假成功"，随即 /dock/cancel 把刚启动的
            # 任务掐掉（2026-09-01 排查）。
            # needs_reapproach 仍保留（dock_mission 在 IDLE 下读它做重试决策）。
            self._success = False
            self._undock_success = False

    def _reset_flags(self):
        self._success = False
        self._undock_success = False
        self._needs_reapproach = False
        self._needs_manual_takeover = False
        self._abort_reason = None
        self._claw_grabbed = False
        self._claw_partial_mask = 0
        # 会话开始（/dock/start、/dock/undock 均经此）：清零 REACQUIRE
        # 推算驾驶的会话级累计预算
        self._reacquire_drive_elapsed = 0.0

    def _fail(self, reason: str):
        self.get_logger().error(f"FAILED: {reason}")
        self._abort_reason = reason
        self._needs_manual_takeover = True
        self._enter(DockState.FAILED)

    def _abort_and_report(self, reason: str):
        """入口外失败：直接回 IDLE 并上报 needs_reapproach。"""
        self.get_logger().warn(f"任务中止（{self._state}）: {reason}")
        self._abort_reason = reason
        self._needs_reapproach = True
        self._enter(DockState.IDLE)

    def _abort_corridor(self, reason: str):
        """坞内失败：先 ABORT_EXIT 驶出。

        2026-10-09 夹爪半抓住保护：WaitIO feedback 见过任一夹爪 DI 置位
        （_claw_partial_mask 非零）说明爪子已扣住船体，驶出会拉拽船坞——
        禁止驶出，就地停车 FAILED 并不重试（needs_manual_takeover）。"""
        if self._claw_partial_mask:
            self.get_logger().error(
                f"坞内中止（{self._state}）: {reason}；夹爪已抓住"
                f"（DI 掩码 0x{self._claw_partial_mask:04X}），禁止驶出，"
                "就地停车报失败"
            )
            self._fail(reason)
            return
        self.get_logger().warn(f"坞内中止（{self._state}）: {reason} -> ABORT_EXIT")
        self._abort_reason = reason
        self._enter(DockState.ABORT_EXIT)

    def _state_elapsed(self) -> float:
        if self._state_enter_time is None:
            return 0.0
        return (self.get_clock().now() - self._state_enter_time).nanoseconds * 1e-9

    def _mission_elapsed(self) -> float:
        if self._mission_start_time is None:
            return 0.0
        return (self.get_clock().now() - self._mission_start_time).nanoseconds * 1e-9

    def _errors(self):
        """返回 (x, e_y, e_yaw)；位姿不可用返回 (None, None, None)。"""
        if self._dock_x is None:
            return None, None, None
        return (
            self._dock_x,
            self._dock_y,
            wrap_angle(self._dock_yaw - math.pi),
        )

    # ══════════════ 船坞夹爪交互（usv_rs485_driver Action）══════════════
    def _claw_on(self) -> bool:
        """坞爪总开关：坞内是否用 WaitIO 判"夹爪抓住 -> DOCKED"。
        无夹爪船坞置 false（此时归港不再有抓住判据，靠位置/超时兜底）。"""
        return bool(self.get_parameter("dock_claw_enabled").value)

    def _claw_open_on_start(self) -> bool:
        """归港是否先请求打开夹爪（WAIT_DOCK_OPEN -> ControlDO）。
        兴飞 false：船体靠机械结构自己把爪子撞开，归港不需要夹爪动作。"""
        return self._claw_on() and bool(
            self.get_parameter("dock_claw_open_on_start").value
        )

    def _claw_release_on_undock(self) -> bool:
        """出港是否先请求抬爪（WAIT_DOCK_RELEASE -> ControlDO）。
        兴飞 true：坞爪被船体扣住，必须电机主动抬爪才能出去。"""
        return self._claw_on() and bool(
            self.get_parameter("dock_claw_release_on_undock").value
        )

    def _ensure_io_clients(self) -> bool:
        """懒创建 Action 客户端；接口包不可用时报错一次并降级（返回 False）。"""
        if self._control_do_client is not None:
            return True
        if self._io_import_failed:
            return False
        try:
            from rclpy.action import ActionClient
            from usv_rs485_driver.action import ControlDO, WaitIO
        except ImportError as exc:
            self._io_import_failed = True
            self.get_logger().error(
                f"dock_claw_enabled=true 但 usv_rs485_driver.action 不可导入"
                f"（{exc}），夹爪交互不可用"
            )
            return False
        p = self.get_parameter
        self._control_do_client = ActionClient(
            self, ControlDO, p("control_do_action").value
        )
        self._wait_io_client = ActionClient(self, WaitIO, p("wait_io_action").value)
        self._ControlDO = ControlDO
        self._WaitIO = WaitIO
        return True

    def _send_control_do(self):
        """请求船坞打开/松开夹爪（ControlDO start=true）。server 未就绪则本次
        跳过（tick 下周期重试，由 dock_open_timeout_sec 兜底）。"""
        if self._control_do_goal_active:
            return
        if not self._ensure_io_clients():
            self._control_do_result = (False, "DOCK_IO_UNAVAILABLE")
            return
        if not self._control_do_client.server_is_ready():
            self.get_logger().warn(
                "等待 control_do action server 就绪...",
                throttle_duration_sec=5.0,
            )
            return
        goal = self._ControlDO.Goal()
        goal.start = True
        self._control_do_goal_active = True
        future = self._control_do_client.send_goal_async(goal)
        future.add_done_callback(self._on_control_do_goal_response)
        self.get_logger().info("已发送 ControlDO（请求船坞夹爪动作）")

    def _on_control_do_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._control_do_goal_active = False
            self._control_do_result = (False, f"SEND_FAILED:{exc}")
            return
        if not handle.accepted:
            self._control_do_goal_active = False
            # GOAL_REJECTED 多为驱动侧已有在途 goal（电机 3s 窗口内 dock_mission
            # 重试竞态，2026-09-30 实测）：不当失败，tick 下周期重发，
            # 由 dock_open_timeout_sec 兜底；驱动持续不可用也会同样超时失败
            self.get_logger().warn(
                "ControlDO goal 被拒（驱动侧有在途 goal？），下周期重试",
                throttle_duration_sec=2.0,
            )
            return
        self._control_do_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_control_do_result_cb)

    def _on_control_do_result_cb(self, future):
        self._control_do_goal_active = False
        self._control_do_goal_handle = None
        try:
            res = future.result().result
            self._control_do_result = (bool(res.success), res.message)
        except Exception as exc:
            self._control_do_result = (False, f"RESULT_ERROR:{exc}")

    def _cancel_control_do(self):
        if self._control_do_goal_handle is not None:
            try:
                self._control_do_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._control_do_goal_handle = None
        self._control_do_goal_active = False
        self._control_do_result = None

    # ══════════════ 船坞背光板（usv_rs485_driver SetDO service）══════════════
    # 归港不再走 ControlDO（兴飞坞爪机械撞开），驱动没有机会顺带点亮背光；
    # 而驱动 ControlDO 成功后 60s 会 writeDO(0x0000) 全清一次（出港紧接归港
    # 时会抹掉刚点亮的灯），所以找 Tag 阶段必须周期保活，不能只点一次。
    def _backlight_on(self) -> bool:
        return bool(self.get_parameter("backlight_enabled").value)

    def _backlight_mask(self) -> int:
        return int(self.get_parameter("backlight_mask").value) & 0xFFFF

    def _ensure_set_do_client(self) -> bool:
        """懒创建 SetDO 服务客户端；接口不可用时告警一次并降级（不影响归港）。"""
        if self._set_do_client is not None:
            return True
        if self._set_do_import_failed:
            return False
        try:
            from usv_rs485_driver.srv import SetDO
        except ImportError as exc:
            self._set_do_import_failed = True
            self.get_logger().error(
                f"backlight_enabled=true 但 usv_rs485_driver.srv 不可导入"
                f"（{exc}），船坞背光不可用（归港不受影响）"
            )
            return False
        self._set_do_client = self.create_client(
            SetDO, str(self.get_parameter("set_do_service").value)
        )
        self._SetDO = SetDO
        return True

    def _send_set_do(self, mask: int, reason: str):
        """写 DO 全掩码（背光 = bit1）。异步 fire-and-forget：服务未就绪只告警，
        下个周期重试，不阻断归港状态机。

        注意 set_do 是 16 位全掩码写（不是单 bit），写入值即所有 DO 的期望状态。
        """
        if not self._backlight_on():
            return
        if not self._ensure_set_do_client():
            return
        cli = self._set_do_client
        if not cli.service_is_ready():
            self.get_logger().warn(
                "等待 set_do 服务就绪...", throttle_duration_sec=5.0
            )
            return
        req = self._SetDO.Request()
        req.digital_output = int(mask) & 0xFFFF
        future = cli.call_async(req)
        future.add_done_callback(
            lambda fut, m=req.digital_output, r=reason: self._on_set_do_done(
                fut, m, r
            )
        )
        self.get_logger().info(
            f"船坞背光: 写 DO 0x{req.digital_output:04X}（{reason}）"
        )

    def _on_set_do_done(self, future, mask: int, reason: str):
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"set_do 0x{mask:04X}（{reason}）调用失败：{exc}")
            return
        if not res.success:
            self.get_logger().warn(
                f"set_do 0x{mask:04X}（{reason}）驱动返回失败：{res.message}"
            )

    def _backlight_session_begin(self, reason: str):
        """归港会话开始：点亮并保持整个会话。

        **失败/重试不关灯**（2026-10-08 现场要求）：归港失败回 IDLE 后
        dock_mission 会原地重试，重试期间相机还要看 Tag，灯一灭就要重新
        等一个 60s 级超时。
        """
        if not self._backlight_on():
            return
        self._backlight_session_active = True
        self._send_set_do(self._backlight_mask(), reason)
        self._backlight_last_send_time = self.get_clock().now()

    def _backlight_session_end(self, reason: str):
        """会话真正结束（归港成功 / /dock/cancel / /dock/undock）才关灯。"""
        if not self._backlight_on() or not self._backlight_session_active:
            return
        self._backlight_session_active = False
        self._send_set_do(0, reason)
        self._backlight_last_send_time = None

    def _backlight_on_enter(self, state: str):
        """状态进入钩子：只有归港成功（DOCKED）关灯；失败/重试保持点亮。

        坞内/夹爪动作段由驱动自己驱动 bit1（见 BACKLIGHT_DRIVER_STATES），
        FSM 只在保活 tick 里判断，不在这里动手。
        """
        if state == DockState.DOCKED:
            self._backlight_session_end("归港成功")

    def _backlight_tick(self):
        """会话期间周期保活（覆盖驱动 60s 全清与出港 ControlDO 的残留影响）。"""
        if not self._backlight_on() or not self._backlight_session_active:
            return
        if self._state in BACKLIGHT_DRIVER_STATES:
            return
        period = max(
            1.0, float(self.get_parameter("backlight_keepalive_sec").value)
        )
        now = self.get_clock().now()
        if (
            self._backlight_last_send_time is None
            or (now - self._backlight_last_send_time).nanoseconds * 1e-9 >= period
        ):
            self._send_set_do(self._backlight_mask(), "找 Tag 阶段保活")
            self._backlight_last_send_time = now

    def _send_wait_io(self):
        """请求嵌软轮询夹爪/停泊到位检测（WaitIO）。server 未就绪则本次跳过
        （tick 下周期重试）；接口包不可用时降级为不发送（停滞/超时检测兜底）。"""
        if self._wait_io_active or self._claw_grabbed:
            return
        if not self._ensure_io_clients():
            return
        if not self._wait_io_client.server_is_ready():
            self.get_logger().warn(
                "等待 wait_io action server 就绪...",
                throttle_duration_sec=5.0,
            )
            return
        goal = self._WaitIO.Goal()
        goal.max_queries = int(self.get_parameter("wait_io_max_queries").value)
        self._wait_io_active = True
        future = self._wait_io_client.send_goal_async(
            goal, feedback_callback=self._on_wait_io_feedback
        )
        future.add_done_callback(self._on_wait_io_goal_response)
        self.get_logger().info("已发送 WaitIO（等待夹爪抓住/停泊到位检测）")

    def _on_wait_io_feedback(self, feedback_msg):
        """WaitIO feedback：digital_input 位掩码，bit0/bit1 = 两个夹爪抓住检测
        （驱动成功判据是两位同时置位，单个置位只出现在 feedback）。
        见过任一夹爪置位即累计到 _claw_partial_mask：爪子已扣住船体，
        此后禁止一切驶出重试（2026-10-09，见 _abort_corridor）。"""
        if not self._wait_io_active:
            # 迟到帧（取消途中/上一 goal 的尾巴）：丢弃，防止污染新会话的
            # 半抓住记录（2026-10-09 实船见过：cancel+start 后 0.15s 旧 goal
            # 最后一帧 feedback 到达）
            return
        mask = int(feedback_msg.feedback.digital_input) & 0x0003
        new_bits = mask & ~self._claw_partial_mask
        if new_bits:
            self._claw_partial_mask |= new_bits
            self.get_logger().warn(
                f"WaitIO feedback：夹爪 DI 置位 0x{new_bits:04X}"
                f"（累计 0x{self._claw_partial_mask:04X}）——爪子已扣住船体，"
                "此后禁止驶出重试，坞内超时将就地停车报失败"
            )

    def _on_wait_io_goal_response(self, future):
        try:
            handle = future.result()
        except Exception:
            self._wait_io_active = False
            self._wait_io_need_retry = True
            return
        if not handle.accepted:
            self._wait_io_active = False
            self._wait_io_need_retry = True
            return
        self._wait_io_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_wait_io_result_cb)

    def _on_wait_io_result_cb(self, future):
        self._wait_io_active = False
        self._wait_io_goal_handle = None
        try:
            res = future.result().result
        except Exception:
            self._wait_io_need_retry = True
            return
        if res.success:
            # 迟到的结果只在坞内生效（离开坞内状态后不再判抓住）
            if self._state in CORRIDOR_STATES:
                self._claw_grabbed = True
                self.get_logger().info(f"WaitIO：夹爪已抓住（{res.message}）")
        else:
            self.get_logger().warn(f"WaitIO 未检测到停泊到位：{res.message}")
            self._wait_io_need_retry = True

    def _cancel_wait_io(self):
        if self._wait_io_goal_handle is not None:
            try:
                self._wait_io_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._wait_io_goal_handle = None
        self._wait_io_active = False
        self._wait_io_need_retry = False

    def _manage_wait_io(self):
        """坞内 tick 调用：维持 WaitIO 在途，失败结果按 wait_io_retry_max 重发。"""
        if self._wait_io_active or self._claw_grabbed:
            return
        if self._wait_io_need_retry:
            self._wait_io_need_retry = False
            self._wait_io_retries += 1
            retry_max = int(self.get_parameter("wait_io_retry_max").value)
            if self._wait_io_retries > retry_max:
                self._abort_corridor("WAIT_IO_FAILED")
                return
            self.get_logger().warn(
                f"重发 WaitIO（{self._wait_io_retries}/{retry_max}）"
            )
        self._send_wait_io()

    def _claw_stuck_check(self, dt, x) -> bool:
        """坞内倒船位移停滞检测（仅夹爪模式）：x 无进展超
        backin_stuck_timeout_sec 返回 True（顶住坞/夹爪未抓住，须驶出重试）。
        倒船方向 x 递增（坞外负值 -> 坞中心 0），进展 = x 超过历史最好值。"""
        if x is None or self._pose_source == SRC_INVALID:
            return False
        progress_m = float(self.get_parameter("backin_stuck_progress_m").value)
        if self._stuck_best_x is None or x > self._stuck_best_x + progress_m:
            self._stuck_best_x = x
            self._stuck_elapsed = 0.0
            return False
        self._stuck_elapsed += dt
        return self._stuck_elapsed > float(
            self.get_parameter("backin_stuck_timeout_sec").value
        )

    # ══════════════ 主循环 ══════════════
    def _tick(self):
        now = self.get_clock().now()
        dt = 0.0
        if self._last_tick_time is not None:
            dt = (now - self._last_tick_time).nanoseconds * 1e-9
        self._last_tick_time = now

        self._transitions(dt)
        self._backlight_tick()
        mode = self._compute_mode()
        self._publish(mode)

    def _global_timeout_hit(self) -> bool:
        limit = float(self.get_parameter("max_docking_duration_sec").value)
        return limit > 0 and self._mission_elapsed() > limit

    def _transitions(self, dt: float):
        s = self._state
        x, e_y, e_yaw = self._errors()

        # 全局总超时：坞内 -> ABORT_EXIT；坞外 -> IDLE 上报
        if s in CORRIDOR_STATES + OUTSIDE_STATES and self._global_timeout_hit():
            if s in CORRIDOR_STATES:
                self._abort_corridor("GLOBAL_TIMEOUT")
            else:
                self._abort_and_report("GLOBAL_TIMEOUT")
            return

        # 状态 -> handler 分派（分派表在 __init__ 构建，handler 见下文章节）；
        # IDLE / DOCKED / FAILED 无 handler：等待外部触发（start/cancel/undock）
        handler = self._trans_handlers.get(s)
        if handler is not None:
            handler(dt, x, e_y, e_yaw)

    def _staging_reached(self, x, e_y, e_yaw, dt: float) -> bool:
        p = self.get_parameter
        conditions_ok = (
            abs(x - float(p("staging_x").value))
            <= float(p("approach_x_tol").value)
            and abs(e_y) <= float(p("approach_y_tol").value)
            and abs(e_yaw)
            <= math.radians(float(p("approach_yaw_tol_deg").value))
            and self._odom_speed is not None
            and self._odom_speed
            <= float(p("approach_stop_speed").value)
        )
        if not conditions_ok:
            self._staging_hold = 0.0
            return False
        self._staging_hold += dt
        return self._staging_hold >= float(p("approach_stop_hold_sec").value)

    def _align_gate_ok(self, x, e_y, e_yaw) -> bool:
        p = self.get_parameter
        return (
            abs(e_y) < float(p("align_y_tol").value)
            and abs(e_yaw) < math.radians(float(p("align_yaw_tol_deg").value))
            and float(p("entry_window_min_x").value)
            <= x
            <= float(p("entry_window_max_x").value)
        )

    def _corridor_violation(self, e_y, e_yaw) -> bool:
        p = self.get_parameter
        return (
            abs(e_y) > float(p("back_in_gate2_y").value)
            or abs(e_yaw) > math.radians(float(p("back_in_gate2_yaw_deg").value))
        )

    # ══════════════ 坞内公共段（BACK_IN/FINAL_DOCK 共用）══════════════
    def _claw_wait_or_docked(self, dt, x, stuck_reason: str) -> bool:
        """夹爪模式坞内公共段：WaitIO 抓住即 DOCKED；维持 WaitIO 在途
        （失败按 wait_io_retry_max 重发）；位移停滞超时 -> ABORT_EXIT 驶出重试。
        返回 True 表示本 tick 已裁决完毕（调用方直接 return）。"""
        if not self._claw_on():
            return False
        if self._claw_grabbed:
            self.get_logger().info("夹爪已抓住（WaitIO）-> DOCKED")
            self._enter(DockState.DOCKED)
            return True
        self._manage_wait_io()
        if self._state not in CORRIDOR_STATES:
            return True  # _manage_wait_io 内可能已 ABORT_EXIT（WAIT_IO_FAILED）
        if self._claw_stuck_check(dt, x):
            self._abort_corridor(stuck_reason)
            return True
        return False

    def _corridor_violation_abort(self, x, e_y, e_yaw) -> bool:
        """走廊违规计数（与控制器门控2一致；连续 violation_cycles 周期 ->
        ABORT_EXIT）。返回 True 表示已中止。"""
        if x is not None and self._corridor_violation(e_y, e_yaw):
            self._violation_count += 1
            if self._violation_count >= int(
                self.get_parameter("violation_cycles").value
            ):
                self._abort_corridor("CORRIDOR_VIOLATION")
                return True
        else:
            self._violation_count = 0
        return False

    # ══════════════ 状态转移 handlers（由 _transitions 分派）══════════════
    def _trans_acquire_tag(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        if self._pose_source == SRC_VISION and self._tag_visible:
            self._acquire_frames += 1
            self._acquire_miss = 0
        else:
            self._acquire_miss += 1
            if self._acquire_miss > int(p("acquire_miss_tolerance").value):
                self._acquire_frames = 0
        if self._acquire_frames >= int(p("tag_acquire_frames").value):
            self._enter(DockState.APPROACH_ENTRY)
        elif self._state_elapsed() > float(p("acquire_timeout_sec").value):
            self._abort_and_report("ACQUIRE_TIMEOUT")

    def _trans_approach_entry(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        reacquire = False
        if self._pose_source == SRC_INVALID:
            reacquire = True
        elif self._pose_source != SRC_VISION:
            # 推算期宽限：视野边缘闪烁属常态，宽限内靠 odom 预测继续逼近；
            # 超宽限才弹 REACQUIRE（08-10 实测零宽限导致 APPROACH<->REACQUIRE
            # 活锁，横向净进度≈0）
            self._approach_tag_loss += dt
            if self._approach_tag_loss > float(
                p("approach_tag_loss_grace_sec").value
            ):
                reacquire = True
        else:
            self._approach_tag_loss = 0.0
        # 预备点退化区打转逃逸（2026-10-07 现场 bag 实锤）：预备线附近带横偏时
        # 方位角奇异 v=0 原地打转，横向修不掉只能干等 120s 超时。|e_yaw| 持续
        # 超放行门槛累计 approach_stuck_sec 收不进 -> ABORT 回 IDLE 上报
        # needs_reapproach（dock_mission 重试，比超时兜底快得多）
        if x is not None and abs(e_yaw) > math.radians(
            float(p("approach_yaw_tol_deg").value)
        ):
            self._approach_stuck += dt
        else:
            self._approach_stuck = 0.0
        if self._approach_stuck > float(p("approach_stuck_sec").value):
            self._abort_and_report("APPROACH_STUCK")
            return
        if reacquire:
            self._enter(DockState.REACQUIRE_TAG)
        elif x is not None and self._staging_reached(x, e_y, e_yaw, dt):
            self._enter(DockState.ALIGN_ENTRY)
        elif self._state_elapsed() > float(p("approach_timeout_sec").value):
            self._abort_and_report("APPROACH_TIMEOUT")

    def _trans_align_entry(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        # 丢 Tag 宽限计时：VISION 清零，推算期累积（INVALID 立即弹）
        if self._pose_source == SRC_VISION:
            self._align_tag_loss = 0.0
        elif self._pose_source != SRC_INVALID:
            self._align_tag_loss += dt
        if self._pose_source == SRC_INVALID:
            self._enter(DockState.REACQUIRE_TAG)
        elif self._pose_source != SRC_VISION:
            # 视野边缘检测逐帧闪烁属常态，宽限期内靠推算继续对准
            if self._align_tag_loss > float(
                p("align_tag_loss_grace_sec").value
            ):
                self._enter(DockState.REACQUIRE_TAG)
        elif x is None:
            pass
        elif abs(e_y) > float(p("align_y_abort").value):
            # 差速船 v=0 修不了横向，退回弧线阶段
            self.get_logger().warn(
                f"ALIGN 中 |e_y|={abs(e_y):.2f} 超限，退回 APPROACH_ENTRY"
            )
            self._enter(DockState.APPROACH_ENTRY)
        elif self._align_gate_ok(x, e_y, e_yaw):
            self._align_hold += dt
            if self._align_hold >= float(p("align_hold_sec").value):
                self._enter(DockState.BACK_IN)
        else:
            self._align_hold = 0.0
            # y 卡死带 (align_y_tol, align_y_abort]：艏向已准但 y 超差，
            # ALIGN v=0 修不了横向只能干等到超时（2026-07-29 实测卡满 45s）。
            # 滞留超时确定性退回 APPROACH 弧线修 y，再到ALIGN复核
            yaw_tol = math.radians(float(p("align_yaw_tol_deg").value))
            if (
                abs(e_yaw) <= yaw_tol
                and abs(e_y) > float(p("align_y_tol").value)
            ):
                self._align_y_stuck += dt
                if self._align_y_stuck > float(
                    p("align_y_stuck_sec").value
                ):
                    self.get_logger().warn(
                        f"ALIGN 艏向已准但 |e_y|={abs(e_y):.2f} 滞留"
                        f" {self._align_y_stuck:.1f}s，回 APPROACH 修 y"
                    )
                    self._enter(DockState.APPROACH_ENTRY)
            else:
                self._align_y_stuck = 0.0
            if self._state_elapsed() > float(p("align_timeout_sec").value):
                self._abort_and_report("ALIGN_TIMEOUT")

    def _trans_back_in(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        # 夹爪模式：WaitIO 抓住即成功；位移停滞超时 -> 驶出重试
        if self._claw_wait_or_docked(dt, x, "BACK_IN_STUCK"):
            return
        # Tag 丢失分级（measurement_age 为 inf 时直接进 ABORT_EXIT）
        if self._pose_source != SRC_VISION:
            age = self._measurement_age
            if age > float(p("back_in_tag_loss_search_sec").value):
                self._abort_corridor("TAG_LOST_IN_BACK_IN")
                return
        # 走廊违规（连续 N 周期）
        if self._corridor_violation_abort(x, e_y, e_yaw):
            return
        # 到 FINAL_DOCK 过渡位：x 进过渡距离且 y 已收敛到终局可修范围
        if x is not None and (
            abs(x - float(p("final_target_x").value))
            < float(p("final_dock_entry_dist").value)
            and abs(e_y) <= float(p("final_dock_entry_y_tol").value)
        ):
            self._enter(DockState.FINAL_DOCK)
            return
        if self._state_elapsed() > float(p("back_in_timeout_sec").value):
            self._abort_corridor("BACK_IN_TIMEOUT")

    def _trans_final_dock(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        # 夹爪模式：WaitIO 抓住即成功（压紧段内外均有效，并行判据）；
        # 顶住坞但夹爪未抓住时位移停滞超时 -> 驶出重试
        if self._claw_wait_or_docked(dt, x, "FINAL_DOCK_STUCK"):
            return
        # 最后阶段不主动搜索：停车等待短时重识别，超时退出
        if self._pose_source != SRC_VISION and (
            self._measurement_age
            > float(p("final_dock_tag_loss_timeout_sec").value)
        ):
            self._abort_corridor("TAG_LOST_IN_FINAL_DOCK")
            return
        if self._corridor_violation_abort(x, e_y, e_yaw):
            return
        # 到达原"死位置" -> 进入压紧段（2026-10-05）：浮动坞位置会漂移，
        # 固定位置到位不再判成功，改由控制器低速继续倒船压向夹爪/充电板；
        # 成功 = WaitIO 夹爪抓住（上文），失败 = 压紧超时 -> ABORT_EXIT 由
        # dock_mission 重试。（2026-10-08 删充电信号判据：rs485 驱动改版后
        # 充电状态只在抓住后才发布，该判据不可能先触发）
        # 段内漂出窗口（浮坞移动/Tag 噪声）不复位计时，避免反复重进
        if x is not None and (
            abs(x - float(p("final_target_x").value))
            < float(p("docked_x_tol").value)
            and abs(e_y) < float(p("docked_y_tol").value)
        ):
            if self._final_push_start is None:
                self._final_push_start = self.get_clock().now()
                self.get_logger().info(
                    "FINAL_DOCK 到达死位置，进入压紧段（低速继续倒船，"
                    "等待夹爪抓住）"
                )
            self._final_push_elapsed += dt
            if self._final_push_elapsed > float(p("final_push_time_sec").value):
                self._abort_corridor("FINAL_PUSH_TIMEOUT")
                return
        if self._state_elapsed() > float(p("final_dock_timeout_sec").value):
            self._abort_corridor("FINAL_DOCK_TIMEOUT")

    def _trans_wait_dock_open(self, dt, x, e_y, e_yaw):
        # 夹爪模式：请求船坞打开夹爪（ControlDO），成功才进 ACQUIRE_TAG；
        # 失败/超时回 IDLE 上报 needs_reapproach（船尚未动，同入口外失败）
        self._trans_wait_dock_io(
            action_desc="打开",
            success_state=DockState.ACQUIRE_TAG,
            fail_prefix="DOCK_OPEN",
            on_failure=self._abort_and_report,
        )

    def _trans_wait_dock_release(self, dt, x, e_y, e_yaw):
        # 夹爪模式出泊：请求船坞松开夹爪，成功才 UNDOCK_EXIT；
        # 失败/超时 FAILED 停车（爪子可能仍扣着，禁止盲动）
        self._trans_wait_dock_io(
            action_desc="松开",
            success_state=DockState.UNDOCK_EXIT,
            fail_prefix="DOCK_RELEASE",
            on_failure=self._fail,
        )

    def _trans_wait_dock_io(self, action_desc, success_state, fail_prefix, on_failure):
        """WAIT_DOCK_OPEN / WAIT_DOCK_RELEASE 公共流程：等 ControlDO 结果，
        成功进 success_state；失败/超时由 on_failure 按各自语义处理
        （OPEN 失败回 IDLE 上报 needs_reapproach；RELEASE 失败 FAILED 停车）。"""
        if self._control_do_result is not None:
            ok, message = self._control_do_result
            if ok:
                self.get_logger().info(f"船坞夹爪已{action_desc}：{message}")
                self._enter(success_state)
            else:
                on_failure(f"{fail_prefix}_FAILED:{message}")
        elif self._state_elapsed() > float(
            self.get_parameter("dock_open_timeout_sec").value
        ):
            on_failure(f"{fail_prefix}_TIMEOUT")
        else:
            self._send_control_do()

    def _trans_reacquire_tag(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        # 推算驾驶预算记时（虚拟坞位回归，2026-10-07）：锚点活着（odom 推算）
        # 期间 _compute_mode 给 MODE_APPROACH 受控对准并接近虚拟坞位；
        # 同一归港会话内累计超 reacquire_drive_max_sec 用完预算，
        # 之后退回 HOLD 等闪烁恢复
        if self._pose_source == SRC_PREDICTION:
            budget = float(p("reacquire_drive_max_sec").value)
            was_within = self._reacquire_drive_elapsed <= budget
            self._reacquire_drive_elapsed += dt
            if was_within and self._reacquire_drive_elapsed > budget:
                self.get_logger().warn(
                    "REACQUIRE 推算驾驶累计超 reacquire_drive_max_sec"
                    f"（{budget:.0f}s）预算，退回 HOLD 等 Tag 恢复"
                )
        if self._pose_source == SRC_VISION and self._tag_visible:
            self._reacquire_frames += 1
            self._reacquire_miss = 0
        else:
            self._reacquire_miss += 1
            if self._reacquire_miss > int(
                p("reacquire_miss_tolerance").value
            ):
                self._reacquire_frames = 0
        if self._reacquire_frames >= int(p("reacquire_frames").value):
            self._route_after_reacquire(x, e_y, e_yaw)
        elif self._state_elapsed() > float(p("reacquire_timeout_sec").value):
            self._abort_and_report("REACQUIRE_TIMEOUT")

    def _trans_abort_exit(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        done = False
        if x is not None and self._pose_source != SRC_INVALID:
            done = x <= float(p("exit_complete_x").value)
        if done:
            self._needs_reapproach = True
            self._enter(DockState.IDLE)
        elif self._state_elapsed() > float(p("abort_exit_timeout_sec").value):
            if self._pose_source == SRC_INVALID:
                # 位姿 INVALID 时控制器输出零速（无"odom 距离跟踪驶出"），
                # 超时退出不得谎报"已驶出"：不置 needs_reapproach，以免
                # dock_mission 在船仍卡坞内时立刻重试；交人工/上层裁决
                self.get_logger().warn(
                    "ABORT_EXIT 超时且位姿 INVALID：驶出未确认，回 IDLE "
                    "上报 ABORT_EXIT_UNCONFIRMED（不置 needs_reapproach）"
                )
                self._abort_reason = "ABORT_EXIT_UNCONFIRMED"
                self._needs_manual_takeover = True
            else:
                self.get_logger().warn(
                    "ABORT_EXIT 超时（位姿有效），回 IDLE 上报 needs_reapproach"
                )
                self._needs_reapproach = True
            self._enter(DockState.IDLE)

    def _trans_undock_exit(self, dt, x, e_y, e_yaw):
        p = self.get_parameter
        done = False
        if x is not None and self._pose_source != SRC_INVALID:
            done = x <= float(p("undock_complete_x").value)
        if done:
            self._undock_success = True
            self._enter(DockState.UNDOCK_SETTLE)
        elif self._state_elapsed() > float(p("undock_timeout_sec").value):
            # 超时不报假成功：船未确认驶出，FAILED（JSON 映射 DOCK_ABORT）
            self._fail("UNDOCK_TIMEOUT")

    def _trans_undock_settle(self, dt, x, e_y, e_yaw):
        if self._state_elapsed() > float(
            self.get_parameter("undock_settle_sec").value
        ):
            self._enter(DockState.IDLE)

    def _route_after_reacquire(self, x, e_y, e_yaw):
        """重捕获后按位置路由：入口窗口内且横向/艏向/速度已达 ALIGN 通过门槛
        -> ALIGN，否则 -> APPROACH。y 必须用 align_y_tol 而非 align_y_abort：
        (align_y_tol, align_y_abort] 区间内 ALIGN 无法通过（原地转修不了横向），
        只能等超时（2026-07-29 实测 y=0.29 卡死带）。
        08-10 无头实测：REACQUIRE 以 e_yaw≈-58° 直接路由进 ALIGN 复现冲线，
        因此此处与 _staging_reached 同一套艏向/停稳门槛。"""
        p = self.get_parameter
        if (
            x is not None
            and float(p("entry_window_min_x").value)
            <= x
            <= float(p("entry_window_max_x").value)
            and abs(e_y) <= float(p("align_y_tol").value)
            and abs(e_yaw)
            <= math.radians(float(p("approach_yaw_tol_deg").value))
            and self._odom_speed is not None
            and self._odom_speed <= float(p("approach_stop_speed").value)
        ):
            self._enter(DockState.ALIGN_ENTRY)
        else:
            self._enter(DockState.APPROACH_ENTRY)

    # ══════════════ target_mode 映射 ══════════════
    def _compute_mode(self) -> str:
        s = self._state
        if s == DockState.ACQUIRE_TAG:
            return MODE_HOLD if self._tag_visible else MODE_SEARCH
        if s == DockState.APPROACH_ENTRY:
            return MODE_APPROACH
        if s == DockState.ALIGN_ENTRY:
            return MODE_ALIGN
        if s == DockState.BACK_IN:
            if self._pose_source != SRC_VISION:
                if self._measurement_age <= float(
                    self.get_parameter("back_in_tag_loss_hold_sec").value
                ):
                    return MODE_HOLD  # 0~0.5s：停车等待
                return MODE_SEARCH_LIMITED  # 0.5~2s：小角度搜索
            return MODE_BACK_IN
        if s == DockState.FINAL_DOCK:
            if self._pose_source != SRC_VISION:
                return MODE_HOLD  # 最后阶段只停车等待
            return MODE_FINAL_DOCK
        if s == DockState.REACQUIRE_TAG:
            # 会话级虚拟坞位回归（2026-10-07）：丢 Tag 但锚点活着（odom 推算中）
            # 时受控对准并接近本次会话锚定的虚拟坞位（MODE_APPROACH，复用
            # APPROACH 控制律的先上轴/限速/迟滞保护；预备点附近主要是原地
            # 转向对准，不平移），不再原地干等；驾驶预算 reacquire_drive_max_sec
            # （同一归港会话内累计）用完后退回 HOLD。VISION 但 tag 闪烁保持
            # HOLD 等其恢复（SEARCH 自转会把刚修好的横向偏差扫漂，08-10 实测
            # 21 轮活锁）。位姿 INVALID（锚点也丢了）才退回原地找：
            # 艏向偏差大（Tag 在船尾后方，纯 HOLD 永远重捕不到）时受控旋转
            # 寻找（MODE_SEARCH 绕推算坞方位闭环转），否则 HOLD。
            if self._pose_source == SRC_PREDICTION:
                # 距离门（2026-10-07）：远场锚点本就不可信，不许拿它盲开
                near = self._dock_x is not None and abs(self._dock_x) <= float(
                    self.get_parameter("reacquire_drive_max_dist_m").value
                )
                if (
                    near
                    and self._reacquire_drive_elapsed
                    <= float(
                        self.get_parameter("reacquire_drive_max_sec").value
                    )
                ):
                    return MODE_APPROACH
                return MODE_HOLD
            if self._pose_source == SRC_VISION:
                return MODE_HOLD
            return MODE_SEARCH
        if s == DockState.ABORT_EXIT:
            return MODE_EXIT_FORWARD
        if s == DockState.UNDOCK_EXIT:
            return MODE_UNDOCK_FORWARD
        if s == DockState.DOCKED:
            return MODE_DOCKED_HOLD
        return MODE_HOLD  # IDLE / FAILED / UNDOCK_SETTLE / WAIT_DOCK_OPEN / WAIT_DOCK_RELEASE

    # ══════════════ 发布 ══════════════
    def _publish(self, mode: str):
        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)

        mode_msg = String()
        mode_msg.data = mode
        self._target_mode_pub.publish(mode_msg)

        x, e_y, e_yaw = self._errors()
        # dock_mission 契约：
        #   MONITOR_DOCK   只看 success / needs_reapproach(+abort_reason)
        #   MONITOR_UNDOCK 看 undock_success / state=="DOCK_ABORT"(+abort_reason)
        # FAILED 必须：
        #   - state 映射为 "DOCK_ABORT"（出泊失败能被上层捕获）
        #   - needs_reapproach 强制 true（入泊失败让上层走重试->人工接管流水线，
        #     否则 MONITOR_DOCK 会挂死）
        failed = self._state == DockState.FAILED
        status = {
            "state": "DOCK_ABORT" if failed else self._state,
            "v2_state": self._state,
            "target_mode": mode,
            "success": self._success,
            "undock_success": self._undock_success,
            "abort_reason": self._abort_reason,
            "needs_reapproach": self._needs_reapproach or failed,
            "needs_manual_takeover": self._needs_manual_takeover,
            "pose_valid": self._pose_source != SRC_INVALID,
            "pose_source": self._pose_source,
            "tag_visible": self._tag_visible,
            "tag_age_sec": (
                round(self._measurement_age, 2)
                if math.isfinite(self._measurement_age)
                else None
            ),
            "mission_elapsed_sec": round(self._mission_elapsed(), 1),
            "dock_claw_enabled": self._claw_on(),
            "dock_claw_open_on_start": self._claw_open_on_start(),
            "dock_claw_release_on_undock": self._claw_release_on_undock(),
            "backlight_enabled": self._backlight_on(),
            "backlight_session_active": self._backlight_session_active,
            "claw_grabbed": self._claw_grabbed,
        }
        if x is not None:
            status["dock_x"] = round(x, 3)
            status["dock_y"] = round(e_y, 3)
            status["e_yaw_deg"] = round(math.degrees(e_yaw), 2)
        status_msg = String()
        status_msg.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DockingFsm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
