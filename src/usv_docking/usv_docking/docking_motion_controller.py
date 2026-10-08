#!/usr/bin/env python3
"""docking_motion_controller — 运动控制器。

消费 FSM 的 target_mode + 估计器的 dock_pose，输出 cmd_vel（差速双推进：
只有 linear.x / angular.z，无横移能力）。

控制律（dock_est 系误差：x, e_y=y, e_yaw=wrap(yaw_b - pi)）：
  - 倒船（BACK_IN/FINAL_DOCK，v<0）：ẏ ≈ |v|·e_yaw
      e_yaw_des = -ky·e_y  →  ω = -kyaw·(e_yaw + ky·e_y)
  - 前进驶出（EXIT_FORWARD/UNDOCK_FORWARD，v>0）：ẏ ≈ -v·e_yaw
      e_yaw_des = +ky·e_y  →  ω = kyaw·(ky·e_y - e_yaw)
  - ALIGN：小倒速弧线消艏向（v=-align_arc_speed，x 超限回退 v=0），
    ω = -kyaw_align·e_yaw
  - APPROACH：船尾朝预备点 (staging_x, 0) 倒退（v<0，全程保住 Tag 视线）；
    stern_bearing 大则原地转，小则倒退弧线
  - SEARCH：有预测位姿则闭环绕"船尾指向坞"方位旋转；无锚点则开环恒速自转
  - SEARCH_LIMITED：闭环旋转，但相对进入时艏向累计限 ±backin_max_search_angle

以上各处角度 P 律输出统一叠加角速度阻尼（2026-09-30，兴飞重船过冲）：
  ω = ω_P − kd_yaw·ω_meas（/mavros/imu/data，best_effort 订阅；无数据退化为纯 P）
  SEARCH/SEARCH_LIMITED 为恒速/定向旋转，不加阻尼。
  - HOLD/DOCKED_HOLD：v→0 斜坡停车，停稳且发够 hold_zero_publish_sec 零速帧后
    静默（不再占用 /cmd_vel_nav，让位 teleop；实船桥有 command_timeout 看门狗兜底）
  - INVALID 运动态：v→0（仍常发，防停车指令丢失）

所有输出经加速度斜坡；safety_stop=true 立即清零（绕过斜坡）。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

# target_mode（MODE_*）与 pose_source（SRC_*）字符串统一在
# usv_docking.docking_common 定义（与 docking_fsm 同一来源，取值不变）
from usv_docking.docking_common import (
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
    wrap_angle,
    yaw_from_quat,
)

MOTION_MODES = (
    MODE_APPROACH,
    MODE_ALIGN,
    MODE_BACK_IN,
    MODE_FINAL_DOCK,
    MODE_EXIT_FORWARD,
    MODE_UNDOCK_FORWARD,
)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class DockingMotionController(Node):
    def __init__(self):
        super().__init__("docking_motion_controller")

        # ── 输入话题 ──
        self.declare_parameter("target_mode_topic", "/docking/target_mode")
        self.declare_parameter("dock_pose_topic", "/docking/dock_pose")
        self.declare_parameter("pose_source_topic", "/docking/pose_source")
        self.declare_parameter("safety_stop_topic", "/docking/safety_stop")

        # ── 输出 ──
        # 代码默认 true 是安全联锁（脱离 yaml 裸跑时只发测试话题，不驱动实船）；
        # yaml/launch 显式给 false 才是接管 /cmd_vel_nav。这是唯一一处**刻意**与
        # yaml 不一致的默认值，已在 test_param_defaults.py 列为豁免项。
        self.declare_parameter("test_only", True)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_test_topic", "/docking/cmd_vel_test")
        self.declare_parameter("control_rate", 20.0)
        # HOLD/DOCKED_HOLD 进入后持续发零速的时长：停稳+发够即静默，
        # 让出 cmd_vel 通道给 teleop（实船 usv_ardupilot_velocity_bridge
        # 有 command_timeout_sec 看门狗，超时自动发零，无需常发保持）
        self.declare_parameter("hold_zero_publish_sec", 1.0)

        # ── 目标点（与 FSM 一致，改动须两处同步）──
        self.declare_parameter("staging_x", -2.0)
        self.declare_parameter("final_target_x", 0.0)

        # ── 全局限幅 / 斜坡 ──
        self.declare_parameter("max_reverse_speed", 0.8)
        # 以下默认值与 config/docking.yaml 的实船标定值对齐（2026-10-05），
        # 不带配置文件启动时也不应落到未标定的激进值
        self.declare_parameter("max_yaw_rate", 0.12)
        self.declare_parameter("max_linear_accel", 0.30)
        self.declare_parameter("max_angular_accel", 0.50)

        # ── 角速度阻尼（D 项）──
        # 重船转动惯量大，纯 P 收敛到 e_yaw=0 时船体仍在转 → 冲过目标荡秋千；
        # w = w_P − kd_yaw·ω_meas 在船体冲向目标时提前刹车。0 = 退化为纯 P
        self.declare_parameter("yaw_rate_topic", "/mavros/imu/data")
        # 2026-10-07：0.5→0.7。转向阶跃试验实测"桥输出→船体实际角速度"滞后 1.36s
        # （CMD→IMU 全程 1.58s），属船体/推进物理特性（见 docs/船体动力学指标.md）；
        # kd_yaw 用 IMU 实测角速度做阻尼，是唯一能部分抵消该滞后的旋钮
        self.declare_parameter("kd_yaw", 0.7)
        # APPROACH 过预备线换向迟滞（m）：越过此深度才前进倒出
        self.declare_parameter("approach_reexit_margin", 0.35)
        # 远横偏先上轴（外八坞）：|e_y| 超过 y_far 且 x 比 far_x 近（更靠坞）时，
        # 目标点换到 (axis_far_x, 0)，前进大弧线先对齐轴线再倒入
        self._axis_mode = False  # 先上轴模式滞回状态
        self.declare_parameter("approach_axis_y_far", 0.8)
        self.declare_parameter("approach_axis_far_x", -4.5)
        # 大横偏限速：|e_y| 超 offaxis_y 时 APPROACH 速度压到 offaxis_speed
        self.declare_parameter("approach_offaxis_y", 0.5)
        # 先上轴模式：退出门限 / 向外多拉距离 / 蟹行角（大角度快速收横偏）
        self.declare_parameter("approach_axis_y_near", 0.55)
        self.declare_parameter("approach_axis_pullout", 2.0)
        self.declare_parameter("approach_axis_crab_deg", 25.0)
        # 上轴模式的外边界（dock 系 x，越负越靠外）：上轴目标不得比它更靠外。
        # 2026-10-07 现场教训：目标原本是"min(x,-4.5) - 2.0"，会随 x 一起后退
        # → 永远差 2m 追不到，船被一路带走到 x≈-13m（超出后视相机 9~10m 的
        # 二维码检测极限，河道也没这个空间，且丢 Tag 后转成盲驾）。按船上传感器
        # 位置与河道空间，取 -8.0（可放宽到 -9.0，须 < 相机检测极限）
        self.declare_parameter("approach_axis_max_out_x", -8.0)
        self.declare_parameter("approach_offaxis_speed", 0.15)
        # 大横偏限速只在坞口近处生效（x 大于此值，坞系）
        self.declare_parameter("approach_offaxis_x", -4.5)
        # 预备点退化区去奇异化（2026-10-07 现场 bag 实锤）：船到预备线附近带
        # 0.2~0.5m 横偏时，"船尾指向目标点"的方位角在 dx≈0 处在 ±90°~±180°
        # 整幅翻转（方位角奇异），v=0 原地打转；且旋转扫弧（Δy≈R_eff·sinθ
        # ≈0.68m）大于换向迟滞 reexit_margin(0.35m)，换向-扫弧自我维持振荡，
        # 横向永远修不掉只能等 120s 超时。退化区内弃用方位角跟踪，改用
        # BACK_IN 同构的轴对准倒船律蠕行修横偏
        self.declare_parameter("approach_degenerate_dist", 0.6)
        self.declare_parameter("approach_degenerate_speed", 0.12)
        self.declare_parameter("approach_y_near", 0.25)

        # ── 搜索 ──
        self.declare_parameter("search_angular_speed", 0.15)
        self.declare_parameter("backin_max_search_angle_deg", 8.0)

        # ── 分阶段速度 ──
        self.declare_parameter("approach_speed", 0.45)
        self.declare_parameter("approach_turn_deg", 25.0)  # |方位差| 超此值原地转
        self.declare_parameter("approach_crab_deg", 8.0)  # 前进倒出蟹行角限幅
        # 接近轴线锥形降速：|e_y|<slow_y 时按比例降速（下限 min_speed），
        # 防全速横移过冲荡秋千（2026-07-29 用户现场反馈；08-10 实测 |y|<0.5 前
        # 一直是 0.45 全速，对 1.15m 船太晚）
        self.declare_parameter("approach_slow_y", 1.0)
        self.declare_parameter("approach_slow_dist", 2.50)
        self.declare_parameter("approach_min_speed", 0.15)
        # 2026-10-07：廊道单侧仅 5cm，0.40/0.20 的"阶跃"上限配 ~2s 转向滞后会
        # 在入口窜速再线性降；整体下调并缩小两级差
        self.declare_parameter("back_in_speed_normal", 0.25)
        self.declare_parameter("back_in_speed_slow", 0.15)
        self.declare_parameter("back_in_gate1_y", 0.20)
        self.declare_parameter("back_in_gate1_yaw_deg", 5.0)
        self.declare_parameter("back_in_gate2_y", 0.35)
        self.declare_parameter("back_in_gate2_yaw_deg", 15.0)
        self.declare_parameter("final_dock_speed", 0.15)
        # 压紧段倒速（2026-10-05）：FINAL_DOCK 到达原"死位置"后不再停车——
        # 浮动坞位置会漂移，死位置停船离夹爪/充电板仍差一截；低速继续倒船
        # 把船压到位，成功/退出判定由 FSM（无线充电信号/WaitIO/压紧超时）负责
        self.declare_parameter("final_push_speed", 0.18)
        # 终局消艏偏窗口（与 FSM docked 判据同值）：x/y 达标后原地消 e_yaw。
        # 倒船律稳态 e_yaw=-ky·e_y，横向残差 7cm 即产生 3.2° 艏偏，超过判据 3°
        # 形成"停船但判据不满足"死锁（2026-07-29 实测 x=0.11,y=0.071 卡死）
        self.declare_parameter("docked_x_tol", 0.15)
        self.declare_parameter("docked_y_tol", 0.10)
        self.declare_parameter("abort_exit_speed", 0.30)
        self.declare_parameter("undock_speed", 0.45)
        self.declare_parameter("exit_turn_deg", 30.0)  # 驶出中 |e_yaw| 超此值先转正

        # ── 控制增益 ──
        self.declare_parameter("ky_approach", 0.40)
        self.declare_parameter("kyaw_approach", 0.45)  # 2026-10-06: 0.85→0.45 拓宽比例带(~15°)，治"角速度总打满"
        self.declare_parameter("kyaw_align", 0.45)   # 2026-10-06: 0.90→0.45 同上
        # ALIGN 弧线化（2026-08-22）：v=0 原地转时差速双推进（船尾 x=-0.3）
        # 使船绕推进器连线中点旋转，base_link 扫弧 Δy≈R_eff·sinθ
        # （sim 实测 R_eff≈0.3~0.42m；实船 MAVROS→ArduPilot Guided 链路下
        # R_eff 未知且必然不同）。改为小倒速弧线：扫弧变受控倒退弧线，
        # 横漂≈|v|·sin(e_yaw)，入口 e_yaw≤10°（fix 8d443ef 门槛）下
        # ≈1.4cm/s；行为层修复，不依赖执行链参数，仿真/实船语义一致。
        self.declare_parameter("align_arc_speed", 0.08)
        # x 门限保护：ALIGN 卡死（艏向不收敛干等超时）时防止弧线持续倒船
        # 滑入坞内——x≥limit 后退回 v=0 纯旋转，由 FSM align_timeout 兜底
        self.declare_parameter("align_arc_x_limit", -1.5)
        self.declare_parameter("ky_back", 0.80)
        self.declare_parameter("kyaw_back", 0.80)
        self.declare_parameter("kx_final", 0.25)
        self.declare_parameter("back_in_max_yaw_rate", 0.12)

        p = self.get_parameter
        qos = QoSProfile(depth=10)

        self._test_only = bool(p("test_only").value)
        out_topic = (
            p("cmd_vel_test_topic").value
            if self._test_only
            else p("cmd_vel_topic").value
        )
        self._cmd_pub = self.create_publisher(Twist, out_topic, qos)

        self.create_subscription(
            String, p("target_mode_topic").value, self._mode_cb, qos
        )
        self.create_subscription(
            PoseStamped, p("dock_pose_topic").value, self._pose_cb, qos
        )
        self.create_subscription(
            String, p("pose_source_topic").value, self._src_cb, qos
        )
        self.create_subscription(
            Bool, p("safety_stop_topic").value, self._safety_cb, qos
        )

        # IMU 角速度反馈（D 项阻尼）；mavros 传感器话题为 best_effort
        imu_qos = QoSProfile(depth=10)
        from rclpy.qos import ReliabilityPolicy

        imu_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Imu, p("yaw_rate_topic").value, self._imu_cb, imu_qos
        )

        # ── 运行状态 ──
        self._mode = MODE_HOLD
        self._pose_x = None
        self._pose_y = None
        self._pose_yaw = None
        self._pose_source = SRC_INVALID
        self._safety_stop = False
        self._yaw_rate = None  # IMU 角速度 z（rad/s，CCW+；无数据时 D 项退化）

        self._v_cmd = 0.0
        self._w_cmd = 0.0

        # 模式切换沿（重置 SEARCH_LIMITED 进入时艏向）
        self._last_mode = MODE_HOLD
        self._limited_entry_yaw = None

        # HOLD/DOCKED_HOLD 零速帧余量（>0 期间即使已停稳也继续发零速）
        self._hold_settle_ticks = 0

        rate = float(p("control_rate").value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"docking_motion_controller 已启动，输出 -> {out_topic}"
            + ("（test_only）" if self._test_only else "（真实 cmd_vel!）")
        )

    # ══════════════ 输入回调 ══════════════
    def _mode_cb(self, msg: String):
        self._mode = msg.data

    def _pose_cb(self, msg: PoseStamped):
        self._pose_x = msg.pose.position.x
        self._pose_y = msg.pose.position.y
        self._pose_yaw = yaw_from_quat(
            msg.pose.orientation.z, msg.pose.orientation.w
        )

    def _src_cb(self, msg: String):
        self._pose_source = msg.data

    def _safety_cb(self, msg: Bool):
        self._safety_stop = bool(msg.data)

    def _imu_cb(self, msg: Imu):
        self._yaw_rate = float(msg.angular_velocity.z)

    def _yaw_damp(self, w_p: float) -> float:
        """P 输出叠加角速度阻尼：w = w_P − kd·ω_meas（ω_meas 与 cmd 同 CCW+）。
        船体冲向目标时提前刹车；无 IMU 数据时退化为纯 P。"""
        if self._yaw_rate is None:
            return w_p
        kd = float(self.get_parameter("kd_yaw").value)
        return w_p - kd * self._yaw_rate

    # ══════════════ 控制律 ══════════════
    def _errors(self):
        if self._pose_x is None:
            return None, None, None
        return (
            self._pose_x,
            self._pose_y,
            wrap_angle(self._pose_yaw - math.pi),
        )

    def _pose_valid(self) -> bool:
        return self._pose_source != SRC_INVALID and self._pose_x is not None

    def _compute_approach(self, x, e_y, e_yaw, yaw_b):
        """APPROACH 双向就位，船尾相机全程朝坞：

        - 船在预备点外侧（x < staging_x）：倒船入位（船尾朝目标倒退）。
        - 船在预备点内侧（x > staging_x，如过冲/坞边重启）：前进倒出，
          与 EXIT_FORWARD 同构（v>0 且 e_yaw->0 保持船尾朝坞）。
          原单一倒船律在此情形要求船尾调转 180°，stern_bearing 落在 ±π
          回绕奇点上，噪声致转向符号 bang-bang 震荡（2026-07-29 实测）。
        """
        p = self.get_parameter
        # 远横偏先上轴（外八坞防斜插撞喇叭口，2026-10-06）：横偏大时把目标换成
        # 轴线上"比当前更靠外"的点，走下方 dx<0 前进弧线分支：向前拉出的同时
        # 用大蟹行角收拢横偏（船尾/相机始终朝坞），回轴后正常倒入。
        # 滞回：>|y_far| 进入，<|y_near| 退出，防边界抖动。
        # 教训：目标取固定远处点时，船在其外侧会"边倒边修 y"斜插（10-06 现场）。
        y_far = float(p("approach_axis_y_far").value)
        y_near = float(p("approach_axis_y_near").value)
        if self._axis_mode and abs(e_y) < y_near:
            self._axis_mode = False
        elif not self._axis_mode and abs(e_y) > y_far:
            self._axis_mode = True
        staging = float(p("staging_x").value)
        crab_deg = float(p("approach_crab_deg").value)
        if self._axis_mode:
            # 目标封底：目标不得比 approach_axis_max_out_x 更靠外（否则目标随 x
            # 后退永远追不到，把船一路带到检测极限外）；船若已在边界外，dx>0 会
            # 自动落到倒船分支把它拉回来
            staging = max(
                min(x, float(p("approach_axis_far_x").value))
                - float(p("approach_axis_pullout").value),
                float(p("approach_axis_max_out_x").value),
            )
            crab_deg = float(p("approach_axis_crab_deg").value)
        dx = staging - x  # >0 船在外侧需倒入；<0 船在内侧需倒出
        dy = 0.0 - e_y
        turn_th = math.radians(float(p("approach_turn_deg").value))
        cap = float(p("approach_speed").value)
        dist = math.hypot(dx, dy)
        # 锥形降速：|e_y|>=slow_y 且距预备点>=slow_dist 全速；任一趋 0 即降速到
        # min_speed 蠕行（08-10 实测旧逻辑只在 |y|<0.5 才开始降，太晚）
        v_min = float(p("approach_min_speed").value)
        slow_y = float(p("approach_slow_y").value)
        slow_dist = float(p("approach_slow_dist").value)
        v_scale = (
            max(
                v_min / cap,
                min(1.0, abs(e_y) / slow_y, dist / slow_dist),
            )
            if cap > 0
            else 1.0
        )
        v_cap = cap * v_scale

        # 大横偏限速（2026-10-06 现场：0.44m/s 横偏 1.2m 斜插撞坞口廊道）：
        # 仅坞口近处（x > offaxis_x）生效：远场开阔水保持快速收敛，
        # 近坞口才压速修横偏（与锥形降速互补，不把远场拖慢）
        if (abs(e_y) > float(p("approach_offaxis_y").value)
                and x > float(p("approach_offaxis_x").value)):
            v_cap = min(v_cap, float(p("approach_offaxis_speed").value))

        # 预备点退化区去奇异化（2026-10-07 现场 bag 实锤）：预备线附近带横偏时
        # stern_bearing 在 dx≈0 处 ±90°~±180° 整幅翻转（方位角奇异），v=0
        # 原地打转，且旋转扫弧 Δy≈0.68m > 换向迟滞 reexit_margin(0.35m)
        # 自我维持振荡，横向永远修不掉。区内弃用方位角跟踪，改用与 BACK_IN
        # 同构的轴对准倒船律（ω = -kyaw·(e_yaw + ky·e_y)，见文件头部公式约定）
        if dist < float(p("approach_degenerate_dist").value):
            w = self._yaw_damp(
                -self.kyaw_approach() * (e_yaw + self.ky_approach() * e_y)
            )
            # 横偏未收敛且还在预备线外侧：恒定小倒速蠕行（给横向律提供行程，
            # 倒船横向律 ẏ≈|v|·e_yaw，v=0 则 y 数学上修不动）；否则 v=0 只修
            # 艏向，让 FSM 到位判据接管
            if abs(e_y) > float(p("approach_y_near").value) and dx > -0.1:
                v = -float(p("approach_degenerate_speed").value)
            else:
                v = 0.0
            return v, clamp(w, self.max_yaw_rate())

        # 迟滞：越过预备线不足 approach_reexit_margin 不换向，防 staging 线两侧
        # 乒乓（2026-10-06 现场：前移一点就停再倒）
        if dx < -float(p("approach_reexit_margin").value):
            # 内侧倒出：前进，艏向/横向同 EXIT 律（ω = kyaw·(e_yaw_des − e_yaw)）
            # 蟹行角限幅：纯P横向律 e_yaw_des=ky·e_y 在大 e_y 下横移过猛且无阻尼，
            # 必冲过 0 到另一侧（2026-07-29 实测 y: 0.49→-0.20 荡秋千）；
            # 限幅后小角度慢修，y 单调平缓收敛
            crab = math.radians(crab_deg)
            e_yaw_des = clamp(self.ky_approach() * e_y, crab)
            w = self._yaw_damp(self.kyaw_approach() * (e_yaw_des - e_yaw))
            if abs(e_yaw) > turn_th:
                return 0.0, clamp(w, self.max_yaw_rate())
            v = min(v_cap, abs(dx))
            return v, clamp(w, self.max_yaw_rate())

        # 外侧倒入：stern_bearing 为目标相对方位（以船尾方向 yaw_b+pi 为基准）
        stern_bearing = wrap_angle(math.atan2(dy, dx) - (yaw_b + math.pi))
        w = clamp(
            self._yaw_damp(self.kyaw_approach() * stern_bearing),
            self.max_yaw_rate(),
        )
        if abs(stern_bearing) > turn_th:
            return 0.0, w
        v = -min(v_cap, dist)
        return v, w

    def _compute_align(self, x, e_yaw):
        """ALIGN 弧线化消艏向：v=-arc_speed（x 超限回退 v=0）。

        旧版 v=0 纯原地转：旋转中心在船尾推进器中点，base_link 扫弧
        （2026-08-10 实测 Δy≈R_eff·sinθ）。小倒速弧线把不可控扫弧变为
        受控倒退弧线（横漂 |v|·sin(e_yaw)，入口 e_yaw≤10° 下极小），
        并顺带向坞推进；x 门限防止卡死时倒船滑入坞内。
        """
        p = self.get_parameter
        w = clamp(
            self._yaw_damp(-float(p("kyaw_align").value) * e_yaw),
            self.max_yaw_rate(),
        )
        arc = float(p("align_arc_speed").value)
        x_limit = float(p("align_arc_x_limit").value)
        v = -arc if (arc > 0.0 and x < x_limit) else 0.0
        return v, w

    def _compute_back_in(self, x, e_y, e_yaw, speed_cap, yaw_cap, push=False):
        p = self.get_parameter
        # 横向/艏向门控：超出门控2 -> v=0 只修艏向（压紧段同样生效，角度偏了不硬推）
        g2_y = float(p("back_in_gate2_y").value)
        g2_yaw = math.radians(float(p("back_in_gate2_yaw_deg").value))
        if abs(e_y) > g2_y or abs(e_yaw) > g2_yaw:
            v = 0.0
        else:
            x_tol = float(p("docked_x_tol").value)
            target = float(p("final_target_x").value)
            dist = target - x  # >0 还需继续倒入
            if abs(dist) < x_tol and abs(e_y) < float(p("docked_y_tol").value):
                # 终局消艏偏：x/y 已达标，原地旋转把 e_yaw 压到 0
                # （解除 e_yaw=-ky·e_y 稳态平衡，该平衡随横向残差必超 3° 判据）
                w = self._yaw_damp(-float(p("kyaw_back").value) * e_yaw)
                if push:
                    # 压紧段（FINAL_DOCK）：到位不停，低速继续倒船压向夹爪/
                    # 充电板；w 保持消艏偏阻尼，到位判定交给 FSM
                    return -float(p("final_push_speed").value), clamp(w, yaw_cap)
                return 0.0, clamp(w, yaw_cap)
            if abs(dist) < x_tol:
                # x 已达位但 |e_y| 仍超 docked_y_tol：kx_final·dist≈0 若直接
                # 清零 v，横向律失去纵向行程（ẏ≈|v|·e_yaw）干等到超时。
                # 保持小倒速继续执行横向修正律；纵向行程以 docked 窗口远缘
                # （target + docked_x_tol）为界，到界 v→0，由 FSM 超时兜底
                remaining = target + x_tol - x
                v = -min(speed_cap, max(0.0, self.kx_final() * remaining))
            else:
                v = -min(speed_cap, max(0.0, self.kx_final() * dist))
        # 倒船：ω = -kyaw·(e_yaw + ky·e_y)，叠加角速度阻尼
        w = self._yaw_damp(
            -float(p("kyaw_back").value)
            * (e_yaw + float(p("ky_back").value) * e_y)
        )
        return v, clamp(w, yaw_cap)

    def _compute_exit_forward(self, e_y, e_yaw, speed):
        # 前进：ω = kyaw·(ky·e_y - e_yaw)（复用 approach 增益，同构前向几何）
        turn_th = math.radians(float(self.get_parameter("exit_turn_deg").value))
        w = self._yaw_damp(
            self.kyaw_approach() * (self.ky_approach() * e_y - e_yaw)
        )
        if abs(e_yaw) > turn_th:
            return 0.0, clamp(w, self.max_yaw_rate())
        return speed, clamp(w, self.max_yaw_rate())

    def _compute_search(self, x, e_y, yaw_b, limited: bool):
        """返回 (v, w)。有预测位姿 -> 闭环绕船尾对坞方位；否则开环恒速自转。"""
        p = self.get_parameter
        search_rate = float(p("search_angular_speed").value)
        if limited:
            max_dev = math.radians(
                float(p("backin_max_search_angle_deg").value)
            )
        else:
            max_dev = None

        if self._pose_valid():
            # 船尾（yaw_b+pi）指向坞中心（原点）的相对方位
            stern_bearing = wrap_angle(
                math.atan2(-e_y, -x) - (yaw_b + math.pi)
            )
            w = clamp(self.kyaw_approach() * stern_bearing, search_rate)
            if max_dev is not None and self._limited_entry_yaw is not None:
                dev = wrap_angle(yaw_b - self._limited_entry_yaw)
                # 限角：只允许朝限界内旋转
                if dev > max_dev and w > 0:
                    w = 0.0
                elif dev < -max_dev and w < 0:
                    w = 0.0
            return 0.0, w

        # 无锚点：开环恒速自转找 Tag。
        # 注：INVALID 时估计器发布的 yaw_b 冻结，任何"按偏差换向/扩角"的
        # 闭环扫描都不可达（历史死代码已移除），实际行为就是固定方向旋转。
        if limited:
            return 0.0, 0.0  # 坞内无预测不做任何动作，等 FSM 裁决
        return 0.0, search_rate

    # 参数快捷访问
    def kyaw_approach(self):
        return float(self.get_parameter("kyaw_approach").value)

    def ky_approach(self):
        return float(self.get_parameter("ky_approach").value)

    def kx_final(self):
        return float(self.get_parameter("kx_final").value)

    def max_yaw_rate(self):
        return float(self.get_parameter("max_yaw_rate").value)

    # ══════════════ 主循环 ══════════════
    def _control_loop(self):
        p = self.get_parameter
        mode = self._mode

        # 模式切换沿：重置 SEARCH_LIMITED 进入时艏向
        if mode != self._last_mode:
            self._limited_entry_yaw = (
                self._pose_yaw if mode == MODE_SEARCH_LIMITED else None
            )
            if mode in (MODE_HOLD, MODE_DOCKED_HOLD):
                # 进入保持态：先发一小段零速确保停住，随后静默让出 cmd_vel
                hold_sec = float(p("hold_zero_publish_sec").value)
                self._hold_settle_ticks = max(
                    1, int(math.ceil(hold_sec / self._dt))
                )
            self._last_mode = mode

        hold_mode = mode in (MODE_HOLD, MODE_DOCKED_HOLD)

        x, e_y, e_yaw = self._errors()
        yaw_b = self._pose_yaw if self._pose_yaw is not None else 0.0

        v_target, w_target = 0.0, 0.0

        if mode in (MODE_HOLD, MODE_DOCKED_HOLD):
            pass
        elif mode in MOTION_MODES and not self._pose_valid():
            # 位姿不可用：运动态全部停车，由 FSM 走异常路径
            pass
        elif mode == MODE_APPROACH:
            v_target, w_target = self._compute_approach(x, e_y, e_yaw, yaw_b)
        elif mode == MODE_ALIGN:
            v_target, w_target = self._compute_align(x, e_yaw)
        elif mode == MODE_BACK_IN:
            g1_y = float(p("back_in_gate1_y").value)
            g1_yaw = math.radians(float(p("back_in_gate1_yaw_deg").value))
            cap = (
                float(p("back_in_speed_normal").value)
                if (abs(e_y) < g1_y and abs(e_yaw) < g1_yaw)
                else float(p("back_in_speed_slow").value)
            )
            v_target, w_target = self._compute_back_in(
                x, e_y, e_yaw, cap, float(p("back_in_max_yaw_rate").value)
            )
        elif mode == MODE_FINAL_DOCK:
            v_target, w_target = self._compute_back_in(
                x,
                e_y,
                e_yaw,
                float(p("final_dock_speed").value),
                float(p("back_in_max_yaw_rate").value),
                push=True,
            )
        elif mode == MODE_EXIT_FORWARD:
            v_target, w_target = self._compute_exit_forward(
                e_y, e_yaw, float(p("abort_exit_speed").value)
            )
        elif mode == MODE_UNDOCK_FORWARD:
            v_target, w_target = self._compute_exit_forward(
                e_y, e_yaw, float(p("undock_speed").value)
            )
        elif mode == MODE_SEARCH:
            v_target, w_target = self._compute_search(x, e_y, yaw_b, False)
        elif mode == MODE_SEARCH_LIMITED:
            v_target, w_target = self._compute_search(x, e_y, yaw_b, True)

        # 全局限幅
        v_target = clamp(v_target, float(p("max_reverse_speed").value))
        w_target = clamp(w_target, self.max_yaw_rate())

        # safety_stop：立即清零，绕过斜坡
        if self._safety_stop:
            self._v_cmd = 0.0
            self._w_cmd = 0.0
            if hold_mode:
                # 保持态被安全停车时至少补发 3 帧零速，防单帧丢失
                self._hold_settle_ticks = max(self._hold_settle_ticks, 3)
        else:
            dv = clamp(
                v_target - self._v_cmd,
                float(p("max_linear_accel").value) * self._dt,
            )
            dw = clamp(
                w_target - self._w_cmd,
                float(p("max_angular_accel").value) * self._dt,
            )
            self._v_cmd += dv
            self._w_cmd += dw

        if hold_mode:
            stopped = abs(self._v_cmd) < 1e-3 and abs(self._w_cmd) < 1e-3
            if stopped and self._hold_settle_ticks <= 0:
                # 已停稳且零速帧发够：静默，把 /cmd_vel_nav 让给 teleop
                # （运动态不受此限，仍 20Hz 常发）
                return
            if self._hold_settle_ticks > 0:
                self._hold_settle_ticks -= 1

        twist = Twist()
        twist.linear.x = self._v_cmd
        twist.angular.z = self._w_cmd
        self._cmd_pub.publish(twist)

    def destroy_node(self):
        try:
            zero = Twist()
            for _ in range(3):
                self._cmd_pub.publish(zero)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DockingMotionController()
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
