#!/usr/bin/env python3
"""docking_motion_controller_v2 — V2 运动控制器。

消费 FSM 的 target_mode + 估计器的 dock_pose，输出 cmd_vel（差速双推进：
只有 linear.x / angular.z，无横移能力）。

控制律（dock_est 系误差：x, e_y=y, e_yaw=wrap(yaw_b - pi)）：
  - 倒船（BACK_IN/FINAL_DOCK，v<0）：ẏ ≈ |v|·e_yaw
      e_yaw_des = -ky·e_y  →  ω = -kyaw·(e_yaw + ky·e_y)
  - 前进驶出（EXIT_FORWARD/UNDOCK_FORWARD，v>0）：ẏ ≈ -v·e_yaw
      e_yaw_des = +ky·e_y  →  ω = kyaw·(ky·e_y - e_yaw)
  - ALIGN：v=0，ω = -kyaw_align·e_yaw
  - APPROACH：船尾朝预备点 (staging_x, 0) 倒退（v<0，全程保住 Tag 视线）；
    stern_bearing 大则原地转，小则倒退弧线
  - SEARCH：有预测位姿则闭环绕"船尾指向坞"方位旋转；无锚点则开环扇形扫描
  - SEARCH_LIMITED：闭环旋转，但相对进入时艏向累计限 ±backin_max_search_angle
  - HOLD/DOCKED_HOLD/INVALID 运动态：v→0

所有输出经加速度斜坡；safety_stop=true 立即清零（绕过斜坡）。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String

# target_mode 字符串（与 docking_fsm_v2 一致）
MODE_HOLD = "HOLD"
MODE_SEARCH = "SEARCH"
MODE_APPROACH = "APPROACH"
MODE_ALIGN = "ALIGN"
MODE_BACK_IN = "BACK_IN"
MODE_FINAL_DOCK = "FINAL_DOCK"
MODE_SEARCH_LIMITED = "SEARCH_LIMITED"
MODE_EXIT_FORWARD = "EXIT_FORWARD"
MODE_UNDOCK_FORWARD = "UNDOCK_FORWARD"
MODE_DOCKED_HOLD = "DOCKED_HOLD"

SRC_VISION = "VISION"
SRC_INVALID = "INVALID"

MOTION_MODES = (
    MODE_APPROACH,
    MODE_ALIGN,
    MODE_BACK_IN,
    MODE_FINAL_DOCK,
    MODE_EXIT_FORWARD,
    MODE_UNDOCK_FORWARD,
)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


class DockingMotionControllerV2(Node):
    def __init__(self):
        super().__init__("docking_motion_controller_v2")

        # ── 输入话题 ──
        self.declare_parameter("state_topic", "/docking_v2/state")
        self.declare_parameter("target_mode_topic", "/docking_v2/target_mode")
        self.declare_parameter("dock_pose_topic", "/docking_v2/dock_pose")
        self.declare_parameter("pose_source_topic", "/docking_v2/pose_source")
        self.declare_parameter("safety_stop_topic", "/docking_v2/safety_stop")

        # ── 输出 ──
        self.declare_parameter("test_only", True)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_test_topic", "/docking_v2/cmd_vel_test")
        self.declare_parameter("control_rate", 20.0)

        # ── 目标点（与 FSM 一致）──
        self.declare_parameter("staging_x", -2.5)
        self.declare_parameter("final_target_x", 0.0)

        # ── 全局限幅 / 斜坡 ──
        self.declare_parameter("max_reverse_speed", 0.5)
        self.declare_parameter("max_yaw_rate", 0.35)
        self.declare_parameter("max_linear_accel", 0.25)
        self.declare_parameter("max_angular_accel", 0.60)

        # ── 搜索 ──
        self.declare_parameter("search_angular_speed", 0.10)
        self.declare_parameter("search_initial_half_angle_deg", 15.0)
        self.declare_parameter("search_angle_increment_deg", 15.0)
        self.declare_parameter("search_max_half_angle_deg", 60.0)
        self.declare_parameter("backin_max_search_angle_deg", 8.0)

        # ── 分阶段速度 ──
        self.declare_parameter("approach_speed", 0.20)
        self.declare_parameter("approach_turn_deg", 25.0)  # |方位差| 超此值原地转
        self.declare_parameter("approach_crab_deg", 8.0)  # 前进倒出蟹行角限幅
        # 接近轴线锥形降速：|e_y|<slow_y 时按比例降速（下限 min_speed），
        # 防全速横移过冲荡秋千（2026-07-29 用户现场反馈）
        self.declare_parameter("approach_slow_y", 0.50)
        self.declare_parameter("approach_min_speed", 0.08)
        self.declare_parameter("back_in_speed_normal", 0.20)
        self.declare_parameter("back_in_speed_slow", 0.10)
        self.declare_parameter("back_in_gate1_y", 0.20)
        self.declare_parameter("back_in_gate1_yaw_deg", 5.0)
        self.declare_parameter("back_in_gate2_y", 0.35)
        self.declare_parameter("back_in_gate2_yaw_deg", 10.0)
        self.declare_parameter("final_dock_speed", 0.08)
        # 终局消艏偏窗口（与 FSM docked 判据同值）：x/y 达标后原地消 e_yaw。
        # 倒船律稳态 e_yaw=-ky·e_y，横向残差 7cm 即产生 3.2° 艏偏，超过判据 3°
        # 形成"停船但判据不满足"死锁（2026-07-29 实测 x=0.11,y=0.071 卡死）
        self.declare_parameter("docked_x_tol", 0.15)
        self.declare_parameter("docked_y_tol", 0.10)
        self.declare_parameter("abort_exit_speed", 0.15)
        self.declare_parameter("undock_speed", 0.25)
        self.declare_parameter("exit_turn_deg", 30.0)  # 驶出中 |e_yaw| 超此值先转正

        # ── 控制增益 ──
        self.declare_parameter("ky_approach", 0.40)
        self.declare_parameter("kyaw_approach", 0.85)
        self.declare_parameter("kyaw_align", 0.90)
        self.declare_parameter("ky_back", 0.20)
        self.declare_parameter("kyaw_back", 0.40)
        self.declare_parameter("kx_final", 0.25)
        self.declare_parameter("back_in_max_yaw_rate", 0.18)

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
            String, p("state_topic").value, self._state_cb, qos
        )
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

        # ── 运行状态 ──
        self._mode = MODE_HOLD
        self._fsm_state = "IDLE"
        self._pose_x = None
        self._pose_y = None
        self._pose_yaw = None
        self._pose_source = SRC_INVALID
        self._safety_stop = False

        self._v_cmd = 0.0
        self._w_cmd = 0.0

        # 模式切换沿（重置搜索内部状态）
        self._last_mode = MODE_HOLD
        # 开环扇形扫描状态
        self._sweep_ref_yaw = None
        self._sweep_dir = 1.0
        self._sweep_half_angle = math.radians(
            float(p("search_initial_half_angle_deg").value)
        )
        # SEARCH_LIMITED 进入时艏向
        self._limited_entry_yaw = None

        rate = float(p("control_rate").value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"docking_motion_controller_v2 已启动，输出 -> {out_topic}"
            + ("（test_only）" if self._test_only else "（真实 cmd_vel!）")
        )

    # ══════════════ 输入回调 ══════════════
    def _state_cb(self, msg: String):
        self._fsm_state = msg.data

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
        dx = float(p("staging_x").value) - x  # >0 船在外侧需倒入；<0 船在内侧需倒出
        dy = 0.0 - e_y
        turn_th = math.radians(float(p("approach_turn_deg").value))
        cap = float(p("approach_speed").value)
        # 接近轴线锥形降速：|e_y|>=slow_y 全速，|e_y|->0 降到 min_speed 蠕行
        v_min = float(p("approach_min_speed").value)
        slow_y = float(p("approach_slow_y").value)
        v_scale = (
            max(v_min / cap, min(1.0, abs(e_y) / slow_y)) if cap > 0 else 1.0
        )
        v_cap = cap * v_scale

        if dx < -0.05:
            # 内侧倒出：前进，艏向/横向同 EXIT 律（ω = kyaw·(e_yaw_des − e_yaw)）
            # 蟹行角限幅：纯P横向律 e_yaw_des=ky·e_y 在大 e_y 下横移过猛且无阻尼，
            # 必冲过 0 到另一侧（2026-07-29 实测 y: 0.49→-0.20 荡秋千）；
            # 限幅后小角度慢修，y 单调平缓收敛
            crab = math.radians(float(p("approach_crab_deg").value))
            e_yaw_des = clamp(self.ky_approach() * e_y, crab)
            w = self.kyaw_approach() * (e_yaw_des - e_yaw)
            if abs(e_yaw) > turn_th:
                return 0.0, clamp(w, self.max_yaw_rate())
            v = min(v_cap, abs(dx))
            return v, clamp(w, self.max_yaw_rate())

        # 外侧倒入：stern_bearing 为目标相对方位（以船尾方向 yaw_b+pi 为基准）
        dist = math.hypot(dx, dy)
        stern_bearing = wrap_angle(math.atan2(dy, dx) - (yaw_b + math.pi))
        w = clamp(self.kyaw_approach() * stern_bearing, self.max_yaw_rate())
        if abs(stern_bearing) > turn_th:
            return 0.0, w
        v = -min(v_cap, dist)
        return v, w

    def _compute_back_in(self, x, e_y, e_yaw, speed_cap, yaw_cap):
        p = self.get_parameter
        # 横向/艏向门控：超出门控2 -> v=0 只修艏向
        g2_y = float(p("back_in_gate2_y").value)
        g2_yaw = math.radians(float(p("back_in_gate2_yaw_deg").value))
        if abs(e_y) > g2_y or abs(e_yaw) > g2_yaw:
            v = 0.0
        else:
            dist = float(p("final_target_x").value) - x  # >0 还需继续倒入
            if (
                abs(dist) < float(p("docked_x_tol").value)
                and abs(e_y) < float(p("docked_y_tol").value)
            ):
                # 终局消艏偏：x/y 已达标，原地旋转把 e_yaw 压到 0
                # （解除 e_yaw=-ky·e_y 稳态平衡，该平衡随横向残差必超 3° 判据）
                w = -float(p("kyaw_back").value) * e_yaw
                return 0.0, clamp(w, yaw_cap)
            v = -min(speed_cap, max(0.0, self.kx_final() * dist))
        # 倒船：ω = -kyaw·(e_yaw + ky·e_y)
        w = -float(p("kyaw_back").value) * (
            e_yaw + float(p("ky_back").value) * e_y
        )
        return v, clamp(w, yaw_cap)

    def _compute_exit_forward(self, e_y, e_yaw, speed):
        # 前进：ω = kyaw·(ky·e_y - e_yaw)（复用 approach 增益，同构前向几何）
        turn_th = math.radians(float(self.get_parameter("exit_turn_deg").value))
        w = self.kyaw_approach() * (self.ky_approach() * e_y - e_yaw)
        if abs(e_yaw) > turn_th:
            return 0.0, clamp(w, self.max_yaw_rate())
        return speed, clamp(w, self.max_yaw_rate())

    def _compute_search(self, x, e_y, yaw_b, limited: bool):
        """返回 (v, w)。有预测位姿 -> 闭环绕船尾对坞方位；否则开环扇形扫描。"""
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

        # 无锚点：开环扇形扫描（左右扫描，半角逐步扩大）
        if limited:
            return 0.0, 0.0  # 坞内无预测不做任何动作，等 FSM 裁决
        if self._sweep_ref_yaw is None:
            self._sweep_ref_yaw = yaw_b
        dev = wrap_angle(yaw_b - self._sweep_ref_yaw)
        half = self._sweep_half_angle
        if dev >= half and self._sweep_dir > 0:
            self._sweep_dir = -1.0
        elif dev <= -half and self._sweep_dir < 0:
            self._sweep_dir = 1.0
            # 完成一个来回 -> 扩大扫描半角
            half_max = math.radians(
                float(p("search_max_half_angle_deg").value)
            )
            inc = math.radians(float(p("search_angle_increment_deg").value))
            self._sweep_half_angle = min(half + inc, half_max)
        return 0.0, self._sweep_dir * search_rate

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

        # 模式切换沿：重置搜索内部状态
        if mode != self._last_mode:
            self._sweep_ref_yaw = None
            self._sweep_dir = 1.0
            self._sweep_half_angle = math.radians(
                float(p("search_initial_half_angle_deg").value)
            )
            self._limited_entry_yaw = (
                self._pose_yaw if mode == MODE_SEARCH_LIMITED else None
            )
            self._last_mode = mode

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
            v_target = 0.0
            w_target = clamp(
                -float(p("kyaw_align").value) * e_yaw, self.max_yaw_rate()
            )
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
    node = DockingMotionControllerV2()
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
