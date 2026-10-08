#!/usr/bin/env python3
"""速度桥水上标定动作序列执行器（船端本地运行，ssh 断开不影响）。

用法:
  python3 calib_seq.py <phase>

phase:
  baseline    静止 15s（GPS 噪声/漂移基线，编排脚本已放到最后执行）
  rot_nohold  原地旋转，位置保持 OFF：wz=±[0.10..0.35]，各 15s
  rot_hold    原地旋转，位置保持 ON： wz=±[0.15,0.25,0.35]，各 15s
  line        直线：vx=[0.2..1.0]，各 15s；每个动作前先转到朝北，过程中小幅航向保持
  env_verify  速度包络 ON：按 LUT 原始边值发 vx，验证桥限幅后的目标可达；动作前先朝北
  env_probe   速度包络 OFF：vx 发到 LUT 边值的 1.15 倍，探测真实饱和边界；动作前先朝北

安全:
  - 每个动作前要求 armed + GUIDED（不在 GUIDED 会自助切模式）；
  - 动作中模式离开 GUIDED（遥控器接管）→ 立即零速并暂停，切回 GUIDED 后自动重跑当前动作；
    暂停超过 PAUSE_TIMEOUT_S 则中止整个序列；
  - 距序列起点 > MAX_RANGE_M 或 odom 中断 > 2s 立即硬中止；
  - 每个指令动作之间发 6s 零速。
退出码: 0=完成 2=安全中止 3=前置条件不满足
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry

CMD_TOPIC = "/cmd_vel_nav"
STATE_TOPIC = "/mavros/state"
ODOM_TOPIC = "/mavros/local_position/odom"
SET_MODE_SRV = "/mavros/set_mode"

PUB_DT = 0.1
SETTLE_S = 6.0
MAX_RANGE_M = 80.0
ODOM_TIMEOUT_S = 2.0
GUIDED_WAIT_S = 20.0
PAUSE_TIMEOUT_S = 300.0

# ENU 系朝西的 yaw（odom 为 ENU：0=东，+90°=北，±180°=西）
WEST_YAW = math.pi
ROTATE_TOL = math.radians(6.0)
ROTATE_TIMEOUT_S = 40.0
ROTATE_KP = 1.5
ROTATE_MAX_WZ = 0.35
HEADING_HOLD_KP = 1.2
HEADING_HOLD_MAX_WZ = 0.25

# 桥内 LUT 原始边值（未乘 0.9 安全系数），与源码 positive/negative_envelope_ 对应
LUT_POS = {0.05: 1.04, 0.10: 0.91, 0.15: 0.84, 0.20: 0.74, 0.25: 0.65, 0.30: 0.60, 0.35: 0.54}
LUT_NEG = {0.05: 0.92, 0.10: 0.88, 0.15: 0.75, 0.20: 0.67, 0.25: 0.59, 0.28: 0.47}


def rotation_phase(wz_list, tag):
    seq = []
    for wz in wz_list:
        for sgn in (1.0, -1.0):
            seq.append(("cmd", f"rot_{tag}_{sgn * wz:+.2f}", 0.0, sgn * wz, 15.0, False))
    return seq


def west_then(step):
    return [("rotate", WEST_YAW), step]


def build_phase(name):
    if name == "baseline":
        return [("cmd", "baseline", 0.0, 0.0, 15.0, False)]
    if name == "rot_nohold":
        return rotation_phase([0.10, 0.15, 0.20, 0.25, 0.30, 0.35], "nohold")
    if name == "rot_hold":
        return rotation_phase([0.15, 0.25, 0.35], "hold")
    if name == "line":
        seq = []
        for vx in (0.2, 0.4, 0.6, 0.8, 1.0):
            seq += west_then(("cmd", f"line_vx_{vx:.1f}", vx, 0.0, 15.0, True))
        return seq
    if name == "env_verify":
        seq = []
        for sgn, lut, wzs in ((1.0, LUT_POS, (0.10, 0.20, 0.30)),
                              (-1.0, LUT_NEG, (0.10, 0.20, 0.28))):
            for wz in wzs:
                seq += west_then(
                    ("cmd", f"envv_{sgn * wz:+.2f}_vx{lut[wz]:.2f}", lut[wz], sgn * wz, 15.0, False))
        return seq
    if name == "env_probe":
        seq = []
        for sgn, lut, wzs in ((1.0, LUT_POS, (0.05, 0.15, 0.25, 0.35)),
                              (-1.0, LUT_NEG, (0.05, 0.15, 0.25, 0.28))):
            for wz in wzs:
                vx = min(lut[wz] * 1.15, 1.5)
                seq += west_then(
                    ("cmd", f"envp_{sgn * wz:+.2f}_vx{vx:.2f}", vx, sgn * wz, 15.0, False))
        return seq
    return None


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


class CalibSeq(Node):
    def __init__(self):
        super().__init__("bridge_calib_seq")
        self.state = State()
        self.odom = None
        self.yaw = 0.0
        self.odom_time = 0.0
        self.origin = None
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(State, STATE_TOPIC, self.state_cb, 10)
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, be)
        self.set_mode_cli = self.create_client(SetMode, SET_MODE_SRV)

    def state_cb(self, msg):
        self.state = msg

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.origin is None:
            self.origin = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def log(self, text):
        print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    def range_ok(self):
        if self.odom is None or self.origin is None:
            return True
        dx = self.odom.pose.pose.position.x - self.origin[0]
        dy = self.odom.pose.pose.position.y - self.origin[1]
        return (dx * dx + dy * dy) ** 0.5 <= MAX_RANGE_M

    def odom_fresh(self):
        return self.odom_time > 0.0 and (time.monotonic() - self.odom_time) <= ODOM_TIMEOUT_S

    def hard_abort_reason(self):
        if not self.range_ok():
            return f"超出 {MAX_RANGE_M}m 活动范围"
        if not self.odom_fresh():
            return "odom 中断"
        return None

    def guided(self):
        return self.state.mode == "GUIDED" and self.state.armed

    def ensure_guided(self):
        if self.guided():
            return True
        if not self.set_mode_cli.wait_for_service(timeout_sec=5.0):
            self.log("!! set_mode 服务不可用")
            return False
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        self.set_mode_cli.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < GUIDED_WAIT_S:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.guided():
                return True
        self.log(f"!! {GUIDED_WAIT_S}s 内未进入 GUIDED+armed（当前 {self.state.mode}）")
        return False

    def wait_guided_resume(self):
        """遥控接管后等待切回 GUIDED；超时或硬中止返回 False。"""
        self.log(f"已暂停，等待切回 GUIDED（最长 {PAUSE_TIMEOUT_S:.0f}s）...")
        t0 = time.monotonic()
        tried_set_mode = False
        while time.monotonic() - t0 < PAUSE_TIMEOUT_S:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.publish_cmd(0.0, 0.0)
            reason = self.hard_abort_reason()
            if reason:
                self.log(f"!! 暂停期间{reason}，硬中止")
                return False
            if self.guided():
                self.log("已回到 GUIDED，继续")
                return True
            if not tried_set_mode and time.monotonic() - t0 > 10.0:
                tried_set_mode = True
                self.ensure_guided()
            time.sleep(PUB_DT)
        self.log("!! 暂停超时，中止")
        return False

    def wait_ready(self, timeout_s=60.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.state.connected and self.odom_fresh():
                return True
        self.log("!! 60s 内飞控未连接或 odom 未就绪（GPS/RTK 定位正常?）")
        return False

    def exec_cmd(self, name, vx, wz, dur, hold_heading):
        """执行一个指令动作。返回 done / pause / abort。"""
        self.log(f"== {name}: vx={vx:.2f} wz={wz:+.2f} {dur:.0f}s"
                 + ("（航向保持朝西）" if hold_heading else ""))
        if not self.ensure_guided():
            return "pause"
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.0)
            reason = self.hard_abort_reason()
            if reason:
                self.log(f"!! {reason}，硬中止")
                return "abort"
            if not self.guided():
                self.log(f"模式变为 {self.state.mode}（遥控接管），动作 {name} 作废")
                return "pause"
            cmd_wz = wz
            if hold_heading:
                cmd_wz = clamp(HEADING_HOLD_KP * wrap_pi(WEST_YAW - self.yaw),
                               -HEADING_HOLD_MAX_WZ, HEADING_HOLD_MAX_WZ)
            self.publish_cmd(vx, cmd_wz)
            time.sleep(PUB_DT)
        self.log(f"-- {name} 完成，零速稳定 {SETTLE_S:.0f}s")
        return "done"

    def exec_rotate(self, target_yaw):
        """原地转到目标航向。返回 done / pause / abort。"""
        self.log(f"== 转向朝西（目标 {math.degrees(target_yaw):.0f}deg ENU，"
                 f"当前 {math.degrees(self.yaw):.0f}deg）")
        if not self.ensure_guided():
            return "pause"
        t0 = time.monotonic()
        while time.monotonic() - t0 < ROTATE_TIMEOUT_S:
            rclpy.spin_once(self, timeout_sec=0.0)
            reason = self.hard_abort_reason()
            if reason:
                self.log(f"!! {reason}，硬中止")
                return "abort"
            if not self.guided():
                self.log(f"模式变为 {self.state.mode}（遥控接管），转向作废")
                return "pause"
            err = wrap_pi(target_yaw - self.yaw)
            if abs(err) < ROTATE_TOL:
                break
            self.publish_cmd(0.0, clamp(ROTATE_KP * err, -ROTATE_MAX_WZ, ROTATE_MAX_WZ))
            time.sleep(PUB_DT)
        else:
            self.log("!! 转向超时，按当前航向继续")
        self.publish_cmd(0.0, 0.0)
        self.log(f"-- 转向完成（当前 {math.degrees(self.yaw):.0f}deg）")
        return "done"

    def run(self, seq):
        if not self.wait_ready():
            return 3
        for step in seq:
            while True:  # 遥控接管 → 暂停 → 切回 GUIDED 后重跑当前步骤
                if step[0] == "rotate":
                    rc = self.exec_rotate(step[1])
                else:
                    _, name, vx, wz, dur, hold_heading = step
                    rc = self.exec_cmd(name, vx, wz, dur, hold_heading)
                if rc == "done":
                    break
                self.publish_zero(1.0)
                if rc == "abort":
                    return 2
                if not self.wait_guided_resume():
                    self.publish_zero(1.0)
                    return 2
            if step[0] == "cmd":
                self.publish_zero(SETTLE_S)
        self.log("序列全部完成")
        return 0

    def publish_zero(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.publish_cmd(0.0, 0.0)
            time.sleep(PUB_DT)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    seq = build_phase(sys.argv[1])
    if seq is None:
        print(f"未知 phase: {sys.argv[1]}")
        print(__doc__)
        return 1
    rclpy.init()
    node = CalibSeq()
    try:
        code = node.run(seq)
    except KeyboardInterrupt:
        code = 2
    finally:
        node.publish_zero(1.0)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
