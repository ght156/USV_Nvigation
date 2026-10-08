#!/usr/bin/env python3
"""转向阶跃试验分析：把 cmd_vel_nav → 桥输出 → 飞控 PWM → IMU 四段两两对齐。

输出每段互相关峰值延迟（正=后段滞后前段）与阶跃响应（10%/90% 到达时间）。
"""

import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WANT = [
    "/cmd_vel_nav",
    "/mavros/setpoint_velocity/cmd_vel_unstamped",
    "/mavros/rc/out",
    "/mavros/imu/data",
]


def load(bag):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {t: get_message(types[t]) for t in WANT if t in types}
    miss = [t for t in WANT if t not in types]
    if miss:
        print("bag 缺话题:", miss)
    data = {t: [] for t in cls}
    while r.has_next():
        topic, raw, ts = r.read_next()
        if topic not in cls:
            continue
        m = deserialize_message(raw, cls[topic])
        t = ts * 1e-9
        if topic == "/mavros/rc/out":
            data[topic].append((t, np.array(m.channels, dtype=float)))
        elif topic == "/mavros/imu/data":
            data[topic].append((t, float(m.angular_velocity.z)))
        else:
            data[topic].append((t, float(m.angular.z)))
    for t, v in data.items():
        print(f"  {t}: {len(v)} 条")
    return data


def main():
    data = load(sys.argv[1])
    t0 = min(v[0][0] for v in data.values() if v)
    t1 = max(v[-1][0] for v in data.values() if v)
    print(f"\n时长 {t1 - t0:.1f}s")

    # rc/out：找出变化最大的两个通道，用其差作为"差速推力"代理
    rc = data.get("/mavros/rc/out") or []
    use_rc = len(rc) > 0
    if use_rc:
        arr = np.array([v for _, v in rc])
        span = arr.max(axis=0) - arr.min(axis=0)
        order = np.argsort(span)[::-1]
        print("rc/out 通道变化幅度 top6:",
              [(int(i), int(span[i])) for i in order[:6] if span[i] > 0])
        a, b = int(order[0]), int(order[1])
        print(f"用通道 {a} - 通道 {b} 作为差速推力代理"
              f"（跨度 {span[a]:.0f}/{span[b]:.0f}）")
    else:
        print("⚠ 本 bag 没有 /mavros/rc/out → 无法把「链路+飞控」再细分")

    # 统一重采样到 50Hz（保持）
    fs = 50.0
    grid = np.arange(t0, t1, 1.0 / fs)

    def resample(seq, idx=None):
        ts = np.array([t for t, _ in seq])
        if idx is None:
            vs = np.array([v for _, v in seq], dtype=float)
        else:
            vs = np.array([v[idx] for _, v in seq], dtype=float)
        # 每列可能长度不一（channels 数变化）→ 这里只用于等长情形
        return np.interp(grid, ts, vs)

    cmd = resample(data["/cmd_vel_nav"])
    sp = resample(data["/mavros/setpoint_velocity/cmd_vel_unstamped"])
    if use_rc:
        dif = np.array([v[a] - v[b] for _, v in rc])
        rc_ts = np.array([t for t, _ in rc])
        rcser = np.interp(grid, rc_ts, dif)
        rcser = rcser - rcser.mean()
    wz = resample(data["/mavros/imu/data"])

    def lag(x, y, maxlag=2.0):
        """返回 (lag_sec, corr)：y 相对 x 的延迟（正=y 滞后）。"""
        n = int(maxlag * fs)
        x = x - x.mean()
        y = y - y.mean()
        best = (0.0, 0.0)
        for k in range(0, n):
            a1, b1 = x[: len(x) - k], y[k:]
            if np.std(a1) < 1e-9 or np.std(b1) < 1e-9:
                continue
            c = float(np.corrcoef(a1, b1)[0, 1])
            if abs(c) > abs(best[1]):
                best = (k / fs, c)
        c0 = float(np.corrcoef(x, y)[0, 1])
        return best, c0

    print("\n=== 各段滞后（互相关峰值；括号内为 0 延迟相关）===")
    pairs = [
        ("cmd_vel_nav  → 桥输出 setpoint", cmd, sp),
        ("cmd_vel_nav  → IMU 角速度（全程）", cmd, wz),
        ("桥输出       → IMU 角速度", sp, wz),
    ]
    if use_rc:
        pairs.insert(1, ("桥输出       → 飞控 rc/out", sp, rcser))
        pairs.insert(2, ("飞控 rc/out  → IMU 角速度", rcser, wz))
    for name, x, y in pairs:
        (lg, c), c0 = lag(x, y)
        print(f"  {name:<34} lag={lg:+.2f}s  corr={c:+.2f}   (lag0={c0:+.2f})")

    # 阶跃响应：cmd 由 0 跳到 ±0.12 的时刻，看各段何时到 10%/90%
    print("\n=== 阶跃响应（cmd 跳变 → 各段到 10% / 90%）===")
    step_idx = [
        i for i in range(1, len(cmd))
        if abs(cmd[i]) > 0.1 and abs(cmd[i - 1]) < 0.05
    ]
    print(f"  检测到 {len(step_idx)} 个正向跳变")
    for i in step_idx[:4]:
        t_step = grid[i]
        tgt = cmd[i]
        row = [f"    t={t_step - t0:5.1f}s → cmd={tgt:+.2f}"]
        sigs = [("桥", sp)] + ([("PWM", rcser)] if use_rc else []) + [("IMU", wz)]
        for nm, sig in sigs:
            seg = sig[i: i + int(2.0 * fs)]
            if np.std(seg) < 1e-9:
                row.append(f"{nm}:无响应")
                continue
            base = seg[: int(0.1 * fs)].mean()
            # 用 |变化| 归一，找 10%/90% 时刻
            d = seg - base
            sgn = np.sign(d[np.argmax(np.abs(d))])
            dd = d * sgn
            mx = dd.max()
            if mx <= 0:
                row.append(f"{nm}:无响应")
                continue
            t10 = np.argmax(dd > 0.1 * mx) / fs
            t90 = np.argmax(dd > 0.9 * mx) / fs
            row.append(f"{nm}: 10%@{t10:.2f}s 90%@{t90:.2f}s")
        print("  ".join(row))


if __name__ == "__main__":
    main()
