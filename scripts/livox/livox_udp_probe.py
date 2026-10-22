#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Livox Mid-360S 雷达时基只读探针（被动收 UDP，不握手、不改雷达任何配置）。

用途：在船端量「雷达点云时间戳 vs 主机墙钟」的偏差（是否超前/落后多少）。
判读：
    offset = 雷达包内 timestamp - 收到时的主机墙钟
    offset > 0 = 雷达钟超前主机（10-14/10-15 那单的形态）
    |offset| < 0.02 s 且 time_type=2 = GPS 授时健康
    time_type=0 = 雷达处于无同步自由振荡（拔线后形态）

用法（船端）：
    python3 /tmp/livox_udp_probe.py            # 收 8 秒
    python3 /tmp/livox_udp_probe.py -s 20      # 收 20 秒
"""

from __future__ import annotations

import argparse
import socket
import struct
import time

HEADER_SIZE = 36
POINT_PORT = 56301         # 点云目的端（驱动配置里的 point_data_port）
HANDSHAKE_PORT = 56000     # 雷达 1 Hz 握手广播 255.255.255.255:56000
TIME_TYPE = {0: "NoSync(无同步)", 1: "PTP", 2: "GPS"}


def parse_header(data: bytes) -> dict | None:
    if len(data) < HEADER_SIZE:
        return None
    (version, length, time_interval, dot_num, udp_cnt,
     frame_cnt, data_type, time_type) = struct.unpack_from("<BHHHHBBB", data, 0)
    ts_ns = struct.unpack_from("<Q", data, 28)[0]
    return {
        "length": length, "dot_num": dot_num, "udp_cnt": udp_cnt,
        "frame_cnt": frame_cnt, "data_type": data_type,
        "time_type": time_type, "ts_ns": ts_ns,
    }


def make_sock(port: int, broadcast: bool = False) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if broadcast:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("0.0.0.0", port))
    s.setblocking(False)
    return s


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--seconds", type=float, default=8.0)
    ap.add_argument("--print-every", type=float, default=1.0)
    args = ap.parse_args()

    socks = {}
    for name, port, br in (("point", POINT_PORT, False), ("handshake", HANDSHAKE_PORT, True)):
        try:
            socks[name] = make_sock(port, br)
        except OSError as exc:
            print(f"[warn] 绑定 {port} 失败: {exc}（该端口可能被驱动占用）")
    if not socks:
        print("两个端口都绑不上，退出。")
        return

    print(f"监听中：point={POINT_PORT} handshake={HANDSHAKE_PORT}，共 {args.seconds:.0f} s")
    print(f"主机墙钟 = {time.strftime('%F %T')} (unix {time.time():.3f})")

    stats = {n: 0 for n in socks}
    offsets: list[float] = []
    last_hm: float | None = None
    last_ts: float | None = None
    t_end = time.time() + args.seconds
    next_print = time.time() + args.print_every

    while time.time() < t_end:
        got = False
        for name, s in socks.items():
            while True:
                try:
                    data, addr = s.recvfrom(65535)
                except BlockingIOError:
                    break
                except OSError:
                    break
                got = True
                stats[name] += 1
                if name == "handshake":
                    last_hm = time.time()
                    continue
                hdr = parse_header(data)
                if hdr is None:
                    continue
                now = time.time()
                ts = hdr["ts_ns"] * 1e-9
                offsets.append(ts - now)
                if last_ts is not None and hdr["udp_cnt"] % 100 == 0:
                    pass
                last_ts = ts
                if len(offsets) == 1:
                    print(f"首包: len={hdr['length']} dot_num={hdr['dot_num']} "
                          f"data_type={hdr['data_type']} time_type={hdr['time_type']}"
                          f"({TIME_TYPE.get(hdr['time_type'], '?')}) "
                          f"frame={hdr['frame_cnt']} offset={offsets[-1]:+.6f} s")
        if time.time() >= next_print:
            n = len(offsets)
            if n:
                srt = sorted(offsets[-4000:])
                med = srt[len(srt) // 2]
                print(f"  t+{time.time() - (t_end - args.seconds):4.1f}s  "
                      f"point_pkts={stats['point']:6d} handshake={stats['handshake']:3d}  "
                      f"offset(med of last)={med:+.4f} s")
            else:
                print(f"  t+{time.time() - (t_end - args.seconds):4.1f}s  "
                      f"未收到点云包 point_pkts=0 handshake={stats['handshake']}")
            next_print += args.print_every
        if not got:
            time.sleep(0.02)

    print("\n===== 结果 =====")
    print(f"点云包: {stats['point']}  握手包: {stats['handshake']}")
    if offsets:
        srt = sorted(offsets)
        n = len(srt)
        mean = sum(srt) / n
        print(f"offset(雷达ts - 主机墙钟): n={n} mean={mean:+.4f} s "
              f"med={srt[n // 2]:+.4f} s min={srt[0]:+.4f} max={srt[-1]:+.4f} "
              f"spread={(srt[-1] - srt[0]) * 1e3:.1f} ms")
        verdict = "超前" if mean > 0.02 else ("落后" if mean < -0.02 else "与主机一致(±20ms 内)")
        print(f">>> 判读: 雷达时间戳{verdict}主机；{mean:+.4f} s")
    else:
        print("未收到点云包：雷达可能处于「等主机」状态（只有握手广播），"
              "或点云目的端不是本机 %d 端口。握手包时间见上。" % POINT_PORT)
        if last_hm:
            print(f"    最近一次握手广播: 收到于 {time.strftime('%T', time.localtime(last_hm))}")


if __name__ == "__main__":
    main()
