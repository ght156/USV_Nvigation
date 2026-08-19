#!/usr/bin/env python3
"""
MID360 UDP 原始数据解析脚本

从 SDK 源码确认的协议结构:
  LivoxLidarEthernetPacket 包头 (36 bytes):
     0:  version      (uint8)
     1:  length       (uint16)   总包长
     3:  time_interval(uint16)   单位 0.1us
     5:  dot_num      (uint16)   本包点数
     7:  udp_cnt      (uint16)   UDP 序号
     9:  frame_cnt    (uint8)    帧序号
    10:  data_type    (uint8)    0=IMU 1=CartesianHigh 2=CartesianLow 3=Spherical
    11:  time_type    (uint8)    0=NoSync 1=PTP 2=GPS
    12:  rsvd[12]
    24:  crc32        (uint32)
    28:  timestamp[8] (uint64)   ns
    36:  data[]

  CartesianHigh 单点 (14 bytes):
     0: x  (int32, mm)
     4: y  (int32, mm)
     8: z  (int32, mm)
    12: reflectivity (uint8)
    13: tag (uint8)

数据流:
  雷达推送点云 -> host UDP:56301 (配置中 point_data_port)
  雷达发送端口 56300
"""

import struct
import sys
import os
import socket
import datetime
import math

# 协议常量
HEADER_SIZE = 36
CART_HIGH_PT_SIZE = 14   # CartesianHigh (int32*3 + uint8*2)
CART_LOW_PT_SIZE = 10    # CartesianLow  (int16*3 + uint8*2)
SPHERE_PT_SIZE = 10      # Spherical     (uint32 + uint16*2 + uint8*2)
IMU_DATA_SIZE = 24       # float32 * 6

DATA_TYPE_NAMES = {0: "IMU", 1: "CartesianHigh", 2: "CartesianLow", 3: "Spherical"}
TIME_TYPE_NAMES = {0: "NoSync ❌", 1: "PTP/gPTP ⚡", 2: "GPS ✅"}

# MID360 默认参数
LIDAR_IP = "192.168.2.191"
POINT_PORT = 56300
HOST_POINT_PORT = 56301


def parse_header(data: bytes) -> dict:
    """解析 36 字节 LivoxLidarEthernetPacket 包头"""
    if len(data) < HEADER_SIZE:
        return None
    (version, length, time_interval, dot_num, udp_cnt,
     frame_cnt, data_type, time_type) = struct.unpack_from('<BHHHHBBB', data, 0)
    rsvd = data[12:24]
    crc32_val = struct.unpack_from('<I', data, 24)[0]
    timestamp_ns = struct.unpack_from('<Q', data, 28)[0]
    return {
        'version': version,
        'length': length,
        'payload_len': len(data) - HEADER_SIZE,
        'time_interval_01us': time_interval,
        'dot_num': dot_num,
        'udp_cnt': udp_cnt,
        'frame_cnt': frame_cnt,
        'data_type': data_type,
        'data_type_name': DATA_TYPE_NAMES.get(data_type, f"未知({data_type})"),
        'time_type': time_type,
        'time_type_name': TIME_TYPE_NAMES.get(time_type, f"未知({time_type})"),
        'crc32': crc32_val,
        'timestamp_ns': timestamp_ns,
    }


def parse_cartesian_high(data: bytes, dot_num: int, base_ts_ns: int, interval_01us: int) -> list:
    """解析 CartesianHigh 格式点云"""
    points = []
    interval_ns = interval_01us * 100  # 0.1us → ns
    for i in range(dot_num):
        off = HEADER_SIZE + i * CART_HIGH_PT_SIZE
        if off + CART_HIGH_PT_SIZE > len(data):
            break
        x, y, z, refl, tag = struct.unpack_from('<iiiBB', data, off)
        points.append({
            'idx': i, 'x': x / 1000.0, 'y': y / 1000.0, 'z': z / 1000.0,
            'x_mm': x, 'y_mm': y, 'z_mm': z,
            'reflectivity': refl, 'tag': tag,
            'ts_ns': base_ts_ns + i * interval_ns,
        })
    return points


def parse_cartesian_low(data: bytes, dot_num: int, base_ts_ns: int, interval_01us: int) -> list:
    """解析 CartesianLow 格式点云"""
    points = []
    interval_ns = interval_01us * 100
    for i in range(dot_num):
        off = HEADER_SIZE + i * CART_LOW_PT_SIZE
        if off + CART_LOW_PT_SIZE > len(data):
            break
        x, y, z, refl, tag = struct.unpack_from('<hhhBB', data, off)
        points.append({
            'idx': i, 'x': x / 100.0, 'y': y / 100.0, 'z': z / 100.0,
            'x_cm': x, 'y_cm': y, 'z_cm': z,
            'reflectivity': refl, 'tag': tag,
            'ts_ns': base_ts_ns + i * interval_ns,
        })
    return points


def parse_spherical(data: bytes, dot_num: int, base_ts_ns: int, interval_01us: int) -> list:
    """解析 Spherical 格式点云"""
    points = []
    interval_ns = interval_01us * 100
    for i in range(dot_num):
        off = HEADER_SIZE + i * SPHERE_PT_SIZE
        if off + SPHERE_PT_SIZE > len(data):
            break
        depth, theta, phi, refl, tag = struct.unpack_from('<IHHBB', data, off)
        # theta/phi 单位 0.01°
        theta_rad = theta / 100.0 * math.pi / 180.0
        phi_rad = phi / 100.0 * math.pi / 180.0
        r = depth / 1000.0  # mm → m
        x_m = r * math.sin(theta_rad) * math.cos(phi_rad)
        y_m = r * math.sin(theta_rad) * math.sin(phi_rad)
        z_m = r * math.cos(theta_rad)
        points.append({
            'idx': i, 'x': x_m, 'y': y_m, 'z': z_m,
            'depth_mm': depth, 'theta_001deg': theta, 'phi_001deg': phi,
            'reflectivity': refl, 'tag': tag,
            'ts_ns': base_ts_ns + i * interval_ns,
        })
    return points


def parse_imu(data: bytes) -> dict:
    """解析 IMU 数据 (data_type=0)"""
    off = HEADER_SIZE
    if off + IMU_DATA_SIZE > len(data):
        return None
    gx, gy, gz, ax, ay, az = struct.unpack_from('<ffffff', data, off)
    return {'gyro': (gx, gy, gz), 'acc': (ax, ay, az)}


def print_header(hdr: dict):
    """打印包头信息"""
    print(f"  Version:        {hdr['version']}")
    print(f"  Length:         {hdr['length']} bytes (payload {hdr['payload_len']} bytes)")
    print(f"  Time interval:  {hdr['time_interval_01us']} * 0.1us = {hdr['time_interval_01us'] * 0.1:.1f} us")
    print(f"  Dot num:        {hdr['dot_num']}")
    print(f"  UDP cnt:        {hdr['udp_cnt']}")
    print(f"  Frame cnt:      {hdr['frame_cnt']}")
    print(f"  Data type:      {hdr['data_type']} ({hdr['data_type_name']})")
    print(f"  ⭐ Time type:   {hdr['time_type']} ({hdr['time_type_name']})")
    print(f"  CRC32:          0x{hdr['crc32']:08X}")

    # 时间戳解析
    ts_ns = hdr['timestamp_ns']
    print(f"  Timestamp:      {ts_ns} ns")
    if hdr['time_type'] == 2 and ts_ns > 1000000000000:  # GPS 同步且值合理
        gps_epoch = datetime.datetime(1980, 1, 6)
        try:
            gps_time = gps_epoch + datetime.timedelta(microseconds=ts_ns / 1000.0)
            unix_time = ts_ns / 1e9 + 315964800  # GPS → Unix 偏移
            print(f"  Timestamp (GPS): {gps_time.strftime('%Y-%m-%d %H:%M:%S.%f')}")
            print(f"  Timestamp (Unix):{datetime.datetime.fromtimestamp(unix_time).strftime('%Y-%m-%d %H:%M:%S.%f')}")
        except Exception:
            pass
    elif ts_ns > 0:
        try:
            unix_sec = ts_ns / 1e9
            if unix_sec > 1e9:  # 合理 Unix 时间戳
                print(f"  Timestamp (Unix):{datetime.datetime.fromtimestamp(unix_sec).strftime('%Y-%m-%d %H:%M:%S.%f')}")
            else:
                print(f"  Timestamp (rel):  {ts_ns / 1e9:.6f} s")
        except Exception:
            pass


def print_points(points: list, max_show: int = 20):
    """打印点云数据"""
    if not points:
        return
    print(f"  点云: {len(points)} 个点")
    print(f"  {'Idx':>4s} {'X(m)':>10s} {'Y(m)':>10s} {'Z(m)':>10s} {'Refl':>5s} {'Tag':>4s}")
    print(f"  {'-'*4} {'-'*10} {'-'*10} {'-'*10} {'-'*5} {'-'*4}")
    for p in points[:max_show]:
        print(f"  {p['idx']:4d} {p['x']:10.3f} {p['y']:10.3f} {p['z']:10.3f} {p['reflectivity']:5d} {p['tag']:4d}")
    if len(points) > max_show:
        print(f"  ... ({len(points) - max_show} 个点未显示)")

    # 统计
    xs = [p['x'] for p in points]
    ys = [p['y'] for p in points]
    zs = [p['z'] for p in points]
    refs = [p['reflectivity'] for p in points]
    print(f"  统计: X [{min(xs):.3f}, {max(xs):.3f}]m, "
          f"Y [{min(ys):.3f}, {max(ys):.3f}]m, "
          f"Z [{min(zs):.3f}, {max(zs):.3f}]m")
    print(f"  反射率: [{min(refs)}, {max(refs)}]")


def parse_one_packet(data: bytes, show_points: bool = True, max_points: int = 20):
    """解析一个 UDP 包并打印"""
    if len(data) < HEADER_SIZE:
        print(f"  [跳过] 包太小 ({len(data)} < {HEADER_SIZE})")
        return None

    hdr = parse_header(data)
    print("\n" + "=" * 70)
    print("=== MID360 数据包 ===")
    print_header(hdr)

    # 解析数据部分
    if hdr['data_type'] == 0:  # IMU
        imu = parse_imu(data)
        if imu:
            gx, gy, gz = imu['gyro']
            ax, ay, az = imu['acc']
            print(f"  IMU: gyro=({gx:.4f}, {gy:.4f}, {gz:.4f}) rad/s")
            print(f"       acc =({ax:.4f}, {ay:.4f}, {az:.4f}) m/s²")

    elif hdr['data_type'] == 1:  # CartesianHigh (MID360 默认)
        pts = parse_cartesian_high(data, hdr['dot_num'],
                                    hdr['timestamp_ns'], hdr['time_interval_01us'])
        if show_points:
            print_points(pts, max_points)

    elif hdr['data_type'] == 2:  # CartesianLow
        pts = parse_cartesian_low(data, hdr['dot_num'],
                                   hdr['timestamp_ns'], hdr['time_interval_01us'])
        if show_points:
            print_points(pts, max_points)

    elif hdr['data_type'] == 3:  # Spherical
        pts = parse_spherical(data, hdr['dot_num'],
                               hdr['timestamp_ns'], hdr['time_interval_01us'])
        if show_points:
            print(f"  点云 (Spherical): {len(pts)} 个点")
            for p in pts[:5]:
                print(f"  [{p['idx']:4d}] depth={p['depth_mm']}mm "
                      f"θ={p['theta_001deg']/100:.2f}° φ={p['phi_001deg']/100:.2f}° "
                      f"→ ({p['x']:.3f},{p['y']:.3f},{p['z']:.3f})m refl={p['reflectivity']}")
    else:
        print(f"  [未知] data_type={hdr['data_type']}, payload={data[HEADER_SIZE:].hex()[:64]}...")

    return hdr


def live_capture(count: int = 10, timeout: int = 10, show_points: bool = True):
    """实时捕获解析"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    sock.bind(('0.0.0.0', HOST_POINT_PORT))

    print(f"[Live] 监听 UDP:{HOST_POINT_PORT} ← {LIDAR_IP}:{POINT_PORT}")
    print(f"[Live] 共捕获 {count} 个包...\n")

    packets = []
    for i in range(count):
        try:
            data, addr = sock.recvfrom(2000)
            packets.append(data)
        except socket.timeout:
            print(f"\n[超时] {timeout}s 内未收到第 {i+1} 个包，已收到 {len(packets)} 个")
            break
        except Exception as e:
            print(f"\n[错误] 解析第 {i+1} 个包: {e}")

    sock.close()
    return packets


def read_pcap(filepath: str) -> list:
    """解析 pcap 文件，提取 UDP payload"""
    with open(filepath, 'rb') as f:
        raw = f.read()

    if len(raw) < 24:
        print("文件太小，不是有效的 pcap")
        return []

    magic = struct.unpack_from('<I', raw, 0)[0]
    if magic == 0xa1b2c3d4:
        endian = '<'
    elif magic == 0xd4c3b2a1:
        endian = '>'
    else:
        print(f"未知 pcap 标识: 0x{magic:08X}")
        return []

    payloads = []
    off = 24
    while off + 16 <= len(raw):
        ts_sec, ts_usec, incl_len, _orig_len = struct.unpack_from(f'{endian}IIII', raw, off)
        off += 16
        if off + incl_len > len(raw):
            break
        pkt = raw[off:off + incl_len]
        off += incl_len

        # Ethernet II
        if len(pkt) < 14:
            continue
        eth_type = struct.unpack_from('!H', pkt, 12)[0]
        if eth_type != 0x0800:  # IPv4 only
            continue

        ip_start = 14
        ip_ihl = (pkt[ip_start] & 0x0F) * 4
        if len(pkt) < ip_start + ip_ihl + 8:
            continue
        proto = pkt[ip_start + 9]
        if proto != 17:  # UDP
            continue

        udp_start = ip_start + ip_ihl
        udp_len = struct.unpack_from('!H', pkt, udp_start + 4)[0]
        payload_start = udp_start + 8
        payload = pkt[payload_start:payload_start + udp_len - 8]
        payloads.append(payload)

    return payloads


def print_summary(all_hdrs: list):
    """打印汇总信息"""
    if not all_hdrs:
        return
    print("\n" + "=" * 70)
    print("=== 汇总 ===")
    print(f"  总包数: {len(all_hdrs)}")
    tt_counts = {}
    dt_counts = {}
    for h in all_hdrs:
        tt_counts[h['time_type']] = tt_counts.get(h['time_type'], 0) + 1
        dt_counts[h['data_type_name']] = dt_counts.get(h['data_type_name'], 0) + 1

    print(f"  Data type 分布: {dt_counts}")
    for tt, cnt in sorted(tt_counts.items()):
        name = TIME_TYPE_NAMES.get(tt, f"未知({tt})")
        print(f"  time_type={tt} ({name}): {cnt}/{len(all_hdrs)} 包")

    gps = tt_counts.get(2, 0)
    nosync = tt_counts.get(0, 0)
    if gps > 0:
        print(f"  ✅ GPS 同步有效 ({gps}/{len(all_hdrs)} 包)")
    else:
        print(f"  ❌ 未检测到 GPS 同步 (所有包 time_type=0)")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="MID360 UDP 原始数据解析器")
    parser.add_argument('source', nargs='?', default=None,
                        help='pcap 文件路径，或 "-" 读取 stdin hex，默认使用实时捕获')
    parser.add_argument('-n', '--count', type=int, default=10,
                        help='实时捕获的包数量 (默认 10)')
    parser.add_argument('-t', '--timeout', type=int, default=10,
                        help='实时捕获超时秒数 (默认 10)')
    parser.add_argument('--no-points', action='store_true',
                        help='不打印点云细节')
    parser.add_argument('--max-points', type=int, default=20,
                        help='每包最多显示点数 (默认 20)')

    args = parser.parse_args()

    packets = []
    all_hdrs = []

    if args.source and args.source != '-':
        # 从 pcap 文件读取
        if not os.path.exists(args.source):
            print(f"文件不存在: {args.source}")
            sys.exit(1)
        print(f"[文件] 读取 pcap: {args.source}")
        packets = read_pcap(args.source)
        print(f"[文件] 提取 {len(packets)} 个 UDP payload\n")
    elif args.source == '-':
        # 从 stdin 读取 hex (配合 tcpdump -X)
        print("[stdin] 读取 hex dump...")
        hex_lines = sys.stdin.read()
        raw_bytes = bytes.fromhex(hex_lines.replace(' ', '').replace('\n', ''))
        if raw_bytes:
            packets = [raw_bytes]
        else:
            print("未读取到有效 hex 数据")
            sys.exit(1)
    else:
        # 实时捕获
        print(f"[Live] 实时捕获 {args.count} 个包到内存，开始解析...")
        packets = live_capture(args.count, args.timeout, not args.no_points)

    for pkt in packets:
        hdr = parse_one_packet(pkt, not args.no_points, args.max_points)
        if hdr:
            all_hdrs.append(hdr)

    print_summary(all_hdrs)


if __name__ == '__main__':
    main()
