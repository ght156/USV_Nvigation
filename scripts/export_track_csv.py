#!/usr/bin/env python3
"""export_track_csv.py — 把 record_track_data.sh 录的 bag 导出成 CSV，便于对比三路轨迹。

用法（主机侧，先 source ROS + 装有 bw_gi320_driver 的工作空间）：
  source /opt/ros/humble/setup.bash
  source ~/USV_NAV/install/setup.bash
  python3 export_track_csv.py ~/usv_bags/track_20260922_153000 [输出目录]

产出（默认 <bag目录>/csv/）：
  gngga.csv      平台 OSD 上报用的数据源（原始 GNSS 解）：t, lat, lon, alt, fix, sats, hdop
  inspvaa.csv    导航使用的定位源（INS 融合解）       ：t, lat, lon, alt, ve, vn, yaw_deg
  odom.csv       Nav2 实际里程计（map/odom 系）       ：t, x, y, yaw_deg, vx, wz
  cmd_vel.csv    指令链对比                          ：t, topic, vx, wz
  plan.csv       Nav2 全局规划（map 系）              ：plan_id, idx, x, y（每次重规划一组）
"""
import csv
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    bag_dir = Path(sys.argv[1]).expanduser().resolve()
    out_dir = Path(sys.argv[2]).expanduser().resolve() if len(sys.argv) > 2 else bag_dir / "csv"
    out_dir.mkdir(parents=True, exist_ok=True)

    reader = open_reader(bag_dir)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    print(f"bag: {bag_dir}")
    for name, ptype in sorted(type_map.items()):
        print(f"  {name:<46} {ptype}")

    writers = {}
    files = {}

    def writer_for(key, header):
        if key not in writers:
            f = open(out_dir / f"{key}.csv", "w", newline="")
            w = csv.writer(f)
            w.writerow(header)
            writers[key] = w
            files[key] = f
        return writers[key]

    plan_id = 0
    n = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        ptype = type_map.get(topic)
        if ptype is None:
            continue
        msg = deserialize_message(data, get_message(ptype))
        t_sec = t * 1e-9

        if topic == "/gi320/gngga":
            writer_for("gngga", ["t", "stamp", "lat", "lon", "alt", "fix_type",
                                 "sats", "hdop", "odometer"]).writerow(
                [f"{t_sec:.3f}", f"{stamp_to_sec(msg.stamp):.3f}",
                 f"{msg.latitude:.8f}", f"{msg.longitude:.8f}", f"{msg.altitude:.3f}",
                 msg.fix_type, msg.satellite_num, f"{msg.hdop:.2f}", f"{msg.odometer:.2f}"])
        elif topic == "/gi320/inspvaa":
            writer_for("inspvaa", ["t", "stamp", "lat", "lon", "alt", "ve", "vn", "vu",
                                   "roll_deg", "pitch_deg", "yaw_deg", "odometer"]).writerow(
                [f"{t_sec:.3f}", f"{stamp_to_sec(msg.stamp):.3f}",
                 f"{msg.latitude:.8f}", f"{msg.longitude:.8f}", f"{msg.altitude:.3f}",
                 f"{msg.velocity_e:.3f}", f"{msg.velocity_n:.3f}", f"{msg.velocity_u:.3f}",
                 f"{msg.roll:.3f}", f"{msg.pitch:.3f}", f"{msg.yaw:.3f}",
                 f"{msg.odometer:.2f}"])
        elif topic == "/mavros/local_position/odom":
            p, tw = msg.pose.pose, msg.twist.twist
            writer_for("odom", ["t", "frame", "x", "y", "yaw_deg", "vx", "wz"]).writerow(
                [f"{t_sec:.3f}", msg.header.frame_id,
                 f"{p.position.x:.3f}", f"{p.position.y:.3f}",
                 f"{math.degrees(yaw_from_quaternion(p.orientation)):.2f}",
                 f"{tw.linear.x:.3f}", f"{tw.angular.z:.3f}"])
        elif topic in ("/cmd_vel_nav", "/mavros/setpoint_velocity/cmd_vel_unstamped"):
            writer_for("cmd_vel", ["t", "topic", "vx", "wz"]).writerow(
                [f"{t_sec:.3f}", topic, f"{msg.linear.x:.4f}", f"{msg.angular.z:.4f}"])
        elif topic == "/plan":
            w = writer_for("plan", ["plan_id", "idx", "x", "y"])
            for i, ps in enumerate(msg.poses):
                w.writerow([plan_id, i, f"{ps.pose.position.x:.3f}", f"{ps.pose.position.y:.3f}"])
            plan_id += 1
        n += 1

    for f in files.values():
        f.close()
    print(f"\n共处理 {n} 条消息，CSV 输出到: {out_dir}")
    for key in sorted(files):
        print(f"  {key}.csv")


if __name__ == "__main__":
    main()
