#!/usr/bin/env python3
"""离线航点下发（不依赖 RViz / 地面站）：经纬度 → /waypoint JSON。

船端离线启动时用（ros2 topic pub 在船上偶发卡死，故用 rclpy 直发）。
mission_bridge 收到后 debounce ~0.45s 自动开始执行（详见 mission_bridge与上层对接.md）。

用法（船端，先 source 环境 + ROS_DOMAIN_ID=5）：
    python3 send_waypoint.py 31.48757 120.37039                      # 单点
    python3 send_waypoint.py 31.48757 120.37039 31.48780 120.37060   # 多点航线
    python3 send_waypoint.py --mission-id m001 31.48757 120.37039

安全说明：
  - 坐标是 WGS84 经纬度（度），mission_bridge 按 map.yaml 锚点转 map 系；
  - 同一航线静止期只发一次（内部连发 3 帧防抖，0.3s 间隔，等效 GCS 一次点击）；
  - 取消任务用：ros2 topic pub --once /gcs_mission/cancel std_msgs/msg/Empty
    （或 mission_bridge 的 cancel_mission service）。
"""
import argparse
import json
import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "5")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

import rclpy
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser(description="经纬度航点 → /waypoint（离线下发）")
    ap.add_argument("coords", nargs="+", type=float,
                    help="lat1 lon1 [lat2 lon2 ...]，WGS84 度")
    ap.add_argument("--mission-id", default="", help="可选任务 ID，原样回填状态")
    ap.add_argument("--topic", default="/waypoint")
    args = ap.parse_args()

    if len(args.coords) % 2 != 0:
        ap.error("经纬度必须成对出现：lat1 lon1 [lat2 lon2 ...]")
    pairs = list(zip(args.coords[0::2], args.coords[1::2]))
    for lat, lon in pairs:
        if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0 and lon == 0):
            ap.error(f"非法坐标 ({lat}, {lon})")

    payload = {"waypoints": [{"latitude": lat, "longitude": lon} for lat, lon in pairs]}
    if args.mission_id:
        payload["mission_id"] = args.mission_id

    rclpy.init()
    node = rclpy.create_node("send_waypoint_cli")
    pub = node.create_publisher(String, args.topic, 10)
    msg = String()
    msg.data = json.dumps(payload)
    # 连发 3 帧（0.3s 间隔）抗丢包；debounce 窗口内等效一次点击
    for i in range(3):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.3)
    print(f"已下发 {len(pairs)} 个航点到 {args.topic}: {msg.data}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
