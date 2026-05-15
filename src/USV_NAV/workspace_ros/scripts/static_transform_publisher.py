#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that loads and publishes a set of static transforms as TF2 frames, using YAML or parameter
#  input. It converts roll–pitch–yaw rotations to quaternions and ensures all valid transforms are
#  broadcasted once for consistent frame alignment.
# ----------------------------------------------------------------------------------------------- #

import math
import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from typing import List, Tuple
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

class StaticTransformsPublisher(Node):
    def __init__(self) -> None:
        super().__init__('static_transforms_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.declare_parameter('static_transform_file', '')
        self.declare_parameter('static_transforms', [])
        transforms = None
        try:
            file_path = self.get_parameter('static_transform_file').value
            if file_path:
                file_path = os.path.expanduser(str(file_path))
                if os.path.isfile(file_path):
                    with open(file_path, 'r') as f:
                        data = yaml.safe_load(f)
                        if isinstance(data, dict) and 'static_transforms' in data and isinstance(data['static_transforms'], list):
                            transforms = data['static_transforms']
                            self.get_logger().info(f"Loaded {len(transforms)} transforms from file: {file_path}")
                        elif isinstance(data, list):
                            transforms = data
                            self.get_logger().info(f"Loaded {len(transforms)} transforms (list) from file: {file_path}")
                        else:
                            self.get_logger().warning(f"YAML file {file_path} parsed but did not contain a list of transforms; ignoring file.")
                else:
                    self.get_logger().warning(f"static_transform_file parameter set to '{file_path}' but file does not exist.")
        except Exception as e:
            self.get_logger().warning(f"Error while reading static_transform_file: {e}")
        if transforms is None:
            try:
                raw = self.get_parameter('static_transforms').value
                if isinstance(raw, list) and len(raw) > 0:
                    transforms = raw
                    self.get_logger().info(f"Loaded {len(transforms)} transforms from parameter 'static_transforms'.")
                elif raw:
                    self.get_logger().warning("Parameter 'static_transforms' provided but is not a list; ignoring parameter.")
            except Exception as e:
                self.get_logger().warning(f"Error while reading 'static_transforms' parameter: {e}")
        if transforms is None or not isinstance(transforms, list) or len(transforms) == 0:
            self.get_logger().error("No static transforms supplied via file or 'static_transforms' parameter. Exiting to avoid publishing defaults.")
            rclpy.shutdown()
            sys.exit(1)
        stamped_list: List[TransformStamped] = []
        now = self.get_clock().now().to_msg()
        for idx, t in enumerate(transforms):
            try:
                parent = str(t['parent'])
                child = str(t['child'])
                tx, ty, tz = [float(x) for x in t.get('translation', [0.0, 0.0, 0.0])]
                roll, pitch, yaw = [float(x) for x in t.get('rotation_rpy', [0.0, 0.0, 0.0])]
            except Exception as e:
                self.get_logger().warning(f"Skipping bad transform entry at index {idx}: {t} ({e})")
                continue
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
            ts = TransformStamped()
            ts.header.stamp = now
            ts.header.frame_id = parent
            ts.child_frame_id = child
            ts.transform.translation.x = tx
            ts.transform.translation.y = ty
            ts.transform.translation.z = tz
            ts.transform.rotation.x = qx
            ts.transform.rotation.y = qy
            ts.transform.rotation.z = qz
            ts.transform.rotation.w = qw
            stamped_list.append(ts)
            self.get_logger().info(f"Prepared static transform: {parent} -> {child}, t={t.get('translation')} rpy={t.get('rotation_rpy')}")
        if stamped_list:
            self.broadcaster.sendTransform(stamped_list)
            self.get_logger().info("Published all static transforms.")
        else:
            self.get_logger().error("No valid static transforms prepared (all entries invalid). Exiting.")
            rclpy.shutdown()
            sys.exit(1)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticTransformsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()