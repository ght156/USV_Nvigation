"""Transform AprilTag dock_pose from camera frame to robot base_link."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _quaternion_to_yaw(q: Quaternion) -> float:
    """Extract yaw assuming base_link z-up; verify sign/magnitude on real boat."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _apply_invert(
    x: float,
    y: float,
    yaw: float,
    invert_x: bool,
    invert_y: bool,
    invert_yaw: bool,
) -> tuple[float, float, float]:
    if invert_x:
        x = -x
    if invert_y:
        y = -y
    if invert_yaw:
        yaw = -yaw
    return x, y, yaw


class DockPoseTransformer:
    """Convert Float64MultiArray dock pose to base_link (x, y, yaw)."""

    def __init__(
        self,
        tf_buffer: Buffer,
        camera_frame: str,
        robot_frame: str,
        tf_timeout_sec: float,
        allow_camera_frame_fallback: bool,
        invert_x: bool,
        invert_y: bool,
        invert_yaw: bool,
    ) -> None:
        self._tf_buffer = tf_buffer
        self._camera_frame = camera_frame
        self._robot_frame = robot_frame
        self._tf_timeout = Duration(seconds=tf_timeout_sec)
        self._allow_camera_frame_fallback = allow_camera_frame_fallback
        self._invert_x = invert_x
        self._invert_y = invert_y
        self._invert_yaw = invert_yaw

    def transform(
        self,
        pose_values: list[float],
        stamp: Optional[Time] = None,
    ) -> tuple[Optional[tuple[float, float, float]], Optional[str]]:
        """Return ((x, y, yaw) in robot frame, error_reason)."""
        if len(pose_values) < 6:
            return None, "INVALID_POSE_LENGTH"

        x, y, z, roll, pitch, yaw = pose_values[:6]
        pose_cam = PoseStamped()
        pose_cam.header.frame_id = self._camera_frame
        if stamp is not None:
            pose_cam.header.stamp = stamp.to_msg()
        pose_cam.pose.position.x = float(x)
        pose_cam.pose.position.y = float(y)
        pose_cam.pose.position.z = float(z)
        pose_cam.pose.orientation = _euler_to_quaternion(roll, pitch, yaw)

        try:
            # Phase 1: always use latest TF (dock_pose has no header stamp).
            transform = self._tf_buffer.lookup_transform(
                self._robot_frame,
                self._camera_frame,
                Time() if stamp is None else stamp,
                timeout=self._tf_timeout,
            )
            pose_base_stamped = do_transform_pose_stamped(pose_cam, transform)
            pose_base = pose_base_stamped.pose
        except TransformException as exc:
            if self._allow_camera_frame_fallback:
                pose_base = pose_cam.pose
            else:
                return None, f"TF_LOOKUP_FAILED:{exc}"

        x_b = float(pose_base.position.x)
        y_b = float(pose_base.position.y)
        yaw_b = float(_quaternion_to_yaw(pose_base.orientation))
        x_b, y_b, yaw_b = _apply_invert(
            x_b, y_b, yaw_b, self._invert_x, self._invert_y, self._invert_yaw
        )
        return (x_b, y_b, yaw_b), None
