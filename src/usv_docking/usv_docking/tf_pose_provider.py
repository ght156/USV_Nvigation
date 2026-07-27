"""TF-based dock pose provider.

Looks up ``dock_frame -> base_link`` via TF2 and returns the boat's pose
in the dock coordinate frame together with a quality assessment.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException


class PoseQuality(Enum):
    GOOD = "good"
    STALE = "stale"
    JUMP = "jump"
    OUT_OF_RANGE = "out_of_range"
    INVALID = "invalid"


@dataclass(slots=True)
class PoseData:
    x: float
    y: float
    yaw: float
    age: float
    quality: PoseQuality


def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TfPoseProvider:
    """Provides the boat pose in dock frame via TF2 lookup."""

    def __init__(
        self,
        tf_buffer: Buffer,
        node,
        dock_frame: str = "dock_frame",
        robot_frame: str = "base_link",
        max_age_sec: float = 0.3,
        jump_threshold_m: float = 1.0,
        min_range_m: float = 0.3,
        max_range_m: float = 30.0,
    ) -> None:
        self._tf_buffer = tf_buffer
        self._node = node
        self._dock_frame = dock_frame
        self._robot_frame = robot_frame
        self._max_age_sec = max_age_sec
        self._jump_threshold = jump_threshold_m
        self._min_range = min_range_m
        self._max_range = max_range_m
        self._last_good: Optional[PoseData] = None

    # ------------------------------------------------------------------
    def get_pose(self, max_age_override: Optional[float] = None) -> PoseData:
        """Return the latest boat-in-dock pose with quality flag."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._dock_frame,
                self._robot_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return PoseData(0.0, 0.0, 0.0, float("inf"), PoseQuality.INVALID)

        now = self._node.get_clock().now()
        tf_time = Time.from_msg(tf.header.stamp)
        age = (now - tf_time).nanoseconds * 1e-9

        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        yaw = _quaternion_to_yaw(tf.transform.rotation)

        max_age = max_age_override if max_age_override is not None else self._max_age_sec
        if age > max_age:
            return PoseData(x, y, yaw, age, PoseQuality.STALE)

        dist = math.hypot(x, y)
        if dist < self._min_range or dist > self._max_range:
            return PoseData(x, y, yaw, age, PoseQuality.OUT_OF_RANGE)

        if self._last_good is not None:
            if (
                abs(x - self._last_good.x) > self._jump_threshold
                or abs(y - self._last_good.y) > self._jump_threshold
            ):
                return PoseData(x, y, yaw, age, PoseQuality.JUMP)

        pose = PoseData(x, y, yaw, age, PoseQuality.GOOD)
        self._last_good = pose
        return pose

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._last_good = None
