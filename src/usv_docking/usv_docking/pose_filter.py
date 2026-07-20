"""Exponential moving average filter for dock pose errors."""

from __future__ import annotations


class PoseEmaFilter:
    """Single-pole low-pass on (x, y, yaw)."""

    def __init__(self, alpha: float) -> None:
        self.alpha = max(0.0, min(1.0, alpha))
        self._initialized = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def reset(self) -> None:
        self._initialized = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    @staticmethod
    def _wrap_yaw(yaw: float) -> float:
        import math

        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw < -math.pi:
            yaw += 2.0 * math.pi
        return yaw

    def update(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        yaw = self._wrap_yaw(yaw)
        if not self._initialized:
            self._x = x
            self._y = y
            self._yaw = yaw
            self._initialized = True
            return self._x, self._y, self._yaw

        a = self.alpha
        self._x = a * x + (1.0 - a) * self._x
        self._y = a * y + (1.0 - a) * self._y
        dyaw = self._wrap_yaw(yaw - self._yaw)
        self._yaw = self._wrap_yaw(self._yaw + a * dyaw)
        return self._x, self._y, self._yaw
