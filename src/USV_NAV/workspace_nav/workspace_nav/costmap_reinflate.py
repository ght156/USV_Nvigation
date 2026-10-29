#!/usr/bin/env python3
"""costmap_reinflate.py —— 清空局部代价图后的状态恢复。

背景（2026-10-29 实船定位，见 docs/bug_debug/）：
  Nav2 Humble 的 ``InflationLayer::reset()`` 只有 ``matchSize(); current_ = false;``，
  不置 ``need_reinflation_``（上游 main 分支已改成 ``setCurrent(false); need_reinflation_ = true;``）。
  本船 local_costmap 只挂了 voxel_layer(enabled=false) + inflation_layer：没有任何插件会
  产生更新边界时，``LayeredCostmap::updateMap()`` 会在

      if (xn < x0 || yn < y0) { return; }

  处提前返回，``InflationLayer::updateCosts()`` 永不执行，``current_`` 便永远回不到 true。
  ``controller_server::computeControl()`` 于是卡在

      while (!costmap_ros_->isCurrent()) { r.sleep(); }   # 100 Hz 空转

  ，不输出任何 ``/cmd_vel_nav``；BT 每秒重规划的新 FollowPath 目标堆进
  ``SimpleActionServer`` 的 pending 槽互相顶掉，每秒产生一条 ``Aborting handle`` WARN。

恢复手段：把 ``inflation_layer.inflation_radius`` 微调 ±delta 再改回原值。参数回调会置
  ``need_reinflation_ = true``，下一轮 ``updateBounds()`` 给出整图边界 → ``updateCosts()``
  执行 → ``current_ = true``。值最终改回原值，行为等价；240×240 的全图重膨胀是毫秒级。

用法::

    reinflater = CostmapReinflater(node)
    ...
    reinflater.trigger("清图后")   # 非阻塞，回调链跑在 executor 里
    reinflater.busy                # 上一次是否还没结束
    reinflater.last_result         # idle / ok / service_not_ready / timeout / fail: ...
"""
from __future__ import annotations

import time
from typing import Any, Optional, Tuple

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters


class CostmapReinflater:
    """对指定 costmap 节点做一次「参数微调 → 强制重膨胀」。"""

    def __init__(
        self,
        node: Any,
        costmap_node: str = "/local_costmap/local_costmap",
        param: str = "inflation_layer.inflation_radius",
        delta: float = 0.01,
        timeout_sec: float = 10.0,
    ) -> None:
        self._node = node
        self._param = param
        self._delta = abs(float(delta)) or 0.01
        self._timeout = max(1.0, float(timeout_sec))
        self._get = node.create_client(GetParameters, f"{costmap_node}/get_parameters")
        self._set = node.create_client(SetParameters, f"{costmap_node}/set_parameters")
        self._busy = False
        self._deadline = 0.0
        self._value: Optional[float] = None
        self._why = ""
        self.last_result = "idle"

    # ------------------------------------------------------------------ #
    # 状态
    # ------------------------------------------------------------------ #
    @property
    def busy(self) -> bool:
        self._expire()
        return self._busy

    def _expire(self) -> None:
        if self._busy and time.monotonic() > self._deadline:
            self._busy = False
            self.last_result = "timeout"
            self._node.get_logger().warning(
                f"[costmap_reinflate]: 超时未完成（{self._why}），已释放状态标记")

    def _fail(self, msg: str) -> None:
        self._busy = False
        self.last_result = f"fail: {msg}"
        self._node.get_logger().error(f"[costmap_reinflate]: {msg}（{self._why}）")

    # ------------------------------------------------------------------ #
    # 触发
    # ------------------------------------------------------------------ #
    def trigger(self, why: str) -> bool:
        self._expire()
        if self._busy:
            return False
        if not (self._get.service_is_ready() and self._set.service_is_ready()):
            self.last_result = "service_not_ready"
            self._node.get_logger().warning(
                f"[costmap_reinflate]: 参数服务未就绪，跳过（{why}）")
            return False
        self._busy = True
        self._why = why
        self._deadline = time.monotonic() + self._timeout
        req = GetParameters.Request()
        req.names = [self._param]
        self._get.call_async(req).add_done_callback(self._on_get)
        return True

    # ------------------------------------------------------------------ #
    # 回调链：get → set(+delta) → set(原值)
    # ------------------------------------------------------------------ #
    def _on_get(self, future: Any) -> None:
        try:
            resp = future.result()
            val = resp.values[0] if resp.values else None
            if val is None or val.type != ParameterType.PARAMETER_DOUBLE:
                raise ValueError(f"unexpected value type: {getattr(val, 'type', None)}")
            self._value = float(val.double_value)
        except Exception as exc:  # noqa: BLE001
            self._fail(f"读取 {self._param} 失败: {exc}")
            return
        self._set_value(self._value + self._delta, self._on_bump)

    def _set_value(self, value: float, done_cb: Any) -> None:
        req = SetParameters.Request()
        param = Parameter()
        param.name = self._param
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        req.parameters = [param]
        self._set.call_async(req).add_done_callback(done_cb)

    @staticmethod
    def _set_ok(future: Any) -> Tuple[bool, str]:
        try:
            resp = future.result()
            if not resp.results:
                return False, "empty result"
            res = resp.results[0]
            return bool(res.successful), str(res.reason)
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)

    def _on_bump(self, future: Any) -> None:
        ok, reason = self._set_ok(future)
        if not ok:
            self._fail(f"放大膨胀半径失败: {reason}")
            return
        if self._value is None:
            self._fail("未读到原始膨胀半径")
            return
        self._set_value(self._value, self._on_restore)

    def _on_restore(self, future: Any) -> None:
        ok, reason = self._set_ok(future)
        self._busy = False
        if ok:
            self.last_result = "ok"
            self._node.get_logger().info(
                "[costmap_reinflate]: 已强制重膨胀，local_costmap 的 current 状态恢复"
                f"（{self._why}）")
        else:
            self.last_result = f"fail: {reason}"
            self._node.get_logger().error(
                f"[costmap_reinflate]: 恢复膨胀半径失败: {reason}（{self._why}）")
