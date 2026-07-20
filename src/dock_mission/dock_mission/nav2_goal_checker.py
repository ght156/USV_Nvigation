"""Nav2 goal checker profile switch (Plan A + param fallback)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@dataclass
class GoalCheckerProfile:
    name: str
    xy_goal_tolerance: float
    yaw_goal_tolerance: float


CRUISE_PROFILE = GoalCheckerProfile(
    name="general_goal_checker",
    xy_goal_tolerance=1.0,
    yaw_goal_tolerance=1.0,
)

DOCKING_PROFILE = GoalCheckerProfile(
    name="docking_goal_checker",
    xy_goal_tolerance=0.6,
    yaw_goal_tolerance=0.15,
)


class _AsyncParametersClient:
    """Set parameters on a remote node via /set_parameters (Humble rclpy)."""

    def __init__(self, node: Node, remote_node_name: str) -> None:
        self._node = node
        name = remote_node_name.lstrip("/")
        self._client = node.create_client(
            SetParameters,
            f"/{name}/set_parameters",
        )

    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def set_parameters(self, params: list[Parameter]):
        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg() for p in params]
        return self._client.call_async(req)


class Nav2GoalCheckerSwitch:
    """Switch Nav2 goal checker for cruise vs dock staging.

    Plan A: publish to ``goal_checker_selector`` (BT GoalCheckerSelector).
    Fallback: set ``general_goal_checker`` tolerances via parameter client.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._controller = str(
            node.get_parameter("nav2_controller_node").value
        )
        self._selector_topic = str(
            node.get_parameter("goal_checker_selector_topic").value
        )
        self._use_selector = bool(node.get_parameter("use_goal_checker_selector").value)
        self._cruise = GoalCheckerProfile(
            name=str(node.get_parameter("cruise_goal_checker_id").value),
            xy_goal_tolerance=float(
                node.get_parameter("cruise_xy_goal_tolerance").value
            ),
            yaw_goal_tolerance=float(
                node.get_parameter("cruise_yaw_goal_tolerance").value
            ),
        )
        self._docking = GoalCheckerProfile(
            name=str(node.get_parameter("docking_goal_checker_id").value),
            xy_goal_tolerance=float(
                node.get_parameter("docking_xy_goal_tolerance").value
            ),
            yaw_goal_tolerance=float(
                node.get_parameter("docking_yaw_goal_tolerance").value
            ),
        )
        self._active: Optional[str] = None
        self._param_client = _AsyncParametersClient(node, self._controller)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._selector_pub = node.create_publisher(
            String, self._selector_topic, qos
        )

    def _publish_selector(self, checker_id: str) -> None:
        if not self._use_selector:
            return
        msg = String()
        msg.data = checker_id
        self._selector_pub.publish(msg)
        self._node.get_logger().info(
            f"goal_checker_selector → {checker_id!r}"
        )

    def _set_param_tolerances(self, profile: GoalCheckerProfile) -> None:
        if not self._param_client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warning(
                f"{self._controller} param service unavailable; "
                "goal checker tolerances not updated"
            )
            return
        prefix = f"{profile.name}."
        params = [
            Parameter(f"{prefix}xy_goal_tolerance", value=profile.xy_goal_tolerance),
            Parameter(f"{prefix}yaw_goal_tolerance", value=profile.yaw_goal_tolerance),
        ]
        future = self._param_client.set_parameters(params)

        def _done(fut) -> None:
            try:
                resp = fut.result()
                results = resp.results if resp is not None else []
                for r in results:
                    if not r.successful:
                        self._node.get_logger().warning(
                            f"param set failed: {r.reason}"
                        )
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().warning(f"param set error: {exc}")

        future.add_done_callback(_done)

    def apply_docking(self) -> None:
        if self._active == "docking":
            return
        self._publish_selector(self._docking.name)
        self._set_param_tolerances(self._docking)
        # Fallback when BT has no GoalCheckerSelector: tighten active general checker.
        self._set_param_tolerances(
            GoalCheckerProfile(
                name=self._cruise.name,
                xy_goal_tolerance=self._docking.xy_goal_tolerance,
                yaw_goal_tolerance=self._docking.yaw_goal_tolerance,
            )
        )
        self._active = "docking"
        self._node.get_logger().info(
            f"Nav2 goal profile DOCKING "
            f"(selector={self._docking.name}, "
            f"xy={self._docking.xy_goal_tolerance}, "
            f"yaw={self._docking.yaw_goal_tolerance})"
        )

    def apply_cruise(self) -> None:
        if self._active == "cruise":
            return
        self._publish_selector(self._cruise.name)
        self._set_param_tolerances(self._cruise)
        self._active = "cruise"
        self._node.get_logger().info(
            f"Nav2 goal profile CRUISE "
            f"(xy={self._cruise.xy_goal_tolerance}, "
            f"yaw={self._cruise.yaw_goal_tolerance})"
        )
