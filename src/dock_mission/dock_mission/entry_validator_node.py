"""ROS node wrapper for DockEntryValidator."""

from __future__ import annotations

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dock_mission.dock_enu import Pose2D, load_bay_from_dict
from dock_mission.dock_enu import DockEnuTransform
from dock_mission.entry_validator import DockEntryValidator, EntryValidationInput


class EntryValidatorNode(Node):
    def __init__(self) -> None:
        super().__init__("dock_entry_validator")
        self.declare_parameter("bay_id", "bay2")
        self.declare_parameter("dock_database_path", "")
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("require_tag_for_proceed", False)
        self.declare_parameter("tag_mismatch_threshold_m", 0.8)

        bay_id = self.get_parameter("bay_id").value
        # TODO: load YAML from dock_database_path; placeholder geometry for skeleton
        bay = load_bay_from_dict(
            bay_id,
            {
                "origin_map": {"x": -0.5, "y": 0.0, "x_axis_yaw": 3.141592653589793},
                "staging_dock_enu": {"x": -4.0, "y": 0.0, "yaw": 3.14159265359},
                "entry_corridor": {
                    "x_min": -6.0,
                    "x_max": 0.0,
                    "y_max": 1.0,
                    "yaw_max": 0.15,
                },
                "standoff_m": 4.0,
            },
        )
        self._tf = DockEnuTransform(bay)
        self._validator = DockEntryValidator(
            self._tf,
            tag_mismatch_threshold_m=float(
                self.get_parameter("tag_mismatch_threshold_m").value
            ),
            require_tag_for_proceed=bool(
                self.get_parameter("require_tag_for_proceed").value
            ),
        )

        self._last_odom: Pose2D | None = None
        self._last_result_json = "{}"

        self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)
        self.create_service(Trigger, "/dock/validate_entry", self._validate_srv)

        rate = max(float(self.get_parameter("status_rate_hz").value), 0.5)
        self.create_timer(1.0 / rate, self._publish_status)
        self._status_pub = self.create_publisher(String, "/dock/entry_status", 10)
        self.get_logger().info(f"dock_entry_validator ready bay={bay_id} (skeleton)")

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._last_odom = Pose2D(x=p.x, y=p.y, yaw=yaw)

    def _run_validation(self) -> dict:
        if self._last_odom is None:
            payload = {
                "valid": False,
                "action": "REJECT",
                "reason": "POSE_UNAVAILABLE",
                "message": "no odometry yet",
            }
            self._last_result_json = json.dumps(payload)
            return payload

        result = self._validator.validate(
            EntryValidationInput(boat_map=self._last_odom, rtk_fix=True)
        )
        payload = {
            "valid": result.valid,
            "action": result.action.value,
            "reason": result.reason.value,
            "ex": round(result.ex, 4),
            "ey": round(result.ey, 4),
            "eyaw": round(result.eyaw, 4),
            "tag_visible": result.tag_visible,
            "message": result.message,
            "bay_id": self._tf.bay.bay_id,
        }
        self._last_result_json = json.dumps(payload, ensure_ascii=False)
        return payload

    def _validate_srv(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        payload = self._run_validation()
        resp.success = bool(payload.get("valid"))
        resp.message = self._last_result_json
        return resp

    def _publish_status(self) -> None:
        payload = self._run_validation()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EntryValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
