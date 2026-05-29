#!/usr/bin/env python3
"""
Integration test for navigation service interfaces.

Tests:
  1. send_waypoints  — normal send, empty reject, WAITING_SYSTEM reject
  2. set_pause       — pause RUNNING, resume PAUSED, reject on wrong state
  3. emergency_stop  — stop from RUNNING, idempotent, clear via cancel

Usage:
  # Terminal 1: launch the navigation stack (mission_bridge + aggregator)
  ros2 launch workspace_nav mission_bridge.launch.py

  # Terminal 2: run tests
  python3 test/test_nav_services.py

  # Or run a single test case:
  python3 test/test_nav_services.py --case send_waypoints
"""

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

# Will be populated after m_common is built
try:
    from m_common.srv import CancelMission, SendWaypoints, SetPause, EmergencyStop
except ImportError:
    print("ERROR: m_common.srv not found. Build with: colcon build --packages-select m_common")
    print("Then source install/setup.bash before running this script.")
    sys.exit(1)


GREEN = "\x1b[32m"
RED = "\x1b[31m"
YELLOW = "\x1b[33m"
RESET = "\x1b[0m"
BOLD = "\x1b[1m"


def ok(msg: str) -> str:
    return f"{GREEN}[OK]{RESET} {msg}"


def fail(msg: str) -> str:
    return f"{RED}[FAIL]{RESET} {msg}"


def warn(msg: str) -> str:
    return f"{YELLOW}[WARN]{RESET} {msg}"


class NavServiceTester(Node):
    """Test node that exercises mission_bridge service interfaces."""

    def __init__(self) -> None:
        super().__init__("nav_service_tester")
        self.results: List[Tuple[str, bool, str]] = []

        # Service clients
        self._send_cli = self.create_client(SendWaypoints, "mission_bridge/send_waypoints")
        self._pause_cli = self.create_client(SetPause, "mission_bridge/set_pause")
        self._emerg_cli = self.create_client(EmergencyStop, "mission_bridge/emergency_stop")
        self._cancel_cli = self.create_client(CancelMission, "mission_bridge/cancel_mission")

        # Subscribe to /mission_bridge/state to observe state changes
        self._latest_state: Optional[str] = None
        self._state_sub = self.create_subscription(
            String, "/mission_bridge/state", self._cb_state, 10)

    def _cb_state(self, msg: String) -> None:
        self._latest_state = msg.data

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def record(self, name: str, passed: bool, detail: str = "") -> None:
        self.results.append((name, passed, detail))
        if passed:
            self.get_logger().info(ok(f"{name}: {detail}" if detail else name))
        else:
            self.get_logger().error(fail(f"{name}: {detail}" if detail else name))

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        """Wait for all mission_bridge services to become available."""
        self.get_logger().info("Waiting for services...")
        services = [
            (self._send_cli, "mission_bridge/send_waypoints"),
            (self._pause_cli, "mission_bridge/set_pause"),
            (self._emerg_cli, "mission_bridge/emergency_stop"),
            (self._cancel_cli, "mission_bridge/cancel_mission"),
        ]
        deadline = time.time() + timeout_sec
        for cli, name in services:
            while time.time() < deadline:
                if cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f"  {name} — available")
                    break
            else:
                self.record("wait_for_services", False, f"{name} not available")
                return False
        self.record("wait_for_services", True, "all services available")
        return True

    def wait_for_state(self, target: str, timeout_sec: float = 5.0) -> bool:
        """Poll /mission_bridge/state until target is reached."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_state == target:
                return True
        return False

    @staticmethod
    def make_pose(x: float, y: float, frame: str = "map") -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = rclpy.time.Time().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        return ps

    # ------------------------------------------------------------------ #
    # Test cases
    # ------------------------------------------------------------------ #

    def test_send_waypoints_empty(self) -> None:
        """send_waypoints with empty array should be rejected."""
        req = SendWaypoints.Request()
        req.waypoints = []
        req.mission_id = ""
        future = self._send_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and not resp.success)
        self.record("send_waypoints (empty)", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")

    def test_send_waypoints_normal(self, waypoints: List[Tuple[float, float]]) -> bool:
        """send_waypoints with valid waypoints should succeed."""
        req = SendWaypoints.Request()
        req.waypoints = [self.make_pose(x, y) for x, y in waypoints]
        req.mission_id = f"test_{int(time.time())}"
        future = self._send_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and resp.success)
        self.record("send_waypoints (normal)", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")
        return passed

    def test_pause_while_running(self) -> None:
        """Pause should succeed when RUNNING."""
        req = SetPause.Request()
        req.pause = True
        future = self._pause_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and resp.success)
        self.record("set_pause (pause RUNNING)", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")

        # Verify state becomes PAUSED
        state_ok = self.wait_for_state("PAUSED", timeout_sec=3.0)
        self.record("set_pause → state=PAUSED", state_ok,
                    f"state={self._latest_state}")

    def test_resume_while_paused(self) -> None:
        """Resume should succeed when PAUSED."""
        req = SetPause.Request()
        req.pause = False
        future = self._pause_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and resp.success)
        self.record("set_pause (resume PAUSED)", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")

        # Verify state becomes RUNNING
        state_ok = self.wait_for_state("RUNNING", timeout_sec=3.0)
        self.record("set_pause → state=RUNNING", state_ok,
                    f"state={self._latest_state}")

    def test_pause_when_idle_fails(self) -> None:
        """Pause should fail when IDLE."""
        req = SetPause.Request()
        req.pause = True
        future = self._pause_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and not resp.success)
        self.record("set_pause (IDLE reject)", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")

    def test_emergency_stop(self) -> None:
        """Emergency stop should succeed and enter EMERGENCY state."""
        req = EmergencyStop.Request()
        future = self._emerg_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and resp.success)
        self.record("emergency_stop", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")

        # Verify state becomes EMERGENCY
        state_ok = self.wait_for_state("EMERGENCY", timeout_sec=3.0)
        self.record("emergency_stop → state=EMERGENCY", state_ok,
                    f"state={self._latest_state}")

    def test_emergency_stop_idempotent(self) -> None:
        """Emergency stop should succeed when already EMERGENCY (idempotent)."""
        req = EmergencyStop.Request()
        future = self._emerg_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and resp.success)
        self.record("emergency_stop (idempotent)", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")

    def test_clear_emergency(self) -> None:
        """cancel_mission service should clear EMERGENCY → IDLE."""
        req = CancelMission.Request()
        future = self._cancel_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        svc_ok = resp is not None and resp.success
        self.record("cancel_mission (clear EMERGENCY)", svc_ok,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")
        state_ok = self.wait_for_state("IDLE", timeout_sec=3.0)
        self.record("cancel_mission → state=IDLE", state_ok,
                    f"state={self._latest_state}")

    def test_resume_when_idle_fails(self) -> None:
        """Resume should fail when IDLE."""
        req = SetPause.Request()
        req.pause = False
        future = self._pause_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and not resp.success)
        self.record("set_pause (resume IDLE reject)", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")

    def test_send_waypoints_while_emergency_fails(self) -> None:
        """send_waypoints should be rejected in EMERGENCY state."""
        req = SendWaypoints.Request()
        req.waypoints = [self.make_pose(10.0, 20.0)]
        req.mission_id = ""
        future = self._send_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        passed = (resp is not None and not resp.success)
        self.record("send_waypoints (EMERGENCY reject)", passed,
                    f"success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'}")

    # ------------------------------------------------------------------ #
    # Orchestration
    # ------------------------------------------------------------------ #

    def run_all(self) -> None:
        """Run the full test suite."""
        print(f"\n{BOLD}{'='*60}{RESET}")
        print(f"{BOLD}  Navigation Service Interface Test Suite{RESET}")
        print(f"{BOLD}{'='*60}{RESET}\n")

        if not self.wait_for_services():
            self.get_logger().fatal("Services not available — is mission_bridge running?")
            return

        # Pre-condition: ensure IDLE
        self.get_logger().info("Pre-flight: waiting for IDLE...")
        if not self.wait_for_state("IDLE", timeout_sec=10.0):
            self.get_logger().warning(
                f"State is {self._latest_state}, not IDLE — "
                "tests may produce unexpected results"
            )

        # 1. Empty waypoints reject
        print(f"\n{BOLD}--- Test 1: send_waypoints (empty reject) ---{RESET}")
        self.test_send_waypoints_empty()

        # 2. Pause when IDLE → fail
        print(f"\n{BOLD}--- Test 2: set_pause (IDLE reject) ---{RESET}")
        self.test_pause_when_idle_fails()

        # 3. Resume when IDLE → fail
        print(f"\n{BOLD}--- Test 3: set_pause resume (IDLE reject) ---{RESET}")
        self.test_resume_when_idle_fails()

        # 4. Normal send_waypoints
        print(f"\n{BOLD}--- Test 4: send_waypoints (normal) ---{RESET}")
        test_waypoints = [(10.0, 10.0), (20.0, 20.0), (30.0, 10.0)]
        if not self.test_send_waypoints_normal(test_waypoints):
            self.get_logger().error("Cannot continue — send_waypoints failed")
            self.print_summary()
            return

        # 5. Pause while RUNNING
        print(f"\n{BOLD}--- Test 5: set_pause (pause RUNNING) ---{RESET}")
        self.test_pause_while_running()

        # 6. Resume from PAUSED
        print(f"\n{BOLD}--- Test 6: set_pause (resume PAUSED) ---{RESET}")
        self.test_resume_while_paused()

        # 7. Emergency stop from RUNNING
        print(f"\n{BOLD}--- Test 7: emergency_stop ---{RESET}")
        self.test_emergency_stop()

        # 8. Emergency stop idempotent
        print(f"\n{BOLD}--- Test 8: emergency_stop (idempotent) ---{RESET}")
        self.test_emergency_stop_idempotent()

        # 9. send_waypoints rejected in EMERGENCY
        print(f"\n{BOLD}--- Test 9: send_waypoints (EMERGENCY reject) ---{RESET}")
        self.test_send_waypoints_while_emergency_fails()

        # 10. Clear EMERGENCY via cancel
        print(f"\n{BOLD}--- Test 10: clear EMERGENCY → IDLE ---{RESET}")
        self.test_clear_emergency()

        self.print_summary()

    def print_summary(self) -> None:
        passed = sum(1 for _, p, _ in self.results if p)
        total = len(self.results)
        print(f"\n{BOLD}{'='*60}{RESET}")
        print(f"{BOLD}  Results: {passed}/{total} passed{RESET}")
        if passed == total:
            print(f"  {GREEN}ALL TESTS PASSED{RESET}")
        else:
            print(f"  {RED}FAILURES:{RESET}")
            for name, p, detail in self.results:
                if not p:
                    print(f"    - {name}: {detail}")
        print(f"{BOLD}{'='*60}{RESET}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Navigation service integration test")
    parser.add_argument("--case", type=str, default="all",
                        choices=["all", "send_waypoints", "pause", "emergency"],
                        help="Run a specific test case or all (default: all)")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    tester = NavServiceTester()

    try:
        if args.case == "all":
            tester.run_all()
        elif args.case == "send_waypoints":
            if tester.wait_for_services():
                tester.test_send_waypoints_normal([(5.0, 5.0), (15.0, 15.0)])
                tester.test_send_waypoints_empty()
                tester.print_summary()
        elif args.case == "pause":
            if tester.wait_for_services():
                tester.test_send_waypoints_normal([(5.0, 5.0), (15.0, 15.0)])
                tester.test_pause_while_running()
                tester.test_resume_while_paused()
                tester.test_pause_when_idle_fails()
                tester.print_summary()
        elif args.case == "emergency":
            if tester.wait_for_services():
                tester.test_emergency_stop()
                tester.test_emergency_stop_idempotent()
                tester.test_send_waypoints_while_emergency_fails()
                tester.test_clear_emergency()
                tester.print_summary()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
