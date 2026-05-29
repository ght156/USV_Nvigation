#!/usr/bin/env python3
"""Quick smoke test for the 3 navigation services + EMERGENCY clear via topic."""
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from m_common.srv import CancelMission, SendWaypoints, SetPause, EmergencyStop


PASS = 0
FAIL = 0


def check(cond, label):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [PASS] {label}")
    else:
        FAIL += 1
        print(f"  [FAIL] {label}")


def main():
    global PASS, FAIL
    rclpy.init(args=sys.argv)
    node = Node("quick_test")
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)

    # Clients
    send_cli = node.create_client(SendWaypoints, "mission_bridge/send_waypoints")
    pause_cli = node.create_client(SetPause, "mission_bridge/set_pause")
    emerg_cli = node.create_client(EmergencyStop, "mission_bridge/emergency_stop")
    cancel_cli = node.create_client(CancelMission, "mission_bridge/cancel_mission")
    state_sub = node.create_subscription(String, "/mission_bridge/state",
                                         lambda m: setattr(node, "_s", m.data), 10)
    node._s = "?"

    # Wait for services
    print("Waiting for services...")
    for cli, name in [
        (send_cli, "send_waypoints"),
        (pause_cli, "set_pause"),
        (emerg_cli, "emergency_stop"),
        (cancel_cli, "cancel_mission"),
    ]:
        ok = cli.wait_for_service(timeout_sec=5.0)
        check(ok, f"service {name} available")

    def spin_until(fn, timeout=5.0):
        dl = time.time() + timeout
        while time.time() < dl:
            rclpy.spin_once(node, timeout_sec=0.05)
            if fn():
                return True
        return False

    def make_pose(x, y):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        return ps

    # Test 1: send_waypoints in WAITING_SYSTEM → reject
    print("\n--- Test: send_waypoints in WAITING_SYSTEM ---")
    req = SendWaypoints.Request()
    req.waypoints = [make_pose(10.0, 20.0)]
    req.mission_id = "test"
    fut = send_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and not resp.success,
          f"reject in WAITING_SYSTEM (success={resp.success if resp else 'None'}, msg={resp.message if resp else 'None'})")

    # Test 2: set_pause in WAITING_SYSTEM → reject
    print("\n--- Test: set_pause in WAITING_SYSTEM ---")
    req = SetPause.Request()
    req.pause = True
    fut = pause_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and not resp.success,
          f"pause reject in WAITING_SYSTEM (success={resp.success if resp else 'None'})")

    # Test 3: set_pause resume in WAITING_SYSTEM → reject
    req = SetPause.Request()
    req.pause = False
    fut = pause_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and not resp.success,
          f"resume reject in WAITING_SYSTEM (success={resp.success if resp else 'None'})")

    # Test 4: emergency_stop → succeed from any non-EMERGENCY state
    print("\n--- Test: emergency_stop ---")
    req = EmergencyStop.Request()
    fut = emerg_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and resp.success,
          f"emergency_stop OK (success={resp.success if resp else 'None'})")

    # Wait for state to become EMERGENCY
    ok = spin_until(lambda: node._s == "EMERGENCY", timeout=3.0)
    check(ok, f"state → EMERGENCY (got: {node._s})")

    # Test 5: emergency_stop again → idempotent
    print("\n--- Test: emergency_stop (idempotent) ---")
    req = EmergencyStop.Request()
    fut = emerg_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and resp.success,
          f"idempotent (success={resp.success if resp else 'None'})")

    # Test 6: send_waypoints in EMERGENCY → reject
    print("\n--- Test: send_waypoints in EMERGENCY ---")
    req = SendWaypoints.Request()
    req.waypoints = [make_pose(10.0, 20.0)]
    req.mission_id = "test"
    fut = send_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and not resp.success,
          f"reject in EMERGENCY (success={resp.success if resp else 'None'})")

    # Test 7: set_pause in EMERGENCY → reject
    print("\n--- Test: set_pause in EMERGENCY ---")
    req = SetPause.Request()
    req.pause = True
    fut = pause_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and not resp.success,
          f"pause reject in EMERGENCY (success={resp.success if resp else 'None'})")

    # Test 8: clear EMERGENCY via cancel_mission service
    print("\n--- Test: clear EMERGENCY via cancel_mission ---")
    req = CancelMission.Request()
    fut = cancel_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and resp.success,
          f"cancel_mission OK (success={resp.success if resp else 'None'})")
    ok = spin_until(lambda: node._s != "EMERGENCY", timeout=3.0)
    check(ok, f"cleared EMERGENCY (state={node._s})")

    # Test 9: empty waypoints → reject
    print("\n--- Test: send_waypoints (empty) ---")
    req = SendWaypoints.Request()
    req.waypoints = []
    req.mission_id = ""
    fut = send_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    check(resp is not None and not resp.success,
          f"empty reject (success={resp.success if resp else 'None'})")

    # Summary
    total = PASS + FAIL
    print(f"\n{'='*50}")
    print(f"  Results: {PASS}/{total} passed, {FAIL} failed")
    if FAIL == 0:
        print(f"  ALL TESTS PASSED")
    print(f"{'='*50}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
