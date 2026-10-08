#!/usr/bin/env python3
import sys, time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

rclpy.init()
n = Node("param_set")
def setp(node_name, pname, val):
    c = n.create_client(SetParameters, f"/{node_name}/set_parameters")
    if not c.wait_for_service(timeout_sec=5):
        print(node_name, "服务不可用")
        return
    req = SetParameters.Request()
    req.parameters = [Parameter(name=pname, value=ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=val))]
    fut = c.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
    print(node_name, pname, "->", val,
          fut.result().results[0].successful if fut.done() else "超时")

for pn, v in [("max_yaw_rate", 0.15), ("search_angular_speed", 0.13), ("back_in_max_yaw_rate", 0.12)]:
    setp("docking_motion_controller", pn, v)
n.destroy_node()
rclpy.shutdown()
