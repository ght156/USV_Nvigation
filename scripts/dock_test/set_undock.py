#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

rclpy.init()
n = Node("param_set2")
c = n.create_client(SetParameters, "/docking_fsm/set_parameters")
if c.wait_for_service(timeout_sec=5):
    req = SetParameters.Request()
    req.parameters = [Parameter(name="undock_complete_x", value=ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=-3.0))]
    fut = c.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
    print("undock_complete_x -> -3.0:", fut.result().results[0].successful if fut.done() else "超时")
else:
    print("服务不可用")
n.destroy_node()
rclpy.shutdown()
