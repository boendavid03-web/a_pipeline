"""Wait on the actual controller parameter service instead of CLI graph cache."""
import json
import time
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = rclpy.create_node('continuation_controller_frame_config')
client = node.create_client(SetParameters,
    '/arena/env_0/task_generator_node/jackal/jackal_velocity_controller/set_parameters')
try:
    assert client.wait_for_service(timeout_sec=45), 'controller parameter service unavailable'
    future = client.call_async(SetParameters.Request(parameters=[
        Parameter('tf_frame_prefix_enable', value=False).to_parameter_msg()]))
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    assert future.done(), 'controller parameter response timeout'
    response = future.result()
    print(json.dumps(message_to_ordereddict(response)), flush=True)
    assert all(result.successful for result in response.results)
finally:
    node.destroy_node()
    rclpy.shutdown()
