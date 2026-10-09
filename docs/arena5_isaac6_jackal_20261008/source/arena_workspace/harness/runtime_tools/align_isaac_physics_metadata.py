"""CONFIG_ONLY: align the owned /arena metadata with the frozen Isaac step.

Does not modify Isaac physics, planner/model dt, rates, or persistent files.
"""
import json
import os
import pathlib
import time

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rosidl_runtime_py.convert import message_to_ordereddict as md

assert os.environ.get('ROS_DOMAIN_ID') == '189'
source = pathlib.Path('/opt/arena_ws/src/Arena/arena_isaac/arena_isaac/arena_isaac/run_isaacsim.py')
assert source.read_text().count('PHYSICS_DT = 1.0 / 60.0') == 1
rclpy.init()
node = rclpy.create_node('a_isaac_physics_metadata_alignment')


def call(typ, name, request):
    client = node.create_client(typ, name)
    assert client.wait_for_service(timeout_sec=15), name
    future = client.call_async(request)
    deadline = time.monotonic() + 20
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
    assert future.done(), name
    response = future.result()
    node.destroy_client(client)
    return response


try:
    before = call(GetParameters, '/arena/get_parameters', GetParameters.Request(names=['physics_dt']))
    assert len(before.values) == 1 and before.values[0].type == 3
    assert abs(before.values[0].double_value - .0333) < 1e-12, 'Unexpected baseline metadata'
    changed = call(SetParametersAtomically, '/arena/set_parameters_atomically',
                   SetParametersAtomically.Request(parameters=[Parameter('physics_dt', value=1.0 / 60.0).to_parameter_msg()]))
    assert changed.result.successful, changed.result.reason
    after = call(GetParameters, '/arena/get_parameters', GetParameters.Request(names=['physics_dt']))
    assert len(after.values) == 1 and abs(after.values[0].double_value - 1.0 / 60.0) < 1e-15
    print(json.dumps({'classification': 'CONFIG_ONLY_RUNTIME_METADATA', 'before': md(before),
                      'change': md(changed), 'after': md(after),
                      'actual_isaac_physics_hz_unchanged': 60.0,
                      'model_dt_and_manifest_unchanged': True}), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
