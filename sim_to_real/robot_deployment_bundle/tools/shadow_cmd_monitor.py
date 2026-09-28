#!/usr/bin/env python3
"""Print commands from a shadow topic; it does not publish anything."""
import sys
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Monitor(Node):
    def __init__(self, topic):
        super().__init__('shadow_cmd_monitor')
        self.create_subscription(Twist, topic, self.show, 20)
    def show(self, message):
        self.get_logger().info('linear.x=%.3f angular.z=%.3f' % (message.linear.x, message.angular.z))


rclpy.init(); node = Monitor(sys.argv[1] if len(sys.argv) > 1 else '/sim_to_real/semantic_cnn/cmd_vel_shadow')
try: rclpy.spin(node)
except KeyboardInterrupt: pass
finally: node.destroy_node(); rclpy.shutdown()
