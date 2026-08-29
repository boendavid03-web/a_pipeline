#!/usr/bin/env python
#
# revision history: xzt
#  20210604 (TE): first version (ROS1)
#
# Converted to ROS2 by [Your Name]
# usage: python drl_vo_inference.py

import sys
import os
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import message_filters

# Import ROS2 messages
from sensor_msgs.msg import LaserScan
from cnn_msgs.msg import CNNData   
from geometry_msgs.msg import Twist, Point

# Import SB3 and custom CNN module
from stable_baselines3 import PPO
import sys

class DrlInference(Node):
    def __init__(self):
        super().__init__('drl_inference')
        # Initialize data
        self.ped_pos = []
        self.scan = []
        self.goal = []
        self.vx = 0
        self.wz = 0
        self.model = None

        # Declare and get parameter for model file
        self.declare_parameter('model_file', '/home/kien/colcon_ws/src/nav2_pyif/drl_vo_controller/model/drl_vo.zip')
        model_file = self.get_parameter('model_file').value

        # Load model
        self.model = PPO.load(model_file)
        self.get_logger().info("Finish loading model.")

        # Create subscription and publisher
        self.cnn_data_sub = self.create_subscription(
            CNNData,
            '/cnn_data',
            self.cnn_data_callback,
            10  # QoS depth
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/drl_cmd_vel', 10)

    def cnn_data_callback(self, cnn_data_msg):
        # Update internal variables from the message
        self.ped_pos = cnn_data_msg.ped_pos_map
        self.scan = cnn_data_msg.scan
        self.goal = cnn_data_msg.goal_cart
        cmd_vel = Twist()

        # Process scan data to find minimum distance
        scan_section = np.array(self.scan[-540:-180])
        valid_scan = scan_section[scan_section != 0]
        if valid_scan.size != 0:
            min_scan_dist = np.amin(valid_scan)
        else:
            min_scan_dist = 10

        # Decide on behavior based on goal and obstacle margins
        if np.linalg.norm(self.goal) <= 0.9:  # goal margin
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
        elif min_scan_dist <= 0.4:  # obstacle margin
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0.7
        else:
            # Scale ped_pos using MaxAbsScaler
            v_min = -2
            v_max = 2
            self.ped_pos = np.array(self.ped_pos, dtype=np.float32)
            self.ped_pos = 2 * (self.ped_pos - v_min) / (v_max - v_min) - 1

            # Process scan data with averaging and scaling
            temp = np.array(self.scan, dtype=np.float32)
            scan_avg = np.zeros((20, 80))
            for n in range(10):
                scan_tmp = temp[n*720:(n+1)*720]
                for i in range(80):
                    scan_avg[2*n, i] = np.min(scan_tmp[i*9:(i+1)*9])
                    scan_avg[2*n+1, i] = np.mean(scan_tmp[i*9:(i+1)*9])
            scan_avg = scan_avg.reshape(1600)
            import numpy.matlib
            scan_avg_map = np.matlib.repmat(scan_avg, 1, 4)
            self.scan = scan_avg_map.reshape(6400)
            s_min = 0
            s_max = 30
            self.scan = 2 * (self.scan - s_min) / (s_max - s_min) - 1

            # Process goal using MaxAbsScaler
            g_min = -2
            g_max = 2
            goal_original = np.array(self.goal, dtype=np.float32)
            self.goal = 2 * (goal_original - g_min) / (g_max - g_min) - 1

            # Create observation by concatenating ped_pos, scan, and goal
            observation = np.concatenate((self.ped_pos, self.scan, self.goal), axis=None)

            # Get action from the loaded PPO model
            action, _states = self.model.predict(observation)
            # Scale action output to command velocities
            vx_min = 0
            vx_max = 0.5
            vz_min = -2
            vz_max = 2
            cmd_vel.linear.x = (action[0] + 1) * (vx_max - vx_min) / 2 + vx_min
            cmd_vel.angular.z = (action[1] + 1) * (vz_max - vz_min) / 2 + vz_min

        # Publish cmd_vel if valid
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z):
            self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = DrlInference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
