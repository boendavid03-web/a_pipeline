#!/usr/bin/env python3
#
# revision history: xzt
#  20210604 (TE): first version (ROS)
#  2025XXXX   (Your Name): Converted to ROS2 (rclpy)
#
# This script publishes pedestrian kinematic maps and a lidar historical map.
#------------------------------------------------------------------------------

import numpy as np
import rclpy
from rclpy.node import Node

from cnn_msgs.msg import CNN_data
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from pedsim_msgs.msg import TrackedPerson, TrackedPersons
from sensor_msgs.msg import LaserScan

# parameters:
NUM_TP = 10     # the number of timestamps
NUM_PEDS = 35   # 34+1 total pedestrians

class CnnDataNode(Node):
    def __init__(self):
        super().__init__('cnn_data')

        # initialize data
        self.ped_pos_map = []
        self.scan = []  # List to accumulate scan data over time
        self.scan_all = np.zeros(1080)
        self.goal_cart = np.zeros(2)
        self.goal_final_cart = np.zeros(2)
        self.vel = np.zeros(2)

        # temporal data
        self.ped_pos_map_tmp = np.zeros((2, 80, 80))  # Cartesian velocity map
        self.scan_tmp = np.zeros(720)
        self.scan_all_tmp = np.zeros(1080)

        self.ts_cnt = 0  # Counter for time steps
        self.rate = 20.0  # 20 Hz

        # Subscribers
        self.create_subscription(TrackedPersons, '/track_ped', self.ped_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Point, '/cnn_goal', self.goal_callback, 10)
        self.create_subscription(Twist, '/mobile_base/commands/velocity', self.vel_callback, 10)

        # Publisher
        self.cnn_data_pub = self.create_publisher(CNN_data, '/cnn_data', 10)

        # Timer (runs at 20 Hz)
        timer_period = 1.0 / self.rate  # seconds
        self.create_timer(timer_period, self.timer_callback)

    def ped_callback(self, msg: TrackedPersons):
        # Reset the temporary pedestrian position map
        self.ped_pos_map_tmp = np.zeros((2, 80, 80))
        if msg.tracks:
            for ped in msg.tracks:
                # Retrieve position and velocity
                x = ped.pose.pose.position.x
                y = ped.pose.pose.position.y
                vx = ped.twist.twist.linear.x
                vy = ped.twist.twist.linear.y

                # Check if pedestrian is within the 20m x 20m occupancy map region
                if 0 <= x <= 20 and np.abs(y) <= 10:
                    # Bin size is 0.25 m; compute grid indices
                    c = int(np.floor(-(y - 10) / 0.25))
                    r = int(np.floor(x / 0.25))

                    if r == 80:
                        r -= 1
                    if c == 80:
                        c -= 1
                    # Store velocity in the cartesian velocity map
                    self.ped_pos_map_tmp[0, r, c] = vx
                    self.ped_pos_map_tmp[1, r, c] = vy

    def scan_callback(self, msg: LaserScan):
        # Initialize temporary scan arrays
        self.scan_tmp = np.zeros(720)
        self.scan_all_tmp = np.zeros(1080)
        scan_data = np.array(msg.ranges, dtype=np.float32)
        scan_data[np.isnan(scan_data)] = 0.0
        scan_data[np.isinf(scan_data)] = 0.0

        # Extract a subset of the scan data (from index 180 to 900)
        self.scan_tmp = scan_data[180:900]
        self.scan_all_tmp = scan_data

    def goal_callback(self, msg: Point):
        # Store goal position (Cartesian coordinates)
        self.goal_cart = np.array([msg.x, msg.y])

    def vel_callback(self, msg: Twist):
        # Store current velocity
        self.vel = np.array([msg.linear.x, msg.angular.z])

    def timer_callback(self):
        # Update pedestrian map and accumulate scan history
        self.ped_pos_map = self.ped_pos_map_tmp
        self.scan.append(self.scan_tmp.tolist())
        self.scan_all = self.scan_all_tmp

        self.ts_cnt += 1
        if self.ts_cnt == NUM_TP:
            # When enough timesteps are collected, publish CNN data
            cnn_data = CNN_data()
            cnn_data.ped_pos_map = [
                float(val)
                for sublist in self.ped_pos_map
                for subb in sublist
                for val in subb
            ]
            cnn_data.scan = [
                float(val)
                for sublist in self.scan
                for val in sublist
            ]
            # Convert numpy array to list if necessary
            cnn_data.scan_all = self.scan_all.tolist() if hasattr(self.scan_all, "tolist") else self.scan_all
            cnn_data.depth = []       # Empty list (customize if needed)
            cnn_data.image_gray = []  # Empty list (customize if needed)
            cnn_data.goal_cart = (
                self.goal_cart.tolist() if hasattr(self.goal_cart, "tolist") else self.goal_cart
            )
            cnn_data.goal_final_polar = []  # Empty list (customize if needed)
            cnn_data.vel = self.vel.tolist() if hasattr(self.vel, "tolist") else self.vel

            self.cnn_data_pub.publish(cnn_data)

            # Reset the scan history list to maintain a sliding window of timesteps
            self.ts_cnt = NUM_TP - 1
            self.scan = self.scan[1:NUM_TP]

def main(args=None):
    rclpy.init(args=args)
    node = CnnDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
