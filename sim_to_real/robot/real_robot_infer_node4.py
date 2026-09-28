import os
import sys
import math
import threading
import numpy as np
import torch

# --- ROS 2 Imports ---
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# [TF Function Disabled]: Commented out to avoid errors since use_tf_pose=false
# from tf2_ros import Buffer, TransformListener 

# --- Model Inference Imports ---
# Note: Ensure the import path matches your file structure for model_infer
from model.model_infer import LidarVLAInference

# ================= 1. Configuration Parameters =================
CONFIG_PATH = "./config/config2.yaml"
CHECKPOINT_PATH = "./checkpoints/vla_dual_lidar_epoch_500.pt"
NORM_STATS_PATH = os.path.dirname(CHECKPOINT_PATH) + "/dataset_norm_config.json"

# Virtual navigation goal (since goal_type='relative', set to always advance 3 meters straight ahead)
MANUAL_NAV_GOAL = np.array([3.0, 0.0, 0.0], dtype=np.float32) 

# ================= 2. ROS 2 Inference Node =================
class LidarVLAInferNode(Node):
    def __init__(self):
        super().__init__('lidar_vla_infer_node')
        
        # --- 1. Create Callback Groups (Core fix for concurrency blocking) ---
        # Sensor callbacks use ReentrantCallbackGroup to ensure real-time LiDAR data reception
        self.sensor_cb_group = ReentrantCallbackGroup()
        # Control and inference loops use MutuallyExclusiveCallbackGroup to prevent logic conflicts
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        self.inference_cb_group = MutuallyExclusiveCallbackGroup()
        
        # --- 2. Load Model Inference Engine ---
        self.get_logger().info('Loading Lidar VLA Model...')
        self.infer_engine = LidarVLAInference(
            config_path=CONFIG_PATH,
            checkpoint_path=CHECKPOINT_PATH,
            norm_stats_path=NORM_STATS_PATH,
            device='cuda' if torch.cuda.is_available() else 'cpu',
            # Orin optimization: downgrade to float16 to prevent underlying NaN errors
            dtype=torch.float16, 
            num_inference_steps=10,
            action_execution_horizon=8
        )
        
        # Optional: Enable torch.compile for acceleration if using PyTorch 2.x
        # self.infer_engine.runner.model = torch.compile(self.infer_engine.runner.model)
        
        self.get_logger().info('Model loading completed.')

        # --- 3. State Cache Variables and Thread Locks ---
        self.latest_scan_01 = None
        self.latest_scan_02 = None
        self.current_tf = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.queue_lock = threading.Lock()  # Used to ensure thread safety of the action queue
        
        # --- 4. Initialize ROS Communication ---
        self.sub_scan_01 = self.create_subscription(
            LaserScan, '/scan_01', self.scan_01_callback, 
            qos_profile_sensor_data, callback_group=self.sensor_cb_group)
            
        self.sub_scan_02 = self.create_subscription(
            LaserScan, '/scan_02', self.scan_02_callback, 
            qos_profile_sensor_data, callback_group=self.sensor_cb_group)
            
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 5. Dual-thread Timers: Separate Inference and Control ---
        # Timer A (10Hz): Solely responsible for popping actions from the queue and publishing
        self.control_timer = self.create_timer(
            0.1, self.publish_control_loop, callback_group=self.control_cb_group)
            
        # Timer B (20Hz): Background checks on the queue; triggers model inference to replenish
        self.inference_timer = self.create_timer(
            0.05, self.background_inference_loop, callback_group=self.inference_cb_group)
        
        self.get_logger().info('Node started, waiting for sensor data...')

    def scan_01_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        self.latest_scan_01 = np.nan_to_num(ranges, posinf=msg.range_max, neginf=0.0)

    def scan_02_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        self.latest_scan_02 = np.nan_to_num(ranges, posinf=msg.range_max, neginf=0.0)

    def background_inference_loop(self):
        """Background inference loop: responsible for replenishing the Action Queue"""
        # 1. Skip and save GPU resources if LiDAR data is not yet available
        if self.latest_scan_01 is None or self.latest_scan_02 is None:
            return
            
        # Receding Horizon threshold: start inferring the next chunk when less than 4 actions remain
        with self.queue_lock:
            queue_len = len(self.infer_engine.action_queue)
        
        if queue_len < 7:
            # Lock the current LiDAR frames for inference
            scan_01_input = self.latest_scan_01.copy().reshape(1, -1)
            scan_02_input = self.latest_scan_02.copy().reshape(1, -1)
            
            try:
                # Core: Directly call predict_chunk for a complete Euler ODE sampling
                full_chunk = self.infer_engine.predict_chunk(
                    scan_01_np=scan_01_input,
                    scan_02_np=scan_02_input,
                    current_tf_np=None,      # Global TF is disabled in config
                    nav_goal_np=MANUAL_NAV_GOAL
                )
                
                # Extract the valid Horizon length
                valid_actions = full_chunk[:self.infer_engine.action_execution_horizon]
                
                # Append the newly inferred actions to the queue
                with self.queue_lock:
                    self.infer_engine.action_queue.extend(valid_actions)
                    
            except Exception as e:
                self.get_logger().error(f"Inference crashed: {e}")

    def publish_control_loop(self):
        """Foreground control loop (strict 10Hz): fetch actions from Queue and control chassis"""
        v_x, w_z = 0.0, 0.0
        
        with self.queue_lock:
            if len(self.infer_engine.action_queue) > 0:
                action = self.infer_engine.action_queue.popleft()
                v_x, w_z = float(action[0]), float(action[1])
            else:
                # If GPU inference lags and causes a drop, print a warning and output 0 speed
                if self.latest_scan_01 is not None:
                    self.get_logger().warning(
                        "Inference queue exhausted, triggering safety brake! Potential GPU compute bottleneck.", 
                        throttle_duration_sec=1.0
                    )
        
        # Publish ROS control commands
        msg = Twist()
        msg.linear.x = v_x
        msg.angular.z = w_z
        self.cmd_publisher.publish(msg)
        
        # Print status
        with self.queue_lock:
            q_size = len(self.infer_engine.action_queue)
            
        self.get_logger().debug(f"Published v: {v_x:+.3f} | w: {w_z:+.3f} (Cache: {q_size})")

    def stop_robot(self):
        """Safety brake function"""
        msg = Twist()
        self.cmd_publisher.publish(msg)
        self.get_logger().info("Robot emergency stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarVLAInferNode()
    
    # Use MultiThreadedExecutor (optimized for Jetson Orin concurrency)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        # Start multi-threaded event loop
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt signal received, preparing to exit...")
    except Exception as e:
        node.get_logger().error(f"Exception occurred: {e}")
    finally:
        # Ensure the robot is stopped before exiting
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
