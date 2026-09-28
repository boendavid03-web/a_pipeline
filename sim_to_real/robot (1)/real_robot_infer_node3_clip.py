import os
import time
import sys
import math
import threading
import yaml
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
from tf2_ros import Buffer, TransformListener

# --- Model Inference Imports ---
from model.model_infer import LidarVLAInference

# ================= 1. Configuration Parameters =================
CONFIG_PATH = "./config/config2.yaml"
CHECKPOINT_PATH = "./checkpoints/vla_dual_lidar_epoch_500.pt"
#CHECKPOINT_PATH = "./checkpoints/vla_dual_lidar_epoch_800_20260909.pt"
NORM_STATS_PATH = os.path.dirname(CHECKPOINT_PATH) + "/dataset_norm_config.json"

# Global navigation target configuration
GLOBAL_FRAME = "map"  # Change to "map" if your system uses a global map
GLOBAL_NAV_GOAL = np.array([-0.671, 1.778, 0.493], dtype=np.float32)
#GLOBAL_NAV_GOAL = np.array([-2.072, 2.059, 0.493], dtype=np.float32)  # Absolute target [X, Y, Yaw] in GLOBAL_FRAME
ARRIVAL_THRESHOLD = 0.3  # Distance threshold in meters to stop the robot

# ================= (Dataset Config) =================
DATASET_CONFIG_PATH = "./dataset_config.yaml"
with open(DATASET_CONFIG_PATH, 'r') as f:
    ds_cfg = yaml.safe_load(f)

LIDAR_MAX_RANGE = ds_cfg['lidar']['max_range']
S01_START = ds_cfg['lidar']['scan_01']['slice_start']
S01_END = ds_cfg['lidar']['scan_01']['slice_end']
S02_START = ds_cfg['lidar']['scan_02']['slice_start']
S02_END = ds_cfg['lidar']['scan_02']['slice_end']

# ================= 2. Helper Functions =================
def quat_to_yaw(x, y, z, w):
    """Convert a quaternion into yaw angle in radians."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

# ================= 3. ROS 2 Inference Node =================
class LidarVLAInferNode(Node):
    def __init__(self):
        super().__init__('lidar_vla_infer_node')
        
        # --- 1. Create Callback Groups for Concurrency ---
        self.sensor_cb_group = ReentrantCallbackGroup()
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        self.inference_cb_group = MutuallyExclusiveCallbackGroup()
        
        # --- 2. Load Model Inference Engine ---
        self.get_logger().info('Loading Lidar VLA Model...')
        self.infer_engine = LidarVLAInference(
            config_path=CONFIG_PATH,
            checkpoint_path=CHECKPOINT_PATH,
            norm_stats_path=NORM_STATS_PATH,
            device='cuda' if torch.cuda.is_available() else 'cpu',
            dtype=torch.float16, 
            num_inference_steps=10,
            action_execution_horizon=8
        )
        self.get_logger().info('Model loading completed.')

        # --- 3. State Cache Variables and Thread Locks ---
        self.latest_scan_01 = None
        self.latest_scan_02 = None
        self.arrived = False  # Status flag to track if the robot reached the destination
        self.queue_lock = threading.Lock()  # Ensure thread safety for action queue operations
        
        # --- 4. Initialize TF Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- 5. Initialize ROS Communication ---
        self.sub_scan_01 = self.create_subscription(
            LaserScan, '/scan_01', self.scan_01_callback, 
            qos_profile_sensor_data, callback_group=self.sensor_cb_group)
            
        self.sub_scan_02 = self.create_subscription(
            LaserScan, '/scan_02', self.scan_02_callback, 
            qos_profile_sensor_data, callback_group=self.sensor_cb_group)
            
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 6. Dual-thread Timers ---
        # Timer A (10Hz): Foreground control execution loop
        self.control_timer = self.create_timer(
            0.1, self.publish_control_loop, callback_group=self.control_cb_group)
            
        # Timer B (20Hz): Background dynamic target calculation and network inference loop
        self.inference_timer = self.create_timer(
            0.05, self.background_inference_loop, callback_group=self.inference_cb_group)
        
        self.get_logger().info('Node started, waiting for sensor data and TF...')

    def scan_01_callback(self, msg):
        ranges_np = np.array(msg.ranges, dtype=np.float32)[S01_START:S01_END]
        ranges_clean = np.nan_to_num(ranges_np, posinf=LIDAR_MAX_RANGE, neginf=0.0)
        #  MAX_RANGE
        self.latest_scan_01 = np.clip(ranges_clean, 0.0, LIDAR_MAX_RANGE)

    def scan_02_callback(self, msg):
        ranges_np = np.array(msg.ranges, dtype=np.float32)[S02_START:S02_END]
        ranges_clean = np.nan_to_num(ranges_np, posinf=LIDAR_MAX_RANGE, neginf=0.0)
        # MAX_RANGE
        self.latest_scan_02 = np.clip(ranges_clean, 0.0, LIDAR_MAX_RANGE)

    def background_inference_loop(self):
        """Background loop: lookup TF, compute relative target, and replenish action queue."""
        if self.latest_scan_01 is None or self.latest_scan_02 is None:
            return

        # 1. Lookup current robot pose in the global frame via TF
        try:
            trans = self.tf_buffer.lookup_transform(GLOBAL_FRAME, 'base_link', rclpy.time.Time())
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
            q = trans.transform.rotation
            robot_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        except Exception as e:
            self.get_logger().warning(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return

        # 2. Calculate distance to global goal
        dx = GLOBAL_NAV_GOAL[0] - robot_x
        dy = GLOBAL_NAV_GOAL[1] - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # 3. Check if the robot has arrived at the destination
        if distance < ARRIVAL_THRESHOLD:
            if not self.arrived:
                self.get_logger().info("Target reached. Stopping the robot.")
                self.arrived = True
                with self.queue_lock:
                    self.infer_engine.action_queue.clear()
            return
        else:
            self.arrived = False

        # 4. Check action queue capacity before executing neural network
        with self.queue_lock:
            queue_len = len(self.infer_engine.action_queue)
        print('queue_len:', queue_len)
        if queue_len < 6:
            # 5. Transform global target coordinates into the robot's local base frame
            local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
            local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
            local_yaw = GLOBAL_NAV_GOAL[2] - robot_yaw
            local_yaw = math.atan2(math.sin(local_yaw), math.cos(local_yaw)) # Normalize to [-pi, pi]
            
            current_local_goal = np.array([local_x, local_y, local_yaw], dtype=np.float32)

            # Copy sensor arrays safely to prevent race conditions during model forward pass
            scan_01_input = self.latest_scan_01.copy().reshape(1, -1)
            scan_02_input = self.latest_scan_02.copy().reshape(1, -1)
            
            try:
                st = time.time()
                # Call model inference with the dynamically updated local target
                full_chunk = self.infer_engine.predict_chunk(
                    scan_01_np=scan_01_input,
                    scan_02_np=scan_02_input,
                    current_tf_np=None,      # Extracted pose from TF is used manually above
                    nav_goal_np=current_local_goal
                )
                print('full_chunk:', full_chunk.shape, 'infer delay:', time.time() - st)
                valid_actions = full_chunk[:self.infer_engine.action_execution_horizon]
                print('valid actions:', valid_actions.shape)
                
                # TODO
                self.infer_engine.action_queue.clear()
                
                with self.queue_lock:
                    self.infer_engine.action_queue.extend(valid_actions)
                    
            except Exception as e:
                self.get_logger().error(f"Inference crashed: {e}")

    def publish_control_loop(self):
        """Foreground loop (10Hz): fetch commands from queue and publish to chassis."""
        v_x, w_z = 0.0, 0.0
        
        # If already arrived at target, bypass the queue and force zero velocity
        if self.arrived:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_publisher.publish(msg)
            return

        with self.queue_lock:
            if len(self.infer_engine.action_queue) > 0:
                action = self.infer_engine.action_queue.popleft()
                v_x, w_z = float(action[0]), float(action[1])
            else:
                if self.latest_scan_01 is not None:
                    self.get_logger().warning(
                        "Inference queue empty, executing safety brake.", 
                        throttle_duration_sec=1.0
                    )
        
        msg = Twist()
        msg.linear.x = v_x
        msg.angular.z = w_z
        self.cmd_publisher.publish(msg)
        
        with self.queue_lock:
            q_size = len(self.infer_engine.action_queue)
        self.get_logger().debug(f"Published v: {v_x:+.3f} | w: {w_z:+.3f} (Cache: {q_size})")

    def stop_robot(self):
        """Emergency stop function."""
        msg = Twist()
        self.cmd_publisher.publish(msg)
        self.get_logger().info("Robot emergency stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarVLAInferNode()
    
    # Configure MultiThreadedExecutor for concurrent processing on Jetson Orin
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt signal received, exiting...")
    except Exception as e:
        node.get_logger().error(f"Exception occurred: {e}")
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
