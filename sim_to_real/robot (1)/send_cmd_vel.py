import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse
import sys

class CmdVelGenerator(Node):
    def __init__(self, mode):
        super().__init__('cmd_vel_generator')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0
        self.max_frames = 20
        self.mode = mode
        
        # 根据命令行传入的模式初始化速度
        if self.mode == 'linear':
            self.linear_velocity = 0.5   # 线速度设为 0.5 m/s
            self.angular_velocity = 0.0
            self.get_logger().info('Starting in LINEAR velocity mode (前进/后退)')
        elif self.mode == 'angular':
            self.linear_velocity = 0.0
            self.angular_velocity = 0.5  # 角速度设为 0.5 rad/s
            self.get_logger().info('Starting in ANGULAR velocity mode (原地旋转)')

        self.get_logger().info('Publishing 20 frames at 10Hz...')

    def timer_callback(self):
        if self.frame_count < self.max_frames:
            msg = Twist()
            
            # 赋值
            msg.linear.x = self.linear_velocity
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.angular_velocity
            
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'Frame {self.frame_count + 1:02d}/{self.max_frames} -> '
                f'linear_x: {msg.linear.x}, angular_z: {msg.angular.z}'
            )
            
            self.frame_count += 1
        else:
            self.get_logger().info('20 frames completed. Shutting down node.')
            self.timer.cancel()
            
            # 发送全 0 停止指令，防止小车失控
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            
            raise SystemExit

def main():
    # 使用 argparse 解析我们自定义的命令行参数
    parser = argparse.ArgumentParser(description='ROS 2 cmd_vel generator with dual modes.')
    parser.add_argument(
        '--mode', 
        type=str, 
        choices=['linear', 'angular'], 
        default='angular',
        help='Choose control mode: "linear" or "angular" (default: angular)'
    )
    
    # parse_known_args 会分离出我们定义的参数，剩下的交给 ROS 2 底层去处理
    parsed_args, ros_args = parser.parse_known_args(sys.argv)
    
    # 初始化 ROS 2，剥离掉我们的自定义参数
    rclpy.init(args=ros_args)
    
    cmd_vel_node = CmdVelGenerator(mode=parsed_args.mode)
    
    try:
        rclpy.spin(cmd_vel_node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()