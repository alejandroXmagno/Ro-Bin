#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class WASDController(Node):
    def __init__(self):
        super().__init__('wasd_controller')
        
        # Create publisher for cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Get terminal settings for non-blocking input
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('WASD Controller initialized!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W = Forward')
        self.get_logger().info('  S = Backward')
        self.get_logger().info('  A = Turn Left')
        self.get_logger().info('  D = Turn Right')
        self.get_logger().info('  Space = Stop')
        self.get_logger().info('  Q = Quit')
        self.get_logger().info('')
        self.get_logger().info('Press any key to start...')
        
        # Start the control loop
        self.control_loop()
    
    def get_key(self):
        """Get a single keypress from stdin"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def control_loop(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'q' or key == 'Q':
                    self.get_logger().info('Quitting...')
                    break
                elif key == 'w' or key == 'W':
                    self.move_forward()
                elif key == 's' or key == 'S':
                    self.move_backward()
                elif key == 'a' or key == 'A':
                    self.turn_left()
                elif key == 'd' or key == 'D':
                    self.turn_right()
                elif key == ' ':
                    self.stop()
                elif key == '':
                    # No key pressed, continue
                    continue
                else:
                    self.get_logger().info(f'Unknown key: {key}')
                
                # Small delay to prevent overwhelming the system
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
        finally:
            self.stop()
            self.get_logger().info('Controller stopped')
    
    def move_forward(self):
        """Move forward"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info(f'Moving forward at {self.linear_speed} m/s')
    
    def move_backward(self):
        """Move backward"""
        twist = Twist()
        twist.linear.x = -self.linear_speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info(f'Moving backward at {self.linear_speed} m/s')
    
    def turn_left(self):
        """Turn left"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)
        self.get_logger().info(f'Turning left at {self.angular_speed} rad/s')
    
    def turn_right(self):
        """Turn right"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.angular_speed
        self.publisher.publish(twist)
        self.get_logger().info(f'Turning right at {self.angular_speed} rad/s')
    
    def stop(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Stopped')

def main(args=None):
    rclpy.init(args=args)
    
    controller = WASDController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
