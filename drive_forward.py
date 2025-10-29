#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')
        
        # Create publisher for cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create timer to execute the movement sequence
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Movement state tracking
        self.start_time = time.time()
        self.movement_duration = 1.0  # 1 second
        self.has_moved = False
        self.has_stopped = False
        
        self.get_logger().info('Simple Driver initialized - will drive forward for 1 second then stop')
    
    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Create Twist message
        twist = Twist()
        
        if elapsed_time < self.movement_duration and not self.has_moved:
            # Drive forward at low speed
            twist.linear.x = 0.2  # Low speed forward (m/s)
            twist.angular.z = 0.0  # No rotation
            self.publisher.publish(twist)
            
            if not self.has_moved:
                self.get_logger().info(f'Driving forward at {twist.linear.x} m/s...')
                self.has_moved = True
                
        elif elapsed_time >= self.movement_duration and not self.has_stopped:
            # Stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            
            if not self.has_stopped:
                self.get_logger().info('Stopping robot...')
                self.has_stopped = True
                
        elif self.has_stopped:
            # Keep publishing stop command for a moment, then shutdown
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            
            if elapsed_time > self.movement_duration + 0.5:  # Wait 0.5 seconds after stopping
                self.get_logger().info('Movement sequence complete. Shutting down...')
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    driver = SimpleDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

