#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import select
import termios
import tty
import math
import time

class WASDController(Node):
    def __init__(self):
        super().__init__('wasd_controller')
        
        # Create publisher for cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to odometry to get current velocities
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10
        )
        
        # Movement parameters (base target speeds)
        self.base_linear_speed = 0.5  # m/s (maximum base target)
        self.base_angular_speed = 1.0  # rad/s (maximum base target)
        
        # Throttle coefficients (like joys_manager throttle control)
        self.linear_throttle_coef = 1.0  # Multiplier for linear speed (0.0 to max)
        self.angular_throttle_coef = 1.0  # Multiplier for angular speed (0.0 to max)
        self.throttle_increment = 0.25  # Increment for throttle adjustment
        
        # Current effective speeds (base * throttle)
        self.linear_speed = self.base_linear_speed * self.linear_throttle_coef
        self.angular_speed = self.base_angular_speed * self.angular_throttle_coef
        
        # Acceleration limits (matching the robot driver)
        self.max_linear_acceleration = 5.0  # m/s^2 (LINEAR_JERK_LIMIT_)
        self.max_angular_acceleration = 30.0  # rad/s^2
        
        # Target velocities (what we want to achieve)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Current/measured velocities (from odometry) - used for acceleration limiting
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Previous command velocities (tracked but not used for limiting)
        self.command_linear_vel = 0.0
        self.command_angular_vel = 0.0
        
        # Time tracking for acceleration limiting
        self.last_update_time = time.time()
        
        # Control loop frequency (30 Hz, matching robot driver)
        self.control_frequency = 30.0  # Hz
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, 
            self.control_timer_callback
        )
        
        # Key press handling
        self.key_pressed = None
        self.key_lock = False  # Prevent rapid key spam
        
        # Get terminal settings for non-blocking input
        self.settings = termios.tcgetattr(sys.stdin)
        # Set terminal to raw mode once (will restore on shutdown)
        tty.setraw(sys.stdin.fileno())
        
        self.get_logger().info('WASD Controller initialized with acceleration limiting!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W = Forward')
        self.get_logger().info('  S = Backward')
        self.get_logger().info('  A = Turn Left')
        self.get_logger().info('  D = Turn Right')
        self.get_logger().info('  Space = Stop')
        self.get_logger().info('  +/- = Increase/Decrease Linear Speed Throttle')
        self.get_logger().info('  [/] = Increase/Decrease Angular Speed Throttle')
        self.get_logger().info('  Q = Quit')
        self.get_logger().info('')
        self.get_logger().info(f'Max linear acceleration: {self.max_linear_acceleration} m/s²')
        self.get_logger().info(f'Max angular acceleration: {self.max_angular_acceleration} rad/s²')
        self.get_logger().info('Controller ready! Use WASD keys to control.')
        self._update_speed_settings()
    
    def odom_callback(self, msg):
        """Update current velocities from odometry"""
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z
    
    def limit_acceleration(self, target_linear, target_angular, dt):
        """
        Limit acceleration to smoothly ramp from measured velocities to target velocities.
        This matches the behavior of limitAcceleration() in control.cpp, which uses
        measured_velocities (actual robot velocities) rather than command velocities.
        """
        # Calculate proposed accelerations from MEASURED velocities (matching robot driver)
        linear_accel = (target_linear - self.current_linear_vel) / dt if dt > 0 else 0.0
        angular_accel = (target_angular - self.current_angular_vel) / dt if dt > 0 else 0.0
        
        # Clip accelerations to maximum limits
        if abs(linear_accel) > self.max_linear_acceleration:
            linear_accel = math.copysign(
                self.max_linear_acceleration, linear_accel
            )
        
        if abs(angular_accel) > self.max_angular_acceleration:
            angular_accel = math.copysign(
                self.max_angular_acceleration, angular_accel
            )
        
        # Calculate new command velocities (smoothly ramped from measured velocities)
        new_linear_vel = self.current_linear_vel + linear_accel * dt
        new_angular_vel = self.current_angular_vel + angular_accel * dt
        
        return new_linear_vel, new_angular_vel
    
    def control_timer_callback(self):
        """Timer callback that runs at 30Hz to publish smooth velocity commands"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0:
            dt = 1.0 / self.control_frequency  # Fallback to expected dt
        self.last_update_time = current_time
        
        # Handle any pending key presses
        key = self.get_key()
        if key:
            self.handle_key(key)
        
        # Limit acceleration to smoothly ramp to target
        new_linear, new_angular = self.limit_acceleration(
            self.target_linear_vel, 
            self.target_angular_vel, 
            dt
        )
        
        # Update command velocities
        self.command_linear_vel = new_linear
        self.command_angular_vel = new_angular
        
        # Publish the smoothly ramped velocity command
        twist = Twist()
        twist.linear.x = self.command_linear_vel
        twist.angular.z = self.command_angular_vel
        self.publisher.publish(twist)
    
    def get_key(self):
        """Get a single keypress from stdin (non-blocking)"""
        try:
            # Terminal is already in raw mode, check for input with very short timeout
            # Using 0.001s timeout - short enough to be responsive, long enough to catch keys
            rlist, _, _ = select.select([sys.stdin], [], [], 0.001)
            if rlist:
                key = sys.stdin.read(1)
                return key
            else:
                return ''
        except (termios.error, OSError, ValueError):
            # Handle case where stdin is not a terminal or select fails
            return ''
    
    def _update_speed_settings(self, update_active_targets=False):
        """Update effective speeds based on throttle coefficients"""
        old_linear_speed = self.linear_speed
        old_angular_speed = self.angular_speed
        
        self.linear_speed = self.base_linear_speed * self.linear_throttle_coef
        self.angular_speed = self.base_angular_speed * self.angular_throttle_coef
        
        # Update active target velocities if we're currently moving
        if update_active_targets:
            if abs(self.target_linear_vel) > 0.001:
                # Scale target linear velocity proportionally
                if old_linear_speed > 0.001:
                    self.target_linear_vel = (self.target_linear_vel / old_linear_speed) * self.linear_speed
                else:
                    self.target_linear_vel = self.linear_speed if self.target_linear_vel > 0 else -self.linear_speed
            if abs(self.target_angular_vel) > 0.001:
                # Scale target angular velocity proportionally
                if old_angular_speed > 0.001:
                    self.target_angular_vel = (self.target_angular_vel / old_angular_speed) * self.angular_speed
                else:
                    self.target_angular_vel = self.angular_speed if self.target_angular_vel > 0 else -self.angular_speed
        
        self.get_logger().info(
            f'Speed settings - Linear: {self.linear_speed:.2f} m/s '
            f'(throttle: {self.linear_throttle_coef:.2f}), '
            f'Angular: {self.angular_speed:.2f} rad/s '
            f'(throttle: {self.angular_throttle_coef:.2f})'
        )
    
    def handle_key(self, key):
        """Handle key presses and update target velocities"""
        if self.key_lock:
            return
        
        if key == 'q' or key == 'Q':
            self.get_logger().info('Quitting...')
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            # Restore terminal before shutdown
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except (termios.error, OSError):
                pass
            rclpy.shutdown()
            return
        elif key == 'w' or key == 'W':
            self.target_linear_vel = self.linear_speed
            self.target_angular_vel = 0.0
            self.get_logger().info(f'Target: Forward at {self.linear_speed:.2f} m/s')
        elif key == 's' or key == 'S':
            self.target_linear_vel = -self.linear_speed
            self.target_angular_vel = 0.0
            self.get_logger().info(f'Target: Backward at {self.linear_speed:.2f} m/s')
        elif key == 'a' or key == 'A':
            self.target_linear_vel = 0.0
            self.target_angular_vel = self.angular_speed
            self.get_logger().info(f'Target: Turn left at {self.angular_speed:.2f} rad/s')
        elif key == 'd' or key == 'D':
            self.target_linear_vel = 0.0
            self.target_angular_vel = -self.angular_speed
            self.get_logger().info(f'Target: Turn right at {self.angular_speed:.2f} rad/s')
        elif key == ' ':
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.get_logger().info('Target: Stop')
        elif key == '+' or key == '=':
            # Increase linear speed throttle
            self.linear_throttle_coef += self.throttle_increment
            if self.linear_throttle_coef > 2.0:  # Cap at 2x
                self.linear_throttle_coef = 2.0
            self._update_speed_settings(update_active_targets=True)
        elif key == '-' or key == '_':
            # Decrease linear speed throttle
            self.linear_throttle_coef -= self.throttle_increment
            if self.linear_throttle_coef < 0.0:
                self.linear_throttle_coef = 0.0
            self._update_speed_settings(update_active_targets=True)
        elif key == ']' or key == '}':
            # Increase angular speed throttle
            self.angular_throttle_coef += self.throttle_increment
            if self.angular_throttle_coef > 2.0:  # Cap at 2x
                self.angular_throttle_coef = 2.0
            self._update_speed_settings(update_active_targets=True)
        elif key == '[' or key == '{':
            # Decrease angular speed throttle
            self.angular_throttle_coef -= self.throttle_increment
            if self.angular_throttle_coef < 0.0:
                self.angular_throttle_coef = 0.0
            self._update_speed_settings(update_active_targets=True)

def main(args=None):
    rclpy.init(args=args)
    
    controller = WASDController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        # Restore terminal settings before shutting down
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, controller.settings)
        except (termios.error, OSError):
            pass
        
        # Send stop command before shutting down
        controller.target_linear_vel = 0.0
        controller.target_angular_vel = 0.0
        for _ in range(10):
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            controller.publisher.publish(twist)
            time.sleep(0.05)
        controller.get_logger().info('Controller stopped')
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
