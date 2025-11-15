#!/usr/bin/env python3
"""
Visual Servoing Person Tracker
State machine that directly controls robot to approach waving people
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import math
from enum import Enum

class RobotState(Enum):
    EXPLORING = 1           # Autonomous navigation active
    TRACKING_PERSON = 2     # Person detected, turning to center them
    APPROACHING_PERSON = 3  # Moving toward person

class VisualServoingTracker(Node):
    def __init__(self):
        super().__init__('visual_servoing_tracker')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.person_sub = self.create_subscription(
            String,
            '/person_detection/results',
            self.person_callback,
            10
        )
        
        # State machine
        self.state = RobotState.EXPLORING
        self.person_detected = False
        self.person_x = 0.5  # Normalized X position (0=left, 1=right, 0.5=center)
        self.person_depth = None  # Depth in meters
        self.last_detection_time = None
        
        # Control parameters
        self.target_distance = 0.3  # Stop 0.3m (1 foot) from person
        self.center_tolerance = 0.25  # Very relaxed - just get roughly centered
        self.approach_speed = 0.25  # m/s - slower for better control
        self.turn_speed = 0.4  # rad/s - gentler turns
        self.approach_time = 0.0  # Track how long we've been approaching
        
        # Control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('🤖 Visual Servoing Tracker initialized')
        self.get_logger().info('   Waiting for waving people...')
        
    def person_callback(self, msg):
        """Handle person detection with depth"""
        try:
            data = json.loads(msg.data)
            
            if data.get('person_detected') and data.get('is_waving'):
                self.person_detected = True
                self.last_detection_time = self.get_clock().now()
                
                person_center = data.get('person_center', {})
                self.person_x = person_center.get('x', 0.5)
                self.person_depth = person_center.get('depth')
                
                # State transitions
                if self.state == RobotState.EXPLORING:
                    self.state = RobotState.TRACKING_PERSON
                    self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                    self.get_logger().info('👋 WAVING PERSON DETECTED!')
                    self.get_logger().info(f'   Position: x={self.person_x:.2f}')
                    depth_str = f'{self.person_depth:.2f}m' if self.person_depth and self.person_depth > 0.1 else 'invalid'
                    self.get_logger().info(f'   Depth: {depth_str}')
                    self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            else:
                # Check if we lost the person
                if self.person_detected and self.last_detection_time:
                    time_since = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                    if time_since > 2.0:  # Lost for 2 seconds
                        if self.state in [RobotState.TRACKING_PERSON, RobotState.APPROACHING_PERSON]:
                            self.get_logger().warn('⚠️  Lost sight of person, resuming exploration')
                            self.state = RobotState.EXPLORING
                            self.person_detected = False
                            self.stop_robot()
                        
        except Exception as e:
            self.get_logger().error(f'Error parsing detection: {e}')
    
    def control_loop(self):
        """Main control loop - state machine"""
        
        if self.state == RobotState.EXPLORING:
            # Do nothing, let autonomous navigator control
            pass
            
        elif self.state == RobotState.TRACKING_PERSON:
            self.track_person()
            
        elif self.state == RobotState.APPROACHING_PERSON:
            self.approach_person()
    
    def track_person(self):
        """Quickly center person and start approaching"""
        if not self.person_detected:
            return
        
        # Just turn briefly, then start approaching
        # Don't wait for perfect centering!
        error = self.person_x - 0.5
        
        if abs(error) > 0.35:
            # Only turn if VERY far from center
            twist = Twist()
            twist.angular.z = -error * self.turn_speed * 2.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'🔄 Quick turn: error={error:.2f}')
        else:
            # Good enough! Start approaching immediately
            self.get_logger().info(f'✅ Roughly centered (error={error:.2f}), starting approach!')
            self.state = RobotState.APPROACHING_PERSON
            self.approach_time = 0.0
            self.stop_robot()
    
    def approach_person(self):
        """Move toward person while keeping them centered"""
        if not self.person_detected:
            return
        
        self.approach_time += 0.1  # 10Hz control loop
        
        twist = Twist()
        error = self.person_x - 0.5
        
        # Check if we've reached the person
        # Use depth if valid (> 0.1m), otherwise use time-based approach
        if self.person_depth and self.person_depth > 0.1:
            if self.person_depth <= self.target_distance:
                # Reached person! Resume exploration immediately
                self.get_logger().info(f'🎉 REACHED PERSON at {self.person_depth:.2f}m! Looking for next person...')
                self.stop_robot()
                self.state = RobotState.EXPLORING
                self.person_detected = False
                self.approach_time = 0.0
                return
        else:
            # Invalid depth, use time-based approach (8 seconds of forward motion)
            if self.approach_time > 8.0:
                self.get_logger().info(f'🎉 REACHED PERSON (time-based: {self.approach_time:.1f}s)! Looking for next person...')
                self.stop_robot()
                self.state = RobotState.EXPLORING
                self.person_detected = False
                self.approach_time = 0.0
                return
        
        # Move forward with gentle steering correction
        twist.linear.x = self.approach_speed
        
        # Only correct if significantly off-center
        if abs(error) > 0.15:
            twist.angular.z = -error * self.turn_speed * 1.5
        
        self.cmd_vel_pub.publish(twist)
        
        # Log progress occasionally
        if int(self.approach_time) % 2 == 0 and self.approach_time % 2 < 0.15:
            depth_str = f'{self.person_depth:.2f}m' if (self.person_depth and self.person_depth > 0.1) else 'invalid'
            self.get_logger().info(f'🚶 Approaching: time={self.approach_time:.1f}s, depth={depth_str}, error={error:.2f}')
    
    def stop_robot(self):
        """Stop all motion"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
