#!/usr/bin/env python3

"""
Autonomous Exploration Node for Mini Rover 2WD
This node implements frontier-based exploration combined with random navigation
to autonomously explore and map an unknown environment while avoiding obstacles.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from collections import deque
import random


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Parameters
        self.declare_parameter('exploration_radius', 8.0)  # Increased to look further ahead
        self.declare_parameter('frontier_threshold', 10)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('goal_timeout', 45.0)  # Increased timeout to reach distant goals
        self.declare_parameter('obstacle_distance_threshold', 0.5)
        self.declare_parameter('random_exploration_probability', 0.3)
        
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.frontier_threshold = self.get_parameter('frontier_threshold').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.random_prob = self.get_parameter('random_exploration_probability').value
        
        # State variables
        self.current_pose = None
        self.map_data = None
        self.map_info = None
        self.goal_active = False
        self.last_goal_time = None
        self.explored_frontiers = []
        self.scan_data = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timers
        self.exploration_timer = self.create_timer(2.0, self.exploration_callback)
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        
        self.get_logger().info('Autonomous Explorer initialized')
        
    def map_callback(self, msg):
        """Store the latest map data"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
    def odom_callback(self, msg):
        """Store current robot pose"""
        self.current_pose = msg.pose.pose
        
    def scan_callback(self, msg):
        """Store laser scan data"""
        self.scan_data = msg
        
    def safety_check(self):
        """Emergency stop if obstacle is too close"""
        if self.scan_data is None:
            return
            
        min_distance = min([r for r in self.scan_data.ranges if not math.isinf(r) and not math.isnan(r)], default=float('inf'))
        
        if min_distance < self.obstacle_threshold and self.goal_active:
            self.get_logger().warn(f'Obstacle too close: {min_distance:.2f}m - Emergency stop!')
            self.cancel_current_goal()
            
    def exploration_callback(self):
        """Main exploration logic - called periodically"""
        if self.map_data is None or self.current_pose is None:
            self.get_logger().info('Waiting for map and pose data...')
            return
            
        # Check if we need a new goal
        if not self.goal_active:
            self.get_logger().info('Searching for new exploration goal...')
            self.find_and_navigate_to_goal()
        else:
            # Check if goal timed out
            if self.last_goal_time is not None:
                elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout:
                    self.get_logger().warn(f'Goal timeout ({elapsed:.1f}s) - Finding new goal')
                    self.cancel_current_goal()
                    
    def find_and_navigate_to_goal(self):
        """Find a new exploration goal and navigate to it - FRONTIER-BASED with random fallback"""
        # Try frontier-based exploration first (prioritizes unexplored areas)
        goal = self.get_frontier_goal()
        
        if goal is not None:
            distance = math.sqrt(
                (goal[0] - self.current_pose.position.x)**2 +
                (goal[1] - self.current_pose.position.y)**2
            )
            self.get_logger().info(f'🗺️  Frontier goal at ({goal[0]:.2f}, {goal[1]:.2f}) - {distance:.2f}m away')
            self.navigate_to_goal(goal)
        else:
            # Fallback to random exploration if no frontiers found
            self.get_logger().info('No frontiers found, trying random exploration...')
            goal = self.get_random_exploration_goal()
            if goal is not None:
                distance = math.sqrt(
                    (goal[0] - self.current_pose.position.x)**2 +
                    (goal[1] - self.current_pose.position.y)**2
                )
                self.get_logger().info(f'🎲 Random goal at ({goal[0]:.2f}, {goal[1]:.2f}) - {distance:.2f}m away')
                self.navigate_to_goal(goal)
            else:
                self.get_logger().warn('Could not find any exploration goal!')
                
    def get_frontier_goal(self):
        """Find frontier cells (boundary between known and unknown space) and prioritize larger unexplored regions"""
        if self.map_data is None or self.map_info is None or self.current_pose is None:
            return None
            
        height, width = self.map_data.shape
        frontiers = []
        
        # Find frontier cells with better scoring
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # Check if cell is free (value 0)
                if self.map_data[y, x] == 0:
                    # Check if any neighbor is unknown (value -1)
                    neighbors = [
                        self.map_data[y-1, x], self.map_data[y+1, x],
                        self.map_data[y, x-1], self.map_data[y, x+1]
                    ]
                    if -1 in neighbors:
                        # Convert grid coordinates to world coordinates
                        world_x = x * self.map_info.resolution + self.map_info.origin.position.x
                        world_y = y * self.map_info.resolution + self.map_info.origin.position.y
                        
                        # Calculate distance from robot
                        dist = math.sqrt(
                            (world_x - self.current_pose.position.x)**2 +
                            (world_y - self.current_pose.position.y)**2
                        )
                        
                        # Include frontiers within exploration radius
                        if dist < self.exploration_radius:
                            # Count nearby unknown cells (measure of unexplored area size)
                            unknown_count = self.count_nearby_unknown(x, y, radius=3)
                            frontiers.append((world_x, world_y, dist, unknown_count))
        
        if not frontiers:
            return None
            
        # Filter out recently explored frontiers
        filtered_frontiers = []
        for frontier in frontiers:
            if not self.is_recently_explored(frontier, threshold=1.5):
                filtered_frontiers.append(frontier)
        
        if not filtered_frontiers:
            self.get_logger().info('All frontiers recently explored, clearing history...')
            self.explored_frontiers = self.explored_frontiers[-10:]  # Keep only last 10
            filtered_frontiers = frontiers  # Use all frontiers
        
        if len(filtered_frontiers) >= self.min_frontier_size:
            # Score frontiers: prioritize larger unexplored areas, with distance as secondary factor
            # Score = unknown_count * 2.0 - distance * 0.5 (higher score = better)
            scored_frontiers = []
            for f in filtered_frontiers:
                world_x, world_y, dist, unknown_count = f
                score = unknown_count * 2.0 - dist * 0.5
                scored_frontiers.append((world_x, world_y, dist, unknown_count, score))
            
            # Sort by score (highest first)
            scored_frontiers.sort(key=lambda f: f[4], reverse=True)
            
            # Take the best frontier
            best = scored_frontiers[0]
            goal = (best[0], best[1])
            
            # Add to explored frontiers history
            self.explored_frontiers.append(goal + (best[2],))
            # Keep last 100 explored frontiers for better memory
            if len(self.explored_frontiers) > 100:
                self.explored_frontiers.pop(0)
                
            return goal
        
        return None
        
    def get_random_exploration_goal(self):
        """Generate a random exploration goal in free space"""
        if self.map_data is None or self.map_info is None or self.current_pose is None:
            return None
            
        # Try multiple random points
        for _ in range(50):
            # Generate random angle and distance
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(1.0, self.exploration_radius)
            
            # Calculate world coordinates
            world_x = self.current_pose.position.x + distance * math.cos(angle)
            world_y = self.current_pose.position.y + distance * math.sin(angle)
            
            # Convert to grid coordinates
            grid_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
            grid_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)
            
            # Check if valid
            if (0 <= grid_x < self.map_data.shape[1] and 
                0 <= grid_y < self.map_data.shape[0]):
                # Check if free space (value 0)
                if self.map_data[grid_y, grid_x] == 0:
                    return (world_x, world_y)
                    
        return None
        
    def count_nearby_unknown(self, grid_x, grid_y, radius=3):
        """Count unknown cells near a frontier point to estimate unexplored area size"""
        if self.map_data is None:
            return 0
        
        height, width = self.map_data.shape
        unknown_count = 0
        
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                ny, nx = grid_y + dy, grid_x + dx
                if 0 <= ny < height and 0 <= nx < width:
                    if self.map_data[ny, nx] == -1:  # Unknown cell
                        unknown_count += 1
        
        return unknown_count
    
    def is_recently_explored(self, frontier, threshold=1.5):
        """Check if a frontier was recently explored (with larger threshold to avoid revisiting)"""
        for explored in self.explored_frontiers[-30:]:  # Check last 30 instead of 10
            dist = math.sqrt(
                (frontier[0] - explored[0])**2 +
                (frontier[1] - explored[1])**2
            )
            if dist < threshold:
                return True
        return False
        
    def navigate_to_goal(self, goal):
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            return
            
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        
        # Set orientation towards goal
        if self.current_pose is not None:
            dx = goal[0] - self.current_pose.position.x
            dy = goal[1] - self.current_pose.position.y
            yaw = math.atan2(dy, dx)
            
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_pose.pose.orientation.w = 1.0
            
        # Create action goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        # Send goal
        self.goal_active = True
        self.last_goal_time = self.get_clock().now()
        
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by navigation server')
            self.goal_active = False
            return
            
        self.get_logger().info('Goal accepted by navigation server')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
        
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Can add progress monitoring here if needed
        pass
        
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.goal_active = False
        self.last_goal_time = None
        
        self.get_logger().info('Navigation goal completed!')
        
    def cancel_current_goal(self):
        """Cancel the current navigation goal"""
        if self.goal_active:
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.goal_active = False
            self.last_goal_time = None
            self.get_logger().info('Current goal cancelled')


def main(args=None):
    rclpy.init(args=args)
    explorer = AutonomousExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



