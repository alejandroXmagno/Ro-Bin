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
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
import numpy as np
import math
from collections import deque
import random
import json


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Parameters
        self.declare_parameter('exploration_radius', 8.0)  # Increased to look further ahead
        self.declare_parameter('frontier_threshold', 10)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('goal_timeout', 120.0)  # Much longer timeout for distant frontiers
        self.declare_parameter('obstacle_distance_threshold', 0.8)  # Cancel if obstacle within 0.8m
        self.declare_parameter('critical_obstacle_threshold', 0.5)  # Emergency stop within 0.5m
        self.declare_parameter('random_exploration_probability', 0.3)
        self.declare_parameter('min_goal_interval', 3.0)  # Minimum time between goal completions
        self.declare_parameter('person_tracking_enabled', True)  # Enable person tracking
        self.declare_parameter('person_approach_distance', 1.5)  # Stop 1.5m from person
        
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.frontier_threshold = self.get_parameter('frontier_threshold').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.critical_obstacle_threshold = self.get_parameter('critical_obstacle_threshold').value
        self.random_prob = self.get_parameter('random_exploration_probability').value
        self.min_goal_interval = self.get_parameter('min_goal_interval').value
        self.person_tracking_enabled = self.get_parameter('person_tracking_enabled').value
        self.person_approach_distance = self.get_parameter('person_approach_distance').value
        
        # State variables
        self.current_pose = None
        self.map_data = None
        self.map_info = None
        self.goal_active = False
        self.last_goal_time = None
        self.last_goal_completion_time = None
        self.explored_frontiers = []
        self.scan_data = None
        self.obstacle_count = 0  # Counter for sustained obstacles
        self.current_goal_distance = 0.0  # Track distance to current goal
        
        # Stuck detection
        self.last_positions = deque(maxlen=20)  # Track last 20 positions (10 seconds)
        self.stuck_count = 0
        self.blacklisted_goals = []  # Goals that led to stuck situations
        self.recovery_in_progress = False
        
        # Person tracking
        self.person_detected = False
        self.person_position = None  # (x, y) in image coordinates
        self.last_person_detection_time = None
        self.tracking_person = False
        self.waiting_by_person = False
        self.person_wait_start_time = None
        self.person_wait_duration = 10.0  # Wait 10 seconds by person
        
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
        
        self.person_detection_sub = self.create_subscription(
            String,
            '/person_detection/results',
            self.person_detection_callback,
            10
        )
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timers
        self.exploration_timer = self.create_timer(2.0, self.exploration_callback)
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        self.stuck_check_timer = self.create_timer(0.5, self.check_if_stuck)
        
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
    
    def person_detection_callback(self, msg):
        """Handle person detection results"""
        if not self.person_tracking_enabled:
            return
        
        try:
            detection_data = json.loads(msg.data)
            
            if detection_data.get('person_detected', False):
                self.person_detected = True
                self.last_person_detection_time = self.get_clock().now()
                
                # Extract person center position
                person_center = detection_data.get('person_center')
                if person_center:
                    self.person_position = (
                        person_center.get('x', 0.5),  # Normalized x
                        person_center.get('y', 0.5)   # Normalized y
                    )
                    
                    if not self.tracking_person:
                        self.get_logger().info(f'👤 Person detected at ({self.person_position[0]:.2f}, {self.person_position[1]:.2f})!')
                        self.tracking_person = True
            else:
                # Clear detection flag immediately
                self.person_detected = False
                
                # But keep tracking for a bit to handle brief occlusions
                current_time = self.get_clock().now()
                if self.last_person_detection_time:
                    time_since_detection = (current_time - self.last_person_detection_time).nanoseconds / 1e9
                    # Stop tracking if no person seen for 2 seconds
                    if time_since_detection > 2.0:
                        if self.tracking_person:
                            self.get_logger().info('Person lost, resuming exploration')
                        self.tracking_person = False
                else:
                    # No detection time recorded, clear tracking immediately
                    self.tracking_person = False
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse person detection data: {e}')
        
    def safety_check(self):
        """Two-level obstacle detection: critical (immediate) and warning (sustained)"""
        if self.scan_data is None or not self.goal_active:
            self.obstacle_count = 0
            return
            
        # Get minimum distance to obstacles
        valid_ranges = [r for r in self.scan_data.ranges if not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges:
            return
        min_distance = min(valid_ranges)
        
        # CRITICAL: Immediate cancel if very close (< 0.5m)
        if min_distance < self.critical_obstacle_threshold:
            self.get_logger().warn(f'⛔ CRITICAL: Obstacle at {min_distance:.2f}m! Emergency cancel!')
            self.cancel_current_goal()
            self.obstacle_count = 0
            return
        
        # WARNING: Cancel if obstacle within 0.8m for 1 second (2 checks)
        if min_distance < self.obstacle_threshold:
            self.obstacle_count += 1
            if self.obstacle_count >= 2:  # Reduced from 3 to 2 (1 second instead of 1.5)
                self.get_logger().warn(f'⚠️  Obstacle at {min_distance:.2f}m - Cancelling to avoid collision!')
                self.cancel_current_goal()
                self.obstacle_count = 0
        else:
            # Reset counter if obstacle clears
            self.obstacle_count = 0
    
    def check_if_stuck(self):
        """Check if robot is stuck (not making progress)"""
        if self.current_pose is None or not self.goal_active or self.recovery_in_progress:
            return
        
        # Record current position
        current_pos = (self.current_pose.position.x, self.current_pose.position.y)
        self.last_positions.append(current_pos)
        
        # Need at least 8 positions (4 seconds) to detect stuck - reduced from 10
        if len(self.last_positions) < 8:
            return
        
        # Calculate total distance moved in last 4 seconds
        total_movement = 0.0
        for i in range(1, len(self.last_positions)):
            dx = self.last_positions[i][0] - self.last_positions[i-1][0]
            dy = self.last_positions[i][1] - self.last_positions[i-1][1]
            total_movement += math.sqrt(dx*dx + dy*dy)
        
        # More aggressive: If moved less than 0.3m in 4 seconds, likely stuck
        # Increased from 0.2m to catch slower stuck situations
        if total_movement < 0.3:
            self.stuck_count += 1
            if self.stuck_count >= 3:  # Stuck for 1.5 seconds (reduced from 5)
                self.get_logger().warn(f'🚨 Robot appears stuck! Moved only {total_movement:.3f}m in 4s')
                self.initiate_recovery()
                self.stuck_count = 0
        else:
            # Reset if making progress
            self.stuck_count = 0
            
    def exploration_callback(self):
        """Main exploration logic - called periodically"""
        if self.map_data is None or self.current_pose is None:
            self.get_logger().info('Waiting for map and pose data...')
            return
        
        # Skip if recovery in progress
        if self.recovery_in_progress:
            return
        
        # Check if we're waiting by a person
        if self.waiting_by_person:
            # Safety check: if wait time was never set, exit waiting state
            if self.person_wait_start_time is None:
                self.get_logger().warn('⚠️  Waiting by person but no start time set, clearing state')
                self.waiting_by_person = False
                self.tracking_person = False
                return
                
            elapsed = (self.get_clock().now() - self.person_wait_start_time).nanoseconds / 1e9
            if elapsed < self.person_wait_duration:
                # Still waiting, log every 2 seconds
                if int(elapsed) % 2 == 0 and elapsed % 2 < 0.5:
                    remaining = self.person_wait_duration - elapsed
                    self.get_logger().info(f'⏳ Waiting by person... {remaining:.0f}s remaining')
                return
            else:
                # Done waiting, look for another person
                self.get_logger().info('✅ Wait complete! Looking for another person...')
                self.waiting_by_person = False
                self.person_wait_start_time = None
                self.tracking_person = False
                self.person_detected = False
                # Continue to explore/find next person
        
        # PRIORITY: Track person if detected (and not already waiting)
        if self.person_detected and self.person_tracking_enabled and not self.waiting_by_person:
            self.navigate_to_person()
            return
            
        # Check if we need a new goal
        if not self.goal_active:
            # Check if enough time has passed since last goal completion
            if self.last_goal_completion_time is not None:
                time_since_completion = (self.get_clock().now() - self.last_goal_completion_time).nanoseconds / 1e9
                if time_since_completion < self.min_goal_interval:
                    # Too soon after last goal, wait longer
                    return
            
            self.get_logger().info('Searching for new exploration goal...')
            self.find_and_navigate_to_goal()
        else:
            # Check if goal timed out (only after significant time)
            if self.last_goal_time is not None:
                elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout:
                    self.get_logger().warn(f'Goal timeout ({elapsed:.1f}s) - Finding new goal')
                    self.cancel_current_goal()
                elif elapsed > 10.0 and elapsed % 30.0 < 2.0:  # Log progress every 30 seconds
                    self.get_logger().info(f'Goal in progress... ({elapsed:.0f}s elapsed)')
                    
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
    
    def navigate_to_person(self):
        """Navigate towards detected person"""
        if not self.person_position or self.current_pose is None:
            return
        
        # Calculate direction to person based on camera view
        # person_position[0] is normalized x (0 = left, 1 = right)
        # Camera is side-facing (90° to robot's forward direction)
        
        # Since camera faces to the side (perpendicular to robot):
        # - If person is on left side of image (x < 0.5), they're ahead of the camera
        # - If person is on right side of image (x > 0.5), they're behind the camera
        
        person_x_norm = self.person_position[0]  # 0 to 1
        
        # Calculate robot's current orientation
        from tf_transformations import euler_from_quaternion
        orientation = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # Camera faces 90° to the right (perpendicular to forward)
        # Estimate person direction relative to robot
        # person_x_norm < 0.5 means person is in front half of camera view (ahead)
        # person_x_norm > 0.5 means person is in back half of camera view (behind)
        
        # Offset angle based on person position in camera frame
        # Map [0, 1] to approximately [-45°, +45°] relative to camera direction
        angle_offset_in_camera = (person_x_norm - 0.5) * (math.pi / 2)  # -π/4 to +π/4
        
        # Camera is at 90° (π/2) to robot's forward direction
        camera_angle = yaw + math.pi / 2
        person_angle = camera_angle + angle_offset_in_camera
        
        # Estimate distance (simplified - assume 2 meters if we can see them)
        estimated_distance = 2.0
        
        # Calculate goal position
        goal_x = self.current_pose.position.x + (estimated_distance - self.person_approach_distance) * math.cos(person_angle)
        goal_y = self.current_pose.position.y + (estimated_distance - self.person_approach_distance) * math.sin(person_angle)
        
        # Cancel current goal if tracking new person
        if self.goal_active and not self.tracking_person:
            self.cancel_current_goal()
        
        # Navigate to person location (if not already there)
        distance_to_goal = math.sqrt(
            (goal_x - self.current_pose.position.x)**2 +
            (goal_y - self.current_pose.position.y)**2
        )
        
        # Only send new goal if not already at person or if goal changed significantly
        if distance_to_goal > 0.3:  # More than 30cm away
            self.get_logger().info(f'🚶 Navigating towards person at ({goal_x:.2f}, {goal_y:.2f})')
            self.navigate_to_goal((goal_x, goal_y))
        else:
            # Already near the person - start waiting
            if not self.waiting_by_person:
                self.get_logger().info(f'👋 Reached person! Waiting {self.person_wait_duration:.0f} seconds...')
                self.waiting_by_person = True
                self.person_wait_start_time = self.get_clock().now()
                # Cancel any active navigation goal
                if self.goal_active:
                    self.cancel_current_goal()
                
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
            
        # Filter out recently explored frontiers AND blacklisted areas
        filtered_frontiers = []
        for frontier in frontiers:
            if not self.is_recently_explored(frontier, threshold=1.5) and not self.is_near_blacklisted(frontier):
                filtered_frontiers.append(frontier)
        
        if not filtered_frontiers:
            self.get_logger().info('All frontiers recently explored or blacklisted, clearing history...')
            self.explored_frontiers = self.explored_frontiers[-10:]  # Keep only last 10
            # Try again without blacklist if desperate
            filtered_frontiers = [f for f in frontiers if not self.is_recently_explored(f, threshold=1.5)]
            if not filtered_frontiers:
                filtered_frontiers = frontiers  # Use all frontiers as last resort
        
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
    
    def is_near_blacklisted(self, frontier, threshold=3.0):
        """Check if a frontier is near a blacklisted (stuck) location"""
        # Increased threshold from 2.0m to 3.0m for wider avoidance
        for blacklisted in self.blacklisted_goals:
            dist = math.sqrt(
                (frontier[0] - blacklisted[0])**2 +
                (frontier[1] - blacklisted[1])**2
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
        
        # Calculate and store goal distance
        if self.current_pose is not None:
            dx = goal[0] - self.current_pose.position.x
            dy = goal[1] - self.current_pose.position.y
            self.current_goal_distance = math.sqrt(dx*dx + dy*dy)
            yaw = math.atan2(dy, dx)
            
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            self.current_goal_distance = 0.0
            goal_pose.pose.orientation.w = 1.0
            
        # Create action goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        # Send goal
        self.goal_active = True
        self.last_goal_time = self.get_clock().now()
        self.obstacle_count = 0  # Reset obstacle counter for new goal
        
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
        
        # Calculate how long this goal took
        if self.last_goal_time is not None:
            elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
            self.get_logger().info(f'✅ Navigation goal completed! ({elapsed:.1f}s, {self.current_goal_distance:.2f}m)')
        else:
            self.get_logger().info('✅ Navigation goal completed!')
        
        # Mark goal as inactive and record completion time
        self.goal_active = False
        self.last_goal_time = None
        self.last_goal_completion_time = self.get_clock().now()
        self.obstacle_count = 0
        
    def initiate_recovery(self):
        """Recovery behavior when robot is stuck"""
        self.recovery_in_progress = True
        self.get_logger().info('🔄 Initiating recovery behavior...')
        
        # Blacklist current goal location
        if self.current_pose is not None:
            stuck_location = (self.current_pose.position.x, self.current_pose.position.y)
            self.blacklisted_goals.append(stuck_location)
            # Keep only last 20 blacklisted locations
            if len(self.blacklisted_goals) > 20:
                self.blacklisted_goals.pop(0)
            self.get_logger().info(f'Blacklisted location: ({stuck_location[0]:.2f}, {stuck_location[1]:.2f})')
        
        # Cancel current goal
        self.cancel_current_goal()
        
        # Execute recovery: back up and rotate
        self.execute_recovery_maneuver()
        
    def execute_recovery_maneuver(self):
        """Execute aggressive recovery maneuver: back up and rotate significantly"""
        self.get_logger().info('⬅️ Backing up aggressively...')
        
        # Back up for 3 seconds at higher speed
        backup_cmd = Twist()
        backup_cmd.linear.x = -0.3  # Back up faster (was -0.2)
        for _ in range(30):  # 3 seconds at 10Hz (was 20)
            self.cmd_vel_pub.publish(backup_cmd)
            
        # Stop
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Rotate 135 degrees (more than 90)
        self.get_logger().info('🔄 Rotating significantly to find clear path...')
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.6  # Rotate faster (was 0.5)
        for _ in range(22):  # ~2.2 seconds for 135° (was 15)
            self.cmd_vel_pub.publish(rotate_cmd)
        
        # Stop
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.get_logger().info('✅ Recovery maneuver complete')
        self.recovery_in_progress = False
        self.last_positions.clear()  # Clear position history
    
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



