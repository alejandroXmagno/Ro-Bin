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
        self.declare_parameter('exploration_radius', 8.0)
        self.declare_parameter('frontier_threshold', 10)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('obstacle_distance_threshold', 0.35)  # Reduced from 0.8m to 0.35m
        self.declare_parameter('critical_obstacle_threshold', 0.25)  # Reduced from 0.5m to 0.25m
        self.declare_parameter('random_exploration_probability', 0.3)
        self.declare_parameter('min_goal_interval', 3.0)
        self.declare_parameter('person_tracking_enabled', True)
        self.declare_parameter('person_approach_distance', 0.3)  # Stop 1 foot (~0.3m) from person
        self.declare_parameter('safety_checks_enabled', True)  # Allow disabling safety checks for tight spaces
        
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
        self.safety_checks_enabled = self.get_parameter('safety_checks_enabled').value
        
        # State variables
        self.current_pose = None
        self.map_data = None
        self.map_info = None
        self.goal_active = False
        self.goal_handle = None  # Track the current goal handle for proper cancellation
        self.last_goal_time = None
        self.last_goal_completion_time = None
        self.explored_frontiers = []
        self.scan_data = None
        self.obstacle_count = 0
        self.current_goal_distance = 0.0
        
        # Stuck detection
        self.last_positions = deque(maxlen=30)  # Increased from 20 to track 15 seconds at 0.5Hz
        self.stuck_count = 0
        self.blacklisted_goals = []
        self.recovery_in_progress = False
        
        # Person tracking - SIMPLIFIED
        self.person_detected = False
        self.person_position = None
        self.person_depth = None
        self.last_person_detection_time = None
        self.tracking_person = False
        self.waiting_by_person = False
        self.turning_away = False
        self.turn_away_start_yaw = None
        self.person_wait_start_time = None
        self.person_wait_duration = 10.0
        self.last_person_goal = None  # Track last goal sent to avoid redundant updates
        self.last_person_goal_time = None
        
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
        self.exploration_timer = self.create_timer(0.1, self.exploration_callback)
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
        """Handle person detection results - only track waving people"""
        if not self.person_tracking_enabled:
            return
        
        try:
            detection_data = json.loads(msg.data)
            
            person_det = detection_data.get('person_detected', False)
            is_waving = detection_data.get('is_waving', False)
            
            # Only respond to people who are waving
            if person_det and is_waving:
                # IMMEDIATELY cancel any exploration goal on first detection
                if not self.tracking_person and self.goal_active:
                    self.get_logger().info('🚨🚨🚨 WAVING PERSON DETECTED! CANCELING ALL EXPLORATION! 🚨🚨🚨')
                    self.cancel_current_goal()
                
                self.person_detected = True
                self.tracking_person = True
                self.last_person_detection_time = self.get_clock().now()
                
                # Extract person center position and depth
                person_center = detection_data.get('person_center')
                if person_center:
                    self.person_position = (
                        person_center.get('x', 0.5),
                        person_center.get('y', 0.5)
                    )
                    self.person_depth = person_center.get('depth', None)
                    
                    depth_str = f"{self.person_depth:.2f}m" if self.person_depth else "unknown"
                    self.get_logger().info(f'👁️ Person position updated: x={self.person_position[0]:.2f}, depth={depth_str}')
            else:
                # Only clear if we've been without detection for a while
                if self.person_detected and self.last_person_detection_time:
                    time_since = (self.get_clock().now() - self.last_person_detection_time).nanoseconds / 1e9
                    if time_since > 3.0:  # Increased tolerance to 3 seconds
                        if not self.waiting_by_person and not self.turning_away:
                            self.get_logger().info('❌ Lost person for 3+ seconds, resuming exploration')
                            self.person_detected = False
                            self.tracking_person = False
                            self.person_position = None
                            self.person_depth = None
                            self.last_person_goal = None  # Clear goal tracking
                            self.last_person_goal_time = None
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse person detection data: {e}')
        
    def safety_check(self):
        """Safety check - DISABLED when tracking person to avoid canceling approach"""
        # Allow completely disabling safety checks via parameter
        if not self.safety_checks_enabled:
            return
            
        # Don't cancel goals when approaching a person!
        if self.tracking_person or self.waiting_by_person:
            # Log occasionally to confirm we're skipping safety
            if hasattr(self, '_safety_skip_log_counter'):
                self._safety_skip_log_counter += 1
                if self._safety_skip_log_counter >= 10:  # Every 5 seconds
                    self.get_logger().info('🛡️ Safety checks DISABLED during person tracking')
                    self._safety_skip_log_counter = 0
            else:
                self._safety_skip_log_counter = 0
                self.get_logger().info('🛡️ Safety checks DISABLED during person tracking')
            return
            
        if self.scan_data is None or not self.goal_active:
            self.obstacle_count = 0
            return
            
        # Get minimum distance to obstacles
        valid_ranges = [r for r in self.scan_data.ranges if not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges:
            return
        min_distance = min(valid_ranges)
        
        # CRITICAL: Immediate cancel if very close (< 0.25m = 10 inches)
        if min_distance < self.critical_obstacle_threshold:
            self.get_logger().warn(f'⛔ CRITICAL: Obstacle at {min_distance:.2f}m! Emergency cancel!')
            self.cancel_current_goal()
            self.obstacle_count = 0
            return
        
        # WARNING: Cancel if obstacle within 0.35m for 2.5 seconds (5 checks at 0.5Hz)
        if min_distance < self.obstacle_threshold:
            self.obstacle_count += 1
            if self.obstacle_count >= 5:  # Increased from 2 to 5 (2.5 seconds instead of 1 second)
                self.get_logger().warn(f'⚠️  Sustained obstacle at {min_distance:.2f}m for 2.5s - Cancelling!')
                self.cancel_current_goal()
                self.obstacle_count = 0
        else:
            self.obstacle_count = 0
    
    def check_if_stuck(self):
        """Check if robot is stuck (not making progress)"""
        if self.current_pose is None or not self.goal_active or self.recovery_in_progress:
            return
        
        # Don't check for stuck when approaching person
        if self.tracking_person or self.waiting_by_person:
            return
        
        current_pos = (self.current_pose.position.x, self.current_pose.position.y)
        self.last_positions.append(current_pos)
        
        # Need at least 16 positions (8 seconds) to detect stuck - increased from 8
        if len(self.last_positions) < 16:
            return
        
        total_movement = 0.0
        for i in range(1, len(self.last_positions)):
            dx = self.last_positions[i][0] - self.last_positions[i-1][0]
            dy = self.last_positions[i][1] - self.last_positions[i-1][1]
            total_movement += math.sqrt(dx*dx + dy*dy)
        
        # If moved less than 0.15m in 8 seconds, likely stuck (reduced from 0.3m in 4s)
        if total_movement < 0.15:
            self.stuck_count += 1
            if self.stuck_count >= 4:  # Stuck for 2 seconds (increased from 3)
                self.get_logger().warn(f'🚨 Robot appears stuck! Moved only {total_movement:.3f}m in 8s')
                self.initiate_recovery()
                self.stuck_count = 0
        else:
            self.stuck_count = 0
            
    def exploration_callback(self):
        """Main exploration logic - PERSON TRACKING IS ABSOLUTE PRIORITY"""
        if self.map_data is None or self.current_pose is None:
            return
        
        if self.recovery_in_progress:
            return
        
        # ==========================================
        # ABSOLUTE PRIORITY 1: PERSON TRACKING
        # This runs BEFORE anything else
        # ==========================================
        if self.person_detected and self.person_tracking_enabled:
            # If we're in waiting or turning states, handle those
            if self.waiting_by_person:
                current_time = self.get_clock().now()
                if self.person_wait_start_time is not None:
                    elapsed = (current_time - self.person_wait_start_time).nanoseconds / 1e9
                    
                    if elapsed < self.person_wait_duration:
                        if int(elapsed) % 2 == 0 and elapsed % 2 < 0.15:
                            remaining = self.person_wait_duration - elapsed
                            self.get_logger().info(f'⏳ Waiting by person... {remaining:.1f}s remaining')
                        return
                    else:
                        self.get_logger().info('✅ Wait complete! Turning 180° away...')
                        self.waiting_by_person = False
                        self.person_wait_start_time = None
                        self.turning_away = True
                        self.turn_away_start_yaw = None
                return
            
            if self.turning_away:
                if self.turn_away_from_person():
                    self.turning_away = False
                    self.turn_away_start_yaw = None
                    self.tracking_person = False
                    self.person_detected = False
                    self.person_position = None
                    self.person_depth = None
                    self.last_person_goal = None
                    self.last_person_goal_time = None
                    self.goal_active = False
                    self.get_logger().info('✅ Turned away! Resuming exploration...')
                return
            
            # ACTIVE PERSON TRACKING - Update goal continuously every cycle
            if not self.waiting_by_person and not self.turning_away:
                # Log person tracking state occasionally
                if not hasattr(self, '_person_track_log_counter'):
                    self._person_track_log_counter = 0
                    self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                    self.get_logger().info('🎯 PERSON TRACKING MODE ACTIVE')
                    self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                
                self._person_track_log_counter += 1
                if self._person_track_log_counter >= 20:  # Every 2 seconds
                    self.get_logger().info(f'📍 Tracking person... goal_active={self.goal_active}')
                    self._person_track_log_counter = 0
                
                self.navigate_to_person()
                return  # Don't do anything else!
        
        # ==========================================
        # PRIORITY 2: Normal exploration (only if no person detected)
        # ==========================================
        if not self.goal_active:
            if self.last_goal_completion_time is not None:
                time_since_completion = (self.get_clock().now() - self.last_goal_completion_time).nanoseconds / 1e9
                if time_since_completion < self.min_goal_interval:
                    return
            
            self.find_and_navigate_to_goal()
        else:
            # Check goal timeout
            if self.last_goal_time is not None:
                elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout:
                    self.get_logger().warn(f'Goal timeout ({elapsed:.1f}s)')
                    self.cancel_current_goal()
                    
    def find_and_navigate_to_goal(self):
        """Find a new exploration goal and navigate to it"""
        goal = self.get_frontier_goal()
        
        if goal is not None:
            distance = math.sqrt(
                (goal[0] - self.current_pose.position.x)**2 +
                (goal[1] - self.current_pose.position.y)**2
            )
            self.get_logger().info(f'🗺️  Frontier goal at ({goal[0]:.2f}, {goal[1]:.2f}) - {distance:.2f}m away')
            self.navigate_to_goal(goal)
        else:
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
        """Navigate towards detected waving person - send goal once and let Nav2 handle it"""
        if not self.person_position or self.current_pose is None:
            return
        
        # If we already have an active goal for this person, let it continue
        if self.goal_active and self.last_person_goal is not None:
            # Just monitor distance
            if self.person_depth and self.person_depth <= (self.person_approach_distance + 0.15):
                if not self.waiting_by_person:
                    self.get_logger().info(f'🎉🎉🎉 REACHED PERSON! Distance: {self.person_depth:.2f}m')
                    self.get_logger().info('Starting 10-second wait...')
                    self.waiting_by_person = True
                    self.person_wait_start_time = self.get_clock().now()
                    
                    # Cancel navigation and stop
                    self.cancel_current_goal()
                    
                    twist = Twist()
                    self.cmd_vel_pub.publish(twist)
            return  # Goal already active, let Nav2 execute
        
        # Calculate person position in world frame
        orientation = self.current_pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Camera is front-facing
        person_x_norm = self.person_position[0]
        angle_offset = (person_x_norm - 0.5) * (math.pi / 1.5)  # ±60° FOV
        person_angle = yaw + angle_offset
        
        # Use actual depth if available, otherwise use fallback
        if self.person_depth and self.person_depth > 0.3:
            estimated_distance = self.person_depth
        else:
            estimated_distance = 3.0  # Larger fallback to be safe
        
        # Calculate goal position - put goal directly at person's location
        # Nav2 will stop naturally when it gets close
        goal_x = self.current_pose.position.x + estimated_distance * math.cos(person_angle)
        goal_y = self.current_pose.position.y + estimated_distance * math.sin(person_angle)
        
        self.get_logger().info(f'🎯 SENDING PERSON GOAL: depth={estimated_distance:.2f}m, angle={math.degrees(angle_offset):.1f}°')
        self.get_logger().info(f'   Goal: ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info(f'   🚨 NAV2 WILL NOW HANDLE APPROACH - NO MORE UPDATES 🚨')
        
        self.last_person_goal = (goal_x, goal_y)
        self.last_person_goal_time = self.get_clock().now()
        self.navigate_to_goal((goal_x, goal_y), is_person_goal=True)
    
    def turn_away_from_person(self):
        """Turn 180° to face away from person"""
        if self.current_pose is None:
            return False
        
        orientation = self.current_pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if self.turn_away_start_yaw is None:
            self.turn_away_start_yaw = current_yaw
            self.get_logger().info('🔄 Turning 180° away from person...')
        
        yaw_diff = current_yaw - self.turn_away_start_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        if abs(abs(yaw_diff) - math.pi) < 0.15:
            self.get_logger().info('✅ 180° turn complete!')
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return True
        else:
            twist = Twist()
            twist.angular.z = 0.8
            self.cmd_vel_pub.publish(twist)
        
        return False
    
    def get_frontier_goal(self):
        """Find frontier cells (boundary between known and unknown space)"""
        if self.map_data is None or self.map_info is None or self.current_pose is None:
            return None
            
        height, width = self.map_data.shape
        frontiers = []
        
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self.map_data[y, x] == 0:
                    neighbors = [
                        self.map_data[y-1, x], self.map_data[y+1, x],
                        self.map_data[y, x-1], self.map_data[y, x+1]
                    ]
                    if -1 in neighbors:
                        world_x = x * self.map_info.resolution + self.map_info.origin.position.x
                        world_y = y * self.map_info.resolution + self.map_info.origin.position.y
                        
                        dist = math.sqrt(
                            (world_x - self.current_pose.position.x)**2 +
                            (world_y - self.current_pose.position.y)**2
                        )
                        
                        if dist < self.exploration_radius:
                            unknown_count = self.count_nearby_unknown(x, y, radius=3)
                            frontiers.append((world_x, world_y, dist, unknown_count))
        
        if not frontiers:
            return None
            
        filtered_frontiers = []
        for frontier in frontiers:
            if not self.is_recently_explored(frontier, threshold=1.5) and not self.is_near_blacklisted(frontier):
                filtered_frontiers.append(frontier)
        
        if not filtered_frontiers:
            self.explored_frontiers = self.explored_frontiers[-10:]
            filtered_frontiers = [f for f in frontiers if not self.is_recently_explored(f, threshold=1.5)]
            if not filtered_frontiers:
                filtered_frontiers = frontiers
        
        if len(filtered_frontiers) >= self.min_frontier_size:
            scored_frontiers = []
            for f in filtered_frontiers:
                world_x, world_y, dist, unknown_count = f
                score = unknown_count * 2.0 - dist * 0.5
                scored_frontiers.append((world_x, world_y, dist, unknown_count, score))
            
            scored_frontiers.sort(key=lambda f: f[4], reverse=True)
            best = scored_frontiers[0]
            goal = (best[0], best[1])
            
            self.explored_frontiers.append(goal + (best[2],))
            if len(self.explored_frontiers) > 100:
                self.explored_frontiers.pop(0)
                
            return goal
        
        return None
        
    def get_random_exploration_goal(self):
        """Generate a random exploration goal in free space"""
        if self.map_data is None or self.map_info is None or self.current_pose is None:
            return None
            
        for _ in range(50):
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(1.0, self.exploration_radius)
            
            world_x = self.current_pose.position.x + distance * math.cos(angle)
            world_y = self.current_pose.position.y + distance * math.sin(angle)
            
            grid_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
            grid_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)
            
            if (0 <= grid_x < self.map_data.shape[1] and 
                0 <= grid_y < self.map_data.shape[0]):
                if self.map_data[grid_y, grid_x] == 0:
                    return (world_x, world_y)
                    
        return None
        
    def count_nearby_unknown(self, grid_x, grid_y, radius=3):
        """Count unknown cells near a frontier point"""
        if self.map_data is None:
            return 0
        
        height, width = self.map_data.shape
        unknown_count = 0
        
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                ny, nx = grid_y + dy, grid_x + dx
                if 0 <= ny < height and 0 <= nx < width:
                    if self.map_data[ny, nx] == -1:
                        unknown_count += 1
        
        return unknown_count
    
    def is_recently_explored(self, frontier, threshold=1.5):
        """Check if a frontier was recently explored"""
        for explored in self.explored_frontiers[-30:]:
            dist = math.sqrt(
                (frontier[0] - explored[0])**2 +
                (frontier[1] - explored[1])**2
            )
            if dist < threshold:
                return True
        return False
    
    def is_near_blacklisted(self, frontier, threshold=3.0):
        """Check if a frontier is near a blacklisted location"""
        for blacklisted in self.blacklisted_goals:
            dist = math.sqrt(
                (frontier[0] - blacklisted[0])**2 +
                (frontier[1] - blacklisted[1])**2
            )
            if dist < threshold:
                return True
        return False
        
    def navigate_to_goal(self, goal, is_person_goal=False):
        """Send navigation goal to Nav2 - handles continuous updates for person tracking"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available!')
            return
            
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        
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
            
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        # For person tracking, we send goals continuously - don't log every time
        if is_person_goal:
            # Only log if this is a new tracking session or significant change
            if not self.goal_active:
                self.get_logger().info(f'📤 Starting PERSON tracking navigation')
        else:
            # Normal exploration goal - always log
            self.get_logger().info(f'📤 Sending exploration goal: ({goal[0]:.2f}, {goal[1]:.2f}), dist={self.current_goal_distance:.2f}m')
        
        self.goal_active = True
        self.last_goal_time = self.get_clock().now()
        self.obstacle_count = 0
        
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
            self.goal_handle = None
            return
            
        self.goal_handle = goal_handle  # Store for cancellation
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
        
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        pass
        
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        
        if self.last_goal_time is not None:
            elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
            self.get_logger().info(f'✅ Navigation goal completed! ({elapsed:.1f}s, {self.current_goal_distance:.2f}m)')
        
        self.goal_active = False
        self.goal_handle = None
        self.last_goal_time = None
        self.last_goal_completion_time = self.get_clock().now()
        self.obstacle_count = 0
        
    def initiate_recovery(self):
        """Recovery behavior when robot is stuck"""
        self.recovery_in_progress = True
        self.get_logger().info('🔄 Initiating recovery behavior...')
        
        if self.current_pose is not None:
            stuck_location = (self.current_pose.position.x, self.current_pose.position.y)
            self.blacklisted_goals.append(stuck_location)
            if len(self.blacklisted_goals) > 20:
                self.blacklisted_goals.pop(0)
        
        self.cancel_current_goal()
        self.execute_recovery_maneuver()
        
    def execute_recovery_maneuver(self):
        """Execute recovery maneuver: back up and rotate - REDUCED DURATION"""
        self.get_logger().info('⬅️ Backing up...')
        
        # Back up for 1.5 seconds at moderate speed (reduced from 3 seconds)
        backup_cmd = Twist()
        backup_cmd.linear.x = -0.2  # Reduced from -0.3
        for _ in range(15):  # 1.5 seconds (reduced from 30 = 3 seconds)
            self.cmd_vel_pub.publish(backup_cmd)
            
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Rotate 90 degrees instead of 135 (faster recovery)
        self.get_logger().info('🔄 Rotating to find clear path...')
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.5  # Reduced from 0.6
        for _ in range(15):  # ~1.5 seconds for 90° (reduced from 22)
            self.cmd_vel_pub.publish(rotate_cmd)
        
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.get_logger().info('✅ Recovery complete')
        self.recovery_in_progress = False
        self.last_positions.clear()
    
    def cancel_current_goal(self):
        """Cancel the current navigation goal - properly cancel Nav2 action"""
        if self.goal_active:
            # Properly cancel the Nav2 action if we have a goal_handle
            if self.goal_handle is not None:
                self.get_logger().info('🛑 Canceling Nav2 goal...')
                cancel_future = self.goal_handle.cancel_goal_async()
                # Don't wait for cancellation to complete
                self.goal_handle = None
            
            # Stop the robot immediately
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