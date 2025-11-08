#!/usr/bin/env python3

"""
Automatic Standoff Filter Node
Records close obstacles on startup and filters them from LiDAR readings.

Usage:
  - If record_standoffs=True: Records obstacles < 1 foot for 2 seconds, saves angles, then filters
  - If record_standoffs=False: Reads existing angles file and filters immediately
  - Always filters with ±2° tolerance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
import json
import os
from collections import defaultdict

# Default file path for standoff angles
DEFAULT_ANGLES_FILE = os.path.expanduser('~/rover_workspace/robot_standoffs_angles.txt')
DEFAULT_RECORDING_DURATION = 2.0  # seconds
DEFAULT_DISTANCE_THRESHOLD = 0.3048  # 1 foot in meters
DEFAULT_ANGLE_TOLERANCE = math.radians(2.0)  # 2 degrees in radians


class AutoStandoffFilter(Node):
    def __init__(self):
        super().__init__('auto_standoff_filter')
        
        # Declare parameters
        self.declare_parameter('record_standoffs', False)
        self.declare_parameter('angles_file', DEFAULT_ANGLES_FILE)
        self.declare_parameter('recording_duration', DEFAULT_RECORDING_DURATION)
        self.declare_parameter('distance_threshold', DEFAULT_DISTANCE_THRESHOLD)
        self.declare_parameter('angle_tolerance', DEFAULT_ANGLE_TOLERANCE)
        
        # Get parameters
        self.record_standoffs = self.get_parameter('record_standoffs').get_parameter_value().bool_value
        self.angles_file = self.get_parameter('angles_file').get_parameter_value().string_value
        self.recording_duration = self.get_parameter('recording_duration').get_parameter_value().double_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        
        # State management
        self.recording = False
        self.filtering = False
        self.start_time = None
        self.angles_to_ignore = set()
        
        # Recording data
        self.close_readings = []
        
        # Statistics
        self.total_scans = 0
        self.total_points_filtered = 0
        self.total_points_processed = 0
        
        # Subscribe to original scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Publisher for filtered scan
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        # Initialize based on mode
        if self.record_standoffs:
            self.get_logger().info('='*80)
            self.get_logger().info('AUTO STANDOFF FILTER: RECORDING MODE')
            self.get_logger().info('='*80)
            self.get_logger().info(f'Recording obstacles closer than {distance_threshold:.3f}m (1 foot)')
            self.get_logger().info(f'Recording duration: {recording_duration} seconds')
            self.get_logger().info(f'Will save angles to: {angles_file}')
            self.get_logger().info('Waiting for first scan to start recording...')
            self.recording = True
        else:
            # Try to load existing angles file
            if os.path.exists(self.angles_file):
                self.load_angles_from_file()
                if self.angles_to_ignore:
                    self.get_logger().info('='*80)
                    self.get_logger().info('AUTO STANDOFF FILTER: FILTERING MODE')
                    self.get_logger().info('='*80)
                    self.get_logger().info(f'Loaded {len(self.angles_to_ignore)} angles from {angles_file}')
                    self.get_logger().info(f'Filtering with ±{math.degrees(angle_tolerance):.1f}° tolerance')
                    self.get_logger().info('Publishing filtered scans to /scan_filtered')
                    self.filtering = True
                else:
                    self.get_logger().warn(f'No angles found in {angles_file}, filtering disabled')
            else:
                self.get_logger().warn(f'Angles file not found: {angles_file}')
                self.get_logger().warn('Set record_standoffs=True to record standoff angles')
                self.get_logger().warn('Filtering disabled - publishing unfiltered scans')
    
    def load_angles_from_file(self):
        """Load angles from the standoff angles file"""
        try:
            with open(self.angles_file, 'r') as f:
                lines = f.readlines()
            
            # Find the radians section
            in_radians_section = False
            for line in lines:
                if 'UNIQUE ANGLES IN RADIANS' in line:
                    in_radians_section = True
                    continue
                
                if in_radians_section:
                    line = line.strip()
                    if line == '[':
                        continue
                    elif line == ']':
                        break
                    elif line and not line.startswith('ANGLE') and not line.startswith('-'):
                        # Extract the number (remove comma if present)
                        angle_str = line.rstrip(',').strip()
                        try:
                            self.angles_to_ignore.add(float(angle_str))
                        except ValueError:
                            pass
            
            self.get_logger().info(f'Successfully loaded {len(self.angles_to_ignore)} angles')
        except Exception as e:
            self.get_logger().error(f'Error loading angles from {self.angles_file}: {e}')
            self.angles_to_ignore = set()
    
    def save_angles_to_file(self, unique_angles_rad):
        """Save unique angles to file in the same format as extract_unique_angles.py"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.angles_file) if os.path.dirname(self.angles_file) else '.', exist_ok=True)
            
            # Convert to sorted lists
            unique_angles_rad_list = sorted(list(unique_angles_rad))
            unique_angles_deg_list = sorted([math.degrees(a) for a in unique_angles_rad_list])
            
            with open(self.angles_file, 'w') as f:
                f.write('='*80 + '\n')
                f.write('ROBOT STANDOFF ANGLES (Auto-recorded)\n')
                f.write('='*80 + '\n')
                f.write(f'Total Readings: {len(self.close_readings)}\n')
                f.write(f'Unique Angles Found: {len(unique_angles_deg_list)}\n')
                f.write(f'Recording Duration: {self.recording_duration} seconds\n')
                f.write(f'Distance Threshold: {self.distance_threshold:.3f}m (1 foot)\n')
                f.write('='*80 + '\n\n')
                
                # Write angles in degrees (array format)
                f.write('UNIQUE ANGLES IN DEGREES (sorted):\n')
                f.write('-'*80 + '\n')
                f.write('[\n')
                for i, angle in enumerate(unique_angles_deg_list):
                    f.write(f'  {angle:.6f}')
                    if i < len(unique_angles_deg_list) - 1:
                        f.write(',\n')
                    else:
                        f.write('\n')
                f.write(']\n\n')
                
                # Write angles in radians (array format)
                f.write('UNIQUE ANGLES IN RADIANS (sorted):\n')
                f.write('-'*80 + '\n')
                f.write('[\n')
                for i, angle in enumerate(unique_angles_rad_list):
                    f.write(f'  {angle:.10f}')
                    if i < len(unique_angles_rad_list) - 1:
                        f.write(',\n')
                    else:
                        f.write('\n')
                f.write(']\n\n')
                
                # Write summary statistics
                f.write('ANGLE STATISTICS:\n')
                f.write('-'*80 + '\n')
                if unique_angles_deg_list:
                    f.write(f'Minimum angle: {min(unique_angles_deg_list):.6f}° ({min(unique_angles_rad_list):.10f} rad)\n')
                    f.write(f'Maximum angle: {max(unique_angles_deg_list):.6f}° ({max(unique_angles_rad_list):.10f} rad)\n')
                    f.write(f'Angular range: {max(unique_angles_deg_list) - min(unique_angles_deg_list):.6f}°\n')
                    f.write(f'Average angle: {sum(unique_angles_deg_list)/len(unique_angles_deg_list):.6f}°\n')
            
            self.get_logger().info(f'✓ Saved {len(unique_angles_rad_list)} unique angles to {self.angles_file}')
            return unique_angles_rad_list
        except Exception as e:
            self.get_logger().error(f'Error saving angles to {self.angles_file}: {e}')
            return []
    
    def should_ignore_angle(self, angle):
        """Check if an angle should be ignored based on the ignore list"""
        for ignored_angle in self.angles_to_ignore:
            if abs(angle - ignored_angle) <= self.angle_tolerance:
                return True
        return False
    
    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        
        # Recording phase
        if self.recording:
            # Start recording on first message
            if self.start_time is None:
                self.start_time = current_time
                self.get_logger().info('Recording started!')
            
            # Check if recording period is complete
            elapsed = (current_time - self.start_time).nanoseconds / 1e9
            if elapsed >= self.recording_duration:
                # Finish recording and process results
                self.process_recording_results(msg)
                self.recording = False
                self.filtering = True
                return
            
            # Record close obstacles
            angle = msg.angle_min
            for i, range_value in enumerate(msg.ranges):
                # Check if reading is valid and within threshold
                if msg.range_min <= range_value <= self.distance_threshold:
                    # Normalize angle to [-180, 180] degrees
                    angle_deg = math.degrees(angle)
                    if angle_deg > 180:
                        angle_deg -= 360
                    
                    # Record the reading
                    self.close_readings.append({
                        'angle_rad': angle,
                        'angle_deg': angle_deg,
                        'distance_m': range_value
                    })
                
                angle += msg.angle_increment
            
            # Progress update
            if int(elapsed * 2) % 2 == 0:  # Every 0.5 seconds
                self.get_logger().info(f'Recording: {elapsed:.1f}s / {self.recording_duration:.1f}s - '
                                     f'Close readings: {len(self.close_readings)}')
            
            # Publish unfiltered scan during recording
            self.publisher.publish(msg)
            return
        
        # Filtering phase
        if self.filtering:
            # Create a copy of the scan message
            filtered_msg = LaserScan()
            filtered_msg.header = msg.header
            filtered_msg.angle_min = msg.angle_min
            filtered_msg.angle_max = msg.angle_max
            filtered_msg.angle_increment = msg.angle_increment
            filtered_msg.time_increment = msg.time_increment
            filtered_msg.scan_time = msg.scan_time
            filtered_msg.range_min = msg.range_min
            filtered_msg.range_max = msg.range_max
            
            # Filter the ranges
            filtered_ranges = []
            filtered_intensities = []
            
            angle = msg.angle_min
            points_filtered_this_scan = 0
            
            for i, range_value in enumerate(msg.ranges):
                if self.should_ignore_angle(angle):
                    # Replace filtered readings with inf (no reading)
                    filtered_ranges.append(float('inf'))
                    points_filtered_this_scan += 1
                else:
                    filtered_ranges.append(range_value)
                
                # Handle intensities if they exist
                if msg.intensities and i < len(msg.intensities):
                    if self.should_ignore_angle(angle):
                        filtered_intensities.append(0.0)
                    else:
                        filtered_intensities.append(msg.intensities[i])
                
                angle += msg.angle_increment
            
            filtered_msg.ranges = filtered_ranges
            if msg.intensities:
                filtered_msg.intensities = filtered_intensities
            
            # Publish filtered scan
            self.publisher.publish(filtered_msg)
            
            # Update statistics
            self.total_scans += 1
            self.total_points_filtered += points_filtered_this_scan
            self.total_points_processed += len(msg.ranges)
            
            # Log statistics every 100 scans
            if self.total_scans % 100 == 0:
                filter_percentage = (self.total_points_filtered / self.total_points_processed * 100) if self.total_points_processed > 0 else 0
                self.get_logger().info(
                    f'Filtered {self.total_scans} scans | '
                    f'Filtered {self.total_points_filtered}/{self.total_points_processed} points '
                    f'({filter_percentage:.2f}%)')
        else:
            # No filtering - just pass through
            self.publisher.publish(msg)
    
    def process_recording_results(self, last_msg):
        """Process recording results and start filtering"""
        self.get_logger().info('')
        self.get_logger().info('='*80)
        self.get_logger().info('RECORDING COMPLETE - Processing Results')
        self.get_logger().info('='*80)
        self.get_logger().info(f'Total close readings: {len(self.close_readings)}')
        
        if len(self.close_readings) == 0:
            self.get_logger().warn('No obstacles detected within 1 foot during recording.')
            self.get_logger().warn('No filtering will be applied.')
            return
        
        # Extract unique angles
        unique_angles_rad = set()
        for reading in self.close_readings:
            unique_angles_rad.add(reading['angle_rad'])
        
        # Save to file
        saved_angles = self.save_angles_to_file(unique_angles_rad)
        
        # Load angles for filtering
        self.angles_to_ignore = set(saved_angles)
        
        self.get_logger().info('')
        self.get_logger().info('Starting LiDAR filtering...')
        self.get_logger().info(f'Filtering {len(self.angles_to_ignore)} angles with ±{math.degrees(self.angle_tolerance):.1f}° tolerance')
        self.get_logger().info('Publishing filtered scans to /scan_filtered')
        self.get_logger().info('='*80)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the node (parameters come from ROS parameters)
    filter_node = AutoStandoffFilter()
    
    try:
        rclpy.spin(filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        if filter_node.filtering:
            filter_node.get_logger().info('')
            filter_node.get_logger().info('='*60)
            filter_node.get_logger().info('FINAL STATISTICS')
            filter_node.get_logger().info('='*60)
            filter_node.get_logger().info(f'Total scans processed: {filter_node.total_scans}')
            filter_node.get_logger().info(f'Total points filtered: {filter_node.total_points_filtered}')
            filter_node.get_logger().info(f'Total points processed: {filter_node.total_points_processed}')
            if filter_node.total_points_processed > 0:
                filter_percentage = (filter_node.total_points_filtered / filter_node.total_points_processed * 100)
                filter_node.get_logger().info(f'Filter rate: {filter_percentage:.2f}%')
            filter_node.get_logger().info('='*60)
        
        filter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

