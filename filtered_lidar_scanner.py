#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import json
import math
import sys

class FilteredLidarScanner(Node):
    def __init__(self, angles_to_ignore=None, angle_tolerance=0.01):
        super().__init__('filtered_lidar_scanner')
        
        # Angles to filter out (in radians)
        self.angles_to_ignore = set(angles_to_ignore) if angles_to_ignore else set()
        self.angle_tolerance = angle_tolerance  # Tolerance in radians for matching angles
        
        self.get_logger().info(f'Filtered LiDAR Scanner initialized')
        self.get_logger().info(f'Ignoring {len(self.angles_to_ignore)} specific angles')
        self.get_logger().info(f'Angle tolerance: ±{math.degrees(angle_tolerance):.3f}°')
        
        # Subscribe to original scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Publisher for filtered scan
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        # Statistics
        self.total_scans = 0
        self.total_points_filtered = 0
        self.total_points_processed = 0
        
        self.get_logger().info('Publishing filtered scans to /scan_filtered')
        self.get_logger().info('Original scans available on /scan')
    
    def should_ignore_angle(self, angle):
        """Check if an angle should be ignored based on the ignore list"""
        for ignored_angle in self.angles_to_ignore:
            if abs(angle - ignored_angle) <= self.angle_tolerance:
                return True
        return False
    
    def scan_callback(self, msg):
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
        
        # Log statistics every 50 scans
        if self.total_scans % 50 == 0:
            filter_percentage = (self.total_points_filtered / self.total_points_processed * 100) if self.total_points_processed > 0 else 0
            self.get_logger().info(
                f'Processed {self.total_scans} scans | '
                f'Filtered {self.total_points_filtered}/{self.total_points_processed} points '
                f'({filter_percentage:.2f}%)')


def load_angles_from_file(filepath):
    """Load angles from a unique angles text file or JSON file"""
    angles_rad = []
    
    if filepath.endswith('.json'):
        # Load from obstacle recording JSON
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        # Extract unique angles in radians
        unique_angles = set()
        for reading in data['readings']:
            unique_angles.add(reading['angle_rad'])
        angles_rad = list(unique_angles)
        
    elif filepath.endswith('.txt'):
        # Load from unique angles text file
        with open(filepath, 'r') as f:
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
                elif line and not line.startswith('ANGLE'):
                    # Extract the number (remove comma if present)
                    angle_str = line.rstrip(',').strip()
                    try:
                        angles_rad.append(float(angle_str))
                    except ValueError:
                        pass
    
    return angles_rad


def main(args=None):
    rclpy.init(args=args)
    
    angles_to_ignore = []
    angle_tolerance = 0.01  # Default tolerance in radians (~0.57 degrees)
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
        
        try:
            angles_to_ignore = load_angles_from_file(input_file)
            print(f'Loaded {len(angles_to_ignore)} angles to ignore from {input_file}')
            
            # Show angle range
            if angles_to_ignore:
                min_angle = min(angles_to_ignore)
                max_angle = max(angles_to_ignore)
                print(f'Angle range: {math.degrees(min_angle):.2f}° to {math.degrees(max_angle):.2f}°')
        except Exception as e:
            print(f'Error loading angles from {input_file}: {e}')
            print('Usage: python3 filtered_lidar_scanner.py <angles_file> [tolerance_degrees]')
            return
    else:
        print('Usage: python3 filtered_lidar_scanner.py <angles_file> [tolerance_degrees]')
        print('')
        print('Examples:')
        print('  python3 filtered_lidar_scanner.py close_obstacles_1762028491_unique_angles.txt')
        print('  python3 filtered_lidar_scanner.py close_obstacles_1762028491.json')
        print('  python3 filtered_lidar_scanner.py angles.txt 1.0  # 1 degree tolerance')
        print('')
        print('This will filter out specific angles from LiDAR scans.')
        print('Original scans: /scan')
        print('Filtered scans: /scan_filtered')
        return
    
    # Optional: custom tolerance
    if len(sys.argv) > 2:
        try:
            angle_tolerance = math.radians(float(sys.argv[2]))
            print(f'Using custom tolerance: ±{sys.argv[2]}°')
        except ValueError:
            print(f'Invalid tolerance value: {sys.argv[2]}, using default')
    
    # Create and run the node
    scanner = FilteredLidarScanner(
        angles_to_ignore=angles_to_ignore,
        angle_tolerance=angle_tolerance
    )
    
    try:
        rclpy.spin(scanner)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics
        scanner.get_logger().info('')
        scanner.get_logger().info('='*60)
        scanner.get_logger().info('FINAL STATISTICS')
        scanner.get_logger().info('='*60)
        scanner.get_logger().info(f'Total scans processed: {scanner.total_scans}')
        scanner.get_logger().info(f'Total points filtered: {scanner.total_points_filtered}')
        scanner.get_logger().info(f'Total points processed: {scanner.total_points_processed}')
        if scanner.total_points_processed > 0:
            filter_percentage = (scanner.total_points_filtered / scanner.total_points_processed * 100)
            scanner.get_logger().info(f'Filter rate: {filter_percentage:.2f}%')
        scanner.get_logger().info('='*60)
        
        scanner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

